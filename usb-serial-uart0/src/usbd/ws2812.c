#include "ws2812.h"

#define INPUT 0
#define OUTPUT 1
#define LOW 0
#define HIGH 1
#define NEO_RGB ((0 << 6) | (0 << 4) | (1 << 2) | (2))
#define NEO_RBG ((0 << 6) | (0 << 4) | (2 << 2) | (1))
#define NEO_GRB ((1 << 6) | (1 << 4) | (0 << 2) | (2))
#define NEO_GBR ((2 << 6) | (2 << 4) | (0 << 2) | (1))
#define NEO_BRG ((1 << 6) | (1 << 4) | (2 << 2) | (0))
#define NEO_BGR ((2 << 6) | (2 << 4) | (1 << 2) | (0))
// RGBW NeoPixel permutations; all 4 offsets are distinct
// Offset:         W          R          G          B
#define NEO_WRGB ((0 << 6) | (1 << 4) | (2 << 2) | (3))
#define NEO_WRBG ((0 << 6) | (1 << 4) | (3 << 2) | (2))
#define NEO_WGRB ((0 << 6) | (2 << 4) | (1 << 2) | (3))
#define NEO_WGBR ((0 << 6) | (3 << 4) | (1 << 2) | (2))
#define NEO_WBRG ((0 << 6) | (2 << 4) | (3 << 2) | (1))
#define NEO_WBGR ((0 << 6) | (3 << 4) | (2 << 2) | (1))
#define NEO_RWGB ((1 << 6) | (0 << 4) | (2 << 2) | (3))
#define NEO_RWBG ((1 << 6) | (0 << 4) | (3 << 2) | (2))
#define NEO_RGWB ((2 << 6) | (0 << 4) | (1 << 2) | (3))
#define NEO_RGBW ((3 << 6) | (0 << 4) | (1 << 2) | (2))
#define NEO_RBWG ((2 << 6) | (0 << 4) | (3 << 2) | (1))
#define NEO_RBGW ((3 << 6) | (0 << 4) | (2 << 2) | (1))
#define NEO_GWRB ((1 << 6) | (2 << 4) | (0 << 2) | (3))
#define NEO_GWBR ((1 << 6) | (3 << 4) | (0 << 2) | (2))
#define NEO_GRWB ((2 << 6) | (1 << 4) | (0 << 2) | (3))
#define NEO_GRBW ((3 << 6) | (1 << 4) | (0 << 2) | (2))
#define NEO_GBWR ((2 << 6) | (3 << 4) | (0 << 2) | (1))
#define NEO_GBRW ((3 << 6) | (2 << 4) | (0 << 2) | (1))
#define NEO_BWRG ((1 << 6) | (2 << 4) | (3 << 2) | (0))
#define NEO_BWGR ((1 << 6) | (3 << 4) | (2 << 2) | (0))
#define NEO_BRWG ((2 << 6) | (1 << 4) | (3 << 2) | (0))
#define NEO_BRGW ((3 << 6) | (1 << 4) | (2 << 2) | (0))
#define NEO_BGWR ((2 << 6) | (3 << 4) | (1 << 2) | (0))
#define NEO_BGRW ((3 << 6) | (2 << 4) | (1 << 2) | (0))
#define NEO_KHZ800 0x0000
#define NEO_KHZ400 0x0100
bool is800KHz;
#define max(a, b) (((a) > (b)) ? (a) : (b))
#define min(a, b) (((a) > (b)) ? (b) : (a))
struct parameterInit_t
{
    uint8_t p;
    uint16_t n;
    uint8_t b;
    neoPixelType t;
} parameterInit;
bool begun;
static uint8_t *pixels[13] = {NULL};
static uint8_t *pixels1[13] = {NULL};
uint8_t
    brightness,
    //*pixels,        // Holds LED color values (3 or 4 bytes each)
    //*pixels1,        // Holds LED color values (3 or 4 bytes each)
    rOffset, // Index of red byte within each 3- or 4-byte pixel
    gOffset, // Index of green byte
    bOffset, // Index of blue byte
    wOffset; // Index of white byte (same as rOffset if no white)
uint16_t
    numLEDs,            // Number of RGB LEDs in strip
    numBytes;           // Size of 'pixels' buffer below (3 or 4 bytes/pixel)
volatile uint8_t *port; // Output PORT register
uint8_t pinMask;        // Output PORT bitmask
uint32_t endTime;
int8_t pin;
bool begin(uint8_t p,uint16_t n,uint8_t bright, neoPixelType t)
{
    if (pin != -1){
        if (parameterInit.p == p && parameterInit.n == n && parameterInit.b == bright && parameterInit.t == t)
            return FALSE;
    }
    parameterInit.p = p;
    parameterInit.n = n;
    parameterInit.b = bright;
    parameterInit.t = t;
    brightness = bright;
    updateType(t, p);
    if (FALSE == updateLength(n, p))
    {
        return FALSE;
    }
    if (pin >= 0)
    {
        gpio_init(GPIOA, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_4);
        digitalWrite(pin, LOW);
    }
    begun = TRUE;
    return TRUE;
}

void updateType(neoPixelType t, uint8_t pin)
{
    bool oldThreeBytesPerPixel = (wOffset == rOffset); // false if RGBW
    wOffset = (t >> 6) & 0b11;                         // See notes in header file
    rOffset = (t >> 4) & 0b11;                         // regarding R/G/B/W offsets
    gOffset = (t >> 2) & 0b11;
    bOffset = t & 0b11;
    if (pixels[pin])
    {
        bool newThreeBytesPerPixel = (wOffset == rOffset);
        if (newThreeBytesPerPixel != oldThreeBytesPerPixel)
            updateLength(numLEDs, pin);
    }
}

bool updateLength(uint16_t n, uint8_t pin)
{
    if (pixels[pin])
    {
        free(pixels[pin]);
        pixels[pin] = NULL;
    } // Free existing data (if any)
    if (pixels1[pin])
    {
        free(pixels1[pin]);
        pixels1[pin] = NULL;
    } // Free existing data (if any)
    numBytes = n * ((wOffset == rOffset) ? 3 : 4);
    if ((pixels[pin] = (uint8_t *)malloc(numBytes)) && (pixels1[pin] = (uint8_t *)malloc(numBytes)))
    {
        memset(pixels[pin], 0, numBytes);
        memset(pixels1[pin], 0, numBytes);
        numLEDs = n;
        return TRUE;
    }
    else
    {
        free(pixels[pin]);
        pixels[pin] = NULL;

        free(pixels1[pin]);
        pixels1[pin] = NULL;

        numLEDs = numBytes = 0;
        return FALSE;
    }
}

void setRangeColor(uint8_t pin, int16_t start, int16_t end, uint32_t c)
{
    if (start < 0 || end < 0)
    {
        start = 0;
        end = numLEDs - 1;
    }
    if (end < start)
    {
        uint16_t num = end;
        end = start;
        start = num;
    }
    if (numLEDs <= 0)
        return;
    start = max(start, 0);
    start = min(start, numLEDs);
    end = max(end, 0);
    end = min(end, numLEDs);
    for (uint16_t n = start; n < end + 1; n++)
    {
        setPixelColor(pin, n, c, FALSE);
    }
    show(pin);
}

void setPixelColor(uint8_t pin, uint16_t n, uint32_t c, bool user)
{
    if (n < numLEDs)
    {
        uint8_t *p, *p1,
            r = (uint8_t)(c >> 16),
            g = (uint8_t)(c >> 8),
            b = (uint8_t)c;
        if (wOffset == rOffset)
        {
            p = &(pixels[pin])[n * 3];
            p1 = &(pixels1[pin])[n * 3];
        }
        else
        {
            p = &(pixels[pin])[n * 4];
            p1 = &(pixels1[pin])[n * 4];
            uint8_t w = (uint8_t)(c >> 24);
            p[wOffset] = brightness ? ((w * brightness) >> 8) : w;
            p1[wOffset] = brightness ? ((w * brightness) >> 8) : w;
        }
        p1[rOffset] = r;
        p1[gOffset] = g;
        p1[bOffset] = b;
        r = (r * brightness) >> 8;
        g = (g * brightness) >> 8;
        b = (b * brightness) >> 8;
        p[rOffset] = r;
        p[gOffset] = g;
        p[bOffset] = b;
    }
    if (user)
        show(pin);
}

void show(uint8_t pin)
{
#if 0
    setPin(pin);
    volatile uint16_t
        i = numBytes; // Loop counter
    volatile uint8_t
        *ptr = pixels[pin], // Pointer to next byte
        b = *ptr++,         // Current byte value
        hi,                 // PORT w/output bit set high
        lo;                 // PORT w/output bit set low
        volatile uint8_t next, bit;
        hi = *port | pinMask;
        lo = *port & ~pinMask;
        next = lo;
        bit = 8;
        asm volatile(
            "head20:"
            "\n\t" // Clk  Pseudocode    (T =  0)
            "st   %a[port],  %[hi]"
            "\n\t" // 2    PORT = hi     (T =  2)
            "sbrc %[byte],  7"
            "\n\t" // 1-2  if(b & 128)
            "mov  %[next], %[hi]"
            "\n\t" // 0-1   next = hi    (T =  4)
            "dec  %[bit]"
            "\n\t" // 1    bit--         (T =  5)
            "st   %a[port],  %[next]"
            "\n\t" // 2    PORT = next   (T =  7)
            "mov  %[next] ,  %[lo]"
            "\n\t" // 1    next = lo     (T =  8)
            "breq nextbyte20"
            "\n\t" // 1-2  if(bit == 0) (from dec above)
            "rol  %[byte]"
            "\n\t" // 1    b <<= 1       (T = 10)
            "rjmp .+0"
            "\n\t" // 2    nop nop       (T = 12)
            "nop"
            "\n\t" // 1    nop           (T = 13)
            "st   %a[port],  %[lo]"
            "\n\t" // 2    PORT = lo     (T = 15)
            "nop"
            "\n\t" // 1    nop           (T = 16)
            "rjmp .+0"
            "\n\t" // 2    nop nop       (T = 18)
            "rjmp head20"
            "\n\t" // 2    -> head20 (next bit out)
            "nextbyte20:"
            "\n\t" //                    (T = 10)
            "ldi  %[bit]  ,  8"
            "\n\t" // 1    bit = 8       (T = 11)
            "ld   %[byte] ,  %a[ptr]+"
            "\n\t" // 2    b = *ptr++    (T = 13)
            "st   %a[port], %[lo]"
            "\n\t" // 2    PORT = lo     (T = 15)
            "nop"
            "\n\t" // 1    nop           (T = 16)
            "sbiw %[count], 1"
            "\n\t" // 2    i--           (T = 18)
            "brne head20"
            "\n" // 2    if(i != 0) -> (next byte)
            : [ port ] "+e"(port),
              [ byte ] "+r"(b),
              [ bit ] "+r"(bit),
              [ next ] "+r"(next),
              [ count ] "+w"(i)
            : [ ptr ] "e"(ptr),
              [ hi ] "r"(hi),
              [ lo ] "r"(lo));
#endif
}