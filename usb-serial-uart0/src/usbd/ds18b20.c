#if 0
#include "DS18B20.h"
#include "gd32vf103_gpio.h"
#include "firmata_to_board.h"

#define _BV(bit) (1 << (bit))

const uint8_t digital_pin_to_bit_mask_PGM[] = {
    _BV(0), /* 0, port D */
    _BV(1),
    _BV(2),
    _BV(3),
    _BV(4),
    _BV(5),
    _BV(6),
    _BV(7),
    _BV(8), /* 8, port B */
    _BV(9),
    _BV(10),
    _BV(11),
    _BV(12),
    _BV(13),
    _BV(14), /* 14, port C */
    _BV(15),
    _BV(1),
    _BV(2),
    _BV(3),
    _BV(4),
};

const uint16_t port_to_output_PGM[] = {
    (uint16_t)GPIOA,
    (uint16_t)GPIOB,
    (uint16_t)GPIOC,
    (uint16_t)GPIOD,
};

const uint16_t port_to_input_PGM[] = {
    (uint16_t)GPIOA,
    (uint16_t)GPIOB,
    (uint16_t)GPIOC,
    (uint16_t)GPIOD,
};

const uint8_t digital_pin_to_port_PGM[] = {
    PA, /* 0 */
    PA,
    PA,
    PA,
    PA,
    PA,
    PA,
    PA,
    PA, /* 8 */
    PA,
    PA,
    PA,
    PA,
    PA,
    PA, /* 14 */
    PA,
    PC,
    PC,
    PC,
    PC,
};

#define digitalPinToBitMask(P) (*(digital_pin_to_bit_mask_PGM + (P)))
#define portOutputRegister(P) ((volatile uint8_t *)((port_to_output_PGM + (P))))
#define portInputRegister(P) ((volatile uint8_t *)((port_to_input_PGM + (P))))
#define digitalPinToPort(P) (*(digital_pin_to_port_PGM + (P)))

#define OUTPUT TRUE
#define INPUT FALSE

extern void change_gpio_mode(uint8_t pin, uint8_t mode);

#define PIN_TO_BASEREG(pin) (portInputRegister(digitalPinToPort(pin)))
#define PIN_TO_BITMASK(pin) (digitalPinToBitMask(pin))
#define IO_REG_TYPE uint8_t
#define IO_REG_BASE_ATTR asm("r30")
#define DIRECT_READ(base, mask) (((*(base)) & (mask)) ? 1 : 0)
#define DIRECT_MODE_INPUT(base, mask) ((*((base) + 1)) &= ~(mask))
#define DIRECT_MODE_OUTPUT(base, mask) ((*((base) + 1)) |= (mask))
#define DIRECT_WRITE_LOW(base, mask) ((*((base) + 2)) &= ~(mask))
#define DIRECT_WRITE_HIGH(base, mask) ((*((base) + 2)) |= (mask))

#define byte unsigned char
IO_REG_TYPE bitmask;
volatile IO_REG_TYPE *baseReg;
typedef uint8_t DeviceAddress[8];

unsigned char ROM_NO[8];
uint8_t LastDiscrepancy;
uint8_t LastFamilyDiscrepancy;
uint8_t LastDeviceFlag;

uint8_t dscrc_table[] = {
    0, 94, 188, 226, 97, 63, 221, 131, 194, 156, 126, 32, 163, 253, 31, 65,
    157, 195, 33, 127, 252, 162, 64, 30, 95, 1, 227, 189, 62, 96, 130, 220,
    35, 125, 159, 193, 66, 28, 254, 160, 225, 191, 93, 3, 128, 222, 60, 98,
    190, 224, 2, 92, 223, 129, 99, 61, 124, 34, 192, 158, 29, 67, 161, 255,
    70, 24, 250, 164, 39, 121, 155, 197, 132, 218, 56, 102, 229, 187, 89, 7,
    219, 133, 103, 57, 186, 228, 6, 88, 25, 71, 165, 251, 120, 38, 196, 154,
    101, 59, 217, 135, 4, 90, 184, 230, 167, 249, 27, 69, 198, 152, 122, 36,
    248, 166, 68, 26, 153, 199, 37, 123, 58, 100, 134, 216, 91, 5, 231, 185,
    140, 210, 48, 110, 237, 179, 81, 15, 78, 16, 242, 172, 47, 113, 147, 205,
    17, 79, 173, 243, 112, 46, 204, 146, 211, 141, 111, 49, 178, 236, 14, 80,
    175, 241, 19, 77, 206, 144, 114, 44, 109, 51, 209, 143, 12, 82, 176, 238,
    50, 108, 142, 208, 83, 13, 239, 177, 240, 174, 76, 18, 145, 207, 45, 115,
    202, 148, 118, 40, 171, 245, 23, 73, 8, 86, 180, 234, 105, 55, 213, 139,
    87, 9, 235, 181, 54, 104, 138, 212, 149, 203, 41, 119, 244, 170, 72, 22,
    233, 183, 85, 11, 136, 214, 52, 106, 43, 117, 151, 201, 74, 20, 246, 168,
    116, 42, 200, 150, 21, 75, 169, 247, 182, 232, 10, 84, 215, 137, 107, 53};

uint8_t ds18b20_init(uint8_t pin)
{
    change_gpio_mode(pin, INPUT);
    bitmask = PIN_TO_BITMASK(pin); //PIN_TO_BITMASK(pin);
    baseReg = PIN_TO_BASEREG(5); //PIN_TO_BASEREG(pin);
    while(1){
        printf("%x\n", baseReg);
    }
    reset_search();
    return 0;
}

void reset_search(void)
{
    LastDiscrepancy = 0;
    LastDeviceFlag = FALSE;
    LastFamilyDiscrepancy = 0;
    for (int i = 7;; i--)
    {
        ROM_NO[i] = 0;
        if (i == 0)
            break;
    }
}

int getTemp(uint8_t *p)
{
    byte data[12];
    byte addr[8];

    if (!search(addr))
    {
        reset_search();
        return 1;
    }
    if (crc8(addr, 7) != addr[7])
    {
        return 2;
    }
    if (addr[0] != 0x10 && addr[0] != 0x28)
    {
        return 3;
    }
    reset();
    Select(addr);
    ds18b20_write(0x44, 1);
    // byte present = 
    reset();
    Select(addr);
    ds18b20_write(0xBE, 0);
    for (int i = 0; i < 9; i++)
    {
        data[i] = ds18b20_read();
    }
    reset_search();
    for (int i = 0; i < 9; i++)
    {
        p[i] = data[i];
    }
    return 0;
}

uint8_t reset(void)
{
    IO_REG_TYPE mask = bitmask;
    volatile IO_REG_TYPE *reg = baseReg;
    uint8_t r;
    uint8_t retries = 125;
    DIRECT_MODE_INPUT(reg, mask);
    do{
        if (--retries == 0)
            return 0;
        delayMicroseconds(2);
    } while (!DIRECT_READ(reg, mask));
    DIRECT_WRITE_LOW(reg, mask);
    DIRECT_MODE_OUTPUT(reg, mask); // drive output low
    delayMicroseconds(480);
    DIRECT_MODE_INPUT(reg, mask); // allow it to float
    delayMicroseconds(70);
    r = !DIRECT_READ(reg, mask);
    delayMicroseconds(410);
    return r;
}

void Select(const uint8_t rom[8])
{
    uint8_t i;
    ds18b20_write(0x55, 0); // Choose ROM
    for (i = 0; i < 8; i++)
        ds18b20_write(rom[i], 0);
}

void ds18b20_write(uint8_t v, uint8_t power)
{
    uint8_t bitMask;
    for (bitMask = 0x01; bitMask; bitMask <<= 1){
        write_bit((bitMask & v) ? 1 : 0);
    }
    if (!power){
        DIRECT_MODE_INPUT(baseReg, bitmask);
        DIRECT_WRITE_LOW(baseReg, bitmask);
    }
}

uint8_t ds18b20_read(void){
    uint8_t bitMask;
    uint8_t r = 0;
    for (bitMask = 0x01; bitMask; bitMask <<= 1){
        if (read_bit())
            r |= bitMask;
    }
    return r;
}

uint8_t read_bit(void){
    IO_REG_TYPE mask = bitmask;
    volatile IO_REG_TYPE *reg = baseReg;
    uint8_t r;
    DIRECT_MODE_OUTPUT(reg, mask);
    DIRECT_WRITE_LOW(reg, mask);
    delayMicroseconds(3);
    DIRECT_MODE_INPUT(reg, mask); 
    delayMicroseconds(10);
    r = DIRECT_READ(reg, mask);
    delayMicroseconds(53);
    return r;
}

void write_bit(uint8_t v)
{
    IO_REG_TYPE mask = bitmask;
    volatile IO_REG_TYPE *reg = baseReg;
    if (v & 1)
    {
        DIRECT_WRITE_LOW(reg, mask);
        DIRECT_MODE_OUTPUT(reg, mask); // drive output low
        delayMicroseconds(10);
        DIRECT_WRITE_HIGH(reg, mask); // drive output high
        delayMicroseconds(55);
    }
    else
    {
        DIRECT_WRITE_LOW(reg, mask);
        DIRECT_MODE_OUTPUT(reg, mask); // drive output low
        delayMicroseconds(65);
        DIRECT_WRITE_HIGH(reg, mask); // drive output high
        delayMicroseconds(5);
    }
}

uint8_t search(uint8_t *newAddr)
{
    uint8_t id_bit_number;
    uint8_t last_zero, rom_byte_number, search_result;
    uint8_t id_bit, cmp_id_bit;
    bool search_mode = TRUE;
    unsigned char rom_byte_mask, search_direction;

    // initialize for search
    id_bit_number = 1;
    last_zero = 0;
    rom_byte_number = 0;
    rom_byte_mask = 1;
    search_result = 0;

    // if the last call was not the last one
    if (!LastDeviceFlag)
    {
        // 1-Wire reset
        if (!reset())
        {
            // reset the search
            LastDiscrepancy = 0;
            LastDeviceFlag = FALSE;
            LastFamilyDiscrepancy = 0;
            return FALSE;
        }

        // issue the search command
        if (search_mode == TRUE)
        {
            ds18b20_write(0xF0, 0); // NORMAL SEARCH
        }
        else
        {
            ds18b20_write(0xEC, 0); // CONDITIONAL SEARCH
        }

        // loop to do the search
        do
        {
            // read a bit and its complement
            id_bit = read_bit();
            cmp_id_bit = read_bit();

            // check for no devices on 1-wire
            if ((id_bit == 1) && (cmp_id_bit == 1))
                break;
            else
            {
                // all devices coupled have 0 or 1
                if (id_bit != cmp_id_bit)
                    search_direction = id_bit; // bit write value for search
                else
                {
                    // if this discrepancy if before the Last Discrepancy
                    // on a previous next then pick the same as last time
                    if (id_bit_number < LastDiscrepancy)
                        search_direction = ((ROM_NO[rom_byte_number] & rom_byte_mask) > 0);
                    else
                        // if equal to last pick 1, if not then pick 0
                        search_direction = (id_bit_number == LastDiscrepancy);

                    // if 0 was picked then record its position in LastZero
                    if (search_direction == 0)
                    {
                        last_zero = id_bit_number;

                        // check for Last discrepancy in family
                        if (last_zero < 9)
                            LastFamilyDiscrepancy = last_zero;
                    }
                }

                // set or clear the bit in the ROM byte rom_byte_number
                // with mask rom_byte_mask
                if (search_direction == 1)
                    ROM_NO[rom_byte_number] |= rom_byte_mask;
                else
                    ROM_NO[rom_byte_number] &= ~rom_byte_mask;

                // serial number search direction write bit
                write_bit(search_direction);

                // increment the byte counter id_bit_number
                // and shift the mask rom_byte_mask
                id_bit_number++;
                rom_byte_mask <<= 1;

                // if the mask is 0 then go to new SerialNum byte rom_byte_number and reset mask
                if (rom_byte_mask == 0)
                {
                    rom_byte_number++;
                    rom_byte_mask = 1;
                }
            }
        } while (rom_byte_number < 8); // loop until through all ROM bytes 0-7

        // if the search was successful then
        if (!(id_bit_number < 65))
        {
            // search successful so set LastDiscrepancy,LastDeviceFlag,search_result
            LastDiscrepancy = last_zero;

            // check for last device
            if (LastDiscrepancy == 0)
                LastDeviceFlag = TRUE;

            search_result = TRUE;
        }
    }

    // if no device found then reset counters so next 'search' will be like a first
    if (!search_result || !ROM_NO[0])
    {
        LastDiscrepancy = 0;
        LastDeviceFlag = FALSE;
        LastFamilyDiscrepancy = 0;
        search_result = FALSE;
    }
    else
    {
        for (int i = 0; i < 8; i++)
            newAddr[i] = ROM_NO[i];
    }
    return search_result;
}

uint8_t crc8(const uint8_t *addr, uint8_t len)
{
    uint8_t crc = 0;
    while (len--)
    {
        crc = dscrc_table[(crc ^ *addr++)];
    }
    return crc;
}
#endif

#include "ds18b20.h"
#include "gd32vf103_gpio.h"

extern void delay_ms(uint32_t time_ms);
void delay_us(uint32_t time_us)
{
    usb_udelay(time_us);
}

uint8_t DS18B20_Start(void)
{
    uint8_t Response = 0;
    gpio_init(GPIOA, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_4); // set the pin as output
    gpio_bit_write(GPIOA, BIT(4), 0);                                  // pull the pin low

    delay_us(480); // delay according to datasheet

    gpio_init(GPIOA, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, GPIO_PIN_4); // set the pin as input
    delay_us(80);                             // delay according to datasheet

    if (!(gpio_input_bit_get(GPIOA, 4)))
        Response = 1; // if the pin is low i.e the presence pulse is detected
    else
        Response = 0;

    delay_us(400); // 480 us delay totally.

    return Response;
}

void DS18B20_Write(uint8_t data)
{
    gpio_init(GPIOA, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_4); // set as output
    for (int i = 0; i < 8; i++)
    {
        if ((data & (1 << i)) != 0) // if the bit is high
        {

            gpio_init(GPIOA, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_4); // set as output
            gpio_bit_write(GPIOA, BIT(4), 0);                                  // pull the pin LOW
            delay_us(1);                                                  // wait for 1 us

            gpio_init(GPIOA, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, GPIO_PIN_4); // set as input
            delay_us(50);                             // wait for 60 us
        }

        else // if the bit is low
        {
            gpio_init(GPIOA, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_4);
            gpio_bit_write(GPIOA, BIT(4), 0);                             // pull the pin LOW
            delay_us(50);                                                 // wait for 60 us

            gpio_init(GPIOA, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, GPIO_PIN_4);
        }
    }
}

extern bool digitalRead(uint8_t pin);

uint8_t DS18B20_Read(void)
{
    uint8_t value = 0;
    gpio_init(GPIOA, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, GPIO_PIN_4);

    for (int i = 0; i < 8; i++)
    {
        gpio_init(GPIOA, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_4); // set as output

        gpio_bit_write(GPIOA, BIT(4), 0); // pull the data pin LOW
        delay_us(2);                                                  // wait for 2 us

        gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_4); // set as input
        if (digitalRead(4)) // if the pin is HIGH
        {
            value |= 1 << i; // read = 1
        }
        delay_us(60); // wait for 60 us
    }
    return value;
}

int DS18B20_GetCelsiusTemp(void)
{
    uint8_t lsb;
    uint8_t msb;
    bool temp_sign;
    uint16_t temp_int;
    int temp_decimal;

    DS18B20_Start();
    delay_ms(1);
    DS18B20_Write(0xCC); // skip ROM
    DS18B20_Write(0x44); // convert t
    delay_ms(800);

    DS18B20_Start();
    delay_ms(1);
    DS18B20_Write(0xCC); // skip ROM
    DS18B20_Write(0xBE); // Read Scratch-pad

    lsb = DS18B20_Read();
    msb = DS18B20_Read();

    temp_int = ((uint16_t)msb << 8) | lsb;
    int negative;
    negative = (temp_int & (1 << 15)) != 0;
    if (negative)
    {
        temp_decimal = temp_int | ~((1 << 16) - 1);
    }
    else
    {
        temp_decimal = temp_int;
    }
    
    temp_decimal /= 16;
    return temp_decimal;
}
