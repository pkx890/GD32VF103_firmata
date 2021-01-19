#include "firmata.h"
#include "cdc_acm_core.h"
#include "string.h"
#include "firmata_to_board.h"

#define TIM_MSEC_DELAY 0x01
#define TIM_USEC_DELAY 0x02

bool parsingSysex;
size_t sysexBytesRead;
size_t waitForData;
uint8_t executeMultiByteCommand;
uint8_t multiByteChannel;
uint8_t dataBufferSize;
uint8_t dataBuffer[MAX_DATA_BYTES];
bool allowBufferUpdate;
uint8_t analogInputsToReport = 0; //用来存储哪些模拟口需要上报的，以位判断，每一位代表一个模拟口

usb_core_driver* DEV;
/**
 * 用来储存需要主动上报的数字口，以端口为单位
 * 每位代表对应的数字口是否需要上报
 * 1 = report this port, 0 = silence
 * Port 0 = pins D0 - D7, port 1 = pins D8 - D15
 */
uint8_t reportPINs[TOTAL_PORTS];

/**
 * 用来存储当前数字口的高低，以端口为单位
 * 每位代表一个数字口的高低
 * Port 0 = pins D0 - D7, port 1 = pins D8 - D15
 */
uint8_t previousPINs[TOTAL_PORTS];

/**
 * 用来存储数字口的模式，1为输入模式，以位运算
 * 8位分别代表8个数字口
 * each bit: 1 = pin in INPUT, 0 = anything else
 * Port 0 = pins D0 - D7, port 1 = pins D8 - D15
 */
uint16_t portConfigInputs[TOTAL_PORTS];

uint8_t firmwareVersionCount;
uint8_t *firmwareVersionVector;
char *myname = "DFRobot firmata\0";

void firmata_init(void)
{
    parsingSysex = FALSE;
    waitForData = 0;
    sysexBytesRead = 0;
    multiByteChannel = 0;
    executeMultiByteCommand = 0;
    dataBufferSize = MAX_DATA_BYTES;
}

static void encodeByteStream(size_t bytec, uint8_t *bytev, size_t max_bytes){
    static const size_t transmit_bits = 7;
    static const uint8_t transmit_mask = ((1 << transmit_bits) - 1);

    size_t bytes_sent = 0;
    size_t outstanding_bits = 0;
    uint8_t outstanding_bit_cache = *bytev;

    if (!max_bytes)
    {
        max_bytes = (size_t)-1;
    }
    for (size_t i = 0; (i < bytec) && (bytes_sent < max_bytes); ++i)
    {
        uint8_t transmit_byte = (outstanding_bit_cache | (bytev[i] << outstanding_bits));
        uint8_t buf[1] = {transmit_mask & transmit_byte};
        uart_write(buf,1);
        ++bytes_sent;
        outstanding_bit_cache = (bytev[i] >> (transmit_bits - outstanding_bits));
        outstanding_bits = (outstanding_bits + (8 - transmit_bits));
        for (; (outstanding_bits >= transmit_bits) && (bytes_sent < max_bytes);)
        {
            transmit_byte = outstanding_bit_cache;
            buf[0]= transmit_mask & transmit_byte;
            uart_write(buf,1);
            ++bytes_sent;
            outstanding_bit_cache >>= transmit_bits;
            outstanding_bits -= transmit_bits;
        }
    }
    if (outstanding_bits && (bytes_sent < max_bytes))
    {
        uint8_t buf = (uint8_t)((1 << outstanding_bits) - 1) & outstanding_bit_cache;
        uart_write(&buf,1);
    }
}

static void sendFirmwareVersion(uint8_t major, uint8_t minor, size_t bytec, uint8_t *bytev){
    uint8_t buf[4] = {START_SYSEX, REPORT_FIRMWARE, major, minor};
    uart_write(buf,4);
    for (size_t i = 0; i < bytec; ++i)
    {
        encodeByteStream(sizeof(bytev[i]), (uint8_t *)(&bytev[i]), 0);
    }
    buf[0] = END_SYSEX;
    uart_write(buf,1);
}
/**
 * 上报版本信息
 */
static void reoprt_firmware(){
    if (sysexBytesRead < 3){
        sendFirmwareVersion(FIRMATA_MAJOR, FIRMATA_MINOR, strlen(myname), (uint8_t *)myname);
    }else{
    }
}
/**
 * 根据命令设置IO口的模式
 */
static void firmata_setPinMode(uint16_t pin, int mode){
    if (getPinMode(pin) == PIN_MODE_IGNORE){
        return;
    }
    //是否将引脚注册为主动上报状态的引脚
    if (mode == PIN_MODE_INPUT || mode == PIN_MODE_PULLUP){
        portConfigInputs[pin / 16] |= (1 << (pin & 15));
    }else{
        portConfigInputs[pin / 16] &= ~(1 << (pin & 15));
    }
    setPinMode(pin, mode);
}

/************************************************************************/
/*                       数字口相关操作                                  */
/************************************************************************/

/**
 * 解析处理firmata协议包里的数字口的操作
 * Port 0 = pins D0 - D7, port 1 = pins D8 - D15
 */
static void firmata_digitalWrite(uint8_t port, int value){
    for (int i = 0; i < 8; i++){
        digitalWrite((port * 8) + i, value & (1 << i));
    }
}

/**
 * 读取端口上设置为输入模式的数字口的电平
 */
static uint8_t firmata_digitalRead(uint8_t port, uint8_t value){
    uint8_t out = 0;
    for (int i = 0; i < 8; i++){
        if (digitalRead(PIN_TO_DIGITAL((port * 8) + i))){
            out |= (1 << i);
        }
    }
    return out;
}

/**
 * 解析处理firmata协议包里的数字口的操作，对应pymata4的digital_pin_write
 */
static void firmata_setDigitalPinValue(uint8_t pin, uint8_t value){    
    if (pin < TOTAL_PINS){
        if (getPinMode(pin) == PIN_MODE_OUTPUT){
            digitalWrite(pin, value);
        }
    }
}

/**
 * 上报对应端口的值
 */
static void firmata_sendDigitalPort(uint8_t portNumber, uint16_t portData){
    uint8_t buf=DIGITAL_MESSAGE | (portNumber & 0xF);
    uart_write(&buf,1);
    // Tx bits  0-6 (protocol v1 and higher)
    // Tx bits 7-13 (bit 7 only for protocol v2 and higher)
    encodeByteStream(sizeof(portData), (uint8_t *)&portData, sizeof(portData));
}

/**
 * 输出数字口的状态，以端口为单位
 * 每位代表一个数字口
 * forceSend为false时，发现数字口的电平与之前的状态不一致，则上报给firmata
 * forceSend为true时，直接将当前状态上报给firmata
 */
static void firmata_outputPort(uint8_t portNumber, uint8_t portValue, uint8_t forceSend){
    portValue = portValue & portConfigInputs[portNumber];
    if (forceSend || previousPINs[portNumber] != portValue)
    {
        firmata_sendDigitalPort(portNumber, portValue);
        previousPINs[portNumber] = portValue;
    }
}

/**
 * 解析处理firmata协议包里需要设置上报哪些数字口的状态
 *
 */
static void firmata_setReportDigital(uint8_t port, int value){
    if (port < TOTAL_PORTS){
        reportPINs[port] = (uint8_t)value;
        if (value){
        }
    }
}
/**
 * 检查所有输入模式下的数字口的状态是否变化
 */
void firmata_checkDigitalInputs(){
    if (TOTAL_PORTS > 0 && reportPINs[0])
        firmata_outputPort(0, firmata_digitalRead(0, portConfigInputs[0]), FALSE);
    if (TOTAL_PORTS > 1 && reportPINs[1])
        firmata_outputPort(1, firmata_digitalRead(1, portConfigInputs[1]), FALSE);
}

/************************************************************************/
/*                          模拟口相关操作                               */
/************************************************************************/
/**
 * 设置哪些模拟口需要上报数据
 */
static void firmata_setReportAnalog(uint8_t analogPin, int value){
    if (analogPin < TOTAL_ANALOG_PINS){
        if (value == 0){
            analogInputsToReport = analogInputsToReport & ~(1 << analogPin);
        }else{
            analogInputsToReport = analogInputsToReport | (1 << analogPin);
        }
    }
}

/**
 * 解析处理firmata协议包里的模拟口的操作
 * pin =(0~5),对应A0~A5
 */

static void firmata_analogWrite(uint8_t pin, uint8_t duty, int freq){
    if (pin < TOTAL_PINS){
        switch (getPinMode(pin)){
            case PIN_MODE_SERVO:
                break;
            case PIN_MODE_PWM:
                set_PWM(pin,duty,freq);
                break;
            default:
                break;
        }
    }
}

/**
 * 根据firmata协议打包模拟量数据并上传
 */
void firmata_sendAnalog(uint8_t pin, uint16_t value){
    uint8_t buf;
    if ((0xF >= pin) && (0x3FFF >= value)){
        buf = ANALOG_MESSAGE | pin;
        uart_write(&buf,1);
        encodeByteStream(sizeof(value), (uint8_t *)&value, sizeof(value));
    }else{
        uint8_t buffer[3]={0xff,0xff,0xff};
        uart_write(buffer,3);
    }
}

/**
 * 主循环用来读取模拟口的值并上报
 */
void firmata_reportAnalog(void){
    uint8_t analogPin;
    for (uint8_t pin = 0; pin < TOTAL_PINS; pin++){
        if (pin < TOTAL_PINS){
            if (IS_PIN_ANALOG(pin) && getPinMode(pin) == PIN_MODE_ANALOG){
                /*用以将模拟口A0~A8转为 0~8*/
                analogPin = PIN_TO_ANALOG(pin);
                if (analogInputsToReport & (1 << analogPin)){
                    firmata_sendAnalog(analogPin, analogRead(analogPin));
                }
            }
        }
    }
}

/************************************************************************/
/*                            IIC相关操作                                */
/************************************************************************/
struct i2c_device_info{
    uint8_t addr;
    int reg;
    uint8_t bytes;
    uint8_t stopTX;
};

// struct i2c_device_info query[I2C_MAX_QUERIES];
// unsigned int i2cReadDelayTime = 0;
// uint8_t i2cRxData[64];
// bool isI2CEnabled = FALSE;
// signed char queryIndex = -1;

void readAndReportData(uint8_t address, int theRegister, uint8_t numBytes, uint8_t stopTX){

}

void enableI2CPins(){
    // uint8_t i;
    // for (i = 0; i < TOTAL_PINS; i++){
    //     if (IS_PIN_I2C(i)){
    //         setPinMode(i, PIN_MODE_I2C);
    //     }
    // }
    // isI2CEnabled = TRUE;
}

/************************************************************************/
/*                            DHT相关操作                                */
/************************************************************************/
void firmata_reportDHT()
{
    uint8_t sendbuf[10];
    if (dhtLoopCounter++ > dhtNumLoops)
    {
        if (numActiveDHTs)
        {
            int rv = get_dht11(nextDHT);
            uint8_t current_pin = DHT_PinNumbers[nextDHT];
            uint8_t current_type = DHT_TYPE[nextDHT];
            dhtLoopCounter = 0;
            currentDHT = nextDHT;
            if (nextDHT++ >= numActiveDHTs - 1)
            {
                nextDHT = 0;
            }
            if (rv == DHTLIB_OK)
            {
                uint8_t sum = _bits[0] + _bits[1] + _bits[2] + _bits[3];
                if (_bits[4] != sum)
                {
                    rv = -1;
                }
            }
            else
            {
                return;
            }
            sendbuf[0] = START_SYSEX;
            sendbuf[1] = DHT_DATA;
            sendbuf[2] = current_pin;
            sendbuf[3] = current_type;
            for (uint8_t i = 0; i < 4; ++i)
            {
                sendbuf[i+3] = current_pin;
            }
            sendbuf[7] = abs(rv);
            sendbuf[8] = END_SYSEX;
            uart_write(sendbuf,9);
        }
    }
}

/************************************************************************/
/*                            超声波测距相关操作                          */
/************************************************************************/
void firmata_reportSonar(){

}

/*!
    \brief      cofigure the I2C0 interfaces
    \param[in]  none
    \param[out] none
    \retval     none
*/

bool isI2CEnabled=0;
extern void i2c1_config(void);
extern void spi0_Config(void);
void i2c_config(void)
{
    i2c1_config();
    isI2CEnabled=1;
}

bool isSPIEnabled=0;

void spi0_config(void){
    spi0_Config();
    isSPIEnabled=1;
}

/**
 * 处理及回应 SYSEX-BASED commands
 */
static void reoprt_sysex(uint8_t command, uint8_t argc, uint8_t *argv){
    uint8_t buf[4];
    uint8_t mode;
    uint8_t slaveAddress;
    uint8_t iic_sendbuf[10];
    int slaveRegister;
    uint8_t data;
    int DHT_Pin;
    int DHT_type;
    int j=0;
    switch (command)
    {
        case RU_THERE:
            buf[0] = START_SYSEX;
            buf[1] = I_AM_HERE;
            buf[2] = ARDUINO_INSTANCE_ID;
            buf[3] = END_SYSEX;
            uart_write(buf, 4);
            break;
        case I2C_REQUEST:
            mode = argv[1] & I2C_READ_WRITE_MODE_MASK;
            if (argv[1] & I2C_10BIT_ADDRESS_MODE_MASK){
                return;
            }else{
                slaveAddress = argv[0];
            }

            switch (mode){
                case I2C_WRITE:
                    for (uint8_t i = 2; i < argc; i += 2)
                    {
                        iic_sendbuf[j++] = argv[i] + (argv[i + 1] << 7);
                    }
                    sendmessage(slaveAddress, iic_sendbuf, j);
                    break;
                case I2C_READ:
                    if (argc == 6){
                        slaveRegister = argv[2] + (argv[3] << 7);
                        data = argv[4] + (argv[5] << 7);
                    }else{
                        slaveRegister = I2C_REGISTER_NOT_SPECIFIED;
                        data = argv[2] + (argv[3] << 7);
                    }
                    // uart_write(&argv[3], 1);
                    receivemessage(slaveAddress, (int)slaveRegister, data);
                    break;
                case I2C_READ_CONTINUOUSLY:
                    break;
                case I2C_STOP_READING:
                    break;
                default:
                    break;
            }
            break;
        case I2C_CONFIG:
            if (!isI2CEnabled)
                i2c_config();
            break;
        case SPI_CONFIG:
            if (!isSPIEnabled)
                spi0_config();
            break;
        case SPI_REQUEST:
            
            break;
        case ANALOG_MAPPING_QUERY: 
            break;
        case SAMPLING_INTERVAL: /*设置主循环的轮询速率，暂未设置，目前以最快速度轮询*/
            break;
        case SERVO_CONFIG:
            break;
        case TONE_DATA:
            break;
        case SONAR_CONFIG: //超声波测距配置
            break;
        case DHT_CONFIG:
            DHT_Pin = argv[0];
            DHT_type = argv[1];
            if (numActiveDHTs < MAX_DHTS){
                if (DHT_type == 22){
                    DHT_WakeUpDelay[numActiveDHTs] = 1;
                }else if(DHT_type == 11){
                    DHT_WakeUpDelay[numActiveDHTs] = 18;
                }
                DHT_PinNumbers[numActiveDHTs] = DHT_Pin;
                DHT_TYPE[numActiveDHTs] = DHT_type;
                setPinMode(DHT_Pin, PIN_MODE_DHT);
                int rv = get_dht11(numActiveDHTs);
                if (rv == DHTLIB_OK){
                    numActiveDHTs++;
                    dhtNumLoops = dhtNumLoops / numActiveDHTs;
                }
            }
            break;
        //如果接收到DFRobot私有协议，那么就按照私有协议处理
        case DFROBOT_MESSAGE:
            break;
        default:
            break;
        }
}

/**
 * 从dataBuffer中解析接收到的完整数据包
 */
void firmata_processSysexMessage(void){
    switch (dataBuffer[0]){ /*第一位是命令位*/
        case REPORT_FIRMWARE:
            reoprt_firmware();
            break;
        case STRING_DATA:
            break;
        default:
            reoprt_sysex(dataBuffer[0], sysexBytesRead - 1, dataBuffer + 1);
            break;
    }
}

bool firmata_bufferDataAtPosition(const uint8_t data, const size_t pos){
    bool bufferOverflow = (pos >= dataBufferSize);
    // Write data to buffer if no overflow condition persist
    if (!bufferOverflow){
        dataBuffer[pos] = data;
    }
    return bufferOverflow;
}

void firmata_CDC_init(usb_core_driver *dev){
    DEV=dev;
}

void uart_write(uint8_t data[],uint8_t len){
    cdc_acm_data_send(DEV, data, len);
}

void firmata_parse(uint8_t inputData)
{
    uint8_t command;
    if (parsingSysex){
        if (inputData == END_SYSEX){ /*当接收到的数据为0xF7时，代表数据包尾，结束命令传输。开始处理命令*/
            parsingSysex = FALSE;
            firmata_processSysexMessage();
        }else{
            firmata_bufferDataAtPosition(inputData, sysexBytesRead);
            ++sysexBytesRead;
        }
    }else if ((waitForData > 0) && (inputData < 128)){
        --waitForData;
        firmata_bufferDataAtPosition(inputData, waitForData); /*把第一位数据存在最后面，依次往前存*/
        if ((waitForData == 0) && executeMultiByteCommand){
            switch (executeMultiByteCommand){
                case ANALOG_MESSAGE:
                    firmata_analogWrite(multiByteChannel, (dataBuffer[2] & 0x7f), ((dataBuffer[1] & 0x07) << 7) + (dataBuffer[0] & 0x7f));
                    break;
                case DIGITAL_MESSAGE:
                    firmata_digitalWrite(multiByteChannel, (dataBuffer[0] << 7) + dataBuffer[1]);
                    break;
                case SET_PIN_MODE:
                    firmata_setPinMode(dataBuffer[1], dataBuffer[0]);
                    break;
                case SET_DIGITAL_PIN_VALUE:
                    firmata_setDigitalPinValue(dataBuffer[1], dataBuffer[0]);
                    break;
                case REPORT_ANALOG:
                    firmata_setReportAnalog(multiByteChannel, dataBuffer[0]);
                    break;
                case REPORT_DIGITAL:
                    firmata_setReportDigital(multiByteChannel, dataBuffer[0]);
                    break;
            }
            executeMultiByteCommand = 0;
        }
    }else{
        if (inputData < 0xF0){
            command = inputData & 0xF0;
            multiByteChannel = inputData & 0x0F;
        }else{
            command = inputData;
        }
        switch (command){
            case ANALOG_MESSAGE:
                waitForData = 3;
                executeMultiByteCommand = command;
                break;
            case DIGITAL_MESSAGE:
            case SET_PIN_MODE:
                waitForData = 2;
                executeMultiByteCommand = command;
                break;
            case SET_DIGITAL_PIN_VALUE:
                waitForData = 2;
                executeMultiByteCommand = command;
                break;
            case REPORT_ANALOG:
            case REPORT_DIGITAL:
                waitForData = 1;
                executeMultiByteCommand = command;
                break;
            case START_SYSEX: /*接收到的数据为0xF0,代表协议包头，开始传输*/
                parsingSysex = TRUE;
                sysexBytesRead = 0;
                break;
            case SYSTEM_RESET:
                break;
            case REPORT_VERSION:
                break;
        }
    }
}