int test(void);
void sendmessage(uint8_t i2caddr, uint8_t *sendbuf, uint8_t len);
void receivemessage(uint8_t address, int theRegister, uint8_t numBytes);
int test_recive(void);

#define TW_WRITE 0
#define TW_READ 1

#define TW_BUS_ERROR 0x00
#define TW_START 0x08

#define TW_REP_START 0x10
#define TW_MT_SLA_ACK 0x18
#define TW_MT_SLA_NACK 0x20
#define TW_MT_DATA_ACK 0x28
#define TW_MT_DATA_NACK 0x30
#define TW_MT_ARB_LOST 0x38
#define TW_MR_SLA_ACK 0x40
#define TW_MR_SLA_NACK 0x48
#define TW_MR_DATA_ACK 0x50
#define TW_MR_DATA_NACK 0x58
#define TW_SR_SLA_ACK 0x60
#define TW_SR_ARB_LOST_SLA_ACK 0x68
#define TW_SR_GCALL_ACK 0x70
#define TW_SR_ARB_LOST_GCALL_ACK 0x78
#define TW_SR_DATA_ACK 0x80
#define TW_SR_DATA_NACK 0x88
#define TW_SR_GCALL_DATA_ACK 0x90
#define TW_SR_GCALL_DATA_NACK 0x98
#define TW_SR_STOP 0xA0
#define TW_ST_SLA_ACK 0xA8
#define TW_ST_ARB_LOST_SLA_ACK 0xB0
#define TW_ST_DATA_ACK 0xB8
#define TW_ST_DATA_NACK 0xC0
#define TW_ST_LAST_DATA 0xC8
#define TW_NO_INFO 0xF8
#define TW_STATUS (TWSR & TW_STATUS_MASK)
#define TW_STATUS_MASK (_BV(TWS7) | _BV(TWS6) | _BV(TWS5) | _BV(TWS4) | _BV(TWS3))

#define I2C_WRITE 0b00000000
#define I2C_READ 0b00001000
#define I2C_READ_CONTINUOUSLY 0b00010000
#define I2C_STOP_READING 0b00011000
#define I2C_READ_WRITE_MODE_MASK 0b00011000
#define I2C_10BIT_ADDRESS_MODE_MASK 0b00100000
#define I2C_END_TX_MASK 0b01000000
#define I2C_STOP_TX 1
#define I2C_RESTART_TX 0
#define I2C_MAX_QUERIES 8
#define I2C_REGISTER_NOT_SPECIFIED -1