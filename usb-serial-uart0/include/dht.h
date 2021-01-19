#include "stdint.h"

#define MAX_DHTS 6
#define DHT_INTER_PING_INTERVAL 2000 
#define DHTLIB_ERROR_TIMEOUT -2
#define DHTLIB_ERROR_CHECKSUM -1
#define DHTLIB_OK 0

#define DHTLIB_TIMEOUT (96000000 / 40000)

extern int dhtNumLoops;
extern int dhtLoopCounter;
extern int numActiveDHTs;
extern uint8_t _bits[];
extern uint8_t DHT_PinNumbers[];
extern uint8_t DHT_WakeUpDelay[];
extern uint8_t DHT_TYPE[];
extern uint8_t nextDHT;
extern uint8_t currentDHT;

int get_dht11(int index);