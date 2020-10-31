#ifndef STAP_H
#define STAP_H

#include <stdint.h>
#include <stdbool.h>
#include "StaP_TARGET.h"

/*
 * Serial interface
 */

#define STAP_PORT_HOST    0
#define STAP_PORT_TELEM   1
#define STAP_PORT_SRXL    2
#define STAP_PORT_GPS     3

#define STAP_PORTID(port) STAP_PORT_ ## port
#define STAP_rxStatus(port) stap_rxStatus(STAP_PORTID(port))
#define STAP_rxGetChar(port) stap_rxGetChar(STAP_PORTID(port))
#define STAP_txStatus(port) stap_txStatus(STAP_PORTID(port))
#define STAP_txPutChar(port, c) stap_txPutChar(STAP_PORTID(port), c)
#define STAP_txPut(port, b, s) stap_txPut(STAP_PORTID(port), b, s)

int stap_rxStatus(int);
uint8_t stap_rxGetChar(int);
int stap_txStatus(int);
void stap_txPutChar(int, uint8_t);
void stap_txPut(int, const uint8_t *, int);

//
// I2C interface
//

typedef struct {
    const uint8_t *data;
    uint8_t size;
} STAP_I2CBuffer_t;

uint8_t stap_I2cWrite(uint8_t, const uint8_t*, uint8_t, const STAP_I2CBuffer_t*, int);
uint8_t stap_I2cRead(uint8_t, const uint8_t*, uint8_t, uint8_t*, uint8_t);
uint8_t stap_I2cWait(uint8_t);
uint16_t stap_i2cErrorCount(void);
uint16_t stap_i2cErrorCode(void);

//
// Trace support
//

#define STAP_TRACEON       stap_traceEnable(true)
#define STAP_TRACEOFF      stap_traceEnable(false)
#define STAP_TRACEDIS(h)   stap_traceDisregard(h)
#define STAP_TRACE(s)      stap_trace(s)
#define STAP_TRACE_T(s, t) stap_trace_ ## t(s)

void *stap_traceEnable(bool);
void stap_traceDisregard(void*);
bool stap_trace(const char *s);
bool stap_trace_uint(unsigned int v);
bool stap_trace_char(char c);

//
// System interface
//

typedef uint32_t STAP_MICROS_T;

void stap_initialize(void);
STAP_MICROS_T stap_timeMicros(void);
uint32_t stap_memoryFree(void);
void stap_reboot(bool bootloader);

//
// RX and Servo (PWM) interface
//

void stap_rxInputPoll(void);
uint16_t stap_rxFrameCount(void);
void stap_servoOutputInit(void);
void stap_servoOutput(int i, float fvalue);
void stap_servoOutputSync(void);

//
// Gyro interface
//

typedef struct stap_Vector3f {
  float x, y, z;
} stap_Vector3f_t;

bool stap_gyroUpdate(void);
bool stap_attiUpdate(void);
bool stap_accUpdate(void);
bool stap_sensorRead(stap_Vector3f_t *acc, stap_Vector3f_t *atti, stap_Vector3f_t *rot);

//
// Altimeter interface
//

bool stap_baroUpdate(void);
float stap_baroRead(void);

//
// Canopy latch interface
//


#endif

