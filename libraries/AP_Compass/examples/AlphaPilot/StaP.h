#ifndef STAP_H
#define STAP_H

#include <stdint.h>
#include <stdbool.h>

//
// Serial interface
//

#define STAP_LINK_HOSTRX    0
#define STAP_LINK_HOSTTX    1
#define STAP_LINK_TELEMRX   2
#define STAP_LINK_TELEMTX   3
#define STAP_LINK_SRXL      4
#define STAP_LINK_GPS       5

#define STAP_LINKDIR(i) ((bool[]) { false, true, false, true, false, false })[i]

#define STAP_LINKID(port) STAP_LINK_ ## port

//
// I2C interface
//

typedef struct {
    const uint8_t *data;
    uint8_t size;
} STAP_I2CBuffer_t;

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

#include "StaP_TARGET.h"

#endif

