#ifndef DATAGRAM_H
#define DATAGRAM_H

#include <stdint.h>
#include <stdbool.h>

#define DG_TRANSMIT_MAX (1<<7)

void datagramTxStart(uint8_t );
void datagramTxStartLocal(uint8_t );
void datagramTxOutByte(const uint8_t c);
void datagramTxOut(const uint8_t *data, int l);
void datagramTxEnd(void);
bool datagramRxInputChar(const uint8_t c);

#define DG_STATUS        0
#define DG_HEARTBEAT     1
#define DG_CONSOLE       2
#define DG_LOGDATA       3
#define DG_LOGINFO       4
#define DG_PARAMS        5
#define DG_SIMLINK       8
#define DG_PING          9
#define DG_DISCONNECT    10
#define DG_AIRDATA       11
#define DG_CONFIG        12
#define DG_ANNUNCIATOR   13

extern void datagramInterpreter(uint8_t t, uint8_t *data, int size);
extern void datagramSerialOut(uint8_t);
extern void datagramRxError(const char *, uint16_t code);
  
extern uint16_t maxDatagramSize;
extern uint8_t datagramRxStore[];
extern bool datagramLocalOnly;

struct SimLinkSensor {
  float alpha, alt, ias;
  float roll, pitch, heading;
  float rrate, prate, yrate;
  float accx, accy, accz;
};

struct TelemetryData {
  float alpha;
  float IAS;
};

struct TelemetryConfig {
  float load;
  float trim;
  float maxAlpha, shakerAlpha, threshAlpha, minAlpha;
  float trimIAS, stallIAS, margin;
};

struct SimLinkControl {
  float aileron, elevator, throttle, rudder;
};

#endif
