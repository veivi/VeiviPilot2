#ifndef CONFIG_HAL_BOARD

#include <string.h>
#include "StaP.h"
#include "CRC16.h"
#include "Math.h"
#include "RxInput.h"
#include "NVState.h"
#include "Objects.h"
#include "platform.h"
#include "drivers/bus.h"
#include "drivers/bus_i2c.h"
#include "drivers/bus_spi.h"
#include "io/serial.h"
#include "drivers/system.h"
#include "drivers/time.h"
#include "flight/servos.h"
#include "drivers/bus_i2c.h"
#include "drivers/rx/rx_pwm.h"
#include "fc/runtime_config.h"

volatile uint8_t nestCount = 0;

#define TRACE_BUFSIZE 0x300

volatile char traceBuf[TRACE_BUFSIZE];
volatile uint16_t traceLen, traceOverflow;
volatile bool tracing;

void *stap_traceEnable(bool v)
{
  void *handle = NULL;

  STAP_FORBID;
  tracing = v;
  if(tracing)
    handle = (void*) traceLen;
  STAP_PERMIT;
  
  return handle;
}

void stap_traceDisregard(void *handle)
{
  STAP_FORBID;
  traceLen = (uint16_t) handle;
  traceBuf[traceLen] = '\0';
  STAP_PERMIT;
}

bool stap_trace(const char *s)
{
  if(!tracing)
    return true;
  
  int l = strlen(s);
  bool success = false;
  
  STAP_FORBID;
  
  if(traceLen + l < TRACE_BUFSIZE) {
    memcpy(&traceBuf[traceLen], s, l+1);
    traceLen += l;
    success = true;
  } else
    traceOverflow++;

  STAP_PERMIT;
  
  return success;
}

bool stap_trace_char(char c)
{
  const char buf[] = { c, '\0' };
  return stap_trace(buf);
}
  
bool stap_trace_uint(unsigned int v)
{
  char buf[8];
  uint8_t len;

  while(v > 0 && len < 8) {
    uint8_t d = v & 0xF;
    buf[len++] = d < 10 ? '0' + d : 'a' + d - 10;
    v >>= 4;
  }

  if(len > 0) {
    stap_trace("0x");

    while(len-- > 0)
      stap_trace_char(buf[len]);

    stap_trace(" ");
  } else
    stap_trace("0 ");
  
  return true;
}

void stap_initialize(void)
{
}

void stap_reboot(bool bootloader)
{
  if(bootloader)
    systemResetToBootloader();
  else
    systemReset();
}

uint8_t BFSTAP_I2CWait(uint8_t d)
{
  void *handle = STAP_TRACEON;
  bool status = i2cWait(I2C_DEVICE, d);
  STAP_TRACEOFF;
  if(status)
    STAP_TRACEDIS(handle);
  return status ? 0 : i2cGetErrorCode();
}

uint16_t BFSTAP_I2CErrorCount(void)
{
  return i2cGetErrorCounterReset();
}

uint16_t BFSTAP_I2CErrorCode(void)
{
  return i2cGetErrorCodeReset();
}

#define MAX_BUFFER 0x100

uint8_t BFSTAP_I2CWrite(uint8_t d, const uint8_t *a, uint8_t as, const STAP_I2CBuffer_t *b, int c)
{
  uint8_t buffer[MAX_BUFFER];
  uint16_t total = 0;

  for(int i = 0; i < c; i++) {
    if(total + b[i].size > MAX_BUFFER)
      break;
    
    memcpy(&buffer[total], b[i].data, b[i].size);
    total += b[i].size;
  }

  void *handle = STAP_TRACEON;
  bool status = i2cWriteGeneric(I2C_DEVICE, d, as, a, total, buffer);
  STAP_TRACEOFF;
  
  if(status)
    STAP_TRACEDIS(handle);
  
  return status ? 0 : i2cGetErrorCode();
}

uint8_t BFSTAP_I2CRead(uint8_t d, const uint8_t *a, uint8_t as, uint8_t *b, uint8_t bs)
{
  void *handle = STAP_TRACEON;
  bool status = i2cReadGeneric(I2C_DEVICE, d, as, a, bs, b);
  STAP_TRACEOFF;
  
  if(status)
    STAP_TRACEDIS(handle);
  
  return status ? 0 : i2cGetErrorCode();
}

#include "sensors/acceleration.h"
#include "sensors/gyro.h"
#include "flight/imu.h"

bool stap_gyroUpdate(void)
{
  gyroUpdate(stap_timeMicros());
  return true;
}

static rollAndPitchTrims_t trims = { 0, 0 };

bool stap_accUpdate(void)
{
  accUpdate(stap_timeMicros(), &trims);
  return true;
}

bool stap_attiUpdate(void)
{
  imuUpdateAttitude(stap_timeMicros());
  return true;
}

bool stap_sensorRead(stap_Vector3f_t *a, stap_Vector3f_t *atti, stap_Vector3f_t *rot)
{
  // Angular velocities
  
  rot->x = gyroRateDps(FD_ROLL)/RADIAN/10.0f;
  rot->y = -gyroRateDps(FD_PITCH)/RADIAN/10.0f;
  rot->z = -gyroRateDps(FD_YAW)/RADIAN/10.0f;

  // Attitude

  atti->x = attitude.values.roll/RADIAN/10.0f;
  atti->y = -attitude.values.pitch/RADIAN/10.0f;
  atti->z = attitude.values.yaw/RADIAN/10.0f;

  // Acceleration
 
  a->x = acc.accADC[X]/200;
  a->y = -acc.accADC[Y]/200;
  a->z = acc.accADC[Z]/200;

  return true;
}

bool stap_baroUpdate(void)
{
  // barometer.update();
  // barometer.accumulate();
  return true;
}

float stap_baroRead(void)
{
  // return (float) barometer.get_altitude();
  return 0;
}

#include "io/serial.h"

serialPort_t *stap_serialPort[4];


/*
int BFSTAP_LinkStatus(int port)
{
  if(traceLen > 0) {
    char buffer[TRACE_BUFSIZE];
    bool overflow = traceOverflow > 0;
    
    STAP_FORBID;
    
    memcpy(buffer, traceBuf, traceLen+1);
    traceLen = 0;
    traceOverflow = 0;
    
    STAP_PERMIT;
    
    consoleNote_P(CS_STRING("TRACE : "));
    consolePrintLn(buffer);
    
    if(overflow > 0)
      consoleNoteLn_P(CS_STRING("TRACE OVERFLOW"));
  }
  
  if(stap_serialPort[port] && serialRxBytesWaiting(stap_serialPort[port]))
    return 1;
  
  return 0;
}
*/

int stap_hostReceive(uint8_t *buffer, int size)
{
  while(size-- > 0)
    *buffer++ = BFSTAP_LinkGetChar(0);

  return 0;
}

uint8_t BFSTAP_LinkGetChar(int port)
{
  if(stap_serialPort[port])
    return serialRead(stap_serialPort[port]);
  else
    return 0;
}

int BFSTAP_LinkStatus(int port)
{
  if(stap_serialPort[port]) {
    if(STAP_LINKDIR(port))
      return (int) serialTxBytesFree(stap_serialPort[port]);
    else
      return serialRxBytesWaiting(stap_serialPort[port]) ? 1 : 0;
  }
  
  return 1;
}

int BFSTAP_LinkPutNB(int port, const uint8_t *buffer, int size)
{
  int len = BFSTAP_LinkStatus(port);
  if(len > size)
    len = size;
  if(stap_serialPort[port])
  serialWriteBuf(stap_serialPort[port], buffer, len);
  return len;
}

void BFSTAP_LinkPut(int port, const uint8_t *buffer, int size)
{
  int left = size;
  
  while(left > 0) {
    int len = BFSTAP_LinkPutNB(port, buffer, left);
    buffer += len;
    left -= len;
  }
}

void BFSTAP_LinkPutChar(int port, uint8_t c)
{
  BFSTAP_LinkPut(port, &c, 1);
}

void BFSTAP_LinkDrain(int port)
{
  if(!stap_serialPort[port])
    return;
  while(!isSerialTransmitBufferEmpty(stap_serialPort[port]));
}

STAP_MICROS_T stap_timeMicros(void)
{
  return (STAP_MICROS_T) micros();
}
    
uint32_t stap_memoryFree(void)
{
  // return hal.util->available_memory();
  return 2000;
}

void BFSTAP_pwmOutput(int num, const uint16_t value[], const bool active[])
{
  int i = 0;

  for(i = 0; i < num; i++) {
    if(active[i])
      pwmWriteServo(i, value[i]);
  }
}

void BFSTAP_rxInputPoll(void)
{
  uint16_t input[MAX_CH];

  if(isPPMDataBeingReceived()) {
    resetPPMDataReceivedState();
    
    for(int i = 0; i < MAX_CH; i++)
      input[i] = ppmRead(i);

    inputSource(input, MAX_CH);
  }
}

#endif
