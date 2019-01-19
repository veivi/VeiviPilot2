#ifndef CONFIG_HAL_BOARD

#include <string.h>
#include "StaP.h"
#include "CRC16.h"
#include "Math.h"
#include "RxInput.h"
#include "NVState.h"
#include "platform.h"
#include "drivers/bus.h"
#include "drivers/bus_i2c.h"
#include "drivers/bus_spi.h"
#include "io/serial.h"
#include "drivers/system.h"
#include "drivers/time.h"
#include "drivers/bus_i2c.h"
#include "drivers/rx/rx_pwm.h"
#include "fc/runtime_config.h"

uint8_t nestCount = 0;
static uint16_t sensorHash = 0xFFFF;

void stap_reboot(bool bootloader)
{
  if(bootloader)
    systemResetToBootloader();
  else
    systemReset();
}

void stap_I2cInit(void)
{
  // i2cInit(I2C_DEVICE);
}

uint8_t stap_I2cWait(uint8_t d)
{
#ifdef STM32F4
  return 1;
#else
  bool status = i2cWait(I2C_DEVICE, d);
  return status ? 0 : i2cGetErrorCode();
#endif
}

#define MAX_BUFFER 0x100

uint16_t stap_i2cErrorCount(void)
{
#ifdef STM32F4
  return 0;
#else
  return i2cGetErrorCounterReset();
#endif
}

uint16_t stap_i2cErrorCode(void)
{
#ifdef STM32F4
  return 0;
#else
  return i2cGetErrorCodeReset();
#endif
}

uint8_t stap_I2cWrite(uint8_t d, const uint8_t *a, uint8_t as, const I2CBuffer_t *b, int c)
{
  uint8_t buffer[MAX_BUFFER];
  uint8_t total = 0;

  for(int i = 0; i < c; i++) {
    if(total + b[i].size > MAX_BUFFER)
      return 1;
    
    memcpy(&buffer[total], b[i].data, b[i].size);
    total += b[i].size;
  }

#ifdef STM32F4
  if(as == 0)
    return !i2cWriteBuffer(I2C_DEVICE, d, 0xFF, total, buffer);
  else if(as == 1)
    return !i2cWriteBuffer(I2C_DEVICE, d, a[0], total, buffer);
  else
    return 1;
#else
  bool status = i2cWriteGeneric(I2C_DEVICE, d, as, a, total, buffer);

  return status ? 0 : i2cGetErrorCode();
#endif
}

uint8_t stap_I2cRead(uint8_t d, const uint8_t *a, uint8_t as, uint8_t *b, uint8_t bs)
{
#ifdef STM32F4
  if(as == 0)
    return !i2cRead(I2C_DEVICE, d, 0xFF, bs, b);
  else if(as == 1)
    return !i2cRead(I2C_DEVICE, d, a[0], bs, b);
  else
    return 1;
#else
  bool status = i2cReadGeneric(I2C_DEVICE, d, as, a, bs, b);
  
  return status ? 0 : i2cGetErrorCode();
#endif
}

#include "sensors/acceleration.h"
#include "sensors/gyro.h"
#include "flight/imu.h"

bool stap_gyroUpdate(void)
{
  gyroUpdate(stap_timeMicros());
  return true;
}

bool stap_accUpdate(void)
{
  static rollAndPitchTrims_t trims;
  
  accUpdate(stap_timeMicros(), &trims);
}

bool stap_attiUpdate(void)
{
  if(vpStatus.armed)
    ENABLE_ARMING_FLAG(ARMED);
  else
    DISABLE_ARMING_FLAG(ARMED);
    
  imuUpdateAttitude(stap_timeMicros());
}

bool stap_sensorRead(stap_Vector3f_t *a, stap_Vector3f_t *atti, stap_Vector3f_t *rot)
{
  // Angular velocities
  
  rot->x = gyroRateDps(FD_ROLL)/RADIAN/10.0f;
  rot->y = -gyroRateDps(FD_PITCH)/RADIAN/10.0f;
  rot->z = gyroRateDps(FD_YAW)/RADIAN/10.0f;

  // Attitude

  atti->x = attitude.values.roll/RADIAN/10.0f;
  atti->y = -attitude.values.pitch/RADIAN/10.0f;
  atti->z = attitude.values.yaw/RADIAN/10.0f;

  // Acceleration
 
  a->x = acc.accADC[X]/200;
  a->y = acc.accADC[Y]/200;
  a->z = acc.accADC[Z]/200;

  return true;
}

bool stap_baroInit(void)
{
  // barometer.init();
  // barometer.calibrate();
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

extern serialPort_t *stap_serialPort;


bool stap_hostInit(void)
{
  return true;
}

int stap_hostReceiveState(void)
{
  if(serialRxBytesWaiting(stap_serialPort))
    return 1;
  return 0;
}

int stap_hostReceive(uint8_t *buffer, int size)
{
  while(size-- > 0)
    *buffer++ = stap_hostReceiveChar();

  return 0;
}

uint8_t stap_hostReceiveChar(void)
{
  return serialRead(stap_serialPort);
}

int stap_hostTransmitState(void)
{
  return (int) serialTxBytesFree(stap_serialPort);
}

int stap_hostTransmitNonblock(const uint8_t *buffer, int size)
{
  int len = stap_hostTransmitState();
  if(len > size)
    len = size;
  serialWriteBuf(stap_serialPort, buffer, len);
  return len;
}

int stap_hostTransmit(const uint8_t *buffer, int size)
{
  int left = size;
  
  while(left > 0) {
    int len = stap_hostTransmitNonblock(buffer, left);
    buffer += len;
    left -= len;
  }
  
  return size;
}

int stap_hostTransmitChar(uint8_t c)
{
  return stap_hostTransmit(&c, 1);
}

void stap_hostFlush()
{
  while(!isSerialTransmitBufferEmpty(stap_serialPort));
}

void stap_entropyDigest(const uint8_t *value, int size)
{
  sensorHash = crc16(sensorHash, value, size);
  srand(sensorHash);
}

uint32_t stap_currentMicros;

uint32_t stap_timeMicros(void)
{
  stap_currentMicros = (uint32_t) micros();
  return stap_currentMicros;
}
    
uint32_t stap_timeMillis(void)
{
  return millis();
}
    
void stap_delayMicros(uint32_t x)
{
  uint32_t current = stap_timeMicros();
  while(stap_timeMicros() < current+x);
}
  
void stap_delayMillis(uint32_t x)
{
  uint32_t current = stap_timeMillis();
  while(stap_timeMillis() < current+x);
}
  
uint32_t stap_memoryFree(void)
{
  // return hal.util->available_memory();
  return 2000;
}

void stap_servoOutputInit(void)
{
  /*
  int i = 0;
  
  for(i = 0; i < MAX_SERVO && pwmOutput[i].timer; i++) {
    setPinState(&pwmOutput[i].pin, 0);
    configureOutput(&pwmOutput[i].pin);
    pwmDisable(&pwmOutput[i]);
    pwmOutput[i].active = false;
  }

  pwmTimerInit(hwTimers, sizeof(hwTimers)/sizeof(struct HWTimer*));
  */
}

void stap_servoOutput(int i, float fvalue)
{
  /*
  struct PWMOutput *output = NULL;

  if(i >= 0 && i < MAX_SERVO)
    output = &pwmOutput[i];
    
  if(!output || !output->timer)
    return;

  uint16_t value = NEUTRAL + RANGE*fvalue;

  *(output->timer->OCR[output->pwmCh]) = constrain_period(value) << 1;

  if(!output->active) {
    configureOutput(&output->pin);
    pwmEnable(output);
    output->active = true;
  }
  */
}

void stap_rxInputInit(void)
{
}

void stap_rxInputPoll(void)
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
