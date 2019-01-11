#ifndef CONFIG_HAL_BOARD

#include "StaP.h"
#include "CRC16.h"
#include "platform.h"
#include "drivers/bus.h"
#include "drivers/bus_i2c.h"
#include "drivers/bus_spi.h"
#include "io/serial.h"
#include "drivers/time.h"

uint8_t nestCount = 0;
static uint16_t sensorHash = 0xFFFF;

void stap_I2cInit(void)
{
  // i2cInit(I2C_DEVICE);
  /*
  I2c.begin();
  I2c.setSpeed(true);
  I2c.pullup(true);
  I2c.timeOut(10);
  */
}

uint8_t stap_I2cWait(uint8_t d)
{
  // return I2c.wait(d);
  return 1;
}
 
uint8_t stap_I2cWrite(uint8_t d, const uint8_t *a, uint8_t as, const I2CBuffer_t *b, int c)
{
  bool status = true;

  for(int i = 0; i < c; i++)
    if(!(status = i2cWriteBuffer(I2C_DEVICE, d, 0xFF, b[i].size, b[i].data)))
      break;

  return status ? 0 : 1;
}

uint8_t stap_I2cRead(uint8_t d, const uint8_t *a, uint8_t as, uint8_t *b, uint8_t bs)
{
  bool status = true;

  if(as == 1)
    status = i2cRead(I2C_DEVICE, d, *a, bs, b);

  return status ? 0 : 1;
}
 
bool stap_gyroInit(void)
{
  // ins.init(AP_InertialSensor::COLD_START, AP_InertialSensor::RATE_50HZ);
  // ahrs.init();
  return true;
}

bool stap_gyroUpdate(void)
{
  // ins.wait_for_sample();
  // ahrs.update();
  return true;
}

bool stap_gyroRead(stap_Vector3f_t *acc, stap_Vector3f_t *atti, stap_Vector3f_t *rot)
{
  // Acceleration
  /*  
  Vector3f ap_acc = ins.get_accel(0);

  acc->x = ap_acc.x;
  acc->y = ap_acc.y;
  acc->z = -ap_acc.z;
  
  // Attitude

  atti->x = ahrs.roll;
  atti->y = ahrs.pitch;
  atti->z = ahrs.yaw;
  
  // Angular velocities

  Vector3f ap_gyro = ins.get_gyro();
  
  rot->x = ap_gyro.x;
  rot->y = ap_gyro.y;
  rot->z = ap_gyro.z;
  */
  return false;
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
  return 0;
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
  // call inputSource() with the data
}
#endif
