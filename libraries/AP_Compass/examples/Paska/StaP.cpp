extern "C" {
#include "StaP.h"
#include "Console.h"
#include "CRC16.h"
}

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL_AVR/AP_HAL_AVR.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_AHRS/AP_AHRS.h>

#include "NewI2C.h"

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;
AP_Baro barometer;
AP_InertialSensor ins;
AP_GPS gps;
AP_AHRS_DCM ahrs {ins,  barometer, gps};

NewI2C I2c = NewI2C();

uint8_t nestCount = 0;
static uint16_t sensorHash = 0xFFFF;

extern "C" bool stap_boot(void)
{
  hal.init(0, NULL);
  vpStatus.consoleLink = true;
  return true;
}  

extern "C" bool stap_gyroInit(void)
{
  ins.init(AP_InertialSensor::COLD_START, AP_InertialSensor::RATE_50HZ);
  ahrs.init();
  return true;
}

extern "C" bool stap_gyroUpdate(void)
{
  ins.wait_for_sample();
  ahrs.update();
  return true;
}

extern "C" bool stap_gyroRead(stap_Vector3f_t *acc, stap_Vector3f_t *atti, stap_Vector3f_t *rot)
{
  // Acceleration
  
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

  return true;
}

extern "C" bool stap_baroInit(void)
{
  barometer.init();
  barometer.calibrate();
  return true;
} 

extern "C" bool stap_baroUpdate(void)
{
  barometer.update();
  barometer.accumulate();
  return true;
  return true;
}

extern "C" float stap_baroRead(void)
{
  return (float) barometer.get_altitude();
}

extern "C" bool stap_hostInit(void)
{
  return true;
}

extern "C" int stap_hostReceiveState(void)
{
  return hal.console->available();
}

extern "C" int stap_hostReceive(uint8_t *buffer, int size)
{
  while(size-- > 0)
    *buffer++ = stap_hostReceiveChar();

  return 0;
}

extern "C" uint8_t stap_hostReceiveChar(void)
{
  return hal.console->read();
}

extern "C" int stap_hostTransmitState(void)
{
  return 1;
}

extern "C" int stap_hostTransmit(const uint8_t *buffer, int size)
{
  while(size-- > 0)
    stap_hostTransmitChar(*buffer++);
  return 0;
}

extern "C" int stap_hostTransmitChar(uint8_t c)
{
  hal.uartA->write(c);
  return 1;
}

extern "C" void stap_hostFlush()
{
  // hal.uartA->flush();
}

void stap_entropyDigest(const uint8_t *value, int size)
{
  sensorHash = crc16(sensorHash, value, size);
  srand(sensorHash);
}

uint32_t currentTime;
  
extern "C" uint32_t currentMicros()
{
  currentTime = hal.scheduler->micros();
  return currentTime;
}
    
extern "C" uint32_t currentMillis()
{
  return hal.scheduler->millis();
}
    
extern "C" void delayMicros(uint32_t x)
{
  uint32_t current = currentMicros();
  while(currentMicros() < current+x);
}
  
extern "C" uint32_t stap_memoryFree(void)
{
  return hal.util->available_memory();
}
