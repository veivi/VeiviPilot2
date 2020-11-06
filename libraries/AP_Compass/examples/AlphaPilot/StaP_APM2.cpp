extern "C" {
#include "StaP.h"
#include "CRC16.h"
#include "Console.h"
#include "AlphaPilot.h"
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

volatile uint8_t nestCount = 0;

static uint16_t i2cErrorCode, i2cErrorCount;

AP_HAL::UARTDriver  *uartPorts[STAP_LINKS];

extern "C" void stap_reboot(bool bootloader)
{/*
  if(bootloader)
    systemResetToBootloader();
  else
    systemReset();
*/
  while(1);
}

extern "C" uint16_t APM2STAP_I2CErrorCount(void)
{
  uint16_t value = i2cErrorCount;
  i2cErrorCount = 0;
  return value;
}

extern "C" uint16_t APM2STAP_I2CErrorCode(void)
{
  uint16_t value = i2cErrorCode;
  i2cErrorCode = 0;
  return value;
}

extern "C" uint8_t APM2STAP_I2CWait(uint8_t d)
{
  uint8_t status = I2c.wait(d);
  if(status) {
    i2cErrorCount++;
    i2cErrorCode = status;
  }
  return status;
}
 
extern "C" uint8_t APM2STAP_I2CWrite(uint8_t d, const uint8_t *a, uint8_t as, const STAP_I2CBuffer_t *b, int c)
{
  uint8_t status = I2c.write(d, a, as, b, c);
  if(status) {
    i2cErrorCount++;
    i2cErrorCode = status;
  }
  return status;
}
  
extern "C" uint8_t APM2STAP_I2CRead(uint8_t d, const uint8_t *a, uint8_t as, uint8_t *b, uint8_t bs)
{
  uint8_t status = I2c.read(d, a, as, b, bs);
  if(status) {
    i2cErrorCount++;
    i2cErrorCode = status;
  }
  return status;
}
 
extern "C" bool stap_gyroUpdate(void)
{
  ins.wait_for_sample();
  ahrs.update();
  return true;
}

extern "C" bool stap_attiUpdate(void)
{
  return true;
}

extern "C" bool stap_accUpdate(void)
{
  return true;
}

extern "C" bool stap_sensorRead(stap_Vector3f_t *acc, stap_Vector3f_t *atti, stap_Vector3f_t *rot)
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

extern "C" bool stap_baroUpdate(void)
{
  barometer.update();
  barometer.accumulate();
  return true;
}

extern "C" float stap_baroRead(void)
{
  return (float) barometer.get_altitude();
}

extern "C" uint8_t APM2STAP_LinkStatus(uint8_t port) {
if(uartPorts[port]) {
  if(STAP_LINKDIR(port))
    return uartPorts[port]->txspace();
  else
    return uartPorts[port]->available();
 }

 return 0;
}

extern "C" uint8_t APM2STAP_LinkGetChar(uint8_t port) {
  if(uartPorts[port])
    return uartPorts[port]->read();
  else
    return 0;
}
  
extern "C" void APM2STAP_LinkPut(uint8_t port, const uint8_t *buffer, int size) {
  if(uartPorts[port])
    uartPorts[port]->write(buffer, size);
}

extern "C" void APM2STAP_LinkPutChar(uint8_t port, uint8_t c) {
  APM2STAP_LinkPut(port, &c, 1);
}

extern "C" STAP_MICROS_T stap_timeMicros(void)
{
  return (STAP_MICROS_T) hal.scheduler->micros();
}

extern "C" uint32_t stap_memoryFree(void)
{
  return hal.util->available_memory();
}

extern "C" {
#include <avr/io.h>
#include "RxInput.h"
#include <avr/interrupt.h>
#include "DSP.h"
#include "NVState.h"
  
extern const struct PortDescriptor portTable[];
  
extern const portName_t pcIntPort[];
extern const uint8_t pcIntMask[];
const struct PinDescriptor led = { PortA, 5 };
const struct PinDescriptor latch = { PortF, 0 };

const struct PortDescriptor portTable[] = {
  [PortA] = { &PINA, &PORTA, &DDRA },
  [PortB] = { &PINB, &PORTB, &DDRB, &PCMSK0, 0 },
  [PortC] = { &PINC, &PORTC, &DDRC },
  [PortD] = { &PIND, &PORTD, &DDRD },
  [PortE] = { &PINE, &PORTE, &DDRE },
  [PortF] = { &PINF, &PORTF, &DDRF },
  [PortG] = { &PING, &PORTG, &DDRG },
  [PortH] = { &PINH, &PORTH, &DDRH },
  [PortK] = { &PINK, &PORTK, &DDRK, &PCMSK2, 2 },
  [PortL] = { &PINL, &PORTL, &DDRL }
};

const portName_t pcIntPort[] = {
  PortB, PortF, PortK
};

const uint8_t pcIntMask[] = {
  1<<PCIE0, 1<<PCIE1, 1<<PCIE2
};

void pinOutputEnable(const struct PinDescriptor *pin, bool output)
{
  if(output)
    *(portTable[pin->port].ddr) |= 1<<(pin->index);
  else
    *(portTable[pin->port].ddr) &= ~(1<<(pin->index));
}  

void setPinState(const struct PinDescriptor *pin, uint8_t state)
{
  if(state > 0)
    *(portTable[pin->port].port) |= 1<<(pin->index);
  else
    *(portTable[pin->port].port) &= ~(1<<(pin->index));
}

uint8_t getPinState(const struct PinDescriptor *pin)
{
  return (*(portTable[pin->port].pin)>>(pin->index)) & 1;
}

void configureInput(const struct PinDescriptor *pin, bool pullup)
{
  pinOutputEnable(pin, false);
  setPinState(pin, pullup ? 1 : 0);
}

void configureOutput(const struct PinDescriptor *pin)
{
  pinOutputEnable(pin, true);
}

#define RC_OUTPUT_MIN_PULSEWIDTH 400
#define RC_OUTPUT_MAX_PULSEWIDTH 2100

typedef enum { COMnA = 0, COMnB = 1, COMnC = 2 } PWM_Ch_t;

struct HWTimer {
  volatile uint8_t *TCCRA, *TCCRB;
  volatile uint16_t *ICR;
  volatile uint16_t *OCR[3]; // 0... 2 = A ... C
  volatile uint16_t *TCNT;
  bool sync;
  int8_t log2scale;
};

struct PWMOutput {
  struct PinDescriptor pin;
  const struct HWTimer *timer;
  PWM_Ch_t pwmCh; // COMnA / COMnB / COMnC
  bool active;
};

#define MICROS_TO_CNT(t, x) (t->log2scale < 0 ? (x)>>(-t->log2scale) : (x)<<t->log2scale)

static const uint8_t outputModeMask[] = { 1<<COM1A1, 1<<COM1B1, 1<<COM1C1 };

//
// HW timer declarations
//

const struct HWTimer hwTimer1 =
  { &TCCR1A, &TCCR1B, &ICR1, { &OCR1A, &OCR1B, &OCR1C }, &TCNT1, SYNC_PWM_OUTPUT, -2 };
  // const struct HWTimer hwTimer3 =
  // { &TCCR3A, &TCCR3B, &ICR3, { &OCR3A, &OCR3B, &OCR3C }, &TCNT3, false, 1 };
const struct HWTimer hwTimer3 =
  { &TCCR3A, &TCCR3B, &ICR3, { &OCR3A, &OCR3B, &OCR3C }, &TCNT3, SYNC_PWM_OUTPUT, -2 };
const struct HWTimer hwTimer4 =
  { &TCCR4A, &TCCR4B, &ICR4, { &OCR4A, &OCR4B, &OCR4C }, &TCNT4, SYNC_PWM_OUTPUT, -2 };

const struct HWTimer *hwTimersOwn[] = 
  { &hwTimer1, &hwTimer3, &hwTimer4 };

const struct HWTimer hwTimer5 =
  { &TCCR5A, &TCCR5B, &OCR5A, { &OCR5A, &OCR5B, &OCR5C }, &TCNT5, false, 1 };

struct PWMOutput pwmOutput[MAX_SERVO] = {
  { { PortB, 6 }, &hwTimer1, COMnB },
  { { PortB, 5 }, &hwTimer1, COMnA },
  { { PortH, 5 }, &hwTimer4, COMnC },
  { { PortH, 4 }, &hwTimer4, COMnB },
  { { PortH, 3 }, &hwTimer4, COMnA },
  { { PortE, 5 }, &hwTimer3, COMnC },
  { { PortE, 4 }, &hwTimer3, COMnB },
  { { PortE, 3 }, &hwTimer3, COMnA },
  { { PortB, 7 }, &hwTimer1, COMnC },
  { { PortL, 4 }, &hwTimer5, COMnB },
  { { PortL, 5 }, &hwTimer5, COMnC }
};

static void pwmTimerInit(const struct HWTimer *timer[], int num)
{
  int i = 0, j = 0;
  
  for(i = 0; i < num; i++) { 
    // WGM, prescaling

    *(timer[i]->TCCRA) = 1<<WGM11;
    *(timer[i]->TCCRB) = (1<<WGM13) | (1<<WGM12) | (1<<CS11) | (timer[i]->sync ? (1<<CS10) : 0);
    
   // PWM frequency

    if(timer[i]->sync)
      *(timer[i]->ICR) = 0xFFFF;
    else
      *(timer[i]->ICR) = MICROS_TO_CNT(timer[i], 1000000UL/PWM_HZ) - 1;
    
   // Output set to 1.5 ms by default

    for(j = 0; j < 3; j++)
      *(timer[i]->OCR[j]) = MICROS_TO_CNT(timer[i], 1500U);
  }
}

static void pwmTimerSync(const struct HWTimer *timer[], int num)
{
  int i = 0;

  for(i = 0; i < num; i++) {
    if(*(timer[i]->TCNT) > MICROS_TO_CNT(timer[i], 5000U))
      *(timer[i]->TCNT) = 0xFFFF;
  }
}

static void pwmEnable(const struct PWMOutput *output)
{
   *(output->timer->TCCRA) |= outputModeMask[output->pwmCh];
}

static void pwmDisable(const struct PWMOutput *output)
{
   *(output->timer->TCCRA) &= ~outputModeMask[output->pwmCh];
}

static uint16_t constrain_period(uint16_t p) {
    if (p > RC_OUTPUT_MAX_PULSEWIDTH)
       return RC_OUTPUT_MAX_PULSEWIDTH;
    else if (p < RC_OUTPUT_MIN_PULSEWIDTH)
       return RC_OUTPUT_MIN_PULSEWIDTH;
    else return p;
}

void APM2stap_pwmOutput(uint8_t num, const uint16_t value[])
{
  int i = 0;

  for(i = 0; i < num; i++) {
    struct PWMOutput *output = &pwmOutput[i];
    
    if(value[i] == 0 || !output->timer)
      continue;

    *(output->timer->OCR[output->pwmCh])
      = MICROS_TO_CNT(output->timer, constrain_period(value[i]));
  
    if(!output->active) {
      configureOutput(&output->pin);
      pwmEnable(output);
      output->active = true;
    }
  }
  
#if SYNC_PWM_OUTPUT
  pwmTimerSync(hwTimersOwn, sizeof(hwTimersOwn)/sizeof(struct HWTimer*));
#endif
}
  
struct PinDescriptor ppmInputPin = { PortL, 1 }; 
  
#define AVR_RC_INPUT_MAX_CHANNELS 12
#define AVR_RC_INPUT_MIN_CHANNELS 6

/*
  mininum pulse width in microseconds to signal end of a PPM-SUM
  frame. This value is chosen to be smaller than the default 3000 sync
  pulse width for OpenLRSng. Note that this is the total pulse with
  (typically 300us low followed by a long high pulse)
 */

#define AVR_RC_INPUT_MIN_SYNC_PULSE_WIDTH 2700

static uint16_t _pulse_capt[AVR_RC_INPUT_MAX_CHANNELS];

ISR(TIMER5_CAPT_vect)
{
  static uint16_t icr5_prev;
  static uint8_t  channel_ctr;

  const uint16_t icr5_current = ICR5;
  uint16_t pulse_width;
    
  if (icr5_current < icr5_prev) {
    pulse_width =  OCR5A + 1 + icr5_current - icr5_prev;
  } else {
    pulse_width = icr5_current - icr5_prev;
  }

  if (pulse_width > AVR_RC_INPUT_MIN_SYNC_PULSE_WIDTH*2) {
    // sync pulse
	
    if( channel_ctr >= AVR_RC_INPUT_MIN_CHANNELS )
      inputSource(_pulse_capt, channel_ctr);

    channel_ctr = 0;
  } else if (channel_ctr < AVR_RC_INPUT_MAX_CHANNELS)
    _pulse_capt[channel_ctr++] = pulse_width/2;

  icr5_prev = icr5_current;
}

}

extern "C" {
  void stap_initialize(void)
{
  uartPorts[STAP_LINK_HOSTRX] = uartPorts[STAP_LINK_HOSTTX] = hal.console;
  
  consoleNote_P(CS_STRING("Initializing serial comms... "));
  consoleFlush();

  uartPorts[STAP_LINK_TELEMRX] = uartPorts[STAP_LINK_TELEMTX] = hal.uartB;
  uartPorts[STAP_LINK_TELEMRX]->begin(115200, 32, 32);
  uartPorts[STAP_LINK_SRXLINA] = hal.uartC;
#ifdef STAP_LINK_SRXLOUTA
  uartPorts[STAP_LINK_SRXLOUTA] = hal.uartC;
#endif
  uartPorts[STAP_LINK_SRXLINA]->begin(115200, 64, 8);

  consoleNote_P(CS_STRING("I2C... "));
  consoleFlush();

  I2c.begin();
  I2c.setSpeed(true);
  I2c.pullup(true);
  I2c.timeOut(10);

  consolePrint_P(CS_STRING("AHRS... "));
  consoleFlush();

  ins.init(AP_InertialSensor::COLD_START, AP_InertialSensor::RATE_50HZ);
  ahrs.init();
  
  consolePrint_P(CS_STRING("Barometer... "));
  consoleFlush();

  barometer.init();
  barometer.calibrate();
  int i = 0;
  
  consolePrint_P(CS_STRING("PPM RX... "));
  consoleFlush();

  configureInput(&ppmInputPin, true);
  
  TCCR5A = _BV(WGM50) | _BV(WGM51);
  TCCR5B |= _BV(WGM53) | _BV(WGM52) | _BV(CS51) | _BV(ICES5);
  OCR5A  = 40000 - 1; // -1 to correct for wrap

  /* OCR5B and OCR5C will be used by RCOutput_APM2. Init to 0xFFFF to prevent premature PWM output */
  OCR5B  = 0xFFFF;
  OCR5C  = 0xFFFF;

  /* Enable input capture interrupt */
  TIMSK5 |= _BV(ICIE5);
  /*
  TCCR5B |= (1<<ICES5) | (1<<CS51);
  TIMSK5 |= 1<<ICIE5;
  OCR5A  = 40000 - 1; // -1 to correct for wrap
    */

  // Enable interrupts

  nestCount = 0;
  sei();
  
  consolePrint_P(CS_STRING("PWM output... "));
  consoleFlush();

  for(i = 0; i < MAX_SERVO && pwmOutput[i].timer; i++) {
    setPinState(&pwmOutput[i].pin, 0);
    configureOutput(&pwmOutput[i].pin);
    pwmDisable(&pwmOutput[i]);
    pwmOutput[i].active = false;
  }

  pwmTimerInit(hwTimersOwn, sizeof(hwTimersOwn)/sizeof(struct HWTimer*));

  configureInput(&latch, true);
  configureOutput(&led);
  
  consolePrintLn_P(CS_STRING("Done."));
}
}
  


