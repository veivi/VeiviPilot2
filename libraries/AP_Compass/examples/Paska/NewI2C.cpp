#include "NewI2C.h"
#include "Console.h"
#include <avr/io.h>
#include <AP_HAL/AP_HAL.h>

#define BACKOFF (1.0e6)

extern const AP_HAL::HAL& hal;

#define START           0x08
#define REPEATED_START  0x10
#define MT_SLA_ACK	0x18
#define MT_SLA_NACK	0x20
#define MT_DATA_ACK     0x28
#define MT_DATA_NACK    0x30
#define MR_SLA_ACK	0x40
#define MR_SLA_NACK	0x48
#define MR_DATA_ACK     0x50
#define MR_DATA_NACK    0x58
#define LOST_ARBTRTN    0x38
#define TWI_STATUS      (TWSR & 0xF8)
#define SLA_W(address)  (address << 1)
#define SLA_R(address)  ((address << 1) + 0x01)
#define cbi(sfr, bit)   (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit)   (_SFR_BYTE(sfr) |= _BV(bit))

uint16_t NewI2C::timeOutDelay = 0;

static uint32_t millis()
{
  return hal.scheduler->millis();
}

I2CDevice::I2CDevice(NewI2C *interface, uint8_t addr, const char *dname)
{
  name = dname;
  backoff = BACKOFF;
}

bool I2CDevice::hasFailed()
{
  return failed && currentTime < failedAt+backoff;
}

bool I2CDevice::status()
{
  return warn || failed;
}

bool I2CDevice::handleStatus(bool fail)
{
  if(fail) {
    warn = true;
    
    consoleNote_P(PSTR("Bad "));
    consolePrintLn(name);

    if(failed)
      backoff += backoff/2;
    else if(++failCount > 3) {
      consoleNote("");
      consolePrint(name);
      consolePrintLn_P(PSTR(" failed"));
      failed = true;
    }
    
    failedAt = currentTime;
  } else {    
    if(failCount > 0) {
      consoleNote("");
      consolePrint(name);
      consolePrintLn_P(PSTR(" recovered"));
      failCount = 0;
      failed = warn = false;
      backoff = BACKOFF;
    }
  }
  
  return fail;
}

////////////// Public Methods ////////////////////////////////////////

void NewI2C::begin()
{
  // initialize twi prescaler and bit rate
  cbi(TWSR, TWPS0);
  cbi(TWSR, TWPS1);
  TWBR = ((F_CPU / 100000) - 16) / 2;
  // enable twi module and acks
  TWCR = _BV(TWEN) | _BV(TWEA); 
}

void NewI2C::end()
{
  TWCR = 0;
}

void NewI2C::timeOut(uint16_t _timeOut)
{
  timeOutDelay = _timeOut;
}

void NewI2C::setSpeed(uint8_t _fast)
{
  if(!_fast)
  {
    TWBR = ((F_CPU / 100000) - 16) / 2;
  }
  else
  {
    TWBR = ((F_CPU / 400000) - 16) / 2;
  }
}
  
void NewI2C::pullup(uint8_t activate)
{
  if(activate)
  {
    #if defined(__AVR_ATmega168__) || defined(__AVR_ATmega8__) || defined(__AVR_ATmega328P__)
      // activate internal pull-ups for twi
      // as per note from atmega8 manual pg167
      sbi(PORTC, 4);
      sbi(PORTC, 5);
    #else
      // activate internal pull-ups for twi
      // as per note from atmega128 manual pg204
      sbi(PORTD, 0);
      sbi(PORTD, 1);
    #endif
  }
  else
  {
    #if defined(__AVR_ATmega168__) || defined(__AVR_ATmega8__) || defined(__AVR_ATmega328P__)
      // deactivate internal pull-ups for twi
      // as per note from atmega8 manual pg167
      cbi(PORTC, 4);
      cbi(PORTC, 5);
    #else
      // deactivate internal pull-ups for twi
      // as per note from atmega128 manual pg204
      cbi(PORTD, 0);
      cbi(PORTD, 1);
    #endif
  }
}
  
/*return values for new functions that use the timeOut feature 
  will now return at what point in the transmission the timeout
  occurred. Looking at a full communication sequence between a 
  master and slave (transmit data and then readback data) there
  a total of 7 points in the sequence where a timeout can occur.
  These are listed below and correspond to the returned value:
  1 - Waiting for successful completion of a Start bit
  2 - Waiting for ACK/NACK while addressing slave in transmit mode (MT)
  3 - Waiting for ACK/NACK while sending data to the slave
  4 - Waiting for successful completion of a Repeated Start
  5 - Waiting for ACK/NACK while addressing slave in receiver mode (MR)
  6 - Waiting for ACK/NACK while receiving data from the slave
  7 - Waiting for successful completion of the Stop bit

  All possible return values:
  0           Function executed with no errors
  1 - 7       Timeout occurred, see above list
  8 - 0xFF    See datasheet for exact meaning */ 


/////////////////////////////////////////////////////

uint8_t NewI2C::wait(uint8_t address)
{
  unsigned long startingTime = millis();
  
  while(1) {
    returnStatus = 0;
    returnStatus = start();
    if(returnStatus){return(returnStatus);}
    returnStatus = transmitByte(SLA_W(address));

    if(!returnStatus) {
      returnStatus = stop();
      if(returnStatus == 1)
        return 7;
      return(returnStatus);
    } else if(returnStatus != MT_SLA_NACK) {
        if(returnStatus == 1){return(2);}
        return(returnStatus);
    }

    if(timeOutDelay > 0 && millis() - startingTime > timeOutDelay)
    {
      lockUp();
      return(1);
    }
  }
}

uint8_t NewI2C::write(uint8_t address, const uint8_t *data, uint8_t numberBytes)
{
  return write(address, NULL, 0, data, numberBytes);
}

uint8_t NewI2C::write(uint8_t address, uint8_t registerAddress, const uint8_t *data, uint8_t numberBytes)
{
  return write(address, &registerAddress, sizeof(registerAddress), data, numberBytes);
}

uint8_t NewI2C::write(uint8_t address, uint16_t memAddress, const uint8_t *data, uint8_t numberBytes)
{
  uint8_t addrArray[sizeof(memAddress)];
  
  for(uint8_t i = 0; i < sizeof(memAddress); i++)
    addrArray[i] = (memAddress >> 8*(sizeof(memAddress) - i - 1)) & 0xFF;
    
  return write(address, addrArray, sizeof(addrArray), data, numberBytes);
}

uint8_t NewI2C::write(uint8_t address, const uint8_t *addrArray, uint8_t addrSize, const uint8_t *data, uint8_t numberBytes)
{
  returnStatus = start();
  if(returnStatus){return(returnStatus);}

  returnStatus = transmitByte(SLA_W(address));
  if(returnStatus)
  {
    if(returnStatus == 1){return(2);}
    return(returnStatus);
  }

  for(uint8_t i = 0; i < addrSize; i++) {
    returnStatus = transmitByte(addrArray[i]);
    if(returnStatus)
    {
      if(returnStatus == 1){return(3);}
      return(returnStatus);
    }
  }

  for (uint8_t i = 0; i < numberBytes; i++)
  {
    returnStatus = transmitByte(data[i]);
    if(returnStatus)
      {
        if(returnStatus == 1){return(3);}
        return(returnStatus);
      }
  }

  returnStatus = stop();

  if(returnStatus == 1)
    return 7;
    
  return returnStatus;
}

uint8_t NewI2C::read(uint8_t address, uint8_t *dataBuffer, uint8_t numberBytes)
{
  return read(address, NULL, 0, dataBuffer, numberBytes);
}

uint8_t NewI2C::read(uint8_t address, uint8_t registerAddress, uint8_t *dataBuffer, uint8_t numberBytes)
{
  return read(address, &registerAddress, sizeof(registerAddress), dataBuffer, numberBytes);
}

uint8_t NewI2C::read(uint8_t address, uint16_t memAddress, uint8_t *dataBuffer, uint8_t numberBytes)
{
  uint8_t addrArray[sizeof(memAddress)];
  
  for(uint8_t i = 0; i < sizeof(memAddress); i++)
    addrArray[i] = (memAddress >> 8*(sizeof(memAddress) - i - 1)) & 0xFF;
    
  return read(address, addrArray, sizeof(addrArray), dataBuffer, numberBytes);
}

uint8_t NewI2C::read(uint8_t address, const uint8_t *addrArray, uint8_t addrSize, uint8_t *dataBuffer, uint8_t numberBytes)
{
  if(numberBytes < 1)
    return 0;

  if(addrSize > 0) {
    returnStatus = start();
    if(returnStatus){return(returnStatus);}

    returnStatus = transmitByte(SLA_W(address));
    if(returnStatus)
    {
      if(returnStatus == 1){return(2);}
      return(returnStatus);
    }

    for(uint8_t i = 0; i < addrSize; i++) {
      returnStatus = transmitByte(addrArray[i]);
      if(returnStatus)
      {
        if(returnStatus == 1){return(3);}
        return(returnStatus);
      }
    }
  }
  
  returnStatus = start();
  if(returnStatus)
  {
    if(returnStatus == 1){return(4);}
    return(returnStatus);
  }
  
  returnStatus = transmitByte(SLA_R(address));
  if(returnStatus)
  {
    if(returnStatus == 1){return(5);}
    return(returnStatus);
  }
  
  for(uint8_t i = 0; i < numberBytes; i++)
  {
    returnStatus = receiveByte(i < (numberBytes - 1));
    
    if(returnStatus == 1)
      return 6;
      
    if(returnStatus != (i < (numberBytes - 1) ? MR_DATA_ACK : MR_DATA_NACK))
      return returnStatus;

    dataBuffer[i] = TWDR;
  }

  returnStatus = stop();
  
  if(returnStatus == 1)
    return 7;

  return returnStatus;
}

/////////////// Private Methods ////////////////////////////////////////


uint8_t NewI2C::start()
{
  unsigned long startingTime = millis();
  TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
  while (!(TWCR & (1<<TWINT)))
  {
    if(timeOutDelay > 0 && millis() - startingTime > timeOutDelay)
    {
      lockUp();
      return(1);
    }
       
  }
  if ((TWI_STATUS == START) || (TWI_STATUS == REPEATED_START))
  {
    return(0);
  }
  uint8_t bufferedStatus = TWI_STATUS;
  if (bufferedStatus == LOST_ARBTRTN)
  {
    lockUp();
    return(bufferedStatus);
  }
  return(TWI_STATUS);
}

uint8_t NewI2C::transmitByte(uint8_t contents)
{
  TWDR = contents;
  unsigned long startingTime = millis();
  TWCR = (1<<TWINT) | (1<<TWEN);
  while (!(TWCR & (1<<TWINT)))
  {
    if(timeOutDelay > 0 && millis() - startingTime > timeOutDelay)
    {
      lockUp();
      return(1);
    }
       
  }
  if ((TWI_STATUS == MT_SLA_ACK) || (TWI_STATUS == MR_SLA_ACK) || (TWI_STATUS == MT_DATA_ACK))
  {
    return(0);
  }
  uint8_t bufferedStatus = TWI_STATUS;
  if ((bufferedStatus == MT_SLA_NACK) || (bufferedStatus == MR_SLA_NACK) || (bufferedStatus == MT_DATA_NACK))
  {
      stop();
      return(bufferedStatus);
  }
  else
  {
    lockUp();
    return(bufferedStatus);
  }
}

uint8_t NewI2C::receiveByte(bool ack)
{
  unsigned long startingTime = millis();
  if(ack)
  {
    TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);

  }
  else
  {
    TWCR = (1<<TWINT) | (1<<TWEN);
  }
  while (!(TWCR & (1<<TWINT)))
  {
    if(timeOutDelay > 0 && millis() - startingTime > timeOutDelay)
    {
      lockUp();
      return(1);
    }
  }
  uint8_t bufferedStatus = TWI_STATUS;
  if (bufferedStatus == LOST_ARBTRTN)
  {
    lockUp();
    return(bufferedStatus);
  }
  return(TWI_STATUS); 
}

uint8_t NewI2C::stop()
{
  unsigned long startingTime = millis();
  TWCR = (1<<TWINT)|(1<<TWEN)| (1<<TWSTO);
  while ((TWCR & (1<<TWSTO)))
  {
    if(timeOutDelay > 0 && millis() - startingTime > timeOutDelay)
    {
      lockUp();
      return(1);
    }
       
  }
  return(0);
}

void NewI2C::lockUp()
{
  TWCR = 0; //releases SDA and SCL lines to high impedance
  TWCR = _BV(TWEN) | _BV(TWEA); //reinitialize TWI 
}

