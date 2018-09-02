#include <string.h>
#include "Storage.h"
#include "Console.h"
#include "StaP.h"
#include "BaseI2C.h"

#define CACHE_PAGE (1L<<7)
#define PAGE_MASK ~(CACHE_PAGE-1)
#define EEPROM_I2C_ADDR 80

static BaseI2CTarget_t target = { "eeprom" };

static uint32_t lastWriteTime;
uint8_t cacheData[CACHE_PAGE];
bool cacheFlag[CACHE_PAGE];
bool cacheValid, cacheModified;
uint32_t cacheTag;
uint32_t writeBytesCum;

bool eepromIsOnline(void)
{
  return basei2cIsOnline(&target);
}

void waitEEPROM(uint32_t addr)
{
  if(stap_timeMillis() - lastWriteTime > EXT_EEPROM_LATENCY)
    // We're cool
    return;
    
  // Write latency not met, wait for acknowledge

  basei2cInvoke(&target, basei2cWait((uint8_t) (EEPROM_I2C_ADDR + (uint8_t) ((addr>>16) & 0x7))));
}

void writeEEPROM(uint32_t addr, const uint8_t *data, int bytes) 
{
  if(!basei2cIsOnline(&target))
    return;
    
  waitEEPROM(addr);
  basei2cInvoke(&target, basei2cWriteWithWord(  (uint8_t) EEPROM_I2C_ADDR + (uint8_t) ((addr>>16) & 0x7), 
				     (uint16_t) (addr & 0xFFFFL), 
				     data, bytes));

  lastWriteTime = stap_timeMillis();
}
 
bool readEEPROM(uint32_t addr, uint8_t *data, int size) 
{
  if(!basei2cIsOnline(&target))
    return false;
    
  waitEEPROM(addr);

  return basei2cInvoke(&target, basei2cReadWithWord((uint8_t) EEPROM_I2C_ADDR + (uint8_t) ((addr>>16) & 0x7), (uint16_t) (addr & 0xFFFFL), data, size));
}

void cacheFlush(void)
{
  if(!cacheModified)
    return;
    
  int i = 0;
  
  do {
    while(!cacheFlag[i] && i < CACHE_PAGE)
      i++;
    
    if(i < CACHE_PAGE) {
      int l = 0;
      
      while(cacheFlag[i+l] && i+l < CACHE_PAGE) {
        cacheFlag[i+l] = false;
        l++;
      }

      writeEEPROM(cacheTag + i, &cacheData[i], l);
        
      i += l;
    }
  } while(i < CACHE_PAGE);
  
  cacheModified = false;
}

#define CACHE_TAG(a) ((a) & PAGE_MASK)

static bool cacheHit(uint32_t addr)
{
  return CACHE_TAG(addr) == cacheTag;
}

static void cacheAlloc(uint32_t addr)
{
  cacheFlush();
  cacheTag = CACHE_TAG(addr);
  cacheValid = false;
}

static void cacheWritePrimitive(uint32_t addr, const uint8_t *value, int size)
{
  int i = 0;
  
  if(!cacheHit(addr))
    cacheAlloc(addr);

  addr &= ~PAGE_MASK;
  
  if(addr+size > CACHE_PAGE) {
    consoleNoteLn_P(PSTR("cacheWritePrimitive() crosses line border, panic"));
    return;
  }
  
  for(i = 0; i < size; i++) {
    cacheData[addr + i] = value ? value[i] : '\0';
    cacheFlag[addr + i] = true;
  }
  
  cacheValid = false;
  cacheModified = true;
  writeBytesCum += size;
}

void cacheWrite(uint32_t addr, const uint8_t *value, int size)
{
  if(CACHE_TAG(addr) == CACHE_TAG(addr+size-1)) {
    // Fits one line, fall back to primitive
    cacheWritePrimitive(addr, value, size);
    return;
  }  
     
  uint32_t ptr = addr;
  
  if(CACHE_TAG(ptr) < ptr) {
    // Starts with a partial line
    
    uint32_t extent = CACHE_TAG(ptr) + CACHE_PAGE - ptr;
    cacheWritePrimitive(ptr, value, extent);
    if(value)
      value += extent;
    ptr += extent;
  }
  
  while(ptr+CACHE_PAGE <= addr+size) {
    // Full lines remain
    
    cacheWritePrimitive(ptr, value, CACHE_PAGE);
    if(value)
      value += CACHE_PAGE;
    ptr += CACHE_PAGE;
  }

  if(ptr < addr+size)
    // Partial line remains
    
    cacheWritePrimitive(ptr, value, addr+size-ptr);
}

int32_t cacheReadIndirect(uint32_t addr, uint8_t **value, int32_t size)
{
  if(cacheModified || !cacheHit(addr))
    cacheAlloc(addr);
  
  if(!cacheValid) {
    if(!readEEPROM(cacheTag, cacheData, CACHE_PAGE)) {
      memset(cacheData, 0xFF, sizeof(cacheData));
      return -1;
    }
    
    cacheValid = true;
  }

  *value = &cacheData[addr-cacheTag];

  int32_t good = CACHE_PAGE-(addr-cacheTag);
  
  if(good < size)
    size = good;
  
  return size;
}

bool cacheRead(uint32_t addr, uint8_t *value, int32_t size) 
{
  while(size > 0) {
    uint8_t *buffer = NULL;
    int good = cacheReadIndirect(addr, &buffer, size);

    if(good < 0)
      return false;
    
    memcpy(value, buffer, good);
    value += good;
    addr += good;
    size -= good;
  }

  return true;
}
