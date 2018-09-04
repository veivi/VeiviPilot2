#include <string.h>
#include "M24XX.h"
#include "Console.h"
#include "StaP.h"
#include "BaseI2C.h"

#define CACHE_PAGE (1L<<7)
#define PAGE_MASK ~(CACHE_PAGE-1)
#define M24XX_I2C_ADDR 80
#define CACHE_TAG(a) ((a) & PAGE_MASK)

uint32_t m24xxBytesWritten;

static BaseI2CTarget_t target = { "M24xx" };

static uint32_t lastWriteTime;
static uint8_t cacheData[CACHE_PAGE];
static bool cacheFlag[CACHE_PAGE];
static bool cacheValid, cacheModified;
static uint32_t cacheTag;

bool m24xxIsOnline(void)
{
  return basei2cIsOnline(&target);
}

void m24xxWait(uint32_t addr)
{
  if(stap_timeMillis() - lastWriteTime > M24XX_LATENCY)
    // We're cool
    return;
    
  // Write latency not met, wait for acknowledge

  basei2cInvoke(&target, basei2cWait((uint8_t) (M24XX_I2C_ADDR + (uint8_t) ((addr>>16) & 0x7))));
}

void m24xxWriteDirect(uint32_t addr, const uint8_t *data, int bytes) 
{
  if(!basei2cIsOnline(&target))
    return;
    
  m24xxWait(addr);
  basei2cInvoke(&target, basei2cWriteWithWord(  (uint8_t) M24XX_I2C_ADDR + (uint8_t) ((addr>>16) & 0x7), 
				     (uint16_t) (addr & 0xFFFFL), 
				     data, bytes));

  lastWriteTime = stap_timeMillis();
}
 
bool m24xxReadDirect(uint32_t addr, uint8_t *data, int size) 
{
  if(!basei2cIsOnline(&target))
    return false;
    
  m24xxWait(addr);

  return basei2cInvoke(&target, basei2cReadWithWord((uint8_t) M24XX_I2C_ADDR + (uint8_t) ((addr>>16) & 0x7), (uint16_t) (addr & 0xFFFFL), data, size));
}

void m24xxFlush(void)
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

      m24xxWriteDirect(cacheTag + i, &cacheData[i], l);
        
      i += l;
    }
  } while(i < CACHE_PAGE);
  
  cacheModified = false;
}

static bool cacheHit(uint32_t addr)
{
  return CACHE_TAG(addr) == cacheTag;
}

static void cacheAlloc(uint32_t addr)
{
  m24xxFlush();
  cacheTag = CACHE_TAG(addr);
  cacheValid = false;
}

static void m24xxWritePrimitive(uint32_t addr, const uint8_t *value, int size)
{
  int i = 0;
  
  if(!cacheHit(addr))
    cacheAlloc(addr);

  addr &= ~PAGE_MASK;
  
  if(addr+size > CACHE_PAGE) {
    consoleNoteLn_P(PSTR("m24xxWritePrimitive() crosses line border, panic"));
    return;
  }
  
  for(i = 0; i < size; i++) {
    cacheData[addr + i] = value ? value[i] : '\0';
    cacheFlag[addr + i] = true;
  }
  
  cacheValid = false;
  cacheModified = true;
  m24xxBytesWritten += size;
}

void m24xxWrite(uint32_t addr, const uint8_t *value, int size)
{
  if(CACHE_TAG(addr) == CACHE_TAG(addr+size-1)) {
    // Fits one line, fall back to primitive
    m24xxWritePrimitive(addr, value, size);
    return;
  }  
     
  uint32_t ptr = addr;
  
  if(CACHE_TAG(ptr) < ptr) {
    // Starts with a partial line
    
    uint32_t extent = CACHE_TAG(ptr) + CACHE_PAGE - ptr;
    m24xxWritePrimitive(ptr, value, extent);
    if(value)
      value += extent;
    ptr += extent;
  }
  
  while(ptr+CACHE_PAGE <= addr+size) {
    // Full lines remain
    
    m24xxWritePrimitive(ptr, value, CACHE_PAGE);
    if(value)
      value += CACHE_PAGE;
    ptr += CACHE_PAGE;
  }

  if(ptr < addr+size)
    // Partial line remains
    
    m24xxWritePrimitive(ptr, value, addr+size-ptr);
}

int32_t m24xxReadIndirect(uint32_t addr, uint8_t **value, int32_t size)
{
  if(cacheModified || !cacheHit(addr))
    cacheAlloc(addr);
  
  if(!cacheValid) {
    if(!m24xxReadDirect(cacheTag, cacheData, CACHE_PAGE)) {
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

bool m24xxRead(uint32_t addr, uint8_t *value, int32_t size) 
{
  while(size > 0) {
    uint8_t *buffer = NULL;
    int good = m24xxReadIndirect(addr, &buffer, size);

    if(good < 0)
      return false;
    
    memcpy(value, buffer, good);
    value += good;
    addr += good;
    size -= good;
  }

  return true;
}
