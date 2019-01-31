#include <string.h>
#include <stdlib.h>
#include "M24XX.h"
#include "Console.h"
#include "StaP.h"
#include "BaseI2C.h"
#include "Math.h"

#define CACHE_PAGE (1L<<3)
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

bool m24xxWait(uint32_t addr)
{
  if(stap_timeMillis() - lastWriteTime > M24XX_LATENCY)
    // We're cool
    return true;
    
  // Write latency not met, wait for acknowledge

  return basei2cInvoke(&target, stap_I2cWait((uint8_t) (M24XX_I2C_ADDR + (uint8_t) ((addr>>16) & 0x7))));
}

bool m24xxWriteDirect(uint32_t addr, const uint8_t *data, int bytes) 
{
  if(!basei2cIsOnline(&target))
    return false;
    
  if(!m24xxWait(addr))
    return false;
  
  STAP_TRACEON;
  bool status = basei2cInvoke(&target, basei2cWriteWithWord(  (uint8_t) M24XX_I2C_ADDR + (uint8_t) ((addr>>16) & 0x7), 
				     (uint16_t) (addr & 0xFFFFL), 
				     data, bytes));
  STAP_TRACEOFF;

  lastWriteTime = stap_timeMillis();

  return status;
}
 
bool m24xxReadDirect(uint32_t addr, uint8_t *data, int size) 
{
  if(!basei2cIsOnline(&target))
    return false;
    
  if(!m24xxWait(addr))
    return false;
  
  STAP_TRACEON;
  bool status = basei2cInvoke(&target, basei2cReadWithWord((uint8_t) M24XX_I2C_ADDR + (uint8_t) ((addr>>16) & 0x7), (uint16_t) (addr & 0xFFFFL), data, size));
  STAP_TRACEOFF;
  return status;
}

bool m24xxFlush(void)
{
  bool status = true;
  
  if(!cacheModified)
    return true;
    
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

      if(!m24xxWriteDirect(cacheTag + i, &cacheData[i], l))
	status = false;
        
      i += l;
    }
  } while(i < CACHE_PAGE);
  
  cacheModified = false;

  return status;
}

static bool cacheHit(uint32_t addr)
{
  return CACHE_TAG(addr) == cacheTag;
}

static bool cacheAlloc(uint32_t addr)
{
  bool status = m24xxFlush();
  cacheTag = CACHE_TAG(addr);
  cacheValid = false;
  return status;
}

static bool m24xxWritePrimitive(uint32_t addr, const uint8_t *value, int size)
{
  int i = 0;
  
  if(!cacheHit(addr) && !cacheAlloc(addr))
    return false;

  addr &= ~PAGE_MASK;
  
  if(addr+size > CACHE_PAGE) {
    consoleNoteLn_P(CS_STRING("m24xxWritePrimitive() crosses line border, panic"));
    return false;
  }
  
  for(i = 0; i < size; i++) {
    cacheData[addr + i] = value ? value[i] : '\0';
    cacheFlag[addr + i] = true;
  }
  
  cacheValid = false;
  cacheModified = true;
  m24xxBytesWritten += size;

  return true;
}

bool m24xxWrite(uint32_t addr, const uint8_t *value, int size)
{
  if(CACHE_TAG(addr) == CACHE_TAG(addr+size-1))
    // Fits one line, fall back to primitive
    return m24xxWritePrimitive(addr, value, size);
     
  uint32_t ptr = addr;
  bool status = true;
  
  if(CACHE_TAG(ptr) < ptr) {
    // Starts with a partial line
    
    uint32_t extent = CACHE_TAG(ptr) + CACHE_PAGE - ptr;

    if(!m24xxWritePrimitive(ptr, value, extent))
      status = false;
    
    if(value)
      value += extent;
    ptr += extent;
  }
  
  while(ptr+CACHE_PAGE <= addr+size) {
    // Full lines remain

    if(!m24xxWritePrimitive(ptr, value, CACHE_PAGE))
      status = false;
    
    if(value)
      value += CACHE_PAGE;
    ptr += CACHE_PAGE;
  }

  // Partial line remains?
  
  if(ptr < addr+size && !m24xxWritePrimitive(ptr, value, addr+size-ptr))
    status = false;

  return status;
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

#define TEST_SIZE   7

void m24xxTest(void)
{
  static bool success = true;
  static uint16_t state = 0xFFFF;
  uint32_t addr;
  uint8_t buf_w[TEST_SIZE], buf_r[TEST_SIZE];

  if(!success)
    return;
  
  pseudoRandom((uint8_t*) &addr, sizeof(addr), &state);
  pseudoRandom(buf_w, sizeof(buf_w), &state);

  addr = 0x1001 + (addr & 0xFFF0);

  consoleNote_P(CS_STRING("MEMTEST addr "));
  consolePrintUI(addr);

  consolePrint_P(CS_STRING(" data written "));
    
  for(int i = 0; i < TEST_SIZE; i++) {
    consolePrintUI(buf_w[i]);
    consolePrint(" ");
  }

  if(!m24xxWriteDirect(addr, buf_w, TEST_SIZE)) {
    consolePrintLn_P(CS_STRING("ERROR"));
    success = false;
    logDisable();
  }
  
  bzero(buf_r, sizeof(buf_r));
  m24xxReadDirect(addr, buf_r, TEST_SIZE);

  if(memcmp(buf_w, buf_r, TEST_SIZE)) {
    consolePrint_P(CS_STRING("read as "));
    
    for(int i = 0; i < TEST_SIZE; i++) {
      consolePrintUI(buf_r[i]);
      consolePrint(" ");
    }

    consolePrintLn_P(CS_STRING("FAIL"));
    success = false;
    logDisable();
  } else
    consolePrintLn_P(CS_STRING("OK"));
}
