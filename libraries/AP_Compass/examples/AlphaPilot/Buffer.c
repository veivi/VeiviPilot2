#include "Buffer.h"
#include <string.h>

#define VPBUFFER_INDEX(p, i) (((p)+(i)+VPBUFFER_SIZE) & (VPBUFFER_SIZE-1))

static void safe_memcpy(char *dst, const char *src, int8_t s)
{
  if(s > 0)
    memcpy((void*) dst, (const void*) src, (size_t) s);
}

int8_t vpbuffer_gauge(VPBuffer_t *i)
{
  return VPBUFFER_INDEX(i->inPtr, -i->outPtr);
}

int8_t vpbuffer_space(VPBuffer_t *i)
{
  return VPBUFFER_SIZE - 1 - vpbuffer_gauge(i);
}

void vpbuffer_adjust(VPBuffer_t *i, int8_t delta)
{
  int8_t g = vpbuffer_gauge(i);
  
  if(delta > g)
    delta = g;
  
  i->inPtr = VPBUFFER_INDEX(i->inPtr, -delta);
}

int8_t vpbuffer_insert(VPBuffer_t *i, const char *b, int8_t s, bool overwrite)
{
  int8_t space = vpbuffer_space(i);

  if(s > VPBUFFER_SIZE - 1)
    s = VPBUFFER_SIZE - 1;
  
  if(s > space) {
    if(overwrite) {
      // Adjust the inPtr for overwriting

      vpbuffer_adjust(i, s - space + 3);
      vpbuffer_insert(i, " ~ ", 3, false);
      vpbuffer_insert(i, b, s, false);
      
      return s;
    } else
      // Truncate
      s = space;
  }

  if(VPBUFFER_INDEX(i->inPtr, s) < i->inPtr) {
    int8_t cut = VPBUFFER_SIZE - i->inPtr;
    safe_memcpy(&i->storage[i->inPtr], b, cut);
    safe_memcpy(i->storage, &b[cut], s - cut);
  } else {
    safe_memcpy(&i->storage[i->inPtr], b, s);
  }
  
  i->inPtr = VPBUFFER_INDEX(i->inPtr, s);
  
  return s;
}

int8_t vpbuffer_extract(VPBuffer_t *i, char *b, int8_t s)
{
  int8_t g = vpbuffer_gauge(i);
  
  if(s > g)
    s = g;

  if(VPBUFFER_INDEX(i->outPtr, s) < i->outPtr) {
    int8_t cut = VPBUFFER_SIZE - i->outPtr;
    safe_memcpy(b, &i->storage[i->outPtr], cut);
    safe_memcpy(&b[cut], i->storage, s - cut);
  } else {
    safe_memcpy(b, &i->storage[i->outPtr], s);

  }
  
  i->outPtr = VPBUFFER_INDEX(i->outPtr, s);

  return s;
}

