#ifndef BUFFER_H
#define BUFFER_H

#include <stdbool.h>
#include <stdint.h>

#define VPBUFFER_SIZE   (1<<6)

typedef struct VPBuffer {
  int8_t inPtr, outPtr;
  char storage[VPBUFFER_SIZE];
} VPBuffer_t;

int8_t vpbuffer_space(VPBuffer_t*);
int8_t vpbuffer_gauge(VPBuffer_t*);
int8_t vpbuffer_insert(VPBuffer_t*, const char *b, int8_t s, bool overwrite);
int8_t vpbuffer_extract(VPBuffer_t*, char *, int8_t);

#endif
