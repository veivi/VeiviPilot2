#ifndef M24XX_H
#define M24XX_H

#include <stdint.h>
#include <stdbool.h>

#define M24XX_LATENCY 10    // ms
#define I2C_MEM_HARD_WAIT  

extern uint32_t m24xxBytesWritten;

bool m24xxIsOnline(void);
bool m24xxWait(uint32_t addr);
bool m24xxWriteDirect(uint32_t addr, const uint8_t *data, int bytes);
bool m24xxReadDirect(uint32_t addr, uint8_t *data, int size);
bool m24xxWrite(uint32_t addr, const uint8_t *value, int size);
bool m24xxRead(uint32_t addr, uint8_t *value, int32_t size);
int32_t m24xxReadIndirect(uint32_t addr, uint8_t **value, int32_t size);
bool m24xxFlush(void);

#endif

