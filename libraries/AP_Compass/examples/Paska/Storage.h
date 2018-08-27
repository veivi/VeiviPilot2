#ifndef STORAGE_H
#define STORAGE_H

#include <stdint.h>
#include <stdbool.h>

#define EXT_EEPROM_LATENCY 6    // ms

extern uint32_t writeBytesCum;

bool eepromIsOnline(void);
void waitEEPROM(uint32_t addr);
void writeEEPROM(uint32_t addr, const uint8_t *data, int bytes);
bool readEEPROM(uint32_t addr, uint8_t *data, int size);
void cacheFlush(void);
void cacheWrite(uint32_t addr, const uint8_t *value, int size);
bool cacheRead(uint32_t addr, uint8_t *value, int32_t size);
int32_t cacheReadIndirect(uint32_t addr, uint8_t **value, int32_t size);

#endif

