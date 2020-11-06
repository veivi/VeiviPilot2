#ifndef SRXL_H
#define SRXL_H

#include <stdint.h>

void srxlInput(uint8_t port);
void srxlOutput(uint8_t port, uint8_t num, const uint16_t pulse[]);

#endif
