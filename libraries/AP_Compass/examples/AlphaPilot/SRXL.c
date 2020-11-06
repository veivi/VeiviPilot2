#include <string.h>
#include "CRC16.h"
#include "SRXL.h"
#include "Time.h"
#include "AlphaPilot.h"
#include "RxInput.h"
#include "StaP.h"

typedef enum { srx_idle, srx_data12, srx_data16, srx_crc1, srx_crc2 } state_t;

static state_t state;
static uint8_t chCount, byteCount;
static uint16_t data[RX_CHANNELS_MAX];
static VP_TIME_MILLIS_T lastInput;
static bool seeingUDI12;
  
void srxlInput(uint8_t port)
{
  static uint8_t msb;
  uint8_t len = STAP_LinkStatus(port);

  if(len == 0) {
    if(state != srx_idle && VP_ELAPSED_MILLIS(lastInput, vpTimeMillisApprox) > 5)
      state = srx_idle;
    return;
  }
  
  lastInput = vpTimeMillisApprox;
  
  while(len-- > 0) {
    uint8_t c = STAP_LinkGetChar(port);
    
    switch(state) {
    case srx_idle:
      if(c == 0xA1)
	state = srx_data12;
      else if(c == 0xA2)
	state = srx_data16;
      
      chCount = 0;
      byteCount = 0;
      break;

    case srx_data12:
      seeingUDI12 = true;

    case srx_data16:
      if(byteCount++ < 1)
	msb = c;
      else {
	if(chCount < RX_CHANNELS_MAX)
	  data[chCount] = (msb<<8) | c;
	chCount++;
	byteCount = 0;
      }

      if((state == srx_data12 && chCount == 12) || chCount == 16)
	state = srx_crc1;
      break;

    case srx_crc1:
      msb = c;
      state = srx_crc2;
      break;

    case srx_crc2:
      state = srx_idle;
      inputSource(data, MAX(chCount, RX_CHANNELS));
      break;
    }
  }
}

void srxlOutput(uint8_t port, uint8_t num, const uint16_t pulse[])
{
  const uint8_t frameLen = num > 12 ? 16 : 12;
  uint8_t buffer[RX_CHANNELS_MAX*sizeof(uint16_t)];
  uint16_t crc = 0xFFFF;
  int i = 0;
  
  if(frameLen > 12)
    STAP_LinkPutChar(port, 0xA2);
  else
    STAP_LinkPutChar(port, 0xA1);

  memset((void*) &buffer, '\0', sizeof(buffer));
  
  for(i = 0; i < num; i++) {
    buffer[i*sizeof(uint16_t)] = (uint8_t) (pulse[i]>>8);
    buffer[i*sizeof(uint16_t) + 1] = (uint8_t) (pulse[i] & 0xFF);
  }

  crc = crc16(0xFFFF, buffer, frameLen*sizeof(uint16_t));

  STAP_LinkPut(port, buffer, frameLen*sizeof(uint16_t));
  STAP_LinkPutChar(port, crc >> 8);
  STAP_LinkPutChar(port, crc & 0xFF);  
}
