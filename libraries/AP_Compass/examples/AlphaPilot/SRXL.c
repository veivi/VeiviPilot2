#include "SRXL.h"
#include "Time.h"
#include "AlphaPilot.h"
#include "RxInput.h"

typedef enum { srx_idle, srx_data12, srx_data16, srx_crc1, srx_crc2 } state_t;

static state_t state;
static uint8_t chCount, byteCount;
static uint16_t data[RX_CHANNELS];
static VP_TIME_MILLIS_T lastInput;
  
bool srxlInputChar(uint8_t c)
{
  static uint8_t msb;
  bool status = false;

  lastInput = vpTimeMillisApprox;
  
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
  case srx_data16:
    if(byteCount++ < 1)
      msb = c;
    else {
      if(chCount < RX_CHANNELS)
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
    status = true;
    break;
  }

  return status;
}

void srxlHeartbeat()
{
  if(vpTimeMillisApprox - lastInput > 5)
    state = srx_idle;
}
