#include <stdint.h>
#include "StaP.h"
#include "Datagram.h"
#include "CRC16.h"

static uint16_t crcState;
static int datagramSize = 0;
static int datagrams, datagramsGood;

const uint8_t FLAG = 0xAB;

static void outputBreak()
{
  datagramSerialOut(FLAG);
  datagramSerialOut(FLAG);
}

void datagramTxOutByte(const uint8_t c)
{
  datagramSerialOut(c);

  if(c == FLAG)
    datagramSerialOut(~FLAG);

  crcState = crc16_update(crcState, c);
}

void datagramTxOut(const uint8_t *data, int l)
{
  while(l-- > 0)
    datagramTxOutByte(*data++);
}

static uint32_t lastTx;

void datagramTxStart(uint8_t dg)
{
  if(stap_currentMicros - lastTx > 0.1e6)
    outputBreak();
  
  crcState = 0xFFFF;
  datagramTxOutByte((const uint8_t) dg);
}

void datagramTxEnd(void)
{
  uint16_t buf = crcState;
  datagramTxOut((const uint8_t*) &buf, sizeof(buf));
  outputBreak();
  datagramSerialFlush();
  lastTx = stap_currentMicros;
}

static void storeByte(const uint8_t c)
{
    if(datagramSize < maxDatagramSize)
        datagramRxStore[datagramSize++] = c;
}
  
static bool datagramRxEnd(void)
{
    bool success = false;
    
    if(datagramSize > 2) {
        datagrams++;
        uint16_t crcReceived = *((uint16_t*) &datagramRxStore[datagramSize-2]);
        uint16_t crc = crc16(0xFFFF, datagramRxStore, datagramSize - 2);
        success = (crc == crcReceived);
        
        if(success) {
            datagramsGood++;
	    datagramInterpreter(datagramRxStore[0], &datagramRxStore[1], datagramSize-3);
        } else
	  datagramRxError("CRC FAIL");
    } else
      datagramRxError("TOO SHORT");
    
    datagramSize = 0;
    return success;
}

bool datagramRxInputChar(const uint8_t c)
{
  static bool busy = false;
  static int flagCnt;
  bool success = false;

  if(c != FLAG) {
    if(busy) {
      if(flagCnt)
	storeByte(FLAG);
      else
	storeByte(c);
    } else {
      busy = true;
      datagramSize = 0;
      storeByte(c);
    }
    
    flagCnt = 0;
  } else if(++flagCnt > 1) {
    if(busy)
      success = datagramRxEnd();
        
    busy = false;
  }
  
  return success;
}


