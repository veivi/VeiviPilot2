#include <stdint.h>
#include "StaP.h"
#include "Datagram.h"
#include "CRC16.h"

static uint16_t crcStateTx, crcStateRx;
static int datagramSize = 0;
static uint8_t rxSeq, rxSeqLast;
uint16_t datagramsGood, datagramsLost, datagramBytes;

#define FLAG       0xAB
#define NOTFLAG    ((~FLAG) & 0xFF)

static void outputBreak()
{
  datagramSerialOut(FLAG);
  datagramSerialOut(FLAG);
}

void datagramTxOutByte(const uint8_t c)
{
  datagramSerialOut(c);

  if(c == FLAG)
    datagramSerialOut(NOTFLAG);

  crcStateTx = crc16_update(crcStateTx, c);
}

void datagramTxOut(const uint8_t *data, int l)
{
  if(l > DG_TRANSMIT_MAX)
    l = DG_TRANSMIT_MAX;
  
  while(l-- > 0)
    datagramTxOutByte(*data++);
}

static uint32_t lastTx;
static uint8_t datagramTxSeq;

void datagramTxStart(uint8_t dg)
{
  if(stap_currentMicros - lastTx > 0.1e6)
    outputBreak();
  
  datagramSerialOut(NOTFLAG + datagramTxSeq);
  crcStateTx = crc16_update(0xFFFF, NOTFLAG + datagramTxSeq);
  datagramTxSeq = (datagramTxSeq + 1) & 0x3f;
  datagramTxOutByte((const uint8_t) dg);
}

void datagramTxStartLocal(uint8_t dg)
{
  datagramLocalOnly = true;
  datagramTxStart(dg);
}

void datagramTxEnd(void)
{
  uint16_t buf = crcStateTx;
  datagramTxOut((const uint8_t*) &buf, sizeof(buf));
  outputBreak();
  lastTx = stap_currentMicros;
  datagramLocalOnly = false;
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
        uint16_t crcReceived = *((uint16_t*) &datagramRxStore[datagramSize-2]);
        uint16_t crc = crc16(crcStateRx, datagramRxStore, datagramSize - 2);
        success = (crc == crcReceived);
        
        if(success) {
            datagramsGood++;
	    datagramBytes += datagramSize - 2;
	    datagramInterpreter(datagramRxStore[0], &datagramRxStore[1], datagramSize-3);
	    uint8_t seqInc = (rxSeq - rxSeqLast + 0x40) & 0x3F;
	    if(seqInc > 1) {
	      datagramsLost += seqInc - 1;
	      datagramRxError("LOST PKT", seqInc - 1);
	    }
	    rxSeqLast = rxSeq;
        } else
	  datagramRxError("CRC FAIL", crcReceived);
    } else
      datagramRxError("TOO SHORT", datagramSize);
    
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
    } else if(c >= NOTFLAG && c <= NOTFLAG + 0x3F) {
      busy = true;
      crcStateRx = crc16_update(0xFFFF, c);
      rxSeq = c - NOTFLAG;
      datagramSize = 0;
    } else
      datagramRxError("BAD START", c);
    
    flagCnt = 0;
  } else if(++flagCnt > 1) {
    if(busy)
      success = datagramRxEnd();
        
    busy = false;
  }
  
  return success;
}


