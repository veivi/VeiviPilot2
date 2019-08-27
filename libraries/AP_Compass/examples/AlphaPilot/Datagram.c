#include <stdint.h>
#include "StaP.h"
#include "Datagram.h"
#include "CRC16.h"

static uint16_t crcStateTx, crcStateRx;
static int datagramSize = 0;
static uint8_t rxSeq, rxSeqLast;
uint16_t datagramsGood, datagramsLost, datagramBytes;
uint32_t datagramLastTxMillis, datagramLastRxMillis;

#define FLAG       0xAA
#define NOTFLAG    (FLAG+1)
#define SEQMASK    0x7F

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

static uint8_t datagramTxSeq;

void datagramTxStart(uint8_t dg)
{
  if(stap_currentMillis - datagramLastTxMillis > 0.1e3)
    outputBreak();
  
  datagramSerialOut(NOTFLAG + datagramTxSeq);
  crcStateTx = crc16_update(0xFFFF, NOTFLAG + datagramTxSeq);
  datagramTxSeq = (datagramTxSeq + 1) & SEQMASK;
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
  datagramLocalOnly = false;
  datagramLastTxMillis = stap_currentMillis;  
}

static void storeByte(const uint8_t c)
{
  if(datagramSize < maxDatagramSize)
    datagramRxStore[datagramSize++] = c;
}
  
static void breakDetected(void)
{
  if(datagramSize >= sizeof(uint16_t)) {
    uint16_t crc =
      *((uint16_t*) &datagramRxStore[datagramSize-sizeof(uint16_t)]);
    int payload = datagramSize - sizeof(uint16_t);
    
    if(crc == crc16(crcStateRx, datagramRxStore, payload)) {
      datagramsGood++;
      datagramBytes += payload;

      uint8_t delta = (rxSeq - rxSeqLast + SEQMASK) & SEQMASK;

      if(delta != 0)
	datagramRxError("LOST_PKT", delta);
	    
      datagramsLost += delta;
      rxSeqLast = rxSeq;

      datagramLastRxMillis = stap_currentMillis;  
      datagramInterpreter(datagramRxStore, payload);
    } else
      datagramRxError("CRC_FAIL", crc);
  } else
    datagramRxError("SHORT", datagramSize);
    
  datagramSize = 0;
}

void datagramRxInputChar(const uint8_t c)
{
  static bool busy = false;
  static int flagCnt;

  if(c != FLAG) {
    if(busy) {
      if(flagCnt)
	storeByte(FLAG);
      else
	storeByte(c);
    } else {
      uint8_t seq = c - NOTFLAG;
      if(seq <= SEQMASK) {
	busy = true;
	crcStateRx = crc16_update(0xFFFF, c);
	rxSeq = seq;
	datagramSize = 0;
      } else
	datagramRxError("BAD_START", c);
    }
    
    flagCnt = 0;
  } else if(++flagCnt > 1 && busy) {
    breakDetected();
    busy = false;
  }
}


