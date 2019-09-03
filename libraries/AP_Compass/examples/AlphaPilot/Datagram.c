#include <stdint.h>
#include "Time.h"
#include "Datagram.h"
#include "CRC16.h"

static uint16_t crcStateTx, crcStateRx;
static int datagramSize = 0;
static uint8_t rxSeq, rxSeqLast;
uint16_t datagramsGood, datagramsLost, datagramBytes;
VP_TIME_MILLIS_T datagramLastTxMillis, datagramLastRxMillis;

#define FLAG       0xAA
#define NOTFLAG    (FLAG+1)
#define SEQMASK    0x7F

void datagramHeartbeat(bool force)
{
  if(force || vpTimeMillisApprox - datagramLastTxMillis > 0.9e3) {
    datagramTxStart(DG_HEARTBEAT);
    datagramTxEnd();
  }
}

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
  if(vpTimeMillisApprox - datagramLastTxMillis > 0.5e3)
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
  datagramLastTxMillis = vpTimeMillisApprox;  
}

static void storeByte(const uint8_t c)
{
  if(datagramSize < maxDatagramSize)
    datagramRxStore[datagramSize++] = c;
}
  
static void handleBreak(void)
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

      datagramLastRxMillis = vpTimeMillisApprox;  
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
    handleBreak();
    busy = false;
  }
}


