#include <stdint.h>
#include "Time.h"
#include "Datagram.h"
#include "CRC16.h"

static uint16_t crcStateTx, crcStateRx[2];
static uint8_t flagRunLength;
static int datagramSize[2];
static uint8_t rxSeq[2], rxSeqLast[2];
uint16_t datagramsGood, datagramsLost, datagramBytes, datagramBytesRaw;
VP_TIME_MILLIS_T datagramLastTxMillis, datagramLastRxMillis;

#define FLAG       0x00
#define START      0x80
#define SEQMASK    0x7F

void datagramHeartbeat(bool force)
{
  if(force || vpTimeMillisApprox - datagramLastTxMillis > 900U) {
    datagramTxStart(DG_HEARTBEAT);
    datagramTxEnd();
  }
}

static void outputBreak()
{
  datagramSerialOut(FLAG);
  datagramSerialOut(FLAG);
}

static void flagRunEnd(void)
{
  if(flagRunLength > 0) {
    datagramSerialOut(FLAG);
    datagramSerialOut(FLAG + flagRunLength);
    flagRunLength = 0;
  }
}

void datagramTxOutByte(const uint8_t c)
{
  crcStateTx = crc16_update(crcStateTx, c);
  
  if(c == FLAG) {
    flagRunLength++;
  } else {
    flagRunEnd();
    datagramSerialOut(c);
  }
}

void datagramTxOut(const uint8_t *data, int l)
{
  if(l > DG_TRANSMIT_MAX)
    l = DG_TRANSMIT_MAX;
  
  while(l-- > 0)
    datagramTxOutByte(*data++);
}

static uint8_t datagramTxSeq;
static bool datagramTxBusy;

void datagramTxStart(uint8_t dg)
{
  if(datagramTxBusy || vpTimeMillisApprox - datagramLastTxMillis > 500U)
    outputBreak();
  
  datagramSerialOut(START + datagramTxSeq);
  crcStateTx = crc16_update(0xFFFF, START + datagramTxSeq);
  datagramTxSeq = (datagramTxSeq + 1) & SEQMASK;
  datagramTxOutByte((const uint8_t) dg);
  datagramTxBusy = true;
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
  flagRunEnd();
  outputBreak();
  datagramLocalOnly = false;
  datagramLastTxMillis = vpTimeMillisApprox;  
  datagramTxBusy = false;
}

static void storeByte(uint8_t port, const uint8_t c)
{
  if(datagramSize[port] < maxDatagramSize)
    datagramRxStore[port*maxDatagramSize + datagramSize[port]++] = c;
}
  
static void handleBreak(uint8_t port)
{
  if(datagramSize[port] >= sizeof(uint16_t)) {
    uint16_t crc =
      *((uint16_t*) &datagramRxStore[port*maxDatagramSize + datagramSize[port]-sizeof(uint16_t)]);
    int payload = datagramSize[port] - sizeof(uint16_t);
    
    if(crc == crc16(crcStateRx[port], &datagramRxStore[port*maxDatagramSize], payload)) {
      datagramsGood++;
      datagramBytes += payload+2;

      uint8_t delta = (rxSeq[port] - rxSeqLast[port] + SEQMASK) & SEQMASK;

      if(delta != 0)
	datagramRxError("LOST_PKT", delta);
	    
      datagramsLost += delta;
      rxSeqLast[port] = rxSeq[port];

      datagramLastRxMillis = vpTimeMillisApprox;  
      datagramInterpreter(port, &datagramRxStore[port*maxDatagramSize], payload);
    } else
      datagramRxError("CRC_FAIL", crc);
  } else
    datagramRxError("SHORT", datagramSize[port]);
    
  datagramSize[port] = 0;
}

void datagramRxInputChar(uint8_t port, uint8_t c)
{
  static bool busy[2];
  static int flagCnt[2];

  if(c != FLAG) {
    if(busy[port]) {
      if(flagCnt[port]) {
	while(c-- != FLAG)
	  storeByte(port, FLAG);
      } else
	storeByte(port, c);

      datagramBytesRaw++;
    } else {
      uint8_t seq = c - START;
      if(seq <= SEQMASK) {
	busy[port] = true;
	crcStateRx[port] = crc16_update(0xFFFF, c);
	rxSeq[port] = seq;
	datagramSize[port] = 0;
      } else
	datagramRxError("BAD_START", c);
    }
    
    flagCnt[port] = 0;
  } else if(++flagCnt[port] > 1 && busy[port]) {
    handleBreak(port);
    busy[port] = false;
  }
}


