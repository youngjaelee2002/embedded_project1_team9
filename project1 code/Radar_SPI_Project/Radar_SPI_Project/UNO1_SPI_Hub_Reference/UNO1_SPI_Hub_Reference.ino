/*
  UNO1 hub-side SPI slave reference (ATmega328P / 16 MHz).
  This is a protocol interoperability example, NOT the completed 4-board hub.
  Wiring: UNO3 D9 -> UNO1 D10 SS; D11->D11, D12<-D12, D13->D13, GND-GND.
  UNO1 UART D0/D1 connects to UNO2 (115200, common GND).
  UNO2 UART input: CMD|0..4|0..255 followed by \n, or RETRY\n.
  UNO1 UART output: RAD|... / OBJ|... / ERR|... . See SPI_PROTOCOL.md.
  This example intentionally does not initialize the UNO4 I2C subsystem.
  Set up UNO4 on A4/A5 without blocking the SPI service when extending it.
*/
#include <Arduino.h>
#include "RadarProtocol.h"
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

#if !defined(__AVR_ATmega328P__) || F_CPU != 16000000UL
#error Select Arduino Uno ATmega328P at 16 MHz.
#endif

// Two 48-byte TX buffers: main loop builds inactive buffer, single-byte ISR-safe swap.
volatile uint8_t txBuffers[2][RadarWire::SIZE] = {};
volatile uint8_t rxBuilding[RadarWire::SIZE] = {};
volatile uint8_t rxReady[RadarWire::SIZE] = {};
volatile uint8_t activeTx = 0, spiIndex = 0;
volatile bool selected = false, rxPending = false;
volatile uint16_t incompleteFrames = 0, droppedFrames = 0;

uint8_t rxCopy[RadarWire::SIZE] = {};
uint8_t commandSeq = 0, desiredMode = 0, desiredTarget = 0, desiredFlags = 0;
uint8_t activeTelemetrySeq = 0xFF;
uint16_t telemetryOK=0, telemetryCRCbad=0;
uint32_t lastPublishMs=0;
char uartInput[36];
uint8_t uartUsed=0;

ISR(PCINT0_vect) { // Pin change on PB2 / hardware SS (D10)
  const bool isLow = !(PINB & _BV(PB2));
  if (isLow && !selected) {
    selected = true;
    spiIndex = 0;
    SPDR = txBuffers[activeTx][0]; // first outgoing byte before first SCK edge
  } else if (!isLow && selected) {
    selected = false;
    if (spiIndex == RadarWire::SIZE && !rxPending) {
      for (uint8_t i=0; i<RadarWire::SIZE; ++i) rxReady[i] = rxBuilding[i];
      rxPending = true;
    } else if (spiIndex != RadarWire::SIZE) {
      if (incompleteFrames != 65535) ++incompleteFrames;
    } else {
      if (droppedFrames != 65535) ++droppedFrames;
    }
  }
}
ISR(SPI_STC_vect) {
  const uint8_t incoming = SPDR;
  uint8_t i = spiIndex;
  if (!selected || i >= RadarWire::SIZE) return;
  rxBuilding[i] = incoming;
  ++i;
  spiIndex = i;
  if (i < RadarWire::SIZE) SPDR = txBuffers[activeTx][i];
}

void stageCommand() {
  const uint8_t inactive = 1 - activeTx;
  for (uint8_t i=0; i<RadarWire::SIZE; ++i) txBuffers[inactive][i] = 0;
  txBuffers[inactive][0] = RadarWire::SYNC;
  txBuffers[inactive][1] = RadarWire::VERSION;
  txBuffers[inactive][2] = commandSeq;
  txBuffers[inactive][3] = desiredMode;
  txBuffers[inactive][4] = desiredFlags;
  txBuffers[inactive][5] = desiredTarget;
  // CRC calculated on prepared buffer, never partially published to ISR.
  uint8_t crc=0;
  for (uint8_t i=0; i<RadarWire::SIZE-1; ++i) {
    crc ^= txBuffers[inactive][i];
    for (uint8_t j=0; j<8; ++j)
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
  }
  txBuffers[inactive][47] = crc;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    if (!selected && (PINB & _BV(PB2))) activeTx = inactive;
  }
}

// Convert both coordinate layouts to a common display form for the UNO2 UART.
void emitObject(const uint8_t *p, bool cyl) {
  const uint8_t id = p[0], flags=p[1], q=p[10], age=p[11];
  float x, y, r, theta, vx, vy, vr, vt;
  if (!cyl) {
    x=(float)RadarWire::getI16(p,2); y=(float)RadarWire::getI16(p,4);
    vx=(float)RadarWire::getI16(p,6); vy=(float)RadarWire::getI16(p,8);
    const float dx=x-500.0f, dy=y-500.0f;
    r=sqrtf(dx*dx+dy*dy);
    theta=atan2f(dy,dx);
    if (theta < 0) theta += 2*PI;
    const float inv=r>0.01f ? 1.0f/r : 0;
    vr=(dx*vx+dy*vy)*inv;
    vt=(-dy*vx+dx*vy)*inv;
  } else {
    r=(float)RadarWire::getU16(p,2);
    theta=(float)RadarWire::getU16(p,4) * (PI / 18000.0f);
    vr=(float)RadarWire::getI16(p,6); vt=(float)RadarWire::getI16(p,8);
    const float c=cosf(theta), s=sinf(theta);
    x=500.0f+r*c; y=500.0f+r*s;
    vx=vr*c-vt*s; vy=vr*s+vt*c;
  }
  Serial.print(F("OBJ|")); Serial.print(id);
  Serial.print('|'); Serial.print((int16_t)lroundf(x));
  Serial.print('|'); Serial.print((int16_t)lroundf(y));
  Serial.print('|'); Serial.print((uint16_t)lroundf(r));
  Serial.print('|'); Serial.print((uint16_t)lroundf(theta * 18000.0f / PI) % 36000U);
  Serial.print('|'); Serial.print((int16_t)lroundf(vx));
  Serial.print('|'); Serial.print((int16_t)lroundf(vy));
  Serial.print('|'); Serial.print((int16_t)lroundf(vr));
  Serial.print('|'); Serial.print((int16_t)lroundf(vt));
  Serial.print('|'); Serial.print(q);
  Serial.print('|'); Serial.print(age);
  Serial.print('|'); Serial.println(flags);
}

void consumeTelemetry() {
  if (!RadarWire::valid(rxCopy)) {
    if (telemetryCRCbad < 65535) ++telemetryCRCbad;
    return;
  }
  if (telemetryOK < 65535) ++telemetryOK;
  const uint8_t seq=rxCopy[2];
  activeTelemetrySeq = seq;
  const uint8_t count=rxCopy[5];
  if (count > RadarWire::SLOT_COUNT) return;
  const bool cyl = (rxCopy[4] & RadarWire::ST_CYLINDRICAL) != 0;
  // Every valid packet describes a fresh snapshot. Missing prior IDs are gone.
  Serial.print(F("RAD|")); Serial.print(seq);
  Serial.print('|'); Serial.print(rxCopy[3]);  // mode
  Serial.print('|'); Serial.print(rxCopy[42]); // state
  Serial.print('|'); Serial.print(rxCopy[4]);  // status bits
  Serial.print('|'); Serial.print(count);
  Serial.print('|'); Serial.print(rxCopy[43]); // Hall strikes
  Serial.print('|'); Serial.print(rxCopy[44]); // selected target
  Serial.print('|'); Serial.print(rxCopy[45]); // command ACK seq
  Serial.print('|'); Serial.print(rxCopy[46]); // error
  Serial.print('|'); Serial.println(cyl ? 1 : 0);
  for (uint8_t i=0; i<count; ++i)
    emitObject(&rxCopy[6+i*RadarWire::SLOT_SIZE],cyl);
}

void parseCommand(char *line) {
  if (!strcmp(line,"RETRY")) {
    desiredFlags = RadarWire::CMD_FLAG_TOF_RETRY;
    ++commandSeq;
    stageCommand();
    return;
  }
  if (strncmp(line,"CMD|",4)) return;
  char *q=line+4, *end=nullptr;
  long mode=strtol(q,&end,10);
  if (end==q || *end != '|') return;
  q=end+1;
  long target=strtol(q,&end,10);
  if (q==end || *end || mode<0 || mode>4 || target<0 || target>255) return;
  desiredMode=(uint8_t)mode;
  desiredTarget=(uint8_t)target;
  desiredFlags=0;
  ++commandSeq; // each UART command is a new SPI command even if mode is same
  stageCommand();
}

void serviceUART() {
  while (Serial.available()) {
    const char c=(char)Serial.read();
    if (c=='\n' || c=='\r') {
      if (uartUsed) { uartInput[uartUsed]=0; parseCommand(uartInput); uartUsed=0; }
    } else if (uartUsed < sizeof(uartInput)-1) uartInput[uartUsed++]=c;
    else uartUsed=0;
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(10,INPUT_PULLUP);
  pinMode(11,INPUT); pinMode(12,OUTPUT); pinMode(13,INPUT);
  SPCR = _BV(SPE) | _BV(SPIE); // SPI SLAVE, MSB first, mode 0
  PCICR |= _BV(PCIE0);
  PCMSK0 |= _BV(PCINT2);      // D10 / PB2 SS only
  stageCommand();            // initial STOP seq 0
  Serial.println(F("HELLO|UNO1|SPI_SLAVE_REFERENCE|1"));
}

void loop() {
  bool ready=false;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    if (rxPending) {
      for (uint8_t i=0; i<RadarWire::SIZE; ++i) rxCopy[i] = rxReady[i];
      rxPending=false;
      ready=true;
    }
  }
  if (ready) consumeTelemetry();
  serviceUART();
  // Periodically stage the current command, recovering from missed staging
  // when CS overlapped a previous buffer-swap attempt.
  if ((uint32_t)(millis()-lastPublishMs) >= 250) {
    lastPublishMs=millis();
    stageCommand();
  }
}
