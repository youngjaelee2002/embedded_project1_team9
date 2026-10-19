#ifndef RADAR_PROTOCOL_H
#define RADAR_PROTOCOL_H
#include <Arduino.h>

// Radar SPI protocol v1. Shared, identical header in UNO3 and UNO1 sketch folders.
// Wire bytes are explicit, packed little-endian. Never cast a C struct to a frame.
namespace RadarWire {
static const uint8_t SIZE = 48;
static const uint8_t SYNC = 0xA5;
static const uint8_t VERSION = 0x01;
static const uint8_t SLOT_COUNT = 3;
static const uint8_t SLOT_SIZE = 12;
static const uint8_t COORD_CARTESIAN = 0;
static const uint8_t COORD_CYLINDRICAL = 1;
static const uint8_t CMD_REHOME = 4;
static const uint8_t CMD_FLAG_TOF_RETRY = 0x01;

// byte 4 of UNO3->UNO1 status frame
static const uint8_t ST_HOMED = 0x01;
static const uint8_t ST_TOF_READY = 0x02;
static const uint8_t ST_TORQUE_HOLD = 0x04;
static const uint8_t ST_HALL_ACTIVE = 0x08;
static const uint8_t ST_REHOME_PENDING = 0x10;
static const uint8_t ST_CYLINDRICAL = 0x20;
static const uint8_t ST_TARGET_LOST = 0x40;
static const uint8_t ST_LINK_TIMEOUT = 0x80;

static const uint8_t OBJ_VELOCITY_VALID = 0x01;
static const uint8_t OBJ_IN_BOUNDS = 0x02;
static const uint8_t OBJ_STALE = 0x04;

// Telemetry byte 46: most recent relevant protocol or scanner error.
enum Error : uint8_t {
  ERR_NONE=0, ERR_RX_CRC=1, ERR_RX_HEADER=2, ERR_BAD_COMMAND=3,
  ERR_NOT_HOMED=4, ERR_TOF_UNAVAILABLE=5, ERR_LINK_TIMEOUT=6,
  ERR_TOF_STALE=7, ERR_HALL_DRIFT=8, ERR_TRACK_LOST=9
};

inline uint8_t crc8(const uint8_t *bytes, uint8_t length) {
  // CRC-8/ATM: poly=0x07, init=0, no reflection, xorout=0.
  uint8_t crc=0;
  for (uint8_t i=0; i<length; ++i) {
    crc ^= bytes[i];
    for (uint8_t b=0; b<8; ++b)
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
  }
  return crc;
}
inline bool valid(const uint8_t *frame) {
  return frame[0] == SYNC && frame[1] == VERSION &&
         crc8(frame, SIZE - 1) == frame[SIZE - 1];
}
inline void putU16(uint8_t *b, uint8_t at, uint16_t v) {
  b[at] = (uint8_t)v; b[at+1] = (uint8_t)(v >> 8);
}
inline uint16_t getU16(const uint8_t *b, uint8_t at) {
  return (uint16_t)b[at] | ((uint16_t)b[at+1] << 8);
}
inline void putI16(uint8_t *b, uint8_t at, int16_t v) {
  putU16(b, at, (uint16_t)v);
}
inline int16_t getI16(const uint8_t *b, uint8_t at) {
  return (int16_t)getU16(b, at);
}
} // namespace RadarWire
#endif
