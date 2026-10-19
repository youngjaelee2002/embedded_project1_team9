#ifndef UNO3_TYPES_H
#define UNO3_TYPES_H

// Arduino IDE inserts automatically generated function prototypes in .ino code.
// Keep types used by those prototypes in a header included at the very top.
#include <Arduino.h>

// ===== Modes and cooperative motion states =====
enum Mode : uint8_t { STOP=0, SEARCH_SLOW=1, SEARCH_FAST=2, TRACK_TARGET=3 };
enum State : uint8_t { CALIBRATION=0, IDLE=1, MOVING=2, FAULT=3 };
enum Leg : uint8_t {
  NO_LEG=0, FULL_FORWARD, FULL_RETURN, TRACK_TRANSIT,
  TRACK_FORWARD, TRACK_RETURN, TO_ZERO
};

struct PositionStamp { uint32_t us; int16_t step; };
struct Target {
  uint8_t id, age, quality;
  bool seenInLeg, velocityValid;
  int16_t x, y, vx, vy, bearingStep;
  uint16_t range;
  uint32_t seenMs;
};
struct Cluster {
  bool active;
  int16_t first, last;
  uint16_t lastRange;
  uint32_t sumRange, firstUs, lastUs;
  uint8_t count, misses;
};

#endif // UNO3_TYPES_H
