/*
  UNO 3 SPI scanner, ATmega328P / 16 MHz. Derived from the supplied tested
  standalone motor, Hall, ToF, cluster and tracking prototype.
  UNO 3 = SPI MASTER (D9->UNO1 D10 SS, D11 MOSI, D12 MISO, D13 SCK, common GND).
  D10 remains OUTPUT/HIGH. UNO 3 USB serial = SEND-ONLY diagnostic telemetry.
  Manually align the Hall magnet while torque is FREE for five seconds.
  Torque becomes HOLD automatically after calibration and stays HOLD elsewhere.
  Verify ENA polarity, driver current and tether-safe mechanical travel first.
*/

#include <Arduino.h>
#include "UNO3_Types.h"
#include "RadarProtocol.h"
#include <Wire.h>
#include <SPI.h>
#include <VL53L0X.h>
#include <util/atomic.h>
#include <avr/interrupt.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

#if !defined(__AVR_ATmega328P__) || F_CPU != 16000000UL
#error Use a 16 MHz ATmega328P Uno R3-compatible board.
#endif

// ===== Hardware constants: preserve the verified pin mapping =====
const uint8_t STEP_PIN = 2, DIR_PIN = 3, ENA_PIN = 4, HALL_PIN = 7, HALL_ANALOG = A0;
// COMMON-ANODE TB6600 "off-line" input: LOW on ENA- usually frees the shaft.
// Check your own driver; if opposite, change this constant to HIGH.
const uint8_t ENA_FREE_LEVEL = LOW;
// EDIT THIS SINGLE SETTING to select the 12-byte SPI object slot layout.
// GUI telemetry always publishes BOTH representations, regardless of setting.
//const uint8_t SPI_COORDINATE_MODE = RadarWire::COORD_CARTESIAN;
const uint8_t SPI_COORDINATE_MODE = RadarWire::COORD_CYLINDRICAL;
static_assert(SPI_COORDINATE_MODE <= RadarWire::COORD_CYLINDRICAL, "Choose Cartesian or cylindrical");
const uint8_t SPI_CS_PIN = 9;
const uint32_t SPI_FREQUENCY_HZ = 125000UL;
const uint16_t SPI_INTERBYTE_GAP_US = 100;
const uint16_t SPI_CS_SETUP_US = 120;
const uint16_t SPI_PERIOD_MS = 100;
const uint32_t SPI_COMMAND_TIMEOUT_MS = 3000UL;
const uint8_t HALL_ACTIVE_LEVEL = LOW;
const uint8_t DIR_FORWARD_LEVEL = LOW;        // Verified test.ino: forward=LOW
const uint16_t PULSES_PER_REV = 3200;         // 200 full steps * 16 microsteps
const uint16_t STEP_LOW_US = 20;              // Common-anode LOW pulse width
const uint16_t START_PPS = 800;               // 0.25 RPS @ 1/16 microstepping
const uint16_t SLOW_PPS = 1600;               // 0.5 RPS
const uint16_t FAST_PPS = 3200;               // 1.0 RPS
const uint16_t TRACK_PPS = 1600;
const uint16_t RAMP_STEPS = 192;              // Same angular ramp (24/400 rev)
const uint16_t TURN_PAUSE_MS = 20;

const uint32_t TOF_BUDGET_US = 20000;         // Verified testtof.ino
const uint16_t MIN_TARGET_MM = 40;
const uint16_t MAX_TARGET_MM = 300;           // Inclusive maximum is not used
const int16_t ZERO_OFFSET_DEG = 0;
const int16_t RADIAL_OFFSET_MM = 0;
const uint16_t HALL_TOLERANCE_STEPS = 176;    // ~19.8 deg @ 3200 steps/rev
const uint8_t MAX_HALL_STRIKES = 3;
const uint8_t MAX_OBJECTS = 3;
// Object validation and expiry thresholds (tune after physical testing).
const uint8_t MAX_MISSED_LEGS = 2;
const uint8_t MIN_CLUSTER_SAMPLES_NORMAL = 3;
const uint8_t MIN_CLUSTER_SAMPLES_FAST = 2;
const uint16_t MATCH_RADIUS_MM = 180;
const uint8_t TRACK_HALF_WIDTH_DEG = 30;
const uint8_t TRACK_VISITS_BEFORE_REFRESH = 4;
const uint16_t TRACK_HALF_STEPS =
    (uint32_t)PULSES_PER_REV * TRACK_HALF_WIDTH_DEG / 360;
const uint16_t CLUSTER_GAP_STEPS =
    (uint32_t)PULSES_PER_REV * 18 / 360;       // 18 degrees
const uint16_t CLUSTER_RANGE_GAP_MM = 60;

// Modes, states and scanner types are in UNO3_Types.h so the Arduino
// auto-prototype generator sees them before any generated function declarations.

// ===== Timer1: direct-register nonblocking STEP generation =====
volatile int16_t motorPos = 0, motorTarget = 0;
volatile int8_t motorDir = 1;
volatile uint16_t motorPeriodUs = 10000;
volatile bool motorRunning = false, pulseLow = false;

ISR(TIMER1_COMPA_vect) {
  if (pulseLow) {
    PORTD |= _BV(PD2);
    pulseLow = false;
    if (!motorRunning || motorPos == motorTarget) {
      motorRunning = false;
      TIMSK1 &= (uint8_t)~_BV(OCIE1A);
      return;
    }
    OCR1A = (uint16_t)(2UL * (motorPeriodUs - STEP_LOW_US) - 1);
  } else {
    if (!motorRunning || motorPos == motorTarget) {
      motorRunning = false;
      TIMSK1 &= (uint8_t)~_BV(OCIE1A);
      return;
    }
    PORTD &= (uint8_t)~_BV(PD2);             // Common-anode active-LOW pulse
    pulseLow = true;
    motorPos += motorDir;
    OCR1A = 2 * STEP_LOW_US - 1;
  }
  TCNT1 = 0;
}

int16_t positionNow() {
  int16_t p;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { p = motorPos; }
  return p;
}

bool movingNow() {
  bool running;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { running = motorRunning; }
  return running;
}

void stopPulses() {
  // Do not truncate an active LOW pulse during a commanded stop.
  if (pulseLow) delayMicroseconds(STEP_LOW_US);
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    TIMSK1 &= (uint8_t)~_BV(OCIE1A);
    motorRunning = false;
    pulseLow = false;
    PORTD |= _BV(PD2);
  }
}

void motorTo(int16_t target) {
  stopPulses();
  // All callers must verify torqueHolding before commanding motion.
  int16_t current = positionNow();
  if (current == target) return;
  int8_t direction = (target > current) ? 1 : -1;
  bool forwardLevel = DIR_FORWARD_LEVEL == HIGH;
  bool level = (direction > 0) ? forwardLevel : !forwardLevel;
  if (level) PORTD |= _BV(PD3);
  else PORTD &= (uint8_t)~_BV(PD3);
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    motorTarget = target;
    motorDir = direction;
    motorPeriodUs = 1000000UL / START_PPS;
    TCNT1 = 0;
    OCR1A = 199;                           // First STEP after 100 us DIR setup
    TIFR1 = _BV(OCF1A);
    motorRunning = true;
    TIMSK1 |= _BV(OCIE1A);
  }
}

// ===== Scanner data =====
// Types are declared in UNO3_Types.h (included above).

const uint8_t HISTORY_CAPACITY = 24;
PositionStamp history[HISTORY_CAPACITY];
uint8_t historyHead = 0, historyCount = 0;
uint32_t lastHistoryUs = 0;
Target targets[MAX_OBJECTS] = {};
Cluster cluster = {};
VL53L0X tof;

State state = CALIBRATION;
Mode mode = STOP, resumeMode = STOP;
Leg leg = NO_LEG, queuedLeg = NO_LEG;
int16_t legStart = 0, queuedTarget = 0, trackLow = 0, trackHigh = 0;
uint16_t legPeak = SLOW_PPS, queuedPeak = SLOW_PPS;
uint32_t queuedAtMs = 0;
bool queued = false, homed = false, tofOK = false, tofStale = false;
bool needManualCalibration = false;
uint8_t hallStrikes = 0, nextId = 1;
uint8_t preferredId = 0, resumeTargetId = 0, activeTrackId = 0;
uint8_t trackVisits = 0, lastRoundRobinId = 0;
uint32_t calibrationStartedMs = 0, lastCalSecond = 0;
uint32_t lastRangeMs = 0, lastTofPollUs = 0;
uint32_t lastStatusMs = 0, acceptFromUs = 0, sweepStartedMs = 0;
uint32_t cycleCount = 0;
bool sawHallInactive = false, hallEdgeCaptured = false, hallEdgeValid = false;
int16_t hallEdgeStep = -1;
uint32_t sampleCount = 0, invalidSampleCount = 0;
uint16_t latestRangeMm = 0;
uint32_t lastSnapshotMs = 0, lastSPIAtMs = 0, lastValidCommandMs = 0;
uint16_t spiGoodFrames = 0, spiBadFrames = 0;
uint8_t telemetrySeq = 0, lastCommandSeq = 0xFF, lastError = RadarWire::ERR_NONE;
bool haveCommand = false, spiTimeout = false, targetLost = false;
uint8_t spiTx[RadarWire::SIZE], spiRx[RadarWire::SIZE];

// Explicit forward declarations (do not depend on Arduino-generated prototypes).
void printObject(const Target &o); // Explicitly declare after the type is known.
void beginCalibration(bool preserveMode);
void requestMode(Mode requested, uint8_t selected);
void startFullScan();
void planTracking();
void beginTrackVisit();
void finishCluster();
void finishObservations();
void queueLeg(Leg next, int16_t target, uint16_t pps);
void clearTargets();
void serviceSPI();
void printStatus();
void publishAllObjects();

// ===== ENA: command holding torque or free shaft =====
// NOTE: This reflects our *requested* state, not hardware feedback.
bool torqueHolding = false;

void setTorqueHolding(bool hold) {
  // Command only; cannot sense actual driver current/torque.
  const uint8_t level = hold ? (ENA_FREE_LEVEL == LOW ? HIGH : LOW)
                             : ENA_FREE_LEVEL;
  digitalWrite(ENA_PIN, level);
  bool changed = (torqueHolding != hold);
  torqueHolding = hold;
  if (changed) {
    Serial.print(F("TORQUE|"));
    Serial.println(hold ? F("HOLD") : F("FREE"));
  }
}

// ===== Hall calibration and reference checks =====
bool hallActive() {
  bool d = (PIND & _BV(PD7)) != 0;
  return d == (HALL_ACTIVE_LEVEL == HIGH);
}

void clearTargets() {
  memset(targets, 0, sizeof(targets));
  cluster.active = false;
  nextId = 1;
  lastRoundRobinId = 0;
  Serial.println(F("RESET|OBJECTS"));
}

void beginCalibration(bool preserveMode) {
  stopPulses();
  setTorqueHolding(false);  // Automatically free the shaft for manual Hall alignment.
  if (preserveMode) {
    resumeMode = mode;
    resumeTargetId = preferredId;
  } else {
    resumeMode = STOP;
    resumeTargetId = 0;
  }
  mode = STOP;
  state = CALIBRATION;
  leg = NO_LEG;
  queued = false;
  homed = false;
  calibrationStartedMs = 0;
  lastCalSecond = 0;
  needManualCalibration = false;
  clearTargets();
  Serial.println(F("CALIBRATION|not-detected"));
  Serial.println(F("INFO|Shaft_FREE_for_manual_Hall_alignment"));
}

void serviceCalibration() {
  if (state != CALIBRATION) return;
  uint32_t now = millis();
  if (!hallActive()) {
    if (calibrationStartedMs != 0) Serial.println(F("CALIBRATION|not-detected"));
    calibrationStartedMs = 0;
    lastCalSecond = 0;
    return;
  }
  if (!calibrationStartedMs) {
    calibrationStartedMs = now + 1; // nonzero sentinel, including millis()==0
    lastCalSecond = 0;
  }
  uint32_t elapsed = (now + 1) - calibrationStartedMs;
  if (elapsed < 5000) {
    uint8_t remain = 5 - elapsed / 1000;
    if (remain != lastCalSecond) {
      lastCalSecond = remain;
      Serial.print(F("CALIBRATION|magnet-detected|"));
      Serial.println(remain);
    }
    return;
  }
  stopPulses();
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { motorPos = 0; motorTarget = 0; }
  setTorqueHolding(true); // Automatic HOLD before ANY non-calibration state.
  homed = true;
  state = IDLE;
  hallStrikes = 0;
  historyCount = historyHead = 0;
  Serial.println(F("CALIBRATION|reference-locked"));
  Serial.println(F("INFO|Calibration_complete;_torque_HOLD"));
  if (resumeMode != STOP && !spiTimeout) {
    Mode requested = resumeMode;
    uint8_t selected = resumeTargetId;
    resumeMode = STOP;
    requestMode(requested, selected);
  }
}

void startHallCheck() {
  sawHallInactive = false;
  hallEdgeCaptured = false;
  hallEdgeValid = false;
  hallEdgeStep = -1;
}

void serviceHallCheck() {
  if (state != MOVING || queued || leg != FULL_FORWARD) return;
  bool active = hallActive();
  if (!active) sawHallInactive = true;
  if (active && sawHallInactive && !hallEdgeCaptured) {
    hallEdgeCaptured = true;
    hallEdgeStep = positionNow();
    hallEdgeValid =
        (hallEdgeStep >= (int16_t)(PULSES_PER_REV - HALL_TOLERANCE_STEPS) &&
         hallEdgeStep <= (int16_t)(PULSES_PER_REV + HALL_TOLERANCE_STEPS));
  }
}

void finishHallCheck() {
  if (hallEdgeValid) {
    hallStrikes = 0;
    Serial.print(F("HALL_CHECK|OK|"));
    Serial.println(hallEdgeStep);
  } else {
    if (hallStrikes < 255) ++hallStrikes;
    Serial.print(F("HALL_CHECK|MISS|"));
    Serial.print(hallEdgeStep);
    Serial.print('|');
    Serial.println(hallStrikes);
    if (hallStrikes >= MAX_HALL_STRIKES) {
      needManualCalibration = true;
      lastError = RadarWire::ERR_HALL_DRIFT;
      Serial.println(F("ERROR|HALL_3_STRIKES|returning-to-zero-before-calibration"));
    }
  }
}

// ===== Midpoint timestamp -> motor step =====
void recordPosition() {
  uint32_t now = micros();
  if ((uint32_t)(now - lastHistoryUs) < 1000) return;
  lastHistoryUs = now;
  history[historyHead].us = now;
  history[historyHead].step = positionNow();
  historyHead = (historyHead + 1) % HISTORY_CAPACITY;
  if (historyCount < HISTORY_CAPACITY) ++historyCount;
}

bool positionAt(uint32_t atUs, int16_t &step) {
  uint32_t best = 0xFFFFFFFFUL;
  for (uint8_t i = 0; i < historyCount; ++i) {
    int32_t diff = (int32_t)(history[i].us - atUs);
    uint32_t delta = diff < 0 ? (uint32_t)(-diff) : (uint32_t)diff;
    if (delta < best) { best = delta; step = history[i].step; }
  }
  return best <= 4000;
}

// ===== Objects: neighboring-range clusters, world XY, stable IDs =====
int8_t findTarget(uint8_t id) {
  if (!id) return -1;
  for (uint8_t i = 0; i < MAX_OBJECTS; ++i)
    if (targets[i].id == id) return i;
  return -1;
}

uint8_t allocateId() {
  for (uint16_t tries = 0; tries < 255; ++tries) {
    uint8_t id = nextId++;
    if (nextId == 0) nextId = 1;
    if (id && findTarget(id) < 0) return id;
  }
  return 0;
}

uint16_t absDifference(int16_t a, int16_t b) {
  return a > b ? (uint16_t)(a-b) : (uint16_t)(b-a);
}

int16_t clampVelocity(int32_t v) {
  if (v > 32000) return 32000;
  if (v < -32000) return -32000;
  return (int16_t)v;
}

void printObject(const Target &o) {
  // USB output is for the PC only. Raw ToF samples are not streamed.
  const float dx = (float)o.x - 300.0f;
  const float dy = (float)o.y - 300.0f;
  const float r = sqrtf(dx*dx + dy*dy);
  float theta = atan2f(dy, dx) * 18000.0f / PI;
  if (theta < 0) theta += 36000.0f;
  const float inv = r > 0.01f ? 1.0f / r : 0;
  const int16_t vr = o.velocityValid ? clampVelocity(lroundf((dx * o.vx + dy * o.vy) * inv)) : 0;
  const int16_t vt = o.velocityValid ? clampVelocity(lroundf((-dy * o.vx + dx * o.vy) * inv)) : 0;
  Serial.print(F("OBJECT|")); Serial.print(o.id);
  Serial.print('|'); Serial.print(o.x);
  Serial.print('|'); Serial.print(o.y);
  Serial.print('|'); Serial.print((uint16_t)lroundf(r));
  Serial.print('|'); Serial.print((uint16_t)lroundf(theta) % 36000U);
  Serial.print('|'); Serial.print(o.velocityValid ? o.vx : 0);
  Serial.print('|'); Serial.print(o.velocityValid ? o.vy : 0);
  Serial.print('|'); Serial.print(vr);
  Serial.print('|'); Serial.print(vt);
  Serial.print('|'); Serial.print(o.quality);
  Serial.print('|'); Serial.print(o.age);
  Serial.print('|'); Serial.println((o.velocityValid ? RadarWire::OBJ_VELOCITY_VALID : 0) |
     ((o.x >= 0 && o.x <= 600 && o.y >= 0 && o.y <= 600) ? RadarWire::OBJ_IN_BOUNDS : 0) |
     (o.age ? RadarWire::OBJ_STALE : 0));
}

void finishCluster() {
  if (!cluster.active) return;
  cluster.active = false;
  // Fast full legs (including tracking refreshes) have fewer angular samples.
  const uint8_t minSamples = (legPeak >= FAST_PPS)
      ? MIN_CLUSTER_SAMPLES_FAST : MIN_CLUSTER_SAMPLES_NORMAL;
  if (cluster.count < minSamples) return;
  int16_t centerStep = (cluster.first + cluster.last) / 2;
  uint16_t range = cluster.sumRange / cluster.count;
  uint32_t midUs = cluster.firstUs + (cluster.lastUs - cluster.firstUs) / 2;
  uint32_t seenMs = millis() - (uint32_t)(micros() - midUs) / 1000;
  float theta = (ZERO_OFFSET_DEG - (float)centerStep * 360.0f / PULSES_PER_REV) * DEG_TO_RAD;
  float radius = (float)range + RADIAL_OFFSET_MM;
  int16_t x = (int16_t)lroundf(300.0f + radius * cosf(theta));
  int16_t y = (int16_t)lroundf(300.0f + radius * sinf(theta));
  if (x < 0 || x > 600 || y < 0 || y > 600) return;

  int8_t slot = -1;
  uint32_t best = (uint32_t)MATCH_RADIUS_MM * MATCH_RADIUS_MM + 1;
  for (uint8_t i = 0; i < MAX_OBJECTS; ++i) {
    Target &o = targets[i];
    if (!o.id || o.seenInLeg) continue;
    int32_t dx = (int32_t)x - o.x, dy = (int32_t)y - o.y;
    uint32_t dist2 = (uint32_t)(dx*dx + dy*dy);
    if (dist2 < best) { best = dist2; slot = i; }
  }
  if (slot < 0) {
    // Suppress a second fragment of the same object within this scan leg.
    for (uint8_t i = 0; i < MAX_OBJECTS; ++i) {
      const Target &o = targets[i];
      if (!o.id || !o.seenInLeg) continue;
      int32_t dx = (int32_t)x - o.x, dy = (int32_t)y - o.y;
      if ((uint32_t)(dx*dx + dy*dy) <
          (uint32_t)MATCH_RADIUS_MM * MATCH_RADIUS_MM) return;
    }
    for (uint8_t i = 0; i < MAX_OBJECTS; ++i)
      if (!targets[i].id) { slot = i; break; }
  }
  if (slot < 0) return;

  Target &o = targets[slot];
  if (!o.id) {
    o.id = allocateId();
    o.velocityValid = false;
    o.vx = o.vy = 0;
  } else {
    uint32_t dt = seenMs - o.seenMs;
    if (dt >= 40 && dt <= 10000) {
      int16_t vx = clampVelocity(((int32_t)x-o.x)*1000L/(int32_t)dt);
      int16_t vy = clampVelocity(((int32_t)y-o.y)*1000L/(int32_t)dt);
      o.vx = o.velocityValid ? (int16_t)(((int32_t)o.vx + vx)/2) : vx;
      o.vy = o.velocityValid ? (int16_t)(((int32_t)o.vy + vy)/2) : vy;
      o.velocityValid = true;
    } else o.velocityValid = false;
  }
  o.x = x; o.y = y;
  o.range = range; o.bearingStep = centerStep;
  o.seenMs = seenMs; o.age = 0; o.seenInLeg = true;
  o.quality = (cluster.count >= 3) ? 220 : ((cluster.count == 2) ? 150 : 60);
  targetLost = false;
  printObject(o);
}

void addSample(int16_t step, uint16_t range, uint32_t midpointUs) {
  if (step < 0 || step > PULSES_PER_REV) return;
  if (range < MIN_TARGET_MM || range >= MAX_TARGET_MM) {
    if (cluster.active && ++cluster.misses >= 2) finishCluster();
    return;
  }
  if (cluster.active && absDifference(step, cluster.last) <= CLUSTER_GAP_STEPS &&
      absDifference(range, cluster.lastRange) <= CLUSTER_RANGE_GAP_MM) {
    cluster.last = step;
    cluster.lastRange = range;
    cluster.lastUs = midpointUs;
    cluster.sumRange += range;
    cluster.misses = 0;
    if (cluster.count < 250) ++cluster.count;
  } else {
    finishCluster();
    cluster.active = true;
    cluster.first = cluster.last = step;
    cluster.lastRange = range;
    cluster.sumRange = range;
    cluster.firstUs = cluster.lastUs = midpointUs;
    cluster.count = 1;
    cluster.misses = 0;
  }
}

void finishObservations() {
  finishCluster();
  bool full = leg == FULL_FORWARD || leg == FULL_RETURN;
  bool narrow = leg == TRACK_FORWARD || leg == TRACK_RETURN;
  for (uint8_t i = 0; i < MAX_OBJECTS; ++i) {
    Target &o = targets[i];
    if (!o.id) continue;
    bool covered = full || (narrow && o.bearingStep >= trackLow && o.bearingStep <= trackHigh);
    if (covered && !o.seenInLeg && o.age < 255) ++o.age;
    o.seenInLeg = false;
    if (o.age >= MAX_MISSED_LEGS) {
      Serial.print(F("LOST|")); Serial.println(o.id);
      targetLost = true;
      memset(&o, 0, sizeof(o));
    }
  }
}

// ===== Sensor acquisition, Pololu registers, nonblocking polling =====
void initToF() {
  tofOK = false;
  tofStale = false;
  tof.setTimeout(100);
  if (!tof.init()) {
    Serial.println(F("ERROR|TOF_INIT|VL53L0X_not_detected"));
    return;
  }
  if (!tof.setMeasurementTimingBudget(TOF_BUDGET_US)) {
    Serial.println(F("ERROR|TOF_BUDGET|20ms_not_supported"));
    return;
  }
  tof.startContinuous();
  tofOK = true;
  lastRangeMs = millis();
  Serial.println(F("TOF|READY|20000us"));
}

void serviceToF() {
  if (!tofOK) return;
  uint32_t nowUs = micros();
  if ((uint32_t)(nowUs - lastTofPollUs) < 2000) return;
  lastTofPollUs = nowUs;
  uint8_t ready = tof.readReg(VL53L0X::RESULT_INTERRUPT_STATUS);
  if (tof.last_status || !(ready & 0x07)) {
    if ((uint32_t)(millis() - lastRangeMs) > 700) {
      if (!tofStale) Serial.println(F("ERROR|TOF_STALE|no_results_700ms"));
      tofStale = true;
      if (state == MOVING) {
        stopPulses();
        state = FAULT;
        mode = STOP;
        homed = false;
        queued = false;
        leg = NO_LEG;
        lastError = RadarWire::ERR_TOF_STALE;
        Serial.println(F("ERROR|SCAN_STOPPED|ToF_stale_rehome_required"));
      }
    }
    return;
  }
  uint16_t mm = tof.readReg16Bit(VL53L0X::RESULT_RANGE_STATUS + 10);
  bool readBad = tof.last_status != 0;
  tof.writeReg(VL53L0X::SYSTEM_INTERRUPT_CLEAR, 0x01);
  if (readBad || tof.last_status) return;
  lastRangeMs = millis();
  tofStale = false;
  uint32_t midUs = micros() - TOF_BUDGET_US / 2;
  int16_t measuredStep = -1;
  bool hasAngle = homed && positionAt(midUs, measuredStep);
  bool valid = (mm >= MIN_TARGET_MM && mm < MAX_TARGET_MM);
  ++sampleCount;
  if (!valid || !hasAngle) ++invalidSampleCount;
  latestRangeMm = mm;
  if (state == MOVING && !queued && hasAngle &&
      (int32_t)(midUs - acceptFromUs) >= 0 &&
      (leg == FULL_FORWARD || leg == FULL_RETURN ||
       leg == TRACK_FORWARD || leg == TRACK_RETURN))
    addSample(measuredStep, valid ? mm : 65535, midUs);
}

// ===== Asynchronous motor motion and scan scheduling =====
void queueLeg(Leg next, int16_t target, uint16_t pps) {
  queuedLeg = next;
  queuedTarget = target;
  queuedPeak = pps;
  queuedAtMs = millis() + TURN_PAUSE_MS;
  queued = true;
  state = MOVING;
}

void startQueuedLeg() {
  queued = false;
  if (!torqueHolding) {
    state = FAULT;
    mode = STOP;
    homed = false;
    leg = NO_LEG;
    Serial.println(F("ERROR|TORQUE_FREE|motor_motion_blocked"));
    return;
  }
  leg = queuedLeg;
  legStart = positionNow();
  legPeak = queuedPeak;
  acceptFromUs = micros() + TOF_BUDGET_US;  // Reject ranges begun before the leg.
  cluster.active = false;
  for (uint8_t i = 0; i < MAX_OBJECTS; ++i) targets[i].seenInLeg = false;
  if (leg == FULL_FORWARD) {
    startHallCheck();
    sweepStartedMs = millis();
  }
  Serial.print(F("LEG|")); Serial.print((uint8_t)leg);
  Serial.print('|'); Serial.print(legStart);
  Serial.print('|'); Serial.println(queuedTarget);
  motorTo(queuedTarget);
}

void rampMotor() {
  if (!movingNow()) return;
  int16_t pos = positionNow(), target;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { target = motorTarget; }
  uint16_t edge = min(absDifference(pos, legStart), absDifference(pos, target));
  if (edge > RAMP_STEPS) edge = RAMP_STEPS;
  uint16_t pps = START_PPS + (uint32_t)(legPeak - START_PPS)*edge/RAMP_STEPS;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { motorPeriodUs = 1000000UL / pps; }
}

void startFullScan() {
  if (state == CALIBRATION || !homed) return;
  uint16_t pps = (mode == SEARCH_SLOW) ? SLOW_PPS : FAST_PPS;
  queueLeg(FULL_FORWARD, PULSES_PER_REV, pps);
}

int8_t chooseTrackSlot() {
  if (preferredId) return findTarget(preferredId);
  int8_t higher = -1, lowest = -1;
  for (uint8_t i = 0; i < MAX_OBJECTS; ++i) {
    if (!targets[i].id) continue;
    if (lowest < 0 || targets[i].id < targets[lowest].id) lowest = i;
    if (targets[i].id > lastRoundRobinId &&
        (higher < 0 || targets[i].id < targets[higher].id)) higher = i;
  }
  return higher >= 0 ? higher : lowest;
}

void beginTrackVisit() {
  if (mode != TRACK_TARGET) { queueLeg(TO_ZERO, 0, SLOW_PPS); return; }
  int8_t slot = chooseTrackSlot();
  if (slot < 0) {
    targetLost = true;
    lastError = RadarWire::ERR_TRACK_LOST;
    Serial.println(F("TARGET_LOST|fallback_search_slow"));
    mode = SEARCH_SLOW;
    queueLeg(TO_ZERO, 0, SLOW_PPS);
    return;
  }
  Target &o = targets[slot];
  activeTrackId = o.id;
  if (!preferredId) lastRoundRobinId = o.id;
  trackLow = max((int16_t)0, (int16_t)(o.bearingStep - TRACK_HALF_STEPS));
  trackHigh = min((int16_t)PULSES_PER_REV, (int16_t)(o.bearingStep + TRACK_HALF_STEPS));
  Serial.print(F("TRACK|")); Serial.print(activeTrackId);
  Serial.print('|'); Serial.print(trackLow);
  Serial.print('|'); Serial.println(trackHigh);
  if (positionNow() == trackLow) queueLeg(TRACK_FORWARD, trackHigh, TRACK_PPS);
  else queueLeg(TRACK_TRANSIT, trackLow, TRACK_PPS);
}

void planTracking() {
  trackVisits = 0;
  beginTrackVisit();
}

void serviceMotion() {
  if (state != MOVING) return;
  if (queued) {
    if ((int32_t)(millis() - queuedAtMs) >= 0) startQueuedLeg();
    return;
  }
  rampMotor();
  serviceHallCheck();
  if (movingNow()) return;
  finishObservations();
  switch (leg) {
    case FULL_FORWARD:
      finishHallCheck();
      queueLeg(FULL_RETURN, 0, mode == SEARCH_SLOW ? SLOW_PPS : FAST_PPS);
      break;
    case FULL_RETURN:
      ++cycleCount;
      Serial.print(F("CYCLE|")); Serial.print(cycleCount);
      Serial.print('|'); Serial.println(millis() - sweepStartedMs);
      if (needManualCalibration) { beginCalibration(true); break; }
      if (mode == TRACK_TARGET) planTracking();
      else if (mode == SEARCH_SLOW || mode == SEARCH_FAST) startFullScan();
      else { state = IDLE; leg = NO_LEG; }
      break;
    case TRACK_TRANSIT:
      if (mode == TRACK_TARGET) queueLeg(TRACK_FORWARD, trackHigh, TRACK_PPS);
      else queueLeg(TO_ZERO, 0, SLOW_PPS);
      break;
    case TRACK_FORWARD:
      if (mode == TRACK_TARGET) queueLeg(TRACK_RETURN, trackLow, TRACK_PPS);
      else queueLeg(TO_ZERO, 0, SLOW_PPS);
      break;
    case TRACK_RETURN:
      if (mode != TRACK_TARGET) queueLeg(TO_ZERO, 0, SLOW_PPS);
      else if (++trackVisits >= TRACK_VISITS_BEFORE_REFRESH)
        queueLeg(TO_ZERO, 0, SLOW_PPS);
      else beginTrackVisit();
      break;
    case TO_ZERO:
      if (needManualCalibration) beginCalibration(true);
      else if (mode == SEARCH_SLOW || mode == SEARCH_FAST || mode == TRACK_TARGET)
        startFullScan();
      else { state = IDLE; leg = NO_LEG; }
      break;
    default:
      state = IDLE;
      leg = NO_LEG;
      break;
  }
}

// ===== Read-only serial diagnostics; no PC-to-UNO3 command parser =====
void printStatus() {
  uint8_t count=0;
  for (uint8_t i=0; i<MAX_OBJECTS; ++i) if (targets[i].id) ++count;
  // STATUS|ms|mode|state|homed|step|hall|dout|aout|tof|strikes|
  // selected|objects|holding|spiOK|spiBad|lastCmdSeq|coord|error|range|samples|invalid|timeout
  Serial.print(F("STATUS|")); Serial.print(millis());
  Serial.print('|'); Serial.print((uint8_t)mode);
  Serial.print('|'); Serial.print((uint8_t)state);
  Serial.print('|'); Serial.print(homed ? 1 : 0);
  Serial.print('|'); Serial.print(positionNow());
  Serial.print('|'); Serial.print(hallActive() ? 1 : 0);
  Serial.print('|'); Serial.print((PIND & _BV(PD7)) ? 1 : 0);
  Serial.print('|'); Serial.print(analogRead(HALL_ANALOG));
  Serial.print('|'); Serial.print(tofOK && !tofStale ? 1 : 0);
  Serial.print('|'); Serial.print(hallStrikes);
  Serial.print('|'); Serial.print(preferredId);
  Serial.print('|'); Serial.print(count);
  Serial.print('|'); Serial.print(torqueHolding ? 1 : 0);
  Serial.print('|'); Serial.print(spiGoodFrames);
  Serial.print('|'); Serial.print(spiBadFrames);
  Serial.print('|'); Serial.print(haveCommand ? lastCommandSeq : 255);
  Serial.print('|'); Serial.print(SPI_COORDINATE_MODE);
  Serial.print('|'); Serial.print(lastError);
  Serial.print('|'); Serial.print(latestRangeMm);
  Serial.print('|'); Serial.print(sampleCount);
  Serial.print('|'); Serial.print(invalidSampleCount);
  Serial.print('|'); Serial.println(spiTimeout ? 1 : 0);
}

void publishAllObjects() {
  for (uint8_t i=0; i<MAX_OBJECTS; ++i)
    if (targets[i].id) printObject(targets[i]);
}

void requestMode(Mode requested, uint8_t selected) {
  if (requested == STOP) {
    resumeMode = STOP;
    if (state == CALIBRATION) return; // Calibration must remain FREE until Hall lock.
    bool interrupted = (state == MOVING);
    stopPulses();
    mode = STOP;
    preferredId = 0;
    state = IDLE;
    queued = false;
    leg = NO_LEG;
    cluster.active = false;
    if (interrupted) {
      homed = false;
      clearTargets();
      Serial.println(F("INFO|Stopped_mid_sweep;_rehome_required"));
    }
    Serial.println(F("MODE|STOP"));
    return;
  }
  if (spiTimeout) {
    lastError = RadarWire::ERR_LINK_TIMEOUT;
    Serial.println(F("ERROR|SPI_LINK_TIMED_OUT"));
    return;
  }
  if (state == CALIBRATION || !homed || state == FAULT) {
    lastError = RadarWire::ERR_NOT_HOMED;
    Serial.println(F("ERROR|NOT_HOMED"));
    return;
  }
  if (!tofOK || tofStale) {
    lastError = RadarWire::ERR_TOF_UNAVAILABLE;
    Serial.println(F("ERROR|TOF_UNAVAILABLE"));
    return;
  }
  // Normally always true, because only CALIBRATION is allowed to release.
  if (!torqueHolding) setTorqueHolding(true);
  resumeMode = STOP;
  mode = requested;
  preferredId = selected;
  targetLost = false;
  Serial.print(F("MODE|")); Serial.print((uint8_t)mode);
  Serial.print('|'); Serial.println(preferredId);
  if (state == MOVING) return; // safely changes at next leg boundary
  if (positionNow() != 0) queueLeg(TO_ZERO, 0, SLOW_PPS);
  else startFullScan();
}

// ===== SPI master framing and bounded command processing =====
void buildTelemetry() {
  memset(spiTx, 0, sizeof(spiTx));
  spiTx[0] = RadarWire::SYNC;
  spiTx[1] = RadarWire::VERSION;
  spiTx[2] = telemetrySeq++;
  spiTx[3] = (uint8_t)mode;
  uint8_t bits = (homed ? RadarWire::ST_HOMED : 0) |
                 (tofOK && !tofStale ? RadarWire::ST_TOF_READY : 0) |
                 (torqueHolding ? RadarWire::ST_TORQUE_HOLD : 0) |
                 (hallActive() ? RadarWire::ST_HALL_ACTIVE : 0) |
                 (needManualCalibration ? RadarWire::ST_REHOME_PENDING : 0) |
                 (SPI_COORDINATE_MODE == RadarWire::COORD_CYLINDRICAL ? RadarWire::ST_CYLINDRICAL : 0) |
                 (targetLost ? RadarWire::ST_TARGET_LOST : 0) |
                 (spiTimeout ? RadarWire::ST_LINK_TIMEOUT : 0);
  spiTx[4] = bits;
  uint8_t count = 0;
  for (uint8_t i=0; i<MAX_OBJECTS; ++i) {
    const Target &o = targets[i];
    if (!o.id) continue;
    const uint8_t at = 6 + count * RadarWire::SLOT_SIZE;
    spiTx[at] = o.id;
    spiTx[at+1] = (o.velocityValid ? RadarWire::OBJ_VELOCITY_VALID : 0) |
           ((o.x>=0 && o.x<=1000 && o.y>=0 && o.y<=1000) ? RadarWire::OBJ_IN_BOUNDS : 0) |
           (o.age ? RadarWire::OBJ_STALE : 0);
    if (SPI_COORDINATE_MODE == RadarWire::COORD_CARTESIAN) {
      RadarWire::putI16(spiTx, at+2, o.x);
      RadarWire::putI16(spiTx, at+4, o.y);
      RadarWire::putI16(spiTx, at+6, o.velocityValid ? o.vx : 0);
      RadarWire::putI16(spiTx, at+8, o.velocityValid ? o.vy : 0);
    } else {
      const float dx = (float)o.x - 300.0f;
      const float dy = (float)o.y - 300.0f;
      const float r = sqrtf(dx*dx + dy*dy);
      float theta = atan2f(dy, dx) * 18000.0f / PI;
      if (theta < 0) theta += 36000.0f;
      const float inv = r > 0.01f ? 1.0f / r : 0;
      const int16_t vr = o.velocityValid ? clampVelocity(lroundf((dx*o.vx + dy*o.vy)*inv)) : 0;
      const int16_t vt = o.velocityValid ? clampVelocity(lroundf((-dy*o.vx + dx*o.vy)*inv)) : 0;
      RadarWire::putU16(spiTx, at+2, (uint16_t)lroundf(r));
      RadarWire::putU16(spiTx, at+4, (uint16_t)lroundf(theta) % 36000U);
      RadarWire::putI16(spiTx, at+6, vr);
      RadarWire::putI16(spiTx, at+8, vt);
    }
    spiTx[at+10] = o.quality;
    spiTx[at+11] = o.age;
    ++count;
  }
  spiTx[5] = count;
  spiTx[42] = (uint8_t)state;
  spiTx[43] = hallStrikes;
  spiTx[44] = preferredId;
  spiTx[45] = haveCommand ? lastCommandSeq : 0xFF;
  spiTx[46] = lastError;
  spiTx[47] = RadarWire::crc8(spiTx,47);
}

void handleHubCommand() {
  if (spiRx[0] != RadarWire::SYNC || spiRx[1] != RadarWire::VERSION) {
    if (!spiTimeout) lastError = RadarWire::ERR_RX_HEADER;
    return;
  }
  if (!RadarWire::valid(spiRx)) {
    if (!spiTimeout) lastError = RadarWire::ERR_RX_CRC;
    return;
  }
  if (spiGoodFrames != 65535) ++spiGoodFrames;
  lastValidCommandMs = millis();
  spiTimeout = false;
  const uint8_t cmdseq = spiRx[2], requested = spiRx[3], flags = spiRx[4], selected = spiRx[5];
  // Reject undefined codes and flags. Reserved bytes 6..46 must be zero for v1.
  bool reservedOK = true;
  for (uint8_t i=6; i<=46; ++i) if (spiRx[i] != 0) { reservedOK = false; break; }
  if (requested > RadarWire::CMD_REHOME || (flags & (uint8_t)~RadarWire::CMD_FLAG_TOF_RETRY) || !reservedOK) {
    lastError = RadarWire::ERR_BAD_COMMAND;
    return;
  }
  if (haveCommand && lastCommandSeq == cmdseq) return; // duplicate; refresh liveness only
  haveCommand = true;
  lastCommandSeq = cmdseq;
  lastError = RadarWire::ERR_NONE;
  if ((flags & RadarWire::CMD_FLAG_TOF_RETRY) && state != MOVING) initToF();
  if (requested == RadarWire::CMD_REHOME) {
    beginCalibration(true); // automatic resume of previous active scanner mode
  } else {
    requestMode((Mode)requested, selected);
  }
}

void serviceSPI() {
  uint32_t now = millis();
  if ((uint32_t)(now - lastSPIAtMs) >= SPI_PERIOD_MS) {
    lastSPIAtMs = now;
    buildTelemetry();
    SPI.beginTransaction(SPISettings(SPI_FREQUENCY_HZ, MSBFIRST, SPI_MODE0));
    digitalWrite(SPI_CS_PIN, LOW);
    delayMicroseconds(SPI_CS_SETUP_US); // Margin for UNO1 pin-change SPDR preload
    for (uint8_t i=0; i<RadarWire::SIZE; ++i) {
      spiRx[i] = SPI.transfer(spiTx[i]);
      // Keep angular history / Hall edge observation alive during the ~8 ms SPI burst.
      // SPI mode and byte gap remain valid; Timer1 stepping stays interrupt-driven.
      if ((i & 3U) == 0U) {
        recordPosition();
        serviceHallCheck();
        rampMotor();
      }
      delayMicroseconds(SPI_INTERBYTE_GAP_US);
    }
    digitalWrite(SPI_CS_PIN, HIGH);
    SPI.endTransaction();
    if (!RadarWire::valid(spiRx)) {
      if (spiBadFrames != 65535) ++spiBadFrames;
    }
    handleHubCommand();
  }
  if (haveCommand && !spiTimeout &&
      (uint32_t)(millis() - lastValidCommandMs) > SPI_COMMAND_TIMEOUT_MS) {
    spiTimeout = true;
    lastError = RadarWire::ERR_LINK_TIMEOUT;
    if (state == MOVING) requestMode(STOP, 0);
    if (state == CALIBRATION) {
      // Do NOT resume an old scan after a manual 5s calibration if hub is gone.
      resumeMode = STOP;
      resumeTargetId = 0;
    }
    Serial.println(F("ERROR|SPI_TIMEOUT|stop_and_rehome_if_interrupted"));
  }
}

void setup() {
  // Pin mapping and direct-register pulse generation are from the prototype.
  // Output ENA latch is set BEFORE output direction to minimize boot glitches.
  if (ENA_FREE_LEVEL == HIGH) PORTD |= _BV(PD4);
  else PORTD &= (uint8_t)~_BV(PD4);
  DDRD |= _BV(DDD4);
  PORTD |= _BV(PD2);
  if (DIR_FORWARD_LEVEL == HIGH) PORTD |= _BV(PD3);
  else PORTD &= (uint8_t)~_BV(PD3);
  DDRD |= _BV(DDD2) | _BV(DDD3);
  DDRD &= (uint8_t)~_BV(DDD7);
  PORTD &= (uint8_t)~_BV(PD7);
  pinMode(HALL_ANALOG, INPUT);

  pinMode(SPI_CS_PIN, OUTPUT);
  digitalWrite(SPI_CS_PIN, HIGH);
  pinMode(10, OUTPUT);   // Physical SS MUST be an output for AVR SPI master.
  digitalWrite(10, HIGH);
  SPI.begin();

  TCCR1A = 0;
  TCCR1B = _BV(WGM12) | _BV(CS11);
  TIMSK1 &= (uint8_t)~_BV(OCIE1A);

  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);
  Serial.println(F("HELLO|UNO3|SPI_MASTER|1"));
  Serial.println(F("CONFIG|PULSES_PER_REV|3200"));
  Serial.print(F("CONFIG|SPI_COORD_MODE|")); Serial.println(SPI_COORDINATE_MODE);
  Serial.println(F("CONFIG|SPI_HZ|125000"));
  Serial.print(F("CONFIG|ENA_FREE_LEVEL|"));
  Serial.println(ENA_FREE_LEVEL == LOW ? F("LOW") : F("HIGH"));
  Serial.println(F("CONFIG|SERIAL|READ_ONLY"));
  initToF();
  beginCalibration(false);
}

void loop() {
  recordPosition();
  serviceCalibration();
  serviceToF();
  serviceMotion();
  serviceSPI();
  if ((uint32_t)(millis() - lastStatusMs) >= 250) {
    lastStatusMs = millis();
    printStatus();
  }
  if ((uint32_t)(millis() - lastSnapshotMs) >= 750) {
    lastSnapshotMs = millis();
    publishAllObjects();
  }
}
