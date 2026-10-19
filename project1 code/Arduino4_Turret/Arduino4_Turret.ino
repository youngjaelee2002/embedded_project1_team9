/*
 * ============================================================
 *  Arduino 4 - 표적 조준 유닛 (포탑 2기, SG90 서보, KY-008 레이저)
 * ============================================================
 *  역할
 *   - I2C 슬레이브(주소 0x20)
 *   - 아두이노 1이 표적을 판단해서 "조준할 좌표"를 원기둥 좌표로 보내면,
 *     포탑 A, B가 각자 자기 위치 기준으로 서보 각도를 계산해 조준한다.
 *   - 레이저는 항상 켜져 있다. 조준 완료 순간 1번 깜박이고, 발포 명령이면 5초 동안 계속 깜박인다.
 *
 *  ---------------- 팀 공통 좌표계 ----------------
 *   원기둥 좌표 (r, θ, z)
 *    - 원점 : 가운데 감시 센서(아두이노 3)의 회전축
 *    - r    : 원점에서의 수평 거리 [mm]
 *    - θ    : 북쪽(N) = 0도, 반시계 방향(+)으로 증가, 0~359.9도
 *             전송 단위는 0.1도 (예: 45.0도 -> 450)
 *    - z    : 높이 [mm] (현재 포탑은 좌우 회전만 하므로 사용하지 않음, 예약)
 *
 *                      N (θ = 0°)
 *                          ^
 *                          |      ● 표적 (r, θ)
 *       W (θ = 90°)  <-- [센서] -->  E (θ = 270°)
 *                          |
 *        [포탑 A]          S (θ = 180°)        [포탑 B]
 *       (남서 꼭짓점)                          (남동 꼭짓점)
 *   - 감시 영역 600 x 600mm, 센서가 정중앙, 포탑은 꼭짓점
 *
 *   - 각 포탑의 위치도 같은 원기둥 좌표 (r, θ)로 TURRET_CFG에 적는다.
 *   - 두 포탑 모두 서보 90도일 때 위쪽(북, θ=0도)을 바라보도록 장착한다 (서보 기준 동일).
 *     서보 각도가 커지면 반시계로 돈다.
 *       A: 0도 = B 방향(오른쪽), 45도 = 센서, 90도 = 위쪽   -> 사용 범위 0~90도
 *       B: 90도 = 위쪽, 135도 = 센서, 180도 = A 방향(왼쪽) -> 사용 범위 90~180도
 *     판(600 x 600) 안의 모든 점이 이 범위 안에 들어온다.
 *     켤 때는 센서 방향에서 시작. 대기(HOME)에서는 움직이지 않고 마지막 위치를 유지한다.
 *
 *  ---------------- 핀 배치 (Arduino Uno) ----------------
 *   A4 (SDA), A5 (SCL) : 아두이노 1과 I2C (GND 공통 필수)
 *   D9  : 포탑 A 서보 신호        D10 : 포탑 B 서보 신호
 *   D5  : 포탑 A 레이저 (KY-008 S)   D6  : 포탑 B 레이저 (KY-008 S),  KY-008 '-' = GND
 *   서보 전원은 외부 5V, 외부 전원 GND와 아두이노 GND 연결
 *
 *  ---------------- I2C 명령 (아두이노 1 -> 아두이노 4) ----------------
 *   모든 다바이트 값은 리틀엔디언(하위 바이트 먼저)
 *   0x01 AIM_BOTH : [0x01][rL][rH][θL][θH][zL][zH]          두 포탑 조준
 *   0x02 AIM_ONE  : [0x02][포탑][rL][rH][θL][θH][zL][zH]    포탑 0=A, 1=B
 *        r = uint16 [mm], θ = uint16 [0.1도, 0~3599], z = int16 [mm]
 *   0x03 HOME     : [0x03]                                  대기: 그 자리 유지, 조준/발포 취소
 *   0x04 LASER    : [0x04][포탑마스크][모드]
 *        포탑마스크: 1 = A, 2 = B, 3 = 둘 다
 *        모드: 3 = 발포 (조준 완료된 포탑에서 5초 동안 깜박임)
 *              0, 1, 2 = 발포 취소 (항상 켜짐 상태로)
 *   ---- 레이저 표시 ----
 *        평소           : 항상 켜짐
 *        조준 완료 순간 : 1번 깜박임 (0.5초 꺼짐 -> 켜짐)
 *        발포           : 5초 동안 빠르게 깜박임 (꺼짐 0.2 / 켜짐 0.2초 반복), 끝나면 다시 항상 켜짐
 *                         조준 완료 깜박임이 끝나고 0.8초 켜진 뒤 시작해서 서로 구분된다.
 *        시간은 아래 AIM_BLINK_OFF_MS / AIM_BLINK_GAP_MS / FIRE_STEP_MS / FIRE_DURATION_MS 로 바꾼다.
 *
 *  ---------------- I2C 상태 (Wire.requestFrom(0x20, 8)) ----------------
 *   [0]    플래그 b0 A조준완료 b1 B조준완료 b2 A범위밖 b3 B범위밖
 *                 b4 잘못된명령 b5 A발포중 b6 B발포중 (발포 대기 + 5초 깜박임 동안 1)
 *   [1][2] 포탑 A가 현재 바라보는 방위각 θ (uint16, 0.1도, 팀 공통 좌표계)
 *   [3][4] 포탑 B가 현재 바라보는 방위각 θ (uint16, 0.1도)
 *   [5]    포탑 A 서보 각도 (0~180)     [6] 포탑 B 서보 각도 (0~180)
 *   [7]    체크섬 = [0]~[6] XOR
 *
 *  ---------------- 시리얼 모니터 단독 테스트 (115200bps, "새 줄") ----------------
 *   T 300 0     -> 두 포탑이 (r=300mm, θ=0도) 조준  (θ는 정수 도 단위로 입력)
 *   C 300 0     -> T와 같음.  C 300 0 0 -> 포탑 A만,  C 300 0 1 -> 포탑 B만
 *   A 200 45    -> 포탑 A만 조준       B 200 315 -> 포탑 B만 조준
 *   F           -> 두 포탑 발포         L 3 3     -> 레이저 [마스크] [모드]
 *   H           -> 대기 (그 자리 유지)  Z         -> 센서 방향으로 이동 (시험용)
 *   P           -> 포탑 설정값 출력
 *   R           -> 1번에서 받은 I2C 명령 출력 켜기/끄기 (기본 켜짐)
 *                  예: [1번->4번] 01 2C 01 00 00 00 00 | AIM A+B  r=300mm  θ=0.0도
 *   ?           -> 현재 상태: 포탑별 서보 각도와 표적 (r, θ) (센서 기준, 북 0도 반시계 +),
 *                  두 레이저 교차점 (r, θ). 같은 표적을 맞히면 세 값이 모두 같다.
 * ============================================================
 */

#include <Wire.h>
#include <Servo.h>
#include <math.h>

// ---------------- 통신 ----------------
const uint8_t I2C_ADDR     = 0x20;
const uint8_t CMD_AIM_BOTH = 0x01;
const uint8_t CMD_AIM_ONE  = 0x02;
const uint8_t CMD_HOME     = 0x03;
const uint8_t CMD_LASER    = 0x04;
const uint8_t STATUS_LEN   = 8;

// ---------------- 서보 (SG90) ----------------
const unsigned long STEP_MS   = 15;    // 15ms마다 한 스텝
const int           STEP_DEG  = 2;     // 한 스텝 2도 (약 130도/초)
const unsigned long SETTLE_MS = 300;   // 크게 움직인 뒤 흔들림이 멈출 때까지 대기
const int           BIG_MOVE  = 3;     // 이보다 크게 움직이면 안정화 대기
const float         MIN_DIST_MM = 50;  // 포탑에서 이보다 가까운 표적은 조준하지 않음

// ---------------- 레이저 (KY-008) ----------------
// 레이저는 항상 켜짐. LASER_FIRE일 때만 발포 깜박임(끝나면 LASER_ON으로 돌아감).
enum LaserMode : uint8_t { LASER_OFF = 0, LASER_ON = 1, LASER_BLINK = 2, LASER_FIRE = 3 };
const unsigned long AIM_BLINK_OFF_MS = 500;  // 조준 완료 깜박임: 꺼져 있는 시간 (눈에 잘 보이게 0.5초)
const unsigned long AIM_BLINK_GAP_MS = 800;  // 조준 깜박임 뒤 발포 깜박임 전까지 켜져 있는 시간
const unsigned long FIRE_STEP_MS     = 200;  // 발포 깜박임 한 단계 (꺼짐/켜짐 각각 0.2초, 빠르게)
const unsigned long FIRE_DURATION_MS = 5000; // 발포 깜박임 전체 시간 (5초 동안 계속 깜박임)
const bool          LASER_ACTIVE_HIGH = true;   // HIGH = 켜짐

// ---------------- 포탑 설정 (팀 공통 원기둥 좌표로 실측 후 수정) ----------------
struct TurretCfg {
  const char* name;
  float   r;         // 포탑 회전축 위치: 센서로부터의 거리 [mm]
  float   thetaDeg;  // 포탑 회전축 위치: 방위각 [도] (북 0, 반시계 +)
  uint8_t servoPin;
  uint8_t laserPin;
  int     dir;       // 서보 각도가 커질 때 포탑이 반시계로 돌면 +1, 시계로 돌면 -1
  float   trimDeg;   // 장착 오차 보정 [도] (조준이 한쪽으로 치우치면 조정)
  int     minDeg;    // 서보 사용 범위 (A 0~90, B 90~180)
  int     maxDeg;
};

// 배치: 감시 영역 600 x 600mm, 센서가 정중앙, 포탑은 남쪽 두 꼭짓점
//   포탑 A (남서 꼭짓점): 센서 기준 서쪽 300mm, 남쪽 300mm -> r = 424.3mm, θ = 135.0도
//   포탑 B (남동 꼭짓점): 센서 기준 동쪽 300mm, 남쪽 300mm -> r = 424.3mm, θ = 225.0도
//   서보 90도 = 위쪽(북)으로 장착: trimDeg로 기준을 센서 방향에서 위쪽으로 옮긴다 (A +45, B -45)
//   -> 판 안 어느 점이든 A는 서보 0~90도, B는 90~180도 안에서 조준 가능
//   포탑을 다른 꼭짓점에 두면 θ만 바꾸면 된다: 북서 45도, 북동 315도 (r은 그대로 424.3)
const TurretCfg CFG[2] = {
  //  이름     r      θ     서보 레이저 방향 보정 최소 최대
  {  "A",  424.3, 135.0,     9,    5,   +1, 45.0,   0,  90 },   // 서보 90 = 위쪽, 0 = B 방향
  {  "B",  424.3, 225.0,    10,    6,   +1,-45.0,  90, 180 },   // 서보 90 = 위쪽, 180 = A 방향
};

// ---------------- 포탑 상태 ----------------
struct Turret {
  Servo         servo;
  float         px, py;       // 포탑 위치 (계산용 직교좌표, x = 동쪽, y = 북쪽) [mm]
  float         mountDeg;     // 서보 90도일 때 포탑이 바라보는 방위각 (= 센서 방향)
  int           cur, goal;    // 현재 / 목표 서보 각도
  bool          hasTarget, outOfRange, settling;
  unsigned long arrivedAt;
  LaserMode     laserMode;
  bool          laserOut;
  unsigned long laserTick;    // 발포 깜박임 시작 시각
  bool          wasAimed;     // 조준 완료 순간(꺼짐 -> 켜짐 변화)을 찾기 위한 이전 상태
  unsigned long aimBlinkAt;   // 조준 완료 깜박임 시작 시각
  float         tgtR, tgtTh;  // 받은 조준 표적 (센서 기준 원기둥 좌표 r [mm], θ [도])
};
Turret tur[2];
bool badCmd = false;

// ============================================================
//  좌표 변환 (팀 공통: 북 0도, 반시계 +)
//   직교좌표는 내부 계산용: x = 동쪽(+), y = 북쪽(+)
//   θ = 0 -> 북(+y), θ = 90 -> 서(-x), θ = 180 -> 남(-y), θ = 270 -> 동(+x)
// ============================================================
void cylToXY(float r, float thetaDeg, float& x, float& y) {
  float t = thetaDeg * PI / 180.0;
  x = -r * sin(t);
  y =  r * cos(t);
}

float bearingDeg(float dx, float dy) {          // (dx, dy) 방향의 방위각, 0~360
  float b = atan2(-dx, dy) * 180.0 / PI;
  if (b < 0) b += 360.0;
  return b;
}

float wrap180(float a) {                        // -180 < a <= 180 으로 정리
  while (a > 180.0)   a -= 360.0;
  while (a <= -180.0) a += 360.0;
  return a;
}

// 서보 각도 -> 포탑이 바라보는 방위각 (상태 보고용)
float servoToBearing(uint8_t i, int servoDeg) {
  float b = tur[i].mountDeg + CFG[i].trimDeg + (servoDeg - 90) / (float)CFG[i].dir;
  b = fmod(b, 360.0);
  if (b < 0) b += 360.0;
  return b;
}

// ============================================================
//  I2C 수신 버퍼 (인터럽트 -> loop)
// ============================================================
const uint8_t QSIZE = 4, MSG_MAX = 8;
volatile uint8_t qBuf[QSIZE][MSG_MAX];
volatile uint8_t qLen[QSIZE];
volatile uint8_t qHead = 0, qTail = 0;
volatile bool    qOverflow = false;
volatile uint8_t statusBuf[STATUS_LEN];

void onI2CReceive(int /*바이트 수*/) {
  uint8_t next = (qHead + 1) % QSIZE;
  if (next == qTail) {
    while (Wire.available()) Wire.read();
    qOverflow = true;
    return;
  }
  uint8_t len = 0;
  while (Wire.available()) {
    uint8_t b = Wire.read();
    if (len < MSG_MAX) qBuf[qHead][len++] = b;
  }
  qLen[qHead] = len;
  qHead = next;
}

void onI2CRequest() {
  uint8_t tmp[STATUS_LEN];
  for (uint8_t k = 0; k < STATUS_LEN; k++) tmp[k] = statusBuf[k];
  Wire.write(tmp, STATUS_LEN);
}

// ============================================================
//  조준
// ============================================================
void aimAt(uint8_t i, float r, float thetaDeg) {
  Turret&          t = tur[i];
  const TurretCfg& c = CFG[i];

  t.tgtR  = r;          // 시리얼 출력용: 이 포탑이 노리는 표적 (센서 기준)
  t.tgtTh = thetaDeg;

  float tx, ty;
  cylToXY(r, thetaDeg, tx, ty);
  float dx = tx - t.px, dy = ty - t.py;

  float need  = bearingDeg(dx, dy);                       // 포탑에서 본 표적 방위각
  float delta = wrap180(need - t.mountDeg - c.trimDeg);   // 장착 방향 기준 회전량
  int   servo = (int)lround(90 + c.dir * delta);

  t.outOfRange = (sqrt(dx * dx + dy * dy) < MIN_DIST_MM) ||
                 servo < c.minDeg || servo > c.maxDeg;
  servo = constrain(servo, c.minDeg, c.maxDeg);

  if (abs(servo - t.goal) > BIG_MOVE || abs(servo - t.cur) > BIG_MOVE) {
    t.settling = true;
    if (t.laserMode == LASER_FIRE) t.laserMode = LASER_ON;    // 표적이 바뀌면 이전 발포 취소
  }
  t.goal      = servo;
  t.hasTarget = true;
}

// 센서(원점)를 바라보는 서보 각도 = 기본 위치 (A 45도, B 135도)
int homeServo(uint8_t i) {
  return constrain((int)lround(90 - CFG[i].dir * CFG[i].trimDeg), CFG[i].minDeg, CFG[i].maxDeg);
}

// true : 대기(HOME)일 때 움직이지 않고 지금 위치 그대로 유지
// false: 대기(HOME)일 때 센서(원점) 방향으로 돌아감
const bool HOME_HOLDS_POSITION = true;

void goHome(uint8_t i) {
  tur[i].goal       = HOME_HOLDS_POSITION ? tur[i].cur : homeServo(i);
  tur[i].hasTarget  = false;
  tur[i].outOfRange = false;
  tur[i].laserMode  = LASER_ON;     // 발포 취소 (레이저는 계속 켜짐)
  if (abs(tur[i].goal - tur[i].cur) > BIG_MOVE) tur[i].settling = true;
}

bool isAimed(uint8_t i) {
  const Turret& t = tur[i];
  return t.hasTarget && !t.outOfRange && !t.settling &&
         abs(t.goal - t.cur) <= STEP_DEG &&
         millis() - t.arrivedAt >= SETTLE_MS;
}

void setLaser(uint8_t mask, uint8_t mode) {
  for (uint8_t i = 0; i < 2; i++) {
    if (!(mask & (1 << i))) continue;
    tur[i].laserMode = (mode == LASER_FIRE) ? LASER_FIRE : LASER_ON;  // 0/1/2 = 발포 취소
    tur[i].laserTick = millis();
  }
}

// ============================================================
//  명령 처리
// ============================================================
uint16_t readU16(const uint8_t* p) { return (uint16_t)(p[0] | (p[1] << 8)); }

// p = [rL][rH][θL][θH][zL][zH]
bool parseCyl(const uint8_t* p, float& r, float& thetaDeg) {
  uint16_t rr = readU16(p), th = readU16(p + 2);
  if (th >= 3600) return false;
  r = rr;
  thetaDeg = th / 10.0;
  // z = (int16_t)readU16(p + 4);   // 높이: 현재 미사용 (상하 회전 추가 시 사용)
  return true;
}

void handleMessage(const uint8_t* m, uint8_t len) {
  if (len == 0) return;
  float r, th;
  bool ok = false;

  if (m[0] == CMD_AIM_BOTH && len >= 7 && parseCyl(m + 1, r, th)) {
    aimAt(0, r, th);
    aimAt(1, r, th);
    ok = true;
  } else if (m[0] == CMD_AIM_ONE && len >= 8 && m[1] < 2 && parseCyl(m + 2, r, th)) {
    aimAt(m[1], r, th);
    ok = true;
  } else if (m[0] == CMD_HOME) {
    goHome(0);
    goHome(1);
    ok = true;
  } else if (m[0] == CMD_LASER && len >= 3 && m[2] <= LASER_FIRE) {
    setLaser(m[1], m[2]);
    ok = true;
  }
  badCmd = !ok;
}

// ---------------- 1번 -> 4번 수신 로그 ----------------
// 받은 I2C 명령을 시리얼 모니터에 출력한다. 시리얼에서 R 입력으로 켜기/끄기.
// 같은 명령이 연속으로 오면(예: 0.5초마다 같은 AIM) 2초에 한 번만 다시 출력한다.
bool i2cLog = true;
const unsigned long LOG_REPEAT_MS = 2000;

void logI2C(const uint8_t* m, uint8_t len, bool ok) {
  static uint8_t last[MSG_MAX], lastLen = 0;
  static unsigned long lastAt = 0;
  static uint16_t repeats = 0;
  if (!i2cLog) return;
  bool same = (len == lastLen) && memcmp(m, last, len) == 0;
  if (same && millis() - lastAt < LOG_REPEAT_MS) { repeats++; return; }

  Serial.print(F("[1번->4번] "));
  for (uint8_t k = 0; k < len; k++) {
    if (m[k] < 0x10) Serial.print('0');
    Serial.print(m[k], HEX); Serial.print(' ');
  }
  Serial.print(F("| "));
  if (!ok || len == 0) {
    Serial.print(F("잘못된 명령"));
  } else if (m[0] == CMD_AIM_BOTH || m[0] == CMD_AIM_ONE) {
    const uint8_t* p = (m[0] == CMD_AIM_BOTH) ? m + 1 : m + 2;
    Serial.print(F("AIM "));
    if (m[0] == CMD_AIM_BOTH) Serial.print(F("A+B"));
    else Serial.print(m[1] == 0 ? F("A") : F("B"));
    Serial.print(F("  r=")); Serial.print(readU16(p));
    Serial.print(F("mm  θ=")); Serial.print(readU16(p + 2) / 10.0, 1); Serial.print(F("도"));
  } else if (m[0] == CMD_HOME) {
    Serial.print(HOME_HOLDS_POSITION ? F("HOME -> 대기 (그 자리 유지, 발포 취소)")
                                     : F("HOME -> 대기 (센서 방향, 발포 취소)"));
  } else if (m[0] == CMD_LASER) {
    Serial.print(m[2] == LASER_FIRE ? F("발포 ") : F("발포 취소/준비 "));
    Serial.print(m[1] == 3 ? F("A+B") : (m[1] == 1 ? F("A") : F("B")));
  }
  if (same && repeats) { Serial.print(F("  (같은 명령 ")); Serial.print(repeats + 1); Serial.print(F("회째)")); }
  Serial.println();

  if (!same) repeats = 0;
  memcpy(last, m, len); lastLen = len; lastAt = millis();
}

void processQueue() {
  while (true) {
    uint8_t msg[MSG_MAX], len;
    noInterrupts();
    if (qTail == qHead) { interrupts(); break; }
    len = qLen[qTail];
    for (uint8_t k = 0; k < len; k++) msg[k] = qBuf[qTail][k];
    qTail = (qTail + 1) % QSIZE;
    interrupts();
    handleMessage(msg, len);
    logI2C(msg, len, !badCmd);
  }
  if (qOverflow) {
    qOverflow = false; badCmd = true;
    if (i2cLog) Serial.println(F("[1번->4번] 수신 버퍼 넘침 (명령 일부 버림)"));
  }
}

// ============================================================
//  주기 작업
// ============================================================
void updateServos() {
  static unsigned long last = 0;
  if (millis() - last < STEP_MS) return;
  last = millis();

  for (uint8_t i = 0; i < 2; i++) {
    Turret& t = tur[i];
    if (t.cur != t.goal) {
      t.cur += constrain(t.goal - t.cur, -STEP_DEG, STEP_DEG);
      t.servo.write(t.cur);
    }
    if (t.cur == t.goal && t.settling) {
      t.settling  = false;
      t.arrivedAt = millis();
    }
  }
}

void writeLaser(uint8_t i, bool on) {
  tur[i].laserOut = on;
  digitalWrite(CFG[i].laserPin, (on == LASER_ACTIVE_HIGH) ? HIGH : LOW);
}

void updateLasers() {
  unsigned long now = millis();
  for (uint8_t i = 0; i < 2; i++) {
    Turret& t = tur[i];
    bool aimed = isAimed(i);
    if (aimed && !t.wasAimed) t.aimBlinkAt = now;           // 조준 완료 순간
    t.wasAimed = aimed;

    unsigned long sinceAim = now - t.aimBlinkAt;
    bool aimBlinkBusy = aimed && sinceAim < AIM_BLINK_OFF_MS + AIM_BLINK_GAP_MS;
    bool want = true;                                        // 기본: 항상 켜짐
    if (aimed && sinceAim < AIM_BLINK_OFF_MS) want = false;  // 조준 완료: 1번 깜박임

    if (t.laserMode == LASER_FIRE && (!t.hasTarget || t.outOfRange))
      t.laserMode = LASER_ON;       // 조준할 수 없는 포탑의 발포는 취소
    if (t.laserMode == LASER_FIRE) {
      if (!aimed || aimBlinkBusy) {
        t.laserTick = now;          // 조준 완료 + 조준 깜박임이 끝난 뒤부터 발포 시작
      } else {
        unsigned long el = now - t.laserTick;
        if (el >= FIRE_DURATION_MS) t.laserMode = LASER_ON;  // 5초 끝 -> 다시 항상 켜짐
        else if ((el / FIRE_STEP_MS) % 2 == 0) want = false; // 꺼짐/켜짐 번갈아
      }
    }
    if (want != t.laserOut) writeLaser(i, want);
  }
}

void buildStatus() {
  uint8_t s[STATUS_LEN];
  uint16_t bA = (uint16_t)lround(servoToBearing(0, tur[0].cur) * 10) % 3600;
  uint16_t bB = (uint16_t)lround(servoToBearing(1, tur[1].cur) * 10) % 3600;
  s[0] = (isAimed(0)        ? 0x01 : 0) | (isAimed(1)        ? 0x02 : 0) |
         (tur[0].outOfRange ? 0x04 : 0) | (tur[1].outOfRange ? 0x08 : 0) |
         (badCmd            ? 0x10 : 0) |
         (tur[0].laserMode == LASER_FIRE ? 0x20 : 0) |       // 발포 중(대기 포함)
         (tur[1].laserMode == LASER_FIRE ? 0x40 : 0);
  s[1] = bA & 0xFF; s[2] = bA >> 8;
  s[3] = bB & 0xFF; s[4] = bB >> 8;
  s[5] = (uint8_t)tur[0].cur;
  s[6] = (uint8_t)tur[1].cur;
  s[7] = 0;
  for (uint8_t k = 0; k < 7; k++) s[7] ^= s[k];

  noInterrupts();
  for (uint8_t k = 0; k < STATUS_LEN; k++) statusBuf[k] = s[k];
  interrupts();
}

// ============================================================
//  시리얼 모니터 (단독 테스트)
// ============================================================
// 두 포탑의 현재 서보 각도로 계산한 레이저 교차점 (센서 기준 원기둥 좌표). 교차하지 않으면 false
bool laserCrossing(float& r, float& thetaDeg) {
  float b0 = servoToBearing(0, tur[0].cur) * PI / 180.0, b1 = servoToBearing(1, tur[1].cur) * PI / 180.0;
  float d0x = -sin(b0), d0y = cos(b0), d1x = -sin(b1), d1y = cos(b1);   // 북 0도, 반시계 + 방향 벡터
  float den = d0x * d1y - d0y * d1x;
  if (fabs(den) < 1e-3) return false;                                   // 평행
  float ex = tur[1].px - tur[0].px, ey = tur[1].py - tur[0].py;
  float t0 = (ex * d1y - ey * d1x) / den, t1 = (ex * d0y - ey * d0x) / den;
  if (t0 <= 0 || t1 <= 0) return false;                                 // 포탑 뒤쪽
  float x = tur[0].px + t0 * d0x, y = tur[0].py + t0 * d0y;
  r = sqrt(x * x + y * y);
  thetaDeg = atan2(-x, y) * 180.0 / PI;
  if (thetaDeg < 0) thetaDeg += 360.0;
  if (r < 1.0) thetaDeg = 0;   // 센서 바로 위면 각도는 의미 없음
  return true;
}

// 출력: 각 포탑이 노리는 표적의 센서 기준 (r, θ)  -> 같은 표적이면 A와 B가 같아야 한다
//       + 지금 서보 각도로 계산한 두 레이저 교차점 (r, θ) -> 조준 완료 뒤 표적과 같아야 한다
void printStatus() {
  for (uint8_t i = 0; i < 2; i++) {
    Serial.print(F("포탑 ")); Serial.print(CFG[i].name);
    Serial.print(F(": 서보 ")); Serial.print(tur[i].cur); Serial.print(F("도"));
    if (tur[i].hasTarget) {
      Serial.print(F(", 표적 r=")); Serial.print(tur[i].tgtR, 0);
      Serial.print(F("mm θ=")); Serial.print(tur[i].tgtTh, 1); Serial.print(F("도"));
    }
    if (tur[i].laserMode == LASER_FIRE) Serial.print(F(" [발포]"));
    if (!tur[i].hasTarget)      Serial.print(F(" [대기]"));
    else if (tur[i].outOfRange) Serial.print(F(" [범위밖]"));
    else if (isAimed(i))        Serial.print(F(" [조준완료]"));
    else                        Serial.print(F(" [조준중]"));
    Serial.print(F("  |  "));
  }
  float r, th;
  Serial.print(F("두 레이저 교차점: "));
  if (laserCrossing(r, th)) {
    Serial.print(F("r=")); Serial.print(r, 0); Serial.print(F("mm θ=")); Serial.print(th, 1); Serial.println(F("도"));
  } else Serial.println(F("없음"));
}

void printConfig() {
  for (uint8_t i = 0; i < 2; i++) {
    Serial.print(F("포탑 ")); Serial.print(CFG[i].name);
    Serial.print(F(": r=")); Serial.print(CFG[i].r, 1);
    Serial.print(F("mm, θ=")); Serial.print(CFG[i].thetaDeg, 1);
    Serial.print(F("도 | 서보 90도 방향 방위=")); Serial.print(servoToBearing(i, 90), 1);
    Serial.print(F("도, 센서 방향 서보=")); Serial.print(homeServo(i));
    Serial.print(F("도 | 조준 가능 방위 약 "));
    Serial.print(servoToBearing(i, CFG[i].dir > 0 ? CFG[i].minDeg : CFG[i].maxDeg), 0);
    Serial.print(F("~"));
    Serial.print(servoToBearing(i, CFG[i].dir > 0 ? CFG[i].maxDeg : CFG[i].minDeg), 0);
    Serial.println(F("도 (반시계 방향으로)"));
  }
}

void localAim(uint8_t cmd, uint8_t turretIdx, int r, int thetaDeg) {
  uint16_t th = (uint16_t)(((long)thetaDeg % 360 + 360) % 360 * 10);
  uint8_t m[MSG_MAX], n = 0;
  m[n++] = cmd;
  if (cmd == CMD_AIM_ONE) m[n++] = turretIdx;
  m[n++] = r & 0xFF;  m[n++] = (r >> 8) & 0xFF;
  m[n++] = th & 0xFF; m[n++] = th >> 8;
  m[n++] = 0;         m[n++] = 0;              // z = 0
  handleMessage(m, n);
}

void handleSerial() {
  static char line[32];
  static uint8_t pos = 0;
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\r') continue;
    if (ch != '\n') { if (pos < sizeof(line) - 1) line[pos++] = ch; continue; }
    line[pos] = '\0';
    pos = 0;

    int a, b, t;
    char c = toupper(line[0]);
    if      (c == 'C' && sscanf(line + 1, "%d %d %d", &a, &b, &t) == 3 && (t == 0 || t == 1))
      localAim(CMD_AIM_ONE, (uint8_t)t, a, b);
    else if ((c == 'T' || c == 'C') && sscanf(line + 1, "%d %d", &a, &b) == 2) localAim(CMD_AIM_BOTH, 0, a, b);
    else if (c == 'A' && sscanf(line + 1, "%d %d", &a, &b) == 2) localAim(CMD_AIM_ONE, 0, a, b);
    else if (c == 'B' && sscanf(line + 1, "%d %d", &a, &b) == 2) localAim(CMD_AIM_ONE, 1, a, b);
    else if (c == 'H') { uint8_t m[1] = { CMD_HOME }; handleMessage(m, 1); }
    else if (c == 'Z') { for (uint8_t i = 0; i < 2; i++) { goHome(i); tur[i].goal = homeServo(i); tur[i].settling = true; } }
    else if (c == 'F') setLaser(3, LASER_FIRE);
    else if (c == 'L' && sscanf(line + 1, "%d %d", &a, &b) == 2) {
      uint8_t m[3] = { CMD_LASER, (uint8_t)a, (uint8_t)b };
      handleMessage(m, 3);
    }
    else if (c == 'P') { printConfig(); continue; }
    else if (c == 'R') { i2cLog = !i2cLog; Serial.println(i2cLog ? F("1번 수신 출력 켜짐") : F("1번 수신 출력 꺼짐")); continue; }
    else if (c != '?') {
      Serial.println(F("명령: T r θ / C r θ [0|1] / A r θ / B r θ / F / L 마스크 모드 / H 대기 / Z 센서방향 / P / R(1번 수신 출력) / ?"));
      continue;
    }
    printStatus();
  }
}

void reportChanges() {
  static uint8_t last = 0xFF;
  if (statusBuf[0] != last) { last = statusBuf[0]; printStatus(); }
}

// ============================================================
void setup() {
  Serial.begin(115200);

  for (uint8_t i = 0; i < 2; i++) {
    Turret& t = tur[i];
    cylToXY(CFG[i].r, CFG[i].thetaDeg, t.px, t.py);
    t.mountDeg   = bearingDeg(-t.px, -t.py);   // 포탑에서 원점(센서)을 보는 방위각
    t.cur = t.goal = homeServo(i);     // 시작도 센서 방향
    t.hasTarget  = t.outOfRange = t.settling = false;
    t.arrivedAt  = 0;
    t.laserMode  = LASER_ON;
    t.laserTick  = 0;
    t.wasAimed   = false;
    t.aimBlinkAt = 0;
    t.tgtR = t.tgtTh = 0;
    pinMode(CFG[i].laserPin, OUTPUT);
    writeLaser(i, true);               // 레이저는 켜자마자 항상 켜짐
    t.servo.write(t.cur);              // attach 전에 시작 각도 지정 (엉뚱한 위치로 튀지 않음)
    t.servo.attach(CFG[i].servoPin);
    delay(500);                        // 두 서보가 동시에 시작해 전류가 몰리지 않도록 0.5초 간격
  }

  buildStatus();
  Wire.begin(I2C_ADDR);
  Wire.onReceive(onI2CReceive);
  Wire.onRequest(onI2CRequest);

  Serial.println(F("아두이노 4 조준 유닛 시작 (I2C 0x20, 원기둥 좌표: 북 0도 반시계)"));
  printConfig();
  Serial.println(F("명령: T r θ / C r θ [0|1] / A r θ / B r θ / F / L 마스크 모드 / H 대기 / Z 센서방향 / P / R(1번 수신 출력) / ?"));
}

void loop() {
  processQueue();     // 아두이노 1 명령
  handleSerial();     // 시리얼 테스트 명령
  updateServos();     // 서보 이동
  updateLasers();     // 레이저
  buildStatus();      // 상태 프레임 갱신
  reportChanges();    // 상태 변화 출력
}
