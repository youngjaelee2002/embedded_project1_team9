#ifndef TEAM_AGREEMENT_H
#define TEAM_AGREEMENT_H
#include "controller_types.h"
#include "ui_uart.h"
#include "scanner_protocol.h"
#include "scanner_session.h"
#include "auto_policy.h"
#include "turret4_protocol.h"

// ============================================================
// [팀 합의 필요] 이 파일의 표시된 부분을 채우면 된다.
// UART는 Arduino2_UI의 3바이트 명령 / 7바이트 표적 상태 규격으로 구현했다.
// SPI는 실제 RadarWire v1과 대조했다. scanner_config.h의 R/T는 팀 확인이 필요하다.
// 4번은 새 원본의 북0 원기둥 AIM 및 8바이트 상태 규격. 센서 중심을 공유한다.
// 나머지 READY는 해당 해석·생성 함수까지 구현한 다음 1로 바꾼다.
// ============================================================
#define UART_PROTOCOL_READY 1
#define SPI_PROTOCOL_READY 1
#define TURRET_PROTOCOL_READY 1
#define CONTROL_POLICY_READY (SCANNER_PROFILE_CONFIRMED && AUTO_PARAMETERS_CONFIRMED && TURRET_ASSIGNMENT_CONFIRMED && TURRET_WORLD_FRAME_CONFIRMED && AUTO_DANGER_RADIUS_MM > 0 && AUTO_DANGER_RADIUS_MM <= SCANNER_MAX_RADIUS_MM && AUTO_LOOKAHEAD_MS > 0)

#define TURRET_TIMEOUT_MS 300UL         // 구현 기본값. 마지막 정상 응답의 유효 시간
#define TURRET_STATUS_POLL_MS 20UL       // 80ms 레이저 펄스 확인용. 실물에서 처리량 확인
#define TURRET_REPORT_BYTES 8
#define DETECTION_LIST_KIND 1           // 실제 3번의 전체 스냅샷. 빠진 ID는 제거

// 정해진 사항: 일반 상태와 추적 좌표 전송은 500ms마다
#define UART_STATUS_PERIOD_MS 500UL
#define TARGET_SEND_PERIOD_MS 500UL

// ------------------------------------------------------------
// UART: 2번 명령 해석 / 2번에 보낼 상태 생성 (구현 완료)
// ------------------------------------------------------------
Arduino2Link::CommandReceiver ui_command_receiver;

uint8_t team_uart_command(uint8_t data) {
    return ui_command_receiver.push(data, millis());
}

// 마지막 TRACK 명령([02 33 ID 03])에 들어 있던 표적 ID
uint8_t team_uart_track_id(void) {
    return ui_command_receiver.track_id;
}

uint8_t team_uart_status(const CentralReport *report, uint8_t *out) {
    return Arduino2Link::encodeStatus(report, out);
}

uint8_t team_uart_link_ok(uint32_t now) {
    (void)now;
    // 현재 2번은 주기 heartbeat/ACK를 보내지 않는다.
    // 무입력 시간을 단선으로 간주하는 검사는 사용하지 않는다.
    // 이 반환값은 물리 연결 확인을 뜻하지 않는다. 단선 검출은 규격 확장 필요.
    return UART_PROTOCOL_READY;
}

// ------------------------------------------------------------
// SPI: RadarWire v1 수신 + 고정 전체 탐색 시작/명령 유지
// ------------------------------------------------------------
uint8_t team_spi_crc_ok(const uint8_t *data, uint8_t length) {
    return ScannerProtocol::crcOk(data, length);
}

uint8_t team_decode_detection(const uint8_t *data, uint8_t length,
                             DetectionFrame *frame) {
    return ScannerProtocol::decode(data, length, frame);
}

uint8_t team_scanner_command(const ScannerCommand *command, uint8_t *out) {
    return ScannerProtocol::encodeCommand(command, out);
}

ScannerSession::CommandLink scanner_link;
ScannerSession::SampleClock scanner_samples;
void team_scanner_observe(const DetectionFrame *frame) { scanner_link.observe(*frame); }
void team_scanner_request(uint8_t command, uint8_t target_id) {
    scanner_link.request(command, target_id);
}
void team_scanner_next(ScannerCommand *command) { *command = scanner_link.command; }
void team_observe_samples(DetectionFrame *frame, const uint8_t *wire, uint32_t now) {
    scanner_samples.apply(*frame, wire, now);
}

// ------------------------------------------------------------
// I2C: 4번 명령 생성 / 상태 해석
// ------------------------------------------------------------
Turret4Link::AimTracker turret_aim_tracker;
uint8_t team_turret_command(const TurretCommand *command, uint8_t *out) {
    return Turret4Link::encode(command, out);
}

uint8_t team_turret_status(const uint8_t *data, uint8_t length,
                          TurretReport *report) {
    if (!Turret4Link::decode(data,length,report,millis())) { turret_aim_tracker.reset(); return 0; }
    turret_aim_tracker.observe(report);
    return 1;
}

void team_turret_sent(const TurretCommand *command, uint8_t ok, uint32_t now) {
    turret_aim_tracker.sent(command,ok,now);
}

void team_turret_read_failed(void) { turret_aim_tracker.reset(); }

uint8_t team_turret_pose_ready(uint8_t mask, uint8_t id, uint16_t seq, uint16_t r, uint16_t theta,
                               const TurretReport *report) {
    return turret_aim_tracker.ready(mask,id,seq,r,theta,report);
}

// ------------------------------------------------------------
// 판단 연결: 위험 기준은 auto_policy.h, R/T는 scanner_config.h에서 확인한다.
// ------------------------------------------------------------
uint8_t team_xy_valid(const Target *target) {
    return AutoPolicy::positionValid(target);
}

uint8_t team_dangerous(const Target *target) {
    return AutoPolicy::dangerous(target);
}

float team_priority(const Target *target, uint8_t mode) {
    // AUTO: T 이내 진입하는 표적 중 가장 빠른 것. 내부 표적은 t=0.
    // MANUAL: 가장 가까운 유효 표적. 동점이면 중앙 제어에서 현재 표적 유지.
    return AutoPolicy::priority(target, mode);
}

uint8_t team_turret_mask(const Target *target) {
    // 받은 4번 코드의 실제 CFG/범위 판정과 동일하게, 도달 가능한 포탑을 모두 선택한다.
    return target ? Turret4Link::reachableMask(target->r_mm,target->theta_cdeg) : TURRET_NONE;
}

uint8_t team_auto_fire(const Target *target, const TurretReport *report) {
    (void)report;
    // 중앙 제어가 최신 측정/조준 완료/STOP/오류를 따로 확인한다.
    // 현재 표적 선택이 유지되는 동안 1회. 표적 교체/상실/정지 시 래치는 초기화된다.
    return AutoPolicy::dangerous(target);
}

uint8_t team_rearm_fire(uint8_t mode, const TurretReport *report) {
    (void)mode; (void)report;
    // [팀 합의 필요] 같은 표적의 추가 수동 승인/자동 반복 조건과 간격.
    // 본체는 completed_fire_seq로 직전 모의 동작 완료를 먼저 확인한다.
    // 수동 모드는 재준비 뒤에도 사용자에게 새 승인을 받아야 한다.
    return 0;
}

uint8_t team_allow_resume(uint8_t user_command) {
    // 2번은 정지 후 MODE 버튼을 누르면 기존 모드 명령을 다시 보낸다.
    // 전체 설정 확인은 controller_process_commands에서 별도로 수행한다.
    return user_command == USER_AUTO || user_command == USER_MANUAL;
}

// 표시 번호 임시 배정: 북1, 북서2, 서3, 남서4, 남5, 남동6, 동7, 북동8.
// 배열 인덱스 순서는 기존 atan2(y,x): 동, 북동, 북, 북서, 서, 남서, 남, 남동.
const uint8_t TEAM_DIRECTION_CODE[8] = {7, 8, 1, 2, 3, 4, 5, 6};
#endif
