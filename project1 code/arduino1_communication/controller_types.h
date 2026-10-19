#ifndef CONTROLLER_TYPES_H
#define CONTROLLER_TYPES_H
#include <stdint.h>

// 1로 바꾸면 USB(Serial)로 디버그 문자열을 출력한다. Serial(D0/D1)은 2번과 공유하므로
// 2번을 연결한 실제 동작에서는 반드시 0으로 둔다. (1로 쓸 때 시리얼 모니터는 9600)
#ifndef A1_USB_DEBUG
#define A1_USB_DEBUG 0
#endif

// 숫자는 프로그램 내부에서만 쓰는 이름이다. 실제 통신 코드값은 팀이 정한다.
#define MODE_MANUAL 0
#define MODE_AUTO 1
#define USER_NONE 0
#define USER_AUTO 1
#define USER_MANUAL 2
#define USER_FIRE 3
#define USER_STOP 4
#define USER_SCAN_STOP 5
#define USER_SCAN_SLOW 6
#define USER_SCAN_FAST 7
#define USER_SCAN_TRACK 8
#define USER_SCAN_HOME 9
#define STATE_WAIT 0
#define STATE_AIMING 1
#define STATE_FIRING 2
#define STATE_STOP 3
#define STATE_ERROR 4
#define TURRET_NONE 0
#define TURRET_A 1
#define TURRET_B 2
#define TURRET_BOTH 3
#define GUN_SAFE 1
#define GUN_ENABLE 2
#define GUN_AIM 3
#define GUN_FIRE 4
#define SCAN_SLOW 1
#define SCAN_FAST 2
#define SCAN_TRACK 3
#define SCAN_WAIT 4
#define SCAN_STOP 5
#define SCAN_HOME 6
#define FAULT_CONFIG 1
#define FAULT_SPI_FRAME 2
#define FAULT_SCANNER 4
#define FAULT_I2C 8
#define FAULT_TURRET_STATUS 16
#define FAULT_UART 32
#define TURRET_FEEDBACK_FULL 0
#define TURRET_FEEDBACK_ANGLES 1
#define MAX_TARGETS 3       // 실제 RadarWire v1의 표적 슬롯 수
#define SPI_RX_MAX 48
#define SPI_TX_MAX 48
#define UART_TX_MAX 32
#define I2C_PACKET_MAX 32

// struct는 관련 변수들을 하나로 묶은 것이다.
// 통신 바이트 배열을 그대로 struct에 복사하지 않고, 합의한 형식으로 해석한다.
typedef struct {
    uint8_t id;
    int16_t x, y;
    int16_t vx, vy;          // Cartesian 수신 속도. 원기둥 수신 시에는 0
    uint16_t r_mm, theta_cdeg; // 북0 / 반시계 +, 1 cdeg = 0.01도
    int16_t vr_mm_s, vtheta_mm_s; // 반지름 방향 / 접선 방향 속도 (각속도 아님)
    uint8_t polar_valid;     // 원기둥 좌표계의 평면 데이터가 있으면 1
    uint8_t velocity_valid;  // 이번 프로파일에서는 vr/vtheta의 유효 여부
    uint8_t quality;
    uint8_t age_scans;
    uint8_t valid;
    uint8_t fresh;           // UNO3가 age=0 / STALE 해제로 보고한 관측
    uint32_t updated_ms;     // 1번이 관측 레코드 변화를 확인한 시각 (측정 시각 아님)
} Target;

typedef struct {
    uint8_t count;           // 아래 배열의 항목 수
    uint8_t valid_count;     // 이 프레임 항목 중 유효한 수. 원시 전체 수와 의미 구분
    uint8_t scan_mode;
    uint8_t scanner_state;    // byte 42: CALIBRATION/IDLE/MOVING/FAULT
    uint8_t status_flags;     // byte 4 원본 비트
    uint8_t hall_strikes, selected_target_id, accepted_cmd_seq, last_error;
    uint8_t sample_times_valid;
    uint8_t scanner_ready;
    uint8_t sequence_valid, sequence; // 전송 번호이며 측정 번호가 아님
    Target targets[MAX_TARGETS];
} DetectionFrame;

typedef struct {
    uint8_t command;
    uint8_t target_id;
    uint16_t command_seq;    // 같은 요청의 반복 전송을 구별하는 번호
} ScannerCommand;

typedef struct {
    uint8_t command;
    uint8_t turret_mask;     // A=1, B=2, 둘 다=3
    uint8_t target_id;
    int16_t x, y;           // 내부 표시/진단용 중심 상대 좌표
    uint16_t r_mm, theta_cdeg; // AIM은 이 값을 사용. 북0, 단위 0.01도
    uint16_t aim_seq;        // 어떤 좌표에 대한 조준인지 구별
    uint16_t fire_seq;       // 1번 내부 발행 번호. 현재 4번 규격으로는 전송하지 않음
} TurretCommand;

typedef struct {
    uint8_t enabled_mask;
    uint8_t feedback_kind;  // 0=시험용 확장 상태, 1=실제 4번의 8바이트 각도/플래그 응답
    uint32_t received_ms;
    uint8_t aimed_mask;
    uint8_t firing_mask;
    uint8_t target_valid_mask;
    uint8_t id_a, id_b;
    uint16_t aim_seq_a, aim_seq_b;
    int16_t angle_a, angle_b; // 서보 각도
    uint16_t bearing_a_ddeg, bearing_b_ddeg; // 4번의 방위각, 0.1도
    uint8_t error_a, error_b;
    uint16_t accepted_fire_seq;
    uint16_t completed_fire_seq; // 확장 응답용 예약. 현재 4번은 완료 번호를 보내지 않음
} TurretReport;

typedef struct {
    uint8_t mode, state, direction;
    uint8_t target_valid, target_id, turret_mask;
    uint16_t target_r_mm, target_theta_cdeg;
    uint8_t enabled_mask, aimed_mask, firing_mask;
    uint8_t turret_status_valid; // 0이면 상태 불명. '사격하지 않음'과 구별
    uint8_t approval_rejected;
    uint16_t faults, aim_seq;
} CentralReport;

// 통신 함수: 본체 .ino에 구현한다. 판단 코드는 이 함수들을 호출한다.
uint16_t link_uart_collect(void);
uint8_t link_uart_send(const CentralReport *report);
uint8_t link_spi_prepare(const ScannerCommand *command);
uint8_t link_i2c_send(const TurretCommand *command);
uint8_t link_i2c_read(TurretReport *report);
#endif
