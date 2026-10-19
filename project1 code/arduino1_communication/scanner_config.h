#ifndef SCANNER_CONFIG_H
#define SCANNER_CONFIG_H

// 제공된 UNO3_SPI_Scanner / RadarProtocol.h의 RadarWire v1.
// MODE0 / MSB first / 125kHz / CS LOW 한 번에 48바이트 양방향.
#define SCANNER_FRAME_BYTES 48
#define SCANNER_SLOT_BYTES 12
#define SCANNER_MAX_TARGETS 3
#define SCANNER_SYNC 0xA5
#define SCANNER_VERSION 0x01

// CRC-8/ATM: poly=0x07, init=0, refin=false, refout=false, xorout=0.
// 계산 범위는 byte 0~46, 저장 위치는 byte 47. 다항식 외 반전 규칙이 바뀌면 함수도 수정.
#define SCANNER_CRC_POLYNOMIAL 0x07
#define SCANNER_CRC_INITIAL 0x00
#define SCANNER_CRC_XOR_OUT 0x00

// 표적 슬롯 flags: 별도의 VALID 비트는 없다.
#define SCANNER_VELOCITY_VALID 0x01
#define SCANNER_IN_BOUNDS 0x02
#define SCANNER_OBJECT_STALE 0x04
#define SCANNER_ORIGIN_X_MM 300   // 3번 판 중심 (600x600). 직교 형식일 때만 사용
#define SCANNER_ORIGIN_Y_MM 300
#define SCANNER_STATE_CALIBRATION 0
#define SCANNER_STATE_IDLE 1
#define SCANNER_STATE_MOVING 2
#define SCANNER_STATE_FAULT 3

// Telemetry byte 4: 2026-10-15에 전달받은 실제 정의. 표적 슬롯 flags와 다른 필드다.
#define SCANNER_HALL_HOME_VALID 0x01
#define SCANNER_TOF_READY 0x02
#define SCANNER_MOTOR_HOLD_COMMANDED 0x04 // HOLD 명령 상태, 실제 토크 측정값 아님
#define SCANNER_HALL_ACTIVE 0x08
#define SCANNER_REHOME_PENDING 0x10
#define SCANNER_CYLINDRICAL_RECORDS 0x20 // 1=(r,theta,vr,vtheta), 0=(x,y,vx,vy)
#define SCANNER_TARGET_LOST 0x40
#define SCANNER_COMMAND_LINK_TIMEOUT 0x80

// 수신 mode는 0..3. 4(REHOME)는 송신 명령에만 사용한다.
#define SCANNER_MODE_STOP_IDLE 0
#define SCANNER_MODE_SEARCH_SLOW 1
#define SCANNER_MODE_SEARCH_FAST 2
#define SCANNER_MODE_TRACK_TARGET 3
#define SCANNER_MODE_REHOME 4

// 실제 3번은 정상 명령을 받은 뒤 3초 watchdog을 사용한다.
#ifndef SCANNER_COMMAND_LINK_REQUIRED
#define SCANNER_COMMAND_LINK_REQUIRED 1
#endif

// 동일 레코드 재전송은 이 시간을 연장하지 않는다. 측정 timestamp가 없는 v1의 보수적 제한.
// 저속 왕복 관측 간격은 1초보다 길 수 있어 정지 표적도 만료될 수 있다.
#define TARGET_TIMEOUT_MS 4500UL   // SEARCH_SLOW 왕복(약 4초) 동안 같은 표적 유지. 사라지면 3번 STALE로 제외
#define SCANNER_TIMEOUT_MS 2000UL
#define SCANNER_MAX_RADIUS_MM 1000U
#define SCANNER_MIN_QUALITY 0U          // 품질 점수의 의미가 미정이므로 점수 하한 미적용

// 사용자가 이번 최종본의 시연 기본값으로 승인한 자동 모의 동작 기준.
#ifndef AUTO_DANGER_RADIUS_MM
#define AUTO_DANGER_RADIUS_MM 200U
#endif
#ifndef AUTO_LOOKAHEAD_MS
#define AUTO_LOOKAHEAD_MS 4000UL
#endif

// SPI 규격은 실제 소스와 대조했다. R/T와 실제 포탑 배치는 별도 확인.
#ifndef SCANNER_PROFILE_CONFIRMED
#define SCANNER_PROFILE_CONFIRMED 1
#endif
#ifndef AUTO_PARAMETERS_CONFIRMED
#define AUTO_PARAMETERS_CONFIRMED 1 // 사용자 승인: 시연 R=200mm, T=4초
#endif
#ifndef TURRET_ASSIGNMENT_CONFIRMED
#define TURRET_ASSIGNMENT_CONFIRMED 1 // 제공된 4번 CFG로 도달 가능한 포탑을 계산
#endif
// 실제 배치의 원점/방향은 turret4_protocol.h의 TURRET_WORLD_FRAME_CONFIRMED로 별도 확인.

#endif
