#ifndef ARDUINO1_UI_UART_H
#define ARDUINO1_UI_UART_H

#include "controller_types.h"
#include "Protocol.h"

// Arduino 2가 실제 사용하는 3바이트 명령 / 7바이트 표적 상태 규격.
// 숫자를 ASCII 문자열로 보내지 않는다.
namespace Arduino2Link {

const uint32_t FRAME_TIMEOUT_MS = 100UL;

inline uint8_t decodeCommand(uint8_t wire_command) {
    switch (wire_command) {
        case TurretProtocol::CMD_AUTO_MODE:   return USER_AUTO;
        case TurretProtocol::CMD_MANUAL_MODE: return USER_MANUAL;
        case TurretProtocol::CMD_FIRE:        return USER_FIRE;
        case TurretProtocol::CMD_SCAN_STOP:   return USER_SCAN_STOP;
        case TurretProtocol::CMD_SEARCH_SLOW: return USER_SCAN_SLOW;
        case TurretProtocol::CMD_SEARCH_FAST: return USER_SCAN_FAST;
        case TurretProtocol::CMD_TRACK_TARGET:return USER_SCAN_TRACK;
        case TurretProtocol::CMD_REHOME:      return USER_SCAN_HOME;
        case TurretProtocol::CMD_STOP:        return USER_STOP;
        default:                             return USER_NONE;
    }
}

// 일반 명령: [02][CMD][03] 3바이트. TRACK만: [02][33][표적 ID][03] 4바이트.
// ID는 위치로 읽으므로 0x02/0x03이어도 STX/ETX로 보지 않는다.
struct CommandReceiver {
    enum Step : uint8_t { WAIT_START, READ_COMMAND, READ_TARGET_ID, READ_END };
    Step step = WAIT_START;
    uint8_t pending = USER_NONE;
    uint8_t pending_id = 0;
    uint8_t track_id = 0;      // 마지막으로 완성된 TRACK 명령의 표적 ID
    uint32_t last_byte_ms = 0;

    void reset() {
        step = WAIT_START;
        pending = USER_NONE;
        pending_id = 0;
    }

    uint8_t push(uint8_t data, uint32_t now) {
        if (step != WAIT_START &&
            (uint32_t)(now - last_byte_ms) > FRAME_TIMEOUT_MS) reset();
        last_byte_ms = now;

        switch (step) {
            case WAIT_START:
                if (data == TurretProtocol::STX) step = READ_COMMAND;
                break;
            case READ_COMMAND:
                pending = decodeCommand(data);
                if (pending == USER_SCAN_TRACK) step = READ_TARGET_ID;
                else if (pending != USER_NONE) step = READ_END;
                else step = data == TurretProtocol::STX ? READ_COMMAND : WAIT_START;
                break;
            case READ_TARGET_ID:
                pending_id = data;      // 데이터 위치: 0x02/0x03도 ID로 받는다
                step = READ_END;
                break;
            case READ_END:
                if (data == TurretProtocol::ETX) {
                    uint8_t command = pending;
                    if (command == USER_SCAN_TRACK) track_id = pending_id;
                    reset();
                    return command; // 종료 바이트까지 받은 명령만 실행 대상이다.
                }
                reset();
                if (data == TurretProtocol::STX) step = READ_COMMAND;
                break;
        }
        return USER_NONE;
    }
};

inline uint8_t encodeStatus(const CentralReport *report, uint8_t *out) {
    if (!report || !out) return 0;
    uint8_t id = report->target_valid ? report->target_id : 0;
    uint16_t r = report->target_valid ? report->target_r_mm : 0;
    uint16_t theta = report->target_valid ? report->target_theta_cdeg : 0;
    if (!TurretProtocol::isValidTarget(id, r, theta)) return 0;

    out[0] = TurretProtocol::STX;
    out[1] = id;
    out[2] = static_cast<uint8_t>(r);
    out[3] = static_cast<uint8_t>(r >> 8);
    out[4] = static_cast<uint8_t>(theta);
    out[5] = static_cast<uint8_t>(theta >> 8);
    out[6] = TurretProtocol::ETX;
    return TurretProtocol::RX_FRAME_SIZE;
}

} // namespace Arduino2Link
#endif
