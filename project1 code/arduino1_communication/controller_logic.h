#ifndef CONTROLLER_LOGIC_H
#define CONTROLLER_LOGIC_H
#include <string.h>
#include <math.h>

// ============================================================
// 중앙 제어 상태: 센서 계산/서보 제어는 여기서 하지 않는다.
// ============================================================
uint8_t current_mode = MODE_MANUAL;
uint8_t system_state = STATE_STOP;
uint8_t stopped = 1;                // 시작은 정지. 전체 설정 완료 후 AUTO/MANUAL로 재개
uint16_t faults = FAULT_CONFIG;
Target targets[MAX_TARGETS];
// Arrival generation belongs to target identity, not coordinate-update time.
uint32_t target_arrival[MAX_TARGETS] = {};
uint32_t target_arrival_counter = 0;
uint8_t selected_valid = 0, selected_id = 0, selected_mask = TURRET_NONE;
int16_t selected_x = 0, selected_y = 0;
uint16_t selected_r_mm = 0, selected_theta_cdeg = 0;
uint16_t aim_seq = 0, last_sent_aim_seq = 0, fire_seq = 0;
uint8_t aim_was_sent = 0, aim_urgent = 0, enable_pending = 0;
uint8_t manual_approval = 0, approval_rejected = 0, shot_latched = 0;
uint8_t approval_id = 0;
uint16_t approval_aim_seq = 0;
uint8_t safe_pending = 1, status_dirty = 1;
uint8_t last_status_firing = 0;
uint32_t last_aim_send_ms = 0, last_status_send_ms = 0;
uint32_t last_turret_poll_ms = 0, last_safe_attempt_ms = 0;
uint8_t safe_attempted = 0;
uint8_t turret_poll_attempted = 0;
uint8_t scanner_seen = 0, scanner_ready = 0, scanner_state = 0, scan_mode = 0;
uint8_t scanner_status_flags = 0, scanner_hall_strikes = 0, scanner_selected_id = 0;
uint8_t scanner_accepted_cmd_seq = 0xFF, scanner_last_error = 0;
uint8_t scanner_sequence_seen = 0, last_scanner_sequence = 0;
uint32_t last_scanner_ms = 0, last_turret_ms = 0;
uint8_t turret_seen = 0;
TurretReport turret_report;
ScannerCommand scanner_command = {SCAN_STOP, 0, 0};

uint8_t controller_settings_ready(void) {
    if (!UART_PROTOCOL_READY || !SPI_PROTOCOL_READY ||
        !TURRET_PROTOCOL_READY || !CONTROL_POLICY_READY) return 0;
    if (!TARGET_TIMEOUT_MS || !SCANNER_TIMEOUT_MS || !TURRET_TIMEOUT_MS) return 0;
    if (!TURRET_STATUS_POLL_MS || !TURRET_REPORT_BYTES ||
        TURRET_REPORT_BYTES > I2C_PACKET_MAX) return 0;
    if (DETECTION_LIST_KIND != 1 && DETECTION_LIST_KIND != 2) return 0;
    for (uint8_t i = 0; i < 8; i++) {
        if (TEAM_DIRECTION_CODE[i] < 1 || TEAM_DIRECTION_CODE[i] > 8) return 0;
        for (uint8_t j = 0; j < i; j++)
            if (TEAM_DIRECTION_CODE[i] == TEAM_DIRECTION_CODE[j]) return 0;
    }
    return 1;
}

uint8_t time_valid(uint8_t seen, uint32_t now, uint32_t stamp, uint32_t limit) {
    return seen && limit > 0 && (uint32_t)(now - stamp) < limit;
}

int8_t find_target(uint8_t id) {
    for (uint8_t i = 0; i < MAX_TARGETS; i++)
        if (targets[i].valid && targets[i].id == id) return i;
    return -1;
}

void controller_request_safe(void) {
    manual_approval = 0;
    if (!safe_pending) safe_attempted = 0; // 새 정지/상실은 재시도 주기를 기다리지 않기
    if (!safe_pending && selected_valid) {
        aim_seq++;               // 안전 모드 전의 조준 완료를 다시 사용하지 않기
        aim_was_sent = 0;
        aim_urgent = 1;
        enable_pending = 1;
    }
    safe_pending = 1;
}

void controller_drop_target(void) {
    if (selected_valid) {
        selected_valid = 0;
        selected_mask = TURRET_NONE;
        aim_seq++;
        aim_was_sent = 0;
        shot_latched = 0;
        aim_urgent = 0;
        enable_pending = 0;
        status_dirty = 1;
        controller_request_safe();
    }
    manual_approval = 0;
}

void controller_stop(void) {
    stopped = 1;
    for (uint8_t i = 0; i < MAX_TARGETS; i++) targets[i].valid = 0;
    controller_drop_target();
    controller_request_safe();
    safe_attempted = 0;
    // 비상 STOP은 포탑뿐 아니라 Arduino 3 스캐너도 STOP/IDLE로 전환한다.
    team_scanner_request(SCAN_STOP, 0);
    status_dirty = 1;
}

uint8_t controller_aim_ready(uint32_t now) {
    if (stopped || faults || safe_pending || !selected_valid || !aim_was_sent) return 0;
    if (last_sent_aim_seq != aim_seq) return 0;
    if (!time_valid(turret_seen, now, last_turret_ms, TURRET_TIMEOUT_MS)) return 0;
    if (turret_report.feedback_kind == TURRET_FEEDBACK_ANGLES)
        return team_turret_pose_ready(selected_mask, selected_id, aim_seq,
                                     selected_r_mm, selected_theta_cdeg, &turret_report);
    if ((turret_report.enabled_mask & selected_mask) != selected_mask) return 0;
    if ((turret_report.aimed_mask & selected_mask) != selected_mask) return 0;
    if ((turret_report.target_valid_mask & selected_mask) != selected_mask) return 0;
    if (selected_mask & TURRET_A) {
        if (turret_report.error_a || turret_report.id_a != selected_id ||
            turret_report.aim_seq_a != aim_seq) return 0;
    }
    if (selected_mask & TURRET_B) {
        if (turret_report.error_b || turret_report.id_b != selected_id ||
            turret_report.aim_seq_b != aim_seq) return 0;
    }
    return 1;
}

// 같은 UART 수신 묶음에 STOP이 있으면 나머지 명령보다 먼저 처리한다.
void controller_process_commands(uint16_t commands, uint32_t now) {
    if (commands & ((uint16_t)1 << USER_STOP)) {
        controller_stop();
        return;
    }
    uint8_t command = USER_NONE;
    if (commands & ((uint16_t)1 << USER_MANUAL)) command = USER_MANUAL;
    else if (commands & ((uint16_t)1 << USER_AUTO)) command = USER_AUTO;
    if (command != USER_NONE) {
        uint8_t next_mode = (command == USER_AUTO) ? MODE_AUTO : MODE_MANUAL;
        if (current_mode != next_mode) {
            current_mode = next_mode;
            controller_request_safe();
            status_dirty = 1;
        }
        manual_approval = 0;
        if (stopped && controller_settings_ready() && team_allow_resume(command)) {
            stopped = 0;
            status_dirty = 1;
        }
    }
    uint8_t scanner_request = USER_NONE;
    if (commands & ((uint16_t)1 << USER_SCAN_STOP)) scanner_request = USER_SCAN_STOP;
    else if (commands & ((uint16_t)1 << USER_SCAN_SLOW)) scanner_request = USER_SCAN_SLOW;
    else if (commands & ((uint16_t)1 << USER_SCAN_FAST)) scanner_request = USER_SCAN_FAST;
    else if (commands & ((uint16_t)1 << USER_SCAN_TRACK)) scanner_request = USER_SCAN_TRACK;
    else if (commands & ((uint16_t)1 << USER_SCAN_HOME)) scanner_request = USER_SCAN_HOME;
    if (scanner_request != USER_NONE) {
        uint8_t scanner_mode = SCAN_STOP;
        if (scanner_request == USER_SCAN_SLOW) scanner_mode = SCAN_SLOW;
        else if (scanner_request == USER_SCAN_FAST) scanner_mode = SCAN_FAST;
        else if (scanner_request == USER_SCAN_TRACK) scanner_mode = SCAN_TRACK;
        else if (scanner_request == USER_SCAN_HOME) scanner_mode = SCAN_HOME;
        uint8_t target_id = 0;
        uint8_t accepted = 1;
        if (scanner_mode == SCAN_TRACK) {
            // 2번이 보낸 ID가 1번의 최신 유효 표적 목록에 있을 때만 추적한다.
            // ID=0을 3번에 보내면 특정 표적이 아니라 순환 추적이 되므로 보내지 않는다.
            target_id = team_uart_track_id();
            int8_t index = target_id ? find_target(target_id) : -1;
            accepted = index >= 0 &&
                time_valid(1, now, targets[index].updated_ms, TARGET_TIMEOUT_MS);
#if A1_USB_DEBUG
            if (!accepted) { Serial.print(F("DBG|TRACK_REJECT|id=")); Serial.println(target_id); }
#endif
        }
        if (accepted) {
            team_scanner_request(scanner_mode, target_id);
            team_scanner_next(&scanner_command);
        }
        status_dirty = 1;
    }

    if (commands & ((uint16_t)1 << USER_FIRE)) {
        int8_t index = find_target(selected_id);
        uint8_t fresh = index >= 0 &&
            time_valid(1, now, targets[index].updated_ms, TARGET_TIMEOUT_MS);
        if (current_mode == MODE_MANUAL && selected_valid && fresh &&
            controller_aim_ready(now) && !shot_latched && !turret_report.firing_mask) {
            manual_approval = 1;
            approval_id = selected_id;
            approval_aim_seq = aim_seq;
            approval_rejected = 0;
        } else {
            manual_approval = 0;
            approval_rejected = 1;
        }
        status_dirty = 1;
    }
}

// 상태 비트는 최신 정상 응답에서 반영한다. 이 함수는 측정 유효 시각을 갱신하지 않는다.
void controller_apply_scanner_status(const DetectionFrame *frame) {
    team_scanner_observe(frame);
#if A1_USB_DEBUG
    static uint8_t debug_seen = 0;
    static uint8_t debug_mode = 0, debug_state = 0, debug_ack = 0;
    static uint8_t debug_id = 0, debug_error = 0, debug_flags = 0;
    if (!debug_seen || debug_mode != frame->scan_mode ||
        debug_state != frame->scanner_state ||
        debug_ack != frame->accepted_cmd_seq ||
        debug_id != frame->selected_target_id ||
        debug_error != frame->last_error ||
        debug_flags != frame->status_flags) {
        Serial.print(F("DBG|A3|mode=")); Serial.print(frame->scan_mode);
        Serial.print(F("|state=")); Serial.print(frame->scanner_state);
        Serial.print(F("|ack=")); Serial.print(frame->accepted_cmd_seq);
        Serial.print(F("|id=")); Serial.print(frame->selected_target_id);
        Serial.print(F("|err=")); Serial.print(frame->last_error);
        Serial.print(F("|flags=0x"));
        if (frame->status_flags < 0x10) Serial.print('0');
        Serial.println(frame->status_flags, HEX);
        debug_seen = 1;
        debug_mode = frame->scan_mode;
        debug_state = frame->scanner_state;
        debug_ack = frame->accepted_cmd_seq;
        debug_id = frame->selected_target_id;
        debug_error = frame->last_error;
        debug_flags = frame->status_flags;
    }
#endif
    if (scanner_status_flags != frame->status_flags || scanner_last_error != frame->last_error)
        status_dirty = 1;
    scanner_status_flags = frame->status_flags;
    scanner_hall_strikes = frame->hall_strikes;
    scanner_selected_id = frame->selected_target_id;
    scanner_accepted_cmd_seq = frame->accepted_cmd_seq;
    scanner_last_error = frame->last_error;
    if (scanner_ready != frame->scanner_ready || scanner_state != frame->scanner_state ||
        scan_mode != frame->scan_mode) status_dirty = 1;
    scanner_ready = frame->scanner_ready;
    scanner_state = frame->scanner_state;
    scan_mode = frame->scan_mode;
    if (!scanner_ready) controller_request_safe();
}

// CRC/형식/중복 ID/표적 수/좌표 검사가 끝난 뒤에만 저장한다.
uint8_t controller_accept_detection(const uint8_t *data, uint8_t length,
                                    uint32_t now) {
    DetectionFrame frame;
    memset(&frame, 0, sizeof(frame));
    uint8_t ok = SPI_PROTOCOL_READY && team_spi_crc_ok(data, length) &&
                 team_decode_detection(data, length, &frame);
    if (!ok || frame.count > MAX_TARGETS || frame.valid_count > frame.count ||
        (DETECTION_LIST_KIND != 1 && DETECTION_LIST_KIND != 2)) {
        faults |= FAULT_SPI_FRAME;
        controller_request_safe();
        return 0;
    }
    uint8_t valid_count = 0;
    for (uint8_t i = 0; i < frame.count; i++) {
        for (uint8_t j = 0; j < i; j++)
            if (frame.targets[i].id == frame.targets[j].id) ok = 0;
        if (frame.targets[i].valid) {
            valid_count++;
            if (!team_xy_valid(&frame.targets[i])) ok = 0;
        }
    }
    if (!ok || valid_count != frame.valid_count) {
        faults |= FAULT_SPI_FRAME;
        controller_request_safe();
        return 0;
    }

    if (frame.sequence_valid && scanner_sequence_seen) {
        uint8_t advance = (uint8_t)(frame.sequence - last_scanner_sequence);
        // 전송 seq 중복: 상태만 반영하고 표적 시각은 유지한다.
        if (!advance) {
            controller_apply_scanner_status(&frame);
            return 1;
        }
        // 정상 연결 중 역순/오래된 번호는 무시. 긴 단절 뒤 다른 번호는 재시작으로 수용.
        if (advance >= 128 && time_valid(scanner_seen, now, last_scanner_ms,
                                        SCANNER_TIMEOUT_MS)) return 1;
    }

    team_observe_samples(&frame, data, now);
    Target next[MAX_TARGETS];
    memcpy(next, targets, sizeof(next));
    if (DETECTION_LIST_KIND == 1)
        for (uint8_t i = 0; i < MAX_TARGETS; i++) next[i].valid = 0;
    for (uint8_t i = 0; i < frame.count; i++) {
        Target value = frame.targets[i];
        int8_t old = find_target(value.id), slot = -1;
        for (uint8_t j = 0; j < MAX_TARGETS; j++)
            if (next[j].valid && next[j].id == value.id) slot = j;
        if (!value.valid) {
            if (slot >= 0) next[slot].valid = 0;
            continue;
        }
        if (!value.fresh && old < 0) continue;   // 신규 탐지가 아닌 재전송은 새 표적 아님
        // age_scans>0의 예측/재전송 값으로 마지막 실제 좌표를 바꾸지 않는다.
        if (!value.fresh) {
            value = targets[old];
            value.fresh = 0;
        }
        if (slot < 0)
            for (uint8_t j = 0; j < MAX_TARGETS; j++)
                if (!next[j].valid) { slot = j; break; }
        if (slot < 0) {
            faults |= FAULT_SPI_FRAME;          // 저장 상한을 넘으면 부분 반영하지 않기
            controller_request_safe();
            return 0;
        }
        if (!frame.sample_times_valid)
            value.updated_ms = value.fresh ? now : targets[old].updated_ms;
        next[slot] = value;
    }
    uint32_t next_arrival[MAX_TARGETS] = {};
    uint8_t has_new_target = 0;
    for (uint8_t i = 0; i < MAX_TARGETS; ++i) {
        if (!next[i].valid) continue;
        int8_t old = find_target(next[i].id);
        if (old >= 0 && time_valid(1, now, targets[old].updated_ms, TARGET_TIMEOUT_MS))
            next_arrival[i] = target_arrival[old];
        else has_new_target = 1;
    }
    // Targets first seen in the same snapshot share a generation.
    if (has_new_target) {
        ++target_arrival_counter;
        for (uint8_t i = 0; i < MAX_TARGETS; ++i) {
            if (!next[i].valid) continue;
            int8_t old = find_target(next[i].id);
            if (old < 0 || !time_valid(1, now, targets[old].updated_ms, TARGET_TIMEOUT_MS))
                next_arrival[i] = target_arrival_counter;
        }
    }
    memcpy(target_arrival, next_arrival, sizeof(target_arrival));
    memcpy(targets, next, sizeof(targets));
    scanner_seen = 1;
    last_scanner_ms = now;
    if (frame.sequence_valid) {
        scanner_sequence_seen = 1;
        last_scanner_sequence = frame.sequence;
    }
    controller_apply_scanner_status(&frame);
    faults &= ~FAULT_SPI_FRAME;
    return 1;
}

void controller_refresh_faults(uint32_t now) {
    uint16_t old = faults;
    if (controller_settings_ready()) faults &= ~FAULT_CONFIG;
    else faults |= FAULT_CONFIG;
    if (time_valid(scanner_seen, now, last_scanner_ms, SCANNER_TIMEOUT_MS) &&
        scanner_ready) faults &= ~FAULT_SCANNER;
    else faults |= FAULT_SCANNER;
    if (time_valid(turret_seen, now, last_turret_ms, TURRET_TIMEOUT_MS) &&
        !turret_report.error_a && !turret_report.error_b)
        faults &= ~FAULT_TURRET_STATUS;
    else faults |= FAULT_TURRET_STATUS;
    if (team_uart_link_ok(now)) faults &= ~FAULT_UART;
    else faults |= FAULT_UART;
    for (uint8_t i = 0; i < MAX_TARGETS; i++) {
        if (targets[i].valid &&
            !time_valid(1, now, targets[i].updated_ms, TARGET_TIMEOUT_MS))
            targets[i].valid = 0;
    }
    if (old != faults) status_dirty = 1;
    if (faults && !old) controller_request_safe();
}

uint8_t coordinate_direction(int16_t x, int16_t y) {
    if (x == 0 && y == 0) return 0;  // 원점 방향의 처리도 팀에서 확인
    float angle = atan2f((float)y, (float)x) * 57.2957795f;
    if (angle < 0) angle += 360.0f;
    uint8_t sector = ((uint16_t)((angle + 22.5f) / 45.0f)) % 8;
    return TEAM_DIRECTION_CODE[sector];
}

void controller_select_target(uint32_t now) {
    if (stopped) { controller_drop_target(); return; }
    int8_t best = -1;
    float best_priority = 0;
    for (uint8_t i = 0; i < MAX_TARGETS; i++) {
        if (!targets[i].valid ||
            !time_valid(1, now, targets[i].updated_ms, TARGET_TIMEOUT_MS)) continue;
        uint8_t mask = team_turret_mask(&targets[i]);
        if (mask == TURRET_NONE || mask > TURRET_BOTH) continue;
        float score = team_priority(&targets[i], current_mode);
        if (score <= 0) continue; // Existing candidate eligibility is unchanged.
        uint8_t newer = current_mode == MODE_AUTO && best >= 0 &&
            (int32_t)(target_arrival[i] - target_arrival[best]) > 0;
        uint8_t same_arrival = current_mode != MODE_AUTO || best < 0 ||
            target_arrival[i] == target_arrival[best];
        if (best < 0 || newer ||
            (same_arrival && (score > best_priority ||
             (score == best_priority && selected_valid && targets[i].id == selected_id)))) {
            best = i;
            best_priority = score;
        }
    }
    if (best < 0) { controller_drop_target(); return; }
    Target *target = &targets[best];
    uint8_t mask = team_turret_mask(target);
    uint8_t identity_changed = !selected_valid || selected_id != target->id;
    uint8_t changed = identity_changed || selected_mask != mask;
    if (changed) {
        controller_request_safe();
        selected_valid = 1;
        selected_id = target->id;
        selected_mask = mask;
        if (identity_changed) shot_latched = 0; // 포탑 변경만으로 추가 사격을 허용하지 않기
        enable_pending = 1;
        aim_urgent = 1;
        status_dirty = 1;
    }
    // Compare against the last commanded aim, not the previous noisy sample.
    // Same ID + <=10mm AND <=5deg: keep aim_seq and any manual approval.
    uint16_t radial_error = selected_r_mm > target->r_mm ?
        selected_r_mm - target->r_mm : target->r_mm - selected_r_mm;
    uint16_t angular_error = selected_theta_cdeg > target->theta_cdeg ?
        selected_theta_cdeg - target->theta_cdeg : target->theta_cdeg - selected_theta_cdeg;
    if (angular_error > 18000U) angular_error = 36000U - angular_error;
    if (changed || radial_error > 10U || angular_error > 500U) {
        if (coordinate_direction(selected_x, selected_y) !=
            coordinate_direction(target->x, target->y)) status_dirty = 1;
        selected_x = target->x;
        selected_y = target->y;
        selected_r_mm = target->r_mm;
        selected_theta_cdeg = target->theta_cdeg;
        aim_seq++;
        aim_was_sent = 0;
        manual_approval = 0;        // 승인 후 표적/좌표가 바뀌면 승인을 다시 받기
    }
}

void controller_update_scanner(void) {
    // 기본 자동 SEARCH_SLOW 또는 UI가 마지막으로 요청한 스캐너 모드를 준비한다.
    team_scanner_next(&scanner_command);
    link_spi_prepare(&scanner_command); // 전송 시작은 3번, 1번은 응답만 준비.
}

TurretCommand make_turret_command(uint8_t command) {
    TurretCommand result;
    memset(&result, 0, sizeof(result));
    result.command = command;
    result.turret_mask = selected_valid ? selected_mask : TURRET_BOTH;
    result.target_id = selected_id;
    result.x = selected_x; result.y = selected_y;
    result.r_mm = selected_r_mm; result.theta_cdeg = selected_theta_cdeg;
    result.aim_seq = aim_seq; result.fire_seq = fire_seq;
    if (command == GUN_SAFE) result.turret_mask = TURRET_BOTH;
    return result;
}

void controller_i2c_failed(void) {
    faults |= FAULT_I2C;
    turret_seen = 0;
    status_dirty = 1;
    controller_request_safe();
}

void controller_service_i2c(uint32_t now) {
    if (!TURRET_PROTOCOL_READY || !TURRET_REPORT_BYTES ||
        TURRET_REPORT_BYTES > I2C_PACKET_MAX) return;

    if ((stopped || faults || !selected_valid) && turret_seen &&
        (turret_report.enabled_mask || turret_report.firing_mask ||
         (turret_report.feedback_kind == TURRET_FEEDBACK_ANGLES && turret_report.aimed_mask))) safe_pending = 1;
    if (safe_pending && (!safe_attempted ||
        (uint32_t)(now - last_safe_attempt_ms) >= 100UL)) {
        safe_attempted = 1;
        last_safe_attempt_ms = now;
        TurretCommand command = make_turret_command(GUN_SAFE);
        if (link_i2c_send(&command)) safe_pending = 0;
        else controller_i2c_failed();
        return;
    }

    if (!turret_poll_attempted ||
        (uint32_t)(now - last_turret_poll_ms) >= TURRET_STATUS_POLL_MS) {
        turret_poll_attempted = 1;
        last_turret_poll_ms = now;
        TurretReport report;
        memset(&report, 0, sizeof(report));
        if (!link_i2c_read(&report) ||
            ((report.enabled_mask | report.aimed_mask | report.firing_mask |
              report.target_valid_mask) & ~TURRET_BOTH)) {
            controller_i2c_failed();
            return;
        }
        turret_report = report;
        turret_seen = 1;
        last_turret_ms = now;
        faults &= ~FAULT_I2C;
        if (!controller_aim_ready(now)) manual_approval = 0;
        if (shot_latched && !turret_report.firing_mask &&
            turret_report.completed_fire_seq == fire_seq &&
            team_rearm_fire(current_mode, &turret_report)) shot_latched = 0;
    }
    controller_refresh_faults(now);
    if (stopped || faults || safe_pending || !selected_valid) return;

    if (enable_pending) {
        TurretCommand command = make_turret_command(GUN_ENABLE);
        if (link_i2c_send(&command)) enable_pending = 0;
        else controller_i2c_failed();
        return;
    }
    if (aim_urgent || (uint32_t)(now - last_aim_send_ms) >= TARGET_SEND_PERIOD_MS) {
        TurretCommand command = make_turret_command(GUN_AIM);
        if (link_i2c_send(&command)) {
            last_sent_aim_seq = aim_seq;
            aim_was_sent = 1;
            aim_urgent = 0;
            last_aim_send_ms = now;
        } else controller_i2c_failed();
        return;
    }

    // 모의 사격 바로 전 UART를 한 번 더 확인하여 대기 중 STOP을 우선 처리.
    controller_process_commands(link_uart_collect(), now);
    if (safe_pending) {
        safe_attempted = 1;
        last_safe_attempt_ms = now;
        TurretCommand command = make_turret_command(GUN_SAFE);
        if (link_i2c_send(&command)) safe_pending = 0;
        else controller_i2c_failed();
        return;
    }
    if (!controller_aim_ready(now) || shot_latched || turret_report.firing_mask ||
        scanner_command.command == SCAN_HOME) return;
    int8_t index = find_target(selected_id);
    if (index < 0 ||
        !time_valid(1, now, targets[index].updated_ms, TARGET_TIMEOUT_MS)) return;
    uint8_t allowed = 0;
    if (current_mode == MODE_MANUAL)
        allowed = manual_approval && approval_id == selected_id && approval_aim_seq == aim_seq;
    else if (current_mode == MODE_AUTO)
        allowed = team_dangerous(&targets[index]) && team_auto_fire(&targets[index], &turret_report);
    if (!allowed) return;

    manual_approval = 0;           // 승인은 한 번만 소비
    shot_latched = 1;              // 불확실한 응답에도 같은 표적에 재발행하지 않기
    fire_seq++;
    TurretCommand command = make_turret_command(GUN_FIRE);
    if (!link_i2c_send(&command)) controller_i2c_failed();
    // 여기서 STATE_FIRING으로 바꾸지 않는다. 실제 상태 읽기로만 확인한다.
}

void controller_update_state(uint32_t now) {
    uint8_t old = system_state;
    if (stopped) system_state = STATE_STOP;
    else if (faults) system_state = STATE_ERROR;
    else if (time_valid(turret_seen, now, last_turret_ms, TURRET_TIMEOUT_MS) &&
             turret_report.firing_mask) system_state = STATE_FIRING;
    else if (selected_valid && !controller_aim_ready(now)) system_state = STATE_AIMING;
    else system_state = STATE_WAIT;
    if (old != system_state) status_dirty = 1;
}

// 2번 LCD에 보여 줄 표적: 선택된 표적이 없으면 가장 가까운 유효 표적.
// (AUTO에서 위험하지 않은 표적도 2번에서 보고 TRACK을 고를 수 있게 한다.)
int8_t controller_display_target(uint32_t now) {
    int8_t best = -1;
    for (uint8_t i = 0; i < MAX_TARGETS; i++) {
        if (!targets[i].valid || !team_xy_valid(&targets[i]) ||
            !time_valid(1, now, targets[i].updated_ms, TARGET_TIMEOUT_MS)) continue;
        if (best < 0 || targets[i].r_mm < targets[best].r_mm) best = i;
    }
    return best;
}

void controller_send_status(uint32_t now) {
    // 정지/오류 상태가 그대로여도 실제 출력의 ON/OFF는 즉시 보고한다.
    uint8_t firing = time_valid(turret_seen, now, last_turret_ms, TURRET_TIMEOUT_MS) &&
                     turret_report.firing_mask;
    if (firing != last_status_firing) status_dirty = 1;
    if (!status_dirty &&
        (uint32_t)(now - last_status_send_ms) < UART_STATUS_PERIOD_MS) return;
    CentralReport report;
    memset(&report, 0, sizeof(report));
    report.mode = current_mode; report.state = system_state;
    report.target_valid = selected_valid;
    report.target_id = selected_id; report.turret_mask = selected_mask;
    report.target_r_mm = selected_valid ? selected_r_mm : 0;
    report.target_theta_cdeg = selected_valid ? selected_theta_cdeg : 0;
    if (!selected_valid && !stopped) {
        int8_t shown = controller_display_target(now);
        if (shown >= 0) {
            report.target_valid = 1;
            report.target_id = targets[shown].id;
            report.target_r_mm = targets[shown].r_mm;
            report.target_theta_cdeg = targets[shown].theta_cdeg;
        }
    }
    report.direction = selected_valid ? coordinate_direction(selected_x, selected_y) : 0;
    report.faults = faults; report.aim_seq = aim_seq;
    report.approval_rejected = approval_rejected;
    if (time_valid(turret_seen, now, last_turret_ms, TURRET_TIMEOUT_MS)) {
        report.turret_status_valid = 1;
        report.enabled_mask = turret_report.enabled_mask;
        report.aimed_mask = turret_report.aimed_mask;
        report.firing_mask = turret_report.firing_mask;
    }
    if (link_uart_send(&report)) {
        status_dirty = 0;
        last_status_firing = firing;
        last_status_send_ms = now;
    }
}
#endif
