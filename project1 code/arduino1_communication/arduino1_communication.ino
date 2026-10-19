// Arduino 1 중앙 제어기 - UNO R3 / ATmega328P / 16 MHz
// 최종 배정: 2번 UART 양방향, 3번 SPI Master -> 1번 Slave, 4번 I2C 0x20
// ISR에서는 바이트 저장/미리 준비한 응답 적재만 한다.
// CRC 검사, 좌표/모드/승인 판단, UART/I2C 송신은 main()에서 처리한다.
#include <avr/io.h>
#include <avr/interrupt.h>
#include <Arduino.h>
#include <string.h>
#include "controller_types.h"
#include "Protocol.h"
#include "team_agreement.h"

// Arduino 2 UART: 하드웨어 Serial D0=RX, D1=TX (9600). USB 시리얼과 같은 핀이다.
//  - 1번에 업로드할 때는 D0 선(2번 D3에서 오는 선)을 잠깐 뺀다.
//  - Serial에는 2번용 바이너리만 쓴다. 디버그 문자열은 A1_USB_DEBUG=1(2번 미연결 시험)일 때만.

// ============================================================
// SPI: 1번 Slave. SS LOW~HIGH 한 구간을 한 프레임으로 사용한다.
// 3번도 SS를 48바이트 전체 동안 LOW로 유지해야 한다. MODE0, MSB first.
// 1->3: 시작 STOP seq0, 준비 후 SEARCH_SLOW. 동일 응답 반복은 통신 유지용.
// ============================================================
#define BANK_FREE 0
#define BANK_FILLING 1
#define BANK_READY 2
#define BANK_READING 3
#define NO_BANK 255
volatile uint8_t spi_rx_buffer[2][SPI_RX_MAX];
volatile uint8_t spi_rx_state[2] = {BANK_FREE, BANK_FREE};
volatile uint8_t spi_rx_length[2] = {0, 0};
volatile uint16_t spi_rx_order[2] = {0, 0};
volatile uint16_t spi_frame_number = 0;
volatile uint8_t spi_write_bank = NO_BANK, spi_rx_index = 0, spi_overflow = 0;
volatile uint8_t spi_transport_error = 0;
volatile uint8_t spi_frame_active = 0;

volatile uint8_t spi_tx_buffer[2][SPI_TX_MAX];
volatile uint8_t spi_tx_length[2] = {0, 0};
volatile uint8_t spi_published_bank = 0, spi_active_tx_bank = 0, spi_tx_index = 0;
uint16_t spi_encoded_seq = 0;
uint8_t spi_response_prepared = 0;

void spi_slave_init(void) {
    DDRB |= (1 << DDB4);                         // D12 MISO: 응답 출력
    DDRB &= ~((1 << DDB3) | (1 << DDB5) | (1 << DDB2)); // MOSI/SCK/SS 입력
    PORTB |= (1 << PORTB2);                      // SS 내부 풀업: 미연결 시 선택 안 됨
    ScannerCommand initial = {SCAN_STOP, 0, 0};
    uint8_t packet[SPI_TX_MAX];
    uint8_t length = team_scanner_command(&initial, packet);
    for (uint8_t i = 0; i < length; ++i) spi_tx_buffer[0][i] = packet[i];
    spi_tx_length[0] = length;
    spi_published_bank = 0;
    spi_encoded_seq = 0;
    spi_response_prepared = length != 0;
    SPCR = (1 << SPE) | (1 << SPIE);             // Slave, Mode 0, SPI 수신 인터럽트
    SPDR = length ? spi_tx_buffer[0][0] : 0;     // 첫 클럭 전에 0xA5 준비
}

// SoftwareSerial이 AVR의 pin-change ISR을 사용하므로 D10/SS 시작은 첫 SPI
// 완료 인터럽트에서 확인하고, SS HIGH 종료는 프레임 사이 main loop에서 확정한다.
void spi_begin_frame(void) {
    spi_frame_active = 1;
    spi_write_bank = NO_BANK;
    for (uint8_t i = 0; i < 2; i++) {
        if (spi_rx_state[i] == BANK_FREE) {
            spi_write_bank = i;
            spi_rx_state[i] = BANK_FILLING;
            break;
        }
    }
    if (spi_write_bank == NO_BANK) spi_transport_error = 1;
    spi_rx_index = 0;
    spi_overflow = 0;
    spi_active_tx_bank = spi_published_bank;
    spi_tx_index = 0; // 응답 byte 0은 프레임 전에 이미 SPDR에 준비되어 있다.
}

// SPI ISR과 SS ISR에서 함께 쓰는 바이트 저장 함수. 판단/통신 호출 없음.
void spi_store_byte(uint8_t value) {
    if (spi_write_bank != NO_BANK) {
        if (spi_rx_index < SPI_RX_MAX)
            spi_rx_buffer[spi_write_bank][spi_rx_index++] = value;
        else { spi_overflow = 1; spi_transport_error = 1; }
    }
    // 이번 CS 구간에 고정된 48바이트 응답을 순서대로 적재한다.
    if (spi_tx_index < SPI_TX_MAX) spi_tx_index++;
    if (spi_tx_index < spi_tx_length[spi_active_tx_bank])
        SPDR = spi_tx_buffer[spi_active_tx_bank][spi_tx_index];
    else SPDR = 0;                              // 준비된 응답 끝 이후 채움 바이트
}

ISR(SPI_STC_vect) {
    uint8_t rx_byte = SPDR;
    if (!spi_frame_active) spi_begin_frame();
    spi_store_byte(rx_byte);
}

void spi_finish_frame(void) {
    if (!spi_frame_active || !(PINB & (1 << PINB2))) return;
    uint8_t saved = SREG;
    cli();
    if (spi_frame_active && (PINB & (1 << PINB2))) {
        if (spi_write_bank != NO_BANK) {
            uint8_t bank = spi_write_bank;
            if (!spi_overflow && spi_rx_index > 0) {
                spi_rx_length[bank] = spi_rx_index;
                spi_rx_order[bank] = ++spi_frame_number;
                spi_rx_state[bank] = BANK_READY;
            } else {
                spi_rx_state[bank] = BANK_FREE;
            }
        }
        spi_write_bank = NO_BANK;
        spi_frame_active = 0;
        spi_active_tx_bank = spi_published_bank;
        spi_tx_index = 0;
        SPDR = spi_tx_length[spi_active_tx_bank] ?
               spi_tx_buffer[spi_active_tx_bank][0] : 0;
    }
    SREG = saved;
}

// ISR이 다른 버퍼를 쓰는 동안 완성된 버퍼를 복사한다. 긴 복사 중 인터럽트 허용.
uint8_t spi_take_frame(uint8_t *out) {
    spi_finish_frame();
    uint8_t saved = SREG;
    cli();
    uint8_t bank = NO_BANK;
    for (uint8_t i = 0; i < 2; i++) {
        if (spi_rx_state[i] == BANK_READY) {
            if (bank == NO_BANK ||
                (int16_t)(spi_rx_order[i] - spi_rx_order[bank]) < 0) bank = i;
        }
    }
    if (bank == NO_BANK) { SREG = saved; return 0; }
    spi_rx_state[bank] = BANK_READING;
    uint8_t length = spi_rx_length[bank];
    SREG = saved;
    for (uint8_t i = 0; i < length; i++) out[i] = spi_rx_buffer[bank][i];
    spi_rx_state[bank] = BANK_FREE;
    return length;
}

uint8_t link_spi_prepare(const ScannerCommand *command) {
    if (!SPI_PROTOCOL_READY) return 0;
    if (spi_response_prepared && spi_encoded_seq == command->command_seq) return 1;
    if (!(PINB & (1 << PINB2))) return 0;         // 프레임 도중 응답 변경하지 않기
    uint8_t packet[SPI_TX_MAX];
    uint8_t length = team_scanner_command(command, packet);
    if (!length || length > SPI_TX_MAX) return 0;
    uint8_t bank = 1 - spi_published_bank;
    for (uint8_t i = 0; i < length; i++) spi_tx_buffer[bank][i] = packet[i];
    spi_tx_length[bank] = length;
    uint8_t saved = SREG;
    cli();
    uint8_t ready = (PINB & (1 << PINB2)) != 0 && !spi_frame_active;
    if (ready) {
        spi_published_bank = bank;
        spi_active_tx_bank = bank;
        spi_tx_index = 0;
        SPDR = spi_tx_length[bank] ? spi_tx_buffer[bank][0] : 0;
    }
    SREG = saved;
    if (ready) {
        spi_encoded_seq = command->command_seq;
        spi_response_prepared = 1;
#if A1_USB_DEBUG
        Serial.print(F("DBG|SPI_READY|seq="));
        Serial.print(packet[2]);
        Serial.print(F("|mode="));
        Serial.print(packet[3]);
        Serial.print(F("|id="));
        Serial.println(packet[5]);
#endif
    }
    return ready;                               // 응답 준비 완료. 실행 확인은 아님.
}

// ============================================================
// UART: 2번의 명령 수신 / 2번에 중앙 상태 전송
// ============================================================
void uart_init(void) {
    Serial.begin(TurretProtocol::BAUD_RATE);      // 9600, 8N1. D0=RX, D1=TX (2번)
#if A1_USB_DEBUG
    Serial.println(F("DBG|BOOT|ARDUINO1"));
#endif
}

void debug_ui_command(uint8_t command) {
#if !A1_USB_DEBUG
    (void)command;
#else
    Serial.print(F("DBG|A2_CMD|"));
    switch (command) {
        case USER_AUTO:       Serial.println(F("AUTO")); break;
        case USER_MANUAL:     Serial.println(F("MANUAL")); break;
        case USER_FIRE:       Serial.println(F("FIRE")); break;
        case USER_STOP:       Serial.println(F("SYSTEM_STOP")); break;
        case USER_SCAN_STOP:  Serial.println(F("SCAN_STOP")); break;
        case USER_SCAN_SLOW:  Serial.println(F("SEARCH_SLOW")); break;
        case USER_SCAN_FAST:  Serial.println(F("SEARCH_FAST")); break;
        case USER_SCAN_TRACK: Serial.print(F("TRACK|id=")); Serial.println(team_uart_track_id()); break;
        case USER_SCAN_HOME:  Serial.println(F("REHOME")); break;
        default:              Serial.println(F("UNKNOWN")); break;
    }
#endif
}

// STOP을 포함한 모든 수신 명령을 한 묶음으로 모아 메인에서 우선순위 판단.
uint16_t link_uart_collect(void) {
    uint16_t commands = 0;
    uint8_t count = 0;
    while (Serial.available() > 0 && count < 64) {
        uint8_t data = Serial.read();
        uint8_t command = team_uart_command(data);
        if (command != USER_NONE) debug_ui_command(command);
        if (command == USER_AUTO || command == USER_MANUAL) {
            // 같은 묶음에 여러 모드 명령이 있으면 마지막 요청을 따른다.
            // 모드 명령보다 앞서 들어온 사격 요청은 취소하고 STOP은 보존한다.
            commands &= ~(((uint16_t)1 << USER_AUTO) |
                          ((uint16_t)1 << USER_MANUAL) |
                          ((uint16_t)1 << USER_FIRE));
        }
        if (command >= USER_SCAN_STOP && command <= USER_SCAN_HOME) {
            const uint16_t scanner_mask =
                ((uint16_t)1 << USER_SCAN_STOP) |
                ((uint16_t)1 << USER_SCAN_SLOW) |
                ((uint16_t)1 << USER_SCAN_FAST) |
                ((uint16_t)1 << USER_SCAN_TRACK) |
                ((uint16_t)1 << USER_SCAN_HOME);
            commands &= ~scanner_mask; // 같은 묶음에서는 마지막 스캐너 요청 사용
        }
        if (command >= USER_AUTO && command <= USER_SCAN_HOME)
            commands |= ((uint16_t)1 << command);
        count++;
    }
    return commands;
}

uint8_t link_uart_send(const CentralReport *report) {
    if (!UART_PROTOCOL_READY) return 0;
    uint8_t packet[UART_TX_MAX];
    uint8_t length = team_uart_status(report, packet);
    if (!length || length > UART_TX_MAX) return 0;
    // 하드웨어 Serial 송신은 버퍼+인터럽트로 처리되어 SPI 인터럽트를 막지 않는다.
    if (Serial.availableForWrite() < length) return 0; // 송신 버퍼 대기로 정지 처리 늦추지 않기
    return Serial.write(packet, length) == length;
}

// ============================================================
// I2C: 1번 Master -> 4번 Slave, 정해진 7비트 주소 0x20
// ============================================================
#define ARDUINO4_ADDRESS 0x20
#define TWI_WAIT_MS 5UL                         // 구현상 대기 상한. 실제 장치와 확인
void twi_init(void) {
    DDRC &= ~((1 << DDC4) | (1 << DDC5));        // A4 SDA, A5 SCL
    TWSR = 0;
    TWBR = 72;                                 // 16MHz 기준 100kHz
    TWCR = (1 << TWEN);
}

uint8_t twi_wait(void) {
    uint32_t start = millis();
    while (!(TWCR & (1 << TWINT)))
        if ((uint32_t)(millis() - start) >= TWI_WAIT_MS) return 0;
    return 1;
}

uint8_t twi_start(void) {
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    if (!twi_wait()) return 0xFF;
    return TWSR & 0xF8;
}

uint8_t twi_stop(void) {
    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
    uint32_t start = millis();
    while (TWCR & (1 << TWSTO)) {
        if ((uint32_t)(millis() - start) >= TWI_WAIT_MS) {
            TWCR = 0; TWCR = (1 << TWEN);
            return 0;
        }
    }
    return 1;
}

uint8_t twi_write(uint8_t data) {
    TWDR = data;
    TWCR = (1 << TWINT) | (1 << TWEN);
    if (!twi_wait()) return 0xFF;
    return TWSR & 0xF8;
}

uint8_t twi_read(uint8_t ack) {
    TWCR = (1 << TWINT) | (1 << TWEN) | (ack ? (1 << TWEA) : 0);
    if (!twi_wait()) return 0;
    return (TWSR & 0xF8) == (ack ? 0x50 : 0x58);
}

uint8_t twi_master_write(const uint8_t *data, uint8_t length) {
    if (!length || length > I2C_PACKET_MAX) return 0;
    if (twi_start() != 0x08) { twi_stop(); return 0; }
    if (twi_write(ARDUINO4_ADDRESS << 1) != 0x18) { twi_stop(); return 0; }
    for (uint8_t i = 0; i < length; i++)
        if (twi_write(data[i]) != 0x28) { twi_stop(); return 0; }
    return twi_stop();
}

uint8_t twi_master_read(uint8_t *data, uint8_t length) {
    if (!length || length > I2C_PACKET_MAX) return 0;
    if (twi_start() != 0x08) { twi_stop(); return 0; }
    if (twi_write((ARDUINO4_ADDRESS << 1) | 1) != 0x40) { twi_stop(); return 0; }
    for (uint8_t i = 0; i < length; i++) {
        uint8_t ack = i + 1 < length;             // 마지막 바이트에는 NACK
        if (!twi_read(ack)) { twi_stop(); return 0; }
        data[i] = TWDR;
    }
    return twi_stop();
}

uint8_t link_i2c_send(const TurretCommand *command) {
    uint8_t packet[I2C_PACKET_MAX];
    uint8_t length = team_turret_command(command, packet);
    uint8_t ok = length && length <= I2C_PACKET_MAX && twi_master_write(packet, length);
    team_turret_sent(command, ok, millis());
    return ok;                                 // ACK는 실제 명령 처리/모의 동작 완료가 아님
}

uint8_t link_i2c_read(TurretReport *report) {
    uint8_t packet[I2C_PACKET_MAX];
    if (!twi_master_read(packet, TURRET_REPORT_BYTES)) { team_turret_read_failed(); return 0; }
    return team_turret_status(packet, TURRET_REPORT_BYTES, report);
}

// 모드/표적/승인/정지/상태 관리는 별도 파일에 모았다.
#include "controller_logic.h"

// ============================================================
// main: STOP 확인 -> 탐지 반영 -> 판단 -> 4번 제어 -> 상태 전송
// ============================================================
void controller_loop_once(void) {
    uint8_t frame[SPI_RX_MAX];
    uint32_t now = millis();
    controller_process_commands(link_uart_collect(), now);

    uint8_t saved = SREG;
    cli();
    uint8_t spi_error = spi_transport_error;
    spi_transport_error = 0;
    SREG = saved;
    if (spi_error) {
        faults |= FAULT_SPI_FRAME;
        controller_request_safe();
    }
    uint8_t length = spi_take_frame(frame);
    if (length) controller_accept_detection(frame, length, now);

    controller_refresh_faults(now);
    controller_select_target(now);
    controller_update_scanner();
    controller_service_i2c(now);
    controller_update_state(now);
    controller_send_status(now);
}

int main(void) {
    init(); uart_init(); twi_init(); spi_slave_init(); sei();
    while (1) controller_loop_once();
}
