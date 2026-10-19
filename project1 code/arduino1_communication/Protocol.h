#ifndef TURRET_UI_PROTOCOL_H
#define TURRET_UI_PROTOCOL_H

#include <Arduino.h>

namespace TurretProtocol {

constexpr uint8_t STX = 0x02;
constexpr uint8_t ETX = 0x03;
constexpr uint32_t BAUD_RATE = 9600;

// Arduino 1 -> Arduino 2 (7 bytes):
// [STX][target ID][r L][r H][theta L][theta H][ETX]
// r: millimetres, theta: centidegrees, both unsigned little-endian.
constexpr uint8_t RX_FRAME_SIZE = 7;
constexpr uint16_t MAX_THETA_CDEG = 35999;

// Arduino 2 -> Arduino 1: [STX][command][ETX]
constexpr uint8_t TX_FRAME_SIZE = 3;

enum Command : uint8_t {
  CMD_AUTO_MODE = 0x10,
  CMD_MANUAL_MODE = 0x11,
  CMD_FIRE = 0x20,
  CMD_SCAN_STOP = 0x30,
  CMD_SEARCH_SLOW = 0x31,
  CMD_SEARCH_FAST = 0x32,
  CMD_TRACK_TARGET = 0x33,
  CMD_REHOME = 0x34,
  CMD_STOP = 0xFF
};

inline bool isValidTarget(uint8_t targetId, uint16_t rMm,
                          uint16_t thetaCdeg) {
  if (targetId == 0) return rMm == 0 && thetaCdeg == 0;
  return thetaCdeg <= MAX_THETA_CDEG;
}

template <typename StreamType>
void sendCommand(StreamType &port, Command command) {
  const uint8_t frame[TX_FRAME_SIZE] = {
    STX, static_cast<uint8_t>(command), ETX
  };
  port.write(frame, TX_FRAME_SIZE);
}

template <typename StreamType>
void sendStatus(StreamType &port, uint8_t targetId, uint16_t rMm,
                uint16_t thetaCdeg) {
  const uint8_t frame[RX_FRAME_SIZE] = {
    STX,
    targetId,
    static_cast<uint8_t>(rMm),
    static_cast<uint8_t>(rMm >> 8),
    static_cast<uint8_t>(thetaCdeg),
    static_cast<uint8_t>(thetaCdeg >> 8),
    ETX
  };
  port.write(frame, RX_FRAME_SIZE);
}

}  // namespace TurretProtocol

#endif
