#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>

// ==========================================================================
// Arduino 1 <-> Arduino 2 UART protocol
// ==========================================================================
namespace UiProtocol {

constexpr uint8_t STX = 0x02;
constexpr uint8_t ETX = 0x03;
constexpr uint32_t BAUD_RATE = 9600;
constexpr uint8_t RX_FRAME_SIZE = 7;
constexpr uint8_t TX_FRAME_SIZE = 3;
constexpr uint8_t TRACK_TX_FRAME_SIZE = 4;
constexpr uint16_t MAX_THETA_CDEG = 35999;

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

bool isValidTarget(uint8_t targetId, uint16_t rMm, uint16_t thetaCdeg) {
  if (targetId == 0) return rMm == 0 && thetaCdeg == 0;
  return thetaCdeg <= MAX_THETA_CDEG;
}

void sendCommand(Stream &port, Command command) {
  const uint8_t frame[TX_FRAME_SIZE] = {
    STX, static_cast<uint8_t>(command), ETX
  };
  port.write(frame, TX_FRAME_SIZE);
}

// TRACK only: [STX][0x33][specific target ID][ETX]
void sendTrackCommand(Stream &port, uint8_t targetId) {
  const uint8_t frame[TRACK_TX_FRAME_SIZE] = {
    STX, static_cast<uint8_t>(CMD_TRACK_TARGET), targetId, ETX
  };
  port.write(frame, TRACK_TX_FRAME_SIZE);
}

}  // namespace UiProtocol

// ==========================================================================
// Pin assignment (Arduino Uno)
// ==========================================================================
constexpr uint8_t PIN_UART_RX = 2;       // Arduino 1 TX (D3) -> Arduino 2 D2
constexpr uint8_t PIN_UART_TX = 3;       // Arduino 1 RX (D2) <- Arduino 2 D3
constexpr uint8_t PIN_BUTTON_MODE = 4;   // Button to GND; INPUT_PULLUP
constexpr uint8_t PIN_BUTTON_FIRE = 5;   // Button to GND; INPUT_PULLUP
constexpr uint8_t PIN_BUTTON_SCANNER = 6; // 5 scanner modes; button to GND
constexpr uint8_t PIN_BUZZER = 8;        // 3-pin passive buzzer module SIG

// Common-anode RGB LED: common pin to 5V; LOW turns a colour on.
constexpr uint8_t PIN_LED_GREEN = 9;     // D9 -> 220 ohm -> green cathode
constexpr uint8_t PIN_LED_RED = 10;      // D10 -> 220 ohm -> red cathode

constexpr uint8_t LCD_ADDRESS = 0x27;    // Try 0x3F if required
constexpr uint32_t USB_DEBUG_BAUD = 115200;
constexpr uint32_t CONTROLLER_TIMEOUT_MS = 1500;
constexpr uint32_t RX_FRAME_TIMEOUT_MS = 100;
constexpr uint16_t BUTTON_DEBOUNCE_MS = 35;
constexpr uint16_t SCANNER_ESTOP_HOLD_MS = 1200;
// FIRE acknowledgement: long high beep, short pause, then a short higher beep.
// This only confirms that Arduino 2 transmitted the FIRE command.
constexpr uint16_t FIRE_TONE_LONG_MS = 550;
constexpr uint16_t FIRE_TONE_GAP_MS = 90;
constexpr uint16_t FIRE_TONE_SHORT_MS = 160;
constexpr uint16_t FIRE_TONE_LONG_HZ = 2100;
constexpr uint16_t FIRE_TONE_SHORT_HZ = 2700;

SoftwareSerial controllerSerial(PIN_UART_RX, PIN_UART_TX);  // RX, TX
LiquidCrystal_I2C lcd(LCD_ADDRESS, 16, 2);

enum UiMode : uint8_t {
  MODE_AUTO,
  MODE_MANUAL
};

enum ScannerMode : uint8_t {
  SCAN_STOP = 0,
  SCAN_SEARCH_SLOW = 1,
  SCAN_SEARCH_FAST = 2,
  SCAN_TRACK_TARGET = 3,
  SCAN_REHOME = 4
};

enum RxStep : uint8_t {
  WAIT_STX,
  READ_TARGET_ID,
  READ_R_LOW,
  READ_R_HIGH,
  READ_THETA_LOW,
  READ_THETA_HIGH,
  READ_ETX
};

enum DisplayState : uint8_t {
  VIEW_LINK_LOST,
  VIEW_TARGET,
  VIEW_NO_TARGET
};

struct DebouncedButton {
  uint8_t pin;
  bool rawPressed;
  bool stablePressed;
  bool pressedEvent;
  bool releasedEvent;
  uint32_t rawChangedAt;
  uint32_t pressedAt;
  uint32_t lastPressDuration;

  void begin() {
    pinMode(pin, INPUT_PULLUP);
    rawPressed = (digitalRead(pin) == LOW);
    stablePressed = rawPressed;
    pressedEvent = false;
    releasedEvent = false;
    rawChangedAt = millis();
    pressedAt = rawChangedAt;
    lastPressDuration = 0;
  }

  void update(uint32_t now) {
    pressedEvent = false;
    releasedEvent = false;
    const bool reading = (digitalRead(pin) == LOW);

    if (reading != rawPressed) {
      rawPressed = reading;
      rawChangedAt = now;
    }

    if (rawPressed != stablePressed &&
        now - rawChangedAt >= BUTTON_DEBOUNCE_MS) {
      stablePressed = rawPressed;
      if (stablePressed) {
        pressedAt = now;
        pressedEvent = true;
      } else {
        lastPressDuration = now - pressedAt;
        releasedEvent = true;
      }
    }
  }
};

DebouncedButton modeButton{PIN_BUTTON_MODE};
DebouncedButton fireButton{PIN_BUTTON_FIRE};
DebouncedButton scannerButton{PIN_BUTTON_SCANNER};

UiMode uiMode = MODE_AUTO;
RxStep rxStep = WAIT_STX;

uint8_t receivedTargetId = 0;
uint16_t receivedRmm = 0;
uint16_t receivedThetaCdeg = 0;

uint8_t pendingTargetId = 0;
uint16_t pendingRmm = 0;
uint16_t pendingThetaCdeg = 0;

uint32_t lastValidPacketAt = 0;
uint32_t rxByteAt = 0;
bool hasReceivedPacket = false;
bool scannerMenuActive = false;
ScannerMode scannerMenuMode = SCAN_STOP;
bool fireToneActive = false;
uint32_t fireToneStartedAt = 0;

// ==========================================================================
// LCD helpers
// ==========================================================================
void writeLcdLine(uint8_t row, const char *text) {
  char padded[17];
  uint8_t i = 0;

  while (i < 16 && text[i] != '\0') {
    padded[i] = text[i];
    ++i;
  }
  while (i < 16) padded[i++] = ' ';

  padded[16] = '\0';
  lcd.setCursor(0, row);
  lcd.print(padded);
}

// ==========================================================================
// Arduino 2 -> Arduino 1 commands
// ==========================================================================
void logCommand(UiProtocol::Command command) {
  Serial.print(F("TX command: 0x"));
  if (static_cast<uint8_t>(command) < 0x10) Serial.print('0');
  Serial.println(static_cast<uint8_t>(command), HEX);
}

void sendControllerCommand(UiProtocol::Command command) {
  UiProtocol::sendCommand(controllerSerial, command);
  logCommand(command);
}

UiProtocol::Command scannerCommand(ScannerMode mode) {
  switch (mode) {
    case SCAN_STOP:         return UiProtocol::CMD_SCAN_STOP;
    case SCAN_SEARCH_SLOW:  return UiProtocol::CMD_SEARCH_SLOW;
    case SCAN_SEARCH_FAST:  return UiProtocol::CMD_SEARCH_FAST;
    case SCAN_TRACK_TARGET: return UiProtocol::CMD_TRACK_TARGET;
    case SCAN_REHOME:       return UiProtocol::CMD_REHOME;
  }
  return UiProtocol::CMD_SCAN_STOP;
}

bool sendScannerMode(ScannerMode mode) {
  if (mode == SCAN_TRACK_TARGET) {
    if (!controllerConnected(millis()) || receivedTargetId == 0) {
      Serial.println(F("TRACK ignored: no current target ID from A1"));
      return false;
    }
    // TRACK includes the exact ID selected on the A2 display. A1 must copy it
    // into RadarWire command byte 5; UNO3 follows findTarget(preferredId).
    Serial.print(F("TRACK current target ID: "));
    Serial.println(receivedTargetId);
    UiProtocol::sendTrackCommand(controllerSerial, receivedTargetId);
    Serial.print(F("TX TRACK frame: 02 33 "));
    if (receivedTargetId < 0x10) Serial.print('0');
    Serial.print(receivedTargetId, HEX);
    Serial.println(F(" 03"));
    Serial.println(F("Scanner mode request: 3"));
    return true;
  }

  sendControllerCommand(scannerCommand(mode));
  Serial.print(F("Scanner mode request: "));
  Serial.println(static_cast<uint8_t>(mode));
  return true;
}

// ==========================================================================
// Arduino 1 -> Arduino 2 fixed-length 7-byte receiver
// [STX][ID][rL][rH][thetaL][thetaH][ETX]
// ==========================================================================
void acceptTarget(uint8_t targetId, uint16_t rMm, uint16_t thetaCdeg,
                  uint32_t now) {
  if (!UiProtocol::isValidTarget(targetId, rMm, thetaCdeg)) {
    Serial.println(F("RX rejected: invalid target data"));
    return;
  }

  const bool linkWasActive =
      hasReceivedPacket && now - lastValidPacketAt <= CONTROLLER_TIMEOUT_MS;

  receivedTargetId = targetId;
  receivedRmm = rMm;
  receivedThetaCdeg = thetaCdeg;
  lastValidPacketAt = now;
  hasReceivedPacket = true;

  Serial.print(F("RX target="));
  Serial.print(receivedTargetId);
  Serial.print(F(", r_mm="));
  Serial.print(receivedRmm);
  Serial.print(F(", theta_cdeg="));
  Serial.println(receivedThetaCdeg);

  // Re-send local AUTO/MANUAL state after initial connection or reconnection.
  if (!linkWasActive) {
    sendControllerCommand(uiMode == MODE_AUTO
                              ? UiProtocol::CMD_AUTO_MODE
                              : UiProtocol::CMD_MANUAL_MODE);
  }
}

void processControllerByte(uint8_t value, uint32_t now) {
  rxByteAt = now;

  switch (rxStep) {
    case WAIT_STX:
      if (value == UiProtocol::STX) rxStep = READ_TARGET_ID;
      break;

    case READ_TARGET_ID:
      pendingTargetId = value;
      rxStep = READ_R_LOW;
      break;

    case READ_R_LOW:
      pendingRmm = value;
      rxStep = READ_R_HIGH;
      break;

    case READ_R_HIGH:
      pendingRmm |= static_cast<uint16_t>(value) << 8;
      rxStep = READ_THETA_LOW;
      break;

    case READ_THETA_LOW:
      pendingThetaCdeg = value;
      rxStep = READ_THETA_HIGH;
      break;

    case READ_THETA_HIGH:
      pendingThetaCdeg |= static_cast<uint16_t>(value) << 8;
      rxStep = READ_ETX;
      break;

    case READ_ETX:
      if (value == UiProtocol::ETX) {
        acceptTarget(pendingTargetId, pendingRmm, pendingThetaCdeg, now);
        rxStep = WAIT_STX;
      } else {
        // Only byte 6 is interpreted as ETX. STX/ETX values inside data are
        // accepted normally because each data byte has a fixed position.
        rxStep = (value == UiProtocol::STX) ? READ_TARGET_ID : WAIT_STX;
        Serial.println(F("RX rejected: bad ETX"));
      }
      break;
  }
}

void pollController(uint32_t now) {
  if (rxStep != WAIT_STX && now - rxByteAt > RX_FRAME_TIMEOUT_MS) {
    rxStep = WAIT_STX;
    Serial.println(F("RX reset: frame timeout"));
  }

  while (controllerSerial.available() > 0) {
    processControllerByte(static_cast<uint8_t>(controllerSerial.read()), now);
  }
}

bool controllerConnected(uint32_t now) {
  return hasReceivedPacket && now - lastValidPacketAt <= CONTROLLER_TIMEOUT_MS;
}

void startFireTone(uint32_t now) {
  fireToneStartedAt = now;
  fireToneActive = true;
  Serial.println(F("FIRE sound: long beep + short beep"));
}

// Returns true while the FIRE sound owns the buzzer. No delay() is used, so
// UART reception and button handling continue during the sound.
bool updateFireTone(uint32_t now) {
  if (!fireToneActive) return false;

  const uint32_t elapsed = now - fireToneStartedAt;
  if (elapsed < FIRE_TONE_LONG_MS) {
    tone(PIN_BUZZER, FIRE_TONE_LONG_HZ);
    return true;
  }

  if (elapsed < FIRE_TONE_LONG_MS + FIRE_TONE_GAP_MS) {
    noTone(PIN_BUZZER);
    return true;
  }

  if (elapsed < FIRE_TONE_LONG_MS + FIRE_TONE_GAP_MS +
                    FIRE_TONE_SHORT_MS) {
    tone(PIN_BUZZER, FIRE_TONE_SHORT_HZ);
    return true;
  }

  fireToneActive = false;
  noTone(PIN_BUZZER);
  return false;
}

// ==========================================================================
// Buttons
// ==========================================================================
void handleButtons(uint32_t now) {
  modeButton.update(now);
  fireButton.update(now);
  scannerButton.update(now);

  // MODE is now dedicated to AUTO/MANUAL.
  if (modeButton.pressedEvent) {
    if (scannerMenuActive) {
      Serial.println(F("MODE ignored while scanner menu is open"));
    } else {
      uiMode = (uiMode == MODE_AUTO) ? MODE_MANUAL : MODE_AUTO;
      sendControllerCommand(uiMode == MODE_AUTO
                                ? UiProtocol::CMD_AUTO_MODE
                                : UiProtocol::CMD_MANUAL_MODE);
    }
  }

  // Former STOP button: a short release opens/cycles the five scanner modes.
  // A 1.2 s hold preserves the distinct system-wide emergency STOP (0xFF).
  if (scannerButton.releasedEvent) {
    if (scannerButton.lastPressDuration >= SCANNER_ESTOP_HOLD_MS) {
      scannerMenuActive = false;
      sendControllerCommand(UiProtocol::CMD_STOP);
      Serial.println(F("SYSTEM EMERGENCY STOP requested"));
    } else {
      if (!scannerMenuActive) {
        scannerMenuActive = true;
        scannerMenuMode = SCAN_STOP;
        Serial.println(F("Scanner menu opened: 0 STOP"));
        if (!controllerConnected(now))
          Serial.println(F("Warning: A1 link lost; non-TRACK commands can still be sent"));
      } else {
        scannerMenuMode = static_cast<ScannerMode>(
            (static_cast<uint8_t>(scannerMenuMode) + 1) % 5);
        Serial.print(F("Scanner menu item: "));
        Serial.println(static_cast<uint8_t>(scannerMenuMode));
      }
    }
  }

  if (fireButton.pressedEvent) {
    if (scannerMenuActive) {
      // FIRE acts as SELECT only while the scanner menu is open.
      if (sendScannerMode(scannerMenuMode)) {
        scannerMenuActive = false;
      }
    } else if (uiMode == MODE_MANUAL && receivedTargetId != 0) {
      sendControllerCommand(UiProtocol::CMD_FIRE);
      startFireTone(now);
    } else {
      Serial.println(F("FIRE ignored: requires MANUAL mode and a target"));
    }
  }
}

// ==========================================================================
// Display, RGB LED and passive buzzer
// ==========================================================================
DisplayState currentDisplayState(uint32_t now) {
  if (!controllerConnected(now)) return VIEW_LINK_LOST;
  if (receivedTargetId != 0) return VIEW_TARGET;
  return VIEW_NO_TARGET;
}

void updateIndicators(uint32_t now) {
  const DisplayState state = currentDisplayState(now);
  bool green = false;
  bool yellow = false;
  bool red = false;

  switch (state) {
    case VIEW_LINK_LOST:
      yellow = ((now / 500) % 2) == 0;
      break;

    case VIEW_TARGET:
      yellow = ((now / 500) % 2) == 0;
      break;

    case VIEW_NO_TARGET:
      green = true;
      break;
  }

  // The FIRE sound has priority over the normal target-detection beep.
  if (!updateFireTone(now)) {
    if (state == VIEW_TARGET && (now % 1000) < 150) {
      tone(PIN_BUZZER, 850);
    } else {
      noTone(PIN_BUZZER);
    }
  }

  // Common-anode RGB LED: LOW = on, HIGH = off.
  // Yellow is produced by turning red and green on together.
  digitalWrite(PIN_LED_GREEN, (green || yellow) ? LOW : HIGH);
  digitalWrite(PIN_LED_RED, (red || yellow) ? LOW : HIGH);
}

void updateDisplay(uint32_t now) {
  static DisplayState previousState = VIEW_LINK_LOST;
  static UiMode previousMode = MODE_MANUAL;
  static uint8_t previousTargetId = 255;
  static uint16_t previousRmm = 0;
  static uint16_t previousThetaCdeg = 0;
  static bool previousScannerMenuActive = false;
  static ScannerMode previousScannerMenuMode = SCAN_REHOME;
  static bool firstDraw = true;

  const DisplayState state = currentDisplayState(now);
  if (!firstDraw && state == previousState && uiMode == previousMode &&
      receivedTargetId == previousTargetId && receivedRmm == previousRmm &&
      receivedThetaCdeg == previousThetaCdeg &&
      scannerMenuActive == previousScannerMenuActive &&
      scannerMenuMode == previousScannerMenuMode) {
    return;
  }

  char line1[17];
  char line2[17];

  if (scannerMenuActive) {
    snprintf(line1, sizeof(line1), "SCANNER MODE");
    switch (scannerMenuMode) {
      case SCAN_STOP:
        snprintf(line2, sizeof(line2), "0 STOP");
        break;
      case SCAN_SEARCH_SLOW:
        snprintf(line2, sizeof(line2), "1 SEARCH SLOW");
        break;
      case SCAN_SEARCH_FAST:
        snprintf(line2, sizeof(line2), "2 SEARCH FAST");
        break;
      case SCAN_TRACK_TARGET:
        snprintf(line2, sizeof(line2), "3 TRACK ID:%u", receivedTargetId);
        break;
      case SCAN_REHOME:
        snprintf(line2, sizeof(line2), "4 REHOME");
        break;
    }
  } else switch (state) {
    case VIEW_LINK_LOST:
      snprintf(line1, sizeof(line1), "MODE: %-6s",
               uiMode == MODE_AUTO ? "AUTO" : "MANUAL");
      snprintf(line2, sizeof(line2), "WAITING A1...");
      break;

    case VIEW_TARGET:
      snprintf(line1, sizeof(line1), "%c I:%u R:%umm",
               uiMode == MODE_AUTO ? 'A' : 'M', receivedTargetId,
               receivedRmm);
      snprintf(line2, sizeof(line2), "TH:%u.%02u deg",
               receivedThetaCdeg / 100, receivedThetaCdeg % 100);
      break;

    case VIEW_NO_TARGET:
      snprintf(line1, sizeof(line1), "MODE: %-6s",
               uiMode == MODE_AUTO ? "AUTO" : "MANUAL");
      snprintf(line2, sizeof(line2), "NO TARGET");
      break;
  }

  writeLcdLine(0, line1);
  writeLcdLine(1, line2);

  previousState = state;
  previousMode = uiMode;
  previousTargetId = receivedTargetId;
  previousRmm = receivedRmm;
  previousThetaCdeg = receivedThetaCdeg;
  previousScannerMenuActive = scannerMenuActive;
  previousScannerMenuMode = scannerMenuMode;
  firstDraw = false;
}

// ==========================================================================
// Arduino entry points
// ==========================================================================
void setup() {
  // Prevent a common-anode RGB LED flash during startup.
  digitalWrite(PIN_LED_GREEN, HIGH);
  digitalWrite(PIN_LED_RED, HIGH);
  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);

  modeButton.begin();
  fireButton.begin();
  scannerButton.begin();

  Serial.begin(USB_DEBUG_BAUD);
  controllerSerial.begin(UiProtocol::BAUD_RATE);

  lcd.init();
  lcd.backlight();
  writeLcdLine(0, "ARDUINO 2 UI");
  writeLcdLine(1, "STARTING...");

  Serial.println(F("Arduino 2 UI started"));
  Serial.println(F("UART: 7-byte target frame, 9600 baud, 8N1"));
  Serial.println(F("A2->A1: 3-byte commands; TRACK uses 02 33 ID 03"));
  Serial.println(F("MODE: AUTO/MANUAL; D6 short: scanner next; FIRE: select"));
  Serial.println(F("D6 hold 1.2 s: SYSTEM EMERGENCY STOP"));
  delay(500);
}

void loop() {
  const uint32_t now = millis();
  pollController(now);
  handleButtons(now);
  updateIndicators(now);
  updateDisplay(now);
}
