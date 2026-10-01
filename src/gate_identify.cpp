// Read-only Herkulex discovery, adapted from Josh Craythorne's Servo_Setup.
// gateidentify emits only STAT and RAM_READ. gatepositions adds explicit
// operator-controlled RAM torque and S_JOG commands. Neither writes EEPROM.
// Never scan Serial1 (drive pins).
#if defined(GATE_IDENTIFY) || defined(GATE_POSITION_TEST)
#include <Arduino.h>
#include "config.h"
#include "motor.h"

namespace {
constexpr uint32_t bauds[] = {115200, 57600, 200000, 250000, 400000, 500000, 666666};
constexpr uint8_t quickIds[] = {1, 253, 0, 2, 3, 4, 5, 6, 7, 8, 9, 10};
constexpr uint32_t replyMs = 25;
constexpr uint8_t statCmd = 0x07, readCmd = 0x04;
bool scanning = false, fullScan = false;
uint8_t baudIndex = 0;
uint16_t scanIndex = 0;
int foundId = -1;
uint32_t rxBytes = 0, echoes = 0, badPackets = 0;
#ifdef GATE_POSITION_TEST
constexpr uint8_t benchId = 4; // physically discovered on CON66 2026-10-01
bool moving = false;
int target = -1;
uint32_t moveStarted = 0, lastPoll = 0;
uint8_t arrivedSamples = 0;
#endif

void sendRequest(uint8_t id, uint8_t cmd, const uint8_t *data, uint8_t count) {
  if (count > 5) return;
  uint8_t packet[12] = {0xFF, 0xFF, uint8_t(7 + count), id, cmd, 0, 0};
  uint8_t checksum = packet[2] ^ id ^ cmd;
  for (uint8_t i = 0; i < count; ++i) {
    packet[7 + i] = data[i]; checksum ^= data[i];
  }
  packet[5] = checksum & 0xFE;
  packet[6] = (~packet[5]) & 0xFE;
  while (GATE_SERIAL.available()) GATE_SERIAL.read();
  GATE_SERIAL.write(packet, 7 + count);
  GATE_SERIAL.flush();
}

bool readReply(uint8_t id, uint8_t cmd, uint8_t *packet) {
  uint8_t state = 0, size = 0, index = 0;
  const uint32_t started = millis();
  while (millis() - started < replyMs) {
    if (!GATE_SERIAL.available()) continue;
    uint8_t b = GATE_SERIAL.read(); ++rxBytes;
    if (state == 0) { if (b == 0xFF) state = 1; continue; }
    if (state == 1) { state = b == 0xFF ? 2 : 0; continue; }
    if (state == 2) {
      if (b == 0xFF) continue; // tolerate an extra header byte
      if (b < 7 || b > 40) { ++badPackets; state = 0; continue; }
      size = b; packet[0] = b; index = 1; state = 3; continue;
    }
    packet[index++] = b;
    if (index < size - 2) continue;
    uint8_t checksum = size ^ packet[1] ^ packet[2];
    for (uint8_t i = 5; i < size - 2; ++i) checksum ^= packet[i];
    checksum &= 0xFE;
    if (checksum != packet[3] || uint8_t((~checksum) & 0xFE) != packet[4]) {
      ++badPackets;
    } else if (packet[2] < 0x40) {
      ++echoes;
    } else if (size >= 9 && packet[1] == id && packet[2] == (cmd | 0x40)) {
      return true;
    }
    state = 0;
  }
  return false;
}

bool ping(uint8_t id, uint8_t *packet) {
  sendRequest(id, statCmd, nullptr, 0);
  return readReply(id, statCmd, packet);
}

int readRegister(uint8_t address, uint8_t count) {
  uint8_t request[] = {address, count}, packet[40];
  sendRequest(uint8_t(foundId), readCmd, request, 2);
  if (!readReply(uint8_t(foundId), readCmd, packet) ||
      packet[0] < 11 + count || packet[5] != address || packet[6] != count) return -1;
  return count == 2 ? (packet[7] | (packet[8] << 8)) : packet[7];
}

void report() {
  if (foundId < 0) { Serial.println("No identified servo. Send f to scan."); return; }
  uint8_t packet[40];
  if (!ping(uint8_t(foundId), packet)) { Serial.println("Identified servo no longer replies."); return; }
  Serial.printf("FOUND ID=%d Serial2/CON66 baud=%lu status_error=0x%02X status_detail=0x%02X\n",
                foundId, (unsigned long)bauds[baudIndex], packet[packet[0]-4], packet[packet[0]-3]);
  int voltage = readRegister(54, 1);
  int position = readRegister(58, 2);
  int absolute = readRegister(60, 2);
  int torque = readRegister(52, 1);
  Serial.printf("position_raw=%d torque_raw=%d supply_raw=%d", position, torque, voltage);
  if (voltage >= 0) Serial.printf(" supply_V=%.2f", voltage * 0.074f);
  Serial.println(" (-1 means register read unavailable)");
  // HerkulexServo::getPosition/getRawPosition and Josh's sketch use 10 bits.
  // Keep unmasked words above for diagnosing upper bits, never mask -1.
  Serial.printf("position_10bit=%d absolute_10bit=%d\n",
                position < 0 ? -1 : position & 0x03FF,
                absolute < 0 ? -1 : absolute & 0x03FF);
  Serial.printf("Round firmware expects ID=%d baud=%d; match=%s\n", GATE_ID, GATE_BAUD,
                foundId == GATE_ID && bauds[baudIndex] == GATE_BAUD ? "YES" : "NO");
}

#ifdef GATE_POSITION_TEST
void releaseGate(const char *reason) {
  const uint8_t data[] = {52, 1, 0};
  sendRequest(benchId, 0x03, data, sizeof(data));
  moving = false;
  Serial.printf("GATE RELEASED: %s\n", reason);
}

void moveGate(int destination) {
  if (moving) { Serial.println("Already moving; x releases immediately."); return; }
  uint8_t packet[40];
  if (!ping(benchId, packet)) { Serial.println("REFUSED: ID 4 not responding."); return; }
  if (packet[packet[0]-4] != 0) { Serial.println("REFUSED: servo fault; inspect p status."); return; }
  int pos = readRegister(58, 2);
  if (pos < 0) { Serial.println("REFUSED: no position feedback."); return; }
  const uint8_t torque[] = {52, 1, 0x60};
  sendRequest(benchId, 0x03, torque, sizeof(torque));
  delay(10);
  if (readRegister(52, 1) != 0x60) {
    releaseGate("torque enable not confirmed"); return;
  }
  // About 2 seconds per movement, slower than the round's saved playtime.
  const uint8_t jog[] = {180, uint8_t(destination & 0xFF), uint8_t(destination >> 8), 0x04, benchId};
  sendRequest(benchId, 0x06, jog, sizeof(jog));
  target = destination; moving = true; arrivedSamples = 0;
  moveStarted = lastPoll = millis();
  Serial.printf("MOVE target=%d from=%d (10-bit); x releases\n", target, pos & 0x03FF);
}

void pollMovement() {
  if (!moving || millis() - lastPoll < 150) return;
  lastPoll = millis();
  uint8_t packet[40];
  if (!ping(benchId, packet) || packet[packet[0]-4] != 0) {
    releaseGate("fault or communication loss"); return;
  }
  int word = readRegister(58, 2);
  if (word < 0) { releaseGate("position feedback lost"); return; }
  int position = word & 0x03FF;
  Serial.printf("MOVE position=%d target=%d error=%d\n", position, target, position-target);
  if (abs(position-target) <= 12) ++arrivedSamples; else arrivedSamples = 0;
  if (millis()-moveStarted >= 2100 && arrivedSamples >= 3) {
    moving = false;
    Serial.printf("ARRIVED target=%d position=%d; confirm physical gate position.\n", target, position);
    return;
  }
  if (millis()-moveStarted > 4000) releaseGate("target not reached within 4 seconds");
}
#endif

void selectBaud() {
  GATE_SERIAL.end();
  GATE_SERIAL.begin(bauds[baudIndex]);
  delay(20);
  rxBytes = echoes = badPackets = 0;
  Serial.printf("SCAN %s baud=%lu\n", fullScan ? "IDs 0..253" : "common IDs",
                (unsigned long)bauds[baudIndex]);
}

void startScan() {
  scanning = true; fullScan = false; baudIndex = 0; scanIndex = 0; foundId = -1;
  Serial.println("Scanning CON66 only. Up to about 50 seconds. Send x to cancel.");
  selectBaud();
}

void scanStep() {
  uint8_t id = fullScan ? uint8_t(scanIndex) : quickIds[scanIndex];
  uint8_t packet[40];
  // Confirm twice to reject isolated noise falsely matching a packet checksum.
  if (ping(id, packet) && ping(id, packet)) {
    foundId = id; scanning = false; report(); return;
  }
  if (++scanIndex < (fullScan ? 254 : sizeof(quickIds))) return;
  Serial.printf("  rx_bytes=%lu echoed_requests=%lu malformed_packets=%lu\n",
                (unsigned long)rxBytes, (unsigned long)echoes, (unsigned long)badPackets);
  scanIndex = 0;
  if (++baudIndex == sizeof(bauds)/sizeof(bauds[0])) {
    baudIndex = 0;
    if (fullScan) {
      scanning = false;
      Serial.println("NOT FOUND at scanned rates. Echo alone does not prove servo communication.");
      Serial.println("Check servo supply, CON66 and interface TX/RX. No settings were changed.");
      return;
    }
    fullScan = true;
  }
  selectBaud();
}

void help() {
#ifdef GATE_POSITION_TEST
  Serial.println("GATE_POSITIONS_V1 - ID 4 Serial2/CON66 115200");
  Serial.printf("c=close (%d); o=open (%d); x=torque OFF; p=status; ?=help\n", GATE_CLOSED_POS, GATE_OPEN_POS);
  Serial.println("No movement on boot. Each move takes about 2 seconds. No automatic cycling.");
  Serial.println("Drive neutral; magnet OFF; crane undriven; GO ignored. No EEPROM writes.");
#else
  Serial.println("GATE_IDENTIFY_V1 - read-only Serial2/CON66 discovery");
  Serial.println("f=find gate; x=stop scan; p=status of identified gate; ?=help");
  Serial.println("Drive neutral; magnet OFF; crane undriven; GO ignored.");
  Serial.println("No gate motion, torque commands, factory reset or saved-setting changes.");
#endif
}
} // namespace

void setup() {
  motor_init(); motorStop(0);
  digitalWrite(PIN_MAGNET, LOW); pinMode(PIN_MAGNET, OUTPUT);
  pinMode(PIN_CRANE_SERVO, INPUT);
  Serial.begin(115200);
#ifdef GATE_POSITION_TEST
  foundId = benchId;
  GATE_SERIAL.begin(115200);
#endif
  const uint32_t start = millis();
  while (!Serial && millis() - start < 2000) {}
  help();
}

void loop() {
  while (Serial.available()) {
    switch (Serial.read()) {
#ifdef GATE_POSITION_TEST
      case 'c': moveGate(GATE_CLOSED_POS); break;
      case 'o': moveGate(GATE_OPEN_POS); break;
      case 'x': releaseGate("operator requested"); break;
      case 'p': report(); break;
#else
      case 'f': startScan(); break;
      case 'x': scanning = false; Serial.println("Scan stopped."); break;
      case 'p': if (!scanning) report(); else Serial.println("Scan in progress; x cancels."); break;
#endif
      case '?': help(); break;
      default: break;
    }
  }
#ifdef GATE_POSITION_TEST
  pollMovement();
#else
  if (scanning) scanStep();
#endif
}
#endif
