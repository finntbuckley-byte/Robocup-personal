// Herkulex DRS-0101 rear flap. Raw protocol lifted from Servo_Setup.ino (proven
// on this servo 2026-09-30; current gate identified 2026-10-01 as ID 4,
// Serial2, 115200) - replaces the HerkulexServo
// library, whose init rebooted the servo and sent torque-on 100 ms later, before
// it had restarted (Servo_Setup waits 900 ms), so torque never came on.
//
// Packet: FF FF size pID CMD CS1 CS2 data[]   CS1 = (size^pID^CMD^data..) & 0xFE
// The bus echoes our own packets back; readAck() skips anything with CMD < 0x40.
#include <Arduino.h>
#include "gate.h"
#include "config.h"

#define CMD_RAM_WRITE  0x03
#define CMD_RAM_READ   0x04
#define CMD_S_JOG      0x06
#define CMD_STAT       0x07
#define RAM_STATUS_ERR 48
#define RAM_TORQUE     52
#define RAM_VOLTAGE    54
#define RAM_CAL_POS    58
#define TORQUE_ON      0x60
#define JOG_LED_GREEN  0x04     // SET byte: position mode + LED
#define JOG_LED_BLUE   0x08
#define REPLY_MS       25

static bool isOpen = false;
static bool found  = false;
static uint8_t missedChecks = 0;

static void sendPacket(uint8_t cmd, const uint8_t *data, uint8_t len)
{
  const uint8_t size = 7 + len;
  uint8_t cs1 = size ^ GATE_ID ^ cmd;
  for (uint8_t i = 0; i < len; i++) cs1 ^= data[i];
  cs1 &= 0xFE;
  const uint8_t cs2 = (uint8_t)(~cs1) & 0xFE;

  while (GATE_SERIAL.available()) GATE_SERIAL.read();
  GATE_SERIAL.write((uint8_t)0xFF); GATE_SERIAL.write((uint8_t)0xFF);
  GATE_SERIAL.write(size); GATE_SERIAL.write((uint8_t)GATE_ID); GATE_SERIAL.write(cmd);
  GATE_SERIAL.write(cs1);  GATE_SERIAL.write(cs2);
  for (uint8_t i = 0; i < len; i++) GATE_SERIAL.write(data[i]);
  GATE_SERIAL.flush();
}

// Bounded synchronous reply; caller polls only while drive is stopped.
static bool readAck(uint8_t wantCmd, uint8_t *pkt)
{
  const uint32_t t0 = millis();
  uint8_t state = 0, size = 0, idx = 0, buf[48];
  while (millis() - t0 < REPLY_MS)
  {
    if (!GATE_SERIAL.available()) continue;
    const uint8_t b = GATE_SERIAL.read();
    switch (state)
    {
      case 0: state = (b == 0xFF) ? 1 : 0; break;
      case 1: state = (b == 0xFF) ? 2 : 0; break;
      case 2:
        if (b == 0xFF) break; // retain header sync across extra FF bytes, as in the proven bench parser
        if (b < 7 || b > 40) { state = 0; break; }
        size = b; buf[0] = b; idx = 1; state = 3;
        break;
      default:
        buf[idx++] = b;
        if (idx < (uint8_t)(size - 2)) break;
        {
          const uint8_t pid = buf[1], cmd = buf[2];
          uint8_t calc = size ^ pid ^ cmd;
          for (uint8_t i = 5; i < (uint8_t)(size - 2); i++) calc ^= buf[i];
          calc &= 0xFE;
          const bool ok = calc == buf[3] && (uint8_t)((~calc) & 0xFE) == buf[4];
          if (ok && pid == GATE_ID && cmd == wantCmd && size >= 9)
          {
            for (uint8_t i = 0; i < (uint8_t)(size - 2); i++) pkt[i] = buf[i];
            return true;
          }
        }
        state = 0; idx = 0;
        break;
    }
  }
  return false;
}

static bool alive()
{
  uint8_t pkt[48];
  sendPacket(CMD_STAT, nullptr, 0);
  return readAck(CMD_STAT | 0x40, pkt);
}

static void ramWrite(uint8_t addr, const uint8_t *v, uint8_t n)
{
  uint8_t d[4] = { addr, n, 0, 0 };
  for (uint8_t i = 0; i < n && i < 2; i++) d[2 + i] = v[i];
  sendPacket(CMD_RAM_WRITE, d, 2 + n);
}

static int ramRead(uint8_t addr, uint8_t len)
{
  const uint8_t d[2] = { addr, len };
  uint8_t pkt[48];
  sendPacket(CMD_RAM_READ, d, 2);
  if (!readAck(CMD_RAM_READ | 0x40, pkt)) return -1;
  if(pkt[0] < 11+len || pkt[5]!=addr || pkt[6]!=len) return -1;
  return (len == 2) ? ((pkt[7] | (pkt[8] << 8)) & 0x03FF) : pkt[7];
}

static int gatePos(int pos)
{
  pos = constrain(pos, GATE_POS_MIN, GATE_POS_MAX);
#if GATE_REVERSE
  pos = GATE_POS_MIN + GATE_POS_MAX - pos;
#endif
  return pos;
}

static void jog(int pos, uint8_t led)
{
  pos = gatePos(pos);
  const uint8_t a[5] = { (uint8_t)GATE_PLAYTIME, (uint8_t)(pos & 0xFF), (uint8_t)(pos >> 8),
                         led, (uint8_t)GATE_ID };
  sendPacket(CMD_S_JOG, a, 5);
}

void gate_init(void)
{
  GATE_SERIAL.begin(GATE_BAUD);
  delay(20);
  found = alive();
  Serial.print("gate: ID "); Serial.print(GATE_ID);
  if (!found)
  {
    Serial.println(" NOT FOUND on Serial2 - check 7.5 V and the CON66 cable (run Servo_Setup.ino)");
    return;
  }
  const uint8_t zero[2] = { 0, 0 };
  ramWrite(RAM_STATUS_ERR, zero, 2);        // a latched fault keeps torque off
  delay(3);
  const uint8_t on = TORQUE_ON;
  ramWrite(RAM_TORQUE, &on, 1);
  delay(3);
  int v = ramRead(RAM_VOLTAGE, 1);
  Serial.print(" ok   supply "); Serial.print(v > 0 ? v * 0.074f : 0.0f, 1);
  Serial.print(" V   pos "); Serial.println(gate_readPos());
  if (GATE_MOVE_ON_INIT) gate_close();
  // else: torque on, holding wherever it is - positions not verified yet
}

void gate_update(void) {
  static uint32_t lastCheck=0;
  if(!GATE_DELIVERY_ENABLED || millis()-lastCheck<(found?500u:2000u)) return;
  lastCheck=millis();
  if(!alive()) {
    if(missedChecks<3) ++missedChecks;
    if(missedChecks>=3) found=false;
    return;
  }
  missedChecks=0;
  if(!found) {
    found=true;
    // Reconnect without rebooting the servo or opening a gate unexpectedly.
    const uint8_t zero[2]={0,0},on=TORQUE_ON;
    ramWrite(RAM_STATUS_ERR,zero,2);
    ramWrite(RAM_TORQUE,&on,1);
    gate_close();
    Serial.println("gate: communication recovered; closing gate");
  }
}

void gate_open(void)  { if (found) {jog(GATE_OPEN_POS, JOG_LED_GREEN); isOpen = true;} }
void gate_close(void) { if (found) {jog(GATE_CLOSED_POS, JOG_LED_BLUE); isOpen = false;} }
void gate_moveTo(int pos) { if (found) jog(pos, JOG_LED_GREEN); }

bool gate_isOpen(void) { return isOpen; }
bool gate_found(void)  { return found; }
int  gate_readPos(void) { return found ? ramRead(RAM_CAL_POS, 2) : -1; }
