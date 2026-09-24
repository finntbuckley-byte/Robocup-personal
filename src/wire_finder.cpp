/* ============================================================================
 *  wire_finder.cpp  -  "what is plugged in where" bring-up sketch
 *
 *  Build + upload:  pio run -e wirefind -t upload
 *  Serial monitor:  pio device monitor -b 115200     (press '?' for the menu)
 *
 *  Only compiled when WIRE_FIND is defined (the wirefind env sets it), so it
 *  never clashes with main.cpp's setup()/loop() in the normal build.
 *
 *  Drives NOTHING except: SX1509 @0x3F bank A (XSHUT0-7), the TCA9548 channel
 *  select, and - only on the 'u' command - short trigger pulses on the RAW
 *  digital ports. Motor pins (0/1), crane servo (28) and magnets (26/27) are
 *  never touched, so the robot stays still.
 *
 *  What it works out (board map = RobocupMiniSchematic.pdf):
 *    1. I2C devices on Wire (RAW I2C0, CON59-61) and Wire1 (RAW I2C1,
 *       CON62-64), identified by chip-ID register where possible.
 *    2. Every TCA9548 channel (CON45-52 = ch0-7) if the mux answers.
 *    3. For XSHUT0-7 (ToF connectors CON27-34): raise one line at a time,
 *       see which bus 0x29 appears on, and whether it's a VL53L0X or L1X.
 *       A 0x29 that answers with ALL XSHUTs low is not a ToF on the XSHUT
 *       chain (TCS34725, or a ToF with no XSHUT line).
 *    4. 't' streams every ToF found in step 3 so you can cover each one by
 *       hand and write down which physical sensor it is.
 *    5. 'p' streams every analog port + RAW digital pin - wave a hand at a
 *       side IR, a steel weight at the inductive sensor, and see which moves.
 *    6. 'u' probes trigger/echo pairs on the RAW digital ports for an HC-SR04.
 * ============================================================================ */
#ifdef WIRE_FIND

#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <VL53L1X.h>

// ---- addresses (CLAUDE.md section 4) ---------------------------------------
static const uint8_t ADDR_TOF_DEFAULT = 0x29;
static const uint8_t ADDR_SX_XSHUT    = 0x3F;
static const uint8_t ADDR_MUX         = 0x70;
static const uint8_t TOF_NEW_BASE     = 0x34;   // clear of the SEN0628 at 0x33

// SX1509 registers (bank A = I/O 0-7 = XSHUT0-7)
static const uint8_t SX_REG_DIR_A  = 0x0F;
static const uint8_t SX_REG_DATA_A = 0x11;

// ---- pins, per the schematic's RAW connectors -------------------------------
struct PinInfo { uint8_t pin; const char *port; };

static const PinInfo ANALOG_PORTS[] = {
  { 14, "A0Z  CON68" }, { 15, "A1Z  CON69" }, { 20, "A6Z  CON70" },
  { 21, "A7Z  CON71" }, { 22, "A8Z  CON23" }, { 23, "A9Z  CON24" },
  { 24, "A10Z CON72" }, { 25, "A11Z CON73" },
  // A12Z/A13Z (26/27) are the magnet FET outputs in collection.cpp - left out
};
static const PinInfo DIGITAL_PORTS[] = {
  {  2, "D2  CON55" }, {  3, "D3  CON55" }, {  4, "D4  CON55" }, {  5, "D5  CON55" },
  { 30, "D30 CON54" }, { 31, "D31 CON54" }, { 32, "D32 CON54" }, { 33, "D33 CON54" },
};
static const int N_ANALOG  = sizeof(ANALOG_PORTS)  / sizeof(ANALOG_PORTS[0]);
static const int N_DIGITAL = sizeof(DIGITAL_PORTS) / sizeof(DIGITAL_PORTS[0]);

static const unsigned long STREAM_MS = 200;

// ---- state -------------------------------------------------------------------
enum TofKind { TOF_NONE, TOF_L0X, TOF_L1X };
struct TofSlot { TofKind kind; TwoWire *bus; bool running; uint16_t mm; };
static TofSlot tofSlot[8];
static VL53L0X l0x[8];
static VL53L1X l1x[8];

static TwoWire *sxBus  = nullptr;   // bus the XSHUT SX1509 was found on
static TwoWire *muxBus = nullptr;

enum Mode { MODE_IDLE, MODE_TOF, MODE_PINS };
static Mode mode = MODE_IDLE;

// ============================================================================
//  I2C helpers
// ============================================================================

static const char *busName(TwoWire *b) { return b == &Wire ? "Wire (I2C0)" : "Wire1(I2C1)"; }

static bool i2cPresent(TwoWire &b, uint8_t addr)
{
  b.beginTransmission(addr);
  return b.endTransmission() == 0;
}

static bool readReg8(TwoWire &b, uint8_t addr, uint8_t reg, uint8_t &out)
{
  b.beginTransmission(addr);
  b.write(reg);
  if (b.endTransmission(false) != 0) return false;
  if (b.requestFrom(addr, (uint8_t)1) != 1) return false;
  out = b.read();
  return true;
}

static bool readReg16Idx(TwoWire &b, uint8_t addr, uint16_t reg, uint8_t &out)
{
  b.beginTransmission(addr);
  b.write((uint8_t)(reg >> 8));
  b.write((uint8_t)(reg & 0xFF));
  if (b.endTransmission(false) != 0) return false;
  if (b.requestFrom(addr, (uint8_t)1) != 1) return false;
  out = b.read();
  return true;
}

static void writeReg8(TwoWire &b, uint8_t addr, uint8_t reg, uint8_t val)
{
  b.beginTransmission(addr);
  b.write(reg);
  b.write(val);
  b.endTransmission();
}

static void muxSelect(uint8_t mask)
{
  if (!muxBus) return;
  muxBus->beginTransmission(ADDR_MUX);
  muxBus->write(mask);
  muxBus->endTransmission();
}

// Check L0X first: its ID read is a plain 8-bit index. The L1X check writes a
// 2-byte index, which an L0X would take as a register write.
static TofKind identifyTof(TwoWire &b)
{
  uint8_t id = 0;
  if (readReg8(b, ADDR_TOF_DEFAULT, 0xC0, id) && id == 0xEE) return TOF_L0X;
  uint8_t hi = 0, lo = 0;
  if (readReg16Idx(b, ADDR_TOF_DEFAULT, 0x010F, hi) &&
      readReg16Idx(b, ADDR_TOF_DEFAULT, 0x0110, lo) && hi == 0xEA && lo == 0xCC)
    return TOF_L1X;
  return TOF_NONE;
}

// best-guess name for an address, confirmed by chip-ID where one exists
static const char *identify(TwoWire &b, uint8_t addr)
{
  uint8_t v = 0;
  switch (addr) {
    case 0x28:
      if (readReg8(b, addr, 0x00, v) && v == 0xA0) return "BNO055 IMU (chip ID ok)";
      return "0x28 - expected BNO055, chip ID mismatch";
    case 0x29:
      if (readReg8(b, addr, 0x00, v) && v == 0xA0) return "BNO055 IMU on ALT address";
      if (readReg8(b, addr, 0x80 | 0x12, v) && (v == 0x44 || v == 0x4D))
        return "TCS34725 colour sensor (ID ok)";
      if (readReg8(b, addr, 0xC0, v) && v == 0xEE) return "VL53L0X (not held in XSHUT)";
      return "0x29 - ToF or TCS34725, unidentified";
    case 0x33: return "SEN0628 8x8 ToF (by address)";
    case 0x3C: return "OLED";
    case 0x3E: return "SX1509 - limit switch / AIO expander";
    case 0x3F: return "SX1509 - XSHUT / BIO expander";
    case 0x70: return "TCA9548 mux";
    case 0x71: return "add-on ToF expander";
    default:
      if (addr >= 0x30 && addr <= 0x38) return "re-addressed ToF (left from a previous run?)";
      return "unknown";
  }
}

static int scanBus(TwoWire &b, const char *label, bool skipMux)
{
  int found = 0;
  for (uint8_t a = 0x08; a < 0x78; a++) {
    if (skipMux && a == ADDR_MUX) continue;
    if (!i2cPresent(b, a)) continue;
    Serial.printf("  %-18s 0x%02X  %s\n", label, a, identify(b, a));
    found++;
  }
  return found;
}

// ============================================================================
//  SX1509 XSHUT control (raw registers - no SparkFun lib needed)
// ============================================================================

static void xshutWriteAll(uint8_t mask)
{
  if (sxBus) writeReg8(*sxBus, ADDR_SX_XSHUT, SX_REG_DATA_A, mask);
}

static void xshutInit()
{
  if (!sxBus) return;
  writeReg8(*sxBus, ADDR_SX_XSHUT, SX_REG_DATA_A, 0x00);   // data low first...
  writeReg8(*sxBus, ADDR_SX_XSHUT, SX_REG_DIR_A,  0x00);   // ...then bank A = outputs
}

// ============================================================================
//  REPORT
// ============================================================================

static void findExpanders()
{
  sxBus = muxBus = nullptr;
  TwoWire *buses[2] = { &Wire, &Wire1 };
  for (TwoWire *b : buses) {
    if (!sxBus  && i2cPresent(*b, ADDR_SX_XSHUT)) sxBus  = b;
    if (!muxBus && i2cPresent(*b, ADDR_MUX))      muxBus = b;
  }
}

static void reportI2C()
{
  Serial.println("\n=== 1. RAW I2C buses (mux channels disabled) ===");
  muxSelect(0x00);
  xshutWriteAll(0x00);            // all ToFs in reset so only non-XSHUT parts show
  delay(20);
  if (scanBus(Wire,  "Wire (I2C0)", false) == 0) Serial.println("  Wire (I2C0)        nothing");
  if (scanBus(Wire1, "Wire1(I2C1)", false) == 0) Serial.println("  Wire1(I2C1)        nothing");
  Serial.println("  (0x29 here = TCS34725 or a ToF with no XSHUT line, see ID)");
}

static void reportMux()
{
  Serial.println("\n=== 2. TCA9548 channels (CON45-52 = ch0-7) ===");
  if (!muxBus) { Serial.println("  mux 0x70 not found on either bus"); return; }
  Serial.printf("  mux is on %s\n", busName(muxBus));
  xshutWriteAll(0x00);
  for (uint8_t ch = 0; ch < 8; ch++) {
    muxSelect(1 << ch);
    delay(5);
    int n = 0;
    for (uint8_t a = 0x08; a < 0x78; a++) {
      if (a == ADDR_MUX) continue;
      if (!i2cPresent(*muxBus, a)) continue;
      // skip devices that answer with the mux closed - they're on the parent bus
      muxSelect(0x00);
      bool onParent = i2cPresent(*muxBus, a);
      muxSelect(1 << ch);
      if (onParent) continue;
      Serial.printf("  ch%u (CON%u)       0x%02X  %s\n", ch, 45 + ch, a, identify(*muxBus, a));
      n++;
    }
    if (n == 0) Serial.printf("  ch%u (CON%u)       empty\n", ch, 45 + ch);
  }
  muxSelect(0x00);
}

static void reportXshut()
{
  Serial.println("\n=== 3. XSHUT chain (ToF connectors CON27-34) ===");
  for (int i = 0; i < 8; i++) { tofSlot[i] = { TOF_NONE, nullptr, false, 0 }; }

  if (!sxBus) { Serial.println("  SX1509 0x3F not found - can't sequence XSHUT"); return; }
  Serial.printf("  SX1509 0x3F is on %s\n", busName(sxBus));
  xshutInit();
  muxSelect(0x00);
  delay(20);

  bool stray[2] = { i2cPresent(Wire, ADDR_TOF_DEFAULT), i2cPresent(Wire1, ADDR_TOF_DEFAULT) };

  Serial.println("  XSHUT  CON    bus           type");
  for (int x = 0; x < 8; x++) {
    xshutWriteAll(1 << x);
    delay(50);                                 // ToF boot time
    TwoWire *hit = nullptr;
    if (!stray[0] && i2cPresent(Wire,  ADDR_TOF_DEFAULT)) hit = &Wire;
    if (!stray[1] && i2cPresent(Wire1, ADDR_TOF_DEFAULT)) hit = &Wire1;

    if (!hit) { Serial.printf("  %d      CON%d  -             empty\n", x, 27 + x); continue; }
    TofKind k = identifyTof(*hit);
    tofSlot[x].kind = k;
    tofSlot[x].bus  = hit;
    Serial.printf("  %d      CON%d  %s  %s\n", x, 27 + x, busName(hit),
                  k == TOF_L0X ? "VL53L0X (short)" : k == TOF_L1X ? "VL53L1X (long)" : "?? no ID");
  }
  xshutWriteAll(0x00);
  if (stray[0] || stray[1])
    Serial.println("  NOTE: 0x29 answered with every XSHUT low - that bus was skipped above;"
                   " unplug the TCS34725 / un-XSHUT'd ToF from it to test that bus.");
}

static void reportPins()
{
  Serial.println("\n=== 4. Port snapshot (12-bit analog, digital w/ pull-up) ===");
  for (int i = 0; i < N_ANALOG; i++)
    Serial.printf("  %-11s pin %2u  %4d\n", ANALOG_PORTS[i].port, ANALOG_PORTS[i].pin,
                  analogRead(ANALOG_PORTS[i].pin));
  for (int i = 0; i < N_DIGITAL; i++)
    Serial.printf("  %-11s pin %2u  %s\n", DIGITAL_PORTS[i].port, DIGITAL_PORTS[i].pin,
                  digitalRead(DIGITAL_PORTS[i].pin) ? "HIGH" : "LOW");
  Serial.println("  floating analog ~ random; side IR ~ 300-2500; inductive ~0 or ~4095");
}

static void fullReport()
{
  findExpanders();
  reportI2C();
  reportMux();
  reportXshut();
  reportPins();
  Serial.println("\n'?' for commands");
}

// ============================================================================
//  STREAMS
// ============================================================================

static void tofStart()
{
  if (!sxBus) { Serial.println("no SX1509 - run 'r' first"); return; }
  xshutInit();
  xshutWriteAll(0x00);
  delay(10);
  uint8_t mask = 0;
  int n = 0;
  for (int x = 0; x < 8; x++) {
    TofSlot &s = tofSlot[x];
    s.running = false;
    if (s.kind == TOF_NONE) continue;
    mask |= (1 << x);
    xshutWriteAll(mask);
    delay(50);
    if (s.kind == TOF_L0X) {
      l0x[x].setBus(s.bus);
      l0x[x].setTimeout(100);
      if (l0x[x].init()) {
        l0x[x].setAddress(TOF_NEW_BASE + x);
        l0x[x].startContinuous(50);
        s.running = true;
      }
    } else {
      l1x[x].setBus(s.bus);
      l1x[x].setTimeout(100);
      if (l1x[x].init()) {
        l1x[x].setAddress(TOF_NEW_BASE + x);
        l1x[x].setDistanceMode(VL53L1X::Long);
        l1x[x].startContinuous(50);
        s.running = true;
      }
    }
    Serial.printf("  XSHUT%d %s @0x%02X %s\n", x, s.kind == TOF_L0X ? "L0X" : "L1X",
                  TOF_NEW_BASE + x, s.running ? "running" : "INIT FAILED");
    n += s.running;
  }
  if (n == 0) { Serial.println("no ToFs to stream - run 'r' first"); return; }
  Serial.println("cover each sensor in turn (hand ~50mm away), note which column drops."
                 " any key = stop");
  Serial.print("ms");
  for (int x = 0; x < 8; x++) if (tofSlot[x].running) Serial.printf("\tX%d", x);
  Serial.println();
  mode = MODE_TOF;
}

static void tofStream()
{
  static unsigned long last = 0;
  for (int x = 0; x < 8; x++) {
    TofSlot &s = tofSlot[x];
    if (!s.running) continue;
    if (s.kind == TOF_L1X && l1x[x].dataReady()) s.mm = l1x[x].read(false);
    if (s.kind == TOF_L0X && millis() - last >= STREAM_MS) {
      uint16_t mm = l0x[x].readRangeContinuousMillimeters();
      s.mm = (mm >= 8000) ? 0 : mm;
    }
  }
  if (millis() - last < STREAM_MS) return;
  last = millis();
  Serial.print(millis());
  for (int x = 0; x < 8; x++) if (tofSlot[x].running) Serial.printf("\t%u", tofSlot[x].mm);
  Serial.println();
}

static void pinsStart()
{
  Serial.println("wave a hand at each side IR, hold steel at the inductive sensor,"
                 " trip the GO button. any key = stop");
  Serial.print("ms");
  for (int i = 0; i < N_ANALOG; i++)  Serial.printf("\tA%u", ANALOG_PORTS[i].pin);
  for (int i = 0; i < N_DIGITAL; i++) Serial.printf("\tD%u", DIGITAL_PORTS[i].pin);
  Serial.println();
  mode = MODE_PINS;
}

static void pinsStream()
{
  static unsigned long last = 0;
  if (millis() - last < STREAM_MS) return;
  last = millis();
  Serial.print(millis());
  for (int i = 0; i < N_ANALOG; i++)  Serial.printf("\t%d", analogRead(ANALOG_PORTS[i].pin));
  for (int i = 0; i < N_DIGITAL; i++) Serial.printf("\t%d", digitalRead(DIGITAL_PORTS[i].pin));
  Serial.println();
}

// HC-SR04: try every trig->echo pair inside each 4-pin RAW digital group.
// Echo must come back through a 5V->3.3V level shift - check that first.
static void ultrasoundProbe()
{
  Serial.println("\n=== HC-SR04 probe (point it at something ~20-50cm away) ===");
  bool any = false;
  for (int g = 0; g < 2; g++) {
    for (int t = 0; t < 4; t++) {
      for (int e = 0; e < 4; e++) {
        if (t == e) continue;
        uint8_t trig = DIGITAL_PORTS[g * 4 + t].pin, echo = DIGITAL_PORTS[g * 4 + e].pin;
        pinMode(echo, INPUT);
        pinMode(trig, OUTPUT);
        digitalWrite(trig, LOW);  delayMicroseconds(4);
        digitalWrite(trig, HIGH); delayMicroseconds(10);
        digitalWrite(trig, LOW);
        unsigned long us = pulseIn(echo, HIGH, 30000);
        pinMode(trig, INPUT_PULLUP);
        pinMode(echo, INPUT_PULLUP);
        if (us > 0) {
          Serial.printf("  trig D%u -> echo D%u : %lu us = %lu mm\n", trig, echo, us, us * 343 / 2000);
          any = true;
        }
        delay(60);                             // let the last echo die away
      }
    }
  }
  if (!any) Serial.println("  no echo on any RAW digital pair - it's elsewhere (or unpowered / 5V echo not shifted)");
}

// ============================================================================
//  SETUP / LOOP
// ============================================================================

static void printHelp()
{
  Serial.println("\n========= wire finder =========");
  Serial.println(" r  full report (I2C, mux, XSHUT map, port snapshot)");
  Serial.println(" t  stream every ToF found (identify positions)");
  Serial.println(" p  stream analog ports + RAW digital pins");
  Serial.println(" u  probe RAW digital ports for an HC-SR04");
  Serial.println(" ?  this menu      any key stops a stream");
  Serial.println("===============================");
}

void setup()
{
  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 3000) {}

  analogReadResolution(12);
  for (int i = 0; i < N_DIGITAL; i++) pinMode(DIGITAL_PORTS[i].pin, INPUT_PULLUP);

  Wire.begin();  Wire.setClock(100000);        // slow + safe for scanning
  Wire1.begin(); Wire1.setClock(100000);

  Serial.println("\n--- RoboCup G23 wire finder ---");
  delay(200);
  fullReport();
}

void loop()
{
  if (Serial.available()) {
    char c = Serial.read();
    while (Serial.available()) Serial.read();
    if (mode != MODE_IDLE) {
      mode = MODE_IDLE;
      Serial.println("--- stopped");
      return;
    }
    switch (c) {
      case 'r': fullReport();      break;
      case 't': tofStart();        break;
      case 'p': pinsStart();       break;
      case 'u': ultrasoundProbe(); break;
      case '?': printHelp();       break;
      default: break;
    }
  }

  if (mode == MODE_TOF)  tofStream();
  if (mode == MODE_PINS) pinsStream();
}

#endif // WIRE_FIND
