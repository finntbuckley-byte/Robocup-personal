// TCS34725 base detection for nav. Raw registers instead of the Adafruit
// library: its getRawData() delays a full integration time every call, which
// would stall the loop. Here the chip free-runs (PON|AEN) and we just read the
// result registers whenever AVALID says a new one is ready.
//
// Classification (colourtest captures 30/9, see config.h):
//   G/R >= COLOUR_BASE_GR_MIN  -> on a base; then B/G >= COLOUR_BLUE_BG_MIN -> blue
//   else                       -> floor (black floor and the base rim both land here)
//   C < COLOUR_MIN_CLEAR       -> unknown (too dark / blocked)
#include <Arduino.h>
#include <Wire.h>
#include "colour.h"
#include "colour_logic.h"
#include "config.h"

#define TCS_CMD      0x80
#define TCS_AUTOINC  0x20
#define REG_ENABLE   0x00
#define REG_ATIME    0x01
#define REG_CONTROL  0x0F
#define REG_ID       0x12
#define REG_STATUS   0x13
#define REG_CDATAL   0x14
#define EN_PON       0x01
#define EN_AEN       0x02
#define ST_AVALID    0x01

static bool found = false;
static uint16_t cC = 0, cR = 0, cG = 0, cB = 0;
static ColourSurface lastRaw = COLOUR_UNKNOWN, stable = COLOUR_UNKNOWN, home = COLOUR_UNKNOWN;
static uint8_t runLen = 0;
static unsigned long lastPoll = 0, lastData = 0;
static unsigned long homeSince = 0;      // 0 = not on our base colour right now

static bool write8(uint8_t reg, uint8_t v)
{
  COLOUR_WIRE.beginTransmission(COLOUR_ADDRESS);
  COLOUR_WIRE.write(TCS_CMD | reg);
  COLOUR_WIRE.write(v);
  return COLOUR_WIRE.endTransmission() == 0;
}

static bool readBlock(uint8_t reg, uint8_t *buf, uint8_t n)
{
  COLOUR_WIRE.beginTransmission(COLOUR_ADDRESS);
  COLOUR_WIRE.write(TCS_CMD | TCS_AUTOINC | reg);
  if (COLOUR_WIRE.endTransmission(false) != 0) return false;
  if (COLOUR_WIRE.requestFrom(COLOUR_ADDRESS, n) != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = COLOUR_WIRE.read();
  return true;
}

static uint8_t gainCode(int x) { return x >= 60 ? 3 : x >= 16 ? 2 : x >= 4 ? 1 : 0; }

static ColourSurface classify(uint16_t c, uint16_t r, uint16_t g, uint16_t b)
{
  return classifyColour(c,r,g,b,COLOUR_MIN_CLEAR,COLOUR_BASE_GR_MIN,COLOUR_BLUE_BG_MIN);
}

bool colourInit(void)
{
  uint8_t id = 0;
  found = readBlock(REG_ID, &id, 1) && (id == 0x44 || id == 0x4D || id == 0x10);
  Serial.print("colour: TCS34725 ");
  if (!found) { Serial.println("NOT FOUND on Wire1 0x29 - collection and delivery inhibited"); return false; }
  if(!write8(REG_ATIME, (uint8_t)(256 - (COLOUR_INTEG_MS * 10 + 12) / 24)) ||
     !write8(REG_CONTROL, gainCode(COLOUR_GAIN_X)) || !write8(REG_ENABLE, EN_PON)) {
      found=false; return false;
  }
  delay(3);                                                              // datasheet: 2.4 ms after PON
  if(!write8(REG_ENABLE, EN_PON | EN_AEN)) {found=false;return false;}
  Serial.print("ok ("); Serial.print(COLOUR_INTEG_MS); Serial.print(" ms, ");
  Serial.print(COLOUR_GAIN_X); Serial.println("x)");
  lastData = millis();
  return true;
}

void colourUpdate(void)
{
  if (!found || millis() - lastPoll < COLOUR_READ_MS) return;
  lastPoll = millis();

  uint8_t st = 0, d[8];
  if (readBlock(REG_STATUS, &st, 1) && (st & ST_AVALID) && readBlock(REG_CDATAL, d, 8))
  {
    cC = d[0] | (d[1] << 8); cR = d[2] | (d[3] << 8);
    cG = d[4] | (d[5] << 8); cB = d[6] | (d[7] << 8);
    lastData = millis();

    ColourSurface s = classify(cC, cR, cG, cB);
    if (s == lastRaw) { if (runLen < 255) runLen++; }
    else              { lastRaw = s; runLen = 1; }
    if (runLen >= COLOUR_CONFIRM_READS) stable = lastRaw;
  }
  if (millis() - lastData > COLOUR_STALE_MS) { stable = COLOUR_UNKNOWN; runLen = 0; }

  // how long we've been continuously on our own base colour
  if (colourOnHomeBase()) { if (homeSince == 0) homeSince = millis(); }
  else                    homeSince = 0;
}

unsigned long colourOnHomeForMs(void) { return homeSince ? millis() - homeSince : 0; }

bool colourOk(void) { return found && millis() - lastData <= COLOUR_STALE_MS; }
ColourSurface colourSurface(void) {
  // Revoke floor/home authorization on the first conflicting sample; require
  // the full confirmation run before authorizing the new surface.
  return colourOk() && stable==lastRaw ? stable : COLOUR_UNKNOWN;
}

const char *colourName(ColourSurface s)
{
  switch (s)
  {
    case COLOUR_FLOOR: return "floor";
    case COLOUR_GREEN: return "green";
    case COLOUR_BLUE:  return "blue";
    default:           return "?";
  }
}

void colourCaptureHome(void)
{
  ColourSurface s = colourSurface();
  home = (s == COLOUR_GREEN || s == COLOUR_BLUE) ? s : COLOUR_UNKNOWN;
  Serial.print(">>> home base colour: ");
  if (home != COLOUR_UNKNOWN) Serial.println(colourName(home));
  else
  {
    Serial.print("NOT DETECTED (reads "); Serial.print(colourName(s));
    Serial.println(") - automatic delivery unavailable");
  }
}

ColourSurface colourHome(void) { return home; }
bool colourGatingOn(void)    { return USE_COLOUR_GATING && found && home != COLOUR_UNKNOWN; }
bool colourOnHomeBase(void)  { return colourGatingOn() && colourSurface() == home; }

bool colourOnEnemyBase(void)
{
  ColourSurface s = colourSurface();
  return colourGatingOn() && (s == COLOUR_GREEN || s == COLOUR_BLUE) && s != home;
}

void colourPrintRaw(void)
{
  Serial.print("COLOUR\tC "); Serial.print(cC); Serial.print("\tR "); Serial.print(cR);
  Serial.print("\tG "); Serial.print(cG); Serial.print("\tB "); Serial.print(cB);
  if (cR && cG)
  {
    Serial.print("\tG/R "); Serial.print((float)cG / cR, 2);
    Serial.print("\tB/G "); Serial.print((float)cB / cG, 2);
  }
  Serial.print("\t-> "); Serial.print(colourName(colourSurface()));
  Serial.print("\thome "); Serial.print(colourName(home));
  Serial.println(colourOk() ? "" : "\t(sensor not up)");
}
