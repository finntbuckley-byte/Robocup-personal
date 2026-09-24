#include "x8.h"
#include "config.h"
#include <Wire.h>
#include "DFRobot_MatrixLidar.h"

// ============================================================================
//  x8.cpp  -  SEN0628 8x8 matrix ToF.
//
//  LIBRARY GOTCHAS (DFRobot_MatrixLidar, from its source):
//    - setRangingMode() blocks for 5s after a successful mode change.
//    - getAllData() sends a request, then polls for the reply with
//      delay(17) steps: every read blocks ~20-40ms. Read at X8_READ_MS only.
//    - if the sensor stops answering, getAllData() blocks for its full 8s
//      receive timeout. So every read is preceded by an address probe, and a
//      missing sensor just leaves the frame stale (x8Fresh() == false) - the
//      nav falls back to a slow crawl rather than freezing for 8s.
// ============================================================================

static DFRobot_MatrixLidar_I2C x8(X8_ADDRESS, &X8_WIRE);

static uint16_t grid[64] = {0};
static bool initOk = false;
static unsigned long lastRead = 0;
static unsigned long lastFrame = 0;

static bool present()
{
  X8_WIRE.beginTransmission(X8_ADDRESS);
  return X8_WIRE.endTransmission() == 0;
}

void x8Init()
{
  if (x8.begin() != 0)
  {
    Serial.println("!! SEN0628 @0x33 not responding - check CON64 / X8_WIRE in config.h");
    return;
  }
  Serial.println("SEN0628 found - setting 8x8 mode (blocks 5s)...");
  if (x8.setRangingMode(eMatrix_8X8) != 0)
  {
    Serial.println("!! SEN0628 would not enter 8x8 mode");
    return;
  }
  initOk = true;
  Serial.println("SEN0628 8x8 ok");
}

void x8Update()
{
  if (!initOk || millis() - lastRead < X8_READ_MS) return;
  lastRead = millis();
  if (!present()) return;                    // don't risk the 8s library timeout
  uint16_t frame[64];
  if (x8.getAllData(frame) == 0)
  {
    memcpy(grid, frame, sizeof(grid));
    lastFrame = millis();
  }
}

bool x8Ok()    { return initOk; }
bool x8Fresh() { return initOk && lastFrame != 0 && millis() - lastFrame < X8_STALE_MS; }

// ---- geometry --------------------------------------------------------------

static inline uint16_t nearest(uint16_t a, uint16_t b)
{
  if (a == 0) return b;
  if (b == 0) return a;
  return (a < b) ? a : b;
}

// zone in ROBOT terms: row 0 = top of the field, col 0 = robot's far left
static uint16_t zone(uint8_t row, uint8_t col)
{
#if X8_ROW_FLIP
  row = 7 - row;
#endif
  if (X8_COL_SIGN < 0) col = 7 - col;
  uint16_t v = grid[row * 8 + col];
  return (v == 0 || v > X8_MAX_VALID_MM) ? 0 : v;
}

static uint16_t bandMin(uint8_t colLo, uint8_t colHi)
{
  if (!x8Fresh()) return 0;
  uint16_t best = 0;
  for (uint8_t r = X8_BAND_LO; r <= X8_BAND_HI; r++)
    for (uint8_t c = colLo; c <= colHi; c++)
      best = nearest(best, zone(r, c));
  return best;
}

uint16_t x8FrontMM() { return bandMin(0, 7); }
uint16_t x8LeftMM()  { return bandMin(0, 3); }
uint16_t x8RightMM() { return bandMin(4, 7); }

float x8BearingDeg(uint16_t *distOut)
{
  uint16_t best = 0;
  uint8_t bestCol = 4;
  if (x8Fresh())
    for (uint8_t r = X8_BAND_LO; r <= X8_BAND_HI; r++)
      for (uint8_t c = 0; c < 8; c++)
      {
        uint16_t v = zone(r, c);
        if (v != 0 && (best == 0 || v < best)) { best = v; bestCol = c; }
      }
  if (distOut) *distOut = best;
  return ((float)bestCol - 3.5f) * (X8_FOV_DEG / 8.0f);
}

// a wall fills (nearly) every column at a similar distance; a 50mm weight
// or the other robot's corner does not
bool x8WallAhead()
{
  if (!x8Fresh()) return false;
  uint16_t lo = 0, hi = 0;
  uint8_t seen = 0;
  for (uint8_t c = 0; c < 8; c++)
  {
    uint16_t colMin = bandMin(c, c);
    if (colMin == 0) continue;
    seen++;
    if (lo == 0 || colMin < lo) lo = colMin;
    if (colMin > hi) hi = colMin;
  }
  return seen >= 7 && (hi - lo) < X8_WALL_SPREAD_MM;
}

void x8PrintGrid()
{
  if (!initOk) { Serial.println("8x8 not initialised"); return; }
  Serial.print("--- 8x8 (mm, robot view, 0 = none) age ");
  Serial.print(millis() - lastFrame); Serial.println("ms ---");
  for (uint8_t r = 0; r < 8; r++)
  {
    Serial.print(r >= X8_BAND_LO && r <= X8_BAND_HI ? "*r" : " r");
    Serial.print(r); Serial.print(":");
    for (uint8_t c = 0; c < 8; c++) Serial.printf("%6u", zone(r, c));
    Serial.println();
  }
  uint16_t d = 0;
  float b = x8BearingDeg(&d);
  Serial.printf("band (*) nearest %umm @ %.1fdeg  L:%u R:%u  wall:%s\n",
                d, b, x8LeftMM(), x8RightMM(), x8WallAhead() ? "YES" : "no");
}
