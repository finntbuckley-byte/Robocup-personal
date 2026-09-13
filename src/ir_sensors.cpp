#include "ir_sensors.h"
#include "config.h"
#include <math.h>

// ============================================================================
//  ir_sensors.cpp  -  2x side analog IR (GP2Y0A41SK), wall/scrape nudge.
//
//  CALIBRATION NOTE (applies to every analog IR sensor in this project):
//  Vout-to-distance is a curve, not a line, and varies unit to unit. The
//  constants in config.h are a published starting point, not ground truth -
//  check a few known distances against the printed telemetry and adjust if
//  they're off. These sensors are also non-monotonic below their rated
//  minimum range (a very close object can misreport as far) - don't trust a
//  sudden "clear" reading right after a "very close" one.
// ============================================================================

uint16_t irSideLMM = 0, irSideRMM = 0;

static unsigned long irLastRead = 0;

static float irReadVolts(int pin)
{
  long sum = 0;
  for (int i = 0; i < IR_SAMPLES; i++) sum += analogRead(pin);
  float counts = (float)sum / IR_SAMPLES;
  return counts * (IR_ADC_VREF / IR_ADC_COUNTS);
}

// volts -> mm using the power-law fit, clamped to the sensor's rated band.
// Outside the band (or a near-zero volts reading) we can't trust the number,
// so we report 0 ("clear") rather than a wrong distance.
static uint16_t irVoltsToMM(float volts, float A, float B, int minMM, int maxMM)
{
  if (volts < 0.10f) return 0;
  float mm = A * pow(volts, B);
  if (mm < minMM || mm > maxMM) return 0;
  return (uint16_t)mm;
}

void irSensorsInit()
{
#if USE_SIDE_IR
  pinMode(PIN_IR_SIDE_L, INPUT);
  pinMode(PIN_IR_SIDE_R, INPUT);
#endif
}

void irSensorsUpdate()
{
  if (millis() - irLastRead < IR_READ_MS) return;
  irLastRead = millis();

#if USE_SIDE_IR
  irSideLMM = irVoltsToMM(irReadVolts(PIN_IR_SIDE_L), IR_SIDE_A, IR_SIDE_B,
                          IR_SIDE_MIN_MM, IR_SIDE_MAX_MM);
  irSideRMM = irVoltsToMM(irReadVolts(PIN_IR_SIDE_R), IR_SIDE_A, IR_SIDE_B,
                          IR_SIDE_MIN_MM, IR_SIDE_MAX_MM);
#endif
}

bool sideNearLeft()  { return irSideLMM > 0 && irSideLMM < SIDE_NEAR_MM; }
bool sideNearRight() { return irSideRMM > 0 && irSideRMM < SIDE_NEAR_MM; }
