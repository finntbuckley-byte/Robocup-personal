/* ============================================================================
 *  imu_test.cpp  -  BNO055 bring-up (imutest env). Drives no actuators.
 *
 *      pio run -e imutest -t upload
 *      pio device monitor -b 115200        ('?' for the menu)
 *
 *  1. Boot: scans Wire and Wire1 for the BNO055 (0x28 / 0x29) and says where
 *     it is - should be Wire 0x28 (CON61). Then IMUPLUS mode.
 *  2. Leave the robot still ~5 s: gyro calibration (gC) should reach 3.
 *  3. 'z' to zero, then turn the robot ~90 deg CLOCKWISE (seen from above)
 *     by hand: 'hdg' should read about +90. About -90 -> set
 *     IMU_HEADING_SIGN = -1 in config.h.
 *  4. Turn it back to the start mark: hdg should return to ~0 (drift check).
 *     Leave it still for a minute and watch hdg for drift.
 * ============================================================================ */
#ifdef IMU_TEST

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include "config.h"

static const unsigned long PRINT_MS = 200;

static Adafruit_BNO055 *bno = nullptr;
static float zeroDeg = 0;

static bool probe(TwoWire &bus, uint8_t addr)
{
  bus.beginTransmission(addr);
  return bus.endTransmission() == 0;
}

static void scan(TwoWire &bus, const char *name)
{
  for (uint8_t a : { (uint8_t)0x28, (uint8_t)0x29 })
    if (probe(bus, a))
    {
      Serial.print("  found 0x"); Serial.print(a, HEX);
      Serial.print(" on "); Serial.println(name);
    }
}

// raw heading 0-360 from the BNO -> signed, zeroed, + = right, -180..180
static float headingDeg()
{
  sensors_event_t e;
  bno->getEvent(&e, Adafruit_BNO055::VECTOR_EULER);
  float h = IMU_HEADING_SIGN * (e.orientation.x - zeroDeg);
  while (h > 180) h -= 360;
  while (h < -180) h += 360;
  return h;
}

static void printHelp()
{
  Serial.println("\n========= imutest =========");
  Serial.println(" z  zero heading here");
  Serial.println(" ?  this menu");
  Serial.print  (" IMU_HEADING_SIGN = "); Serial.println(IMU_HEADING_SIGN);
  Serial.println(" hdg: + = turned RIGHT (clockwise from above) once the sign is right");
  Serial.println("===========================");
  Serial.println("ms\thdg\trawX\troll\tpitch\tgyroZ\tsC\tgC\taC\tmC");
}

void setup()
{
  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 2000) {}

  Wire.begin();  Wire.setClock(400000);
  Wire1.begin(); Wire1.setClock(400000);

  Serial.println("\nScanning for the BNO055...");
  scan(Wire, "Wire (RAW I2C0)");
  scan(Wire1, "Wire1 (RAW I2C1)");

  if (!probe(IMU_WIRE, IMU_ADDR))
  {
    Serial.println("!! Not at IMU_WIRE / IMU_ADDR from config.h - fix those (see scan above).");
    while (1) delay(1000);
  }

  static Adafruit_BNO055 dev(55, IMU_ADDR, &IMU_WIRE);
  bno = &dev;
  if (!bno->begin(OPERATION_MODE_IMUPLUS))
  {
    Serial.println("!! BNO055 answered but begin() failed");
    while (1) delay(1000);
  }
  delay(100);
  Serial.println("BNO055 ok, IMUPLUS mode. Keep still for gyro calibration (gC -> 3).");

  sensors_event_t e;
  bno->getEvent(&e, Adafruit_BNO055::VECTOR_EULER);
  zeroDeg = e.orientation.x;
  printHelp();
}

void loop()
{
  if (Serial.available())
  {
    char c = Serial.read();
    while (Serial.available()) Serial.read();
    if (c == 'z')
    {
      sensors_event_t e;
      bno->getEvent(&e, Adafruit_BNO055::VECTOR_EULER);
      zeroDeg = e.orientation.x;
      Serial.println("zeroed");
    }
    else if (c == '?') printHelp();
  }

  static unsigned long last = 0;
  if (millis() - last < PRINT_MS) return;
  last = millis();

  sensors_event_t eul, gyr;
  bno->getEvent(&eul, Adafruit_BNO055::VECTOR_EULER);
  bno->getEvent(&gyr, Adafruit_BNO055::VECTOR_GYROSCOPE);
  uint8_t sC, gC, aC, mC;
  bno->getCalibration(&sC, &gC, &aC, &mC);

  Serial.print(millis());                  Serial.print('\t');
  Serial.print(headingDeg(), 1);           Serial.print('\t');
  Serial.print(eul.orientation.x, 1);      Serial.print('\t');
  Serial.print(eul.orientation.y, 1);      Serial.print('\t');
  Serial.print(eul.orientation.z, 1);      Serial.print('\t');
  Serial.print(IMU_GYRO_SIGN * gyr.gyro.z * 57.2958f, 1);  Serial.print('\t');   // deg/s, + = right
  Serial.print(sC); Serial.print('\t');
  Serial.print(gC); Serial.print('\t');
  Serial.print(aC); Serial.print('\t');
  Serial.println(mC);
}

#endif // IMU_TEST
