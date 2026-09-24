/*#include <Wire.h>
#include <EEPROM.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire1);

const int EEPROM_ID_ADDR = 0;
const int EEPROM_CALIB_ADDR = sizeof(long);

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  Wire1.begin();
  if (!bno.begin(OPERATION_MODE_NDOF)) {
    Serial.println("BNO055 not detected — check wiring/address/bus");
    while (1) {}
  }
  delay(1000);

  // Try to restore a previously saved calibration for THIS sensor
  long storedID;
  EEPROM.get(EEPROM_ID_ADDR, storedID);
  sensor_t sensor;
  bno.getSensor(&sensor);

  if (storedID == sensor.sensor_id) {
    adafruit_bno055_offsets_t calibData;
    EEPROM.get(EEPROM_CALIB_ADDR, calibData);
    bno.setSensorOffsets(calibData);
    Serial.println("Restored saved calibration from EEPROM.");
  } else {
    Serial.println("No saved calibration for this sensor — calibrate manually, then send 's' to save.");
  }

  bno.setExtCrystalUse(true);  // must be called AFTER restoring offsets
  Serial.println("Ready. 'g' = 10s log, 's' = save current calibration.");
}

void loop() {
  if (!Serial.available()) return;
  char c = Serial.read();
  while (Serial.available()) Serial.read();

  if (c == 's' || c == 'S') {
    adafruit_bno055_offsets_t calibData;
    bno.getSensorOffsets(calibData);
    sensor_t sensor;
    bno.getSensor(&sensor);
    EEPROM.put(EEPROM_ID_ADDR, sensor.sensor_id);
    EEPROM.put(EEPROM_CALIB_ADDR, calibData);
    Serial.println("Calibration saved to EEPROM.");
  }

  if (c == 'g' || c == 'G') {
    uint8_t sys, gyro, accel, mag;
    bno.getCalibration(&sys, &gyro, &accel, &mag);
    Serial.print("Calib sys/gyro/accel/mag: ");
    Serial.print(sys); Serial.print(","); Serial.print(gyro); Serial.print(",");
    Serial.print(accel); Serial.print(","); Serial.println(mag);

    unsigned long start = millis();
    while (millis() - start < 10000) {
      sensors_event_t event;
      bno.getEvent(&event);
      Serial.println(event.orientation.x, 2);
      delay(100);
    }
    Serial.println("Done. 'g' = log again, 's' = re-save calibration.");
  }
}*/