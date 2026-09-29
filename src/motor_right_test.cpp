/* ============================================================================
 *  motor_right_test.cpp  —  isolated right-motor drive test
 *
 *  Build + upload:  pio run -e motortest -t upload
 *  Serial monitor:  pio device monitor -b 115200
 *
 *  Right motor = motor2, pin D1, DFR0513 PPM driver.
 *  1500 us = stop,  1950 us = full forward,  1050 us = full reverse.
 *
 *  Commands (Enter to send):
 *      f      forward at 50%
 *      b      reverse at 50%
 *      s      stop (neutral pulse)
 *      f 75   forward at 75%    (0–100)
 *      b 30   reverse at 30%
 *      r      ramp forward 0->100->0, 5% steps, watching for jitter
 *      ?      this menu
 * ============================================================================ */
#ifdef MOTOR_RIGHT_TEST

#include <Arduino.h>
#include <Servo.h>

#define RIGHT_MOTOR_PIN  1     // D1, motor2 on DFR0513 (CON65 / SERIAL1 TX)
#define NEUTRAL_US       1500
#define FULL_FWD_US      1950
#define FULL_REV_US      1050
#define LED_PIN          13

static Servo rightMotor;

static void setPct(int pct)
{
    // pct: -100 = full reverse, 0 = stop, +100 = full forward
    pct = constrain(pct, -100, 100);
    int us;
    if      (pct > 0) us = map(pct, 0, 100, NEUTRAL_US, FULL_FWD_US);
    else if (pct < 0) us = map(-pct, 0, 100, NEUTRAL_US, FULL_REV_US);
    else              us = NEUTRAL_US;
    rightMotor.writeMicroseconds(us);
    Serial.printf("[motor] %+4d%%  ->  %d us\n", pct, us);
}

static void printMenu()
{
    Serial.println(F("\n=== right motor test (D1, DFR0513 motor2) ==="));
    Serial.println(F("  f [pct]  forward (default 50%)"));
    Serial.println(F("  b [pct]  reverse (default 50%)"));
    Serial.println(F("  s        stop"));
    Serial.println(F("  r        ramp 0->100->0 forward (jitter check)"));
    Serial.println(F("  ?        this menu"));
}

void setup()
{
    pinMode(LED_PIN, OUTPUT);
    Serial.begin(115200);
    unsigned long t0 = millis();
    while (!Serial && millis() - t0 < 3000) {}

    rightMotor.attach(RIGHT_MOTOR_PIN);
    rightMotor.writeMicroseconds(NEUTRAL_US);
    delay(500);   // let the driver see the neutral pulse before anything else

    Serial.println(F("\n[motor_right_test] booted - right motor stopped"));
    printMenu();
}

void loop()
{
    digitalWrite(LED_PIN, (millis() / 500) & 1);

    if (!Serial.available()) return;
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) return;

    int sp  = line.indexOf(' ');
    String key = sp < 0 ? line : line.substring(0, sp);
    int    val = sp < 0 ? 50   : line.substring(sp + 1).toInt();
    key.toLowerCase();

    if      (key == "f") setPct(+val);
    else if (key == "b") setPct(-val);
    else if (key == "s") setPct(0);
    else if (key == "r")
    {
        Serial.println(F("[ramp] 0->100->0 forward, 5% steps, 150 ms each"));
        for (int p = 0; p <= 100; p += 5)  { setPct(p);  delay(150); }
        for (int p = 100; p >= 0; p -= 5)  { setPct(p);  delay(150); }
        setPct(0);
        Serial.println(F("[ramp] done"));
    }
    else if (key == "?" || key == "h") printMenu();
    else Serial.println(F("unknown command - '?' for menu"));
}

#endif // MOTOR_RIGHT_TEST
