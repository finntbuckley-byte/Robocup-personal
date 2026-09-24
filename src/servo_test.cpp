/* ============================================================================
 *  servo_test.cpp  -  isolated bench test for the collection "big servo"
 *
 *  Build + upload:  pio run -e servotest -t upload
 *  Serial monitor:  pio device monitor -b 115200     (press '?' for the menu)
 *
 *  Only compiled when SERVO_TEST is defined (the servotest env sets it), so it
 *  never clashes with main.cpp's setup()/loop() in the normal build.
 *
 *  Reproduces the exact moves collection.cpp makes on pin 28:
 *      PICKUP        -> write(110), dwell SERVODELAY1
 *      MOVE_TO_DROP  -> cosine-eased ramp 116 -> -5 over 800 ms, dwell SERVODELAY2
 *      DROP          -> 1000 ms pause + WEIGHTDROPDELAY (magnets would drop here)
 *      FINISHED      -> write(AT_REST = 70), dwell SERVODELAY3
 *
 *  Magnet pins are held LOW the whole time - this rig only moves the servo.
 *
 *  Commands (end each line with Enter):
 *      ?          menu
 *      c          one full collection cycle (same timings as collection.cpp)
 *      l          loop full cycles until any key is pressed
 *      r          rest     (70 deg)
 *      p          pickup   (110 deg)
 *      d          eased drop ramp only (116 -> -5, clamps to 0)
 *      s          slow sweep 0 -> 116 -> 0, 1 deg steps (range / binding check)
 *      x          detach   (stop pulses, servo goes limp)
 *      a          re-attach
 *      0..180     go to that angle
 *      500..2500  raw pulse width in microseconds (bypasses angle mapping)
 * ============================================================================ */
#ifdef SERVO_TEST

#include <Arduino.h>
#include <Servo.h>

// --- copied from collection.cpp - keep in sync -------------------------------
#define MAG1 26
#define MAG2 27
#define BIG_SERVO 28

#define SERVODELAY1 1200
#define SERVODELAY2 900
#define SERVODELAY3 1500
#define WEIGHTDROPDELAY 1000

#define PICKUP_ANGLE             110
#define MOVE_TO_DROP_START_ANGLE 116
#define MOVE_TO_DROP_END_ANGLE   -5
#define MOVE_TO_DROP_RAMP_MS     800
#define AT_REST                  70
// -----------------------------------------------------------------------------

#define LED_PIN 13
#define SERVO_MIN_US 544   // Servo library defaults, stated explicitly
#define SERVO_MAX_US 2400

Servo bigServo;
static int lastAngle = AT_REST;

static void printMenu(void)
{
    Serial.println(F("\n=== big servo bench test (pin 28) ==="));
    Serial.println(F("  c  full collection cycle      l  loop cycles (any key stops)"));
    Serial.println(F("  r  rest (70)                  p  pickup (110)"));
    Serial.println(F("  d  eased drop ramp 116->-5    s  slow sweep 0-116-0"));
    Serial.println(F("  x  detach (limp)              a  re-attach"));
    Serial.println(F("  0..180    angle in degrees"));
    Serial.println(F("  500..2500 raw pulse width in us"));
    Serial.print(F("  attached: "));
    Serial.println(bigServo.attached() ? F("yes") : F("NO"));
}

static void goAngle(int angle)
{
    if (!bigServo.attached()) bigServo.attach(BIG_SERVO, SERVO_MIN_US, SERVO_MAX_US);
    bigServo.write(angle);
    lastAngle = constrain(angle, 0, 180);
    Serial.print(F("[servo] write("));
    Serial.print(angle);
    Serial.print(F(")  -> "));
    Serial.print(bigServo.readMicroseconds());
    Serial.println(F(" us"));
}

// returns true if a key was pressed during the wait (used to abort loops)
static bool waitMs(unsigned long ms)
{
    unsigned long start = millis();
    while (millis() - start < ms)
    {
        digitalWrite(LED_PIN, (millis() / 100) & 1);
        if (Serial.available()) return true;
    }
    return false;
}

// same cosine ease-in-out as COLLECTION_MOVE_TO_DROP
static void easedDropRamp(void)
{
    Serial.print(F("[servo] eased ramp "));
    Serial.print(MOVE_TO_DROP_START_ANGLE);
    Serial.print(F(" -> "));
    Serial.print(MOVE_TO_DROP_END_ANGLE);
    Serial.print(F(" over "));
    Serial.print(MOVE_TO_DROP_RAMP_MS);
    Serial.println(F(" ms"));

    if (!bigServo.attached()) bigServo.attach(BIG_SERVO, SERVO_MIN_US, SERVO_MAX_US);
    unsigned long rampStart = millis();
    unsigned long rampElapsed;
    int prevAngle = 999;
    while ((rampElapsed = millis() - rampStart) <= MOVE_TO_DROP_RAMP_MS)
    {
        float t = (float)rampElapsed / (float)MOVE_TO_DROP_RAMP_MS;
        float easedT = (1.0f - cos(t * PI)) / 2.0f;
        int angle = MOVE_TO_DROP_START_ANGLE +
                    (int)((MOVE_TO_DROP_END_ANGLE - MOVE_TO_DROP_START_ANGLE) * easedT);
        bigServo.write(angle);
        if (angle != prevAngle && (angle % 10 == 0 || angle < 5))
        {
            Serial.print(F("    t="));
            Serial.print(rampElapsed);
            Serial.print(F("ms angle="));
            Serial.println(angle);
        }
        prevAngle = angle;
    }
    lastAngle = constrain(MOVE_TO_DROP_END_ANGLE, 0, 180);
    Serial.print(F("[servo] ramp done, pulse = "));
    Serial.print(bigServo.readMicroseconds());
    Serial.println(F(" us"));
}

// returns true if aborted by a key press
static bool collectionCycle(void)
{
    unsigned long t0 = millis();

    Serial.println(F("\n[cycle] PICKUP"));
    goAngle(PICKUP_ANGLE);
    if (waitMs(SERVODELAY1)) return true;

    Serial.println(F("[cycle] MOVE_TO_DROP"));
    unsigned long moveStart = millis();
    easedDropRamp();
    unsigned long spent = millis() - moveStart;
    if (spent < SERVODELAY2 && waitMs(SERVODELAY2 - spent)) return true;

    Serial.println(F("[cycle] DROP (1000 ms pause + drop dwell, magnets stay off)"));
    if (waitMs(1000 + WEIGHTDROPDELAY)) return true;

    Serial.println(F("[cycle] FINISHED -> rest"));
    goAngle(AT_REST);
    if (waitMs(SERVODELAY3)) return true;

    Serial.print(F("[cycle] done in "));
    Serial.print(millis() - t0);
    Serial.println(F(" ms"));
    return false;
}

static void slowSweep(void)
{
    Serial.println(F("[servo] slow sweep 0 -> 116 -> 0 (any key aborts)"));
    // ease down from wherever we are so the sweep doesn't start with a jump
    for (int a = lastAngle; a >= 0; a--)
    {
        bigServo.write(a);
        if (waitMs(15)) { lastAngle = a; return; }
    }
    for (int a = 0; a <= MOVE_TO_DROP_START_ANGLE; a++)
    {
        bigServo.write(a);
        if (a % 10 == 0) { Serial.print(F("    ")); Serial.println(a); }
        if (waitMs(20)) { lastAngle = a; return; }
    }
    for (int a = MOVE_TO_DROP_START_ANGLE; a >= 0; a--)
    {
        bigServo.write(a);
        if (a % 10 == 0) { Serial.print(F("    ")); Serial.println(a); }
        if (waitMs(20)) { lastAngle = a; return; }
    }
    lastAngle = 0;
    Serial.println(F("[servo] sweep done"));
}

static void flushInput(void)
{
    delay(5);
    while (Serial.available()) Serial.read();
}

void setup()
{
    pinMode(LED_PIN, OUTPUT);
    pinMode(MAG1, OUTPUT);
    pinMode(MAG2, OUTPUT);
    digitalWrite(MAG1, LOW);
    digitalWrite(MAG2, LOW);

    Serial.begin(115200);
    unsigned long start = millis();
    while (!Serial && millis() - start < 3000) {}

    bigServo.attach(BIG_SERVO, SERVO_MIN_US, SERVO_MAX_US);
    Serial.println(F("\n[servo_test] booted, moving to rest"));
    goAngle(AT_REST);
    printMenu();
}

void loop()
{
    digitalWrite(LED_PIN, (millis() / 500) & 1);   // slow blink = alive, waiting

    if (!Serial.available()) return;

    char c = Serial.peek();
    if (isDigit(c))
    {
        long v = Serial.parseInt();
        flushInput();
        if (v >= 0 && v <= 180) goAngle((int)v);
        else if (v >= 500 && v <= 2500)
        {
            if (!bigServo.attached()) bigServo.attach(BIG_SERVO, SERVO_MIN_US, SERVO_MAX_US);
            bigServo.writeMicroseconds((int)v);
            Serial.print(F("[servo] writeMicroseconds("));
            Serial.print(v);
            Serial.println(F(")"));
        }
        else Serial.println(F("out of range: 0-180 deg or 500-2500 us"));
        return;
    }

    Serial.read();
    flushInput();
    switch (c)
    {
        case '?': case 'h': printMenu(); break;
        case 'c': if (collectionCycle()) { flushInput(); Serial.println(F("[cycle] aborted")); } break;
        case 'l':
        {
            int n = 0;
            Serial.println(F("[loop] running cycles - press any key to stop"));
            while (!collectionCycle())
            {
                Serial.print(F("[loop] cycles completed: "));
                Serial.println(++n);
            }
            flushInput();
            Serial.print(F("[loop] stopped after "));
            Serial.print(n);
            Serial.println(F(" full cycles"));
            break;
        }
        case 'r': goAngle(AT_REST); break;
        case 'p': goAngle(PICKUP_ANGLE); break;
        case 'd': easedDropRamp(); break;
        case 's': slowSweep(); flushInput(); break;
        case 'x': bigServo.detach(); Serial.println(F("[servo] detached - no pulses, should be limp")); break;
        case 'a':
            bigServo.attach(BIG_SERVO, SERVO_MIN_US, SERVO_MAX_US);
            goAngle(lastAngle);
            break;
        case '\r': case '\n': case ' ': break;
        default: Serial.print(F("unknown command '")); Serial.print(c); Serial.println(F("' - '?' for menu")); break;
    }
}

#endif // SERVO_TEST
