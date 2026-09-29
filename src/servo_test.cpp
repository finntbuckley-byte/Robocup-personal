/* ============================================================================
 *  servo_test.cpp  -  crane bench test, using the REAL collection.cpp FSM
 *
 *  Build + upload:  pio run -e servotest -t upload
 *  Serial monitor:  pio device monitor -b 115200     (type a command + Enter)
 *
 *  Only compiled when SERVO_TEST is defined (the servotest env sets it).
 *  It links collection.cpp + smooth_servo.cpp, so a cycle here is exactly
 *  the cycle the robot runs - there's no copy of the timings to keep in sync.
 *  The robot does NOT drive in this build.
 *
 *  Angles and speeds start from config.h (CRANE_*) and can be changed live.
 *  When a setting works, type  s  and paste the printed lines into config.h.
 *
 *  Commands (end each with Enter):
 *      c              one full collection cycle (magnets on at pickup,
 *                     off at drop - exactly as on the robot)
 *      l              loop full cycles until any key
 *      p / d / r      eased move to pickup / drop / rest at its own speed
 *      pa 120         set pickup angle      ps 45   set pickup speed (deg/s)
 *      da 40          set dr1
 * op angle        ds 60   set drop speed (deg/s)
 *      ra 70          set rest angle        rs 100  set rest speed (deg/s)
 *      + / -          nudge the arm 1 deg      ++ / --  nudge 5 deg
 *      j 105          eased move to exactly 105 deg
 *      save p|d|r     store the CURRENT arm angle as pickup / drop / rest
 *      m              magnets on/off (for manual positioning checks)
 *      s              show current settings, paste-ready for config.h
 *      ?              this menu
 * ============================================================================ */
#ifdef SERVO_TEST

#include <Arduino.h>
#include "collection.h"
#include "config.h"

#define LED_PIN 13

static bool magnetsOn  = false;
static bool goArmed    = false;   // true after GO pressed; inductive triggers a cycle

static void printMenu(void)
{
    Serial.println(F("\n=== crane bench test (real collection.cpp cycle) ==="));
    Serial.println(F("  c  one full cycle           l  loop cycles (any key stops)"));
    Serial.println(F("  p / d / r   move to pickup / drop / rest (eased)"));
    Serial.println(F("  pa <deg>  ps <deg/s>   pickup angle / speed"));
    Serial.println(F("  da <deg>  ds <deg/s>   drop angle / speed"));
    Serial.println(F("  ra <deg>  rs <deg/s>   rest angle / speed"));
    Serial.println(F("  + / -  nudge 1 deg   ++ / --  nudge 5 deg   j <deg>  go to angle"));
    Serial.println(F("  save p | save d | save r   use the CURRENT angle for pickup/drop/rest"));
    Serial.println(F("  m  magnets on/off           s  show settings (paste into config.h)"));
}

static void printSettings(void)
{
    CraneTuning &t = collection_tuning();
    Serial.println(F("\n--- paste into include/config.h ---"));
    Serial.printf("const int   CRANE_PICKUP_ANGLE = %d;\n", t.pickupAngle);
    Serial.printf("const float CRANE_PICKUP_DPS   = %.1ff;\n", t.pickupDps);
    Serial.printf("const int   CRANE_DROP_ANGLE   = %d;\n", t.dropAngle);
    Serial.printf("const float CRANE_DROP_DPS     = %.1ff;\n", t.dropDps);
    Serial.printf("const int   CRANE_REST_ANGLE   = %d;\n", t.restAngle);
    Serial.printf("const float CRANE_REST_DPS     = %.1ff;\n", t.restDps);
    Serial.printf("(arm is at %d deg%s)\n", collection_arm_angle(),
                  collection_arm_busy() ? ", moving" : "");
}

// run the FSM until the cycle finishes; returns true if aborted by a key
static bool runCycle(void)
{
    unsigned long t0 = millis();
    collection_start();
    while (collection_busy())
    {
        collection_update();
        digitalWrite(LED_PIN, (millis() / 100) & 1);
        if (Serial.available()) return true;     // loop() keeps ticking the FSM, so the cycle still finishes
    }
    Serial.printf("[cycle] done in %lu ms\n", millis() - t0);
    return false;
}

static void moveAndWait(int angle, float dps, const char *name)
{
    if (!collection_move_to(angle, dps)) { Serial.println(F("busy - cycle in progress")); return; }
    Serial.printf("[move] -> %s %d deg @ %.0f deg/s\n", name, angle, dps);
    unsigned long t0 = millis();
    while (collection_arm_busy()) collection_update();
    Serial.printf("[move] done in %lu ms\n", millis() - t0);
}

static const float JOG_DPS = 30.0f;   // gentle, so a nudge never jerks the arm

static void jogTo(int angle)
{
    angle = constrain(angle, 0, 180);
    if (!collection_move_to(angle, JOG_DPS)) { Serial.println(F("busy - cycle in progress")); return; }
    while (collection_arm_busy()) collection_update();
    Serial.print(F("[jog] arm at ")); Serial.print(collection_arm_angle()); Serial.println(F(" deg"));
}

static bool setValue(const String &key, float v)
{
    CraneTuning &t = collection_tuning();
    bool isAngle = key.endsWith("a");
    if (isAngle && (v < 0 || v > 180)) { Serial.println(F("angle must be 0-180")); return true; }
    if (!isAngle && (v < 5 || v > 400)) { Serial.println(F("speed must be 5-400 deg/s")); return true; }

    if      (key == "pa") t.pickupAngle = (int)v;
    else if (key == "ps") t.pickupDps   = v;
    else if (key == "da") t.dropAngle   = (int)v;
    else if (key == "ds") t.dropDps     = v;
    else if (key == "ra") t.restAngle   = (int)v;
    else if (key == "rs") t.restDps     = v;
    else return false;

    Serial.printf("[set] %s = %g\n", key.c_str(), v);
    return true;
}

// ---------------------------------------------------------------------------
// GO-armed inductive trigger
// ---------------------------------------------------------------------------
static void checkInductiveTrigger()
{
    if (!goArmed || collection_busy()) return;

    static bool     lastMetal    = false;
    static unsigned long metalAt = 0;

    bool metal = (digitalRead(PIN_INDUCTIVE) == LOW);   // active LOW
    if (metal && !lastMetal) metalAt = millis();
    lastMetal = metal;

    if (metal && (millis() - metalAt >= INDUCTIVE_CONFIRM_MS))
    {
        Serial.println(F("[inductive] metal - starting cycle"));
        runCycle();
    }
}

void setup()
{
    pinMode(LED_PIN,       OUTPUT);
    pinMode(PIN_INDUCTIVE, INPUT);
    pinMode(PIN_GO,        INPUT);   // active HIGH, no pull-up (matches nav)
    Serial.begin(115200);
    unsigned long start = millis();
    while (!Serial && millis() - start < 3000) {}

    collection_init();          // attaches the servo, parks at the rest angle
    collection_magnets(false);
    Serial.println(F("\n[servo_test] booted - crane parked at rest"));
    Serial.println(F("  Press GO to arm inductive trigger (inductive hit -> cycle runs)"));
    Serial.println(F("  Press GO again to disarm"));
    printMenu();
    printSettings();
}

void loop()
{
    collection_update();                        // keeps eased moves running

    // GO button: toggle armed mode (debounced)
    static bool lastGo = false;
    bool go = (digitalRead(PIN_GO) == HIGH);
    if (go && !lastGo)
    {
        goArmed = !goArmed;
        Serial.println(goArmed ? F("[GO] armed  - inductive trigger ON")
                               : F("[GO] disarmed - inductive trigger OFF"));
    }
    lastGo = go;

    checkInductiveTrigger();

    digitalWrite(LED_PIN, goArmed ? ((millis() / 100) & 1)   // fast blink = armed
                                  : ((millis() / 500) & 1)); // slow blink = idle

    if (!Serial.available()) return;
    String line = Serial.readStringUntil('\n');
    line.trim();
    line.toLowerCase();
    if (line.length() == 0) return;

    int sp = line.indexOf(' ');
    String key = sp < 0 ? line : line.substring(0, sp);
    String arg = sp < 0 ? "" : line.substring(sp + 1);
    CraneTuning &t = collection_tuning();

    if (key == "j" && arg.length() > 0) { jogTo(arg.toInt()); return; }
    if (key == "save")
    {
        int a = collection_arm_angle();
        if      (arg == "p") t.pickupAngle = a;
        else if (arg == "d") t.dropAngle   = a;
        else if (arg == "r") t.restAngle   = a;
        else { Serial.println(F("use: save p | save d | save r")); return; }
        Serial.print(F("[save] "));
        Serial.print(arg == "p" ? "pickup" : arg == "d" ? "drop" : "rest");
        Serial.print(F(" angle = ")); Serial.print(a);
        Serial.println(F(" deg  ('s' to print for config.h)"));
        return;
    }

    if (arg.length() > 0)
    {
        if (!setValue(key, arg.toFloat())) Serial.println(F("unknown setting - '?' for menu"));
        return;
    }

    if      (key == "+")  jogTo(collection_arm_angle() + 1);
    else if (key == "-")  jogTo(collection_arm_angle() - 1);
    else if (key == "++") jogTo(collection_arm_angle() + 5);
    else if (key == "--") jogTo(collection_arm_angle() - 5);
    else if (key == "?" || key == "h") printMenu();
    else if (key == "s") printSettings();
    else if (key == "c")
    {
        if (runCycle()) { while (Serial.available()) Serial.read(); Serial.println(F("[cycle] stopped watching - the crane finishes the cycle on its own")); }
    }
    else if (key == "l")
    {
        int n = 0;
        Serial.println(F("[loop] running cycles - press Enter to stop"));
        while (!runCycle()) Serial.printf("[loop] cycles completed: %d\n", ++n);
        while (Serial.available()) Serial.read();
        Serial.printf("[loop] stopped after %d full cycles\n", n);
    }
    else if (key == "p") moveAndWait(t.pickupAngle, t.pickupDps, "pickup");
    else if (key == "d") moveAndWait(t.dropAngle,   t.dropDps,   "drop");
    else if (key == "r") moveAndWait(t.restAngle,   t.restDps,   "rest");
    else if (key == "m")
    {
        magnetsOn = !magnetsOn;
        collection_magnets(magnetsOn);
        Serial.println(magnetsOn ? F("[mag] ON") : F("[mag] OFF"));
    }
    else Serial.println(F("unknown command - '?' for menu"));
}

#endif // SERVO_TEST
