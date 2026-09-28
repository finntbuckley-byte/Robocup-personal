/* ============================================================================
 *  encoder_test.cpp  -  drive-encoder + heading-hold PID calibration (enctest env)
 *
 *      pio run -e enctest -t upload
 *      pio device monitor -b 115200        ('?' for the menu, type a line + Enter)
 *
 *  1. Robot on blocks: 'l' drives ONLY the left motor forward, 'r' ONLY the
 *     right. Only that motor's count should move, and it should go UP.
 *     Other side moves -> swap the PIN_ENC_L_* / PIN_ENC_R_* pairs.
 *     Goes down -> flip that ENC_*_SIGN.
 *  2. On the floor, UNTETHERED: press GO -> GO_RUN_DELAY_MS to step back ->
 *     timed forward run of timedMs (soft start included) -> result stored.
 *     Press GO again for the next run. GO during the countdown or a run
 *     cancels it. Plug back in and 'L' lists every stored run.
 *     Tethered: 'g' / 'h' do the same timed run forward / reverse.
 *     Stored avg count / measured metres = ENC_COUNTS_PER_M.
 *
 *  3. Heading hold: 'H' toggles it (default ON, needs the IMU). Stored runs
 *     record hold on/off and the IMU heading at the end (+ = turned right),
 *     so hold-off vs hold-on runs compare directly. 'T' prints a per-run
 *     trace (heading / steer / integral every 100 ms) for judging weave vs
 *     steady-state offset vs slow-to-settle.
 *
 *  4. PID tuning, live (no reflash): "kp <v>" "ki <v>" "kd <v>" "imax <v>"
 *     "maxsteer <v>" set gains for the NEXT run (headingTuning() in imu.h/.cpp
 *     - the same struct nav's build would use if it ever called the setter,
 *     which it doesn't, so this never affects the real robot). 's' prints the
 *     current gains AND the steady 'integral' from the last run, paste-ready
 *     for config.h (HEADING_KP/KI/KD/I_MAX/MAX_STEER/I_START). Recommended
 *     order: tune kp alone (ki=kd=0) for the largest gain that doesn't weave,
 *     add kd to damp any overshoot, add ki last to kill the remaining
 *     steady-state drift, then read the settled integral off 'T' into
 *     HEADING_I_START so rounds start already compensated.
 *
 *  Motors go through drive() so the trims, soft start and DRIVE_SCALE_PCT
 *  all apply, same as the nav build. Results live in RAM: they survive
 *  unplugging USB (the robot's power module keeps the Teensy up) but not
 *  switching the robot off.
 * ============================================================================ */
#ifdef ENC_TEST

#include <Arduino.h>
#include <Encoder.h>
#include <Wire.h>
#include "config.h"
#include "motor.h"
#include "drive.h"
#include "imu.h"

static Encoder encL(PIN_ENC_L_A, PIN_ENC_L_B);
static Encoder encR(PIN_ENC_R_A, PIN_ENC_R_B);

static const unsigned long ENCTEST_MAX_RUN_MS = 6000;   // safety auto-stop
static const unsigned long PRINT_MS          = 100;
static const unsigned long COAST_SETTLE_MS   = 800;    // wait for the tracks to stop before storing
static const unsigned long GO_RUN_DELAY_MS   = 2000;   // press GO, step back

static int  testPct = 100;       // command while driving, before DRIVE_SCALE_PCT (100 = nav cruise)
static int  runL = 0, runR = 0;  // per motor: +1 forward, -1 reverse, 0 off
static unsigned long runStart = 0;
static unsigned long runLimitMs = ENCTEST_MAX_RUN_MS;

// timed calibration runs
static unsigned long timedMs = 3000;
static int  timedDir = 0;              // direction of the run in progress
static bool resultPending = false;     // timed run finished, waiting to coast to a stop
static unsigned long stoppedAt = 0;
static unsigned long goCountdownAt = 0;  // 0 = no countdown

// heading hold ('H' toggles). Target = the heading at the start of each run.
static bool holdOn = true;

// per-run trace of the hold: every TRACE_MS during a timed run ('T' prints)
static const int TRACE_N = 50;
static const unsigned long TRACE_MS = 100;
struct Trace { uint8_t n; float hdg[TRACE_N]; int8_t steer[TRACE_N]; float integ[TRACE_N]; };
static Trace cur;
static int traceN = 0;
static unsigned long traceAt = 0;

struct RunResult { unsigned long ms; int pct; int dir; long l, r; bool hold; float hdg; };
static const int MAX_RESULTS = 20;
static RunResult results[MAX_RESULTS];
static Trace traces[MAX_RESULTS];
static int nResults = 0;

static long cntL() { return ENC_L_SIGN * encL.read(); }
static long cntR() { return ENC_R_SIGN * encR.read(); }

// ---- GO button: same safety rule as round.cpp - it only counts after it
// has been seen RELEASED, and each press fires once ----------------------
static bool goArmed = false;
static unsigned long goUpSince = 0, goDownSince = 0;

static bool goActiveRaw()
{
  return GO_ACTIVE_LOW ? (digitalRead(PIN_GO) == LOW) : (digitalRead(PIN_GO) == HIGH);
}

static bool goPressedOnce()
{
  if (PIN_GO < 0) return false;
  bool active = goActiveRaw();
  if (!goArmed)
  {
    if (active) { goUpSince = 0; return false; }
    if (goUpSince == 0) goUpSince = millis();
    if (millis() - goUpSince >= GO_DEBOUNCE_MS) goArmed = true;
    return false;
  }
  if (!active) { goDownSince = 0; return false; }
  if (goDownSince == 0) goDownSince = millis();
  if (millis() - goDownSince < GO_DEBOUNCE_MS) return false;
  goArmed = false;          // must be released before the next press counts
  goUpSince = 0; goDownSince = 0;
  return true;
}

// ---- output ------------------------------------------------------------
static void printHelp()
{
  Serial.println("\n========= enctest =========");
  Serial.println(" GO     (untethered) wait 2 s, timed forward run, store result");
  Serial.println(" L      list stored runs        C  clear them");
  Serial.println(" g / h  timed run forward / reverse from here");
  Serial.println(" [ / ]  timed run length -/+ 250 ms");
  Serial.println(" z      zero both counts");
  Serial.println(" f / b  drive both forward / reverse (6 s max)");
  Serial.println(" l / r  drive ONLY the left / right motor forward (on blocks)");
  Serial.println(" any unrecognised key stops a run/countdown in progress");
  Serial.println(" + / -  test speed +/-10 %");
  Serial.println(" p      print counts once");
  Serial.println(" H      heading hold on/off (IMU) - stored with each run");
  Serial.println(" R      reset the hold's learned integral     T  print run traces");
  Serial.println(" kp <v>  ki <v>  kd <v>  imax <v>  maxsteer <v>   set live (next run)");
  Serial.println(" s      show current PID gains, paste-ready for config.h");
  Serial.print  (" speed "); Serial.print(testPct);
  Serial.print  ("%  (x DRIVE_SCALE_PCT "); Serial.print(DRIVE_SCALE_PCT); Serial.println("%)");
  Serial.print  (" timed run "); Serial.print(timedMs); Serial.println(" ms");
  Serial.print  (" stored runs "); Serial.println(nResults);
  Serial.print  (" IMU "); Serial.print(imuOk() ? "ok" : "NOT FOUND");
  Serial.print  ("   heading hold "); Serial.println(holdOn && imuOk() ? "ON" : "off");
  Serial.print  (" ENC_COUNTS_PER_M = "); Serial.println(ENC_COUNTS_PER_M);
  Serial.println("===========================");
  Serial.println("ms\tcmdL\tcmdR\tcntL\tcntR\tmmL\tmmR\tmmAvg\tcpsL\tcpsR");
}

static void printSettings()
{
  HeadingTuning &t = headingTuning();
  Serial.println("\n--- paste into include/config.h ---");
  Serial.print("const float HEADING_KP        = "); Serial.print(t.kp, 2);  Serial.println("f;");
  Serial.print("const float HEADING_KI        = "); Serial.print(t.ki, 2);  Serial.println("f;");
  Serial.print("const float HEADING_KD        = "); Serial.print(t.kd, 2);  Serial.println("f;");
  Serial.print("const float HEADING_I_MAX     = "); Serial.print(t.iMax, 1); Serial.println("f;");
  Serial.print("const int   HEADING_MAX_STEER = "); Serial.println(t.maxSteer);
  Serial.print("const float HEADING_I_START   = "); Serial.print(t.iStart, 1); Serial.println("f;");
  Serial.print("(current learned integral = "); Serial.print(headingHoldIntegral(), 1);
  Serial.println(" - once a run's 'T' trace shows this settled/steady, that's a HEADING_I_START candidate)");
}

static void printRow()
{
  long l = cntL(), r = cntR();
  Serial.print(millis());            Serial.print('\t');
  Serial.print(lastDriveLeftPct());  Serial.print('\t');
  Serial.print(lastDriveRightPct()); Serial.print('\t');
  Serial.print(l);                   Serial.print('\t');
  Serial.print(r);                   Serial.print('\t');
  Serial.print(l * 1000.0f / ENC_COUNTS_PER_M, 0); Serial.print('\t');
  Serial.print(r * 1000.0f / ENC_COUNTS_PER_M, 0); Serial.print('\t');
  Serial.print((l + r) * 500.0f / ENC_COUNTS_PER_M, 0); Serial.print('\t');

  // counts per second since the last row - compares motor speeds on blocks
  static long ql = 0, qr = 0;
  static unsigned long qt = 0;
  unsigned long now = millis();
  if (qt == 0 || now == qt) Serial.println("-\t-");
  else
  {
    float dt = (now - qt) / 1000.0f;
    Serial.print((l - ql) / dt, 0); Serial.print('\t');
    Serial.println((r - qr) / dt, 0);
  }
  ql = l; qr = r; qt = now;
}

static void printResultRow(int i)
{
  const RunResult &x = results[i];
  Serial.print("RUN\t");  Serial.print(i + 1);
  Serial.print('\t');     Serial.print(x.dir > 0 ? "fwd" : "rev");
  Serial.print('\t');     Serial.print(x.ms);
  Serial.print('\t');     Serial.print(x.pct);
  Serial.print('\t');     Serial.print(x.l);
  Serial.print('\t');     Serial.print(x.r);
  Serial.print('\t');     Serial.print((x.l + x.r) / 2);
  Serial.print('\t');     Serial.print(x.r != 0 ? (float)x.l / x.r : 0.0f, 3);
  Serial.print('\t');     Serial.print(x.hold ? "on" : "off");
  Serial.print('\t');     Serial.println(x.hdg, 1);
}

static void printTraces()
{
  Serial.println("TRACE\trun\tt_ms\thdg\tsteer\tintegral");
  for (int i = 0; i < nResults; i++)
    for (int k = 0; k < traces[i].n; k++)
    {
      Serial.print("TRACE\t"); Serial.print(i + 1);
      Serial.print('\t'); Serial.print(k * TRACE_MS);
      Serial.print('\t'); Serial.print(traces[i].hdg[k], 1);
      Serial.print('\t'); Serial.print(traces[i].steer[k]);
      Serial.print('\t'); Serial.println(traces[i].integ[k], 1);
    }
}

static void listResults()
{
  Serial.println("RUN\t#\tdir\trunMs\tspeedPct\tcntL\tcntR\tavg\tL/R\thold\thdgEnd");
  for (int i = 0; i < nResults; i++) printResultRow(i);
  if (nResults == 0) Serial.println("(no runs stored)");
}

// ---- runs --------------------------------------------------------------
static void stopRun(const char *why)
{
  runL = runR = 0;
  stopMotors();
  Serial.print("STOP ("); Serial.print(why); Serial.println(")");
  printRow();
  stoppedAt = millis();
}

static void startRun(int l, int r, unsigned long limitMs)
{
  runL = l; runR = r;
  runLimitMs = limitMs;
  runStart = millis();
}

static void startTimed(int dir)
{
  encL.write(0); encR.write(0);
  imuZero();                      // hold target = straight ahead from here
  // NOT resetting the hold's integral: the drag bias it learned carries over
  // to the next run, like it does between FORWARD stretches in nav. 'R' resets.
  traceN = 0; traceAt = 0;
  timedDir = dir;
  resultPending = true;
  Serial.print("TIMED "); Serial.print(dir > 0 ? "forward " : "reverse ");
  Serial.print(timedMs); Serial.println(" ms");
  startRun(dir, dir, timedMs);
}

static void cancelTimed(const char *why)
{
  resultPending = false;
  goCountdownAt = 0;
  if (runL || runR) stopRun(why);
  else { Serial.print("cancelled ("); Serial.print(why); Serial.println(")"); }
}

// store once the tracks have coasted to a stop
static void storeResult()
{
  RunResult x = { timedMs, testPct, timedDir, cntL(), cntR(), holdOn && imuOk(), imuHeadingDeg() };
  cur.n = traceN;
  if (nResults < MAX_RESULTS) { traces[nResults] = cur; results[nResults++] = x; }
  else Serial.println("result store full - 'C' to clear");
  listResults();
}

void setup()
{
  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 2000) {}
  motor_init();
  stopMotors();
  Wire.begin(); Wire.setClock(400000);
  if (!imuInit()) Serial.println("!! IMU not found - heading hold unavailable");
  if (PIN_GO >= 0) pinMode(PIN_GO, GO_ACTIVE_LOW ? INPUT_PULLUP : INPUT);
  printHelp();
}

// Multi-char commands ("kp 2.5") for live PID tuning, plus every legacy
// single-char command unchanged. One line in, so "kp 2.5\n" isn't split
// across two reads the way single-char parsing would mangle it.
static void handleLine(String line)
{
  line.trim();
  if (line.length() == 0) return;

  int sp = line.indexOf(' ');
  String key = sp < 0 ? line : line.substring(0, sp);
  String arg = sp < 0 ? "" : line.substring(sp + 1);
  String keyLower = key; keyLower.toLowerCase();

  if (keyLower == "kp" && arg.length())
  { headingTuning().kp = arg.toFloat(); Serial.print("kp = "); Serial.println(headingTuning().kp, 2); return; }
  if (keyLower == "ki" && arg.length())
  { headingTuning().ki = arg.toFloat(); Serial.print("ki = "); Serial.println(headingTuning().ki, 2); return; }
  if (keyLower == "kd" && arg.length())
  { headingTuning().kd = arg.toFloat(); Serial.print("kd = "); Serial.println(headingTuning().kd, 2); return; }
  if (keyLower == "imax" && arg.length())
  { headingTuning().iMax = arg.toFloat(); Serial.print("imax = "); Serial.println(headingTuning().iMax, 1); return; }
  if (keyLower == "maxsteer" && arg.length())
  { headingTuning().maxSteer = arg.toInt(); Serial.print("maxsteer = "); Serial.println(headingTuning().maxSteer); return; }
  if (keyLower == "s") { printSettings(); return; }

  if (line.length() == 1)
  {
    switch (line[0])
    {
      case '?': printHelp(); return;
      case 'z': encL.write(0); encR.write(0); Serial.println("zeroed"); return;
      case 'f': startRun(+1, +1, ENCTEST_MAX_RUN_MS); return;
      case 'b': startRun(-1, -1, ENCTEST_MAX_RUN_MS); return;
      case 'l': startRun(+1,  0, ENCTEST_MAX_RUN_MS); return;
      case 'r': startRun( 0, +1, ENCTEST_MAX_RUN_MS); return;
      case 'g': startTimed(+1); return;
      case 'h': startTimed(-1); return;
      case '[': timedMs = max(timedMs - 250, 250UL); Serial.print("timed run "); Serial.println(timedMs); return;
      case ']': timedMs = min(timedMs + 250, ENCTEST_MAX_RUN_MS); Serial.print("timed run "); Serial.println(timedMs); return;
      case '+': testPct = min(testPct + 10, 100); Serial.print("speed "); Serial.println(testPct); return;
      case '-': testPct = max(testPct - 10, 10);  Serial.print("speed "); Serial.println(testPct); return;
      case 'p': printRow(); return;
      case 'L': listResults(); return;
      case 'C': nResults = 0; Serial.println("stored runs cleared"); return;
      case 'H': holdOn = !holdOn; Serial.print("heading hold "); Serial.println(holdOn ? "ON" : "off"); return;
      case 'R': headingHoldReset(); Serial.println("hold integral reset"); return;
      case 'T': printTraces(); return;
      default: break;
    }
  }

  // anything unrecognised stops a run/countdown in progress
  if (runL || runR || goCountdownAt) cancelTimed("key");
}

void loop()
{
  if (Serial.available())
    handleLine(Serial.readStringUntil('\n'));

  // GO: start a countdown, or cancel whatever is in progress
  if (goPressedOnce())
  {
    if (runL || runR || goCountdownAt || resultPending) cancelTimed("GO");
    else
    {
      goCountdownAt = millis();
      Serial.println("GO: forward run in 2 s");
    }
  }
  if (goCountdownAt && millis() - goCountdownAt >= GO_RUN_DELAY_MS)
  {
    goCountdownAt = 0;
    startTimed(+1);
  }

  imuUpdate();

  bool running = runL || runR;
  if (running)
  {
    // hold only on straight runs (both tracks the same way) - steer + = right
    int steer = (holdOn && runL == runR) ? headingHoldSteer(0) : 0;
    drive(runL * testPct + steer, runR * testPct - steer);   // every loop so the soft start ramps
    if (resultPending && traceN < TRACE_N && millis() - traceAt >= TRACE_MS)
    {
      traceAt = millis();
      cur.hdg[traceN] = imuHeadingDeg();
      cur.steer[traceN] = (int8_t)steer;
      cur.integ[traceN] = headingHoldIntegral();
      traceN++;
    }
    if (millis() - runStart >= runLimitMs) stopRun(resultPending ? "timed" : "timeout");
  }
  else
  {
    stopMotors();
    if (resultPending && millis() - stoppedAt >= COAST_SETTLE_MS)
    {
      resultPending = false;
      storeResult();
    }
  }

  // print while moving, or when a hand-turned track changes a count
  static unsigned long lastPrint = 0;
  static long pl = 0, pr = 0;
  if (millis() - lastPrint >= PRINT_MS && (running || cntL() != pl || cntR() != pr))
  {
    lastPrint = millis();
    pl = cntL(); pr = cntR();
    printRow();
  }
}

#endif // ENC_TEST
