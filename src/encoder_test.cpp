/* ============================================================================
 *  encoder_test.cpp  -  drive-encoder bring-up + calibration (enctest env)
 *
 *      pio run -e enctest -t upload
 *      pio device monitor -b 115200        ('?' for the menu)
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
 *  Motors go through drive() so the trims, soft start and DRIVE_SCALE_PCT
 *  all apply, same as the nav build. Results live in RAM: they survive
 *  unplugging USB (the robot's power module keeps the Teensy up) but not
 *  switching the robot off.
 * ============================================================================ */
#ifdef ENC_TEST

#include <Arduino.h>
#include <Encoder.h>
#include "config.h"
#include "motor.h"
#include "drive.h"

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

struct RunResult { unsigned long ms; int pct; int dir; long l, r; };
static const int MAX_RESULTS = 20;
static RunResult results[MAX_RESULTS];
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
  Serial.println(" s      stop (any other key also stops)");
  Serial.println(" + / -  test speed +/-10 %");
  Serial.println(" p      print counts once");
  Serial.print  (" speed "); Serial.print(testPct);
  Serial.print  ("%  (x DRIVE_SCALE_PCT "); Serial.print(DRIVE_SCALE_PCT); Serial.println("%)");
  Serial.print  (" timed run "); Serial.print(timedMs); Serial.println(" ms");
  Serial.print  (" stored runs "); Serial.println(nResults);
  Serial.print  (" ENC_COUNTS_PER_M = "); Serial.println(ENC_COUNTS_PER_M);
  Serial.println("===========================");
  Serial.println("ms\tcmdL\tcmdR\tcntL\tcntR\tmmL\tmmR\tmmAvg\tcpsL\tcpsR");
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
  Serial.print('\t');     Serial.println(x.r != 0 ? (float)x.l / x.r : 0.0f, 3);
}

static void listResults()
{
  Serial.println("RUN\t#\tdir\trunMs\tspeedPct\tcntL\tcntR\tavg\tL/R");
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
  RunResult x = { timedMs, testPct, timedDir, cntL(), cntR() };
  if (nResults < MAX_RESULTS) results[nResults++] = x;
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
  if (PIN_GO >= 0) pinMode(PIN_GO, GO_ACTIVE_LOW ? INPUT_PULLUP : INPUT);
  printHelp();
}

void loop()
{
  if (Serial.available())
  {
    char c = Serial.read();
    while (Serial.available()) Serial.read();
    switch (c)
    {
      case '?': printHelp(); break;
      case 'z': encL.write(0); encR.write(0); Serial.println("zeroed"); break;
      case 'f': startRun(+1, +1, ENCTEST_MAX_RUN_MS); break;
      case 'b': startRun(-1, -1, ENCTEST_MAX_RUN_MS); break;
      case 'l': startRun(+1,  0, ENCTEST_MAX_RUN_MS); break;
      case 'r': startRun( 0, +1, ENCTEST_MAX_RUN_MS); break;
      case 'g': startTimed(+1); break;
      case 'h': startTimed(-1); break;
      case '[': timedMs = max(timedMs - 250, 250UL); Serial.print("timed run "); Serial.println(timedMs); break;
      case ']': timedMs = min(timedMs + 250, ENCTEST_MAX_RUN_MS); Serial.print("timed run "); Serial.println(timedMs); break;
      case '+': testPct = min(testPct + 10, 100); Serial.print("speed "); Serial.println(testPct); break;
      case '-': testPct = max(testPct - 10, 10);  Serial.print("speed "); Serial.println(testPct); break;
      case 'p': printRow(); break;
      case 'L': listResults(); break;
      case 'C': nResults = 0; Serial.println("stored runs cleared"); break;
      case '\n': case '\r': break;
      default: if (runL || runR || goCountdownAt) cancelTimed("key"); break;
    }
  }

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

  bool running = runL || runR;
  if (running)
  {
    drive(runL * testPct, runR * testPct);   // called every loop so the soft start ramps
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
