# Plan: recover home colour after unloading alignment

Status: proposed only, 2 October 2026. Installed firmware remains V13.

## Evidence

In `logs/v12-stopped-live-20261002-124514.log`, the robot remained in
RECHECK_HOME with three weights, the third held, a responding closed gate,
healthy IMU/pose, and heading error 0.8 degrees. Its stored home was green but
the colour sensor consistently reported floor. The controller deliberately
commanded zero drive while waiting for colour; it had no recovery transition.
This establishes the blocker, but not whether the sensor was physically beyond
the coloured patch or misclassified it.

## Proposed behaviour

1. At each confirmed home observation during delivery approach, remember the
   local encoder/IMU position as a recovery waypoint. Preserve the original GO
   heading and round origin; do not zero the IMU or reset the round.
2. In RECHECK_HOME, permit up to one second of continuous fresh non-home colour
   before recovery. A brief dropout only resets the existing 600 ms confirmation.
   Stale/unknown colour should stop movement and wait for fresh classification,
   rather than being interpreted as physical departure from home.
3. After sustained fresh floor/enemy colour, transition to REACQUIRE_HOME and
   use the existing homing obstacle avoidance toward the last confirmed home
   location. If the estimated waypoint is reached without finding home colour,
   use a local search around that waypoint, not around a potentially drifted
   round-start position. Never authorize unloading on enemy colour.
4. Stop when own-home colour is found. Confirm it for 600 ms, then restore the
   recorded starting heading and reconfirm colour and heading at rest. Track
   the 15 cm advance as already attempted for this delivery, so recovery cannot
   blindly repeat it and progressively drive into the corner.
5. Open the gate only after fresh colour and heading confirmation. Preserve
   gate-first unloading, held-weight retention, pickup counts and GO stop.
   Recovery applies before gate opening; it must not start driving while the
   gate is open or the crane is releasing a weight.
6. If repeated alignment loses colour again, select another local search
   waypoint rather than replaying the same failed approach. Reuse homing
   repetition handling for travel. Do not add a fixed overall return timeout.

## Implementation scope

- `config.h`: colour-loss grace period and bounded local search settings.
- `home_core.h/.cpp`: recovery state, waypoint, colour-loss timer, delivery-entry
  attempt flag; preserve existing gate and crane interlocks.
- `homing.cpp`: pass colour freshness/classification explicitly; log recovery
  entry, cause, target, attempts and reacquisition. Keep ordinary collection
  navigation unchanged.
- Handle recovery before the current gate-readiness wait where appropriate, so
  a disconnected gate does not hide sustained colour loss. Gate readiness still
  gates opening/release.

## Verification plan

Adapt existing delivery fixtures for V13's measured advance first. Then cover
brief colour loss, sustained floor, enemy colour, stale readings, recovering
home, repeated alignment loss, no repeated 15 cm advance, GO stop with magnet
held, gate unavailability and unloading followed by another pickup. Assert that
neither gate opening nor third-weight release occurs without current home and
heading confirmation. Build the nav environment and run the updated checks
when implementation/testing is requested.

Arena test: record passively. Observe a normal delivery, then a return near the
base edge where alignment moves the sensor onto floor. Expected: brief stop,
controlled home reacquisition, realignment, confirmed gate-first unloading and
resumed searching. Verify the sensor's physical location when it reports floor;
if it is clearly over green, inspect raw colour classification separately.
