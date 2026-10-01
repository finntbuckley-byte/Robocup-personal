# V10: deliver facing the round's starting orientation

Uploaded 2 October 2026. Passive COM7 readback confirmed
`HOMING_V10_START_HEADING` on Teensy 7697690, waiting for GO.
Evidence: `logs/v10-upload-20261002-115943.log`. At verification the colour sensor
read floor and gateReady=0; place on the base and enable main power before GO.

## Behaviour

At GO, the existing `imuZero()` captures the IMU's current orientation as this
round's zero heading. It is a fresh reference every round, not a fixed compass
direction. The IMU driver and GO capture code are unchanged.

After home colour is confirmed within the existing 600 mm estimated region,
delivery now enters ALIGN_START. It turns toward the recorded GO orientation
using the shortest angular error, instead of rotating 180 degrees from arrival.
If already within 10 degrees, it stops without an unnecessary turn.

Before gate opening, it must remain within 10 degrees and on confirmed home
colour for 600 ms at rest. If the heading overshoots during settling, it realigns
and starts confirmation again. If colour is lost, it waits stopped and requires
a fresh confirmation dwell after colour returns. There is no blind gate release.
Colour, position and heading are also checked during gate opening and unloading;
loss stops the delivery. Gate reconnect cannot bypass these conditions.

Gate-first unloading remains: two stored weights get their initial dwell, then
the held third releases through the open gate, followed by final dwell/closure.
Collection navigation, pickup retries, crane parameters, turning effort, homing
detours and round timing are unchanged. The existing commanded-turn time guard
also limits unsuccessful alignment corrections; no return-travel timeout added.

Telemetry identifies `HOMING_V10_START_HEADING`, state `ALIGN_START`, and
`startHeadingError` in degrees. `turnProgress` is diagnostic angular travel only;
it no longer authorizes release at 180 degrees.

## Software checks

The main firmware builds. The suite covers the recorded 112-degree and 50-degree
arrivals, both turn directions, already aligned arrivals, angular wrap, overshoot
while settling, colour loss/recovery and gate-first unloading. Integration cases
start at raw IMU headings 30, 210 and 355 degrees, restore the separately captured
reference, unload, exit and perform a further inductive-triggered pickup.
Collection replay and pickup retry scenarios remain included.

## Next arena check after upload

1. Start on home colour with the robot in the intended round-start orientation.
2. Collect and return from an oblique angle. Expected: ALIGN_START corrects the
   front to roughly the original GO direction, regardless of arrival direction.
3. Watch the stopped confirmation period. Expected: gate opens only with home
   colour still confirmed and heading within tolerance; two stored weights drain
   before the third releases.
4. Repeat with another starting orientation and return angle. Expected: it uses
   that round's new reference, not the previous round's orientation.

Physical settling, base-edge clearance and the actual landing positions still
need checking. Matching heading and one downward colour sensor cannot by
themselves establish the entire gate/drop path lies inside the base.

The user reported a good physical test and approved saving this version with
the commit message `fairly good homing integration`. Referenced serial logs
remain local to the Homing experiment workspace.
