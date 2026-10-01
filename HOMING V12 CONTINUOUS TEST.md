# V12: continuous arena testing and incidental home delivery

Prepared 2 October 2026. Firmware identity: `HOMING_V12_CONTINUOUS`.
Build and software checks passed. Uploaded 2 October 2026; passive COM7 readback
confirmed V12 on Teensy 7697690, waiting for GO with drive stopped and timed=0.
Evidence: `logs/v12-upload-confirmation.log`. IMU became ready during readback;
gateReady remained 0 and colour read floor. Enable/check main power and start on
home colour before the arena test. No new commits or pushes.

## Behaviour

- Run until a second GO press. The stop remains latched until reset; further GO
  presses do not resume it. Drive stops and crane motion freezes, preserving the
  magnet's existing PWM so a held weight remains held while powered.
- No two-minute automatic stop or remaining-time refusal to pick up or unload.
  `ROUND_TIME_LIMIT_ENABLED=0` selects this requested arena test mode.
- Return after three confirmed pickups. Retain the one-shot 88.5-second return
  request, but only with a nonempty load. After that return, continue collecting.
- Encountering fresh own-home colour with one or more counted weights also
  requests delivery. Zero-load encounters do not open the gate or request homing.
- Own-home colour can confirm arrival even when encoder position has drifted.
  IMU/pose health checks remain. Align to the heading recorded at GO and require
  stable home colour and heading before unloading.
- Open the gate and drain stored weights before releasing the held third weight;
  close the gate, clear the delivered count, leave base and resume searching.
- Collection steering, pickup mechanism, V11 earlier creep, and homing repetition
  recovery are unchanged by V12.

## Software verification

- PlatformIO `nav` build passed.
- All 32 integration scenarios passed, including incidental delivery with one
  and two weights, delivery after three minutes, another pickup after unloading,
  empty-load suppression, and GO stop during pickup or third-weight hold.
- Round checks passed in both unlimited and timed configurations.
- Core/helper checks passed, including 100,000 randomized core updates and
  colour-confirmed arrival with a deliberately drifted position estimate.

These checks use simulated sensor/actuator inputs. They do not prove physical
colour recognition, heading accuracy, gate motion or weight retention.

## Short arena test after uploading

1. Start on home colour at the intended unloading heading. With an empty load,
   revisit home: no unload sequence should start.
2. Collect one or two weights and cross own home: stop, confirm colour, align to
   the recorded start heading, open the gate and unload. Enemy colour must not
   trigger delivery. Check that searching resumes after exiting home.
3. Continue past two minutes: navigation and delivery should remain available.
   Test a three-weight trip and observe gate opening before the crane releases
   the third weight.
4. With a weight held, press GO again: drive and crane stop, magnet stays powered,
   and further GO presses leave the robot stopped. Support the weight before
   resetting or switching power off.

During a recorded round, read serial output passively only; do not upload or send
commands. The onboard weight count remains inferred from verified pickup cycles
and timed unloading, rather than a storage inventory sensor.
