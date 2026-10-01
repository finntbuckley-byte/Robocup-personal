# V13: advance 15 cm before unloading alignment

Prepared 2 October 2026. Identity: `HOMING_V13_HOME_ADVANCE`.
Build passed (`nav`). Uploaded 2 October 2026; passive COM7 readback confirmed
V13 on Teensy 7697690, waiting for GO with target=150, IMU ready and gateReady=1.
Gate position was not yet confirmed (gate=-1). Colour read floor; place on home
colour before GO. Evidence: `logs/v13-upload-confirmation.log`.
No tests run for this revision; the V12
test results do not verify the new advance phase. Existing delivery test
fixtures will need to simulate forward travel through `ADVANCE_HOME` before
expecting alignment. No commits or pushes.

After home colour stays confirmed for the existing 600 ms, record the arrival
position and heading. Drive forward at a 70-percent base command, with bounded
IMU steering correction, until encoder/IMU position shows 150 mm of forward
progress along that heading. Stop, then align to the heading captured at GO and
reconfirm home colour before opening the gate. Third-weight release remains
after the gate opens and the stored weights have had time to clear.

Fresh matrix obstacles closer than 270 mm (centre or either half), or side IR
obstacles closer than 120 mm, end the advance early and proceed to alignment.
A stale matrix pauses the advance. Existing pose/IMU fault handling and GO stop
still apply. Telemetry reports `HOME_ADVANCE progress=... target=150`.

Physical distance depends on encoder calibration and track slip. In the next
arena test, observe the advance, rotation, colour recheck and gate-first unload.
This implements option 1 only: if colour is still lost after alignment, the
existing stationary colour wait remains. Automatic colour reacquisition was
not part of this requested change.
