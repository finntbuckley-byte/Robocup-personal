# V11: shorter homing turns, repetition recovery, earlier creep

Uploaded 2 October 2026; passive COM7 readback confirmed
`HOMING_V11_REPEAT_RECOVERY` on Teensy 7697690.
Evidence: `logs/v11-upload-20261002-121628.log`.
Robot was waiting for GO. IMU and matrix were responding; colour read floor,
and gateReady was 0. Enable main power/check gate and start on home colour.

## Changes

- Homing obstacle-turn minimum is 400 ms. Collection remains 700 ms. These are
  minimums: a blocked view can require a longer turn. Escape timing is unchanged.
- Two completed avoidance episodes within 12 seconds, staying within 150 mm of
  the first episode's estimated position, flag repetition. This anchor persists
  across turns, unlike the ordinary per-pass distance measurement.
- At the next obstacle turn, repetition allows a switch opposite the committed
  direction only if that matrix half is open and the side IR is not near. If
  blocked, continue ordinary avoidance and reconsider on a later turn.
- Following a switch, another switch is inhibited until at least 150 mm of
  estimated displacement. This prevents immediate left/right alternation.
  Log markers are `HOME_REPEAT detected` and `HOME_REPEAT recovery`.
- Creep threshold increases from 200 to 250 mm. Close targets still visible to
  crossed lower beams retain approach steering, but use the 30-percent creep
  base command. The centre target enters CREEP at the earlier threshold.

The recorded GO heading, home-colour confirmation, gate-first unloading and
pickup retries remain unchanged. No reverse phase or fixed return timeout was
added. Recovery thresholds depend on encoder position and need arena validation;
one continuous turn is not counted as several completed avoidance episodes.

## Verification and next test

Build passed; all 26 integration scenarios passed, plus driver/core/helper
checks and 100,000 randomized core ticks. Tests cover the separate turn minima,
repetition across turns, blocked alternative direction, recovery commitment,
timer wrap, earlier creep, retained approach steering, unloading and another
pickup. One formatting warning in the new test harness was fixed before the
successful full run. No physical navigation has been tested with V11 yet.

Next run: record passively from before GO. Observe whether close approaches
knock over fewer weights; whether homing clears obstacles with fewer repeated
turns; and whether delivery still aligns to the starting heading, confirms home,
opens the gate before the third release, and resumes searching. No uploads or
serial commands during the recording.

No new commits or pushes. Confirmed V10 remains saved remotely as 8e15896
(`fairly good homing integration`) on round-2-win.
