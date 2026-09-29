# No-reverse trial — 30 September 2026

The user-confirmed working source and uploaded firmware were first copied to the sibling folder **WORK NAV FIRST ROUND USE**. All 104 source files were checked against their saved copies by hash. The saved firmware hash is recorded in that folder's `SAVED VERSION.md`.

## Changes in this trial

- `NAV_RECOVERY_REVERSE_ENABLED = false`: weight rejection and obstacle escape skip their backing-up phases. Escape timing also omits the disabled reverse interval. Weight rejection initialises its heading and timer immediately for the pivot.
- `REJECT_FROM_TOP_NOTCH_MISMATCH = false`: an absent or discrepant top sensor no longer labels an approaching upright weight as lying down and rejects it.
- The existing notch-assisted creep and its hard time cap remain active. No-metal timeouts still turn away; inductive confirmation still gates pickup.
- Pivot turns still reverse one track. There is no commanded two-track backing-up phase in the current nav configuration.
- Collection, motor calibration, sensor mapping, capacity and round timing are unchanged.

## Verification and next observation

The nav build and upload to COM3 succeeded. No physical run of this adjustment has been performed by the assistant. With a single upright steel weight, check that the robot approaches, creeps and collects without backing away. If it still turns away, the remaining causes include obstacle avoidance or the no-metal creep timeout; this adjustment does not disable either. Removing reverse may make escape from a tight corner less effective.

To restore the confirmed working version, build/upload the `nav` environment from the sibling **WORK NAV FIRST ROUND USE** folder, or flash its saved `WORK NAV FIRST ROUND USE.hex`.
