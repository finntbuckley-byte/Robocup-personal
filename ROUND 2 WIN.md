# Round 2 win

User confirmed this firmware is working well on 30 September 2026.
Base commit: `4cdb7ba608da29a6794fde3b968bbe15494e2d31`.
Build and upload environment: `nav`.

- Crossed lower sensors mapped by viewing direction for weight detection and approach steering.
- 8x8 orientation retained from measured captures; grid labels clarified.
- Recovery backing-up phases and top/notch mismatch rejection disabled.
- Drive scale 90%; pickup angle 113 degrees; seating wait 1.5 seconds after reaching the pickup angle.
- Up to three pickup attempts when metal remains detected after a completed cycle.
- The third recorded successful pickup ends the round immediately in place. The arm is parked and the magnet remains at its existing 60% holding duty until power off.
- GO starts after release; another press stops the run. A stopped round cannot restart through GO.

The nav build and upload succeeded. User confirmation is the physical-run evidence; no new automated tests were run for this commit request. The existing geometry test previously passed 260 assertions. Pickup success is inferred from the completed crane cycle and metal leaving the notch, not a separate storage sensor.

Earlier source and firmware backups remain outside this repository in the local sibling folders `WORK NAV FIRST ROUND USE` and `WORKING BOT GO GO GO`.
