# Project Rules

## Repository Scope

- Track the Arduino sketch folder `Motor/` as the repository root.
- Do not commit `.DS_Store`, editor folders, logs, Arduino build outputs, or generated binaries.
- Keep course reference code outside this repo unless a file is intentionally copied into `Motor/`.

## Configuration

- Change mission tuning values in `MissionConfig.h` first.
- Do not add mission-tuning `#define` values to `Motor.ino`.
- Keep low-level pin assignments in `Pins.h`.
- Keep low-level mobile-base ID / control-table defaults in `Mobilebase.h`.
- Pass mission speeds from `CFG.speed` explicitly in `Motor.ino`.
- Treat `MOBILEBASE_DEFAULT_DRIVING_SPEED` and `MOBILEBASE_DEFAULT_DRIVING_MM_PER_S` as low-level fallback defaults only.

## Storage Rack Convention

The physical storage rack numbering is fixed:

```text
camera side

1  2  3  4
5  6  7  8
```

- Keep this numbering in `CFG.storageRack`.
- Current picking logic does not drive to a configured storage slot number.
- Current picking logic scans with Pixy2, centers the target signature on X, and uses Pixy2 Y to choose upper/lower grip pose.
- If slot-addressed picking is added later, implement it against the fixed `1 2 3 4 / 5 6 7 8` convention.

## Mission-Zone Convention

- `CFG.mission.goalPositions` is the destination order announced on competition day.
- Mission-zone placement pose ID is `CFG.pose.missionZoneStartId + goalPosition`.
- EEPROM pose IDs `7` to `14` must match mission-zone positions `1` to `8`.

## Change Discipline

- Prefer small commits that keep robot behavior reviewable.
- Document changed field-tuning values in `README.md`.
- After any drivetrain change, test once with wheels lifted before field testing.
- After any manipulator/pose change, verify EEPROM pose existence before a full autonomous run.
