# Project Rules

Repository name: `wcrc-logistics-robot-motor`

## Repository Scope

- Track the Arduino sketch folder `Motor/` as the repository root.
- Do not commit `.DS_Store`, editor folders, logs, Arduino build outputs, or generated binaries.
- Keep course reference code outside this repo unless a file is intentionally copied into `Motor/`.
- Keep field-operation records in `ops/` so Git history can explain code, tuning, EEPROM, and test decisions together.

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
- Current picking logic detects whether a Pixy2 block center is inside configured pickup regions.
- Tune `CFG.storagePickupRegion` from `MissionRouteTuner` using `pixy storage lower` / `pixy storage all`.
- Default pick order is `1, 5, 2, 6, 3, 7, 4, 8`; the current field-test run caps this to the first 6 source slots.

## Mission-Zone Convention

- `CFG.mission.goalPositions` is the destination order announced on competition day. The current field-test default uses mission-zone slots `1~6`.
- Mission-zone placement pose ID is `CFG.pose.missionZoneStartId + goalPosition`.
- EEPROM pose IDs `7` to `12` must match mission-zone positions `1` to `6`.

## Change Discipline

- Prefer small commits that keep robot behavior reviewable.
- Record behavior-affecting changes in `ops/events.jsonl` with the Git branch or commit.
- Record EEPROM pose verification in `ops/eeprom-poses.json`.
- Record durable design choices in `ops/decisions.md`.
- Document changed field-tuning values in `README.md`.
- After any drivetrain change, test once with wheels lifted before field testing.
- After any manipulator/pose change, verify EEPROM pose existence before a full autonomous run.
