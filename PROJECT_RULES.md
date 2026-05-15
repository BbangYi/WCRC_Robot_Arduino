# Project Rules

Repository name: `wcrc-logistics-robot-motor`

## Repository Scope

- Track the Arduino sketch folder `Motor/` as the repository root.
- Do not commit `.DS_Store`, editor folders, logs, Arduino build outputs, or generated binaries.
- Keep course reference code outside this repo unless a file is intentionally copied into `Motor/`.
- Keep field-operation records in `ops/` so Git history can explain code, tuning, EEPROM, and test decisions together.

## Configuration

- Change shared mission tuning values in `MissionRouteTuner/WcrcMissionSharedConfig.h` first.
- Keep `Motor/MissionConfig.h` and `MissionRouteTuner/MissionConfig.h` as thin sketch-specific wrappers.
- Put only Motor-only autonomous timing in `Motor/MissionConfig.h`.
- Put only tuner console/test timing in `MissionRouteTuner/MissionConfig.h`.
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
- `CFG.storageRack.pickSlotOrder` is legacy fallback/debug data. The autonomous mission uses storage survey detections as the real source positions.

## Mission-Zone Convention

- Current autonomous placement uses the detected storage source slot as the mission-zone goal slot: `goalSlot = sourceSlot`.
- `CFG.mission.goalPositions` is retained as legacy/manual fallback data.
- Mission-zone placement pose ID is `CFG.pose.missionZoneStartId + goalSlot`.
- EEPROM pose IDs `7` to `14` must match mission-zone positions `1` to `8`.

## Change Discipline

- Prefer small commits that keep robot behavior reviewable.
- Record behavior-affecting changes in `ops/events.jsonl` with the Git branch or commit.
- Record EEPROM pose verification in `ops/eeprom-poses.json`.
- Record durable design choices in `ops/decisions.md`.
- Document changed field-tuning values in `README.md`.
- After any drivetrain change, test once with wheels lifted before field testing.
- After any manipulator/pose change, verify EEPROM pose existence before a full autonomous run.
