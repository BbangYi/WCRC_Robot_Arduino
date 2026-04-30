# wcrc-logistics-robot-motor

WCRC Logistics Robot 2 - Motor Mission.

`Motor.ino` is an Arduino mission sketch for the MobileManipulator Blackberry platform.
This version targets the WCRC logistics robot 2 sorting mission and keeps the full mission flow in one sketch.

## Current Mission Flow

1. Initialize Dynamixel, Pixy2, PSD sensors, gripper, RGB LED, then wait for the start button.
2. Drive forward until the front PSD first detects the obstacle, slow down, then align to the mission-instruction zone with the left PSD.
3. Scan the mission-instruction blocks with Pixy2 and sort the detected signatures from left to right.
4. Move toward the storage rack with distance-based position control, move right, then perform SL + FR PSD fine alignment.
5. Scan the storage rack, pick matching blocks, move to the mission zone, and place each block at the configured zone pose.
6. Return to the Finish Zone by reversing while tracking the left-side obstacle boundary, then add a timed reverse margin.

## Project Structure

| File | Role |
| --- | --- |
| `Motor.ino` | Main 6-step autonomous mission sequence |
| `MissionConfig.h` | Single configuration entry point for mission order, PSD thresholds, waits, timeouts, and distances |
| `PROJECT_RULES.md` | Repo, configuration, storage-rack, and test rules |
| `ops/` | Git-tracked operations records for field tests, EEPROM poses, safety checks, and design decisions |
| `Mobilebase.cpp` / `Mobilebase.h` | Mecanum mobile-base control, velocity mode, position mode, PSD-assisted movement |
| `Manipulator.h` | EEPROM-based manipulator pose execution |
| `PSD.cpp` / `PSD.h` | Front and side PSD sensor reads |
| `Pixy.cpp` / `Pixy.h` / `Camera.h` | Pixy2 initialization and camera tuning constants |
| `Gripper.cpp` / `Gripper.h` | Pixy2 servo gripper open / close |
| `RGBLED.h` | RGB LED feedback helpers |
| `Pins.h` | Button, LED, buzzer, and PSD pin assignments |

## Configuration Rules

- Edit mission tuning values in `MissionConfig.h` first.
- Do not add new mission-tuning `#define` values in `Motor.ino`.
- Keep execution logic in `Motor.ino`; keep field values, timeouts, waits, thresholds, and mission order in `MissionConfig.h`.
- Use names that include physical units when useful: `Mm`, `Ms`, `Adc`.
- Keep low-level pin assignments in `Pins.h`.
- Keep low-level mobile-base motor defaults in `Mobilebase.h`.
- Use `CFG.speed` for mission-level mobile-base speeds.
- Treat `MOBILEBASE_DEFAULT_DRIVING_SPEED` and `MOBILEBASE_DEFAULT_DRIVING_MM_PER_S` as fallback defaults for the low-level mobile-base helper.

## Operations Records

- Keep the operational source of truth in `ops/`.
- Use `ops/events.jsonl` for code changes, field tests, safety checks, and rollback notes.
- Use `ops/eeprom-poses.json` for pose ID 1-14 verification status and motor values.
- Use `ops/decisions.md` for design decisions that should outlive a single test run.
- Use `ops/safety-checklist.md` before upload and field testing.
- The Public&Private private console may read these files as a dashboard, but Git remains the record authority.

## Current Field-Tuning Notes

- `CFG.front.firstDetectAdc`: `350`
- `CFG.front.cruiseSpeed`: `250`
- `CFG.front.slowSpeed`: `150`
- `CFG.speed.psdCorrectionSpeed`: `200`
- `CFG.speed.cameraFineTuneSpeed`: `200`
- `CFG.speed.storageScanSpeed`: `200`
- `CFG.speed.returnSpeed`: `200`
- `CFG.speed.positionMoveMmPerSec`: `150`
- `CFG.storageDrive.firstForwardMm`: `450.0`
- `CFG.storageDrive.extraForwardMm`: `350.0`
- `CFG.storageDrive.rightMm`: `60.0`
- `CFG.finishReturn.finishExtraMs`: `3000`
- `CFG.mission.blockCount`: `3`
- `CFG.mission.goalPositions`: currently `1, 2, 3, 4, 5, 6`

Tune the PSD thresholds and mission positions on the actual competition field before running an official attempt.

## Fixed Storage Rack Convention

Storage-rack numbering is fixed and documented in `PROJECT_RULES.md`.

```text
1  2  3  4
5  6  7  8
```

Current pick logic scans the storage rack with Pixy2 and picks by target signature. It does not currently drive to a hard-coded storage slot number.

## Required Hardware / Library Context

- Arduino-compatible controller used in the class reference kit
- Dynamixel2Arduino
- Dynamixel motors for mobile base IDs `1` to `4`
- Dynamixel motors for manipulator IDs expected by `Manipulator.h`
- Pixy2 over SPI
- PSD sensors: `FL=A0`, `SL=A1`, `FR=A2`, `SR=A3`
- EEPROM manipulator poses saved before mission execution

## EEPROM Pose Map

| ID | Pose |
| --- | --- |
| `1` | INITIAL / mission-instruction camera pose |
| `2` | STORAGE / safe folded pose |
| `3` | PRE_GRIP_UPPER |
| `4` | GRIP_UPPER |
| `5` | PRE_GRIP_LOWER |
| `6` | GRIP_LOWER |
| `7` to `14` | Mission-zone placement poses `1` to `8` |

## Before Uploading to the Robot

1. Set the competition-day block count in `CFG.mission.blockCount`.
2. Set destination zones in `CFG.mission.goalPositions`.
3. Verify all EEPROM manipulator poses.
4. Check Pixy2 signatures against `CFG.mission.blockSignatureMap`.
5. Re-check PSD thresholds on the real field.
6. Run the mission with the robot lifted once to confirm wheel directions before field testing.
