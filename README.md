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
- Use `CFG.poseTiming` for manipulator motion profile times.
- Use `CFG.wait.poseSettleMs` only as the short settle time after each manipulator profile.
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
- `CFG.psd.missionSl`: `540` (`SL` 목표값. 값을 올리면 미션지시존에서 덜 오른쪽에 멈춥니다.)
- `CFG.speed.psdCorrectionSpeed`: `200`
- `CFG.speed.cameraFineTuneSpeed`: `140`
- `CFG.speed.storageScanSpeed`: `200`
- `CFG.speed.returnSpeed`: `200`
- `CFG.speed.positionMoveMmPerSec`: `150`
- `CFG.poseTiming.startupInitialMs`: `1200`
- `CFG.poseTiming.missionInstructionMs`: `800`
- `CFG.poseTiming.storageMs`: `700`
- `CFG.poseTiming.preGripMs`: `750`
- `CFG.poseTiming.gripMs`: `850`
- `CFG.poseTiming.storageWithBlockMs`: `800`
- `CFG.poseTiming.missionZoneTurnMs`: `650`
- `CFG.poseTiming.missionZonePlaceMs`: `800`
- `CFG.poseTiming.finishStorageMs`: `650`
- `CFG.poseTiming.finishInitialMs`: `800`
- `CFG.wait.poseSettleMs`: `100`
- `CFG.wait.gripperActionMs`: `250`
- `CFG.cameraScan.missionInstructionMinBlockArea`: `80`
- `CFG.cameraScan.missionInstructionLampOn`: `false`
- `CFG.cameraScan.storageMinBlockArea`: `80`
- `CFG.cameraScan.storageYUpperLowerSplit`: `132`
- `CFG.psd.alignFl/alignSl/alignFr`: `266 / 354 / 269` (적재함 1열 스캔 기준 위치, SR ignored)
- `CFG.psd.alignTolerance`: `8`
- `CFG.psd.gripAlignFl/gripAlignSl/gripAlignFr`: `349 / 359 / 363` (상층 집기 직전)
- `CFG.psd.lowerGripAlignFl/lowerGripAlignSl/lowerGripAlignFr`: `325 / 354 / 337` (하층 집기 직전)
- `CFG.psd.missionZoneSl/missionZoneFr`: `635 / 220` (미션수행존 배치 기준, SL+FR)
- `CFG.psd.missionZoneTolerance`: `8`
- `CFG.storagePickupRegion`: upper `(129,57)~(180,110)`, lower `(139,155)~(178,207)`, `margin=0`
- `CFG.storageGripTarget`: upper `(129,57)~(180,110)`, lower `(139,155)~(178,207)`, center tolerance `4/4`
- `CFG.storageGripTarget.alignTimeoutMs`: `5000`
- `CFG.storageGripTarget.alignStepMs`: `15`
- `CFG.storageGripTarget.upperExtraForwardMm/lowerExtraForwardMm`: `8.0 / 5.0` at `60mm/s`
- `CFG.storageGripTarget.fineAlignGain`: `0.45`
- `CFG.storageGripTarget.fineAlignMinStepMm/fineAlignMaxStepMm`: `1.0 / 6.0`
- `CFG.storageGripTarget.fineAlignForwardMaxStepMm`: `3.0`
- `CFG.storageGripTarget.fineAlignSpeedMmPerSec`: `35`
- `CFG.storageRack.pickSlotOrder`: legacy fallback/debug order, currently `1, 5, 2, 6, 3, 7, 4, 8`
- `CFG.storageRack.scanColumnStepMm`: `72.0`
- `CFG.storageRack.scanColumnMoveMmPerSec`: `150`
- `CFG.storageRack.scanFramesPerStop`: `5`
- `CFG.storageRack.scanMinBlockArea`: `0`
- `CFG.storageRack.perSlotScanMs`: `1800`; legacy fallback only. The main mission uses fixed column-step scans.
- `CFG.storageRack.columnXCenters`: currently `70, 125, 180, 235`
- `CFG.storageRack.columnXTolerance`: `35`
- `CFG.storageDrive.firstForwardMm`: `450.0`
- `CFG.storageDrive.extraForwardMm`: `350.0`
- `CFG.storageDrive.rightMm`: `60.0`
- `CFG.finishReturn.finishExtraMs`: `3000`
- `CFG.mission.dynamicBlockCount`: `true`
- `CFG.pose.missionZoneSlotCount`: `8`, so placement uses EEPROM pose `7~14`
- `CFG.mission.blockCount`: `8` when dynamic count is disabled
- `CFG.mission.goalPositions`: currently `1, 2, 3, 4, 5, 6, 7, 8`

Tune the PSD thresholds and mission positions on the actual competition field before running an official attempt.
If the Pixy area filter hides valid blocks during testing, set the relevant `MinBlockArea` value to `0` first, then retune from measured `pixy scan` area values.
The mission-instruction zone no longer pauses for SW1 before scanning. It keeps the Pixy lamp off by default through `missionInstructionLampOn`, scans with `missionInstructionMinBlockArea`, then retries once with area `0` if nothing passes the filter. If both fail, it prints raw Pixy frames with signature, x/y, area, map pass, and area pass before returning.
The production `Motor` mission only uses SW1 for the initial start. Step-by-step SW1 progression is a `MissionRouteTuner` feature and is not used inside the autonomous mission flow.

## Fixed Storage Rack Convention

Storage-rack numbering is fixed and documented in `PROJECT_RULES.md`.

```text
1  2  3  4
5  6  7  8
```

Current pick logic first surveys the storage rack from column 1 to 4, records every target block as `signature/sourceSlot/column/layer/x/y/area`, then builds a pick/place queue from the mission-instruction signatures. The mission-zone destination uses the detected storage slot directly: `goalSlot = sourceSlot`.
The current field-test run supports eight destination slots. Slot `1~8` maps to EEPROM pose `7~14`, so `poseId = 6 + slot`. For example, storage column 3 lower is `sourceSlot=7`, `goalSlot=7`, and EEPROM pose `13`.
`CFG.storageRack.pickSlotOrder` is retained as a legacy fallback/debug contract, but the autonomous mission no longer treats it as the real source position list.
Storage picking now requires both the mission-instruction signature and the configured pickup region to match before gripping.
If a mission-instruction scan sees zero blocks, the robot skips storage picking and returns instead of retrying forever.
If the storage survey cannot match one of the requested signatures, that block is logged as skipped and the loop advances without repeating the same search forever.
After all detected signatures are placed, the robot does not re-align to the storage rack; it starts the finish reverse from the current mission-zone position.
Use `MissionRouteTuner` command `mission survey`, then `mission plan` to inspect the detected queue. Use `pixy alignslow lower 5000 0` to test the same low-speed micro-step center alignment used by the main mission.

Upper-row gripping uses a staged joint order `m4 -> m3 -> m2` for the upper grip pose and the lift back to storage. This is intentionally slower than a single sync move because it reduces RFID-card interference while lifting the block.

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
| `2` | Reserved / manual test |
| `3` | STORAGE_VIEW_SAFE / storage camera and safe folded pose |
| `4` | GRIP_UPPER |
| `5` | GRIP_LOWER |
| `6` | Reserved / manual test |
| `7` to `14` | Mission-zone placement poses `1` to `8` |

## Before Uploading to the Robot

1. Keep `CFG.mission.dynamicBlockCount=true` if the mission-instruction camera should decide the count, or set `blockCount` for a fixed run.
2. Set destination zones in `CFG.mission.goalPositions`.
3. Verify required EEPROM manipulator poses: `1,3,4,5,7~14`.
4. Check Pixy2 signatures against `CFG.mission.blockSignatureMap`.
5. Check Pixy block areas with `MissionRouteTuner` so `missionInstructionMinBlockArea` and `storageMinBlockArea` do not hide valid blocks.
6. Check storage slot x/y mapping with `pixy storage lower` and `pixy storage all`.
7. Re-check PSD thresholds on the real field. If the mission-instruction stop is still too far right, raise `CFG.psd.missionSl` by 10-20; if it is too far left, lower it.
8. Run the mission with the robot lifted once to confirm wheel directions before field testing.
