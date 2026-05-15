#ifndef WCRC_MISSION_SHARED_CONFIG_H
#define WCRC_MISSION_SHARED_CONFIG_H

#include <stdint.h>

/*
 * Shared mission contract for Motor.ino and MissionRouteTuner.
 *
 * Put values here only when both sketches must agree on the same field
 * calibration or mission contract. Keep sketch-specific timing/UI behavior in
 * each sketch's local MissionConfig.h wrapper.
 */
class WcrcMissionSharedConfig
{
public:
  static const uint8_t MAX_MISSION_BLOCKS = 8;
  static const uint8_t STORAGE_PICKUP_REGION_COUNT = 2;
  static const uint8_t STORAGE_GRIP_TARGET_COUNT = 2;

  struct PoseConfig
  {
    uint8_t missionZoneStartId = 6;   // pose 7 = slot 1
    uint8_t missionZoneSlotCount = 8; // pose 7~14
  };

  struct MissionOrderConfig
  {
    uint8_t blockSignatureMap = 0b00111111; // sig1=bit0, sig2=bit1, ...
    bool dynamicBlockCount = true;
    uint8_t blockCount = 8;

    // Legacy/manual fallback only. The autonomous mission now places to the
    // slot detected during the storage survey: goalSlot = sourceSlot.
    uint8_t goalPositions[MAX_MISSION_BLOCKS] = {
        1, 2, 3, 4, 5, 6, 7, 8
    };
  };

  struct PsdConfig
  {
    // Legacy FL/FR approach targets. Current approach uses front.firstDetectAdc.
    int16_t approachFl = 500;
    int16_t approachFr = 510;
    int16_t approachTolerance = 5;

    // Mission-instruction side alignment. Larger SL means closer to the left wall.
    int16_t missionSl = 640;
    int16_t missionTolerance = 5;

    // Storage rack scan position for column 1. SR is intentionally ignored.
    int16_t alignFl = 266;
    int16_t alignSl = 354;
    int16_t alignFr = 269;
    int16_t alignTolerance = 8;

    // Upper-row grip depth position. FL/FR are used for front-depth control.
    // SL is kept only as a reference/log value so grip approach does not strafe
    // sideways after Pixy center alignment.
    int16_t gripAlignFl = 349;
    int16_t gripAlignSl = 359;
    int16_t gripAlignFr = 363;
    int16_t gripAlignTolerance = 8;

    // Lower-row grip depth position. FL/FR are used; SL is reference/log only.
    int16_t lowerGripAlignFl = 325;
    int16_t lowerGripAlignSl = 354;
    int16_t lowerGripAlignFr = 337;
    int16_t lowerGripAlignTolerance = 8;

    // Mission-zone placement position. SL+FR are used; SR is ignored.
    int16_t missionZoneSl = 635;
    int16_t missionZoneFr = 220;
    int16_t missionZoneTolerance = 8;

    // Tuner/manual scan helpers.
    int16_t scanSl = 526;
    int16_t scanSlTolerance = 5;
    int16_t storageScanFrNoObstacle = 220;
  };

  struct FrontApproachConfig
  {
    int16_t firstDetectAdc = 500;
    int16_t decelWindowAdc = 40;
    int32_t cruiseSpeed = 250;
    int32_t slowSpeed = 150;
    int32_t brakeStep = 15;
    uint16_t brakeDelayMs = 60;
  };

  struct SpeedConfig
  {
    int32_t psdCorrectionSpeed = 200;
    int32_t cameraFineTuneSpeed = 140;
    int32_t storageScanSpeed = 200;
    int32_t returnSpeed = 200;
    int32_t positionMoveMmPerSec = 150;
  };

  struct StorageDriveConfig
  {
    float firstForwardMm = 450.0;
    float extraForwardMm = 350.0;
    float rightMm = 60.0;
  };

  struct StorageRackLayoutConfig
  {
    // Fixed numbering: upper row 1 2 3 4, lower row 5 6 7 8.
    uint8_t upperRowSlots[4] = {1, 2, 3, 4};
    uint8_t lowerRowSlots[4] = {5, 6, 7, 8};

    // Legacy fallback/debug order only. Survey results are the real source.
    uint8_t pickSlotOrder[8] = {1, 5, 2, 6, 3, 7, 4, 8};
    uint8_t pickSlotCount = 8;
    uint16_t perSlotScanMs = 1800; // legacy fallback; column survey is primary

    float scanColumnStepMm = 72.0;
    int32_t scanColumnMoveMmPerSec = 150;
    uint8_t scanFramesPerStop = 5;
    uint16_t scanMinBlockArea = 0;

    // Pixy x-center candidates for storage columns when the robot is aligned.
    int16_t columnXCenters[4] = {70, 125, 180, 235};
    int16_t columnXTolerance = 35;

    // Physical notes only. These are not used directly by the mission loops.
    uint16_t nominalCellPitchMm = 60;
    uint16_t nominalOuterLengthMm = 270;
    uint16_t nominalOuterWidthMm = 130;
    uint16_t nominalOuterHeightMm = 82;
  };

  struct StoragePickupRegionConfig
  {
    // Pixy center must be inside one of these windows to count as pickable.
    int16_t xMin[STORAGE_PICKUP_REGION_COUNT] = {129, 139};
    int16_t xMax[STORAGE_PICKUP_REGION_COUNT] = {180, 178};
    int16_t yMin[STORAGE_PICKUP_REGION_COUNT] = {57, 155};
    int16_t yMax[STORAGE_PICKUP_REGION_COUNT] = {110, 207};
    int16_t xMargin = 0;
    int16_t yMargin = 0;
    uint8_t useUpperGripPose[STORAGE_PICKUP_REGION_COUNT] = {1, 0};
  };

  struct StorageGripTargetConfig
  {
    // Pixy center target windows before final grip-depth approach.
    int16_t xMin[STORAGE_GRIP_TARGET_COUNT] = {129, 139};
    int16_t xMax[STORAGE_GRIP_TARGET_COUNT] = {180, 178};
    int16_t yMin[STORAGE_GRIP_TARGET_COUNT] = {57, 155};
    int16_t yMax[STORAGE_GRIP_TARGET_COUNT] = {110, 207};
    int16_t xMargin = 0;
    int16_t yMargin = 0;
    int16_t centerToleranceX = 4;
    int16_t centerToleranceY = 4;
    uint8_t useUpperGripPose[STORAGE_GRIP_TARGET_COUNT] = {1, 0};

    uint16_t alignTimeoutMs = 5000;
    uint16_t alignStepMs = 15;
    uint16_t alignSettleMs = 0;
    int16_t alignFullSpeedPixelError = 28;
    float upperExtraForwardMm = 8.0;
    float lowerExtraForwardMm = 5.0;
    int32_t extraForwardMmPerSec = 60;
    float fineAlignGain = 0.45;
    float fineAlignMinStepMm = 1.0;
    float fineAlignMaxStepMm = 6.0;
    float fineAlignForwardMaxStepMm = 3.0;
    int32_t fineAlignSpeedMmPerSec = 35;
    bool fineAlignUseY = true;

    // If field testing proves forward/backward is reversed for y-error, flip this.
    bool yErrorUsesForwardDirection = true;
  };

  struct CameraScanConfig
  {
    // Pixy2 CCC signatures are learned in PixyMon. These maps only filter them.
    uint8_t maxSignature = 7;
    uint8_t missionInstructionAllowedSignatureMap = 0b00111111;
    uint8_t storageAllowedSignatureMap = 0b00111111;
    bool missionInstructionLampOn = false;

    // Legacy center check used by older helpers. Pickup/grip regions are primary.
    int16_t storageXSetpoint = 157;
    int16_t storageXTolerance = 6;

    // Storage rack visible boundary in Pixy coordinates.
    int16_t storageBoundaryXMin = 94;
    int16_t storageBoundaryXMax = 235;
    int16_t storageBoundaryYMin = 4;
    int16_t storageBoundaryYMax = 205;
    int16_t storageBoundaryMargin = 2;
    int16_t storageYUpperLowerSplit = 132;

    uint16_t missionInstructionMinBlockArea = 80;
    uint16_t storageMinBlockArea = 80;
  };

  struct TimeoutConfig
  {
    unsigned long psdLoopMs = 10000;
    unsigned long positionMoveMs = 10000;
    unsigned long scanPhaseMs = 8000;
    unsigned long fineTuneMs = 3000;
    unsigned long returnPhaseMs = 10000;
    unsigned long missionTimeLimitMs = 115000;
  };

  struct FinishReturnConfig
  {
    int16_t boundaryAdc = 220;
    int16_t trackSl = 317;
    int16_t trackTolerance = 20;
    int32_t correctionMaxSpeed = 40;
    int32_t openSideLeftBiasSpeed = 25;
    int32_t wheelMaxSpeed = 200;
    unsigned long finishExtraMs = 3000;
  };

  PoseConfig pose;
  MissionOrderConfig mission;
  PsdConfig psd;
  FrontApproachConfig front;
  SpeedConfig speed;
  StorageDriveConfig storageDrive;
  StorageRackLayoutConfig storageRack;
  StoragePickupRegionConfig storagePickupRegion;
  StorageGripTargetConfig storageGripTarget;
  CameraScanConfig cameraScan;
  TimeoutConfig timeout;
  FinishReturnConfig finishReturn;
};

#endif
