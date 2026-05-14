#ifndef MISSION_CONFIG_H
#define MISSION_CONFIG_H

#include <stdint.h>

/*
 * MissionRouteTuner mission-step movement values.
 *
 * Keep these aligned with Motor/MissionConfig.h before field testing. Arduino IDE
 * builds sketches as isolated folders, so the tuner keeps a local copy instead
 * of including the Motor sketch's header directly.
 */
class MissionConfig
{
public:
  static const uint8_t MAX_MISSION_BLOCKS = 8;
  static const uint8_t STORAGE_PICKUP_REGION_COUNT = 2;
  static const uint8_t STORAGE_GRIP_TARGET_COUNT = 2;

  struct PoseConfig
  {
    uint8_t missionZoneStartId = 6;  // 7번 = 1번 칸
    uint8_t missionZoneSlotCount = 8; // 현재 보정된 배치 자세는 7~14번
  };

  struct MissionOrderConfig
  {
    uint8_t blockSignatureMap = 0b00111111;
    bool dynamicBlockCount = true;
    uint8_t blockCount = 8;
    uint8_t goalPositions[MAX_MISSION_BLOCKS] = {
        1, 2, 3, 4, 5, 6, 7, 8
    };
  };

  struct PsdConfig
  {
    int16_t approachFl = 480;
    int16_t approachFr = 500;
    int16_t approachTolerance = 5;
    // SL 값이 클수록 왼쪽 벽에 더 가까운 위치라서, 값을 올리면 덜 오른쪽에서 멈춘다.
    int16_t missionSl = 640;
    int16_t missionTolerance = 5;
    // 적재함 1열을 처음 보는 기준 위치. Side-right PSD는 정렬에 사용하지 않는다.
    int16_t alignFl = 266;
    int16_t alignSl = 354;
    int16_t alignFr = 269;
    int16_t alignTolerance = 8;
    // 상층 집기 직전 기준 위치. 스캔 위치보다 적재함에 더 붙어서 그립한다.
    int16_t gripAlignFl = 349;
    int16_t gripAlignSl = 359;
    int16_t gripAlignFr = 363;
    int16_t gripAlignTolerance = 8;
    // 하층 집기 직전 기준 위치. 상층보다 덜 깊게 들어간다.
    int16_t lowerGripAlignFl = 325;
    int16_t lowerGripAlignSl = 354;
    int16_t lowerGripAlignFr = 337;
    int16_t lowerGripAlignTolerance = 8;
    // 미션수행존 배치 직전 기준 위치. FL/SR은 이 위치에서 불안정해서 SL+FR만 쓴다.
    int16_t missionZoneSl = 635;
    int16_t missionZoneFr = 220;
    int16_t missionZoneTolerance = 8;
    int16_t scanSl = 526;
    int16_t scanSlTolerance = 5;
    int16_t storageScanFrNoObstacle = 220;
  };

  struct FrontApproachConfig
  {
    int16_t firstDetectAdc = 350;
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
    uint8_t upperRowSlots[4] = {1, 2, 3, 4};
    uint8_t lowerRowSlots[4] = {5, 6, 7, 8};
    uint8_t pickSlotOrder[8] = {1, 5, 2, 6, 3, 7, 4, 8};
    uint8_t pickSlotCount = 8;
    uint16_t perSlotScanMs = 1800;
    float scanColumnStepMm = 72.0;
    int32_t scanColumnMoveMmPerSec = 150;
    uint8_t scanFramesPerStop = 5;
    uint16_t scanMinBlockArea = 0;
    int16_t columnXCenters[4] = {70, 125, 180, 235};
    int16_t columnXTolerance = 35;
  };

  struct StoragePickupRegionConfig
  {
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
    bool yErrorUsesForwardDirection = true;
  };

  struct CameraScanConfig
  {
    uint8_t maxSignature = 7;
    uint8_t missionInstructionAllowedSignatureMap = 0b00111111;
    uint8_t storageAllowedSignatureMap = 0b00111111;
    bool missionInstructionLampOn = false;
    int16_t storageXSetpoint = 157;
    int16_t storageXTolerance = 6;
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

  struct WaitConfig
  {
    uint16_t buttonDebounceMs = 120;
    uint16_t driveSettleMs = 80;
    uint16_t blockFeedbackMs = 120;
    uint16_t scanSampleMs = 30;
    uint16_t cameraLampMs = 250;
    uint16_t gripperActionMs = 350;
    uint16_t pose800Ms = 850;
    uint16_t pose1000Ms = 1050;
    uint16_t pose1300Ms = 1100;
    uint16_t pose1500Ms = 1200;
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
  WaitConfig wait;
  FinishReturnConfig finishReturn;

  static const MissionConfig &get()
  {
    static const MissionConfig instance;
    return instance;
  }

private:
  MissionConfig() {}
  MissionConfig(const MissionConfig &);
  MissionConfig &operator=(const MissionConfig &);
};

#endif
