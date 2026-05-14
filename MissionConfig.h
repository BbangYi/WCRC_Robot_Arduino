#ifndef MISSION_CONFIG_H
#define MISSION_CONFIG_H

#include <stdint.h>

/*
 * 미션 튜닝값의 단일 원본.
 *
 * 규칙:
 * 1. 대회 당일 바꾸는 값은 가능한 한 이 파일에서만 수정한다.
 * 2. Motor.ino에는 새 #define 튜닝값을 추가하지 않는다.
 * 3. 값 이름은 사용 위치가 아니라 의미 기준으로 정한다.
 * 4. mm, ms, ADC처럼 단위가 있는 값은 이름에 단위를 남긴다.
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
    uint8_t blockSignatureMap = 0b00111111; // sig1=bit0, sig2=bit1, ...
    bool dynamicBlockCount = true;          // true면 미션지시존에서 인식한 개수를 사용
    uint8_t blockCount = 8;                 // dynamicBlockCount=false일 때 사용할 고정 개수
    uint8_t goalPositions[MAX_MISSION_BLOCKS] = {
        1, 2, 3, 4, 5, 6, 7, 8 // 실제 테스트 기본값: pose 7~14 사용
    };
  };

  struct PsdConfig
  {
    // 2단계: 기존 FL/FR 목표값. 현재는 front.firstDetectAdc 기반 접근을 사용한다.
    int16_t approachFl = 500;
    int16_t approachFr = 510;
    int16_t approachTolerance = 5;

    // 2단계: 미션지시존 위치 정렬.
    // SL 값이 클수록 왼쪽 벽에 더 가까운 위치라서, 값을 올리면 덜 오른쪽에서 멈춘다.
    int16_t missionSl = 540;
    int16_t missionTolerance = 5;

    // 4단계: 적재함 1열을 처음 보는 기준 위치. SR은 정렬에 사용하지 않는다.
    int16_t alignFl = 266;
    int16_t alignSl = 354;
    int16_t alignFr = 269;
    int16_t alignTolerance = 8;

    // 5단계: 집기 직전 기준 위치. 상층은 더 깊게, 하층은 조금 덜 깊게 들어간다.
    int16_t gripAlignFl = 349;
    int16_t gripAlignSl = 359;
    int16_t gripAlignFr = 363;
    int16_t gripAlignTolerance = 8;
    int16_t lowerGripAlignFl = 325;
    int16_t lowerGripAlignSl = 354;
    int16_t lowerGripAlignFr = 337;
    int16_t lowerGripAlignTolerance = 8;

    // 5단계: 미션수행존 / 적재함 스캔
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
    // Velocity-control helpers. Keep these explicit in Motor.ino calls instead of relying on Mobilebase.h defaults.
    int32_t psdCorrectionSpeed = 200;
    int32_t cameraFineTuneSpeed = 140;
    int32_t storageScanSpeed = 200;
    int32_t returnSpeed = 200;

    // Extended-position / time-based profile helpers.
    int32_t positionMoveMmPerSec = 150;
  };

  struct StorageRackLayoutConfig
  {
    // 고정 번호 규칙: 윗줄 1 2 3 4, 아랫줄 5 6 7 8.
    uint8_t upperRowSlots[4] = {1, 2, 3, 4};
    uint8_t lowerRowSlots[4] = {5, 6, 7, 8};
    uint8_t pickSlotOrder[8] = {1, 5, 2, 6, 3, 7, 4, 8};
    uint8_t pickSlotCount = 8;
    uint16_t perSlotScanMs = 1800; // 한 칸/한 후보를 훑는 시간. 현장 테스트 후 1500~2500ms에서 조정한다.

    // 적재함 기준 위치에서 Pixy 화면에 보이는 각 열의 x 중심 후보.
    // MissionRouteTuner의 pixy storage lower로 현장 확인 후 조정한다.
    int16_t columnXCenters[4] = {70, 125, 180, 235};
    int16_t columnXTolerance = 35;

    uint16_t nominalCellPitchMm = 60;
    uint16_t nominalOuterLengthMm = 270;
    uint16_t nominalOuterWidthMm = 130;
    uint16_t nominalOuterHeightMm = 82;
  };

  struct StoragePickupRegionConfig
  {
    // Pixy 화면의 블록 중심점이 이 영역 안에 들어오면 집기 후보로 본다.
    // 1=upper, 2=lower. 튜너 MissionRouteTuner와 같은 계약을 유지한다.
    int16_t xMin[STORAGE_PICKUP_REGION_COUNT] = {129, 139};
    int16_t xMax[STORAGE_PICKUP_REGION_COUNT] = {180, 178};
    int16_t yMin[STORAGE_PICKUP_REGION_COUNT] = {57, 155};
    int16_t yMax[STORAGE_PICKUP_REGION_COUNT] = {110, 207};
    int16_t xMargin = 0;
    int16_t yMargin = 0;

    // 매니퓰레이터 pose는 현재 상/하층 2종류만 사용한다.
    uint8_t useUpperGripPose[STORAGE_PICKUP_REGION_COUNT] = {1, 0};
  };

  struct StorageGripTargetConfig
  {
    // pickup region에서 블록을 찾은 뒤, 그립 직전 Pixy 중심점이 들어가야 하는 목표 창.
    // 1=upper, 2=lower. 최종 깊이는 아래 extraForwardMm에서 보정한다.
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

    // yError 양수일 때 사용할 기준 축. 현장에서 반대로 움직이면 false로 바꾼다.
    bool yErrorUsesForwardDirection = true;
  };

  struct StorageDriveConfig
  {
    float firstForwardMm = 450.0;
    float extraForwardMm = 350.0;
    float rightMm = 60.0;
  };

  struct CameraScanConfig
  {
    // Pixy2 CCC signature는 PixyMon에서 학습된 1~7번 안에서만 동작한다.
    // 아래 값은 새 색상 세트로 교체하는 값이 아니라, 이미 학습된 signature 중
    // 어느 번호를 받을지 고르는 필터다. bit0=sig1, bit1=sig2 ...
    uint8_t maxSignature = 7;
    uint8_t missionInstructionAllowedSignatureMap = 0b00111111;
    uint8_t storageAllowedSignatureMap = 0b00111111;
    bool missionInstructionLampOn = false;

    // 스토리지에서 목표 블록을 중앙에 맞추고 상/하층을 판별하는 기준.
    int16_t storageXSetpoint = 157;
    int16_t storageXTolerance = 6;
    int16_t storageYUpperLowerSplit = 132;

    // Pixy가 작은 반사/쪼개진 조각을 블록으로 세지 않도록 하는 최소 면적.
    // MissionRouteTuner의 pixy scan/watch/sweep으로 현장 확인 후 조정한다.
    // 임시로 필터를 끄고 원래처럼 보고 싶으면 0으로 둔다.
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

  struct PoseTimingConfig
  {
    // Manipulator profile times. Smaller values are faster but increase shake and placement error.
    uint16_t startupInitialMs = 1200;
    uint16_t missionInstructionMs = 800;
    uint16_t storageMs = 700;
    uint16_t preGripMs = 750;
    uint16_t gripMs = 850;
    uint16_t upperGripStageMs = 280;
    uint16_t upperLiftStageMs = 320;
    uint16_t storageWithBlockMs = 800;
    uint16_t missionZoneTurnMs = 650;
    uint16_t missionZonePlaceMs = 800;
    uint16_t finishStorageMs = 650;
    uint16_t finishInitialMs = 800;
  };

  struct WaitConfig
  {
    uint16_t buttonDebounceMs = 80;
    uint16_t driveSettleMs = 50;
    uint16_t blockFeedbackMs = 80;
    uint16_t scanSampleMs = 20;
    uint16_t cameraLampMs = 150;
    uint16_t gripperActionMs = 250;
    uint16_t poseSettleMs = 100;
  };

  struct FinishReturnConfig
  {
    int16_t boundaryAdc = 220; // SL PSD: 이 값 이상이면 장애물 있음
    int16_t trackSl = 317;     // 후진 중 왼쪽 장애물을 따라갈 때 목표 SL 값
    int16_t trackTolerance = 20;
    int32_t correctionMaxSpeed = 40;
    int32_t openSideLeftBiasSpeed = 25; // SL이 장애물을 놓쳤을 때 왼쪽으로 살짝 붙는 보정
    int32_t wheelMaxSpeed = 200;        // 후진 보정 중 특정 바퀴만 과하게 도는 것을 제한
    unsigned long finishExtraMs = 3000; // 현장 테스트로 맞춘 값. 기록 없이 줄이지 않는다.
  };

  PoseConfig pose;
  MissionOrderConfig mission;
  PsdConfig psd;
  FrontApproachConfig front;
  SpeedConfig speed;
  StorageRackLayoutConfig storageRack;
  StoragePickupRegionConfig storagePickupRegion;
  StorageGripTargetConfig storageGripTarget;
  StorageDriveConfig storageDrive;
  CameraScanConfig cameraScan;
  TimeoutConfig timeout;
  PoseTimingConfig poseTiming;
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
