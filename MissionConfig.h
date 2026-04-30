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
  static const uint8_t MAX_MISSION_BLOCKS = 6;

  struct PoseConfig
  {
    uint8_t missionZoneStartId = 6; // 7번 = 1번 칸, 14번 = 8번 칸
  };

  struct MissionOrderConfig
  {
    uint8_t blockSignatureMap = 0b00111111; // sig1=bit0, sig2=bit1, ...
    uint8_t blockCount = 3;                 // 대회 당일 발표 후 수정
    uint8_t goalPositions[MAX_MISSION_BLOCKS] = {
        1, 2, 3, 4, 5, 6 // 대회 당일 발표된 칸 번호 순서
    };
  };

  struct PsdConfig
  {
    // 2단계: 기존 FL/FR 목표값. 현재는 front.firstDetectAdc 기반 접근을 사용한다.
    int16_t approachFl = 460;
    int16_t approachFr = 500;
    int16_t approachTolerance = 5;

    // 2단계: 미션지시존 위치 정렬
    int16_t missionSl = 518;
    int16_t missionTolerance = 5;

    // 4단계: 적재함 앞 2축 정렬
    int16_t alignSl = 317;
    int16_t alignFr = 314;
    int16_t alignTolerance = 5;

    // 5단계: 미션수행존 / 적재함 스캔
    int16_t missionZoneSl = 595;
    int16_t missionZoneTolerance = 5;
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
    int32_t cameraFineTuneSpeed = 200;
    int32_t storageScanSpeed = 200;
    int32_t returnSpeed = 200;

    // Extended-position / time-based profile helpers.
    int32_t positionMoveMmPerSec = 150;
  };

  struct StorageRackLayoutConfig
  {
    // 고정 번호 규칙: 윗줄 1 2 3 4, 아랫줄 5 6 7 8.
    // 현재 코드는 번호로 슬롯에 직접 진입하지 않고 Pixy2로 목표 signature를 찾아 집는다.
    uint8_t upperRowSlots[4] = {1, 2, 3, 4};
    uint8_t lowerRowSlots[4] = {5, 6, 7, 8};

    uint16_t nominalCellPitchMm = 60;
    uint16_t nominalOuterLengthMm = 270;
    uint16_t nominalOuterWidthMm = 130;
    uint16_t nominalOuterHeightMm = 82;
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

    // 스토리지에서 목표 블록을 중앙에 맞추고 상/하층을 판별하는 기준.
    int16_t storageXSetpoint = 157;
    int16_t storageXTolerance = 6;
    int16_t storageYUpperLowerSplit = 103;
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
  StorageDriveConfig storageDrive;
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
