/*
 * MissionRouteTuner
 *
 * 목적:
 *   실제 Motor.ino 미션 코드를 직접 바꾸지 않고, EEPROM 자세/짧은 주행/
 *   집기/배치 시퀀스를 현장에서 빠르게 반복 테스트하는 안전 튜닝 콘솔.
 *
 * 시리얼 모니터:
 *   USB 115200 baud primary / Bluetooth Serial2 9600 baud input-only fallback / Newline
 *
 * 핵심 안전 기준:
 *   - 시작하자마자 자동 동작하지 않음
 *   - 모든 위험 동작은 명령을 명시해야 실행
 *   - 현재 실제 테스트에 필요한 pose 1,3,4,5,7~14 누락 시 실행 중단
 *   - 주행 거리, 주행 속도, 예상 주행 시간을 profile로 제한
 *   - 마지막 동작은 replay로 반복 가능하지만, stop은 항상 정지 우선
 */

#include "Debug.h"
#include "Motor.h"
#include "Mobilebase.h"
#include "Manipulator.h"
#include "Pixy.h"
#include "Gripper.h"
#include "RGBLED.h"
#include "MissionConfig.h"
#include <EEPROM.h>

Pixy2SPI_SS pixy;
const MissionConfig &CFG = MissionConfig::get();

enum MissionPoseId {
  POSE_INITIAL = 1,
  POSE_STORAGE = 3,
  POSE_GRIP_UPPER = 4,
  POSE_GRIP_LOWER = 5,
  POSE_MISSION_ZONE_START = 6 // 7번=1번 칸, 14번=8번 칸
};

struct TunerProfile {
  const char *name;
  uint16_t minPoseMs;
  uint16_t maxPoseMs;
  float msPerRawTick;
  uint16_t maxDriveMm;
  uint16_t maxDriveMmPerSec;
  uint16_t maxDriveTimeMs;
  float missionVelocityScale;
  float missionPoseMsScale;
  uint16_t maxMissionVelocityRaw;
};

const TunerProfile PROFILES[] = {
  {"safe",   250, 500, 0.60,  300, 120, 5000, 0.75, 1.20, 180},
  {"normal", 200, 500, 0.25,  700, 200, 5000, 1.00, 1.00, 260},
  {"fast",   150, 500, 0.15, 1000, 260, 5000, 1.35, 0.80, 360},
  {"max",    100, 500, 0.10, 1200, 400, 5000, 1.65, 0.65, 480}
};
const uint8_t PROFILE_COUNT = sizeof(PROFILES) / sizeof(PROFILES[0]);
uint8_t profileIndex = 1;
const uint16_t MISSION_ACTUATOR_MS = 200;
const uint16_t MISSION_STAGED_POSE_MIN_MS = 100;
const uint16_t MANIPULATOR_SETTLE_MS = 50;
const uint16_t STATUS_LED_FEEDBACK_MS = 120;
const uint16_t DEFAULT_INSTRUCTION_SCAN_MS = 500;
const uint16_t DEFAULT_INSTRUCTION_SCAN_SAMPLE_MS = 10;
const uint16_t DEFAULT_INSTRUCTION_FINAL_FORWARD_MS = 500;
const int32_t DEFAULT_INSTRUCTION_FINAL_FORWARD_RAW = 120;

#define RECEIVE_BUFFER_SIZE 120
char receiveBuffer[RECEIVE_BUFFER_SIZE] = {0,};
uint8_t receiveBufferIdx = 0;

bool manipulatorReady = false;
bool mobileReady = false;
bool pixyReady = false;

ManipulatorPose lastTunedPose;
bool hasLastTunedPose = false;
ManipulatorPose pendingPoseWrite;
bool hasPendingPoseWrite = false;
String lastReplayableCommand = "";

enum MissionStepperStage {
  MISSION_IDLE = 0,
  MISSION_START_TO_INSTRUCTION,
  MISSION_INSTRUCTION_SCAN_HOLD,
  MISSION_GO_TO_STORAGE,
  MISSION_COLUMN_MOVE_OR_SCAN,
  MISSION_PICK_HOLD,
  MISSION_PLACE_HOLD,
  MISSION_REALIGN_OR_NEXT,
  MISSION_FINISH,
  MISSION_FINISHED
};

MissionStepperStage missionStage = MISSION_IDLE;
const __FlashStringHelper *missionStageName(MissionStepperStage stage);
bool commandMissionNext();
bool commandMissionStorageGripAlignForLayer(bool lower);
uint8_t missionBlockIndex = 1;
String missionPickLayer = "";
uint8_t missionPlaceSlot = 0;
bool missionColumnScanHasDecision = false;
uint8_t missionDetectedSignature = 0;
uint8_t missionDetectedSourceSlot = 0;
uint8_t missionDetectedGoalSlot = 0;
uint8_t missionDetectedPickupRegion = 0;
uint8_t missionDetectedPixyColumn = 0;
int16_t missionDetectedX = 0;
int16_t missionDetectedY = 0;
uint32_t missionDetectedArea = 0;
uint8_t missionStorageColumn = 1;
uint8_t missionQueueCount = 0;
uint8_t missionQueueSignatures[MissionConfig::MAX_MISSION_BLOCKS] = {0};
uint8_t missionQueueSourceSlots[MissionConfig::MAX_MISSION_BLOCKS] = {0};
uint8_t missionQueueGoalSlots[MissionConfig::MAX_MISSION_BLOCKS] = {0};
bool missionQueueCompleted[MissionConfig::MAX_MISSION_BLOCKS] = {false};
struct MissionStorageSurveyDetection {
  uint8_t signature;
  uint8_t sourceSlot;
  uint8_t column;
  uint8_t pickupRegion;
  int16_t x;
  int16_t y;
  uint32_t area;
  int16_t psdFl;
  int16_t psdFr;
  int16_t psdSl;
  int16_t psdSr;
  bool assigned;
};
MissionStorageSurveyDetection missionSurveyDetections[MissionConfig::MAX_MISSION_BLOCKS] = {};
uint8_t missionSurveyDetectionCount = 0;
uint8_t missionSurveyEndColumn = 1;
bool missionSurveyHasResults = false;
float missionColumnStepMm = CFG.storageRack.scanColumnStepMm;
int32_t missionColumnMoveMmPerSec = CFG.storageRack.scanColumnMoveMmPerSec;
uint16_t missionColumnScanSettleMs = CFG.storageRack.scanSettleMs;
uint8_t missionColumnScanFrames = CFG.storageRack.scanFramesPerStop;
uint16_t missionColumnScanSampleMs = CFG.wait.scanSampleMs;
int16_t missionFrontFirstDetectAdc = CFG.front.firstDetectAdc;
int16_t missionFrontDecelWindowAdc = CFG.front.decelWindowAdc;
float missionFrontAfterDetectMm = 15.0;
int32_t missionFrontAfterDetectMmPerSec = 80;
int16_t missionInstructionSl = CFG.psd.missionSl;
uint16_t missionInstructionFinalForwardMs = DEFAULT_INSTRUCTION_FINAL_FORWARD_MS;
int32_t missionInstructionFinalForwardSpeed = DEFAULT_INSTRUCTION_FINAL_FORWARD_RAW;
uint16_t missionInstructionScanMs = DEFAULT_INSTRUCTION_SCAN_MS;
uint16_t missionInstructionScanSampleMs = DEFAULT_INSTRUCTION_SCAN_SAMPLE_MS;
float missionStorageFirstForwardMm = CFG.storageDrive.firstForwardMm;
float missionStorageExtraForwardMm = CFG.storageDrive.extraForwardMm;
float missionStorageRightMm = CFG.storageDrive.rightMm;
int16_t missionStorageApproachFrDetectAdc = CFG.psd.storageApproachFrDetectAdc;
int16_t missionStorageApproachFrLeadDeltaAdc = CFG.psd.storageApproachFrLeadDeltaAdc;
uint8_t missionStorageApproachFrLeadConfirmSamples = CFG.psd.storageApproachFrLeadConfirmSamples;
int16_t missionStorageApproachSlGateTolerance = CFG.psd.storageApproachSlGateTolerance;
int16_t missionStorageApproachSlLeaveAdc = CFG.psd.storageApproachSlLeaveAdc;
int16_t missionStorageApproachSlReenterAdc = CFG.psd.storageApproachSlReenterAdc;
uint8_t missionStorageApproachSlReenterConfirmSamples = CFG.psd.storageApproachSlReenterConfirmSamples;
uint16_t missionStorageApproachIgnoreReentryMs = CFG.psd.storageApproachIgnoreReentryMs;
int32_t missionStorageApproachRightSpeed = CFG.speed.storageApproachRightSpeed;
int32_t missionStorageApproachForwardSpeed = CFG.speed.storageApproachForwardSpeed;
float missionWheelVelocityTrimFl = 1.0;
float missionWheelVelocityTrimFr = 1.0;
float missionWheelVelocityTrimBl = 1.0;
float missionWheelVelocityTrimBr = 1.0;
int16_t missionAlignFl = CFG.psd.alignFl;
int16_t missionAlignFr = CFG.psd.alignFr;
int16_t missionAlignSl = CFG.psd.alignSl;
int16_t missionAlignTolerance = CFG.psd.alignTolerance;
int16_t missionGripAlignFl = CFG.psd.gripAlignFl;
int16_t missionGripAlignFr = CFG.psd.gripAlignFr;
int16_t missionGripAlignSl = CFG.psd.gripAlignSl;
int16_t missionGripAlignTolerance = CFG.psd.gripAlignTolerance;
int16_t missionLowerGripAlignFl = CFG.psd.lowerGripAlignFl;
int16_t missionLowerGripAlignFr = CFG.psd.lowerGripAlignFr;
int16_t missionLowerGripAlignSl = CFG.psd.lowerGripAlignSl;
int16_t missionLowerGripAlignTolerance = CFG.psd.lowerGripAlignTolerance;
int16_t missionPlaceSl = CFG.psd.missionZoneSl;
int16_t missionPlaceFr = CFG.psd.missionZoneFr;
int16_t missionPlaceTolerance = CFG.psd.missionZoneTolerance;
int16_t missionFinishPreAlignSl = CFG.finishReturn.preAlignSl;
int16_t missionFinishPreAlignTolerance = CFG.finishReturn.preAlignTolerance;
int32_t missionFinishPreAlignSpeed = CFG.finishReturn.preAlignSpeed;
float missionUpperGripDepthMm = CFG.storageGripTarget.upperExtraForwardMm;
float missionLowerGripDepthMm = CFG.storageGripTarget.lowerExtraForwardMm;
int32_t missionGripDepthMmPerSec = CFG.storageGripTarget.extraForwardMmPerSec;
uint8_t missionStorageScanTargetColumn = 1;
uint8_t missionColumnSearchMissColumn = 0;
bool missionButtonMode = false;
bool missionButtonWasPressed = false;
unsigned long missionButtonLastChangeMs = 0;
bool missionUndoAvailable = false;
float missionUndoDistanceMm = 0.0;
uint8_t missionUndoDirection = 0;
int32_t missionUndoSpeedMmPerSec = 0;

struct PsdSnapshot {
  int16_t fl;
  int16_t fr;
  int16_t sl;
  int16_t sr;
};

bool psdWatchEnabled = false;
uint16_t psdWatchIntervalMs = 250;
unsigned long psdWatchLastPrintMs = 0;

static const uint32_t TUNER_CAL_MAGIC = 0x54554331UL; // "TUC1"
static const uint8_t TUNER_CAL_VERSION = 12;
static const int TUNER_CAL_EEPROM_ADDR =
  MANIPULATOR_POSE_ID_MAX_CNT * MANIPULATOR_POSE_DATA_SIZE;

struct __attribute__((packed)) TunerCalibrationRecord {
  uint32_t magic;
  uint8_t version;
  uint8_t size;
  uint8_t savedProfileIndex;
  uint8_t reserved;
  int16_t frontCruiseSpeed;
  int16_t frontSlowSpeed;
  int16_t psdCorrectionSpeed;
  int16_t positionMoveMmPerSec;
  int16_t columnStepTenths;
  int16_t columnMoveMmPerSec;
  int16_t frontFirstDetectAdc;
  int16_t frontDecelWindowAdc;
  int16_t frontAfterDetectTenths;
  int16_t frontAfterDetectMmPerSec;
  int16_t instructionSl;
  int16_t instructionFinalForwardMs;
  int16_t instructionFinalForwardSpeed;
  uint16_t instructionScanMs;
  uint16_t instructionScanSampleMs;
  int16_t storageFirstForwardTenths;
  int16_t storageExtraForwardTenths;
  int16_t storageRightTenths;
  int16_t alignFl;
  int16_t alignFr;
  int16_t alignSl;
  int16_t alignTolerance;
  int16_t gripAlignFl;
  int16_t gripAlignFr;
  int16_t gripAlignSl;
  int16_t gripAlignTolerance;
  int16_t lowerGripAlignFl;
  int16_t lowerGripAlignFr;
  int16_t lowerGripAlignSl;
  int16_t lowerGripAlignTolerance;
  int16_t placeSl;
  int16_t placeFr;
  int16_t placeTolerance;
  int16_t upperGripDepthTenths;
  int16_t lowerGripDepthTenths;
  int16_t gripDepthMmPerSec;
  uint16_t checksum;
};

struct __attribute__((packed)) TunerCalibrationRecordV11 {
  uint32_t magic;
  uint8_t version;
  uint8_t size;
  uint8_t savedProfileIndex;
  uint8_t reserved;
  int16_t frontCruiseSpeed;
  int16_t frontSlowSpeed;
  int16_t psdCorrectionSpeed;
  int16_t positionMoveMmPerSec;
  int16_t columnStepTenths;
  int16_t columnMoveMmPerSec;
  int16_t frontFirstDetectAdc;
  int16_t frontDecelWindowAdc;
  int16_t frontAfterDetectTenths;
  int16_t frontAfterDetectMmPerSec;
  int16_t instructionSl;
  int16_t instructionFinalForwardMs;
  int16_t instructionFinalForwardSpeed;
  uint16_t instructionScanMs;
  uint16_t instructionScanSampleMs;
  int16_t storageFirstForwardTenths;
  int16_t storageExtraForwardTenths;
  int16_t storageRightTenths;
  int16_t alignFl;
  int16_t alignFr;
  int16_t alignSl;
  int16_t alignTolerance;
  int16_t gripAlignFl;
  int16_t gripAlignFr;
  int16_t gripAlignSl;
  int16_t gripAlignTolerance;
  int16_t lowerGripAlignFl;
  int16_t lowerGripAlignFr;
  int16_t lowerGripAlignSl;
  int16_t lowerGripAlignTolerance;
  int16_t placeSl;
  int16_t placeFr;
  int16_t placeTolerance;
  uint16_t checksum;
};

struct MissionRuntimeMotion {
  int32_t frontCruiseSpeed;
  int32_t frontSlowSpeed;
  int32_t psdCorrectionSpeed;
  int32_t frontDepthCorrectionSpeed;
  int32_t missionZonePlaceCorrectionSpeed;
  int32_t positionMoveMmPerSec;
  uint16_t initialPoseMs;
  uint16_t storagePoseMs;
  uint16_t preGripPoseMs;
  uint16_t gripPoseMs;
  uint16_t placePoseMs;
  uint16_t returnPoseMs;
  uint16_t gripHoldMs;
  uint16_t placeHoldMs;
};

MissionRuntimeMotion missionMotion = {
  250, 150, 120, 80, 80, 150,
  MISSION_ACTUATOR_MS, MISSION_ACTUATOR_MS, MISSION_ACTUATOR_MS,
  MISSION_ACTUATOR_MS, MISSION_ACTUATOR_MS, MISSION_ACTUATOR_MS,
  MISSION_ACTUATOR_MS, MISSION_ACTUATOR_MS
};

const TunerProfile &profile() {
  return PROFILES[profileIndex];
}

int32_t scaledMissionSpeed(int32_t baseValue, float scale, int32_t maxValue) {
  int32_t scaled = (int32_t)round((float)baseValue * scale);
  if (scaled < 1) scaled = 1;
  if (scaled > maxValue) scaled = maxValue;
  return scaled;
}

int32_t clampInt32(int32_t value, int32_t minValue, int32_t maxValue) {
  if (value < minValue) return minValue;
  if (value > maxValue) return maxValue;
  return value;
}

uint16_t scaledMissionMs(uint16_t baseMs) {
  long scaled = (long)round((float)baseMs * profile().missionPoseMsScale);
  return (uint16_t)constrain(scaled, 100L, 500L);
}

void applyMissionSpeedPresetForProfile() {
  int32_t velocityMax = profile().maxMissionVelocityRaw;
  missionMotion.frontCruiseSpeed = scaledMissionSpeed(CFG.front.cruiseSpeed,
                                                       profile().missionVelocityScale,
                                                       velocityMax);
  missionMotion.frontSlowSpeed = scaledMissionSpeed(CFG.front.slowSpeed,
                                                     profile().missionVelocityScale,
                                                     velocityMax);
  if (missionMotion.frontSlowSpeed >= missionMotion.frontCruiseSpeed) {
    missionMotion.frontSlowSpeed = max((int32_t)1, missionMotion.frontCruiseSpeed * 2 / 3);
  }
  missionMotion.psdCorrectionSpeed = scaledMissionSpeed(CFG.speed.psdCorrectionSpeed,
                                                         profile().missionVelocityScale,
                                                         velocityMax);
  missionMotion.frontDepthCorrectionSpeed = scaledMissionSpeed(CFG.speed.frontDepthCorrectionSpeed,
                                                                profile().missionVelocityScale,
                                                                velocityMax);
  missionMotion.missionZonePlaceCorrectionSpeed = scaledMissionSpeed(CFG.speed.missionZonePlaceCorrectionSpeed,
                                                                      profile().missionVelocityScale,
                                                                      velocityMax);
  missionMotion.positionMoveMmPerSec = scaledMissionSpeed(CFG.speed.positionMoveMmPerSec,
                                                           profile().missionVelocityScale,
                                                           profile().maxDriveMmPerSec);
}

int16_t encodeTenths(float value) {
  long scaled = (long)round(value * 10.0);
  if (scaled < -32768L) scaled = -32768L;
  if (scaled > 32767L) scaled = 32767L;
  return (int16_t)scaled;
}

float decodeTenths(int16_t value) {
  return (float)value / 10.0;
}

uint16_t maxProfileDriveMm() {
  uint16_t maxValue = 0;
  for (uint8_t i = 0; i < PROFILE_COUNT; i++) {
    if (PROFILES[i].maxDriveMm > maxValue) maxValue = PROFILES[i].maxDriveMm;
  }
  return maxValue;
}

uint16_t maxProfileDriveMmPerSec() {
  uint16_t maxValue = 0;
  for (uint8_t i = 0; i < PROFILE_COUNT; i++) {
    if (PROFILES[i].maxDriveMmPerSec > maxValue) maxValue = PROFILES[i].maxDriveMmPerSec;
  }
  return maxValue;
}

uint16_t maxProfileVelocityRaw() {
  uint16_t maxValue = 0;
  for (uint8_t i = 0; i < PROFILE_COUNT; i++) {
    if (PROFILES[i].maxMissionVelocityRaw > maxValue) maxValue = PROFILES[i].maxMissionVelocityRaw;
  }
  return maxValue;
}

void resetTunerCalibrationToDefaults() {
  applyMissionSpeedPresetForProfile();
  missionColumnStepMm = CFG.storageRack.scanColumnStepMm;
  missionColumnMoveMmPerSec = CFG.storageRack.scanColumnMoveMmPerSec;
  missionColumnScanSettleMs = CFG.storageRack.scanSettleMs;
  missionColumnScanFrames = CFG.storageRack.scanFramesPerStop;
  missionColumnScanSampleMs = CFG.wait.scanSampleMs;
  missionFrontFirstDetectAdc = CFG.front.firstDetectAdc;
  missionFrontDecelWindowAdc = CFG.front.decelWindowAdc;
  missionFrontAfterDetectMm = 15.0;
  missionFrontAfterDetectMmPerSec = 80;
  missionInstructionSl = CFG.psd.missionSl;
  missionInstructionFinalForwardMs = DEFAULT_INSTRUCTION_FINAL_FORWARD_MS;
  missionInstructionFinalForwardSpeed = DEFAULT_INSTRUCTION_FINAL_FORWARD_RAW;
  missionInstructionScanMs = DEFAULT_INSTRUCTION_SCAN_MS;
  missionInstructionScanSampleMs = DEFAULT_INSTRUCTION_SCAN_SAMPLE_MS;
  missionStorageFirstForwardMm = CFG.storageDrive.firstForwardMm;
  missionStorageExtraForwardMm = CFG.storageDrive.extraForwardMm;
  missionStorageRightMm = CFG.storageDrive.rightMm;
  missionStorageApproachFrDetectAdc = CFG.psd.storageApproachFrDetectAdc;
  missionStorageApproachFrLeadDeltaAdc = CFG.psd.storageApproachFrLeadDeltaAdc;
  missionStorageApproachFrLeadConfirmSamples = CFG.psd.storageApproachFrLeadConfirmSamples;
  missionStorageApproachSlGateTolerance = CFG.psd.storageApproachSlGateTolerance;
  missionStorageApproachSlLeaveAdc = CFG.psd.storageApproachSlLeaveAdc;
  missionStorageApproachSlReenterAdc = CFG.psd.storageApproachSlReenterAdc;
  missionStorageApproachSlReenterConfirmSamples = CFG.psd.storageApproachSlReenterConfirmSamples;
  missionStorageApproachIgnoreReentryMs = CFG.psd.storageApproachIgnoreReentryMs;
  missionStorageApproachRightSpeed = CFG.speed.storageApproachRightSpeed;
  missionStorageApproachForwardSpeed = CFG.speed.storageApproachForwardSpeed;
  missionAlignFl = CFG.psd.alignFl;
  missionAlignFr = CFG.psd.alignFr;
  missionAlignSl = CFG.psd.alignSl;
  missionAlignTolerance = CFG.psd.alignTolerance;
  missionGripAlignFl = CFG.psd.gripAlignFl;
  missionGripAlignFr = CFG.psd.gripAlignFr;
  missionGripAlignSl = CFG.psd.gripAlignSl;
  missionGripAlignTolerance = CFG.psd.gripAlignTolerance;
  missionLowerGripAlignFl = CFG.psd.lowerGripAlignFl;
  missionLowerGripAlignFr = CFG.psd.lowerGripAlignFr;
  missionLowerGripAlignSl = CFG.psd.lowerGripAlignSl;
  missionLowerGripAlignTolerance = CFG.psd.lowerGripAlignTolerance;
  missionPlaceSl = CFG.psd.missionZoneSl;
  missionPlaceFr = CFG.psd.missionZoneFr;
  missionPlaceTolerance = CFG.psd.missionZoneTolerance;
  missionFinishPreAlignSl = CFG.finishReturn.preAlignSl;
  missionFinishPreAlignTolerance = CFG.finishReturn.preAlignTolerance;
  missionFinishPreAlignSpeed = CFG.finishReturn.preAlignSpeed;
  missionUpperGripDepthMm = CFG.storageGripTarget.upperExtraForwardMm;
  missionLowerGripDepthMm = CFG.storageGripTarget.lowerExtraForwardMm;
  missionGripDepthMmPerSec = CFG.storageGripTarget.extraForwardMmPerSec;
}

bool tunerCalibrationFitsEEPROM() {
  return TUNER_CAL_EEPROM_ADDR >= 0 &&
         TUNER_CAL_EEPROM_ADDR + (int)sizeof(TunerCalibrationRecord) <= EEPROM.length();
}

uint16_t tunerCalibrationChecksumBytes(const uint8_t *bytes, uint8_t byteCount) {
  uint16_t checksum = 0x51A7;
  for (uint8_t i = 0; i < byteCount; i++) {
    checksum = (uint16_t)((checksum << 5) - checksum + bytes[i]);
  }
  return checksum;
}

uint16_t tunerCalibrationChecksum(const TunerCalibrationRecord &record) {
  return tunerCalibrationChecksumBytes(
    (const uint8_t *)&record,
    sizeof(TunerCalibrationRecord) - sizeof(record.checksum));
}

uint16_t tunerCalibrationChecksumV11(const TunerCalibrationRecordV11 &record) {
  return tunerCalibrationChecksumBytes(
    (const uint8_t *)&record,
    sizeof(TunerCalibrationRecordV11) - sizeof(record.checksum));
}

void fillTunerCalibrationRecord(TunerCalibrationRecord *record) {
  memset(record, 0, sizeof(TunerCalibrationRecord));
  record->magic = TUNER_CAL_MAGIC;
  record->version = TUNER_CAL_VERSION;
  record->size = sizeof(TunerCalibrationRecord);
  record->savedProfileIndex = profileIndex;
  record->frontCruiseSpeed = (int16_t)missionMotion.frontCruiseSpeed;
  record->frontSlowSpeed = (int16_t)missionMotion.frontSlowSpeed;
  record->psdCorrectionSpeed = (int16_t)missionMotion.psdCorrectionSpeed;
  record->positionMoveMmPerSec = (int16_t)missionMotion.positionMoveMmPerSec;
  record->columnStepTenths = encodeTenths(missionColumnStepMm);
  record->columnMoveMmPerSec = (int16_t)missionColumnMoveMmPerSec;
  record->frontFirstDetectAdc = missionFrontFirstDetectAdc;
  record->frontDecelWindowAdc = missionFrontDecelWindowAdc;
  record->frontAfterDetectTenths = encodeTenths(missionFrontAfterDetectMm);
  record->frontAfterDetectMmPerSec = (int16_t)missionFrontAfterDetectMmPerSec;
  record->instructionSl = missionInstructionSl;
  record->instructionFinalForwardMs = (int16_t)missionInstructionFinalForwardMs;
  record->instructionFinalForwardSpeed = (int16_t)missionInstructionFinalForwardSpeed;
  record->instructionScanMs = missionInstructionScanMs;
  record->instructionScanSampleMs = missionInstructionScanSampleMs;
  record->storageFirstForwardTenths = encodeTenths(missionStorageFirstForwardMm);
  record->storageExtraForwardTenths = encodeTenths(missionStorageExtraForwardMm);
  record->storageRightTenths = encodeTenths(missionStorageRightMm);
  record->alignFl = missionAlignFl;
  record->alignFr = missionAlignFr;
  record->alignSl = missionAlignSl;
  record->alignTolerance = missionAlignTolerance;
  record->gripAlignFl = missionGripAlignFl;
  record->gripAlignFr = missionGripAlignFr;
  record->gripAlignSl = missionGripAlignSl;
  record->gripAlignTolerance = missionGripAlignTolerance;
  record->lowerGripAlignFl = missionLowerGripAlignFl;
  record->lowerGripAlignFr = missionLowerGripAlignFr;
  record->lowerGripAlignSl = missionLowerGripAlignSl;
  record->lowerGripAlignTolerance = missionLowerGripAlignTolerance;
  record->placeSl = missionPlaceSl;
  record->placeFr = missionPlaceFr;
  record->placeTolerance = missionPlaceTolerance;
  record->upperGripDepthTenths = encodeTenths(missionUpperGripDepthMm);
  record->lowerGripDepthTenths = encodeTenths(missionLowerGripDepthMm);
  record->gripDepthMmPerSec = (int16_t)missionGripDepthMmPerSec;
  record->checksum = tunerCalibrationChecksum(*record);
}

bool tunerCalibrationRecordLooksSafe(const TunerCalibrationRecord &record) {
  if (record.magic != TUNER_CAL_MAGIC ||
      record.version != TUNER_CAL_VERSION ||
      record.size != sizeof(TunerCalibrationRecord) ||
      record.checksum != tunerCalibrationChecksum(record)) {
    return false;
  }

  uint16_t maxDriveMm = maxProfileDriveMm();
  uint16_t maxDriveSpeed = maxProfileDriveMmPerSec();
  uint16_t maxVelocity = maxProfileVelocityRaw();
  float columnStepMm = decodeTenths(record.columnStepTenths);
  float frontAfterDetectMm = decodeTenths(record.frontAfterDetectTenths);
  float storageFirstMm = decodeTenths(record.storageFirstForwardTenths);
  float storageExtraMm = decodeTenths(record.storageExtraForwardTenths);
  float storageRightMm = decodeTenths(record.storageRightTenths);
  float upperGripDepthMm = decodeTenths(record.upperGripDepthTenths);
  float lowerGripDepthMm = decodeTenths(record.lowerGripDepthTenths);

  if (record.frontCruiseSpeed <= 0 || record.frontCruiseSpeed > (int16_t)maxVelocity ||
      record.frontSlowSpeed <= 0 || record.frontSlowSpeed >= record.frontCruiseSpeed ||
      record.psdCorrectionSpeed <= 0 || record.psdCorrectionSpeed > (int16_t)maxVelocity ||
      record.positionMoveMmPerSec <= 0 || record.positionMoveMmPerSec > (int16_t)maxDriveSpeed ||
      record.columnMoveMmPerSec <= 0 || record.columnMoveMmPerSec > (int16_t)maxDriveSpeed ||
      record.gripDepthMmPerSec <= 0 || record.gripDepthMmPerSec > (int16_t)maxDriveSpeed ||
      record.frontAfterDetectMmPerSec <= 0 || record.frontAfterDetectMmPerSec > (int16_t)maxVelocity ||
      record.instructionFinalForwardSpeed <= 0 || record.instructionFinalForwardSpeed > (int16_t)maxVelocity) {
    return false;
  }
  if (columnStepMm <= 0.0 || columnStepMm > (float)maxDriveMm ||
      frontAfterDetectMm < 0.0 || frontAfterDetectMm > (float)maxDriveMm ||
      storageFirstMm <= 0.0 || storageFirstMm > (float)maxDriveMm ||
      storageExtraMm <= 0.0 || storageExtraMm > (float)maxDriveMm ||
      storageRightMm <= 0.0 || storageRightMm > (float)maxDriveMm ||
      upperGripDepthMm < 0.0 || upperGripDepthMm > 80.0 ||
      lowerGripDepthMm < 0.0 || lowerGripDepthMm > 80.0) {
    return false;
  }
  if (record.frontFirstDetectAdc < 1 || record.frontFirstDetectAdc > 1023 ||
      record.frontDecelWindowAdc < 0 || record.frontDecelWindowAdc > 400 ||
      record.instructionSl < 1 || record.instructionSl > 1023 ||
      record.alignFl < 1 || record.alignFl > 1023 ||
      record.alignFr < 1 || record.alignFr > 1023 ||
      record.alignSl < 1 || record.alignSl > 1023 ||
      record.alignTolerance < 1 || record.alignTolerance > 80 ||
      record.gripAlignFl < 1 || record.gripAlignFl > 1023 ||
      record.gripAlignFr < 1 || record.gripAlignFr > 1023 ||
      record.gripAlignSl < 1 || record.gripAlignSl > 1023 ||
      record.gripAlignTolerance < 1 || record.gripAlignTolerance > 80 ||
      record.lowerGripAlignFl < 1 || record.lowerGripAlignFl > 1023 ||
      record.lowerGripAlignFr < 1 || record.lowerGripAlignFr > 1023 ||
      record.lowerGripAlignSl < 1 || record.lowerGripAlignSl > 1023 ||
      record.lowerGripAlignTolerance < 1 || record.lowerGripAlignTolerance > 80 ||
      record.placeSl < 1 || record.placeSl > 1023 ||
      record.placeFr < 1 || record.placeFr > 1023 ||
      record.placeTolerance < 1 || record.placeTolerance > 80 ||
      record.instructionScanMs < 300 || record.instructionScanMs > 8000 ||
      record.instructionScanSampleMs < 10 || record.instructionScanSampleMs > 200 ||
      record.instructionFinalForwardMs < 0 || record.instructionFinalForwardMs > 5000) {
    return false;
  }
  return true;
}

void applyTunerCalibrationRecord(const TunerCalibrationRecord &record) {
  missionMotion.frontCruiseSpeed = record.frontCruiseSpeed;
  missionMotion.frontSlowSpeed = record.frontSlowSpeed;
  missionMotion.psdCorrectionSpeed = record.psdCorrectionSpeed;
  missionMotion.positionMoveMmPerSec = record.positionMoveMmPerSec;
  missionColumnStepMm = decodeTenths(record.columnStepTenths);
  missionColumnMoveMmPerSec = record.columnMoveMmPerSec;
  missionFrontFirstDetectAdc = record.frontFirstDetectAdc;
  missionFrontDecelWindowAdc = record.frontDecelWindowAdc;
  missionFrontAfterDetectMm = decodeTenths(record.frontAfterDetectTenths);
  missionFrontAfterDetectMmPerSec = record.frontAfterDetectMmPerSec;
  missionInstructionSl = record.instructionSl;
  missionInstructionFinalForwardMs = (uint16_t)record.instructionFinalForwardMs;
  missionInstructionFinalForwardSpeed = record.instructionFinalForwardSpeed;
  missionInstructionScanMs = record.instructionScanMs;
  missionInstructionScanSampleMs = record.instructionScanSampleMs;
  missionStorageFirstForwardMm = decodeTenths(record.storageFirstForwardTenths);
  missionStorageExtraForwardMm = decodeTenths(record.storageExtraForwardTenths);
  missionStorageRightMm = decodeTenths(record.storageRightTenths);
  missionAlignFl = record.alignFl;
  missionAlignFr = record.alignFr;
  missionAlignSl = record.alignSl;
  missionAlignTolerance = record.alignTolerance;
  missionGripAlignFl = record.gripAlignFl;
  missionGripAlignFr = record.gripAlignFr;
  missionGripAlignSl = record.gripAlignSl;
  missionGripAlignTolerance = record.gripAlignTolerance;
  missionLowerGripAlignFl = record.lowerGripAlignFl;
  missionLowerGripAlignFr = record.lowerGripAlignFr;
  missionLowerGripAlignSl = record.lowerGripAlignSl;
  missionLowerGripAlignTolerance = record.lowerGripAlignTolerance;
  missionPlaceSl = record.placeSl;
  missionPlaceFr = record.placeFr;
  missionPlaceTolerance = record.placeTolerance;
  missionUpperGripDepthMm = decodeTenths(record.upperGripDepthTenths);
  missionLowerGripDepthMm = decodeTenths(record.lowerGripDepthTenths);
  missionGripDepthMmPerSec = record.gripDepthMmPerSec;
}

bool upgradeTunerCalibrationRecordV11(const TunerCalibrationRecordV11 &legacy,
                                      TunerCalibrationRecord *record) {
  if (legacy.magic != TUNER_CAL_MAGIC ||
      legacy.version != 11 ||
      legacy.size != sizeof(TunerCalibrationRecordV11) ||
      legacy.checksum != tunerCalibrationChecksumV11(legacy)) {
    return false;
  }

  memset(record, 0, sizeof(TunerCalibrationRecord));
  record->magic = TUNER_CAL_MAGIC;
  record->version = TUNER_CAL_VERSION;
  record->size = sizeof(TunerCalibrationRecord);
  record->savedProfileIndex = legacy.savedProfileIndex;
  record->frontCruiseSpeed = legacy.frontCruiseSpeed;
  record->frontSlowSpeed = legacy.frontSlowSpeed;
  record->psdCorrectionSpeed = legacy.psdCorrectionSpeed;
  record->positionMoveMmPerSec = legacy.positionMoveMmPerSec;
  record->columnStepTenths = legacy.columnStepTenths;
  record->columnMoveMmPerSec = legacy.columnMoveMmPerSec;
  record->frontFirstDetectAdc = legacy.frontFirstDetectAdc;
  record->frontDecelWindowAdc = legacy.frontDecelWindowAdc;
  record->frontAfterDetectTenths = legacy.frontAfterDetectTenths;
  record->frontAfterDetectMmPerSec = legacy.frontAfterDetectMmPerSec;
  record->instructionSl = legacy.instructionSl;
  record->instructionFinalForwardMs = legacy.instructionFinalForwardMs;
  record->instructionFinalForwardSpeed = legacy.instructionFinalForwardSpeed;
  record->instructionScanMs = legacy.instructionScanMs;
  record->instructionScanSampleMs = legacy.instructionScanSampleMs;
  record->storageFirstForwardTenths = legacy.storageFirstForwardTenths;
  record->storageExtraForwardTenths = legacy.storageExtraForwardTenths;
  record->storageRightTenths = legacy.storageRightTenths;
  record->alignFl = legacy.alignFl;
  record->alignFr = legacy.alignFr;
  record->alignSl = legacy.alignSl;
  record->alignTolerance = legacy.alignTolerance;
  record->gripAlignFl = legacy.gripAlignFl;
  record->gripAlignFr = legacy.gripAlignFr;
  record->gripAlignSl = legacy.gripAlignSl;
  record->gripAlignTolerance = legacy.gripAlignTolerance;
  record->lowerGripAlignFl = legacy.lowerGripAlignFl;
  record->lowerGripAlignFr = legacy.lowerGripAlignFr;
  record->lowerGripAlignSl = legacy.lowerGripAlignSl;
  record->lowerGripAlignTolerance = legacy.lowerGripAlignTolerance;
  record->placeSl = legacy.placeSl;
  record->placeFr = legacy.placeFr;
  record->placeTolerance = legacy.placeTolerance;
  record->upperGripDepthTenths = encodeTenths(CFG.storageGripTarget.upperExtraForwardMm);
  record->lowerGripDepthTenths = encodeTenths(CFG.storageGripTarget.lowerExtraForwardMm);
  record->gripDepthMmPerSec = (int16_t)CFG.storageGripTarget.extraForwardMmPerSec;
  record->checksum = tunerCalibrationChecksum(*record);
  return tunerCalibrationRecordLooksSafe(*record);
}

bool readTunerCalibrationRecord(TunerCalibrationRecord *record) {
  if (!tunerCalibrationFitsEEPROM()) return false;
  EEPROM.get(TUNER_CAL_EEPROM_ADDR, *record);
  if (tunerCalibrationRecordLooksSafe(*record)) return true;

  TunerCalibrationRecordV11 legacy;
  EEPROM.get(TUNER_CAL_EEPROM_ADDR, legacy);
  return upgradeTunerCalibrationRecordV11(legacy, record);
}

bool loadTunerCalibrationFromEEPROM(bool verbose) {
  TunerCalibrationRecord record;
  if (!readTunerCalibrationRecord(&record)) {
    if (verbose) {
      DEBUG_SERIAL.println(F("[cal load] 저장된 튜너 보정값이 없거나 검증에 실패했습니다."));
    }
    return false;
  }
  applyTunerCalibrationRecord(record);
  if (verbose) {
    DEBUG_SERIAL.print(F("[cal load] EEPROM "));
    DEBUG_SERIAL.print(TUNER_CAL_EEPROM_ADDR);
    DEBUG_SERIAL.println(F("번 이후의 튜너 보정값을 적용했습니다. pose EEPROM은 읽기만 했습니다."));
  }
  return true;
}

void printLine() {
  DEBUG_SERIAL.println(F("----------------------------------------"));
}

void stopMobilebase() {
  if (mobileReady) {
    SetMobileGoalVelocityForSyncWrite(dxl, 0, 0, 0, 0);
  }
}

void holdManipulatorAtCurrentPose(uint16_t holdMs = 100) {
  if (!manipulatorReady) return;

  int32_t presentMotor1 = dxl.readControlTableItem(PRESENT_POSITION, ARM_DXL_IDS[0]);
  int32_t presentMotor2 = dxl.readControlTableItem(PRESENT_POSITION, ARM_DXL_IDS[1]);
  int32_t presentMotor3 = dxl.readControlTableItem(PRESENT_POSITION, ARM_DXL_IDS[2]);
  int32_t presentMotor4 = dxl.readControlTableItem(PRESENT_POSITION, ARM_DXL_IDS[3]);
  SetManipulatorForwardMoveWithMotorValueForSyncWrite(presentMotor1,
                                                       presentMotor2,
                                                       presentMotor3,
                                                       presentMotor4,
                                                       holdMs);
}

void stopAll(const __FlashStringHelper *reason) {
  stopMobilebase();
  holdManipulatorAtCurrentPose();
  setRGBLEDRed();
  DEBUG_SERIAL.println();
  DEBUG_SERIAL.println(reason);
  DEBUG_SERIAL.println(F("[정지] 모바일베이스 속도 0으로 정지했습니다."));
  DEBUG_SERIAL.println(F("      팔은 현재 위치를 목표로 다시 걸어 멈추고, torque off는 하지 않습니다."));
  delay(250);
  setRGBLEDOff();
}

bool checkEmergencyStopInput() {
  bool emergency = false;
  while (DEBUG_SERIAL.available()) {
    char c = DEBUG_SERIAL.read();
    if (c == '!') {
      emergency = true;
    }
  }
  return emergency;
}

void initPSDInputs() {
  // Arduino analog input pins do not require pinMode setup.
}

void readFrontPSDSensors(int16_t *flValue, int16_t *frValue) {
  *flValue = analogRead(PIN_FRONT_LEFT_PSD);
  *frValue = analogRead(PIN_FRONT_RIGHT_PSD);
}

void readFrontRightPSDSensor(int16_t *frValue) {
  *frValue = analogRead(PIN_FRONT_RIGHT_PSD);
}

void readSideLeftPSDSensor(int16_t *slValue) {
  *slValue = analogRead(PIN_SIDE_LEFT_PSD);
}

void readSideRightPSDSensor(int16_t *srValue) {
  *srValue = analogRead(PIN_SIDE_RIGHT_PSD);
}

void readAllPSDSensors(PsdSnapshot *snapshot) {
  readFrontPSDSensors(&snapshot->fl, &snapshot->fr);
  readSideLeftPSDSensor(&snapshot->sl);
  readSideRightPSDSensor(&snapshot->sr);
}

bool interruptibleDelay(unsigned long waitMs) {
  unsigned long startedAt = millis();
  while (millis() - startedAt < waitMs) {
    if (checkEmergencyStopInput()) {
      stopAll(F("[긴급정지] ! 입력"));
      return false;
    }
    delay(10);
  }
  return true;
}

void turnOnLEDRed500ms() {
  setRGBLEDRed();
  delay(STATUS_LED_FEEDBACK_MS);
  setRGBLEDOff();
}

void turnOnLEDGreen500ms() {
  setRGBLEDGreen();
  delay(STATUS_LED_FEEDBACK_MS);
  setRGBLEDOff();
}

bool parseLongStrict(String text, long *value) {
  text.trim();
  if (text.length() == 0) return false;
  uint8_t start = 0;
  char first = text.charAt(0);
  if (first == '+' || first == '-') {
    start = 1;
    if (text.length() == 1) return false;
  }
  for (uint8_t i = start; i < text.length(); i++) {
    if (!isDigit(text.charAt(i))) return false;
  }
  *value = text.toInt();
  return true;
}

bool parseFloatStrict(String text, float *value) {
  text.trim();
  if (text.length() == 0) return false;

  uint8_t start = 0;
  bool hasDigit = false;
  bool hasDot = false;
  char first = text.charAt(0);
  if (first == '+' || first == '-') {
    start = 1;
    if (text.length() == 1) return false;
  }

  for (uint8_t i = start; i < text.length(); i++) {
    char c = text.charAt(i);
    if (isDigit(c)) {
      hasDigit = true;
    } else if (c == '.' && !hasDot) {
      hasDot = true;
    } else {
      return false;
    }
  }

  if (!hasDigit) return false;
  *value = text.toFloat();
  return true;
}

bool isCommandSpace(char c) {
  return c == ' ' || c == '\t' || c == '\n' || c == '\r';
}

String tokenAt(const String &input, uint8_t index) {
  uint8_t current = 0;
  int start = -1;
  uint16_t length = input.length();

  for (uint16_t i = 0; i <= length; i++) {
    bool atEnd = i == length;
    bool isSpaceChar = !atEnd && isCommandSpace(input.charAt(i));
    if (start < 0 && !atEnd && !isSpaceChar) {
      start = i;
    }
    if (start >= 0 && (atEnd || isSpaceChar)) {
      if (current == index) {
        return input.substring(start, i);
      }
      current++;
      start = -1;
    }
  }

  return String("");
}

String restAfterToken(const String &input, uint8_t tokenCount) {
  uint8_t current = 0;
  bool insideToken = false;
  uint16_t length = input.length();

  for (uint16_t i = 0; i < length; i++) {
    bool isSpaceChar = isCommandSpace(input.charAt(i));
    if (!insideToken && !isSpaceChar) {
      insideToken = true;
      if (current == tokenCount) {
        String rest = input.substring(i);
        rest.trim();
        return rest;
      }
    }
    if (insideToken && isSpaceChar) {
      insideToken = false;
      current++;
    }
  }

  return String("");
}

uint8_t tokenCount(const String &input) {
  uint8_t count = 0;
  bool insideToken = false;
  uint16_t length = input.length();
  for (uint16_t i = 0; i < length; i++) {
    bool isSpaceChar = isCommandSpace(input.charAt(i));
    if (!insideToken && !isSpaceChar) {
      count++;
      insideToken = true;
    } else if (insideToken && isSpaceChar) {
      insideToken = false;
    }
  }
  return count;
}

bool parsePoseId(String text, uint8_t *poseId, bool missionOnly = false) {
  long value = 0;
  if (!parseLongStrict(text, &value)) return false;
  if (missionOnly && (value < 1 || value > 14)) return false;
  if (!missionOnly && (value < 1 || value > MANIPULATOR_POSE_ID_MAX_CNT)) return false;
  *poseId = (uint8_t)value;
  return true;
}

bool parsePoseMs(String text, uint16_t *timeMs) {
  long value = 0;
  if (!parseLongStrict(text, &value) || value <= 0 || value > 60000) {
    DEBUG_SERIAL.println(F("[오류] ms는 1~60000 사이의 양수로 입력하세요."));
    return false;
  }
  *timeMs = (uint16_t)value;
  return true;
}

bool parseSignatureMapText(String text, uint8_t *signatureMap) {
  text.trim();
  text.toLowerCase();
  if (text.length() == 0 || text == "all" || text == "전체") {
    *signatureMap = 0x7F; // Pixy2 CCC signature 1~7
    return true;
  }

  long parsed = 0;
  if (text.startsWith("0b")) {
    if (text.length() < 3 || text.length() > 9) return false;
    for (uint8_t i = 2; i < text.length(); i++) {
      char c = text.charAt(i);
      if (c != '0' && c != '1') return false;
      parsed = (parsed << 1) + (c == '1' ? 1 : 0);
    }
  } else if (!parseLongStrict(text, &parsed)) {
    return false;
  }

  if (parsed < 1 || parsed > 0x7F) return false;
  *signatureMap = (uint8_t)parsed;
  return true;
}

bool parseMotorTuneValue(String text,
                         int16_t baseValue,
                         int16_t minValue,
                         int16_t maxValue,
                         int16_t *targetValue) {
  text.trim();
  if (text.length() == 0) return false;

  bool absolute = false;
  if (text.charAt(0) == '=') {
    absolute = true;
    text = text.substring(1);
    text.trim();
  }

  long parsed = 0;
  if (!parseLongStrict(text, &parsed)) return false;

  long rawTarget = absolute ? parsed : (long)baseValue + parsed;
  rawTarget = constrain(rawTarget, minValue, maxValue);

  *targetValue = (int16_t)rawTarget;
  return true;
}

void printPoseValues(const __FlashStringHelper *label, ManipulatorPose pose) {
  DEBUG_SERIAL.println(label);
  DEBUG_SERIAL.print(F("  id=")); DEBUG_SERIAL.print(pose.id);
  DEBUG_SERIAL.print(F(", desc=")); DEBUG_SERIAL.println(pose.description);
  DEBUG_SERIAL.print(F("  m1=")); DEBUG_SERIAL.print(pose.manipulatorMotor1Value);
  DEBUG_SERIAL.print(F(", m2=")); DEBUG_SERIAL.print(pose.manipulatorMotor2Value);
  DEBUG_SERIAL.print(F(", m3=")); DEBUG_SERIAL.print(pose.manipulatorMotor3Value);
  DEBUG_SERIAL.print(F(", m4=")); DEBUG_SERIAL.println(pose.manipulatorMotor4Value);
}

ManipulatorPose readCurrentManipulatorPoseSnapshot() {
  ManipulatorPose pose;
  memset(&pose, 0, sizeof(pose));
  pose.isTherePoseData = true;
  pose.id = 0;
  pose.manipulatorMotor1Value = dxl.readControlTableItem(PRESENT_POSITION, ARM_DXL_IDS[0]);
  pose.manipulatorMotor2Value = dxl.readControlTableItem(PRESENT_POSITION, ARM_DXL_IDS[1]);
  pose.manipulatorMotor3Value = dxl.readControlTableItem(PRESENT_POSITION, ARM_DXL_IDS[2]);
  pose.manipulatorMotor4Value = dxl.readControlTableItem(PRESENT_POSITION, ARM_DXL_IDS[3]);
  strncpy(pose.description, "current", MANIPULATOR_POSE_DESCRIPTION_SIZE - 1);
  return pose;
}

void printPoseDelta(const __FlashStringHelper *label, ManipulatorPose fromPose, ManipulatorPose toPose) {
  DEBUG_SERIAL.println(label);
  DEBUG_SERIAL.print(F("  dm1="));
  DEBUG_SERIAL.print(toPose.manipulatorMotor1Value - fromPose.manipulatorMotor1Value);
  DEBUG_SERIAL.print(F(", dm2="));
  DEBUG_SERIAL.print(toPose.manipulatorMotor2Value - fromPose.manipulatorMotor2Value);
  DEBUG_SERIAL.print(F(", dm3="));
  DEBUG_SERIAL.print(toPose.manipulatorMotor3Value - fromPose.manipulatorMotor3Value);
  DEBUG_SERIAL.print(F(", dm4="));
  DEBUG_SERIAL.println(toPose.manipulatorMotor4Value - fromPose.manipulatorMotor4Value);
}

int16_t maxRawDeltaBetweenPoses(ManipulatorPose fromPose, ManipulatorPose toPose) {
  int32_t maxDelta = abs((int32_t)toPose.manipulatorMotor1Value - fromPose.manipulatorMotor1Value);
  maxDelta = max(maxDelta, abs((int32_t)toPose.manipulatorMotor2Value - fromPose.manipulatorMotor2Value));
  maxDelta = max(maxDelta, abs((int32_t)toPose.manipulatorMotor3Value - fromPose.manipulatorMotor3Value));
  maxDelta = max(maxDelta, abs((int32_t)toPose.manipulatorMotor4Value - fromPose.manipulatorMotor4Value));
  return (int16_t)min(maxDelta, 32767L);
}

bool parseRawMotorValue(String text,
                        int16_t minValue,
                        int16_t maxValue,
                        int16_t *targetValue) {
  text.trim();
  if (text.charAt(0) == '=') {
    text = text.substring(1);
    text.trim();
  }

  long parsed = 0;
  if (!parseLongStrict(text, &parsed) || parsed < minValue || parsed > maxValue) {
    DEBUG_SERIAL.print(F("[오류] raw 값 범위는 "));
    DEBUG_SERIAL.print(minValue);
    DEBUG_SERIAL.print(F("~"));
    DEBUG_SERIAL.print(maxValue);
    DEBUG_SERIAL.println(F("입니다."));
    return false;
  }

  *targetValue = (int16_t)parsed;
  return true;
}

void clearPendingPoseWrite() {
  memset(&pendingPoseWrite, 0, sizeof(pendingPoseWrite));
  hasPendingPoseWrite = false;
}

uint16_t clampPoseTime(uint16_t requestedMs) {
  uint16_t clamped = constrain(requestedMs, profile().minPoseMs, profile().maxPoseMs);
  if (clamped != requestedMs) {
    DEBUG_SERIAL.print(F("[시간 보정] 요청 "));
    DEBUG_SERIAL.print(requestedMs);
    DEBUG_SERIAL.print(F("ms -> "));
    DEBUG_SERIAL.print(clamped);
    DEBUG_SERIAL.println(F("ms"));
  }
  return clamped;
}

uint16_t safePoseTimeFromDelta(uint16_t requestedMs, int16_t maxRawDelta) {
  uint16_t profileMin = clampPoseTime(requestedMs);
  uint16_t deltaMin = (uint16_t)min((float)profile().maxPoseMs,
                                    profile().minPoseMs + abs(maxRawDelta) * profile().msPerRawTick);
  uint16_t safeMs = max(profileMin, deltaMin);
  if (safeMs != requestedMs) {
    DEBUG_SERIAL.print(F("[속도 보정] raw 변화량 "));
    DEBUG_SERIAL.print(abs(maxRawDelta));
    DEBUG_SERIAL.print(F(" 기준 동작시간 "));
    DEBUG_SERIAL.print(safeMs);
    DEBUG_SERIAL.println(F("ms로 실행합니다."));
  }
  return safeMs;
}

bool ensureManipulatorReady() {
  if (manipulatorReady) return true;
  DEBUG_SERIAL.println(F("[사용 불가] 매니퓰레이터 초기화가 성공하지 않았습니다."));
  DEBUG_SERIAL.println(F("          Dynamixel 전원/배선/ID를 확인하고 리셋하세요."));
  turnOnLEDRed500ms();
  return false;
}

bool ensureMobileReady() {
  if (mobileReady) return true;
  DEBUG_SERIAL.println(F("[사용 불가] 모바일베이스 초기화가 성공하지 않았습니다."));
  DEBUG_SERIAL.println(F("          drive 명령은 사용할 수 없습니다."));
  turnOnLEDRed500ms();
  return false;
}

int32_t motor1RawFromAngle(float motor1Angle) {
  return map(constrain(round(motor1Angle * 100), -10000, 10000),
             -18000, 18000, ARM_DXL_1_POSITION_MIN, ARM_DXL_1_POSITION_MAX);
}

int16_t maxRawDeltaFromPresentPose(ManipulatorPose targetPose, float motor1Angle) {
  int32_t targetMotor1 = (motor1Angle == -360.0)
                         ? targetPose.manipulatorMotor1Value
                         : motor1RawFromAngle(motor1Angle);
  int32_t presentMotor1 = dxl.readControlTableItem(PRESENT_POSITION, ARM_DXL_IDS[0]);
  int32_t presentMotor2 = dxl.readControlTableItem(PRESENT_POSITION, ARM_DXL_IDS[1]);
  int32_t presentMotor3 = dxl.readControlTableItem(PRESENT_POSITION, ARM_DXL_IDS[2]);
  int32_t presentMotor4 = dxl.readControlTableItem(PRESENT_POSITION, ARM_DXL_IDS[3]);

  int32_t maxDelta = abs(targetMotor1 - presentMotor1);
  maxDelta = max(maxDelta, abs((int32_t)targetPose.manipulatorMotor2Value - presentMotor2));
  maxDelta = max(maxDelta, abs((int32_t)targetPose.manipulatorMotor3Value - presentMotor3));
  maxDelta = max(maxDelta, abs((int32_t)targetPose.manipulatorMotor4Value - presentMotor4));
  return (int16_t)min(maxDelta, 32767L);
}

bool runPoseRequired(uint8_t poseId, uint16_t requestedMs, float motor1Angle = -360.0) {
  if (!ensureManipulatorReady()) return false;
  ManipulatorPose pose = ReadManipulatorPresentPoseToEEPROM(poseId);
  if (!pose.isTherePoseData) {
    DEBUG_SERIAL.print(F("[자세 없음] EEPROM "));
    DEBUG_SERIAL.print(poseId);
    DEBUG_SERIAL.println(F("번 자세가 없습니다."));
    DEBUG_SERIAL.println(F("             pose save 또는 manageManipulatorPose로 먼저 저장하세요."));
    turnOnLEDRed500ms();
    return false;
  }

  int16_t maxRawDelta = maxRawDeltaFromPresentPose(pose, motor1Angle);
  uint16_t operatingMs = safePoseTimeFromDelta(requestedMs, maxRawDelta);
  if (motor1Angle != -360.0) {
    DEBUG_SERIAL.print(F("[m1angle] 하부 회전 목표 "));
    DEBUG_SERIAL.print(motor1Angle);
    DEBUG_SERIAL.print(F("도, raw "));
    DEBUG_SERIAL.println(motor1RawFromAngle(motor1Angle));
  }
  RunManipulatorPoseWithPoseDataInEEPROM(poseId, operatingMs, motor1Angle);
  if (!interruptibleDelay(operatingMs + MANIPULATOR_SETTLE_MS)) return false;
  printPoseValues(F("[자세 실행]"), pose);
  turnOnLEDGreen500ms();
  return true;
}

bool runStagedPoseM4M3M2(uint8_t poseId, uint16_t stageMs, float motor1Angle) {
  if (!ensureManipulatorReady()) return false;
  ManipulatorPose targetPose = ReadManipulatorPresentPoseToEEPROM(poseId);
  if (!targetPose.isTherePoseData) {
    DEBUG_SERIAL.print(F("[자세 없음] EEPROM "));
    DEBUG_SERIAL.print(poseId);
    DEBUG_SERIAL.println(F("번 자세가 없습니다."));
    turnOnLEDRed500ms();
    return false;
  }

  ManipulatorPose currentPose = readCurrentManipulatorPoseSnapshot();
  int32_t current[4] = {
      currentPose.manipulatorMotor1Value,
      currentPose.manipulatorMotor2Value,
      currentPose.manipulatorMotor3Value,
      currentPose.manipulatorMotor4Value};
  int32_t target[4] = {
      motor1Angle == -360.0 ? targetPose.manipulatorMotor1Value : motor1RawFromAngle(motor1Angle),
      targetPose.manipulatorMotor2Value,
      targetPose.manipulatorMotor3Value,
      targetPose.manipulatorMotor4Value};
  static const uint8_t order[3] = {3, 2, 1};

  current[0] = target[0];
  DEBUG_SERIAL.println(F("[staged pose] m4 -> m3 -> m2"));
  for (uint8_t i = 0; i < 3; i++) {
    uint8_t motorIndex = order[i];
    current[motorIndex] = target[motorIndex];
    SetManipulatorForwardMoveWithMotorValueForSyncWrite(current[0],
                                                        current[1],
                                                        current[2],
                                                        current[3],
                                                        stageMs);
    if (!interruptibleDelay(stageMs + MANIPULATOR_SETTLE_MS)) return false;
  }
  printPoseValues(F("[staged 자세 실행]"), targetPose);
  turnOnLEDGreen500ms();
  return true;
}

bool writePoseToEEPROM(ManipulatorPose pose, String description) {
  if (!ensureManipulatorReady()) return false;
  description.trim();
  if (description.length() == 0) description = String("tuned");
  if (description.length() >= MANIPULATOR_POSE_DESCRIPTION_SIZE) {
    DEBUG_SERIAL.println(F("[저장 실패] 설명은 29byte 이하로 짧게 입력하세요."));
    turnOnLEDRed500ms();
    return false;
  }

  memset(pose.description, 0, MANIPULATOR_POSE_DESCRIPTION_SIZE);
  description.toCharArray(pose.description, MANIPULATOR_POSE_DESCRIPTION_SIZE);
  EEPROM.put(MANIPULATOR_POSE_DATA_SIZE * (pose.id - 1), pose);

  ManipulatorPose verifyPose = ReadManipulatorPresentPoseToEEPROM(pose.id);
  if (!verifyPose.isTherePoseData) {
    DEBUG_SERIAL.println(F("[저장 실패] 저장 후 다시 읽기 검증에 실패했습니다."));
    turnOnLEDRed500ms();
    return false;
  }

  printPoseValues(F("[저장 완료]"), verifyPose);
  DEBUG_SERIAL.print(F("{\"type\":\"eeprom-pose\",\"poseId\":"));
  DEBUG_SERIAL.print(verifyPose.id);
  DEBUG_SERIAL.print(F(",\"m1\":")); DEBUG_SERIAL.print(verifyPose.manipulatorMotor1Value);
  DEBUG_SERIAL.print(F(",\"m2\":")); DEBUG_SERIAL.print(verifyPose.manipulatorMotor2Value);
  DEBUG_SERIAL.print(F(",\"m3\":")); DEBUG_SERIAL.print(verifyPose.manipulatorMotor3Value);
  DEBUG_SERIAL.print(F(",\"m4\":")); DEBUG_SERIAL.print(verifyPose.manipulatorMotor4Value);
  DEBUG_SERIAL.print(F(",\"notes\":\"")); DEBUG_SERIAL.print(verifyPose.description);
  DEBUG_SERIAL.println(F("\"}"));
  turnOnLEDGreen500ms();
  return true;
}

bool stagePoseWrite(ManipulatorPose pose, String description,
                    const __FlashStringHelper *sourceLabel) {
  if (!ensureManipulatorReady()) return false;
  if (pose.id < 1 || pose.id > MANIPULATOR_POSE_ID_MAX_CNT) {
    DEBUG_SERIAL.println(F("[저장 실패] pose id 범위 오류"));
    turnOnLEDRed500ms();
    return false;
  }

  description.trim();
  if (description.length() == 0) description = String("pending");
  if (description.length() >= MANIPULATOR_POSE_DESCRIPTION_SIZE) {
    DEBUG_SERIAL.println(F("[저장 실패] 설명은 29byte 이하로 짧게 입력하세요."));
    turnOnLEDRed500ms();
    return false;
  }

  memset(pose.description, 0, MANIPULATOR_POSE_DESCRIPTION_SIZE);
  description.toCharArray(pose.description, MANIPULATOR_POSE_DESCRIPTION_SIZE);

  ManipulatorPose savedPose = ReadManipulatorPresentPoseToEEPROM(pose.id);
  DEBUG_SERIAL.println(F("[EEPROM 저장 대기] 아직 저장하지 않았습니다."));
  DEBUG_SERIAL.print(F("  source="));
  DEBUG_SERIAL.println(sourceLabel);
  if (savedPose.isTherePoseData) {
    printPoseValues(F("[기존 EEPROM]"), savedPose);
    printPoseDelta(F("[덮어쓰기 변화량] 새 값 - 기존 EEPROM"), savedPose, pose);
    DEBUG_SERIAL.print(F("  maxRawDelta="));
    DEBUG_SERIAL.println(maxRawDeltaBetweenPoses(savedPose, pose));
  } else {
    DEBUG_SERIAL.print(F("[기존 EEPROM] "));
    DEBUG_SERIAL.print(pose.id);
    DEBUG_SERIAL.println(F("번 자세 없음"));
  }
  printPoseValues(F("[새로 저장될 값]"), pose);
  DEBUG_SERIAL.println(F("실제 저장하려면: pose confirm"));
  DEBUG_SERIAL.println(F("취소하려면: pose cancel"));

  pendingPoseWrite = pose;
  hasPendingPoseWrite = true;
  turnOnLEDGreen500ms();
  return true;
}

bool commandPoseRun(const String &input) {
  uint8_t poseId = 0;
  uint16_t timeMs = 0;
  if (!parsePoseId(tokenAt(input, 2), &poseId, false) ||
      !parsePoseMs(tokenAt(input, 3), &timeMs)) {
    DEBUG_SERIAL.println(F("사용법: pose run <id> <ms> [m1angle]"));
    return false;
  }

  float motor1Angle = -360.0;
  if (tokenCount(input) >= 5) {
    long angle = 0;
    if (!parseLongStrict(tokenAt(input, 4), &angle)) {
      DEBUG_SERIAL.println(F("[오류] m1angle은 정수 각도로 입력하세요. 예: -90"));
      return false;
    }
    motor1Angle = (float)constrain(angle, -100, 100);
  }

  return runPoseRequired(poseId, timeMs, motor1Angle);
}

bool commandPoseTune(const String &input) {
  uint8_t poseId = 0;
  uint16_t timeMs = 0;
  if (!parsePoseId(tokenAt(input, 2), &poseId, false) ||
      !parsePoseMs(tokenAt(input, 3), &timeMs) ||
      tokenCount(input) < 8) {
    DEBUG_SERIAL.println(F("사용법: pose tune <id> <ms> <m1> <m2> <m3> <m4>"));
    DEBUG_SERIAL.println(F("예시: pose tune 1 300 0 -30 +20 0"));
    DEBUG_SERIAL.println(F("예시: pose tune 1 300 =2048 =2200 =1800 =2000"));
    return false;
  }
  if (!ensureManipulatorReady()) return false;

  ManipulatorPose basePose = ReadManipulatorPresentPoseToEEPROM(poseId);
  if (!basePose.isTherePoseData) {
    DEBUG_SERIAL.println(F("[자세 없음] 먼저 EEPROM에 저장된 자세가 필요합니다."));
    turnOnLEDRed500ms();
    return false;
  }

  ManipulatorPose tunedPose = basePose;
  if (!parseMotorTuneValue(tokenAt(input, 4), basePose.manipulatorMotor1Value,
                           ARM_DXL_1_POSITION_MIN, ARM_DXL_1_POSITION_MAX,
                           &tunedPose.manipulatorMotor1Value)) return false;
  if (!parseMotorTuneValue(tokenAt(input, 5), basePose.manipulatorMotor2Value,
                           ARM_DXL_2_POSITION_MIN, ARM_DXL_2_POSITION_MAX,
                           &tunedPose.manipulatorMotor2Value)) return false;
  if (!parseMotorTuneValue(tokenAt(input, 6), basePose.manipulatorMotor3Value,
                           ARM_DXL_3_POSITION_MIN, ARM_DXL_3_POSITION_MAX,
                           &tunedPose.manipulatorMotor3Value)) return false;
  if (!parseMotorTuneValue(tokenAt(input, 7), basePose.manipulatorMotor4Value,
                           ARM_DXL_4_POSITION_MIN, ARM_DXL_4_POSITION_MAX,
                           &tunedPose.manipulatorMotor4Value)) return false;

  ManipulatorPose presentPose = readCurrentManipulatorPoseSnapshot();
  int16_t maxStepDelta = maxRawDeltaBetweenPoses(presentPose, tunedPose);
  int16_t maxEepromDelta = maxRawDeltaBetweenPoses(basePose, tunedPose);
  uint16_t operatingMs = safePoseTimeFromDelta(timeMs, maxStepDelta);

  SetManipulatorForwardMoveWithMotorValueForSyncWrite(tunedPose.manipulatorMotor1Value,
                                                       tunedPose.manipulatorMotor2Value,
                                                       tunedPose.manipulatorMotor3Value,
                                                       tunedPose.manipulatorMotor4Value,
                                                       operatingMs);
  if (!interruptibleDelay(operatingMs + MANIPULATOR_SETTLE_MS)) return false;

  lastTunedPose = tunedPose;
  hasLastTunedPose = true;
  clearPendingPoseWrite();
  printPoseValues(F("[기준 자세]"), basePose);
  printPoseValues(F("[테스트 자세]"), tunedPose);
  printPoseDelta(F("[실제 1회 이동량] 테스트 - 현재"), presentPose, tunedPose);
  DEBUG_SERIAL.print(F("  stepMaxRawDelta="));
  DEBUG_SERIAL.print(maxStepDelta);
  DEBUG_SERIAL.print(F(", eepromMaxRawDelta="));
  DEBUG_SERIAL.println(maxEepromDelta);
  DEBUG_SERIAL.println(F("아직 저장되지 않았습니다. 저장 후보 생성: pose apply <description>, 실제 저장: pose confirm"));
  turnOnLEDGreen500ms();
  return true;
}

bool commandPoseSave(const String &input) {
  uint8_t poseId = 0;
  if (!parsePoseId(tokenAt(input, 2), &poseId, false)) {
    DEBUG_SERIAL.println(F("사용법: pose save <id> <description>"));
    return false;
  }
  if (!ensureManipulatorReady()) return false;

  String description = restAfterToken(input, 3);
  description.trim();
  if (description.length() == 0) description = String("manual");
  if (description.length() >= MANIPULATOR_POSE_DESCRIPTION_SIZE) {
    DEBUG_SERIAL.println(F("[저장 실패] 설명은 29byte 이하로 짧게 입력하세요."));
    turnOnLEDRed500ms();
    return false;
  }

  ManipulatorPose pose = readCurrentManipulatorPoseSnapshot();
  pose.id = poseId;
  return stagePoseWrite(pose, description, F("present"));
}

bool commandPoseApply(const String &input) {
  if (!hasLastTunedPose) {
    DEBUG_SERIAL.println(F("[적용 불가] 먼저 pose tune을 실행하세요."));
    return false;
  }
  String description = restAfterToken(input, 2);
  return stagePoseWrite(lastTunedPose, description, F("last-tune"));
}

bool commandPoseRestore(const String &input) {
  uint8_t poseId = 0;
  if (!parsePoseId(tokenAt(input, 2), &poseId, false) || tokenCount(input) < 7) {
    DEBUG_SERIAL.println(F("사용법: pose restore <id> <m1> <m2> <m3> <m4> [description]"));
    return false;
  }

  ManipulatorPose pose;
  memset(&pose, 0, sizeof(pose));
  pose.isTherePoseData = true;
  pose.id = poseId;
  if (!parseRawMotorValue(tokenAt(input, 3), ARM_DXL_1_POSITION_MIN, ARM_DXL_1_POSITION_MAX,
                          &pose.manipulatorMotor1Value)) return false;
  if (!parseRawMotorValue(tokenAt(input, 4), ARM_DXL_2_POSITION_MIN, ARM_DXL_2_POSITION_MAX,
                          &pose.manipulatorMotor2Value)) return false;
  if (!parseRawMotorValue(tokenAt(input, 5), ARM_DXL_3_POSITION_MIN, ARM_DXL_3_POSITION_MAX,
                          &pose.manipulatorMotor3Value)) return false;
  if (!parseRawMotorValue(tokenAt(input, 6), ARM_DXL_4_POSITION_MIN, ARM_DXL_4_POSITION_MAX,
                          &pose.manipulatorMotor4Value)) return false;

  String description = restAfterToken(input, 7);
  if (description.length() == 0) description = String("restored");
  return stagePoseWrite(pose, description, F("restore-raw"));
}

bool commandPoseConfirm() {
  if (!hasPendingPoseWrite) {
    DEBUG_SERIAL.println(F("[저장 없음] 대기 중인 pose 저장이 없습니다."));
    DEBUG_SERIAL.println(F("  먼저 pose save/apply/restore를 실행해 저장 후보를 확인하세요."));
    return false;
  }

  String description = String(pendingPoseWrite.description);
  ManipulatorPose pose = pendingPoseWrite;
  clearPendingPoseWrite();
  return writePoseToEEPROM(pose, description);
}

bool commandPoseCancel() {
  if (!hasPendingPoseWrite) {
    DEBUG_SERIAL.println(F("[취소 없음] 대기 중인 pose 저장이 없습니다."));
    return false;
  }
  clearPendingPoseWrite();
  DEBUG_SERIAL.println(F("[저장 취소] EEPROM에 쓰지 않았습니다."));
  turnOnLEDGreen500ms();
  return true;
}

void commandPoseList() {
  PrintManipulatorPoseListFromEEPROM();
}

void commandPoseBackup() {
  DEBUG_SERIAL.println(F("====== EEPROM POSE BACKUP ======"));
  for (uint8_t id = 1; id <= 14; id++) {
    ManipulatorPose pose = ReadManipulatorPresentPoseToEEPROM(id);
    if (pose.isTherePoseData) {
      DEBUG_SERIAL.print(pose.id); DEBUG_SERIAL.print(F(","));
      DEBUG_SERIAL.print(pose.manipulatorMotor1Value); DEBUG_SERIAL.print(F(","));
      DEBUG_SERIAL.print(pose.manipulatorMotor2Value); DEBUG_SERIAL.print(F(","));
      DEBUG_SERIAL.print(pose.manipulatorMotor3Value); DEBUG_SERIAL.print(F(","));
      DEBUG_SERIAL.print(pose.manipulatorMotor4Value); DEBUG_SERIAL.print(F(","));
      DEBUG_SERIAL.println(pose.description);
    } else {
      DEBUG_SERIAL.print(F("# missing pose "));
      DEBUG_SERIAL.println(id);
    }
  }
  DEBUG_SERIAL.println(F("================================"));
}

bool commandPosePresent() {
  if (!ensureManipulatorReady()) return false;
  ManipulatorPose presentPose = readCurrentManipulatorPoseSnapshot();
  printPoseValues(F("[현재 자세]"), presentPose);
  DEBUG_SERIAL.print(F("{\"type\":\"pose-present\",\"m1\":"));
  DEBUG_SERIAL.print(presentPose.manipulatorMotor1Value);
  DEBUG_SERIAL.print(F(",\"m2\":")); DEBUG_SERIAL.print(presentPose.manipulatorMotor2Value);
  DEBUG_SERIAL.print(F(",\"m3\":")); DEBUG_SERIAL.print(presentPose.manipulatorMotor3Value);
  DEBUG_SERIAL.print(F(",\"m4\":")); DEBUG_SERIAL.print(presentPose.manipulatorMotor4Value);
  DEBUG_SERIAL.println(F("}"));
  return true;
}

bool commandPoseDiff(const String &input) {
  uint8_t poseId = 0;
  if (!parsePoseId(tokenAt(input, 2), &poseId, false)) {
    DEBUG_SERIAL.println(F("사용법: pose diff <id>"));
    return false;
  }
  if (!ensureManipulatorReady()) return false;

  ManipulatorPose savedPose = ReadManipulatorPresentPoseToEEPROM(poseId);
  if (!savedPose.isTherePoseData) {
    DEBUG_SERIAL.print(F("[자세 없음] EEPROM "));
    DEBUG_SERIAL.print(poseId);
    DEBUG_SERIAL.println(F("번 자세가 없습니다."));
    turnOnLEDRed500ms();
    return false;
  }

  ManipulatorPose presentPose = readCurrentManipulatorPoseSnapshot();
  printPoseValues(F("[현재 자세]"), presentPose);
  printPoseValues(F("[EEPROM 기준 자세]"), savedPose);
  printPoseDelta(F("[차이] EEPROM - 현재"), presentPose, savedPose);
  DEBUG_SERIAL.print(F("  maxRawDelta="));
  DEBUG_SERIAL.println(maxRawDeltaBetweenPoses(presentPose, savedPose));
  return true;
}

bool isRequiredMissionPose(uint8_t id) {
  if (id == POSE_INITIAL ||
      id == POSE_STORAGE ||
      id == POSE_GRIP_UPPER ||
      id == POSE_GRIP_LOWER) {
    return true;
  }
  return id > CFG.pose.missionZoneStartId &&
         id <= CFG.pose.missionZoneStartId + CFG.pose.missionZoneSlotCount;
}

bool commandPoseVerify() {
  bool ok = true;
  DEBUG_SERIAL.println(F("====== EEPROM 미션 자세 검증 ======"));
  for (uint8_t id = 1;
       id <= CFG.pose.missionZoneStartId + CFG.pose.missionZoneSlotCount;
       id++) {
    if (!isRequiredMissionPose(id)) continue;
    ManipulatorPose pose = ReadManipulatorPresentPoseToEEPROM(id);
    DEBUG_SERIAL.print(F("  "));
    DEBUG_SERIAL.print(id);
    DEBUG_SERIAL.print(F("번: "));
    if (pose.isTherePoseData) {
      DEBUG_SERIAL.print(F("OK  "));
      DEBUG_SERIAL.print(pose.description);
      DEBUG_SERIAL.print(F("  m1=")); DEBUG_SERIAL.print(pose.manipulatorMotor1Value);
      DEBUG_SERIAL.print(F(", m2=")); DEBUG_SERIAL.print(pose.manipulatorMotor2Value);
      DEBUG_SERIAL.print(F(", m3=")); DEBUG_SERIAL.print(pose.manipulatorMotor3Value);
      DEBUG_SERIAL.print(F(", m4=")); DEBUG_SERIAL.println(pose.manipulatorMotor4Value);
    } else {
      DEBUG_SERIAL.println(F("누락"));
      ok = false;
    }
  }
  DEBUG_SERIAL.println(ok ? F("[검증 OK] 현재 실제 테스트에 필요한 미션 자세가 모두 있습니다.")
                          : F("[검증 실패] 누락된 자세를 먼저 저장해야 합니다."));
  DEBUG_SERIAL.println(F("=================================="));
  if (!ok) turnOnLEDRed500ms();
  return ok;
}

void printJsonEscapedChar(uint8_t ch) {
  switch (ch) {
    case '"': DEBUG_SERIAL.print(F("\\\"")); break;
    case '\\': DEBUG_SERIAL.print(F("\\\\")); break;
    case '\b': DEBUG_SERIAL.print(F("\\b")); break;
    case '\f': DEBUG_SERIAL.print(F("\\f")); break;
    case '\n': DEBUG_SERIAL.print(F("\\n")); break;
    case '\r': DEBUG_SERIAL.print(F("\\r")); break;
    case '\t': DEBUG_SERIAL.print(F("\\t")); break;
    default:
      if (ch < 32) {
        DEBUG_SERIAL.print(F("\\u00"));
        if (ch < 16) DEBUG_SERIAL.print('0');
        DEBUG_SERIAL.print(ch, HEX);
      } else {
        DEBUG_SERIAL.print((char)ch);
      }
      break;
  }
}

void printJsonString(const char *value) {
  DEBUG_SERIAL.print('"');
  if (value != NULL) {
    while (*value) {
      printJsonEscapedChar((uint8_t)*value);
      value++;
    }
  }
  DEBUG_SERIAL.print('"');
}

void printJsonString(const String &value) {
  DEBUG_SERIAL.print('"');
  for (uint16_t i = 0; i < value.length(); i++) {
    printJsonEscapedChar((uint8_t)value.charAt(i));
  }
  DEBUG_SERIAL.print('"');
}

void printJsonBool(bool value) {
  DEBUG_SERIAL.print(value ? F("true") : F("false"));
}

void printJsonPsdSnapshot(const PsdSnapshot &snapshot) {
  DEBUG_SERIAL.print(F("{\"fl\":"));
  DEBUG_SERIAL.print(snapshot.fl);
  DEBUG_SERIAL.print(F(",\"fr\":"));
  DEBUG_SERIAL.print(snapshot.fr);
  DEBUG_SERIAL.print(F(",\"sl\":"));
  DEBUG_SERIAL.print(snapshot.sl);
  DEBUG_SERIAL.print(F(",\"sr\":"));
  DEBUG_SERIAL.print(snapshot.sr);
  DEBUG_SERIAL.print(F(",\"targets\":{\"instructionSl\":"));
  DEBUG_SERIAL.print(missionInstructionSl);
  DEBUG_SERIAL.print(F(",\"alignFl\":"));
  DEBUG_SERIAL.print(missionAlignFl);
  DEBUG_SERIAL.print(F(",\"alignFr\":"));
  DEBUG_SERIAL.print(missionAlignFr);
  DEBUG_SERIAL.print(F(",\"alignSl\":"));
  DEBUG_SERIAL.print(missionAlignSl);
  DEBUG_SERIAL.print(F(",\"alignTolerance\":"));
  DEBUG_SERIAL.print(missionAlignTolerance);
  DEBUG_SERIAL.print(F(",\"gripAlignFl\":"));
  DEBUG_SERIAL.print(missionGripAlignFl);
  DEBUG_SERIAL.print(F(",\"gripAlignFr\":"));
  DEBUG_SERIAL.print(missionGripAlignFr);
  DEBUG_SERIAL.print(F(",\"gripAlignSl\":"));
  DEBUG_SERIAL.print(missionGripAlignSl);
  DEBUG_SERIAL.print(F(",\"gripAlignTolerance\":"));
  DEBUG_SERIAL.print(missionGripAlignTolerance);
  DEBUG_SERIAL.print(F(",\"lowerGripAlignFl\":"));
  DEBUG_SERIAL.print(missionLowerGripAlignFl);
  DEBUG_SERIAL.print(F(",\"lowerGripAlignFr\":"));
  DEBUG_SERIAL.print(missionLowerGripAlignFr);
  DEBUG_SERIAL.print(F(",\"lowerGripAlignSl\":"));
  DEBUG_SERIAL.print(missionLowerGripAlignSl);
  DEBUG_SERIAL.print(F(",\"lowerGripAlignTolerance\":"));
  DEBUG_SERIAL.print(missionLowerGripAlignTolerance);
  DEBUG_SERIAL.print(F(",\"placeSl\":"));
  DEBUG_SERIAL.print(missionPlaceSl);
  DEBUG_SERIAL.print(F(",\"placeFr\":"));
  DEBUG_SERIAL.print(missionPlaceFr);
  DEBUG_SERIAL.print(F(",\"placeTolerance\":"));
  DEBUG_SERIAL.print(missionPlaceTolerance);
  DEBUG_SERIAL.print(F(",\"finishPreAlignSl\":"));
  DEBUG_SERIAL.print(missionFinishPreAlignSl);
  DEBUG_SERIAL.print(F(",\"finishPreAlignTolerance\":"));
  DEBUG_SERIAL.print(missionFinishPreAlignTolerance);
  DEBUG_SERIAL.print(F(",\"sideRightUsedForAlign\":false}}"));
}

void printJsonUint8Array(const uint8_t *values, uint8_t count) {
  DEBUG_SERIAL.print('[');
  for (uint8_t i = 0; i < count; i++) {
    if (i > 0) DEBUG_SERIAL.print(',');
    DEBUG_SERIAL.print(values[i]);
  }
  DEBUG_SERIAL.print(']');
}

void printJsonInt16Array(const int16_t *values, uint8_t count) {
  DEBUG_SERIAL.print('[');
  for (uint8_t i = 0; i < count; i++) {
    if (i > 0) DEBUG_SERIAL.print(',');
    DEBUG_SERIAL.print(values[i]);
  }
  DEBUG_SERIAL.print(']');
}

void printJsonRequiredPoseIds() {
  DEBUG_SERIAL.print(F("[1,3,4,5"));
  for (uint8_t slot = 1; slot <= CFG.pose.missionZoneSlotCount; slot++) {
    DEBUG_SERIAL.print(',');
    DEBUG_SERIAL.print(CFG.pose.missionZoneStartId + slot);
  }
  DEBUG_SERIAL.print(']');
}

void printJsonPoseObject(uint8_t poseId, const ManipulatorPose &pose) {
  DEBUG_SERIAL.print(F("{\"id\":"));
  DEBUG_SERIAL.print(poseId);
  DEBUG_SERIAL.print(F(",\"required\":"));
  printJsonBool(isRequiredMissionPose(poseId));
  DEBUG_SERIAL.print(F(",\"present\":"));
  printJsonBool(pose.isTherePoseData);
  if (pose.isTherePoseData) {
    DEBUG_SERIAL.print(F(",\"m1\":"));
    DEBUG_SERIAL.print(pose.manipulatorMotor1Value);
    DEBUG_SERIAL.print(F(",\"m2\":"));
    DEBUG_SERIAL.print(pose.manipulatorMotor2Value);
    DEBUG_SERIAL.print(F(",\"m3\":"));
    DEBUG_SERIAL.print(pose.manipulatorMotor3Value);
    DEBUG_SERIAL.print(F(",\"m4\":"));
    DEBUG_SERIAL.print(pose.manipulatorMotor4Value);
    DEBUG_SERIAL.print(F(",\"description\":"));
    printJsonString(pose.description);
  }
  DEBUG_SERIAL.print('}');
}

void printJsonPoseSnapshot(const ManipulatorPose &pose) {
  DEBUG_SERIAL.print(F("{\"m1\":"));
  DEBUG_SERIAL.print(pose.manipulatorMotor1Value);
  DEBUG_SERIAL.print(F(",\"m2\":"));
  DEBUG_SERIAL.print(pose.manipulatorMotor2Value);
  DEBUG_SERIAL.print(F(",\"m3\":"));
  DEBUG_SERIAL.print(pose.manipulatorMotor3Value);
  DEBUG_SERIAL.print(F(",\"m4\":"));
  DEBUG_SERIAL.print(pose.manipulatorMotor4Value);
  DEBUG_SERIAL.print('}');
}

void printJsonMissionMotion();

void printJsonTunerCalibration() {
  TunerCalibrationRecord storedRecord;
  bool hasStoredRecord = readTunerCalibrationRecord(&storedRecord);
  DEBUG_SERIAL.print(F("{\"eepromAddress\":"));
  DEBUG_SERIAL.print(TUNER_CAL_EEPROM_ADDR);
  DEBUG_SERIAL.print(F(",\"recordSize\":"));
  DEBUG_SERIAL.print(sizeof(TunerCalibrationRecord));
  DEBUG_SERIAL.print(F(",\"poseEepromEnd\":"));
  DEBUG_SERIAL.print(TUNER_CAL_EEPROM_ADDR);
  DEBUG_SERIAL.print(F(",\"eepromLength\":"));
  DEBUG_SERIAL.print(EEPROM.length());
  DEBUG_SERIAL.print(F(",\"storedValid\":"));
  printJsonBool(hasStoredRecord);
  DEBUG_SERIAL.print(F(",\"current\":"));
  printJsonMissionMotion();
  DEBUG_SERIAL.print(F(",\"align\":{\"fl\":"));
  DEBUG_SERIAL.print(missionAlignFl);
  DEBUG_SERIAL.print(F(",\"fr\":"));
  DEBUG_SERIAL.print(missionAlignFr);
  DEBUG_SERIAL.print(F(",\"sl\":"));
  DEBUG_SERIAL.print(missionAlignSl);
  DEBUG_SERIAL.print(F(",\"tolerance\":"));
  DEBUG_SERIAL.print(missionAlignTolerance);
  DEBUG_SERIAL.print(F(",\"sideRightUsed\":false},\"gripAlign\":{\"fl\":"));
  DEBUG_SERIAL.print(missionGripAlignFl);
  DEBUG_SERIAL.print(F(",\"fr\":"));
  DEBUG_SERIAL.print(missionGripAlignFr);
  DEBUG_SERIAL.print(F(",\"sl\":"));
  DEBUG_SERIAL.print(missionGripAlignSl);
  DEBUG_SERIAL.print(F(",\"tolerance\":"));
  DEBUG_SERIAL.print(missionGripAlignTolerance);
  DEBUG_SERIAL.print(F(",\"sideLeftUsedForMotion\":false,\"sideRightUsed\":false},\"lowerGripAlign\":{\"fl\":"));
  DEBUG_SERIAL.print(missionLowerGripAlignFl);
  DEBUG_SERIAL.print(F(",\"fr\":"));
  DEBUG_SERIAL.print(missionLowerGripAlignFr);
  DEBUG_SERIAL.print(F(",\"sl\":"));
  DEBUG_SERIAL.print(missionLowerGripAlignSl);
  DEBUG_SERIAL.print(F(",\"tolerance\":"));
  DEBUG_SERIAL.print(missionLowerGripAlignTolerance);
  DEBUG_SERIAL.print(F(",\"sideLeftUsedForMotion\":false,\"sideRightUsed\":false},\"placeAlign\":{\"sl\":"));
  DEBUG_SERIAL.print(missionPlaceSl);
  DEBUG_SERIAL.print(F(",\"fr\":"));
  DEBUG_SERIAL.print(missionPlaceFr);
  DEBUG_SERIAL.print(F(",\"tolerance\":"));
  DEBUG_SERIAL.print(missionPlaceTolerance);
  DEBUG_SERIAL.print(F(",\"sideRightUsed\":false,\"frontLeftUsed\":false},\"gripDepth\":{\"upperMm\":"));
  DEBUG_SERIAL.print(missionUpperGripDepthMm, 2);
  DEBUG_SERIAL.print(F(",\"lowerMm\":"));
  DEBUG_SERIAL.print(missionLowerGripDepthMm, 2);
  DEBUG_SERIAL.print(F(",\"speedMmPerSec\":"));
  DEBUG_SERIAL.print(missionGripDepthMmPerSec);
  DEBUG_SERIAL.print(F("}}"));
}

void printJsonMissionMotion() {
  DEBUG_SERIAL.print(F("{\"frontCruiseSpeed\":"));
  DEBUG_SERIAL.print(missionMotion.frontCruiseSpeed);
  DEBUG_SERIAL.print(F(",\"frontSlowSpeed\":"));
  DEBUG_SERIAL.print(missionMotion.frontSlowSpeed);
  DEBUG_SERIAL.print(F(",\"psdCorrectionSpeed\":"));
  DEBUG_SERIAL.print(missionMotion.psdCorrectionSpeed);
  DEBUG_SERIAL.print(F(",\"frontDepthCorrectionSpeed\":"));
  DEBUG_SERIAL.print(missionMotion.frontDepthCorrectionSpeed);
  DEBUG_SERIAL.print(F(",\"missionZonePlaceCorrectionSpeed\":"));
  DEBUG_SERIAL.print(missionMotion.missionZonePlaceCorrectionSpeed);
  DEBUG_SERIAL.print(F(",\"positionMoveMmPerSec\":"));
  DEBUG_SERIAL.print(missionMotion.positionMoveMmPerSec);
  DEBUG_SERIAL.print(F(",\"wheelVelocityTrim\":{\"fl\":"));
  DEBUG_SERIAL.print(missionWheelVelocityTrimFl, 3);
  DEBUG_SERIAL.print(F(",\"fr\":"));
  DEBUG_SERIAL.print(missionWheelVelocityTrimFr, 3);
  DEBUG_SERIAL.print(F(",\"bl\":"));
  DEBUG_SERIAL.print(missionWheelVelocityTrimBl, 3);
  DEBUG_SERIAL.print(F(",\"br\":"));
  DEBUG_SERIAL.print(missionWheelVelocityTrimBr, 3);
  DEBUG_SERIAL.print(F("}"));
  DEBUG_SERIAL.print(F(",\"fixedActuatorMs\":"));
  DEBUG_SERIAL.print(MISSION_ACTUATOR_MS);
  DEBUG_SERIAL.print(F(",\"columnStepMm\":"));
  DEBUG_SERIAL.print(missionColumnStepMm, 2);
  DEBUG_SERIAL.print(F(",\"columnMoveMmPerSec\":"));
  DEBUG_SERIAL.print(missionColumnMoveMmPerSec);
  DEBUG_SERIAL.print(F(",\"columnScanSettleMs\":"));
  DEBUG_SERIAL.print(missionColumnScanSettleMs);
  DEBUG_SERIAL.print(F(",\"columnScanFrames\":"));
  DEBUG_SERIAL.print(missionColumnScanFrames);
  DEBUG_SERIAL.print(F(",\"columnScanSampleMs\":"));
  DEBUG_SERIAL.print(missionColumnScanSampleMs);
  DEBUG_SERIAL.print(F(",\"frontFirstDetectAdc\":"));
  DEBUG_SERIAL.print(missionFrontFirstDetectAdc);
  DEBUG_SERIAL.print(F(",\"frontDecelWindowAdc\":"));
  DEBUG_SERIAL.print(missionFrontDecelWindowAdc);
  DEBUG_SERIAL.print(F(",\"frontAfterDetectMm\":"));
  DEBUG_SERIAL.print(missionFrontAfterDetectMm, 2);
  DEBUG_SERIAL.print(F(",\"frontAfterDetectRaw\":"));
  DEBUG_SERIAL.print(missionFrontAfterDetectMmPerSec);
  DEBUG_SERIAL.print(F(",\"instructionSl\":"));
  DEBUG_SERIAL.print(missionInstructionSl);
  DEBUG_SERIAL.print(F(",\"instructionFinalForwardMs\":"));
  DEBUG_SERIAL.print(missionInstructionFinalForwardMs);
  DEBUG_SERIAL.print(F(",\"instructionFinalForwardSpeed\":"));
  DEBUG_SERIAL.print(missionInstructionFinalForwardSpeed);
  DEBUG_SERIAL.print(F(",\"instructionScanMs\":"));
  DEBUG_SERIAL.print(missionInstructionScanMs);
  DEBUG_SERIAL.print(F(",\"instructionScanSampleMs\":"));
  DEBUG_SERIAL.print(missionInstructionScanSampleMs);
  DEBUG_SERIAL.print(F(",\"storageFirstForwardMm\":"));
  DEBUG_SERIAL.print(missionStorageFirstForwardMm, 2);
  DEBUG_SERIAL.print(F(",\"storageExtraForwardMm\":"));
  DEBUG_SERIAL.print(missionStorageExtraForwardMm, 2);
  DEBUG_SERIAL.print(F(",\"storageRightMm\":"));
  DEBUG_SERIAL.print(missionStorageRightMm, 2);
  DEBUG_SERIAL.print(F(",\"storageApproachSlLeave\":"));
  DEBUG_SERIAL.print(missionStorageApproachSlLeaveAdc);
  DEBUG_SERIAL.print(F(",\"storageApproachSlReenter\":"));
  DEBUG_SERIAL.print(missionStorageApproachSlReenterAdc);
  DEBUG_SERIAL.print(F(",\"storageApproachSlReenterConfirmSamples\":"));
  DEBUG_SERIAL.print(missionStorageApproachSlReenterConfirmSamples);
  DEBUG_SERIAL.print(F(",\"storageApproachIgnoreReentryMs\":"));
  DEBUG_SERIAL.print(missionStorageApproachIgnoreReentryMs);
  DEBUG_SERIAL.print(F(",\"storageApproachFrMin\":"));
  DEBUG_SERIAL.print(missionStorageApproachFrDetectAdc);
  DEBUG_SERIAL.print(F(",\"storageApproachFrLeadDelta\":"));
  DEBUG_SERIAL.print(missionStorageApproachFrLeadDeltaAdc);
  DEBUG_SERIAL.print(F(",\"storageApproachFrLeadConfirmSamples\":"));
  DEBUG_SERIAL.print(missionStorageApproachFrLeadConfirmSamples);
  DEBUG_SERIAL.print(F(",\"storageApproachSlGateTolerance\":"));
  DEBUG_SERIAL.print(missionStorageApproachSlGateTolerance);
  DEBUG_SERIAL.print(F(",\"storageApproachRightSpeed\":"));
  DEBUG_SERIAL.print(missionStorageApproachRightSpeed);
  DEBUG_SERIAL.print(F(",\"storageApproachForwardSpeed\":"));
  DEBUG_SERIAL.print(missionStorageApproachForwardSpeed);
  DEBUG_SERIAL.print(F(",\"alignFl\":"));
  DEBUG_SERIAL.print(missionAlignFl);
  DEBUG_SERIAL.print(F(",\"alignFr\":"));
  DEBUG_SERIAL.print(missionAlignFr);
  DEBUG_SERIAL.print(F(",\"alignSl\":"));
  DEBUG_SERIAL.print(missionAlignSl);
  DEBUG_SERIAL.print(F(",\"alignTolerance\":"));
  DEBUG_SERIAL.print(missionAlignTolerance);
  DEBUG_SERIAL.print(F(",\"gripAlignFl\":"));
  DEBUG_SERIAL.print(missionGripAlignFl);
  DEBUG_SERIAL.print(F(",\"gripAlignFr\":"));
  DEBUG_SERIAL.print(missionGripAlignFr);
  DEBUG_SERIAL.print(F(",\"gripAlignSl\":"));
  DEBUG_SERIAL.print(missionGripAlignSl);
  DEBUG_SERIAL.print(F(",\"gripAlignTolerance\":"));
  DEBUG_SERIAL.print(missionGripAlignTolerance);
  DEBUG_SERIAL.print(F(",\"upperGripDepthMm\":"));
  DEBUG_SERIAL.print(missionUpperGripDepthMm, 2);
  DEBUG_SERIAL.print(F(",\"lowerGripDepthMm\":"));
  DEBUG_SERIAL.print(missionLowerGripDepthMm, 2);
  DEBUG_SERIAL.print(F(",\"gripDepthMmPerSec\":"));
  DEBUG_SERIAL.print(missionGripDepthMmPerSec);
  DEBUG_SERIAL.print(F(",\"placeSl\":"));
  DEBUG_SERIAL.print(missionPlaceSl);
  DEBUG_SERIAL.print(F(",\"placeTolerance\":"));
  DEBUG_SERIAL.print(missionPlaceTolerance);
  DEBUG_SERIAL.print('}');
}

void printJsonMissionQueue() {
  DEBUG_SERIAL.print(F("{\"count\":"));
  DEBUG_SERIAL.print(missionQueueCount);
  DEBUG_SERIAL.print(F(",\"currentBlockIndex\":"));
  DEBUG_SERIAL.print(missionBlockIndex);
  DEBUG_SERIAL.print(F(",\"currentPickLayer\":"));
  printJsonString(missionPickLayer);
  DEBUG_SERIAL.print(F(",\"currentPlaceSlot\":"));
  DEBUG_SERIAL.print(missionPlaceSlot);
  DEBUG_SERIAL.print(F(",\"currentStorageColumn\":"));
  DEBUG_SERIAL.print(missionStorageColumn);
  DEBUG_SERIAL.print(F(",\"targetStorageColumn\":"));
  DEBUG_SERIAL.print(missionStorageScanTargetColumn);
  DEBUG_SERIAL.print(F(",\"items\":["));
  for (uint8_t i = 0; i < missionQueueCount; i++) {
    if (i > 0) DEBUG_SERIAL.print(',');
    DEBUG_SERIAL.print(F("{\"index\":"));
    DEBUG_SERIAL.print(i + 1);
    DEBUG_SERIAL.print(F(",\"signature\":"));
    DEBUG_SERIAL.print(missionQueueSignatures[i]);
    DEBUG_SERIAL.print(F(",\"sourceSlot\":"));
    DEBUG_SERIAL.print(missionQueueSourceSlots[i]);
    DEBUG_SERIAL.print(F(",\"goalSlot\":"));
    DEBUG_SERIAL.print(missionQueueGoalSlots[i]);
    DEBUG_SERIAL.print(F(",\"completed\":"));
    printJsonBool(missionQueueCompleted[i]);
    DEBUG_SERIAL.print('}');
  }
  DEBUG_SERIAL.print(F("]}"));
}

void printJsonProfileConfig() {
  DEBUG_SERIAL.print(F("["));
  for (uint8_t i = 0; i < PROFILE_COUNT; i++) {
    if (i > 0) DEBUG_SERIAL.print(',');
    DEBUG_SERIAL.print(F("{\"name\":"));
    printJsonString(PROFILES[i].name);
    DEBUG_SERIAL.print(F(",\"minPoseMs\":"));
    DEBUG_SERIAL.print(PROFILES[i].minPoseMs);
    DEBUG_SERIAL.print(F(",\"maxPoseMs\":"));
    DEBUG_SERIAL.print(PROFILES[i].maxPoseMs);
    DEBUG_SERIAL.print(F(",\"msPerRawTick\":"));
    DEBUG_SERIAL.print(PROFILES[i].msPerRawTick, 2);
    DEBUG_SERIAL.print(F(",\"maxDriveMm\":"));
    DEBUG_SERIAL.print(PROFILES[i].maxDriveMm);
    DEBUG_SERIAL.print(F(",\"maxDriveMmPerSec\":"));
    DEBUG_SERIAL.print(PROFILES[i].maxDriveMmPerSec);
    DEBUG_SERIAL.print(F(",\"maxDriveTimeMs\":"));
    DEBUG_SERIAL.print(PROFILES[i].maxDriveTimeMs);
    DEBUG_SERIAL.print(F(",\"missionVelocityScale\":"));
    DEBUG_SERIAL.print(PROFILES[i].missionVelocityScale, 2);
    DEBUG_SERIAL.print(F(",\"missionPoseMsScale\":"));
    DEBUG_SERIAL.print(PROFILES[i].missionPoseMsScale, 2);
    DEBUG_SERIAL.print(F(",\"maxMissionVelocityRaw\":"));
    DEBUG_SERIAL.print(PROFILES[i].maxMissionVelocityRaw);
    DEBUG_SERIAL.print('}');
  }
  DEBUG_SERIAL.print(']');
}

void printJsonTunerConfig() {
  DEBUG_SERIAL.print(F("{\"pose\":{\"missionZoneStartId\":"));
  DEBUG_SERIAL.print(CFG.pose.missionZoneStartId);
  DEBUG_SERIAL.print(F(",\"missionZoneSlotCount\":"));
  DEBUG_SERIAL.print(CFG.pose.missionZoneSlotCount);
  DEBUG_SERIAL.print(F("},\"mission\":{\"blockSignatureMap\":"));
  DEBUG_SERIAL.print(CFG.mission.blockSignatureMap);
  DEBUG_SERIAL.print(F(",\"dynamicBlockCount\":"));
  printJsonBool(CFG.mission.dynamicBlockCount);
  DEBUG_SERIAL.print(F(",\"blockCount\":"));
  DEBUG_SERIAL.print(CFG.mission.blockCount);
  DEBUG_SERIAL.print(F(",\"goalPositions\":"));
  printJsonUint8Array(CFG.mission.goalPositions, MissionConfig::MAX_MISSION_BLOCKS);

  DEBUG_SERIAL.print(F("},\"psd\":{\"approachFl\":"));
  DEBUG_SERIAL.print(CFG.psd.approachFl);
  DEBUG_SERIAL.print(F(",\"approachFr\":"));
  DEBUG_SERIAL.print(CFG.psd.approachFr);
  DEBUG_SERIAL.print(F(",\"approachTolerance\":"));
  DEBUG_SERIAL.print(CFG.psd.approachTolerance);
  DEBUG_SERIAL.print(F(",\"missionSl\":"));
  DEBUG_SERIAL.print(CFG.psd.missionSl);
  DEBUG_SERIAL.print(F(",\"missionTolerance\":"));
  DEBUG_SERIAL.print(CFG.psd.missionTolerance);
  DEBUG_SERIAL.print(F(",\"alignFl\":"));
  DEBUG_SERIAL.print(CFG.psd.alignFl);
  DEBUG_SERIAL.print(F(",\"alignSl\":"));
  DEBUG_SERIAL.print(CFG.psd.alignSl);
  DEBUG_SERIAL.print(F(",\"alignFr\":"));
  DEBUG_SERIAL.print(CFG.psd.alignFr);
  DEBUG_SERIAL.print(F(",\"alignTolerance\":"));
  DEBUG_SERIAL.print(CFG.psd.alignTolerance);
  DEBUG_SERIAL.print(F(",\"gripAlignFl\":"));
  DEBUG_SERIAL.print(CFG.psd.gripAlignFl);
  DEBUG_SERIAL.print(F(",\"gripAlignSl\":"));
  DEBUG_SERIAL.print(CFG.psd.gripAlignSl);
  DEBUG_SERIAL.print(F(",\"gripAlignFr\":"));
  DEBUG_SERIAL.print(CFG.psd.gripAlignFr);
  DEBUG_SERIAL.print(F(",\"gripAlignTolerance\":"));
  DEBUG_SERIAL.print(CFG.psd.gripAlignTolerance);
  DEBUG_SERIAL.print(F(",\"lowerGripAlignFl\":"));
  DEBUG_SERIAL.print(CFG.psd.lowerGripAlignFl);
  DEBUG_SERIAL.print(F(",\"lowerGripAlignSl\":"));
  DEBUG_SERIAL.print(CFG.psd.lowerGripAlignSl);
  DEBUG_SERIAL.print(F(",\"lowerGripAlignFr\":"));
  DEBUG_SERIAL.print(CFG.psd.lowerGripAlignFr);
  DEBUG_SERIAL.print(F(",\"lowerGripAlignTolerance\":"));
  DEBUG_SERIAL.print(CFG.psd.lowerGripAlignTolerance);
  DEBUG_SERIAL.print(F(",\"missionZoneSl\":"));
  DEBUG_SERIAL.print(CFG.psd.missionZoneSl);
  DEBUG_SERIAL.print(F(",\"missionZoneFr\":"));
  DEBUG_SERIAL.print(CFG.psd.missionZoneFr);
  DEBUG_SERIAL.print(F(",\"missionZoneTolerance\":"));
  DEBUG_SERIAL.print(CFG.psd.missionZoneTolerance);
  DEBUG_SERIAL.print(F(",\"finishPreAlignSl\":"));
  DEBUG_SERIAL.print(CFG.finishReturn.preAlignSl);
  DEBUG_SERIAL.print(F(",\"finishPreAlignTolerance\":"));
  DEBUG_SERIAL.print(CFG.finishReturn.preAlignTolerance);
  DEBUG_SERIAL.print(F(",\"scanSl\":"));
  DEBUG_SERIAL.print(CFG.psd.scanSl);
  DEBUG_SERIAL.print(F(",\"scanSlTolerance\":"));
  DEBUG_SERIAL.print(CFG.psd.scanSlTolerance);
  DEBUG_SERIAL.print(F(",\"storageScanFrNoObstacle\":"));
  DEBUG_SERIAL.print(CFG.psd.storageScanFrNoObstacle);

  DEBUG_SERIAL.print(F("},\"front\":{\"firstDetectAdc\":"));
  DEBUG_SERIAL.print(CFG.front.firstDetectAdc);
  DEBUG_SERIAL.print(F(",\"decelWindowAdc\":"));
  DEBUG_SERIAL.print(CFG.front.decelWindowAdc);
  DEBUG_SERIAL.print(F(",\"cruiseSpeed\":"));
  DEBUG_SERIAL.print(CFG.front.cruiseSpeed);
  DEBUG_SERIAL.print(F(",\"slowSpeed\":"));
  DEBUG_SERIAL.print(CFG.front.slowSpeed);
  DEBUG_SERIAL.print(F(",\"brakeStep\":"));
  DEBUG_SERIAL.print(CFG.front.brakeStep);
  DEBUG_SERIAL.print(F(",\"brakeDelayMs\":"));
  DEBUG_SERIAL.print(CFG.front.brakeDelayMs);

  DEBUG_SERIAL.print(F("},\"speed\":{\"psdCorrectionSpeed\":"));
  DEBUG_SERIAL.print(CFG.speed.psdCorrectionSpeed);
  DEBUG_SERIAL.print(F(",\"frontDepthCorrectionSpeed\":"));
  DEBUG_SERIAL.print(CFG.speed.frontDepthCorrectionSpeed);
  DEBUG_SERIAL.print(F(",\"missionZonePlaceCorrectionSpeed\":"));
  DEBUG_SERIAL.print(CFG.speed.missionZonePlaceCorrectionSpeed);
  DEBUG_SERIAL.print(F(",\"cameraFineTuneSpeed\":"));
  DEBUG_SERIAL.print(CFG.speed.cameraFineTuneSpeed);
  DEBUG_SERIAL.print(F(",\"storageScanSpeed\":"));
  DEBUG_SERIAL.print(CFG.speed.storageScanSpeed);
  DEBUG_SERIAL.print(F(",\"returnSpeed\":"));
  DEBUG_SERIAL.print(CFG.speed.returnSpeed);
  DEBUG_SERIAL.print(F(",\"positionMoveMmPerSec\":"));
  DEBUG_SERIAL.print(CFG.speed.positionMoveMmPerSec);

  DEBUG_SERIAL.print(F("},\"storageDrive\":{\"firstForwardMm\":"));
  DEBUG_SERIAL.print(CFG.storageDrive.firstForwardMm, 2);
  DEBUG_SERIAL.print(F(",\"extraForwardMm\":"));
  DEBUG_SERIAL.print(CFG.storageDrive.extraForwardMm, 2);
  DEBUG_SERIAL.print(F(",\"rightMm\":"));
  DEBUG_SERIAL.print(CFG.storageDrive.rightMm, 2);

  DEBUG_SERIAL.print(F("},\"storageRack\":{\"upperRowSlots\":"));
  printJsonUint8Array(CFG.storageRack.upperRowSlots, 4);
  DEBUG_SERIAL.print(F(",\"lowerRowSlots\":"));
  printJsonUint8Array(CFG.storageRack.lowerRowSlots, 4);
  DEBUG_SERIAL.print(F(",\"pickSlotOrder\":"));
  printJsonUint8Array(CFG.storageRack.pickSlotOrder, 8);
  DEBUG_SERIAL.print(F(",\"pickSlotCount\":"));
  DEBUG_SERIAL.print(CFG.storageRack.pickSlotCount);
  DEBUG_SERIAL.print(F(",\"perSlotScanMs\":"));
  DEBUG_SERIAL.print(CFG.storageRack.perSlotScanMs);
  DEBUG_SERIAL.print(F(",\"scanColumnStepMm\":"));
  DEBUG_SERIAL.print(CFG.storageRack.scanColumnStepMm, 2);
  DEBUG_SERIAL.print(F(",\"scanColumnMoveMmPerSec\":"));
  DEBUG_SERIAL.print(CFG.storageRack.scanColumnMoveMmPerSec);
  DEBUG_SERIAL.print(F(",\"scanSettleMs\":"));
  DEBUG_SERIAL.print(CFG.storageRack.scanSettleMs);
  DEBUG_SERIAL.print(F(",\"scanFramesPerStop\":"));
  DEBUG_SERIAL.print(CFG.storageRack.scanFramesPerStop);
  DEBUG_SERIAL.print(F(",\"scanMinBlockArea\":"));
  DEBUG_SERIAL.print(CFG.storageRack.scanMinBlockArea);
  DEBUG_SERIAL.print(F(",\"columnXCenters\":"));
  printJsonInt16Array(CFG.storageRack.columnXCenters, 4);
  DEBUG_SERIAL.print(F(",\"columnXTolerance\":"));
  DEBUG_SERIAL.print(CFG.storageRack.columnXTolerance);

  DEBUG_SERIAL.print(F("},\"storagePickupRegion\":{\"xMin\":"));
  printJsonInt16Array(CFG.storagePickupRegion.xMin, MissionConfig::STORAGE_PICKUP_REGION_COUNT);
  DEBUG_SERIAL.print(F(",\"xMax\":"));
  printJsonInt16Array(CFG.storagePickupRegion.xMax, MissionConfig::STORAGE_PICKUP_REGION_COUNT);
  DEBUG_SERIAL.print(F(",\"yMin\":"));
  printJsonInt16Array(CFG.storagePickupRegion.yMin, MissionConfig::STORAGE_PICKUP_REGION_COUNT);
  DEBUG_SERIAL.print(F(",\"yMax\":"));
  printJsonInt16Array(CFG.storagePickupRegion.yMax, MissionConfig::STORAGE_PICKUP_REGION_COUNT);
  DEBUG_SERIAL.print(F(",\"xMargin\":"));
  DEBUG_SERIAL.print(CFG.storagePickupRegion.xMargin);
  DEBUG_SERIAL.print(F(",\"yMargin\":"));
  DEBUG_SERIAL.print(CFG.storagePickupRegion.yMargin);
  DEBUG_SERIAL.print(F(",\"useUpperGripPose\":"));
  printJsonUint8Array(CFG.storagePickupRegion.useUpperGripPose, MissionConfig::STORAGE_PICKUP_REGION_COUNT);

  DEBUG_SERIAL.print(F("},\"storageGripTarget\":{\"xMin\":"));
  printJsonInt16Array(CFG.storageGripTarget.xMin, MissionConfig::STORAGE_GRIP_TARGET_COUNT);
  DEBUG_SERIAL.print(F(",\"xMax\":"));
  printJsonInt16Array(CFG.storageGripTarget.xMax, MissionConfig::STORAGE_GRIP_TARGET_COUNT);
  DEBUG_SERIAL.print(F(",\"yMin\":"));
  printJsonInt16Array(CFG.storageGripTarget.yMin, MissionConfig::STORAGE_GRIP_TARGET_COUNT);
  DEBUG_SERIAL.print(F(",\"yMax\":"));
  printJsonInt16Array(CFG.storageGripTarget.yMax, MissionConfig::STORAGE_GRIP_TARGET_COUNT);
  DEBUG_SERIAL.print(F(",\"xMargin\":"));
  DEBUG_SERIAL.print(CFG.storageGripTarget.xMargin);
  DEBUG_SERIAL.print(F(",\"yMargin\":"));
  DEBUG_SERIAL.print(CFG.storageGripTarget.yMargin);
  DEBUG_SERIAL.print(F(",\"centerToleranceX\":"));
  DEBUG_SERIAL.print(CFG.storageGripTarget.centerToleranceX);
  DEBUG_SERIAL.print(F(",\"centerToleranceY\":"));
  DEBUG_SERIAL.print(CFG.storageGripTarget.centerToleranceY);
  DEBUG_SERIAL.print(F(",\"useUpperGripPose\":"));
  printJsonUint8Array(CFG.storageGripTarget.useUpperGripPose, MissionConfig::STORAGE_GRIP_TARGET_COUNT);
  DEBUG_SERIAL.print(F(",\"alignTimeoutMs\":"));
  DEBUG_SERIAL.print(CFG.storageGripTarget.alignTimeoutMs);
  DEBUG_SERIAL.print(F(",\"alignStepMs\":"));
  DEBUG_SERIAL.print(CFG.storageGripTarget.alignStepMs);
  DEBUG_SERIAL.print(F(",\"alignSettleMs\":"));
  DEBUG_SERIAL.print(CFG.storageGripTarget.alignSettleMs);
  DEBUG_SERIAL.print(F(",\"yErrorUsesForwardDirection\":"));
  printJsonBool(CFG.storageGripTarget.yErrorUsesForwardDirection);

  DEBUG_SERIAL.print(F("},\"cameraScan\":{\"maxSignature\":"));
  DEBUG_SERIAL.print(CFG.cameraScan.maxSignature);
  DEBUG_SERIAL.print(F(",\"missionInstructionAllowedSignatureMap\":"));
  DEBUG_SERIAL.print(CFG.cameraScan.missionInstructionAllowedSignatureMap);
  DEBUG_SERIAL.print(F(",\"storageAllowedSignatureMap\":"));
  DEBUG_SERIAL.print(CFG.cameraScan.storageAllowedSignatureMap);
  DEBUG_SERIAL.print(F(",\"missionInstructionLampOn\":"));
  printJsonBool(CFG.cameraScan.missionInstructionLampOn);
  DEBUG_SERIAL.print(F(",\"storageXSetpoint\":"));
  DEBUG_SERIAL.print(CFG.cameraScan.storageXSetpoint);
  DEBUG_SERIAL.print(F(",\"storageXTolerance\":"));
  DEBUG_SERIAL.print(CFG.cameraScan.storageXTolerance);
  DEBUG_SERIAL.print(F(",\"storageBoundaryXMin\":"));
  DEBUG_SERIAL.print(CFG.cameraScan.storageBoundaryXMin);
  DEBUG_SERIAL.print(F(",\"storageBoundaryXMax\":"));
  DEBUG_SERIAL.print(CFG.cameraScan.storageBoundaryXMax);
  DEBUG_SERIAL.print(F(",\"storageBoundaryYMin\":"));
  DEBUG_SERIAL.print(CFG.cameraScan.storageBoundaryYMin);
  DEBUG_SERIAL.print(F(",\"storageBoundaryYMax\":"));
  DEBUG_SERIAL.print(CFG.cameraScan.storageBoundaryYMax);
  DEBUG_SERIAL.print(F(",\"storageBoundaryMargin\":"));
  DEBUG_SERIAL.print(CFG.cameraScan.storageBoundaryMargin);
  DEBUG_SERIAL.print(F(",\"storageYUpperLowerSplit\":"));
  DEBUG_SERIAL.print(CFG.cameraScan.storageYUpperLowerSplit);
  DEBUG_SERIAL.print(F(",\"missionInstructionMinBlockArea\":"));
  DEBUG_SERIAL.print(CFG.cameraScan.missionInstructionMinBlockArea);
  DEBUG_SERIAL.print(F(",\"storageMinBlockArea\":"));
  DEBUG_SERIAL.print(CFG.cameraScan.storageMinBlockArea);

  DEBUG_SERIAL.print(F("},\"timeout\":{\"psdLoopMs\":"));
  DEBUG_SERIAL.print(CFG.timeout.psdLoopMs);
  DEBUG_SERIAL.print(F(",\"positionMoveMs\":"));
  DEBUG_SERIAL.print(CFG.timeout.positionMoveMs);
  DEBUG_SERIAL.print(F(",\"scanPhaseMs\":"));
  DEBUG_SERIAL.print(CFG.timeout.scanPhaseMs);
  DEBUG_SERIAL.print(F(",\"fineTuneMs\":"));
  DEBUG_SERIAL.print(CFG.timeout.fineTuneMs);
  DEBUG_SERIAL.print(F(",\"returnPhaseMs\":"));
  DEBUG_SERIAL.print(CFG.timeout.returnPhaseMs);
  DEBUG_SERIAL.print(F(",\"missionTimeLimitMs\":"));
  DEBUG_SERIAL.print(CFG.timeout.missionTimeLimitMs);

  DEBUG_SERIAL.print(F("},\"wait\":{\"buttonDebounceMs\":"));
  DEBUG_SERIAL.print(CFG.wait.buttonDebounceMs);
  DEBUG_SERIAL.print(F(",\"driveSettleMs\":"));
  DEBUG_SERIAL.print(CFG.wait.driveSettleMs);
  DEBUG_SERIAL.print(F(",\"blockFeedbackMs\":"));
  DEBUG_SERIAL.print(CFG.wait.blockFeedbackMs);
  DEBUG_SERIAL.print(F(",\"scanSampleMs\":"));
  DEBUG_SERIAL.print(CFG.wait.scanSampleMs);
  DEBUG_SERIAL.print(F(",\"cameraLampMs\":"));
  DEBUG_SERIAL.print(CFG.wait.cameraLampMs);
  DEBUG_SERIAL.print(F(",\"gripperActionMs\":"));
  DEBUG_SERIAL.print(CFG.wait.gripperActionMs);

  DEBUG_SERIAL.print(F("},\"finishReturn\":{\"boundaryAdc\":"));
  DEBUG_SERIAL.print(CFG.finishReturn.boundaryAdc);
  DEBUG_SERIAL.print(F(",\"preAlignSl\":"));
  DEBUG_SERIAL.print(CFG.finishReturn.preAlignSl);
  DEBUG_SERIAL.print(F(",\"preAlignTolerance\":"));
  DEBUG_SERIAL.print(CFG.finishReturn.preAlignTolerance);
  DEBUG_SERIAL.print(F(",\"preAlignSpeed\":"));
  DEBUG_SERIAL.print(CFG.finishReturn.preAlignSpeed);
  DEBUG_SERIAL.print(F(",\"trackSl\":"));
  DEBUG_SERIAL.print(CFG.finishReturn.trackSl);
  DEBUG_SERIAL.print(F(",\"trackTolerance\":"));
  DEBUG_SERIAL.print(CFG.finishReturn.trackTolerance);
  DEBUG_SERIAL.print(F(",\"correctionMaxSpeed\":"));
  DEBUG_SERIAL.print(CFG.finishReturn.correctionMaxSpeed);
  DEBUG_SERIAL.print(F(",\"openSideLeftBiasSpeed\":"));
  DEBUG_SERIAL.print(CFG.finishReturn.openSideLeftBiasSpeed);
  DEBUG_SERIAL.print(F(",\"wheelMaxSpeed\":"));
  DEBUG_SERIAL.print(CFG.finishReturn.wheelMaxSpeed);
  DEBUG_SERIAL.print(F(",\"finishExtraMs\":"));
  DEBUG_SERIAL.print(CFG.finishReturn.finishExtraMs);
  DEBUG_SERIAL.print(F("}}"));
}

void printJsonPixySnapshot() {
  DEBUG_SERIAL.print(F("{\"ready\":"));
  printJsonBool(pixyReady);
  if (pixyReady) {
    pixy.ccc.getBlocks(true, 0x7F);
    DEBUG_SERIAL.print(F(",\"numBlocks\":"));
    DEBUG_SERIAL.print(pixy.ccc.numBlocks);
    DEBUG_SERIAL.print(F(",\"blocks\":["));
    for (uint8_t i = 0; i < pixy.ccc.numBlocks; i++) {
      if (i > 0) DEBUG_SERIAL.print(',');
      uint32_t area = (uint32_t)pixy.ccc.blocks[i].m_width *
                      (uint32_t)pixy.ccc.blocks[i].m_height;
      DEBUG_SERIAL.print(F("{\"signature\":"));
      DEBUG_SERIAL.print(pixy.ccc.blocks[i].m_signature);
      DEBUG_SERIAL.print(F(",\"x\":"));
      DEBUG_SERIAL.print(pixy.ccc.blocks[i].m_x);
      DEBUG_SERIAL.print(F(",\"y\":"));
      DEBUG_SERIAL.print(pixy.ccc.blocks[i].m_y);
      DEBUG_SERIAL.print(F(",\"width\":"));
      DEBUG_SERIAL.print(pixy.ccc.blocks[i].m_width);
      DEBUG_SERIAL.print(F(",\"height\":"));
      DEBUG_SERIAL.print(pixy.ccc.blocks[i].m_height);
      DEBUG_SERIAL.print(F(",\"area\":"));
      DEBUG_SERIAL.print(area);
      DEBUG_SERIAL.print('}');
    }
    DEBUG_SERIAL.print(']');
  }
  DEBUG_SERIAL.print('}');
}

bool commandExportJson() {
  DEBUG_SERIAL.print(F("{\"type\":\"mission-route-tuner-export\",\"schemaVersion\":1"));
  DEBUG_SERIAL.print(F(",\"source\":\"MissionRouteTuner\""));
  DEBUG_SERIAL.print(F(",\"serial\":{\"usbBaud\":"));
  DEBUG_SERIAL.print(SERIAL_BAUD_RATE);
  DEBUG_SERIAL.print(F(",\"bluetoothEnabled\":"));
#if ENABLE_BLUETOOTH_SERIAL
  printJsonBool(true);
  DEBUG_SERIAL.print(F(",\"bluetoothPort\":\"Serial2\",\"bluetoothBaud\":"));
  DEBUG_SERIAL.print(BLUETOOTH_BAUD_RATE);
  DEBUG_SERIAL.print(F(",\"bluetoothOutputEnabled\":"));
#if ENABLE_BLUETOOTH_SERIAL_OUTPUT
  printJsonBool(true);
#else
  printJsonBool(false);
#endif
#else
  printJsonBool(false);
#endif
  DEBUG_SERIAL.print(F("}"));

  DEBUG_SERIAL.print(F(",\"status\":{\"profile\":"));
  printJsonString(profile().name);
  DEBUG_SERIAL.print(F(",\"manipulatorReady\":"));
  printJsonBool(manipulatorReady);
  DEBUG_SERIAL.print(F(",\"mobileReady\":"));
  printJsonBool(mobileReady);
  DEBUG_SERIAL.print(F(",\"pixyReady\":"));
  printJsonBool(pixyReady);
  DEBUG_SERIAL.print(F(",\"missionStage\":\""));
  DEBUG_SERIAL.print(missionStageName(missionStage));
  DEBUG_SERIAL.print(F("\",\"missionButtonMode\":"));
  printJsonBool(missionButtonMode);
  DEBUG_SERIAL.print(F(",\"lastReplay\":"));
  printJsonString(lastReplayableCommand);
  DEBUG_SERIAL.print('}');

  DEBUG_SERIAL.print(F(",\"poseContract\":["));
  DEBUG_SERIAL.print(F("{\"id\":1,\"role\":\"initial-mission-instruction\"},"));
  DEBUG_SERIAL.print(F("{\"id\":2,\"role\":\"reserved-manual\"},"));
  DEBUG_SERIAL.print(F("{\"id\":3,\"role\":\"storage-view-safe\"},"));
  DEBUG_SERIAL.print(F("{\"id\":4,\"role\":\"grip-upper\"},"));
  DEBUG_SERIAL.print(F("{\"id\":5,\"role\":\"grip-lower\"},"));
  DEBUG_SERIAL.print(F("{\"id\":6,\"role\":\"reserved-manual\"},"));
  DEBUG_SERIAL.print(F("{\"range\":\"7-14\",\"role\":\"mission-zone-slots-1-8\"}]"));
  DEBUG_SERIAL.print(F(",\"requiredPoseIds\":"));
  printJsonRequiredPoseIds();

  DEBUG_SERIAL.print(F(",\"eepromPoses\":["));
  for (uint8_t id = 1; id <= 14; id++) {
    if (id > 1) DEBUG_SERIAL.print(',');
    ManipulatorPose pose = ReadManipulatorPresentPoseToEEPROM(id);
    printJsonPoseObject(id, pose);
  }
  DEBUG_SERIAL.print(']');

  DEBUG_SERIAL.print(F(",\"currentPose\":"));
  if (manipulatorReady) {
    ManipulatorPose presentPose = readCurrentManipulatorPoseSnapshot();
    printJsonPoseSnapshot(presentPose);
  } else {
    DEBUG_SERIAL.print(F("null"));
  }

  DEBUG_SERIAL.print(F(",\"pendingPose\":"));
  if (hasPendingPoseWrite) {
    printJsonPoseObject(pendingPoseWrite.id, pendingPoseWrite);
  } else {
    DEBUG_SERIAL.print(F("null"));
  }

  DEBUG_SERIAL.print(F(",\"lastTunedPose\":"));
  if (hasLastTunedPose) {
    printJsonPoseObject(lastTunedPose.id, lastTunedPose);
  } else {
    DEBUG_SERIAL.print(F("null"));
  }

  DEBUG_SERIAL.print(F(",\"missionMotion\":"));
  printJsonMissionMotion();
  PsdSnapshot psdSnapshot;
  readAllPSDSensors(&psdSnapshot);
  DEBUG_SERIAL.print(F(",\"psdSnapshot\":"));
  printJsonPsdSnapshot(psdSnapshot);
  DEBUG_SERIAL.print(F(",\"tunerCalibration\":"));
  printJsonTunerCalibration();
  DEBUG_SERIAL.print(F(",\"missionQueue\":"));
  printJsonMissionQueue();
  DEBUG_SERIAL.print(F(",\"missionColumnDecision\":"));
  if (missionColumnScanHasDecision) {
    DEBUG_SERIAL.print(F("{\"sig\":"));
    DEBUG_SERIAL.print(missionDetectedSignature);
    DEBUG_SERIAL.print(F(",\"sourceSlot\":"));
    DEBUG_SERIAL.print(missionDetectedSourceSlot);
    DEBUG_SERIAL.print(F(",\"goalSlot\":"));
    DEBUG_SERIAL.print(missionDetectedGoalSlot);
    DEBUG_SERIAL.print(F(",\"pickLayer\":"));
    printJsonString(missionPickLayer);
    DEBUG_SERIAL.print(F(",\"pickupRegion\":"));
    DEBUG_SERIAL.print(missionDetectedPickupRegion);
    DEBUG_SERIAL.print(F(",\"pixyColumn\":"));
    DEBUG_SERIAL.print(missionDetectedPixyColumn);
    DEBUG_SERIAL.print(F(",\"x\":"));
    DEBUG_SERIAL.print(missionDetectedX);
    DEBUG_SERIAL.print(F(",\"y\":"));
    DEBUG_SERIAL.print(missionDetectedY);
    DEBUG_SERIAL.print(F(",\"area\":"));
    DEBUG_SERIAL.print(missionDetectedArea);
    DEBUG_SERIAL.print(F("}"));
  } else {
    DEBUG_SERIAL.print(F("null"));
  }
  DEBUG_SERIAL.print(F(",\"profiles\":"));
  printJsonProfileConfig();
  DEBUG_SERIAL.print(F(",\"config\":"));
  printJsonTunerConfig();
  DEBUG_SERIAL.print(F(",\"pixySnapshot\":"));
  printJsonPixySnapshot();
  DEBUG_SERIAL.println(F("}"));
  return true;
}

bool commandExport(const String &input) {
  String kind = tokenAt(input, 1);
  kind.toLowerCase();
  if (kind.length() == 0 || kind == "json" || kind == "all" || kind == "전체") {
    return commandExportJson();
  }
  DEBUG_SERIAL.println(F("사용법: export json"));
  DEBUG_SERIAL.println(F("  출력된 한 줄 JSON을 Mac에서 Motor/ops/local-tuning-runs.jsonl에 저장하세요."));
  return false;
}

bool commandPosePlan(const String &input) {
  uint8_t poseId = 0;
  uint16_t timeMs = 0;
  if (!parsePoseId(tokenAt(input, 2), &poseId, false) ||
      !parsePoseMs(tokenAt(input, 3), &timeMs)) {
    DEBUG_SERIAL.println(F("사용법: pose plan <id> <ms> [m1angle]"));
    return false;
  }
  if (!ensureManipulatorReady()) return false;

  ManipulatorPose targetPose = ReadManipulatorPresentPoseToEEPROM(poseId);
  if (!targetPose.isTherePoseData) {
    DEBUG_SERIAL.print(F("[자세 없음] EEPROM "));
    DEBUG_SERIAL.print(poseId);
    DEBUG_SERIAL.println(F("번 자세가 없습니다."));
    return false;
  }

  float motor1Angle = -360.0;
  if (tokenCount(input) >= 5) {
    long angle = 0;
    if (!parseLongStrict(tokenAt(input, 4), &angle)) {
      DEBUG_SERIAL.println(F("[오류] m1angle은 정수 각도로 입력하세요. 예: -90"));
      return false;
    }
    motor1Angle = (float)constrain(angle, -100, 100);
    targetPose.manipulatorMotor1Value = motor1RawFromAngle(motor1Angle);
  }

  ManipulatorPose presentPose = readCurrentManipulatorPoseSnapshot();
  int16_t maxRawDelta = maxRawDeltaBetweenPoses(presentPose, targetPose);
  uint16_t safeMs = safePoseTimeFromDelta(timeMs, maxRawDelta);

  printPoseValues(F("[현재 자세]"), presentPose);
  printPoseValues(F("[목표 자세]"), targetPose);
  printPoseDelta(F("[예상 변화량] 목표 - 현재"), presentPose, targetPose);
  DEBUG_SERIAL.print(F("  요청 ms=")); DEBUG_SERIAL.print(timeMs);
  DEBUG_SERIAL.print(F(", 실행 예정 ms=")); DEBUG_SERIAL.print(safeMs);
  DEBUG_SERIAL.print(F(", maxRawDelta=")); DEBUG_SERIAL.println(maxRawDelta);
  DEBUG_SERIAL.println(F("[미실행] 실제 모터는 움직이지 않았습니다."));
  return true;
}

bool commandPoseTunePlan(const String &input) {
  uint8_t poseId = 0;
  uint16_t timeMs = 0;
  if (!parsePoseId(tokenAt(input, 2), &poseId, false) ||
      !parsePoseMs(tokenAt(input, 3), &timeMs) ||
      tokenCount(input) < 8) {
    DEBUG_SERIAL.println(F("사용법: pose tuneplan <id> <ms> <m1> <m2> <m3> <m4>"));
    DEBUG_SERIAL.println(F("예시: pose tuneplan 3 300 0 +30 -30 0"));
    return false;
  }
  if (!ensureManipulatorReady()) return false;

  ManipulatorPose basePose = ReadManipulatorPresentPoseToEEPROM(poseId);
  if (!basePose.isTherePoseData) {
    DEBUG_SERIAL.println(F("[자세 없음] 먼저 EEPROM에 저장된 자세가 필요합니다."));
    return false;
  }

  ManipulatorPose tunedPose = basePose;
  if (!parseMotorTuneValue(tokenAt(input, 4), basePose.manipulatorMotor1Value,
                           ARM_DXL_1_POSITION_MIN, ARM_DXL_1_POSITION_MAX,
                           &tunedPose.manipulatorMotor1Value)) return false;
  if (!parseMotorTuneValue(tokenAt(input, 5), basePose.manipulatorMotor2Value,
                           ARM_DXL_2_POSITION_MIN, ARM_DXL_2_POSITION_MAX,
                           &tunedPose.manipulatorMotor2Value)) return false;
  if (!parseMotorTuneValue(tokenAt(input, 6), basePose.manipulatorMotor3Value,
                           ARM_DXL_3_POSITION_MIN, ARM_DXL_3_POSITION_MAX,
                           &tunedPose.manipulatorMotor3Value)) return false;
  if (!parseMotorTuneValue(tokenAt(input, 7), basePose.manipulatorMotor4Value,
                           ARM_DXL_4_POSITION_MIN, ARM_DXL_4_POSITION_MAX,
                           &tunedPose.manipulatorMotor4Value)) return false;

  ManipulatorPose presentPose = readCurrentManipulatorPoseSnapshot();
  int16_t maxEepromDelta = maxRawDeltaBetweenPoses(basePose, tunedPose);
  int16_t maxStepDelta = maxRawDeltaBetweenPoses(presentPose, tunedPose);
  uint16_t safeMs = safePoseTimeFromDelta(timeMs, maxStepDelta);
  printPoseValues(F("[EEPROM 기준 자세]"), basePose);
  printPoseValues(F("[현재 자세]"), presentPose);
  printPoseValues(F("[튜닝 목표 자세]"), tunedPose);
  printPoseDelta(F("[예상 변화량] 목표 - EEPROM"), basePose, tunedPose);
  printPoseDelta(F("[예상 1회 이동량] 목표 - 현재"), presentPose, tunedPose);
  DEBUG_SERIAL.print(F("  요청 ms=")); DEBUG_SERIAL.print(timeMs);
  DEBUG_SERIAL.print(F(", 실행 예정 ms=")); DEBUG_SERIAL.print(safeMs);
  DEBUG_SERIAL.print(F(", stepMaxRawDelta=")); DEBUG_SERIAL.print(maxStepDelta);
  DEBUG_SERIAL.print(F(", eepromMaxRawDelta=")); DEBUG_SERIAL.println(maxEepromDelta);
  DEBUG_SERIAL.println(F("[미실행] 실제 모터는 움직이지 않았습니다. 실행은 pose tune을 사용하세요."));
  return true;
}

bool commandSeq(const String &input) {
  String name = tokenAt(input, 1);
  name.toLowerCase();

  if (name == "initial" || name == "init" || name == "초기") {
    return runPoseRequired(POSE_INITIAL, missionMotion.initialPoseMs);
  }
  if (name == "storage" || name == "보관") {
    return runPoseRequired(POSE_STORAGE, missionMotion.storagePoseMs, 0.0);
  }
  if (name == "camera" || name == "view" || name == "카메라") {
    return runPoseRequired(POSE_STORAGE, missionMotion.preGripPoseMs, 0.0);
  }
  if (name == "pick") {
    String layer = tokenAt(input, 2);
    layer.toLowerCase();
    if (layer == "upper" || layer == "상층") {
      if (!runPoseRequired(POSE_STORAGE, missionMotion.preGripPoseMs, 0.0)) return false;
      uint16_t gripStageMs = max(MISSION_STAGED_POSE_MIN_MS, (uint16_t)(missionMotion.gripPoseMs / 3));
      uint16_t liftStageMs = max(MISSION_STAGED_POSE_MIN_MS, (uint16_t)(missionMotion.returnPoseMs / 3));
      if (!runStagedPoseM4M3M2(POSE_GRIP_UPPER, gripStageMs, 0.0)) return false;
      CloseGripper(pixy);
      if (!interruptibleDelay(missionMotion.gripHoldMs)) return false;
      return runStagedPoseM4M3M2(POSE_STORAGE, liftStageMs, 0.0);
    }
    if (layer == "lower" || layer == "하층") {
      if (!runPoseRequired(POSE_STORAGE, missionMotion.preGripPoseMs, 0.0)) return false;
      if (!runPoseRequired(POSE_GRIP_LOWER, missionMotion.gripPoseMs, 0.0)) return false;
      CloseGripper(pixy);
      if (!interruptibleDelay(missionMotion.gripHoldMs)) return false;
      return runPoseRequired(POSE_STORAGE, missionMotion.returnPoseMs, 0.0);
    }
  }
  if (name == "placeall" || name == "전체배치" || name == "배치전체") {
    long startSlot = 1;
    long endSlot = CFG.pose.missionZoneSlotCount;
    if (tokenCount(input) >= 3 && !parseLongStrict(tokenAt(input, 2), &startSlot)) {
      DEBUG_SERIAL.println(F("사용법: seq placeall [startSlot] [endSlot]"));
      return false;
    }
    if (tokenCount(input) >= 4 && !parseLongStrict(tokenAt(input, 3), &endSlot)) {
      DEBUG_SERIAL.println(F("사용법: seq placeall [startSlot] [endSlot]"));
      return false;
    }
    if (startSlot < 1 || startSlot > CFG.pose.missionZoneSlotCount ||
        endSlot < 1 || endSlot > CFG.pose.missionZoneSlotCount ||
        startSlot > endSlot) {
      DEBUG_SERIAL.print(F("[오류] slot 범위는 1~"));
      DEBUG_SERIAL.print(CFG.pose.missionZoneSlotCount);
      DEBUG_SERIAL.println(F("이고 start <= end 이어야 합니다."));
      return false;
    }

    DEBUG_SERIAL.println(F("[seq placeall] 미션수행존 자세 일괄 확인"));
    DEBUG_SERIAL.println(F("  안전을 위해 그리퍼 open/close는 실행하지 않습니다."));
    for (uint8_t slot = (uint8_t)startSlot; slot <= (uint8_t)endSlot; slot++) {
      DEBUG_SERIAL.print(F("  slot "));
      DEBUG_SERIAL.println(slot);
      uint8_t placePoseId = POSE_MISSION_ZONE_START + slot;
      if (!runPoseRequired(POSE_STORAGE, missionMotion.storagePoseMs, -90.0)) return false;
      if (!runPoseRequired(placePoseId, missionMotion.placePoseMs)) return false;
      if (!interruptibleDelay(MISSION_ACTUATOR_MS)) return false;
      if (!runPoseRequired(POSE_STORAGE, missionMotion.returnPoseMs, 0.0)) return false;
    }
    return true;
  }

  if (name == "place" || name == "배치") {
    long slot = 0;
    if (!parseLongStrict(tokenAt(input, 2), &slot) ||
        slot < 1 || slot > CFG.pose.missionZoneSlotCount) {
      DEBUG_SERIAL.print(F("사용법: seq place <1~"));
      DEBUG_SERIAL.print(CFG.pose.missionZoneSlotCount);
      DEBUG_SERIAL.println(F(">"));
      return false;
    }
    uint8_t placePoseId = POSE_MISSION_ZONE_START + (uint8_t)slot;
    if (!runPoseRequired(POSE_STORAGE, missionMotion.storagePoseMs, -90.0)) return false;
    if (!runPoseRequired(placePoseId, missionMotion.placePoseMs)) return false;
    OpenGripper(pixy);
    if (!interruptibleDelay(missionMotion.placeHoldMs)) return false;
    return runPoseRequired(POSE_STORAGE, missionMotion.returnPoseMs, 0.0);
  }

  DEBUG_SERIAL.println(F("사용법: seq initial | seq storage | seq camera | seq pick upper/lower | seq place <slot> | seq placeall"));
  return false;
}

bool commandDrive(const String &input) {
  String directionText = tokenAt(input, 1);
  directionText.toLowerCase();
  if (directionText == "trim" || directionText == "보정") {
    return commandDriveTrim(input);
  }
  if (directionText == "balance" || directionText == "wheeltrim" ||
      directionText == "velocitytrim" || directionText == "속도보정" ||
      directionText == "좌우보정") {
    return commandDriveBalance(input);
  }

  if (!ensureMobileReady()) return false;

  long distanceMm = 0;
  long speedMmPerSec = 0;
  if (!parseLongStrict(tokenAt(input, 2), &distanceMm) ||
      !parseLongStrict(tokenAt(input, 3), &speedMmPerSec)) {
    DEBUG_SERIAL.println(F("사용법: drive forward|back|left|right <mm> <mmPerSec>"));
    DEBUG_SERIAL.println(F("또는: drive trim <방향> <mm> <mmPerSec> <leftScale> <rightScale>"));
    DEBUG_SERIAL.println(F("또는: drive balance set/test ..."));
    return false;
  }

  uint8_t direction = DRIVE_DIRECTION_FORWARD;
  if (!parseDriveDirection(directionText, &direction)) {
    DEBUG_SERIAL.println(F("[오류] 방향은 forward/back/left/right 중 하나입니다."));
    return false;
  }

  if (!checkDriveLimits(distanceMm, speedMmPerSec)) return false;
  unsigned long estimatedMs = (unsigned long)distanceMm * 1000UL / (unsigned long)speedMmPerSec;

  ChangeMobilebaseMode2ExtendedPositionControlWithTimeBasedProfileMode(dxl);
  DriveDistanceAndMmPerSecAndDirection(dxl, (float)distanceMm, direction, speedMmPerSec);
  return waitForDriveCompletion(estimatedMs);
}

bool commandDelayTest(const String &input) {
  uint8_t fromPose = 0;
  uint8_t toPose = 0;
  uint16_t timeMs = 0;
  long repeat = 1;
  if (!parsePoseId(tokenAt(input, 1), &fromPose, true) ||
      !parsePoseId(tokenAt(input, 2), &toPose, true) ||
      !parsePoseMs(tokenAt(input, 3), &timeMs)) {
    DEBUG_SERIAL.println(F("사용법: delaytest <fromPose> <toPose> <ms> [repeat]"));
    return false;
  }
  if (tokenCount(input) >= 5) {
    if (!parseLongStrict(tokenAt(input, 4), &repeat)) repeat = 1;
  }
  repeat = constrain(repeat, 1, 5);

  for (uint8_t i = 0; i < repeat; i++) {
    DEBUG_SERIAL.print(F("[delaytest] 반복 "));
    DEBUG_SERIAL.print(i + 1);
    DEBUG_SERIAL.print(F("/"));
    DEBUG_SERIAL.println(repeat);
    if (!runPoseRequired(fromPose, timeMs)) return false;
    if (!runPoseRequired(toPose, timeMs)) return false;
  }
  return true;
}

void printSignatureMap(uint8_t signatureMap) {
  DEBUG_SERIAL.print(F("0b"));
  for (int bit = 6; bit >= 0; bit--) {
    DEBUG_SERIAL.print((signatureMap >> bit) & 0x01);
  }
}

void printPixyBlock(uint8_t index) {
  uint32_t area = (uint32_t)pixy.ccc.blocks[index].m_width *
                  (uint32_t)pixy.ccc.blocks[index].m_height;
  DEBUG_SERIAL.print(F("  #"));
  DEBUG_SERIAL.print(index);
  DEBUG_SERIAL.print(F(" sig="));
  DEBUG_SERIAL.print(pixy.ccc.blocks[index].m_signature);
  DEBUG_SERIAL.print(F(", x="));
  DEBUG_SERIAL.print(pixy.ccc.blocks[index].m_x);
  DEBUG_SERIAL.print(F(", y="));
  DEBUG_SERIAL.print(pixy.ccc.blocks[index].m_y);
  DEBUG_SERIAL.print(F(", w="));
  DEBUG_SERIAL.print(pixy.ccc.blocks[index].m_width);
  DEBUG_SERIAL.print(F(", h="));
  DEBUG_SERIAL.print(pixy.ccc.blocks[index].m_height);
  DEBUG_SERIAL.print(F(", area="));
  DEBUG_SERIAL.println(area);
}

uint32_t pixyBlockArea(uint8_t index) {
  return (uint32_t)pixy.ccc.blocks[index].m_width *
         (uint32_t)pixy.ccc.blocks[index].m_height;
}

uint8_t storageColumnForPixyX(int16_t blockX) {
  uint8_t bestColumn = 0;
  int16_t bestError = 32767;
  for (uint8_t i = 0; i < 4; i++) {
    int16_t error = abs(blockX - CFG.storageRack.columnXCenters[i]);
    if (error < bestError) {
      bestError = error;
      bestColumn = i + 1;
    }
  }

  if (bestError > CFG.storageRack.columnXTolerance) return 0;
  return bestColumn;
}

bool storagePixyInRackBoundary(int16_t blockX, int16_t blockY) {
  int16_t margin = CFG.cameraScan.storageBoundaryMargin;
  return blockX >= CFG.cameraScan.storageBoundaryXMin - margin &&
         blockX <= CFG.cameraScan.storageBoundaryXMax + margin &&
         blockY >= CFG.cameraScan.storageBoundaryYMin - margin &&
         blockY <= CFG.cameraScan.storageBoundaryYMax + margin;
}

uint8_t storageSlotForPixyXY(int16_t blockX, int16_t blockY) {
  if (!storagePixyInRackBoundary(blockX, blockY)) return 0;
  uint8_t column = storageColumnForPixyX(blockX);
  if (column == 0) return 0;
  return (blockY < CFG.cameraScan.storageYUpperLowerSplit)
             ? CFG.storageRack.upperRowSlots[column - 1]
             : CFG.storageRack.lowerRowSlots[column - 1];
}

uint8_t storagePickupRegionForPixyXY(int16_t blockX, int16_t blockY) {
  if (!storagePixyInRackBoundary(blockX, blockY)) return 0;
  for (uint8_t i = 0; i < MissionConfig::STORAGE_PICKUP_REGION_COUNT; i++) {
    int16_t xMin = CFG.storagePickupRegion.xMin[i] - CFG.storagePickupRegion.xMargin;
    int16_t xMax = CFG.storagePickupRegion.xMax[i] + CFG.storagePickupRegion.xMargin;
    int16_t yMin = CFG.storagePickupRegion.yMin[i] - CFG.storagePickupRegion.yMargin;
    int16_t yMax = CFG.storagePickupRegion.yMax[i] + CFG.storagePickupRegion.yMargin;
    if (blockX >= xMin && blockX <= xMax && blockY >= yMin && blockY <= yMax) {
      return i + 1;
    }
  }
  return 0;
}

bool storagePickupRegionUsesUpperGrip(uint8_t regionId) {
  if (regionId < 1 || regionId > MissionConfig::STORAGE_PICKUP_REGION_COUNT) return false;
  return CFG.storagePickupRegion.useUpperGripPose[regionId - 1] != 0;
}

const __FlashStringHelper *storagePickupRegionName(uint8_t regionId) {
  switch (regionId) {
    case 1: return F("upper");
    case 2: return F("lower");
  }
  return F("none");
}

uint8_t storageGripTargetBit(uint8_t targetId) {
  if (targetId < 1 || targetId > MissionConfig::STORAGE_GRIP_TARGET_COUNT) return 0;
  return (uint8_t)1 << (targetId - 1);
}

uint8_t storageGripTargetMaskForFilter(const String &rowFilter) {
  uint8_t mask = 0;
  bool allowUpper = rowFilter == "upper" || rowFilter == "all";
  bool allowLower = rowFilter == "lower" || rowFilter == "all";
  for (uint8_t i = 0; i < MissionConfig::STORAGE_GRIP_TARGET_COUNT; i++) {
    bool targetUsesUpper = CFG.storageGripTarget.useUpperGripPose[i] != 0;
    if ((targetUsesUpper && allowUpper) || (!targetUsesUpper && allowLower)) {
      mask |= storageGripTargetBit(i + 1);
    }
  }
  return mask;
}

bool storageGripTargetUsesUpperGrip(uint8_t targetId) {
  if (targetId < 1 || targetId > MissionConfig::STORAGE_GRIP_TARGET_COUNT) return false;
  return CFG.storageGripTarget.useUpperGripPose[targetId - 1] != 0;
}

bool storageGripTargetContainsPixyXY(uint8_t targetId, int16_t blockX, int16_t blockY) {
  if (targetId < 1 || targetId > MissionConfig::STORAGE_GRIP_TARGET_COUNT) return false;
  uint8_t i = targetId - 1;
  int16_t xMin = CFG.storageGripTarget.xMin[i] - CFG.storageGripTarget.xMargin;
  int16_t xMax = CFG.storageGripTarget.xMax[i] + CFG.storageGripTarget.xMargin;
  int16_t yMin = CFG.storageGripTarget.yMin[i] - CFG.storageGripTarget.yMargin;
  int16_t yMax = CFG.storageGripTarget.yMax[i] + CFG.storageGripTarget.yMargin;
  return blockX >= xMin && blockX <= xMax && blockY >= yMin && blockY <= yMax;
}

uint8_t storageGripTargetForPixyXY(uint8_t allowedTargetMask, int16_t blockX, int16_t blockY) {
  for (uint8_t targetId = 1; targetId <= MissionConfig::STORAGE_GRIP_TARGET_COUNT; targetId++) {
    if ((allowedTargetMask & storageGripTargetBit(targetId)) == 0) continue;
    if (storageGripTargetContainsPixyXY(targetId, blockX, blockY)) return targetId;
  }
  return 0;
}

int16_t storageGripTargetCenterX(uint8_t targetId) {
  uint8_t i = targetId - 1;
  return (CFG.storageGripTarget.xMin[i] + CFG.storageGripTarget.xMax[i]) / 2;
}

int16_t storageGripTargetCenterY(uint8_t targetId) {
  uint8_t i = targetId - 1;
  return (CFG.storageGripTarget.yMin[i] + CFG.storageGripTarget.yMax[i]) / 2;
}

uint8_t closestStorageGripTargetForPixyXY(uint8_t allowedTargetMask, int16_t blockX, int16_t blockY) {
  uint8_t bestTarget = 0;
  int32_t bestScore = 2147483647L;
  for (uint8_t targetId = 1; targetId <= MissionConfig::STORAGE_GRIP_TARGET_COUNT; targetId++) {
    if ((allowedTargetMask & storageGripTargetBit(targetId)) == 0) continue;
    int32_t dx = blockX - storageGripTargetCenterX(targetId);
    int32_t dy = blockY - storageGripTargetCenterY(targetId);
    int32_t score = dx * dx + dy * dy;
    if (score < bestScore) {
      bestScore = score;
      bestTarget = targetId;
    }
  }
  return bestTarget;
}

const __FlashStringHelper *storageGripTargetName(uint8_t targetId) {
  switch (targetId) {
    case 1: return F("upper-center");
    case 2: return F("lower-center");
  }
  return F("none");
}

void printStorageRackCalibrationInfo() {
  DEBUG_SERIAL.print(F("  rack boundary x="));
  DEBUG_SERIAL.print(CFG.cameraScan.storageBoundaryXMin);
  DEBUG_SERIAL.print(F("~"));
  DEBUG_SERIAL.print(CFG.cameraScan.storageBoundaryXMax);
  DEBUG_SERIAL.print(F(", y="));
  DEBUG_SERIAL.print(CFG.cameraScan.storageBoundaryYMin);
  DEBUG_SERIAL.print(F("~"));
  DEBUG_SERIAL.print(CFG.cameraScan.storageBoundaryYMax);
  DEBUG_SERIAL.print(F(", margin="));
  DEBUG_SERIAL.println(CFG.cameraScan.storageBoundaryMargin);
  DEBUG_SERIAL.print(F("  columnX centers="));
  for (uint8_t i = 0; i < 4; i++) {
    if (i > 0) DEBUG_SERIAL.print(F(","));
    DEBUG_SERIAL.print(CFG.storageRack.columnXCenters[i]);
  }
  DEBUG_SERIAL.print(F(", xTolerance="));
  DEBUG_SERIAL.print(CFG.storageRack.columnXTolerance);
  DEBUG_SERIAL.print(F(", ySplit="));
  DEBUG_SERIAL.println(CFG.cameraScan.storageYUpperLowerSplit);
  DEBUG_SERIAL.println(F("  pickup regions:"));
  for (uint8_t i = 0; i < MissionConfig::STORAGE_PICKUP_REGION_COUNT; i++) {
    DEBUG_SERIAL.print(F("    "));
    DEBUG_SERIAL.print(storagePickupRegionName(i + 1));
    DEBUG_SERIAL.print(F(": x="));
    DEBUG_SERIAL.print(CFG.storagePickupRegion.xMin[i]);
    DEBUG_SERIAL.print(F("~"));
    DEBUG_SERIAL.print(CFG.storagePickupRegion.xMax[i]);
    DEBUG_SERIAL.print(F(", y="));
    DEBUG_SERIAL.print(CFG.storagePickupRegion.yMin[i]);
    DEBUG_SERIAL.print(F("~"));
    DEBUG_SERIAL.print(CFG.storagePickupRegion.yMax[i]);
    DEBUG_SERIAL.print(F(", grip="));
    DEBUG_SERIAL.println(CFG.storagePickupRegion.useUpperGripPose[i] ? F("upper") : F("lower"));
  }
  DEBUG_SERIAL.print(F("    margin x/y="));
  DEBUG_SERIAL.print(CFG.storagePickupRegion.xMargin);
  DEBUG_SERIAL.print(F("/"));
  DEBUG_SERIAL.println(CFG.storagePickupRegion.yMargin);
  DEBUG_SERIAL.println(F("  grip targets:"));
  for (uint8_t i = 0; i < MissionConfig::STORAGE_GRIP_TARGET_COUNT; i++) {
    DEBUG_SERIAL.print(F("    "));
    DEBUG_SERIAL.print(storageGripTargetName(i + 1));
    DEBUG_SERIAL.print(F(": x="));
    DEBUG_SERIAL.print(CFG.storageGripTarget.xMin[i]);
    DEBUG_SERIAL.print(F("~"));
    DEBUG_SERIAL.print(CFG.storageGripTarget.xMax[i]);
    DEBUG_SERIAL.print(F(", y="));
    DEBUG_SERIAL.print(CFG.storageGripTarget.yMin[i]);
    DEBUG_SERIAL.print(F("~"));
    DEBUG_SERIAL.print(CFG.storageGripTarget.yMax[i]);
    DEBUG_SERIAL.print(F(", grip="));
    DEBUG_SERIAL.println(CFG.storageGripTarget.useUpperGripPose[i] ? F("upper") : F("lower"));
  }
  DEBUG_SERIAL.print(F("    margin x/y="));
  DEBUG_SERIAL.print(CFG.storageGripTarget.xMargin);
  DEBUG_SERIAL.print(F("/"));
  DEBUG_SERIAL.println(CFG.storageGripTarget.yMargin);
  DEBUG_SERIAL.print(F("    center tolerance x/y="));
  DEBUG_SERIAL.print(CFG.storageGripTarget.centerToleranceX);
  DEBUG_SERIAL.print(F("/"));
  DEBUG_SERIAL.println(CFG.storageGripTarget.centerToleranceY);
}

bool commandPixyStorageMap(const String &input) {
  if (!pixyReady) {
    DEBUG_SERIAL.println(F("[주의] Pixy init이 실패했을 수 있습니다. 그래도 storage map을 시도합니다."));
  }

  String rowFilter = tokenAt(input, 2);
  rowFilter.toLowerCase();
  long frames = 5;
  long minArea = CFG.cameraScan.storageMinBlockArea;
  uint8_t firstNumericToken = 2;

  if (rowFilter == "upper" || rowFilter == "위" ||
      rowFilter == "lower" || rowFilter == "아래" ||
      rowFilter == "all" || rowFilter == "전체") {
    if (rowFilter == "위") rowFilter = "upper";
    if (rowFilter == "아래") rowFilter = "lower";
    if (rowFilter == "전체") rowFilter = "all";
    firstNumericToken = 3;
  } else {
    rowFilter = "lower";
  }

  if (tokenCount(input) > firstNumericToken &&
      !parseLongStrict(tokenAt(input, firstNumericToken), &frames)) {
    DEBUG_SERIAL.println(F("사용법: pixy storage [lower|all|upper] [frames] [minArea]"));
    return false;
  }
  if (tokenCount(input) > firstNumericToken + 1 &&
      !parseLongStrict(tokenAt(input, firstNumericToken + 1), &minArea)) {
    DEBUG_SERIAL.println(F("사용법: pixy storage [lower|all|upper] [frames] [minArea]"));
    return false;
  }

  frames = constrain(frames, 1, 20);
  minArea = constrain(minArea, 0, 10000);

  DEBUG_SERIAL.print(F("[Pixy 적재함 pickup region] row="));
  DEBUG_SERIAL.print(rowFilter);
  DEBUG_SERIAL.print(F(", frames="));
  DEBUG_SERIAL.print(frames);
  DEBUG_SERIAL.print(F(", minArea="));
  DEBUG_SERIAL.println(minArea);
  printStorageRackCalibrationInfo();

  bool foundAny = false;
  for (uint8_t frame = 0; frame < frames; frame++) {
    pixy.ccc.getBlocks(true, CFG.cameraScan.storageAllowedSignatureMap);
    DEBUG_SERIAL.print(F("frame "));
    DEBUG_SERIAL.print(frame + 1);
    DEBUG_SERIAL.print(F(": count="));
    DEBUG_SERIAL.println(pixy.ccc.numBlocks);

    for (uint8_t i = 0; i < pixy.ccc.numBlocks; i++) {
      uint32_t area = pixyBlockArea(i);
      if (area < (uint32_t)minArea) continue;

      int16_t x = pixy.ccc.blocks[i].m_x;
      int16_t y = pixy.ccc.blocks[i].m_y;
      uint8_t region = storagePickupRegionForPixyXY(x, y);
      bool regionUpper = storagePickupRegionUsesUpperGrip(region);
      if (rowFilter == "upper" && (region == 0 || !regionUpper)) continue;
      if (rowFilter == "lower" && (region == 0 || regionUpper)) continue;

      uint8_t column = storageColumnForPixyX(x);
      uint8_t slot = storageSlotForPixyXY(x, y);
      DEBUG_SERIAL.print(F("  sig="));
      DEBUG_SERIAL.print(pixy.ccc.blocks[i].m_signature);
      DEBUG_SERIAL.print(F(", x="));
      DEBUG_SERIAL.print(x);
      DEBUG_SERIAL.print(F(", y="));
      DEBUG_SERIAL.print(y);
      DEBUG_SERIAL.print(F(", pickupRegion="));
      DEBUG_SERIAL.print(storagePickupRegionName(region));
      DEBUG_SERIAL.print(F(", grip="));
      if (region == 0) DEBUG_SERIAL.print(F("?"));
      else DEBUG_SERIAL.print(regionUpper ? F("upper") : F("lower"));
      DEBUG_SERIAL.print(F(", col="));
      if (column == 0) DEBUG_SERIAL.print(F("?"));
      else DEBUG_SERIAL.print(column);
      DEBUG_SERIAL.print(F(", slot="));
      if (slot == 0) DEBUG_SERIAL.print(F("?"));
      else DEBUG_SERIAL.print(slot);
      DEBUG_SERIAL.print(F(", area="));
      DEBUG_SERIAL.println(area);
      foundAny = true;
    }
    if (!interruptibleDelay(80)) return false;
  }

  if (!foundAny) {
    DEBUG_SERIAL.println(F("[인식 없음] 하단 기준이면 pixy storage lower 10 0으로 필터를 낮춰 먼저 확인하세요."));
    turnOnLEDRed500ms();
    return false;
  }
  turnOnLEDGreen500ms();
  return true;
}

bool readBestBlockForStorageGripTarget(uint8_t allowedTargetMask,
                                       uint8_t signatureMap,
                                       uint16_t minArea,
                                       uint8_t requiredColumn,
                                       int16_t *blockX,
                                       int16_t *blockY,
                                       uint8_t *blockSig,
                                       uint8_t *pickupRegion,
                                       uint8_t *targetId,
                                       bool *targetReached) {
  signatureMap &= CFG.cameraScan.storageAllowedSignatureMap & 0x7F;
  if (signatureMap == 0) signatureMap = CFG.cameraScan.storageAllowedSignatureMap & 0x7F;
  pixy.ccc.getBlocks(true, signatureMap);
  if (pixy.ccc.numBlocks == 0) return false;

  int8_t bestIndex = -1;
  uint8_t bestPickupRegion = 0;
  uint8_t bestTarget = 0;
  bool bestReached = false;
  int32_t bestScore = 2147483647L;
  uint32_t bestArea = 0;

  for (uint8_t i = 0; i < pixy.ccc.numBlocks; i++) {
    uint32_t area = pixyBlockArea(i);
    if (area < minArea) continue;

    int16_t x = pixy.ccc.blocks[i].m_x;
    int16_t y = pixy.ccc.blocks[i].m_y;
    if (!storagePixyInRackBoundary(x, y)) continue;
    uint8_t pixyColumn = storageColumnForPixyX(x);
    if (requiredColumn > 0 && pixyColumn > 0 &&
        abs((int)pixyColumn - (int)requiredColumn) > 1) {
      continue;
    }
    uint8_t reached = storageGripTargetForPixyXY(allowedTargetMask, x, y);
    uint8_t closest = reached ? reached : closestStorageGripTargetForPixyXY(allowedTargetMask, x, y);
    if (closest == 0) continue;

    int32_t dx = x - storageGripTargetCenterX(closest);
    int32_t dy = y - storageGripTargetCenterY(closest);
    int32_t score = dx * dx + dy * dy;

    if (score < bestScore || (score == bestScore && area > bestArea)) {
      bestScore = score;
      bestArea = area;
      bestIndex = i;
      bestPickupRegion = storagePickupRegionForPixyXY(x, y);
      bestTarget = closest;
      bestReached = reached != 0;
    }
  }

  if (bestIndex < 0) return false;

  *blockX = pixy.ccc.blocks[bestIndex].m_x;
  *blockY = pixy.ccc.blocks[bestIndex].m_y;
  *blockSig = pixy.ccc.blocks[bestIndex].m_signature;
  *pickupRegion = bestPickupRegion;
  *targetId = bestTarget;
  *targetReached = bestReached;
  return true;
}

int32_t cameraFineTuneSignedVelocityForError(int16_t error, int16_t tolerance) {
  int16_t absError = abs(error);
  if (absError <= tolerance) return 0;

  int32_t maxSpeed = CFG.speed.cameraFineTuneSpeed;
  int32_t minSpeed = maxSpeed / 4;
  if (minSpeed < 25) minSpeed = 25;
  if (minSpeed > maxSpeed) minSpeed = maxSpeed;

  int16_t fullSpeedError = CFG.storageGripTarget.alignFullSpeedPixelError;
  if (fullSpeedError <= tolerance) fullSpeedError = tolerance + 1;

  int32_t numerator = (int32_t)(absError - tolerance);
  int32_t denominator = (int32_t)(fullSpeedError - tolerance);
  int32_t speed = minSpeed + ((maxSpeed - minSpeed) * numerator) / denominator;
  if (speed > maxSpeed) speed = maxSpeed;
  return error > 0 ? speed : -speed;
}

float fineAlignMmPerPixelXForTuner() {
  int32_t pixelSum = 0;
  uint8_t gapCount = 0;
  for (uint8_t i = 0; i < 3; i++) {
    int16_t gap = abs(CFG.storageRack.columnXCenters[i + 1] -
                      CFG.storageRack.columnXCenters[i]);
    if (gap > 0) {
      pixelSum += gap;
      gapCount++;
    }
  }
  if (gapCount == 0 || pixelSum == 0) return 1.0;
  float avgPixelGap = (float)pixelSum / (float)gapCount;
  return missionColumnStepMm / avgPixelGap;
}

float clampFineAlignDistanceForTuner(float value, float minValue, float maxValue) {
  if (value < minValue) return minValue;
  if (value > maxValue) return maxValue;
  return value;
}

float fineAlignDistanceForPixelErrorForTuner(int16_t error, float maxStepMm) {
  float distanceMm = (float)abs(error) *
                     fineAlignMmPerPixelXForTuner() *
                     CFG.storageGripTarget.fineAlignGain;
  return clampFineAlignDistanceForTuner(distanceMm,
                                        CFG.storageGripTarget.fineAlignMinStepMm,
                                        maxStepMm);
}

bool driveFineAlignDistanceForTuner(uint8_t direction, float distanceMm) {
  if (distanceMm <= 0.0) return true;
  ChangeMobilebaseMode2ExtendedPositionControlWithTimeBasedProfileMode(dxl);
  DriveDistanceAndMmPerSecAndDirection(dxl, distanceMm, direction,
                                       CFG.storageGripTarget.fineAlignSpeedMmPerSec);
  bool ok = waitForMobilePositionOrTimeout(CFG.timeout.positionMoveMs,
                                           F("  alignslow 이동 타임아웃"));
  ChangeMobilebaseMode2VelocityControlMode(dxl);
  stopMobilebase();
  return ok;
}

bool stepCameraGripAlignmentSlowForTuner(int16_t xError, int16_t yError) {
  if (abs(xError) > CFG.storageGripTarget.centerToleranceX) {
    float sideStepMm = fineAlignDistanceForPixelErrorForTuner(
      xError, CFG.storageGripTarget.fineAlignMaxStepMm);
    uint8_t direction = (xError > 0) ? DRIVE_DIRECTION_RIGHT : DRIVE_DIRECTION_LEFT;
    DEBUG_SERIAL.print(F("    alignslow X "));
    DEBUG_SERIAL.print(sideStepMm, 2);
    DEBUG_SERIAL.println(direction == DRIVE_DIRECTION_RIGHT ? F("mm right") : F("mm left"));
    return driveFineAlignDistanceForTuner(direction, sideStepMm);
  }

  if (CFG.storageGripTarget.fineAlignUseY &&
      abs(yError) > CFG.storageGripTarget.centerToleranceY) {
    float forwardStepMm = fineAlignDistanceForPixelErrorForTuner(
      yError, CFG.storageGripTarget.fineAlignForwardMaxStepMm);
    bool forward = (yError > 0) == CFG.storageGripTarget.yErrorUsesForwardDirection;
    uint8_t direction = forward ? DRIVE_DIRECTION_FORWARD : DRIVE_DIRECTION_BACKWARD;
    DEBUG_SERIAL.print(F("    alignslow Y "));
    DEBUG_SERIAL.print(forwardStepMm, 2);
    DEBUG_SERIAL.println(forward ? F("mm forward") : F("mm backward"));
    return driveFineAlignDistanceForTuner(direction, forwardStepMm);
  }

  return true;
}

bool stepCameraGripAlignmentForTuner(int16_t xError, int16_t yError) {
  int32_t sideVelocity = cameraFineTuneSignedVelocityForError(
    xError, CFG.storageGripTarget.centerToleranceX);
  int32_t forwardVelocity = cameraFineTuneSignedVelocityForError(
    yError, CFG.storageGripTarget.centerToleranceY);
  if (!CFG.storageGripTarget.yErrorUsesForwardDirection) {
    forwardVelocity = -forwardVelocity;
  }

  SetMobileGoalVelocityForSyncWrite(
    dxl,
    (-forwardVelocity + sideVelocity) / 2,
    (-forwardVelocity - sideVelocity) / 2,
    (-forwardVelocity - sideVelocity) / 2,
    (-forwardVelocity + sideVelocity) / 2);
  return interruptibleDelay(CFG.storageGripTarget.alignStepMs);
}

bool commandPixyAlignGripTargetMode(const String &input, bool slowMicroStep) {
  if (!ensureMobileReady()) return false;
  if (!pixyReady) {
    DEBUG_SERIAL.println(F("[주의] Pixy init이 실패했을 수 있습니다. 그래도 align을 시도합니다."));
  }

  String rowFilter = tokenAt(input, 2);
  rowFilter.toLowerCase();
  long timeoutMs = CFG.storageGripTarget.alignTimeoutMs;
  long minArea = CFG.cameraScan.storageMinBlockArea;
  uint8_t signatureMap = CFG.cameraScan.storageAllowedSignatureMap & 0x7F;
  uint8_t firstNumericToken = 2;

  if (rowFilter == "upper" || rowFilter == "위" ||
      rowFilter == "lower" || rowFilter == "아래" ||
      rowFilter == "all" || rowFilter == "전체") {
    if (rowFilter == "위") rowFilter = "upper";
    if (rowFilter == "아래") rowFilter = "lower";
    if (rowFilter == "전체") rowFilter = "all";
    firstNumericToken = 3;
  } else {
    rowFilter = "lower";
  }

  if (tokenCount(input) > firstNumericToken &&
      !parseLongStrict(tokenAt(input, firstNumericToken), &timeoutMs)) {
    DEBUG_SERIAL.println(F("사용법: pixy align|alignslow [lower|upper|all] [timeoutMs] [minArea] [signatureMap]"));
    return false;
  }
  if (tokenCount(input) > firstNumericToken + 1 &&
      !parseLongStrict(tokenAt(input, firstNumericToken + 1), &minArea)) {
    DEBUG_SERIAL.println(F("사용법: pixy align|alignslow [lower|upper|all] [timeoutMs] [minArea] [signatureMap]"));
    return false;
  }
  if (tokenCount(input) > firstNumericToken + 2 &&
      !parseSignatureMapText(tokenAt(input, firstNumericToken + 2), &signatureMap)) {
    DEBUG_SERIAL.println(F("사용법: pixy align|alignslow [lower|upper|all] [timeoutMs] [minArea] [signatureMap]"));
    return false;
  }

  timeoutMs = constrain(timeoutMs, 300, 15000);
  minArea = constrain(minArea, 0, 10000);
  uint8_t allowedTargetMask = storageGripTargetMaskForFilter(rowFilter);
  if (allowedTargetMask == 0) {
    DEBUG_SERIAL.println(F("[오류] 선택한 row에 해당하는 grip target이 없습니다."));
    return false;
  }

  DEBUG_SERIAL.print(slowMicroStep ? F("[Pixy grip target alignslow] row=")
                                   : F("[Pixy grip target align] row="));
  DEBUG_SERIAL.print(rowFilter);
  DEBUG_SERIAL.print(F(", timeoutMs="));
  DEBUG_SERIAL.print(timeoutMs);
  DEBUG_SERIAL.print(F(", minArea="));
  DEBUG_SERIAL.print(minArea);
  DEBUG_SERIAL.print(F(", signatureMap="));
  printSignatureMap(signatureMap);
  DEBUG_SERIAL.println();
  uint8_t requiredColumn = 0;
  if (missionStage == MISSION_PICK_HOLD &&
      missionStorageColumn >= 1 && missionStorageColumn <= 4) {
    requiredColumn = missionStorageColumn;
    DEBUG_SERIAL.print(F("  required current column="));
    DEBUG_SERIAL.println(requiredColumn);
  }
  if (slowMicroStep) {
    DEBUG_SERIAL.print(F("  micro-step gain/min/max/speed="));
    DEBUG_SERIAL.print(CFG.storageGripTarget.fineAlignGain, 2);
    DEBUG_SERIAL.print(F(" / "));
    DEBUG_SERIAL.print(CFG.storageGripTarget.fineAlignMinStepMm, 2);
    DEBUG_SERIAL.print(F(" / "));
    DEBUG_SERIAL.print(CFG.storageGripTarget.fineAlignMaxStepMm, 2);
    DEBUG_SERIAL.print(F("mm @ "));
    DEBUG_SERIAL.print(CFG.storageGripTarget.fineAlignSpeedMmPerSec);
    DEBUG_SERIAL.print(F("mm/s, mmPerPixelX="));
    DEBUG_SERIAL.println(fineAlignMmPerPixelXForTuner(), 3);
  } else {
    DEBUG_SERIAL.print(F("  fine speed="));
    DEBUG_SERIAL.print(CFG.speed.cameraFineTuneSpeed);
    DEBUG_SERIAL.print(F(", vector sampleMs="));
    DEBUG_SERIAL.print(CFG.storageGripTarget.alignStepMs);
    DEBUG_SERIAL.print(F(", fullSpeedPx="));
    DEBUG_SERIAL.println(CFG.storageGripTarget.alignFullSpeedPixelError);
  }
  printStorageRackCalibrationInfo();

  ChangeMobilebaseMode2VelocityControlMode(dxl);
  unsigned long startedAt = millis();
  uint16_t stepCount = 0;
  while (millis() - startedAt <= (unsigned long)timeoutMs) {
    if (checkEmergencyStopInput()) {
      stopAll(F("[긴급정지] ! 입력"));
      return false;
    }

    int16_t blockX = 0;
    int16_t blockY = 0;
    uint8_t blockSig = 0;
    uint8_t pickupRegion = 0;
    uint8_t targetId = 0;
    bool targetReached = false;

    if (!readBestBlockForStorageGripTarget(allowedTargetMask, signatureMap, (uint16_t)minArea,
                                           requiredColumn,
                                           &blockX, &blockY, &blockSig,
                                           &pickupRegion, &targetId, &targetReached)) {
      DEBUG_SERIAL.print(F("  t="));
      DEBUG_SERIAL.print(millis() - startedAt);
      DEBUG_SERIAL.println(F("ms block=none"));
      stopMobilebase();
      if (!interruptibleDelay(CFG.storageGripTarget.alignStepMs)) return false;
      continue;
    }

    int16_t xError = blockX - storageGripTargetCenterX(targetId);
    int16_t yError = blockY - storageGripTargetCenterY(targetId);
    bool centerReached =
      abs(xError) <= CFG.storageGripTarget.centerToleranceX &&
      abs(yError) <= CFG.storageGripTarget.centerToleranceY;
    DEBUG_SERIAL.print(F("  step="));
    DEBUG_SERIAL.print(stepCount++);
    DEBUG_SERIAL.print(F(", t="));
    DEBUG_SERIAL.print(millis() - startedAt);
    DEBUG_SERIAL.print(F("ms, sig="));
    DEBUG_SERIAL.print(blockSig);
    DEBUG_SERIAL.print(F(", x="));
    DEBUG_SERIAL.print(blockX);
    DEBUG_SERIAL.print(F(", y="));
    DEBUG_SERIAL.print(blockY);
    DEBUG_SERIAL.print(F(", pickup="));
    DEBUG_SERIAL.print(storagePickupRegionName(pickupRegion));
    DEBUG_SERIAL.print(F(", target="));
    DEBUG_SERIAL.print(storageGripTargetName(targetId));
    DEBUG_SERIAL.print(F(", boundary="));
    DEBUG_SERIAL.print(targetReached ? F("inside") : F("outside"));
    DEBUG_SERIAL.print(F(", dx="));
    DEBUG_SERIAL.print(xError);
    DEBUG_SERIAL.print(F(", dy="));
    DEBUG_SERIAL.println(yError);

    if (centerReached) {
      stopMobilebase();
      DEBUG_SERIAL.print(F("[정렬 완료] elapsedMs="));
      DEBUG_SERIAL.println(millis() - startedAt);
      turnOnLEDGreen500ms();
      return true;
    }

    if (slowMicroStep) {
      if (!stepCameraGripAlignmentSlowForTuner(xError, yError)) return false;
    } else {
      if (!stepCameraGripAlignmentForTuner(xError, yError)) return false;
    }
  }

  stopMobilebase();
  DEBUG_SERIAL.print(F("[정렬 타임아웃] elapsedMs="));
  DEBUG_SERIAL.println(millis() - startedAt);
  turnOnLEDRed500ms();
  return false;
}

bool commandPixyAlignGripTarget(const String &input) {
  return commandPixyAlignGripTargetMode(input, false);
}

bool commandPixyAlignGripTargetSlow(const String &input) {
  return commandPixyAlignGripTargetMode(input, true);
}

bool commandPixyScan(uint8_t signatureMap, uint8_t frames) {
  if (!pixyReady) {
    DEBUG_SERIAL.println(F("[주의] Pixy init이 실패했을 수 있습니다. 그래도 스캔을 시도합니다."));
  }

  frames = constrain(frames, 1, 20);
  DEBUG_SERIAL.print(F("[Pixy 스캔] signatureMap="));
  printSignatureMap(signatureMap);
  DEBUG_SERIAL.print(F(", frames="));
  DEBUG_SERIAL.println(frames);

  bool foundAny = false;
  for (uint8_t frame = 0; frame < frames; frame++) {
    pixy.ccc.getBlocks(true, signatureMap);
    DEBUG_SERIAL.print(F("frame "));
    DEBUG_SERIAL.print(frame + 1);
    DEBUG_SERIAL.print(F(": count="));
    DEBUG_SERIAL.println(pixy.ccc.numBlocks);

    for (uint8_t i = 0; i < pixy.ccc.numBlocks; i++) {
      printPixyBlock(i);
      foundAny = true;
    }
    if (!interruptibleDelay(80)) return false;
  }

  if (!foundAny) {
    DEBUG_SERIAL.println(F("[인식 없음] 조명, PixyMon signature, 거리, 필터 map을 확인하세요."));
    turnOnLEDRed500ms();
    return false;
  }
  turnOnLEDGreen500ms();
  return true;
}

bool parsePixyTargetToSignatureMap(String text, uint8_t *signatureMap) {
  text.trim();
  text.toLowerCase();
  if (text.length() == 0 || text == "all" || text == "전체") {
    *signatureMap = 0x7F;
    return true;
  }
  if (text.startsWith("0b")) {
    return parseSignatureMapText(text, signatureMap);
  }

  long signature = 0;
  if (!parseLongStrict(text, &signature) || signature < 1 || signature > 7) {
    return false;
  }
  *signatureMap = (uint8_t)(1 << (signature - 1));
  return true;
}

bool commandPixyWatch(const String &input) {
  if (!pixyReady) {
    DEBUG_SERIAL.println(F("[주의] Pixy init이 실패했을 수 있습니다. 그래도 watch를 시도합니다."));
  }

  uint8_t signatureMap = 0x7F;
  long frames = 30;
  long intervalMs = 60;
  if (tokenCount(input) >= 3 && !parsePixyTargetToSignatureMap(tokenAt(input, 2), &signatureMap)) {
    DEBUG_SERIAL.println(F("사용법: pixy watch <1~7|all|0b1111111> [frames] [intervalMs]"));
    return false;
  }
  if (tokenCount(input) >= 4 && !parseLongStrict(tokenAt(input, 3), &frames)) {
    DEBUG_SERIAL.println(F("[오류] frames는 숫자로 입력하세요."));
    return false;
  }
  if (tokenCount(input) >= 5 && !parseLongStrict(tokenAt(input, 4), &intervalMs)) {
    DEBUG_SERIAL.println(F("[오류] intervalMs는 숫자로 입력하세요."));
    return false;
  }
  frames = constrain(frames, 5, 80);
  intervalMs = constrain(intervalMs, 30, 300);

  uint16_t seenFrames = 0;
  uint32_t sumX = 0;
  uint32_t sumY = 0;
  uint32_t sumArea = 0;
  uint32_t minArea = 0xFFFFFFFF;
  uint32_t maxArea = 0;
  int16_t minX = 32767;
  int16_t maxX = -32768;
  int16_t minY = 32767;
  int16_t maxY = -32768;

  DEBUG_SERIAL.print(F("[Pixy 안정도 watch] signatureMap="));
  printSignatureMap(signatureMap);
  DEBUG_SERIAL.print(F(", frames="));
  DEBUG_SERIAL.print(frames);
  DEBUG_SERIAL.print(F(", intervalMs="));
  DEBUG_SERIAL.println(intervalMs);

  for (uint8_t frame = 0; frame < frames; frame++) {
    pixy.ccc.getBlocks(true, signatureMap);

    int8_t bestIndex = -1;
    uint32_t bestArea = 0;
    for (uint8_t i = 0; i < pixy.ccc.numBlocks; i++) {
      uint32_t area = (uint32_t)pixy.ccc.blocks[i].m_width *
                      (uint32_t)pixy.ccc.blocks[i].m_height;
      if (area > bestArea) {
        bestArea = area;
        bestIndex = i;
      }
    }

    if (bestIndex >= 0) {
      seenFrames++;
      int16_t x = pixy.ccc.blocks[bestIndex].m_x;
      int16_t y = pixy.ccc.blocks[bestIndex].m_y;
      sumX += x;
      sumY += y;
      sumArea += bestArea;
      minArea = min(minArea, bestArea);
      maxArea = max(maxArea, bestArea);
      minX = min(minX, x);
      maxX = max(maxX, x);
      minY = min(minY, y);
      maxY = max(maxY, y);
    }

    if (!interruptibleDelay(intervalMs)) return false;
  }

  DEBUG_SERIAL.print(F("  인식 프레임="));
  DEBUG_SERIAL.print(seenFrames);
  DEBUG_SERIAL.print(F("/"));
  DEBUG_SERIAL.print(frames);
  DEBUG_SERIAL.print(F(" ("));
  DEBUG_SERIAL.print((uint16_t)((seenFrames * 100UL) / frames));
  DEBUG_SERIAL.println(F("%)"));

  if (seenFrames == 0) {
    DEBUG_SERIAL.println(F("[판정] 인식 없음: 조명, PixyMon signature, 거리, 배경색을 먼저 확인하세요."));
    turnOnLEDRed500ms();
    return false;
  }

  uint16_t avgX = (uint16_t)(sumX / seenFrames);
  uint16_t avgY = (uint16_t)(sumY / seenFrames);
  uint32_t avgArea = sumArea / seenFrames;
  DEBUG_SERIAL.print(F("  평균 x=")); DEBUG_SERIAL.print(avgX);
  DEBUG_SERIAL.print(F(", y=")); DEBUG_SERIAL.print(avgY);
  DEBUG_SERIAL.print(F(", area=")); DEBUG_SERIAL.println(avgArea);
  DEBUG_SERIAL.print(F("  흔들림 xRange=")); DEBUG_SERIAL.print(maxX - minX);
  DEBUG_SERIAL.print(F(", yRange=")); DEBUG_SERIAL.print(maxY - minY);
  DEBUG_SERIAL.print(F(", areaRange=")); DEBUG_SERIAL.println(maxArea - minArea);

  DEBUG_SERIAL.print(F("{\"type\":\"pixy-watch\",\"seen\":"));
  DEBUG_SERIAL.print(seenFrames);
  DEBUG_SERIAL.print(F(",\"frames\":"));
  DEBUG_SERIAL.print(frames);
  DEBUG_SERIAL.print(F(",\"avgX\":"));
  DEBUG_SERIAL.print(avgX);
  DEBUG_SERIAL.print(F(",\"avgY\":"));
  DEBUG_SERIAL.print(avgY);
  DEBUG_SERIAL.print(F(",\"avgArea\":"));
  DEBUG_SERIAL.print(avgArea);
  DEBUG_SERIAL.print(F(",\"xRange\":"));
  DEBUG_SERIAL.print(maxX - minX);
  DEBUG_SERIAL.print(F(",\"yRange\":"));
  DEBUG_SERIAL.print(maxY - minY);
  DEBUG_SERIAL.println(F("}"));

  if (seenFrames * 100UL < frames * 80UL) {
    DEBUG_SERIAL.println(F("[판정] 불안정: 인식률 80% 미만입니다. 조명 고정/배경 제거/재학습을 권장합니다."));
    turnOnLEDRed500ms();
    return false;
  }
  if ((maxX - minX) > 35 || (maxY - minY) > 25) {
    DEBUG_SERIAL.println(F("[판정] 위치 흔들림 큼: 카메라/블록/조명 고정 상태를 확인하세요."));
  } else {
    DEBUG_SERIAL.println(F("[판정] 인식 안정도 양호"));
  }
  turnOnLEDGreen500ms();
  return true;
}

uint8_t countQualifiedPixyBlocksInFrame(uint8_t signatureMap, uint32_t minArea) {
  uint8_t count = 0;
  for (uint8_t i = 0; i < pixy.ccc.numBlocks; i++) {
    uint8_t signature = pixy.ccc.blocks[i].m_signature;
    if (signature < 1 || signature > 7) continue;
    if (((1 << (signature - 1)) & signatureMap) == 0) continue;

    uint32_t area = (uint32_t)pixy.ccc.blocks[i].m_width *
                    (uint32_t)pixy.ccc.blocks[i].m_height;
    if (area < minArea) continue;

    count++;
  }
  return count;
}

bool pixyBlockPassesMissionInstructionFilter(uint8_t index, uint8_t signatureMap, uint16_t minArea) {
  uint8_t signature = pixy.ccc.blocks[index].m_signature;
  if (signature < 1 || signature > CFG.cameraScan.maxSignature) return false;
  if ((((uint8_t)1 << (signature - 1)) & signatureMap) == 0) return false;
  return pixyBlockArea(index) >= minArea;
}

uint8_t collectMissionInstructionBlocks(uint8_t signatureMap,
                                        uint16_t minArea,
                                        int16_t *blockX,
                                        uint8_t *blockSig,
                                        uint32_t *totalArea) {
  uint8_t count = 0;
  *totalArea = 0;
  for (uint8_t i = 0; i < pixy.ccc.numBlocks && count < MissionConfig::MAX_MISSION_BLOCKS; i++) {
    if (!pixyBlockPassesMissionInstructionFilter(i, signatureMap, minArea)) continue;
    blockX[count] = pixy.ccc.blocks[i].m_x;
    blockSig[count] = pixy.ccc.blocks[i].m_signature;
    *totalArea += pixyBlockArea(i);
    count++;
  }
  return count;
}

void sortMissionInstructionBlocksByX(int16_t *blockX, uint8_t *blockSig, uint8_t count) {
  for (uint8_t i = 0; i < count; i++) {
    for (uint8_t j = i + 1; j < count; j++) {
      if (blockX[j] < blockX[i]) {
        int16_t tempX = blockX[i];
        blockX[i] = blockX[j];
        blockX[j] = tempX;
        uint8_t tempSig = blockSig[i];
        blockSig[i] = blockSig[j];
        blockSig[j] = tempSig;
      }
    }
  }
}

uint8_t missionInstructionSignatureMap() {
  return CFG.cameraScan.missionInstructionAllowedSignatureMap &
         CFG.mission.blockSignatureMap &
         0x7F;
}

void updateMissionInstructionBestFromPixy(uint8_t signatureMap,
                                          uint16_t minArea,
                                          int16_t *bestX,
                                          uint8_t *bestSig,
                                          uint8_t *bestCount,
                                          uint32_t *bestAreaScore) {
  pixy.ccc.getBlocks(true, signatureMap);
  int16_t candidateX[MissionConfig::MAX_MISSION_BLOCKS] = {0};
  uint8_t candidateSig[MissionConfig::MAX_MISSION_BLOCKS] = {0};
  uint32_t areaScore = 0;
  uint8_t candidateCount = collectMissionInstructionBlocks(signatureMap, minArea,
                                                           candidateX, candidateSig,
                                                           &areaScore);
  if (candidateCount > *bestCount ||
      (candidateCount == *bestCount && areaScore > *bestAreaScore)) {
    *bestCount = candidateCount;
    *bestAreaScore = areaScore;
    for (uint8_t i = 0; i < candidateCount; i++) {
      bestX[i] = candidateX[i];
      bestSig[i] = candidateSig[i];
    }
  }
}

bool commandPixySweep(const String &input) {
  if (!pixyReady) {
    DEBUG_SERIAL.println(F("[주의] Pixy init이 실패했을 수 있습니다. 그래도 sweep을 시도합니다."));
  }

  uint8_t signatureMap = 0x7F;
  long expectedBlocks = 4;
  long startBrightness = 40;
  long endBrightness = 140;
  long stepBrightness = 20;
  long frames = 12;
  long minArea = 80;

  if (tokenCount(input) >= 3 && !parsePixyTargetToSignatureMap(tokenAt(input, 2), &signatureMap)) {
    DEBUG_SERIAL.println(F("사용법: pixy sweep [1~7|all|0b1111111] [expected] [start] [end] [step] [frames] [minArea]"));
    return false;
  }
  if (tokenCount(input) >= 4 && !parseLongStrict(tokenAt(input, 3), &expectedBlocks)) return false;
  if (tokenCount(input) >= 5 && !parseLongStrict(tokenAt(input, 4), &startBrightness)) return false;
  if (tokenCount(input) >= 6 && !parseLongStrict(tokenAt(input, 5), &endBrightness)) return false;
  if (tokenCount(input) >= 7 && !parseLongStrict(tokenAt(input, 6), &stepBrightness)) return false;
  if (tokenCount(input) >= 8 && !parseLongStrict(tokenAt(input, 7), &frames)) return false;
  if (tokenCount(input) >= 9 && !parseLongStrict(tokenAt(input, 8), &minArea)) return false;

  expectedBlocks = constrain(expectedBlocks, 1, 7);
  startBrightness = constrain(startBrightness, 0, 255);
  endBrightness = constrain(endBrightness, 0, 255);
  stepBrightness = constrain(stepBrightness, 1, 80);
  frames = constrain(frames, 5, 40);
  minArea = constrain(minArea, 1, 10000);

  if (startBrightness > endBrightness) {
    long tmp = startBrightness;
    startBrightness = endBrightness;
    endBrightness = tmp;
  }

  long bestBrightness = startBrightness;
  uint16_t bestScore = 0;
  uint16_t bestFullFrames = 0;
  uint16_t bestAvgCountTimes10 = 0;

  DEBUG_SERIAL.print(F("[Pixy 밝기 sweep] signatureMap="));
  printSignatureMap(signatureMap);
  DEBUG_SERIAL.print(F(", expected="));
  DEBUG_SERIAL.print(expectedBlocks);
  DEBUG_SERIAL.print(F(", range="));
  DEBUG_SERIAL.print(startBrightness);
  DEBUG_SERIAL.print(F("~"));
  DEBUG_SERIAL.print(endBrightness);
  DEBUG_SERIAL.print(F(", step="));
  DEBUG_SERIAL.print(stepBrightness);
  DEBUG_SERIAL.print(F(", frames="));
  DEBUG_SERIAL.print(frames);
  DEBUG_SERIAL.print(F(", minArea="));
  DEBUG_SERIAL.println(minArea);

  for (long brightness = startBrightness; brightness <= endBrightness; brightness += stepBrightness) {
    int8_t setResult = pixy.setCameraBrightness((uint8_t)brightness);
    if (!interruptibleDelay(220)) return false;

    uint16_t fullFrames = 0;
    uint16_t seenFrames = 0;
    uint16_t totalCount = 0;
    uint8_t maxCount = 0;

    for (uint8_t frame = 0; frame < frames; frame++) {
      pixy.ccc.getBlocks(true, signatureMap);
      uint8_t count = countQualifiedPixyBlocksInFrame(signatureMap, (uint32_t)minArea);
      if (count > 0) seenFrames++;
      if (count >= expectedBlocks) fullFrames++;
      totalCount += count;
      maxCount = max(maxCount, count);
      if (!interruptibleDelay(60)) return false;
    }

    uint16_t avgCountTimes10 = (uint16_t)((totalCount * 10UL) / frames);
    uint16_t score = fullFrames * 100 + seenFrames * 10 + avgCountTimes10;

    DEBUG_SERIAL.print(F("  brightness="));
    DEBUG_SERIAL.print(brightness);
    DEBUG_SERIAL.print(F(", result="));
    DEBUG_SERIAL.print(setResult);
    DEBUG_SERIAL.print(F(", full="));
    DEBUG_SERIAL.print(fullFrames);
    DEBUG_SERIAL.print(F("/"));
    DEBUG_SERIAL.print(frames);
    DEBUG_SERIAL.print(F(", seen="));
    DEBUG_SERIAL.print(seenFrames);
    DEBUG_SERIAL.print(F("/"));
    DEBUG_SERIAL.print(frames);
    DEBUG_SERIAL.print(F(", avgCount="));
    DEBUG_SERIAL.print(avgCountTimes10 / 10);
    DEBUG_SERIAL.print(F("."));
    DEBUG_SERIAL.print(avgCountTimes10 % 10);
    DEBUG_SERIAL.print(F(", maxCount="));
    DEBUG_SERIAL.println(maxCount);

    if (score > bestScore) {
      bestScore = score;
      bestBrightness = brightness;
      bestFullFrames = fullFrames;
      bestAvgCountTimes10 = avgCountTimes10;
    }
  }

  pixy.setCameraBrightness((uint8_t)bestBrightness);
  DEBUG_SERIAL.print(F("[추천 밝기] "));
  DEBUG_SERIAL.print(bestBrightness);
  DEBUG_SERIAL.print(F("  fullFrames="));
  DEBUG_SERIAL.print(bestFullFrames);
  DEBUG_SERIAL.print(F("/"));
  DEBUG_SERIAL.print(frames);
  DEBUG_SERIAL.print(F(", avgCount="));
  DEBUG_SERIAL.print(bestAvgCountTimes10 / 10);
  DEBUG_SERIAL.print(F("."));
  DEBUG_SERIAL.println(bestAvgCountTimes10 % 10);

  DEBUG_SERIAL.print(F("{\"type\":\"pixy-brightness-sweep\",\"bestBrightness\":"));
  DEBUG_SERIAL.print(bestBrightness);
  DEBUG_SERIAL.print(F(",\"fullFrames\":"));
  DEBUG_SERIAL.print(bestFullFrames);
  DEBUG_SERIAL.print(F(",\"frames\":"));
  DEBUG_SERIAL.print(frames);
  DEBUG_SERIAL.print(F(",\"avgCountTimes10\":"));
  DEBUG_SERIAL.print(bestAvgCountTimes10);
  DEBUG_SERIAL.println(F("}"));

  if (bestFullFrames == 0) {
    DEBUG_SERIAL.println(F("[주의] 모든 블록을 찾은 밝기가 없습니다. 추천값은 가장 많이 잡힌 후보입니다."));
    turnOnLEDRed500ms();
    return false;
  }

  turnOnLEDGreen500ms();
  return true;
}

bool parseDriveDirection(String directionText, uint8_t *direction) {
  directionText.toLowerCase();
  if (directionText == "forward" || directionText == "f" || directionText == "전진") {
    *direction = DRIVE_DIRECTION_FORWARD;
    return true;
  }
  if (directionText == "back" || directionText == "backward" || directionText == "b" || directionText == "후진") {
    *direction = DRIVE_DIRECTION_BACKWARD;
    return true;
  }
  if (directionText == "left" || directionText == "l" || directionText == "좌") {
    *direction = DRIVE_DIRECTION_LEFT;
    return true;
  }
  if (directionText == "right" || directionText == "r" || directionText == "우") {
    *direction = DRIVE_DIRECTION_RIGHT;
    return true;
  }
  return false;
}

bool checkDriveLimits(long distanceMm, long speedMmPerSec) {
  if (distanceMm <= 0 || distanceMm > profile().maxDriveMm) {
    DEBUG_SERIAL.print(F("[제한] 현재 profile의 최대 1회 이동거리는 "));
    DEBUG_SERIAL.print(profile().maxDriveMm);
    DEBUG_SERIAL.println(F("mm입니다."));
    return false;
  }
  if (speedMmPerSec <= 0 || speedMmPerSec > profile().maxDriveMmPerSec) {
    DEBUG_SERIAL.print(F("[제한] 현재 profile의 최대 주행속도는 "));
    DEBUG_SERIAL.print(profile().maxDriveMmPerSec);
    DEBUG_SERIAL.println(F("mm/s입니다."));
    return false;
  }

  unsigned long estimatedMs = (unsigned long)distanceMm * 1000UL / (unsigned long)speedMmPerSec;
  if (estimatedMs > profile().maxDriveTimeMs) {
    DEBUG_SERIAL.print(F("[제한] 예상 주행 시간이 "));
    DEBUG_SERIAL.print(estimatedMs);
    DEBUG_SERIAL.print(F("ms입니다. 현재 profile 한도는 "));
    DEBUG_SERIAL.print(profile().maxDriveTimeMs);
    DEBUG_SERIAL.println(F("ms입니다."));
    return false;
  }

  return true;
}

bool waitForDriveCompletion(unsigned long estimatedMs) {
  unsigned long startedAt = millis();
  while (!CheckIfMobilebaseIsInPosition(dxl)) {
    if (millis() - startedAt > estimatedMs + 1500UL) {
      stopAll(F("[주행 타임아웃] 목표 위치 도착 확인이 늦어 정지합니다."));
      return false;
    }
    if (!interruptibleDelay(10)) return false;
  }
  stopMobilebase();
  ChangeMobilebaseMode2VelocityControlMode(dxl);
  DEBUG_SERIAL.println(F("[주행 완료]"));
  turnOnLEDGreen500ms();
  return true;
}

bool scaleWithinTrimLimit(float scale) {
  return scale >= 0.70 && scale <= 1.30;
}

int32_t scaledWheelRaw(int32_t baseRaw, float scale) {
  return (int32_t)round((float)baseRaw * scale);
}

void printTrimHint() {
  DEBUG_SERIAL.println(F("보정 해석:"));
  DEBUG_SERIAL.println(F("  1.00은 기본값, 0.95는 해당 바퀴 목표 회전량 5% 감소, 1.05는 5% 증가입니다."));
  DEBUG_SERIAL.println(F("  후진이 한쪽으로 휘면 0.03~0.05 단위로 한쪽 비율만 바꿔 짧게 확인하세요."));
  DEBUG_SERIAL.println(F("  예: drive trim 후진 200 100 1.00 0.95"));
}

void applyMissionWheelVelocityTrim() {
  SetMobileWheelVelocityTrim(missionWheelVelocityTrimFl, missionWheelVelocityTrimFr,
                             missionWheelVelocityTrimBl, missionWheelVelocityTrimBr);
}

void printMissionWheelVelocityTrimStatus() {
  DEBUG_SERIAL.print(F("[drive balance] velocity trim FL/FR/BL/BR="));
  DEBUG_SERIAL.print(missionWheelVelocityTrimFl, 3);
  DEBUG_SERIAL.print(F("/"));
  DEBUG_SERIAL.print(missionWheelVelocityTrimFr, 3);
  DEBUG_SERIAL.print(F("/"));
  DEBUG_SERIAL.print(missionWheelVelocityTrimBl, 3);
  DEBUG_SERIAL.print(F("/"));
  DEBUG_SERIAL.println(missionWheelVelocityTrimBr, 3);
  DEBUG_SERIAL.print(F("{\"type\":\"drive-balance\",\"fl\":"));
  DEBUG_SERIAL.print(missionWheelVelocityTrimFl, 3);
  DEBUG_SERIAL.print(F(",\"fr\":"));
  DEBUG_SERIAL.print(missionWheelVelocityTrimFr, 3);
  DEBUG_SERIAL.print(F(",\"bl\":"));
  DEBUG_SERIAL.print(missionWheelVelocityTrimBl, 3);
  DEBUG_SERIAL.print(F(",\"br\":"));
  DEBUG_SERIAL.print(missionWheelVelocityTrimBr, 3);
  DEBUG_SERIAL.println(F("}"));
  DEBUG_SERIAL.println(F("  적용 범위: velocity-mode 주행/PSD 정렬. position-mode drive/mission columnstep은 영향 없음."));
}

bool setMissionWheelVelocityTrim(float flScale, float frScale,
                                 float blScale, float brScale) {
  if (!scaleWithinTrimLimit(flScale) || !scaleWithinTrimLimit(frScale) ||
      !scaleWithinTrimLimit(blScale) || !scaleWithinTrimLimit(brScale)) {
    DEBUG_SERIAL.println(F("[제한] velocity wheel balance 비율은 0.70~1.30 범위만 허용합니다."));
    return false;
  }
  missionWheelVelocityTrimFl = flScale;
  missionWheelVelocityTrimFr = frScale;
  missionWheelVelocityTrimBl = blScale;
  missionWheelVelocityTrimBr = brScale;
  applyMissionWheelVelocityTrim();
  printMissionWheelVelocityTrimStatus();
  return true;
}

void buildDirectionalVelocityRaw(uint8_t direction, int32_t raw,
                                 int32_t *flRaw, int32_t *frRaw,
                                 int32_t *blRaw, int32_t *brRaw) {
  *flRaw = raw;
  *frRaw = raw;
  *blRaw = raw;
  *brRaw = raw;
  switch (direction) {
    case DRIVE_DIRECTION_BACKWARD:
      *flRaw = -*flRaw;
      *frRaw = -*frRaw;
      *blRaw = -*blRaw;
      *brRaw = -*brRaw;
      break;
    case DRIVE_DIRECTION_LEFT:
      *flRaw = -raw;
      *frRaw = raw;
      *blRaw = raw;
      *brRaw = -raw;
      break;
    case DRIVE_DIRECTION_RIGHT:
      *flRaw = raw;
      *frRaw = -raw;
      *blRaw = -raw;
      *brRaw = raw;
      break;
    case DRIVE_DIRECTION_FORWARD:
    default:
      break;
  }
}

bool commandDriveBalance(const String &input) {
  String sub = tokenAt(input, 2);
  sub.toLowerCase();
  if (sub.length() == 0 || sub == "status" || sub == "상태") {
    printMissionWheelVelocityTrimStatus();
    DEBUG_SERIAL.println(F("사용법: drive balance set <leftScale> <rightScale>"));
    DEBUG_SERIAL.println(F("       drive balance test <forward|back|left|right> <raw> <ms> [leftScale rightScale]"));
    return true;
  }

  if (sub == "reset" || sub == "초기화") {
    return setMissionWheelVelocityTrim(1.0, 1.0, 1.0, 1.0);
  }

  if (sub == "set" || sub == "설정") {
    float flScale = 1.0;
    float frScale = 1.0;
    float blScale = 1.0;
    float brScale = 1.0;
    if (tokenCount(input) >= 7) {
      if (!parseFloatStrict(tokenAt(input, 3), &flScale) ||
          !parseFloatStrict(tokenAt(input, 4), &frScale) ||
          !parseFloatStrict(tokenAt(input, 5), &blScale) ||
          !parseFloatStrict(tokenAt(input, 6), &brScale)) {
        DEBUG_SERIAL.println(F("사용법: drive balance set <fl> <fr> <bl> <br>"));
        return false;
      }
    } else if (tokenCount(input) >= 5) {
      float leftScale = 1.0;
      float rightScale = 1.0;
      if (!parseFloatStrict(tokenAt(input, 3), &leftScale) ||
          !parseFloatStrict(tokenAt(input, 4), &rightScale)) {
        DEBUG_SERIAL.println(F("사용법: drive balance set <leftScale> <rightScale>"));
        return false;
      }
      flScale = leftScale;
      blScale = leftScale;
      frScale = rightScale;
      brScale = rightScale;
    } else {
      DEBUG_SERIAL.println(F("사용법: drive balance set <leftScale> <rightScale>"));
      DEBUG_SERIAL.println(F("또는: drive balance set <fl> <fr> <bl> <br>"));
      return false;
    }
    return setMissionWheelVelocityTrim(flScale, frScale, blScale, brScale);
  }

  if (sub == "test" || sub == "테스트") {
    if (!ensureMobileReady()) return false;

    uint8_t direction = DRIVE_DIRECTION_FORWARD;
    long raw = 0;
    long durationMs = 0;
    if (!parseDriveDirection(tokenAt(input, 3), &direction) ||
        !parseLongStrict(tokenAt(input, 4), &raw) ||
        !parseLongStrict(tokenAt(input, 5), &durationMs)) {
      DEBUG_SERIAL.println(F("사용법: drive balance test <forward|back|left|right> <raw> <ms> [leftScale rightScale]"));
      return false;
    }
    if (raw <= 0 || raw > profile().maxMissionVelocityRaw) {
      DEBUG_SERIAL.print(F("[제한] raw 속도는 1~"));
      DEBUG_SERIAL.print(profile().maxMissionVelocityRaw);
      DEBUG_SERIAL.println(F(" 범위에서 테스트하세요."));
      return false;
    }
    if (durationMs <= 0 || durationMs > profile().maxDriveTimeMs) {
      DEBUG_SERIAL.print(F("[제한] 테스트 시간은 1~"));
      DEBUG_SERIAL.print(profile().maxDriveTimeMs);
      DEBUG_SERIAL.println(F("ms 범위에서 테스트하세요."));
      return false;
    }
    if (tokenCount(input) >= 10) {
      float flScale = 1.0;
      float frScale = 1.0;
      float blScale = 1.0;
      float brScale = 1.0;
      if (!parseFloatStrict(tokenAt(input, 6), &flScale) ||
          !parseFloatStrict(tokenAt(input, 7), &frScale) ||
          !parseFloatStrict(tokenAt(input, 8), &blScale) ||
          !parseFloatStrict(tokenAt(input, 9), &brScale) ||
          !setMissionWheelVelocityTrim(flScale, frScale, blScale, brScale)) {
        DEBUG_SERIAL.println(F("사용법: drive balance test <방향> <raw> <ms> <fl> <fr> <bl> <br>"));
        return false;
      }
    } else if (tokenCount(input) >= 8) {
      float leftScale = 1.0;
      float rightScale = 1.0;
      if (!parseFloatStrict(tokenAt(input, 6), &leftScale) ||
          !parseFloatStrict(tokenAt(input, 7), &rightScale) ||
          !setMissionWheelVelocityTrim(leftScale, rightScale, leftScale, rightScale)) {
        DEBUG_SERIAL.println(F("사용법: drive balance test <방향> <raw> <ms> <leftScale> <rightScale>"));
        return false;
      }
    }

    int32_t flRaw = 0;
    int32_t frRaw = 0;
    int32_t blRaw = 0;
    int32_t brRaw = 0;
    buildDirectionalVelocityRaw(direction, (int32_t)raw, &flRaw, &frRaw, &blRaw, &brRaw);
    DEBUG_SERIAL.print(F("[drive balance test] raw FL/FR/BL/BR="));
    DEBUG_SERIAL.print(flRaw);
    DEBUG_SERIAL.print(F("/"));
    DEBUG_SERIAL.print(frRaw);
    DEBUG_SERIAL.print(F("/"));
    DEBUG_SERIAL.print(blRaw);
    DEBUG_SERIAL.print(F("/"));
    DEBUG_SERIAL.print(brRaw);
    DEBUG_SERIAL.print(F(", ms="));
    DEBUG_SERIAL.println(durationMs);
    DEBUG_SERIAL.print(F("{\"type\":\"drive-balance-test\",\"raw\":"));
    DEBUG_SERIAL.print(raw);
    DEBUG_SERIAL.print(F(",\"durationMs\":"));
    DEBUG_SERIAL.print(durationMs);
    DEBUG_SERIAL.print(F(",\"trim\":{\"fl\":"));
    DEBUG_SERIAL.print(missionWheelVelocityTrimFl, 3);
    DEBUG_SERIAL.print(F(",\"fr\":"));
    DEBUG_SERIAL.print(missionWheelVelocityTrimFr, 3);
    DEBUG_SERIAL.print(F(",\"bl\":"));
    DEBUG_SERIAL.print(missionWheelVelocityTrimBl, 3);
    DEBUG_SERIAL.print(F(",\"br\":"));
    DEBUG_SERIAL.print(missionWheelVelocityTrimBr, 3);
    DEBUG_SERIAL.println(F("}}"));

    ChangeMobilebaseMode2VelocityControlMode(dxl);
    SetMobileGoalVelocityForSyncWrite(dxl, flRaw, frRaw, blRaw, brRaw);
    unsigned long startedAt = millis();
    while (millis() - startedAt < (unsigned long)durationMs) {
      if (checkEmergencyStopInput()) {
        stopAll(F("[긴급정지] ! 입력"));
        return false;
      }
      delay(10);
    }
    stopMobilebase();
    return true;
  }

  DEBUG_SERIAL.println(F("사용법: drive balance status|reset|set|test"));
  return false;
}

bool commandDriveTrim(const String &input) {
  if (!ensureMobileReady()) return false;

  uint8_t direction = DRIVE_DIRECTION_FORWARD;
  long distanceMm = 0;
  long speedMmPerSec = 0;
  if (!parseDriveDirection(tokenAt(input, 2), &direction) ||
      !parseLongStrict(tokenAt(input, 3), &distanceMm) ||
      !parseLongStrict(tokenAt(input, 4), &speedMmPerSec)) {
    DEBUG_SERIAL.println(F("사용법: drive trim <forward|back|left|right> <mm> <mmPerSec> <leftScale> <rightScale>"));
    DEBUG_SERIAL.println(F("예시: drive trim 후진 200 100 1.00 0.95"));
    return false;
  }

  float leftScale = 1.0;
  float rightScale = 1.0;
  float flScale = 1.0;
  float frScale = 1.0;
  float blScale = 1.0;
  float brScale = 1.0;
  bool wheelMode = false;

  if (tokenCount(input) >= 9) {
    wheelMode = true;
    if (!parseFloatStrict(tokenAt(input, 5), &flScale) ||
        !parseFloatStrict(tokenAt(input, 6), &frScale) ||
        !parseFloatStrict(tokenAt(input, 7), &blScale) ||
        !parseFloatStrict(tokenAt(input, 8), &brScale)) {
      DEBUG_SERIAL.println(F("사용법: drive trim <방향> <mm> <mmPerSec> <fl> <fr> <bl> <br>"));
      return false;
    }
  } else if (tokenCount(input) >= 7) {
    if (!parseFloatStrict(tokenAt(input, 5), &leftScale) ||
        !parseFloatStrict(tokenAt(input, 6), &rightScale)) {
      DEBUG_SERIAL.println(F("사용법: drive trim <방향> <mm> <mmPerSec> <leftScale> <rightScale>"));
      return false;
    }
    flScale = leftScale;
    blScale = leftScale;
    frScale = rightScale;
    brScale = rightScale;
  } else {
    DEBUG_SERIAL.println(F("사용법: drive trim <방향> <mm> <mmPerSec> <leftScale> <rightScale>"));
    DEBUG_SERIAL.println(F("또는: drive trim <방향> <mm> <mmPerSec> <fl> <fr> <bl> <br>"));
    return false;
  }

  if (!scaleWithinTrimLimit(flScale) || !scaleWithinTrimLimit(frScale) ||
      !scaleWithinTrimLimit(blScale) || !scaleWithinTrimLimit(brScale)) {
    DEBUG_SERIAL.println(F("[제한] 보정 비율은 0.70~1.30 범위만 허용합니다."));
    return false;
  }

  if (!checkDriveLimits(distanceMm, speedMmPerSec)) return false;

  int32_t baseRaw = RADIANS_2_MOTOR_VALUE((float)distanceMm / WHEEL_RADIUS_MM);
  int32_t flRaw = baseRaw;
  int32_t frRaw = baseRaw;
  int32_t blRaw = baseRaw;
  int32_t brRaw = baseRaw;

  switch (direction) {
    case DRIVE_DIRECTION_BACKWARD:
      flRaw = -baseRaw;
      frRaw = -baseRaw;
      blRaw = -baseRaw;
      brRaw = -baseRaw;
      break;
    case DRIVE_DIRECTION_LEFT:
      flRaw = -baseRaw;
      frRaw = baseRaw;
      blRaw = baseRaw;
      brRaw = -baseRaw;
      break;
    case DRIVE_DIRECTION_RIGHT:
      flRaw = baseRaw;
      frRaw = -baseRaw;
      blRaw = -baseRaw;
      brRaw = baseRaw;
      break;
    case DRIVE_DIRECTION_FORWARD:
    default:
      break;
  }

  flRaw = scaledWheelRaw(flRaw, flScale);
  frRaw = scaledWheelRaw(frRaw, frScale);
  blRaw = scaledWheelRaw(blRaw, blScale);
  brRaw = scaledWheelRaw(brRaw, brScale);

  unsigned long estimatedMs = (unsigned long)distanceMm * 1000UL / (unsigned long)speedMmPerSec;
  DEBUG_SERIAL.println(F("[drive trim]"));
  DEBUG_SERIAL.print(F("  mode="));
  DEBUG_SERIAL.println(wheelMode ? F("개별 바퀴") : F("좌/우"));
  DEBUG_SERIAL.print(F("  fl=")); DEBUG_SERIAL.print(flScale, 3);
  DEBUG_SERIAL.print(F(", fr=")); DEBUG_SERIAL.print(frScale, 3);
  DEBUG_SERIAL.print(F(", bl=")); DEBUG_SERIAL.print(blScale, 3);
  DEBUG_SERIAL.print(F(", br=")); DEBUG_SERIAL.println(brScale, 3);
  DEBUG_SERIAL.print(F("  raw fl=")); DEBUG_SERIAL.print(flRaw);
  DEBUG_SERIAL.print(F(", fr=")); DEBUG_SERIAL.print(frRaw);
  DEBUG_SERIAL.print(F(", bl=")); DEBUG_SERIAL.print(blRaw);
  DEBUG_SERIAL.print(F(", br=")); DEBUG_SERIAL.println(brRaw);
  DEBUG_SERIAL.print(F("  예상 시간="));
  DEBUG_SERIAL.print(estimatedMs);
  DEBUG_SERIAL.println(F("ms"));

  ChangeMobilebaseMode2ExtendedPositionControlWithTimeBasedProfileMode(dxl);
  SetMobileRelativePositionForSyncWrite(dxl, flRaw, frRaw, blRaw, brRaw, estimatedMs);
  return waitForDriveCompletion(estimatedMs);
}

bool commandPixy(const String &input) {
  String sub = tokenAt(input, 1);
  sub.toLowerCase();

  if (sub == "storage" || sub == "rack" || sub == "적재함") {
    return commandPixyStorageMap(input);
  }

  if (sub == "align" || sub == "정렬") {
    return commandPixyAlignGripTarget(input);
  }

  if (sub == "alignslow" || sub == "slowalign" || sub == "저속정렬") {
    return commandPixyAlignGripTargetSlow(input);
  }

  if (sub == "scan" || sub == "스캔" || sub.length() == 0) {
    uint8_t signatureMap = 0x7F;
    long frames = 3;
    if (tokenCount(input) >= 3 && !parseSignatureMapText(tokenAt(input, 2), &signatureMap)) {
      DEBUG_SERIAL.println(F("사용법: pixy scan [signatureMap|0b1111111|all] [frames]"));
      return false;
    }
    if (tokenCount(input) >= 4 && !parseLongStrict(tokenAt(input, 3), &frames)) {
      DEBUG_SERIAL.println(F("[오류] frames는 숫자로 입력하세요."));
      return false;
    }
    return commandPixyScan(signatureMap, (uint8_t)constrain(frames, 1, 20));
  }

  if (sub == "sig" || sub == "signature" || sub == "시그니처") {
    long signature = 0;
    long frames = 3;
    if (!parseLongStrict(tokenAt(input, 2), &signature) || signature < 1 || signature > 7) {
      DEBUG_SERIAL.println(F("사용법: pixy sig <1~7> [frames]"));
      return false;
    }
    if (tokenCount(input) >= 4 && !parseLongStrict(tokenAt(input, 3), &frames)) {
      DEBUG_SERIAL.println(F("[오류] frames는 숫자로 입력하세요."));
      return false;
    }
    uint8_t signatureMap = (uint8_t)(1 << (signature - 1));
    return commandPixyScan(signatureMap, (uint8_t)constrain(frames, 1, 20));
  }

  if (sub == "watch" || sub == "stability" || sub == "안정도" || sub == "관찰") {
    return commandPixyWatch(input);
  }

  if (sub == "sweep" || sub == "밝기탐색" || sub == "탐색") {
    return commandPixySweep(input);
  }

  if (sub == "lamp" || sub == "조명") {
    String mode = tokenAt(input, 2);
    mode.toLowerCase();
    if (mode == "on" || mode == "켜기") {
      pixy.setLamp(1, 1);
      DEBUG_SERIAL.println(F("[Pixy 조명] 켜기"));
      return true;
    }
    if (mode == "off" || mode == "끄기") {
      pixy.setLamp(0, 0);
      DEBUG_SERIAL.println(F("[Pixy 조명] 끄기"));
      return true;
    }
    DEBUG_SERIAL.println(F("사용법: pixy lamp on|off"));
    return false;
  }

  if (sub == "brightness" || sub == "밝기") {
    long brightness = 0;
    if (!parseLongStrict(tokenAt(input, 2), &brightness) || brightness < 0 || brightness > 255) {
      DEBUG_SERIAL.println(F("사용법: pixy brightness <0~255>"));
      DEBUG_SERIAL.println(F("예시: pixy brightness 80"));
      return false;
    }
    int8_t result = pixy.setCameraBrightness((uint8_t)brightness);
    DEBUG_SERIAL.print(F("[Pixy 밝기] "));
    DEBUG_SERIAL.print(brightness);
    DEBUG_SERIAL.print(F(", result="));
    DEBUG_SERIAL.println(result);
    return result >= 0;
  }

  if (sub == "fps" || sub == "프레임") {
    int8_t fps = pixy.getFPS();
    DEBUG_SERIAL.print(F("[Pixy FPS] "));
    DEBUG_SERIAL.println(fps);
    if (fps < 0) {
      DEBUG_SERIAL.println(F("[주의] FPS 읽기에 실패했습니다."));
      return false;
    }
    if (fps < 30) {
      DEBUG_SERIAL.println(F("[판정] FPS가 낮습니다. 조명 부족 또는 노출 설정을 확인하세요."));
    }
    return true;
  }

  DEBUG_SERIAL.println(F("pixy scan|sig|watch|sweep|lamp|brightness|fps"));
  return false;
}

bool commandGripCycle(const String &input) {
  long repeat = 1;
  long holdMs = 500;
  if (tokenCount(input) >= 3 && !parseLongStrict(tokenAt(input, 2), &repeat)) {
    DEBUG_SERIAL.println(F("사용법: grip cycle [repeat] [holdMs]"));
    return false;
  }
  if (tokenCount(input) >= 4 && !parseLongStrict(tokenAt(input, 3), &holdMs)) {
    DEBUG_SERIAL.println(F("사용법: grip cycle [repeat] [holdMs]"));
    return false;
  }
  repeat = constrain(repeat, 1, 5);
  holdMs = constrain(holdMs, 200, 2000);

  for (uint8_t i = 0; i < repeat; i++) {
    DEBUG_SERIAL.print(F("[집게 테스트] "));
    DEBUG_SERIAL.print(i + 1);
    DEBUG_SERIAL.print(F("/"));
    DEBUG_SERIAL.println(repeat);
    OpenGripper(pixy);
    if (!interruptibleDelay(holdMs)) return false;
    CloseGripper(pixy);
    if (!interruptibleDelay(holdMs)) return false;
  }
  OpenGripper(pixy);
  DEBUG_SERIAL.print(F("[집게 테스트 완료] 열림값="));
  DEBUG_SERIAL.print(GRIP_ANGLE_OPEN);
  DEBUG_SERIAL.print(F(", 닫힘값="));
  DEBUG_SERIAL.println(GRIP_ANGLE_CLOSE);
  turnOnLEDGreen500ms();
  return true;
}

bool parseGripServoValue(String text, uint16_t *value) {
  long parsed = 0;
  if (!parseLongStrict(text, &parsed) || parsed < 0 || parsed > 1000) {
    return false;
  }
  *value = (uint16_t)parsed;
  return true;
}

bool commandGripSet(const String &input) {
  uint16_t value = 0;
  if (!parseGripServoValue(tokenAt(input, 2), &value)) {
    DEBUG_SERIAL.println(F("사용법: grip set <0~1000>"));
    DEBUG_SERIAL.println(F("예시: grip set 600"));
    return false;
  }
  pixy.setServos(0, value);
  DEBUG_SERIAL.print(F("[집게 위치] "));
  DEBUG_SERIAL.println(value);
  DEBUG_SERIAL.print(F("{\"type\":\"grip-set\",\"servo\":"));
  DEBUG_SERIAL.print(value);
  DEBUG_SERIAL.println(F("}"));
  return true;
}

bool commandGripTest(const String &input) {
  uint16_t openValue = GRIP_ANGLE_OPEN;
  uint16_t closeValue = GRIP_ANGLE_CLOSE;
  long repeat = 1;
  long holdMs = 500;

  if (tokenCount(input) >= 4) {
    if (!parseGripServoValue(tokenAt(input, 2), &openValue) ||
        !parseGripServoValue(tokenAt(input, 3), &closeValue)) {
      DEBUG_SERIAL.println(F("사용법: grip test [openValue closeValue] [repeat] [holdMs]"));
      DEBUG_SERIAL.println(F("예시: grip test 100 650 2 500"));
      return false;
    }
  }
  if (tokenCount(input) >= 5 && !parseLongStrict(tokenAt(input, 4), &repeat)) {
    DEBUG_SERIAL.println(F("[오류] repeat는 숫자로 입력하세요."));
    return false;
  }
  if (tokenCount(input) >= 6 && !parseLongStrict(tokenAt(input, 5), &holdMs)) {
    DEBUG_SERIAL.println(F("[오류] holdMs는 숫자로 입력하세요."));
    return false;
  }

  repeat = constrain(repeat, 1, 5);
  holdMs = constrain(holdMs, 200, 2000);

  DEBUG_SERIAL.print(F("[집게 각도 테스트] open="));
  DEBUG_SERIAL.print(openValue);
  DEBUG_SERIAL.print(F(", close="));
  DEBUG_SERIAL.print(closeValue);
  DEBUG_SERIAL.print(F(", repeat="));
  DEBUG_SERIAL.print(repeat);
  DEBUG_SERIAL.print(F(", holdMs="));
  DEBUG_SERIAL.println(holdMs);

  for (uint8_t i = 0; i < repeat; i++) {
    pixy.setServos(0, openValue);
    if (!interruptibleDelay(holdMs)) return false;
    pixy.setServos(0, closeValue);
    if (!interruptibleDelay(holdMs)) return false;
  }
  pixy.setServos(0, openValue);

  DEBUG_SERIAL.print(F("{\"type\":\"grip-test\",\"open\":"));
  DEBUG_SERIAL.print(openValue);
  DEBUG_SERIAL.print(F(",\"close\":"));
  DEBUG_SERIAL.print(closeValue);
  DEBUG_SERIAL.print(F(",\"repeat\":"));
  DEBUG_SERIAL.print(repeat);
  DEBUG_SERIAL.println(F("}"));
  turnOnLEDGreen500ms();
  return true;
}

bool waitForMobilePositionOrTimeout(unsigned long timeoutMs, const __FlashStringHelper *timeoutMessage) {
  unsigned long startedAt = millis();
  while (!CheckIfMobilebaseIsInPosition(dxl)) {
    if (checkEmergencyStopInput()) {
      stopAll(F("[긴급정지] ! 입력"));
      return false;
    }
    if (millis() - startedAt > timeoutMs) {
      DEBUG_SERIAL.println(timeoutMessage);
      stopMobilebase();
      return false;
    }
    delay(10);
  }
  stopMobilebase();
  return true;
}

void driveForwardWithInstructionSideCorrection(int16_t slVal) {
  int16_t sideError = slVal - missionInstructionSl;
  int32_t sideCorrection = 0;
  if (abs(sideError) > CFG.psd.missionTolerance) {
    sideCorrection = (int32_t)sideError * missionMotion.psdCorrectionSpeed /
                     max((int32_t)1, (int32_t)CFG.psd.missionTolerance * 5);
    sideCorrection = clampInt32(sideCorrection,
                                -missionMotion.psdCorrectionSpeed,
                                missionMotion.psdCorrectionSpeed);
  }

  int32_t maxVelocity = profile().maxMissionVelocityRaw;
  int32_t base = missionInstructionFinalForwardSpeed;
  int32_t flSpeed = clampInt32(base + sideCorrection, -maxVelocity, maxVelocity);
  int32_t frSpeed = clampInt32(base - sideCorrection, -maxVelocity, maxVelocity);
  int32_t blSpeed = clampInt32(base - sideCorrection, -maxVelocity, maxVelocity);
  int32_t brSpeed = clampInt32(base + sideCorrection, -maxVelocity, maxVelocity);
  SetMobileGoalVelocityForSyncWrite(dxl, flSpeed, frSpeed, blSpeed, brSpeed);
}

bool runInstructionFinalForwardCorrection() {
  if (missionInstructionFinalForwardMs == 0) return true;

  uint8_t signatureMap = missionInstructionSignatureMap();
  uint16_t minArea = CFG.cameraScan.missionInstructionMinBlockArea;
  int16_t bestX[MissionConfig::MAX_MISSION_BLOCKS] = {0};
  uint8_t bestSig[MissionConfig::MAX_MISSION_BLOCKS] = {0};
  uint8_t bestCount = 0;
  uint32_t bestAreaScore = 0;
  unsigned long lastScanAt = 0;

  DEBUG_SERIAL.print(F("  미션지시존 최종 직진 보정 "));
  DEBUG_SERIAL.print(missionInstructionFinalForwardMs);
  DEBUG_SERIAL.print(F("ms, target SL="));
  DEBUG_SERIAL.print(missionInstructionSl);
  DEBUG_SERIAL.print(F(", forward raw="));
  DEBUG_SERIAL.println(missionInstructionFinalForwardSpeed);
  if (signatureMap != 0) {
    uint8_t lamp = CFG.cameraScan.missionInstructionLampOn ? 1 : 0;
    pixy.setLamp(lamp, lamp);
    DEBUG_SERIAL.print(F("  주행 중 Pixy scan signatureMap="));
    printSignatureMap(signatureMap);
    DEBUG_SERIAL.print(F(", minArea="));
    DEBUG_SERIAL.println(minArea);
  }

  ChangeMobilebaseMode2VelocityControlMode(dxl);
  unsigned long startedAt = millis();
  int16_t slVal = 0;
  while (millis() - startedAt < missionInstructionFinalForwardMs) {
    if (checkEmergencyStopInput()) {
      stopAll(F("[긴급정지] ! 입력"));
      return false;
    }
    readSideLeftPSDSensor(&slVal);
    driveForwardWithInstructionSideCorrection(slVal);
    unsigned long now = millis();
    if (signatureMap != 0 &&
        (lastScanAt == 0 || now - lastScanAt >= missionInstructionScanSampleMs)) {
      lastScanAt = now;
      updateMissionInstructionBestFromPixy(signatureMap, minArea,
                                           bestX, bestSig,
                                           &bestCount, &bestAreaScore);
    }
    delay(10);
  }
  stopMobilebase();
  DEBUG_SERIAL.print(F("  최종 직진 보정 종료 SL="));
  DEBUG_SERIAL.println(slVal);
  if (signatureMap != 0 && bestCount > 0) {
    DEBUG_SERIAL.println(F("[mission scan] 주행 중 스캔 결과로 queue를 갱신합니다."));
    commitMissionInstructionQueue(bestX, bestSig, bestCount);
  } else if (signatureMap != 0) {
    DEBUG_SERIAL.println(F("[mission scan] 주행 중 감지 없음. 필요하면 mission rescan으로 정지 스캔하세요."));
  }
  return interruptibleDelay(CFG.wait.driveSettleMs);
}

bool commandMissionApproachInstructionZone() {
  if (!ensureMobileReady()) return false;

  DEBUG_SERIAL.println(F("[mission move] 미션지시존 접근"));
  DEBUG_SERIAL.print(F("  front firstDetectAdc="));
  DEBUG_SERIAL.print(missionFrontFirstDetectAdc);
  DEBUG_SERIAL.print(F(", decelWindow="));
  DEBUG_SERIAL.println(missionFrontDecelWindowAdc);
  DEBUG_SERIAL.print(F("  afterDetect="));
  DEBUG_SERIAL.print(missionFrontAfterDetectMm, 2);
  DEBUG_SERIAL.print(F("mm @ "));
  DEBUG_SERIAL.print(missionFrontAfterDetectMmPerSec);
  DEBUG_SERIAL.println(F(" raw"));
  DEBUG_SERIAL.print(F("  speed cruise/slow="));
  DEBUG_SERIAL.print(missionMotion.frontCruiseSpeed);
  DEBUG_SERIAL.print(F("/"));
  DEBUG_SERIAL.println(missionMotion.frontSlowSpeed);
  DEBUG_SERIAL.print(F("  instruction SL target="));
  DEBUG_SERIAL.print(missionInstructionSl);
  DEBUG_SERIAL.print(F(", finalForwardMs="));
  DEBUG_SERIAL.print(missionInstructionFinalForwardMs);
  DEBUG_SERIAL.print(F(", finalForwardRaw="));
  DEBUG_SERIAL.println(missionInstructionFinalForwardSpeed);

  ChangeMobilebaseMode2VelocityControlMode(dxl);

  int16_t flVal = 0;
  int16_t frVal = 0;
  int32_t currentForwardSpeed = missionMotion.frontCruiseSpeed;
  bool frontDetected = false;
  unsigned long startedAt = millis();

  while (true) {
    if (checkEmergencyStopInput()) {
      stopAll(F("[긴급정지] ! 입력"));
      return false;
    }

    readFrontPSDSensors(&flVal, &frVal);
    int16_t frontDetectVal = max(flVal, frVal);
    if (frontDetectVal >= missionFrontFirstDetectAdc) {
      frontDetected = true;
      DEBUG_SERIAL.print(F("  첫 감지 front="));
      DEBUG_SERIAL.print(frontDetectVal);
      DEBUG_SERIAL.print(F(" FL="));
      DEBUG_SERIAL.print(flVal);
      DEBUG_SERIAL.print(F(" FR="));
      DEBUG_SERIAL.println(frVal);
      break;
    }

    currentForwardSpeed = (frontDetectVal >= missionFrontFirstDetectAdc - missionFrontDecelWindowAdc)
                            ? missionMotion.frontSlowSpeed
                            : missionMotion.frontCruiseSpeed;
    SetMobileGoalVelocityForSyncWrite(dxl, currentForwardSpeed, currentForwardSpeed,
                                      currentForwardSpeed, currentForwardSpeed);

    if (millis() - startedAt > CFG.timeout.psdLoopMs) {
      DEBUG_SERIAL.println(F("  [2-1] 첫 감지 타임아웃"));
      break;
    }
    delay(10);
  }

  if (frontDetected && missionFrontAfterDetectMm > 0.0) {
    unsigned long continueMs =
      (unsigned long)round(missionFrontAfterDetectMm * 1000.0 /
                           max((int32_t)1, missionFrontAfterDetectMmPerSec));
    currentForwardSpeed = min(missionMotion.frontSlowSpeed, missionFrontAfterDetectMmPerSec);
    DEBUG_SERIAL.print(F("  첫 감지 후 이어서 저속 전진 "));
    DEBUG_SERIAL.print(missionFrontAfterDetectMm, 2);
    DEBUG_SERIAL.print(F("mm 상당, "));
    DEBUG_SERIAL.print(continueMs);
    DEBUG_SERIAL.print(F("ms @ raw "));
    DEBUG_SERIAL.println(currentForwardSpeed);

    unsigned long continueStartedAt = millis();
    while (millis() - continueStartedAt < continueMs) {
      if (checkEmergencyStopInput()) {
        stopAll(F("[긴급정지] ! 입력"));
        return false;
      }
      SetMobileGoalVelocityForSyncWrite(dxl, currentForwardSpeed, currentForwardSpeed,
                                        currentForwardSpeed, currentForwardSpeed);
      readFrontPSDSensors(&flVal, &frVal);
      delay(10);
    }
  }

  int32_t decelStartSpeed = max((int32_t)1, currentForwardSpeed);
  for (int32_t speed = decelStartSpeed; speed > 0; speed -= CFG.front.brakeStep) {
    if (checkEmergencyStopInput()) {
      stopAll(F("[긴급정지] ! 입력"));
      return false;
    }
    SetMobileGoalVelocityForSyncWrite(dxl, speed, speed, speed, speed);
    delay(CFG.front.brakeDelayMs / 2);
  }
  stopMobilebase();

  if (!frontDetected) {
    DEBUG_SERIAL.println(F("  [주의] 첫 감지 없이 정지했습니다. 위치를 확인하세요."));
    return false;
  }
  DEBUG_SERIAL.print(F("  접근 종료 FL="));
  DEBUG_SERIAL.print(flVal);
  DEBUG_SERIAL.print(F(" FR="));
  DEBUG_SERIAL.println(frVal);
  if (!interruptibleDelay(CFG.wait.driveSettleMs)) return false;

  DEBUG_SERIAL.println(F("  SL PSD로 미션지시존 열 정렬"));
  DEBUG_SERIAL.print(F("  목표 SL="));
  DEBUG_SERIAL.print(missionInstructionSl);
  DEBUG_SERIAL.print(F(", tolerance="));
  DEBUG_SERIAL.print(CFG.psd.missionTolerance);
  DEBUG_SERIAL.print(F(", speed="));
  DEBUG_SERIAL.println(missionMotion.psdCorrectionSpeed);
  int16_t slVal = 0;
  startedAt = millis();
  while (true) {
    if (checkEmergencyStopInput()) {
      stopAll(F("[긴급정지] ! 입력"));
      return false;
    }
    readSideLeftPSDSensor(&slVal);
    if (!DriveWithOneSensor(dxl, slVal - missionInstructionSl,
                            CFG.psd.missionTolerance, DRIVE_DIRECTION_LEFT,
                            missionMotion.psdCorrectionSpeed)) {
      break;
    }
    if (millis() - startedAt > CFG.timeout.psdLoopMs) {
      DEBUG_SERIAL.println(F("  [2-2] 미션지시존 정렬 타임아웃"));
      stopMobilebase();
      return false;
    }
    delay(10);
  }
  stopMobilebase();
  DEBUG_SERIAL.print(F("  미션지시존 정렬 종료 SL="));
  DEBUG_SERIAL.println(slVal);
  clearMissionUndoCandidate();
  if (!interruptibleDelay(CFG.wait.driveSettleMs)) return false;
  return runInstructionFinalForwardCorrection();
}

bool commandMissionStoragePose() {
  if (!ensureMobileReady()) return false;
  return commandSeq(String("seq storage"));
}

void clearMissionUndoCandidate() {
  missionUndoAvailable = false;
  missionUndoDistanceMm = 0.0;
  missionUndoDirection = 0;
  missionUndoSpeedMmPerSec = 0;
}

uint8_t reverseDriveDirection(uint8_t direction) {
  switch (direction) {
    case DRIVE_DIRECTION_FORWARD: return DRIVE_DIRECTION_BACKWARD;
    case DRIVE_DIRECTION_BACKWARD: return DRIVE_DIRECTION_FORWARD;
    case DRIVE_DIRECTION_LEFT: return DRIVE_DIRECTION_RIGHT;
    case DRIVE_DIRECTION_RIGHT: return DRIVE_DIRECTION_LEFT;
  }
  return direction;
}

void rememberMissionFixedMoveUndo(float distanceMm, uint8_t direction, int32_t speedMmPerSec) {
  missionUndoAvailable = true;
  missionUndoDistanceMm = distanceMm;
  missionUndoDirection = reverseDriveDirection(direction);
  missionUndoSpeedMmPerSec = speedMmPerSec;
}

bool commandMissionDriveDistanceStep(const __FlashStringHelper *label,
                                     float distanceMm,
                                     uint8_t direction,
                                     int32_t speedMmPerSec,
                                     bool recordUndo = true) {
  if (!ensureMobileReady()) return false;

  DEBUG_SERIAL.print(label);
  DEBUG_SERIAL.print(F(" "));
  DEBUG_SERIAL.print(distanceMm);
  DEBUG_SERIAL.print(F("mm @ "));
  DEBUG_SERIAL.print(speedMmPerSec);
  DEBUG_SERIAL.println(F("mm/s"));

  ChangeMobilebaseMode2ExtendedPositionControlWithTimeBasedProfileMode(dxl);
  DriveDistanceAndMmPerSecAndDirection(dxl, distanceMm, direction, speedMmPerSec);
  bool ok = waitForMobilePositionOrTimeout(CFG.timeout.positionMoveMs, F("  주행 타임아웃"));
  if (ok && recordUndo) {
    rememberMissionFixedMoveUndo(distanceMm, direction, speedMmPerSec);
  }
  return ok;
}

bool commandMissionJog(const String &input) {
  uint8_t direction = DRIVE_DIRECTION_FORWARD;
  long distanceMm = 0;
  long speedMmPerSec = 0;
  if (!parseDriveDirection(tokenAt(input, 2), &direction) ||
      !parseLongStrict(tokenAt(input, 3), &distanceMm) ||
      !parseLongStrict(tokenAt(input, 4), &speedMmPerSec)) {
    DEBUG_SERIAL.println(F("사용법: mission jog <forward|back|left|right> <mm> <mm/s>"));
    DEBUG_SERIAL.println(F("예시: mission jog right 20 80"));
    return false;
  }
  if (!checkDriveLimits(distanceMm, speedMmPerSec)) return false;

  DEBUG_SERIAL.println(F("[mission jog] 정위치 보정용 임의 이동입니다. logical column은 변경하지 않습니다."));
  DEBUG_SERIAL.print(F("{\"type\":\"mission-jog\",\"distanceMm\":"));
  DEBUG_SERIAL.print(distanceMm);
  DEBUG_SERIAL.print(F(",\"speedMmPerSec\":"));
  DEBUG_SERIAL.print(speedMmPerSec);
  DEBUG_SERIAL.println(F("}"));
  bool ok = commandMissionDriveDistanceStep(F("  jog 이동"),
                                            (float)distanceMm,
                                            direction,
                                            speedMmPerSec);
  if (ok) {
    missionColumnSearchMissColumn = 0;
    clearMissionColumnScanDecision();
  }
  return ok;
}

bool commandMissionColumnNudge(uint8_t direction, uint8_t steps) {
  if (steps < 1) steps = 1;
  if (steps > 3) steps = 3;

  int8_t delta = (direction == DRIVE_DIRECTION_RIGHT) ? (int8_t)steps : -(int8_t)steps;
  int16_t nextColumn = (int16_t)missionStorageColumn + delta;
  if (missionStorageColumn < 1 || missionStorageColumn > 4) {
    DEBUG_SERIAL.println(F("[mission column] 현재 column 기준이 비정상입니다. 먼저 mission column <1~4>로 지정하세요."));
    return false;
  }
  if (nextColumn < 1 || nextColumn > 4) {
    DEBUG_SERIAL.println(F("[mission column] 1~4열 범위를 벗어나는 이동은 막았습니다."));
    return false;
  }

  float distanceMm = missionColumnStepMm * (float)steps;
  long distanceForLimit = (long)(distanceMm + 0.5);
  if (!checkDriveLimits(distanceForLimit, missionColumnMoveMmPerSec)) return false;

  const __FlashStringHelper *label = (direction == DRIVE_DIRECTION_RIGHT)
                                      ? F("  column right")
                                      : F("  column left");
  DEBUG_SERIAL.print(F("[mission column] "));
  DEBUG_SERIAL.print(missionStorageColumn);
  DEBUG_SERIAL.print(F(" -> "));
  DEBUG_SERIAL.print(nextColumn);
  DEBUG_SERIAL.print(F(", stepMm="));
  DEBUG_SERIAL.print(missionColumnStepMm, 2);
  DEBUG_SERIAL.print(F(", steps="));
  DEBUG_SERIAL.print(steps);
  DEBUG_SERIAL.print(F(", speed="));
  DEBUG_SERIAL.println(missionColumnMoveMmPerSec);

  bool ok = commandMissionDriveDistanceStep(label,
                                            distanceMm,
                                            direction,
                                            missionColumnMoveMmPerSec);
  if (ok) {
    missionStorageColumn = (uint8_t)nextColumn;
    missionStorageScanTargetColumn = missionStorageColumn;
    missionColumnSearchMissColumn = 0;
    clearMissionColumnScanDecision();
    DEBUG_SERIAL.print(F("{\"type\":\"mission-column-nudge\",\"currentColumn\":"));
    DEBUG_SERIAL.print(missionStorageColumn);
    DEBUG_SERIAL.print(F(",\"stepMm\":"));
    DEBUG_SERIAL.print(missionColumnStepMm, 2);
    DEBUG_SERIAL.print(F(",\"steps\":"));
    DEBUG_SERIAL.print(steps);
    DEBUG_SERIAL.println(F("}"));
  }
  return ok;
}

bool validateMissionStoragePathValue(float distanceMm, const __FlashStringHelper *label) {
  if (distanceMm <= 0.0 || distanceMm > (float)profile().maxDriveMm) {
    DEBUG_SERIAL.print(F("[제한] "));
    DEBUG_SERIAL.print(label);
    DEBUG_SERIAL.print(F(" 값은 0~"));
    DEBUG_SERIAL.print(profile().maxDriveMm);
    DEBUG_SERIAL.println(F("mm 범위에서 테스트하세요."));
    return false;
  }
  return true;
}

bool validateMissionOptionalDistance(float distanceMm, const __FlashStringHelper *label) {
  if (distanceMm < 0.0 || distanceMm > (float)profile().maxDriveMm) {
    DEBUG_SERIAL.print(F("[제한] "));
    DEBUG_SERIAL.print(label);
    DEBUG_SERIAL.print(F(" 값은 0~"));
    DEBUG_SERIAL.print(profile().maxDriveMm);
    DEBUG_SERIAL.println(F("mm 범위에서 테스트하세요."));
    return false;
  }
  return true;
}

void printMissionApproachStatus() {
  DEBUG_SERIAL.print(F("[mission approach] firstDetectAdc="));
  DEBUG_SERIAL.print(missionFrontFirstDetectAdc);
  DEBUG_SERIAL.print(F(", decelWindowAdc="));
  DEBUG_SERIAL.print(missionFrontDecelWindowAdc);
  DEBUG_SERIAL.print(F(", afterDetect="));
  DEBUG_SERIAL.print(missionFrontAfterDetectMm, 2);
  DEBUG_SERIAL.print(F("mm @ "));
  DEBUG_SERIAL.print(missionFrontAfterDetectMmPerSec);
  DEBUG_SERIAL.println(F(" raw"));
  DEBUG_SERIAL.print(F("{\"type\":\"mission-approach\",\"firstDetectAdc\":"));
  DEBUG_SERIAL.print(missionFrontFirstDetectAdc);
  DEBUG_SERIAL.print(F(",\"decelWindowAdc\":"));
  DEBUG_SERIAL.print(missionFrontDecelWindowAdc);
  DEBUG_SERIAL.print(F(",\"afterDetectMm\":"));
  DEBUG_SERIAL.print(missionFrontAfterDetectMm, 2);
  DEBUG_SERIAL.print(F(",\"afterDetectRaw\":"));
  DEBUG_SERIAL.print(missionFrontAfterDetectMmPerSec);
  DEBUG_SERIAL.println(F("}"));
}

bool commandMissionApproachTuning(const String &input) {
  if (tokenCount(input) < 4) {
    printMissionApproachStatus();
    DEBUG_SERIAL.println(F("사용법: mission approach <afterDetectMm> <continueRaw> [firstDetectAdc] [decelWindowAdc]"));
    DEBUG_SERIAL.println(F("예시: mission approach 15 80"));
    DEBUG_SERIAL.println(F("센서값은 맞고 선만 덜 밟으면 afterDetectMm를 5~10씩 올려 테스트하세요."));
    return true;
  }

  float afterDetectMm = 0.0;
  long continueRaw = 0;
  long firstDetectAdc = missionFrontFirstDetectAdc;
  long decelWindowAdc = missionFrontDecelWindowAdc;
  if (!parseFloatStrict(tokenAt(input, 2), &afterDetectMm) ||
      !parseLongStrict(tokenAt(input, 3), &continueRaw)) {
    DEBUG_SERIAL.println(F("사용법: mission approach <afterDetectMm> <continueRaw> [firstDetectAdc] [decelWindowAdc]"));
    return false;
  }
  if (tokenCount(input) >= 5 &&
      !parseLongStrict(tokenAt(input, 4), &firstDetectAdc)) {
    DEBUG_SERIAL.println(F("사용법: mission approach <afterDetectMm> <continueRaw> [firstDetectAdc] [decelWindowAdc]"));
    return false;
  }
  if (tokenCount(input) >= 6 &&
      !parseLongStrict(tokenAt(input, 5), &decelWindowAdc)) {
    DEBUG_SERIAL.println(F("사용법: mission approach <afterDetectMm> <continueRaw> [firstDetectAdc] [decelWindowAdc]"));
    return false;
  }

  if (!validateMissionOptionalDistance(afterDetectMm, F("afterDetectMm")) ||
      !validateMissionVelocitySpeed(continueRaw)) {
    return false;
  }
  if (firstDetectAdc < 1 || firstDetectAdc > 1023 ||
      decelWindowAdc < 0 || decelWindowAdc > 400) {
    DEBUG_SERIAL.println(F("[제한] firstDetectAdc는 1~1023, decelWindowAdc는 0~400 범위입니다."));
    return false;
  }

  missionFrontAfterDetectMm = afterDetectMm;
  missionFrontAfterDetectMmPerSec = continueRaw;
  missionFrontFirstDetectAdc = (int16_t)firstDetectAdc;
  missionFrontDecelWindowAdc = (int16_t)decelWindowAdc;
  printMissionApproachStatus();
  return true;
}

void printMissionInstructionPathStatus() {
  DEBUG_SERIAL.print(F("[mission instruction] targetSl="));
  DEBUG_SERIAL.print(missionInstructionSl);
  DEBUG_SERIAL.print(F(", finalForwardMs="));
  DEBUG_SERIAL.print(missionInstructionFinalForwardMs);
  DEBUG_SERIAL.print(F(", finalForwardRaw="));
  DEBUG_SERIAL.println(missionInstructionFinalForwardSpeed);
  DEBUG_SERIAL.print(F("{\"type\":\"mission-instruction\",\"targetSl\":"));
  DEBUG_SERIAL.print(missionInstructionSl);
  DEBUG_SERIAL.print(F(",\"finalForwardMs\":"));
  DEBUG_SERIAL.print(missionInstructionFinalForwardMs);
  DEBUG_SERIAL.print(F(",\"finalForwardRaw\":"));
  DEBUG_SERIAL.print(missionInstructionFinalForwardSpeed);
  DEBUG_SERIAL.println(F("}"));
}

void printMissionInstructionScanStatus() {
  DEBUG_SERIAL.print(F("[mission scanrate] scanMs="));
  DEBUG_SERIAL.print(missionInstructionScanMs);
  DEBUG_SERIAL.print(F(", sampleMs="));
  DEBUG_SERIAL.print(missionInstructionScanSampleMs);
  DEBUG_SERIAL.print(F(", approxFrames="));
  DEBUG_SERIAL.println(max((uint16_t)1, (uint16_t)(missionInstructionScanMs / missionInstructionScanSampleMs)));
  DEBUG_SERIAL.print(F("{\"type\":\"mission-scanrate\",\"scanMs\":"));
  DEBUG_SERIAL.print(missionInstructionScanMs);
  DEBUG_SERIAL.print(F(",\"sampleMs\":"));
  DEBUG_SERIAL.print(missionInstructionScanSampleMs);
  DEBUG_SERIAL.println(F("}"));
}

bool commandMissionInstructionScanTiming(const String &input) {
  if (tokenCount(input) < 4) {
    printMissionInstructionScanStatus();
    DEBUG_SERIAL.println(F("사용법: mission scanrate <scanMs> <sampleMs>"));
    DEBUG_SERIAL.println(F("예시: mission scanrate 500 10"));
    DEBUG_SERIAL.println(F("거의 멈추지 않는 테스트: mission scanrate 200 10"));
    DEBUG_SERIAL.println(F("scanMs를 줄이면 미션지시존 signature queue 생성이 빨라집니다."));
    return true;
  }

  long scanMs = 0;
  long sampleMs = 0;
  if (!parseLongStrict(tokenAt(input, 2), &scanMs) ||
      !parseLongStrict(tokenAt(input, 3), &sampleMs)) {
    DEBUG_SERIAL.println(F("사용법: mission scanrate <scanMs> <sampleMs>"));
    return false;
  }
  if (scanMs < 100 || scanMs > 8000 ||
      sampleMs < 10 || sampleMs > 200 ||
      sampleMs > scanMs) {
    DEBUG_SERIAL.println(F("[제한] scanMs는 100~8000, sampleMs는 10~200이며 sampleMs <= scanMs이어야 합니다."));
    return false;
  }

  missionInstructionScanMs = (uint16_t)scanMs;
  missionInstructionScanSampleMs = (uint16_t)sampleMs;
  printMissionInstructionScanStatus();
  return true;
}

bool commandMissionInstructionPath(const String &input) {
  if (tokenCount(input) < 4) {
    printMissionInstructionPathStatus();
    DEBUG_SERIAL.println(F("사용법: mission instruction <targetSl> <finalForwardMs> <finalForwardRaw>"));
    DEBUG_SERIAL.println(F("예시: mission instruction 640 500 120"));
    DEBUG_SERIAL.println(F("선을 더 확실히 밟기: mission instruction 640 700 120"));
    DEBUG_SERIAL.println(F("SL 값이 클수록 왼쪽 장애물에 더 가깝습니다. finalForwardMs=0이면 최종 직진 보정 OFF입니다."));
    return true;
  }

  long targetSl = 0;
  long finalForwardMs = 0;
  long finalForwardRaw = 0;
  if (!parseLongStrict(tokenAt(input, 2), &targetSl) ||
      !parseLongStrict(tokenAt(input, 3), &finalForwardMs) ||
      !parseLongStrict(tokenAt(input, 4), &finalForwardRaw)) {
    DEBUG_SERIAL.println(F("사용법: mission instruction <targetSl> <finalForwardMs> <finalForwardRaw>"));
    return false;
  }
  if (targetSl < 1 || targetSl > 1023 ||
      finalForwardMs < 0 || finalForwardMs > 5000) {
    DEBUG_SERIAL.println(F("[제한] targetSl은 1~1023, finalForwardMs는 0~5000ms 범위입니다."));
    return false;
  }
  if (!validateMissionVelocitySpeed(finalForwardRaw)) return false;

  missionInstructionSl = (int16_t)targetSl;
  missionInstructionFinalForwardMs = (uint16_t)finalForwardMs;
  missionInstructionFinalForwardSpeed = finalForwardRaw;
  printMissionInstructionPathStatus();
  return true;
}

void printMissionAlignStatus() {
  DEBUG_SERIAL.print(F("[mission align] scan SL="));
  DEBUG_SERIAL.print(missionAlignSl);
  DEBUG_SERIAL.print(F(", FL="));
  DEBUG_SERIAL.print(missionAlignFl);
  DEBUG_SERIAL.print(F(", FR="));
  DEBUG_SERIAL.print(missionAlignFr);
  DEBUG_SERIAL.print(F(", tolerance="));
  DEBUG_SERIAL.print(missionAlignTolerance);
  DEBUG_SERIAL.println(F(" (SR ignored)"));
  DEBUG_SERIAL.print(F("{\"type\":\"mission-align\",\"sl\":"));
  DEBUG_SERIAL.print(missionAlignSl);
  DEBUG_SERIAL.print(F(",\"fl\":"));
  DEBUG_SERIAL.print(missionAlignFl);
  DEBUG_SERIAL.print(F(",\"fr\":"));
  DEBUG_SERIAL.print(missionAlignFr);
  DEBUG_SERIAL.print(F(",\"tolerance\":"));
  DEBUG_SERIAL.print(missionAlignTolerance);
  DEBUG_SERIAL.println(F(",\"sideRightUsed\":false}"));
}

void printMissionGripAlignStatus() {
  DEBUG_SERIAL.print(F("[mission gripalign] upper FL="));
  DEBUG_SERIAL.print(missionGripAlignFl);
  DEBUG_SERIAL.print(F(", FR="));
  DEBUG_SERIAL.print(missionGripAlignFr);
  DEBUG_SERIAL.print(F(", SL(ref)="));
  DEBUG_SERIAL.print(missionGripAlignSl);
  DEBUG_SERIAL.print(F(", tolerance="));
  DEBUG_SERIAL.print(missionGripAlignTolerance);
  DEBUG_SERIAL.println(F(" (SL/SR ignored for motion)"));
  DEBUG_SERIAL.print(F("                    lower FL="));
  DEBUG_SERIAL.print(missionLowerGripAlignFl);
  DEBUG_SERIAL.print(F(", FR="));
  DEBUG_SERIAL.print(missionLowerGripAlignFr);
  DEBUG_SERIAL.print(F(", SL(ref)="));
  DEBUG_SERIAL.print(missionLowerGripAlignSl);
  DEBUG_SERIAL.print(F(", tolerance="));
  DEBUG_SERIAL.print(missionLowerGripAlignTolerance);
  DEBUG_SERIAL.println(F(" (SL/SR ignored for motion)"));
  DEBUG_SERIAL.print(F("{\"type\":\"mission-gripalign\",\"sl\":"));
  DEBUG_SERIAL.print(missionGripAlignSl);
  DEBUG_SERIAL.print(F(",\"fl\":"));
  DEBUG_SERIAL.print(missionGripAlignFl);
  DEBUG_SERIAL.print(F(",\"fr\":"));
  DEBUG_SERIAL.print(missionGripAlignFr);
  DEBUG_SERIAL.print(F(",\"tolerance\":"));
  DEBUG_SERIAL.print(missionGripAlignTolerance);
  DEBUG_SERIAL.print(F(",\"sideRightUsed\":false,\"lower\":{\"sl\":"));
  DEBUG_SERIAL.print(missionLowerGripAlignSl);
  DEBUG_SERIAL.print(F(",\"fl\":"));
  DEBUG_SERIAL.print(missionLowerGripAlignFl);
  DEBUG_SERIAL.print(F(",\"fr\":"));
  DEBUG_SERIAL.print(missionLowerGripAlignFr);
  DEBUG_SERIAL.print(F(",\"tolerance\":"));
  DEBUG_SERIAL.print(missionLowerGripAlignTolerance);
  DEBUG_SERIAL.println(F(",\"sideLeftUsedForMotion\":false,\"sideRightUsed\":false}}"));
}

bool commandMissionAlignTuning(const String &input) {
  String mode = tokenAt(input, 2);
  mode.toLowerCase();
  if (mode == "current" || mode == "now" || mode == "현재") {
    long tolerance = missionAlignTolerance;
    if (tokenCount(input) >= 4 &&
        !parseLongStrict(tokenAt(input, 3), &tolerance)) {
      DEBUG_SERIAL.println(F("사용법: mission align current [tolerance]"));
      return false;
    }
    if (tolerance < 1 || tolerance > 80) {
      DEBUG_SERIAL.println(F("[제한] tolerance는 1~80 범위입니다."));
      return false;
    }
    PsdSnapshot snapshot;
    readAllPSDSensors(&snapshot);
    missionAlignSl = snapshot.sl;
    missionAlignFl = snapshot.fl;
    missionAlignFr = snapshot.fr;
    missionAlignTolerance = (int16_t)tolerance;
    DEBUG_SERIAL.println(F("[mission align] 현재 PSD 값을 적재함 스캔 정렬 기준으로 적용했습니다. SR은 저장하지 않습니다."));
    printMissionAlignStatus();
    printPsdStatus();
    return true;
  }

  if (tokenCount(input) < 5) {
    printMissionAlignStatus();
    DEBUG_SERIAL.println(F("사용법: mission align <targetSl> <targetFl> <targetFr> [tolerance]"));
    DEBUG_SERIAL.println(F("       mission align current [tolerance]"));
    DEBUG_SERIAL.println(F("예시: mission align 354 266 269 8"));
    DEBUG_SERIAL.println(F("Side-right PSD는 출력만 보고, 적재함 정렬 제어에는 사용하지 않습니다."));
    return true;
  }

  long targetSl = 0;
  long targetFl = 0;
  long targetFr = 0;
  long tolerance = missionAlignTolerance;
  if (!parseLongStrict(tokenAt(input, 2), &targetSl) ||
      !parseLongStrict(tokenAt(input, 3), &targetFl) ||
      !parseLongStrict(tokenAt(input, 4), &targetFr)) {
    DEBUG_SERIAL.println(F("사용법: mission align <targetSl> <targetFl> <targetFr> [tolerance]"));
    return false;
  }
  if (tokenCount(input) >= 6 &&
      !parseLongStrict(tokenAt(input, 5), &tolerance)) {
    DEBUG_SERIAL.println(F("사용법: mission align <targetSl> <targetFl> <targetFr> [tolerance]"));
    return false;
  }
  if (targetSl < 1 || targetSl > 1023 ||
      targetFl < 1 || targetFl > 1023 ||
      targetFr < 1 || targetFr > 1023 ||
      tolerance < 1 || tolerance > 80) {
    DEBUG_SERIAL.println(F("[제한] PSD target은 1~1023, tolerance는 1~80 범위입니다."));
    return false;
  }

  missionAlignSl = (int16_t)targetSl;
  missionAlignFl = (int16_t)targetFl;
  missionAlignFr = (int16_t)targetFr;
  missionAlignTolerance = (int16_t)tolerance;
  printMissionAlignStatus();
  return true;
}

bool parseGripAlignLayer(String text, bool *useLower) {
  text.toLowerCase();
  if (text == "lower" || text == "아래" || text == "하층") {
    *useLower = true;
    return true;
  }
  if (text == "upper" || text == "위" || text == "상층") {
    *useLower = false;
    return true;
  }
  return false;
}

void setMissionGripAlignTarget(bool lower,
                               int16_t targetSl,
                               int16_t targetFl,
                               int16_t targetFr,
                               int16_t tolerance) {
  if (lower) {
    missionLowerGripAlignSl = targetSl;
    missionLowerGripAlignFl = targetFl;
    missionLowerGripAlignFr = targetFr;
    missionLowerGripAlignTolerance = tolerance;
  } else {
    missionGripAlignSl = targetSl;
    missionGripAlignFl = targetFl;
    missionGripAlignFr = targetFr;
    missionGripAlignTolerance = tolerance;
  }
}

void readMissionGripAlignTarget(bool lower,
                                int16_t *targetSl,
                                int16_t *targetFl,
                                int16_t *targetFr,
                                int16_t *tolerance) {
  if (lower) {
    *targetSl = missionLowerGripAlignSl;
    *targetFl = missionLowerGripAlignFl;
    *targetFr = missionLowerGripAlignFr;
    *tolerance = missionLowerGripAlignTolerance;
  } else {
    *targetSl = missionGripAlignSl;
    *targetFl = missionGripAlignFl;
    *targetFr = missionGripAlignFr;
    *tolerance = missionGripAlignTolerance;
  }
}

bool commandMissionGripAlignTuning(const String &input) {
  bool lower = false;
  uint8_t argOffset = 0;
  if (parseGripAlignLayer(tokenAt(input, 2), &lower)) {
    argOffset = 1;
  }

  String mode = tokenAt(input, 2 + argOffset);
  mode.toLowerCase();
  if (mode == "run" || mode == "go" || mode == "move" || mode == "실행" || mode == "이동") {
    return commandMissionStorageGripAlignForLayer(lower);
  }
  if (mode == "current" || mode == "now" || mode == "현재") {
    int16_t currentSl = 0;
    int16_t currentFl = 0;
    int16_t currentFr = 0;
    int16_t currentTolerance = 0;
    readMissionGripAlignTarget(lower, &currentSl, &currentFl, &currentFr, &currentTolerance);
    long tolerance = currentTolerance;
    if (tokenCount(input) >= 4 + argOffset &&
        !parseLongStrict(tokenAt(input, 3 + argOffset), &tolerance)) {
      DEBUG_SERIAL.println(F("사용법: mission gripalign [upper|lower] current [tolerance]"));
      return false;
    }
    if (tolerance < 1 || tolerance > 80) {
      DEBUG_SERIAL.println(F("[제한] tolerance는 1~80 범위입니다."));
      return false;
    }
    PsdSnapshot snapshot;
    readAllPSDSensors(&snapshot);
    setMissionGripAlignTarget(lower, snapshot.sl, snapshot.fl, snapshot.fr, (int16_t)tolerance);
    DEBUG_SERIAL.println(lower
      ? F("[mission gripalign] 현재 FL/FR 값을 하층 집기 깊이 기준으로 적용했습니다. SL은 참고값으로만 저장합니다.")
      : F("[mission gripalign] 현재 FL/FR 값을 상층 집기 깊이 기준으로 적용했습니다. SL은 참고값으로만 저장합니다."));
    printMissionGripAlignStatus();
    printPsdStatus();
    return true;
  }

  uint8_t count = tokenCount(input);
  uint8_t firstValueIndex = 2 + argOffset;
  uint8_t numericCount = count > firstValueIndex ? count - firstValueIndex : 0;

  if (numericCount < 2) {
    printMissionGripAlignStatus();
    DEBUG_SERIAL.println(F("사용법: mission gripalign [upper|lower] <targetFl> <targetFr> [tolerance]"));
    DEBUG_SERIAL.println(F("       legacy: mission gripalign [upper|lower] <targetSl> <targetFl> <targetFr> <tolerance>"));
    DEBUG_SERIAL.println(F("       mission gripalign [upper|lower] current [tolerance]"));
    DEBUG_SERIAL.println(F("       mission gripalign [upper|lower] run"));
    DEBUG_SERIAL.println(F("예시: mission gripalign upper 349 363 8"));
    DEBUG_SERIAL.println(F("예시: mission gripalign lower 325 337 8"));
    DEBUG_SERIAL.println(F("집기 직전 이동은 FL/FR 전후 깊이만 맞추며, SL/SR은 출력 참고값입니다."));
    return true;
  }

  long targetSl = 0;
  long targetFl = 0;
  long targetFr = 0;
  int16_t currentSl = 0;
  int16_t currentFl = 0;
  int16_t currentFr = 0;
  int16_t currentTolerance = 0;
  readMissionGripAlignTarget(lower, &currentSl, &currentFl, &currentFr, &currentTolerance);
  targetSl = currentSl;
  long tolerance = currentTolerance;

  if (numericCount >= 4) {
    if (!parseLongStrict(tokenAt(input, firstValueIndex), &targetSl) ||
        !parseLongStrict(tokenAt(input, firstValueIndex + 1), &targetFl) ||
        !parseLongStrict(tokenAt(input, firstValueIndex + 2), &targetFr) ||
        !parseLongStrict(tokenAt(input, firstValueIndex + 3), &tolerance)) {
      DEBUG_SERIAL.println(F("사용법: mission gripalign [upper|lower] <targetFl> <targetFr> [tolerance]"));
      return false;
    }
  } else {
    if (!parseLongStrict(tokenAt(input, firstValueIndex), &targetFl) ||
        !parseLongStrict(tokenAt(input, firstValueIndex + 1), &targetFr)) {
      DEBUG_SERIAL.println(F("사용법: mission gripalign [upper|lower] <targetFl> <targetFr> [tolerance]"));
      return false;
    }
    if (numericCount >= 3 &&
        !parseLongStrict(tokenAt(input, firstValueIndex + 2), &tolerance)) {
      DEBUG_SERIAL.println(F("사용법: mission gripalign [upper|lower] <targetFl> <targetFr> [tolerance]"));
      return false;
    }
  }

  if (targetSl < 1 || targetSl > 1023 ||
      targetFl < 1 || targetFl > 1023 ||
      targetFr < 1 || targetFr > 1023 ||
      tolerance < 1 || tolerance > 80) {
    DEBUG_SERIAL.println(F("[제한] PSD target은 1~1023, tolerance는 1~80 범위입니다."));
    return false;
  }

  setMissionGripAlignTarget(lower, (int16_t)targetSl, (int16_t)targetFl,
                            (int16_t)targetFr, (int16_t)tolerance);
  printMissionGripAlignStatus();
  return true;
}

void printMissionPlaceAlignStatus() {
  DEBUG_SERIAL.print(F("[mission placealign] place SL="));
  DEBUG_SERIAL.print(missionPlaceSl);
  DEBUG_SERIAL.print(F(", FR="));
  DEBUG_SERIAL.print(missionPlaceFr);
  DEBUG_SERIAL.print(F(", tolerance="));
  DEBUG_SERIAL.println(missionPlaceTolerance);
  DEBUG_SERIAL.print(F("{\"type\":\"mission-placealign\",\"sl\":"));
  DEBUG_SERIAL.print(missionPlaceSl);
  DEBUG_SERIAL.print(F(",\"fr\":"));
  DEBUG_SERIAL.print(missionPlaceFr);
  DEBUG_SERIAL.print(F(",\"tolerance\":"));
  DEBUG_SERIAL.print(missionPlaceTolerance);
  DEBUG_SERIAL.println(F(",\"sideRightUsed\":false,\"frontLeftUsed\":false}"));
}

bool commandMissionPlaceAlignTuning(const String &input) {
  String mode = tokenAt(input, 2);
  mode.toLowerCase();
  if (mode == "run" || mode == "go" || mode == "move" || mode == "실행" || mode == "이동") {
    return commandMissionMoveToPlaceAlign();
  }
  if (mode == "current" || mode == "now" || mode == "현재") {
    long tolerance = missionPlaceTolerance;
    if (tokenCount(input) >= 4 &&
        !parseLongStrict(tokenAt(input, 3), &tolerance)) {
      DEBUG_SERIAL.println(F("사용법: mission placealign current [tolerance]"));
      return false;
    }
    if (tolerance < 1 || tolerance > 80) {
      DEBUG_SERIAL.println(F("[제한] tolerance는 1~80 범위입니다."));
      return false;
    }
    PsdSnapshot snapshot;
    readAllPSDSensors(&snapshot);
    missionPlaceSl = snapshot.sl;
    missionPlaceFr = snapshot.fr;
    missionPlaceTolerance = (int16_t)tolerance;
    DEBUG_SERIAL.println(F("[mission placealign] 현재 SL+FR 값을 미션수행존 배치 기준으로 적용했습니다. FL/SR은 저장하지 않습니다."));
    printMissionPlaceAlignStatus();
    printPsdStatus();
    return true;
  }

  if (tokenCount(input) < 3) {
    printMissionPlaceAlignStatus();
    DEBUG_SERIAL.println(F("사용법: mission placealign <targetSl> [targetFr] [tolerance]"));
    DEBUG_SERIAL.println(F("       mission placealign current [tolerance]"));
    DEBUG_SERIAL.println(F("       mission placealign run"));
    DEBUG_SERIAL.println(F("예시: mission placealign 635 220 8"));
    DEBUG_SERIAL.println(F("배치 이동은 SL+FR 기준입니다. FL/SR은 이 위치에서 의미가 약해서 쓰지 않습니다."));
    return true;
  }

  long targetSl = 0;
  long targetFr = missionPlaceFr;
  long tolerance = missionPlaceTolerance;
  if (!parseLongStrict(tokenAt(input, 2), &targetSl)) {
    DEBUG_SERIAL.println(F("사용법: mission placealign <targetSl> [targetFr] [tolerance]"));
    return false;
  }
  if (tokenCount(input) >= 5) {
    if (!parseLongStrict(tokenAt(input, 3), &targetFr) ||
        !parseLongStrict(tokenAt(input, 4), &tolerance)) {
      DEBUG_SERIAL.println(F("사용법: mission placealign <targetSl> [targetFr] [tolerance]"));
      return false;
    }
  } else if (tokenCount(input) >= 4) {
    if (!parseLongStrict(tokenAt(input, 3), &tolerance)) {
      DEBUG_SERIAL.println(F("사용법: mission placealign <targetSl> [targetFr] [tolerance]"));
      return false;
    }
  }
  if (targetSl < 1 || targetSl > 1023 ||
      targetFr < 1 || targetFr > 1023 ||
      tolerance < 1 || tolerance > 80) {
    DEBUG_SERIAL.println(F("[제한] targetSl/targetFr은 1~1023, tolerance는 1~80 범위입니다."));
    return false;
  }

  missionPlaceSl = (int16_t)targetSl;
  missionPlaceFr = (int16_t)targetFr;
  missionPlaceTolerance = (int16_t)tolerance;
  printMissionPlaceAlignStatus();
  return true;
}

void printMissionStoragePathStatus() {
  DEBUG_SERIAL.print(F("[mission storagepath] first="));
  DEBUG_SERIAL.print(missionStorageFirstForwardMm, 2);
  DEBUG_SERIAL.print(F("mm, extra="));
  DEBUG_SERIAL.print(missionStorageExtraForwardMm, 2);
  DEBUG_SERIAL.print(F("mm, right="));
  DEBUG_SERIAL.print(missionStorageRightMm, 2);
  DEBUG_SERIAL.print(F("mm, speed="));
  DEBUG_SERIAL.print(missionMotion.positionMoveMmPerSec);
  DEBUG_SERIAL.println(F("mm/s"));
  DEBUG_SERIAL.print(F("{\"type\":\"mission-storagepath\",\"firstForwardMm\":"));
  DEBUG_SERIAL.print(missionStorageFirstForwardMm, 2);
  DEBUG_SERIAL.print(F(",\"extraForwardMm\":"));
  DEBUG_SERIAL.print(missionStorageExtraForwardMm, 2);
  DEBUG_SERIAL.print(F(",\"rightMm\":"));
  DEBUG_SERIAL.print(missionStorageRightMm, 2);
  DEBUG_SERIAL.print(F(",\"speedMmPerSec\":"));
  DEBUG_SERIAL.print(missionMotion.positionMoveMmPerSec);
  DEBUG_SERIAL.println(F("}"));
}

bool commandMissionStoragePath(const String &input) {
  if (tokenCount(input) < 5) {
    printMissionStoragePathStatus();
    DEBUG_SERIAL.println(F("사용법: mission storagepath <firstForwardMm> <extraForwardMm> <rightMm> [speedMm/s]"));
    DEBUG_SERIAL.println(F("예시: mission storagepath 450 390 60 150"));
    return true;
  }

  float firstForwardMm = 0.0;
  float extraForwardMm = 0.0;
  float rightMm = 0.0;
  long speedMmPerSec = missionMotion.positionMoveMmPerSec;
  if (!parseFloatStrict(tokenAt(input, 2), &firstForwardMm) ||
      !parseFloatStrict(tokenAt(input, 3), &extraForwardMm) ||
      !parseFloatStrict(tokenAt(input, 4), &rightMm)) {
    DEBUG_SERIAL.println(F("사용법: mission storagepath <firstForwardMm> <extraForwardMm> <rightMm> [speedMm/s]"));
    return false;
  }
  if (tokenCount(input) >= 6 &&
      !parseLongStrict(tokenAt(input, 5), &speedMmPerSec)) {
    DEBUG_SERIAL.println(F("사용법: mission storagepath <firstForwardMm> <extraForwardMm> <rightMm> [speedMm/s]"));
    return false;
  }

  if (!validateMissionStoragePathValue(firstForwardMm, F("firstForwardMm")) ||
      !validateMissionStoragePathValue(extraForwardMm, F("extraForwardMm")) ||
      !validateMissionStoragePathValue(rightMm, F("rightMm")) ||
      !validateMissionPositionSpeed(speedMmPerSec)) {
    return false;
  }

  missionStorageFirstForwardMm = firstForwardMm;
  missionStorageExtraForwardMm = extraForwardMm;
  missionStorageRightMm = rightMm;
  missionMotion.positionMoveMmPerSec = speedMmPerSec;
  printMissionStoragePathStatus();
  return true;
}

void printMissionStorageApproachGateStatus() {
  DEBUG_SERIAL.print(F("[mission storagegate] slLeave="));
  DEBUG_SERIAL.print(missionStorageApproachSlLeaveAdc);
  DEBUG_SERIAL.print(F(", slReenter="));
  DEBUG_SERIAL.print(missionStorageApproachSlReenterAdc);
  DEBUG_SERIAL.print(F(", confirmSamples="));
  DEBUG_SERIAL.print(missionStorageApproachSlReenterConfirmSamples);
  DEBUG_SERIAL.print(F(", ignoreMs="));
  DEBUG_SERIAL.print(missionStorageApproachIgnoreReentryMs);
  DEBUG_SERIAL.print(F(", forwardRaw="));
  DEBUG_SERIAL.println(missionStorageApproachForwardSpeed);
  DEBUG_SERIAL.print(F("{\"type\":\"mission-storagegate\",\"slLeave\":"));
  DEBUG_SERIAL.print(missionStorageApproachSlLeaveAdc);
  DEBUG_SERIAL.print(F(",\"slReenter\":"));
  DEBUG_SERIAL.print(missionStorageApproachSlReenterAdc);
  DEBUG_SERIAL.print(F(",\"confirmSamples\":"));
  DEBUG_SERIAL.print(missionStorageApproachSlReenterConfirmSamples);
  DEBUG_SERIAL.print(F(",\"ignoreMs\":"));
  DEBUG_SERIAL.print(missionStorageApproachIgnoreReentryMs);
  DEBUG_SERIAL.print(F(",\"forwardRaw\":"));
  DEBUG_SERIAL.print(missionStorageApproachForwardSpeed);
  DEBUG_SERIAL.println(F("}"));
}

bool commandMissionStorageGateTuning(const String &input) {
  if (tokenCount(input) < 4) {
    printMissionStorageApproachGateStatus();
    DEBUG_SERIAL.println(F("사용법: mission storagegate <slLeave> <slReenter> [forwardRaw] [confirmSamples] [ignoreMs]"));
    DEBUG_SERIAL.println(F("예시: mission storagegate 500 550 150 2 2000"));
    return true;
  }

  long slLeave = 0;
  long slReenter = 0;
  long forwardRaw = missionStorageApproachForwardSpeed;
  long confirmSamples = missionStorageApproachSlReenterConfirmSamples;
  long ignoreMs = missionStorageApproachIgnoreReentryMs;
  if (!parseLongStrict(tokenAt(input, 2), &slLeave) ||
      !parseLongStrict(tokenAt(input, 3), &slReenter)) {
    DEBUG_SERIAL.println(F("사용법: mission storagegate <slLeave> <slReenter> [forwardRaw] [confirmSamples] [ignoreMs]"));
    return false;
  }
  if (tokenCount(input) >= 5 &&
      !parseLongStrict(tokenAt(input, 4), &forwardRaw)) {
    DEBUG_SERIAL.println(F("사용법: mission storagegate <slLeave> <slReenter> [forwardRaw] [confirmSamples] [ignoreMs]"));
    return false;
  }
  if (tokenCount(input) >= 6 &&
      !parseLongStrict(tokenAt(input, 5), &confirmSamples)) {
    DEBUG_SERIAL.println(F("사용법: mission storagegate <slLeave> <slReenter> [forwardRaw] [confirmSamples] [ignoreMs]"));
    return false;
  }
  if (tokenCount(input) >= 7 &&
      !parseLongStrict(tokenAt(input, 6), &ignoreMs)) {
    DEBUG_SERIAL.println(F("사용법: mission storagegate <slLeave> <slReenter> [forwardRaw] [confirmSamples] [ignoreMs]"));
    return false;
  }

  if (slLeave < 1 || slLeave > 1023 ||
      slReenter < 1 || slReenter > 1023 ||
      slLeave >= slReenter ||
      confirmSamples < 1 || confirmSamples > 20 ||
      ignoreMs < 1 || ignoreMs > 10000 ||
      !validateMissionVelocitySpeed(forwardRaw)) {
    DEBUG_SERIAL.println(F("[제한] slLeave/slReenter는 1~1023, slLeave < slReenter, samples는 1~20, ignoreMs는 1~10000 범위입니다."));
    return false;
  }

  missionStorageApproachSlLeaveAdc = (int16_t)slLeave;
  missionStorageApproachSlReenterAdc = (int16_t)slReenter;
  missionStorageApproachSlReenterConfirmSamples = (uint8_t)confirmSamples;
  missionStorageApproachIgnoreReentryMs = (uint16_t)ignoreMs;
  missionStorageApproachForwardSpeed = (int32_t)forwardRaw;
  printMissionStorageApproachGateStatus();
  return true;
}

bool commandMissionUndo() {
  if (!missionUndoAvailable) {
    DEBUG_SERIAL.println(F("[mission undo] 되돌릴 수 있는 마지막 고정 거리 이동이 없습니다."));
    DEBUG_SERIAL.println(F("  PSD 정렬, 장애물 접근, 집기, 배치는 자동 원복하지 않습니다."));
    return false;
  }

  DEBUG_SERIAL.println(F("[mission undo] 마지막 고정 거리 이동을 반대 방향으로 1회 실행합니다."));
  bool ok = commandMissionDriveDistanceStep(F("  undo 이동"),
                                            missionUndoDistanceMm,
                                            missionUndoDirection,
                                            missionUndoSpeedMmPerSec,
                                            false);
  if (ok) clearMissionUndoCandidate();
  return ok;
}

bool commandMissionStorageForward1() {
  DEBUG_SERIAL.println(F("[mission move] 적재함 접근 1/4: 첫 전진"));
  DEBUG_SERIAL.print(F("  position speed="));
  DEBUG_SERIAL.println(missionMotion.positionMoveMmPerSec);
  return commandMissionDriveDistanceStep(F("  전진"), missionStorageFirstForwardMm,
                                         DRIVE_DIRECTION_FORWARD,
                                         missionMotion.positionMoveMmPerSec);
}

bool commandMissionStorageForward2() {
  DEBUG_SERIAL.println(F("[mission move] 적재함 접근 2/4: 추가 전진"));
  return commandMissionDriveDistanceStep(F("  추가 전진"), missionStorageExtraForwardMm,
                                         DRIVE_DIRECTION_FORWARD,
                                         missionMotion.positionMoveMmPerSec);
}

bool commandMissionStorageRight() {
  DEBUG_SERIAL.println(F("[mission move] 적재함 접근 3/4: 우측 이동"));
  return commandMissionDriveDistanceStep(F("  우측 이동"), missionStorageRightMm,
                                         DRIVE_DIRECTION_RIGHT,
                                         missionMotion.positionMoveMmPerSec);
}

int16_t storageAlignFrontErrorFor(int16_t flVal, int16_t frVal,
                                  int16_t targetFl, int16_t targetFr,
                                  int16_t tolerance) {
  int16_t flError = flVal - targetFl;
  int16_t frError = frVal - targetFr;
  bool flOk = abs(flError) <= tolerance;
  bool frOk = abs(frError) <= tolerance;
  if (flOk && frOk) return 0;
  if (flOk) return frError;
  if (frOk) return flError;
  if ((flError < 0 && frError > 0) || (flError > 0 && frError < 0)) {
    return abs(flError) >= abs(frError) ? flError : frError;
  }
  return (flError + frError) / 2;
}

int16_t storageAlignFrontError(int16_t flVal, int16_t frVal) {
  return storageAlignFrontErrorFor(flVal, frVal,
                                   missionAlignFl, missionAlignFr,
                                   missionAlignTolerance);
}

bool commandMissionStorageForwardUntilSlReentry() {
  if (!ensureMobileReady()) return false;

  DEBUG_SERIAL.println(F("[mission move] 적재함 접근 1/3: 지시존 이탈 후 다음 SL 박스 감지까지 전진"));
  DEBUG_SERIAL.print(F("  SL leave/reenter="));
  DEBUG_SERIAL.print(missionStorageApproachSlLeaveAdc);
  DEBUG_SERIAL.print(F("/"));
  DEBUG_SERIAL.print(missionStorageApproachSlReenterAdc);
  DEBUG_SERIAL.print(F(" samples="));
  DEBUG_SERIAL.print(missionStorageApproachSlReenterConfirmSamples);
  DEBUG_SERIAL.print(F(", ignoreMs="));
  DEBUG_SERIAL.print(missionStorageApproachIgnoreReentryMs);
  DEBUG_SERIAL.print(F(", forward raw="));
  DEBUG_SERIAL.println(missionStorageApproachForwardSpeed);

  ChangeMobilebaseMode2VelocityControlMode(dxl);
  int16_t slVal = 0;
  bool sawLeave = false;
  uint8_t reenterSamples = 0;
  unsigned long ignoreReentryUntil = 0;
  unsigned long lastProgressLogMs = millis();
  while (true) {
    if (checkEmergencyStopInput()) {
      stopAll(F("[긴급정지] ! 입력"));
      return false;
    }
    readSideLeftPSDSensor(&slVal);
    if (!sawLeave) {
      if (slVal <= missionStorageApproachSlLeaveAdc) {
        sawLeave = true;
        ignoreReentryUntil = millis() + missionStorageApproachIgnoreReentryMs;
        DEBUG_SERIAL.print(F("  SL 이탈 감지 SL="));
        DEBUG_SERIAL.println(slVal);
        DEBUG_SERIAL.print(F("  이후 "));
        DEBUG_SERIAL.print(missionStorageApproachIgnoreReentryMs);
        DEBUG_SERIAL.println(F("ms 동안 재감지 무시하고 전진"));
      }
    } else {
      if ((long)(millis() - ignoreReentryUntil) < 0) {
        reenterSamples = 0;
      } else if (slVal >= missionStorageApproachSlReenterAdc) {
        if (reenterSamples < missionStorageApproachSlReenterConfirmSamples) {
          reenterSamples++;
        }
      } else {
        reenterSamples = 0;
      }
      if (reenterSamples >= missionStorageApproachSlReenterConfirmSamples) {
        break;
      }
    }

    SetMobileGoalVelocityForSyncWrite(dxl,
                                      missionStorageApproachForwardSpeed,
                                      missionStorageApproachForwardSpeed,
                                      missionStorageApproachForwardSpeed,
                                      missionStorageApproachForwardSpeed);
    if (millis() - lastProgressLogMs >= 1000) {
      DEBUG_SERIAL.print(F("  SL 재감지 대기 중 SL="));
      DEBUG_SERIAL.print(slVal);
      DEBUG_SERIAL.print(F(" sawLeave="));
      DEBUG_SERIAL.print(sawLeave ? F("yes") : F("no"));
      DEBUG_SERIAL.print(F(" samples="));
      DEBUG_SERIAL.println(reenterSamples);
      lastProgressLogMs = millis();
    }
    delay(10);
  }

  stopMobilebase();
  DEBUG_SERIAL.print(F("  SL 재감지 전진 종료 SL="));
  DEBUG_SERIAL.print(slVal);
  DEBUG_SERIAL.print(F(" sawLeave="));
  DEBUG_SERIAL.print(sawLeave ? F("yes") : F("no"));
  DEBUG_SERIAL.print(F(" samples="));
  DEBUG_SERIAL.println(reenterSamples);
  interruptibleDelay(CFG.wait.driveSettleMs);
  return sawLeave &&
         reenterSamples >= missionStorageApproachSlReenterConfirmSamples;
}

bool missionStorageApproachFrLeadDetected(int16_t flVal, int16_t frVal) {
  return frVal >= missionStorageApproachFrDetectAdc &&
         (frVal - flVal) >= missionStorageApproachFrLeadDeltaAdc;
}

bool missionStorageApproachSlGateReady(int16_t slVal) {
  return slVal <= missionAlignSl + missionStorageApproachSlGateTolerance;
}

bool missionStorageApproachFrontNearScanDepth(int16_t flVal, int16_t frVal) {
  int16_t frontError = storageAlignFrontErrorFor(flVal, frVal,
                                                 missionAlignFl, missionAlignFr,
                                                 missionAlignTolerance);
  return frontError >= -(missionAlignTolerance * 3);
}

void setMissionStorageApproachVelocity(int32_t rightSpeed, int32_t forwardSpeed) {
  SetMobileGoalVelocityForSyncWrite(dxl,
                                    forwardSpeed + rightSpeed,
                                    forwardSpeed - rightSpeed,
                                    forwardSpeed - rightSpeed,
                                    forwardSpeed + rightSpeed);
}

bool commandMissionStorageRightUntilFrLeadDetected() {
  if (!ensureMobileReady()) return false;

  DEBUG_SERIAL.println(F("[mission move] 적재함 접근 보조: FR>FL 감지까지 우측 이동"));
  DEBUG_SERIAL.print(F("  FR min="));
  DEBUG_SERIAL.print(missionStorageApproachFrDetectAdc);
  DEBUG_SERIAL.print(F(", FR-FL delta="));
  DEBUG_SERIAL.print(missionStorageApproachFrLeadDeltaAdc);
  DEBUG_SERIAL.print(F(", confirm samples="));
  DEBUG_SERIAL.print(missionStorageApproachFrLeadConfirmSamples);
  DEBUG_SERIAL.print(F(", right raw="));
  DEBUG_SERIAL.println(missionStorageApproachRightSpeed);

  ChangeMobilebaseMode2VelocityControlMode(dxl);
  int16_t flVal = 0;
  int16_t frVal = 0;
  uint8_t frLeadSamples = 0;
  unsigned long startedAt = millis();
  while (true) {
    if (checkEmergencyStopInput()) {
      stopAll(F("[긴급정지] ! 입력"));
      return false;
    }
    readFrontPSDSensors(&flVal, &frVal);
    if (missionStorageApproachFrLeadDetected(flVal, frVal)) {
      if (frLeadSamples < missionStorageApproachFrLeadConfirmSamples) {
        frLeadSamples++;
      }
    } else {
      frLeadSamples = 0;
    }
    if (frLeadSamples >= missionStorageApproachFrLeadConfirmSamples) {
      break;
    }
    setMissionStorageApproachVelocity(missionStorageApproachRightSpeed, 0);
    if (millis() - startedAt > CFG.timeout.psdLoopMs) {
      DEBUG_SERIAL.println(F("  FR>FL 감지 우측 이동 타임아웃"));
      break;
    }
    delay(10);
  }

  stopMobilebase();
  DEBUG_SERIAL.print(F("  FR>FL 감지 우측 이동 종료 FL="));
  DEBUG_SERIAL.print(flVal);
  DEBUG_SERIAL.print(F(" FR="));
  DEBUG_SERIAL.print(frVal);
  DEBUG_SERIAL.print(F(" delta="));
  DEBUG_SERIAL.print(frVal - flVal);
  DEBUG_SERIAL.print(F(" samples="));
  DEBUG_SERIAL.println(frLeadSamples);
  if (frLeadSamples < missionStorageApproachFrLeadConfirmSamples) {
    DEBUG_SERIAL.println(F("  [주의] FR>FL 감지 실패. 현재 위치에서 다음 정렬을 계속 시도합니다."));
  }
  return interruptibleDelay(CFG.wait.driveSettleMs);
}

bool commandMissionStorageOpenSideGate() {
  if (!ensureMobileReady()) return false;

  DEBUG_SERIAL.println(F("[mission move] 적재함 접근 2/4: SL 먼저 벌려 대각선 시작점 확보"));
  DEBUG_SERIAL.print(F("  target SL="));
  DEBUG_SERIAL.print(missionAlignSl);
  DEBUG_SERIAL.print(F(", gate tolerance="));
  DEBUG_SERIAL.print(missionStorageApproachSlGateTolerance);
  DEBUG_SERIAL.print(F(", speed="));
  DEBUG_SERIAL.println(missionStorageApproachRightSpeed);

  ChangeMobilebaseMode2VelocityControlMode(dxl);
  int16_t slVal = 0;
  unsigned long startedAt = millis();
  while (true) {
    if (checkEmergencyStopInput()) {
      stopAll(F("[긴급정지] ! 입력"));
      return false;
    }
    readSideLeftPSDSensor(&slVal);
    if (missionStorageApproachSlGateReady(slVal)) {
      break;
    }
    DriveWithOneSensor(dxl, slVal - missionAlignSl,
                       missionStorageApproachSlGateTolerance,
                       DRIVE_DIRECTION_LEFT,
                       missionStorageApproachRightSpeed);
    if (millis() - startedAt > CFG.timeout.psdLoopMs) {
      DEBUG_SERIAL.println(F("  SL 게이트 확보 타임아웃"));
      break;
    }
    delay(10);
  }

  stopMobilebase();
  DEBUG_SERIAL.print(F("  SL 게이트 종료 SL="));
  DEBUG_SERIAL.print(slVal);
  DEBUG_SERIAL.print(F(" err="));
  DEBUG_SERIAL.println(slVal - missionAlignSl);
  interruptibleDelay(CFG.wait.driveSettleMs);
  return missionStorageApproachSlGateReady(slVal);
}

bool commandMissionStorageDiagonalApproach() {
  if (!ensureMobileReady()) return false;

  DEBUG_SERIAL.println(F("[mission move] 적재함 접근 2/4: FR>FL 감지 후 우측+전진 대각선 접근"));
  DEBUG_SERIAL.print(F("  target SL/FL/FR/tol="));
  DEBUG_SERIAL.print(missionAlignSl);
  DEBUG_SERIAL.print(F("/"));
  DEBUG_SERIAL.print(missionAlignFl);
  DEBUG_SERIAL.print(F("/"));
  DEBUG_SERIAL.print(missionAlignFr);
  DEBUG_SERIAL.print(F("/"));
  DEBUG_SERIAL.println(missionAlignTolerance);
  DEBUG_SERIAL.print(F("  FR min/delta="));
  DEBUG_SERIAL.print(missionStorageApproachFrDetectAdc);
  DEBUG_SERIAL.print(F("/"));
  DEBUG_SERIAL.print(missionStorageApproachFrLeadDeltaAdc);
  DEBUG_SERIAL.print(F(" samples="));
  DEBUG_SERIAL.print(missionStorageApproachFrLeadConfirmSamples);
  DEBUG_SERIAL.print(F(", right/forward raw="));
  DEBUG_SERIAL.print(missionStorageApproachRightSpeed);
  DEBUG_SERIAL.print(F("/"));
  DEBUG_SERIAL.println(missionStorageApproachForwardSpeed);

  ChangeMobilebaseMode2VelocityControlMode(dxl);
  int16_t slVal = 0;
  int16_t flVal = 0;
  int16_t frVal = 0;
  bool frLeadSeen = false;
  uint8_t frLeadSamples = 0;
  unsigned long startedAt = millis();
  while (true) {
    if (checkEmergencyStopInput()) {
      stopAll(F("[긴급정지] ! 입력"));
      return false;
    }
    readSideLeftPSDSensor(&slVal);
    readFrontPSDSensors(&flVal, &frVal);
    if (missionStorageApproachFrLeadDetected(flVal, frVal)) {
      if (frLeadSamples < missionStorageApproachFrLeadConfirmSamples) {
        frLeadSamples++;
      }
    } else {
      frLeadSamples = 0;
    }
    frLeadSeen = frLeadSeen ||
                 frLeadSamples >= missionStorageApproachFrLeadConfirmSamples;
    bool slGateReady = missionStorageApproachSlGateReady(slVal);
    bool forwardAllowed = frLeadSeen && slGateReady;
    bool frontNear = missionStorageApproachFrontNearScanDepth(flVal, frVal);
    if (forwardAllowed && frontNear) {
      break;
    }

    int32_t rightSpeed = 0;
    if (!frLeadSeen || !slGateReady ||
        slVal > missionAlignSl + missionAlignTolerance) {
      rightSpeed = missionStorageApproachRightSpeed;
    }
    int32_t forwardSpeed = forwardAllowed ? missionStorageApproachForwardSpeed : 0;
    setMissionStorageApproachVelocity(rightSpeed, forwardSpeed);

    if (millis() - startedAt > CFG.timeout.psdLoopMs) {
      DEBUG_SERIAL.println(F("  대각선 접근 타임아웃"));
      break;
    }
    delay(10);
  }

  stopMobilebase();
  DEBUG_SERIAL.print(F("  대각선 접근 종료 SL="));
  DEBUG_SERIAL.print(slVal);
  DEBUG_SERIAL.print(F(" FL="));
  DEBUG_SERIAL.print(flVal);
  DEBUG_SERIAL.print(F(" FR="));
  DEBUG_SERIAL.print(frVal);
  DEBUG_SERIAL.print(F(" FRlead="));
  DEBUG_SERIAL.print(frLeadSeen ? F("yes") : F("no"));
  DEBUG_SERIAL.print(F(" samples="));
  DEBUG_SERIAL.println(frLeadSamples);
  interruptibleDelay(CFG.wait.driveSettleMs);
  return frLeadSeen;
}

bool commandMissionStorageSideAlignToTarget(const __FlashStringHelper *title,
                                            int16_t targetSl,
                                            int16_t tolerance,
                                            uint16_t settleMs) {
  if (!ensureMobileReady()) return false;

  DEBUG_SERIAL.println(title);
  DEBUG_SERIAL.print(F("  target SL/tol="));
  DEBUG_SERIAL.print(targetSl);
  DEBUG_SERIAL.print(F("/"));
  DEBUG_SERIAL.println(tolerance);
  DEBUG_SERIAL.print(F("  psd speed="));
  DEBUG_SERIAL.println(missionMotion.psdCorrectionSpeed);
  ChangeMobilebaseMode2VelocityControlMode(dxl);

  int16_t slVal = 0;
  unsigned long startedAt = millis();
  while (true) {
    if (checkEmergencyStopInput()) {
      stopAll(F("[긴급정지] ! 입력"));
      return false;
    }
    readSideLeftPSDSensor(&slVal);
    if (!DriveWithOneSensor(dxl, slVal - targetSl, tolerance,
                            DRIVE_DIRECTION_LEFT,
                            missionMotion.psdCorrectionSpeed)) {
      break;
    }
    if (millis() - startedAt > CFG.timeout.psdLoopMs) {
      DEBUG_SERIAL.println(F("  SL 정렬 타임아웃"));
      break;
    }
    delay(10);
  }

  stopMobilebase();
  DEBUG_SERIAL.print(F("  SL 정렬 종료 SL="));
  DEBUG_SERIAL.print(slVal);
  DEBUG_SERIAL.print(F(" err="));
  DEBUG_SERIAL.println(slVal - targetSl);
  clearMissionUndoCandidate();
  return interruptibleDelay(settleMs);
}

bool commandMissionStorageAlignToTarget(const __FlashStringHelper *title,
                                        int16_t targetSl, int16_t targetFl,
                                        int16_t targetFr, int16_t tolerance,
                                        bool resetStorageColumn,
                                        bool enablePixyLamp,
                                        uint16_t settleMs) {
  if (!ensureMobileReady()) return false;

  DEBUG_SERIAL.println(title);
  DEBUG_SERIAL.print(F("  target SL/FL/FR/tol="));
  DEBUG_SERIAL.print(targetSl);
  DEBUG_SERIAL.print(F("/"));
  DEBUG_SERIAL.print(targetFl);
  DEBUG_SERIAL.print(F("/"));
  DEBUG_SERIAL.print(targetFr);
  DEBUG_SERIAL.print(F("/"));
  DEBUG_SERIAL.print(tolerance);
  DEBUG_SERIAL.println(F(" (SR ignored)"));
  DEBUG_SERIAL.print(F("  psd speed="));
  DEBUG_SERIAL.println(missionMotion.psdCorrectionSpeed);
  ChangeMobilebaseMode2VelocityControlMode(dxl);
  int16_t slVal = 0;
  int16_t flVal = 0;
  int16_t frVal = 0;
  int16_t frontError = 0;
  unsigned long startedAt = millis();
  while (true) {
    if (checkEmergencyStopInput()) {
      stopAll(F("[긴급정지] ! 입력"));
      return false;
    }
    readSideLeftPSDSensor(&slVal);
    readFrontPSDSensors(&flVal, &frVal);
    frontError = storageAlignFrontErrorFor(flVal, frVal, targetFl, targetFr, tolerance);
    if (!LocatingWithTwoSensors(dxl, slVal - targetSl,
                                frontError, tolerance,
                                DRIVE_DIRECTION_LEFT,
                                missionMotion.psdCorrectionSpeed)) {
      break;
    }
    if (millis() - startedAt > CFG.timeout.psdLoopMs) {
      DEBUG_SERIAL.println(F("  PSD 정렬 타임아웃"));
      break;
    }
    delay(10);
  }
  stopMobilebase();
  DEBUG_SERIAL.print(F("  PSD 정렬 종료 SL="));
  DEBUG_SERIAL.print(slVal);
  DEBUG_SERIAL.print(F(" FL="));
  DEBUG_SERIAL.print(flVal);
  DEBUG_SERIAL.print(F(" FR="));
  DEBUG_SERIAL.print(frVal);
  DEBUG_SERIAL.print(F(" frontError="));
  DEBUG_SERIAL.print(frontError);
  DEBUG_SERIAL.print(F(" FLerr="));
  DEBUG_SERIAL.print(flVal - targetFl);
  DEBUG_SERIAL.print(F(" FRerr="));
  DEBUG_SERIAL.println(frVal - targetFr);
  if (resetStorageColumn) {
    missionStorageColumn = 1;
  }
  if (enablePixyLamp) {
    pixy.setLamp(1, 1);
  }
  clearMissionUndoCandidate();
  return interruptibleDelay(settleMs);
}

bool commandMissionStorageFrontDepthAlignToTarget(const __FlashStringHelper *title,
                                                  int16_t targetFl,
                                                  int16_t targetFr,
                                                  int16_t tolerance,
                                                  uint16_t settleMs) {
  if (!ensureMobileReady()) return false;

  DEBUG_SERIAL.println(title);
  DEBUG_SERIAL.print(F("  target FL/FR/tol="));
  DEBUG_SERIAL.print(targetFl);
  DEBUG_SERIAL.print(F("/"));
  DEBUG_SERIAL.print(targetFr);
  DEBUG_SERIAL.print(F("/"));
  DEBUG_SERIAL.print(tolerance);
  DEBUG_SERIAL.println(F(" (SL/SR ignored for motion)"));
  DEBUG_SERIAL.print(F("  front depth speed="));
  DEBUG_SERIAL.println(missionMotion.frontDepthCorrectionSpeed);
  ChangeMobilebaseMode2VelocityControlMode(dxl);

  int16_t slVal = 0;
  int16_t flVal = 0;
  int16_t frVal = 0;
  int16_t flError = 0;
  int16_t frError = 0;
  unsigned long startedAt = millis();
  while (true) {
    if (checkEmergencyStopInput()) {
      stopAll(F("[긴급정지] ! 입력"));
      return false;
    }
    readSideLeftPSDSensor(&slVal);
    readFrontPSDSensors(&flVal, &frVal);
    flError = flVal - targetFl;
    frError = frVal - targetFr;
    if (!GoForwardWithTwoSensors(dxl, flError, frError, tolerance,
                                 missionMotion.frontDepthCorrectionSpeed)) {
      break;
    }
    if (millis() - startedAt > CFG.timeout.psdLoopMs) {
      DEBUG_SERIAL.println(F("  PSD 전후 깊이 정렬 타임아웃"));
      break;
    }
    delay(10);
  }

  stopMobilebase();
  DEBUG_SERIAL.print(F("  PSD 깊이 정렬 종료 SL(ref)="));
  DEBUG_SERIAL.print(slVal);
  DEBUG_SERIAL.print(F(" FL="));
  DEBUG_SERIAL.print(flVal);
  DEBUG_SERIAL.print(F(" FR="));
  DEBUG_SERIAL.print(frVal);
  DEBUG_SERIAL.print(F(" FLerr="));
  DEBUG_SERIAL.print(flError);
  DEBUG_SERIAL.print(F(" FRerr="));
  DEBUG_SERIAL.println(frError);
  clearMissionUndoCandidate();
  return interruptibleDelay(settleMs);
}

bool commandMissionStorageAlign() {
  DEBUG_SERIAL.println(F("[mission move] 적재함 접근 4/4: 최종 스캔 기준 정렬"));
  bool ok = commandMissionStorageSideAlignToTarget(
    F("  [scan align A] SL 단독 정렬"),
    missionAlignSl, missionAlignTolerance, CFG.wait.driveSettleMs);
  if (ok) {
    ok = commandMissionStorageFrontDepthAlignToTarget(
      F("  [scan align B] FL/FR 전방 깊이 정렬"),
      missionAlignFl, missionAlignFr, missionAlignTolerance,
      CFG.wait.cameraLampMs);
  }
  if (ok) {
    missionStorageColumn = 1;
    pixy.setLamp(1, 1);
    clearMissionUndoCandidate();
  }
  return ok;
}

bool commandMissionStorageDynamicApproach() {
  bool ok = commandMissionStorageForwardUntilSlReentry();
  if (!ok) {
    DEBUG_SERIAL.println(F("[mission move] SL 이탈/재감지가 완전하지 않습니다. 현재 위치에서 최종 정렬을 시도합니다."));
  }
  ok = commandMissionStorageAlign();
  return ok;
}

bool commandMissionStorageGripAlignForLayer(bool lower) {
  int16_t targetSl = 0;
  int16_t targetFl = 0;
  int16_t targetFr = 0;
  int16_t tolerance = 0;
  readMissionGripAlignTarget(lower, &targetSl, &targetFl, &targetFr, &tolerance);
  (void)targetSl;
  return commandMissionStorageFrontDepthAlignToTarget(
    lower
      ? F("[mission move] 집기 직전: 하층 그립 기준 FL/FR 전후 깊이 정렬")
      : F("[mission move] 집기 직전: 상층 그립 기준 FL/FR 전후 깊이 정렬"),
    targetFl, targetFr, tolerance, CFG.wait.driveSettleMs);
}

bool commandMissionStorageGripAlign() {
  return commandMissionStorageGripAlignForLayer(missionPickLayer == "lower");
}

float missionGripDepthMmForLayer(bool lower) {
  return lower ? missionLowerGripDepthMm : missionUpperGripDepthMm;
}

bool validateMissionGripDepth(float distanceMm, long speedMmPerSec) {
  if (distanceMm < 0.0 || distanceMm > 80.0) {
    DEBUG_SERIAL.println(F("[제한] grip depth는 0~80mm 범위에서 테스트하세요."));
    return false;
  }
  return validateMissionPositionSpeed(speedMmPerSec);
}

void printMissionGripDepthStatus() {
  DEBUG_SERIAL.print(F("[mission gripdepth] upper="));
  DEBUG_SERIAL.print(missionUpperGripDepthMm, 2);
  DEBUG_SERIAL.print(F("mm, lower="));
  DEBUG_SERIAL.print(missionLowerGripDepthMm, 2);
  DEBUG_SERIAL.print(F("mm @ "));
  DEBUG_SERIAL.print(missionGripDepthMmPerSec);
  DEBUG_SERIAL.println(F("mm/s"));
  DEBUG_SERIAL.print(F("{\"type\":\"mission-gripdepth\",\"upperMm\":"));
  DEBUG_SERIAL.print(missionUpperGripDepthMm, 2);
  DEBUG_SERIAL.print(F(",\"lowerMm\":"));
  DEBUG_SERIAL.print(missionLowerGripDepthMm, 2);
  DEBUG_SERIAL.print(F(",\"speedMmPerSec\":"));
  DEBUG_SERIAL.print(missionGripDepthMmPerSec);
  DEBUG_SERIAL.println(F("}"));
}

bool commandMissionGripDepthMoveForLayer(bool lower,
                                         uint8_t direction,
                                         const __FlashStringHelper *label) {
  float distanceMm = missionGripDepthMmForLayer(lower);
  if (distanceMm <= 0.0) {
    DEBUG_SERIAL.println(F("[mission gripdepth] 추가 전진값이 0mm라 이동을 생략합니다."));
    return true;
  }
  if (!validateMissionGripDepth(distanceMm, missionGripDepthMmPerSec)) return false;
  return commandMissionDriveDistanceStep(label, distanceMm, direction,
                                         missionGripDepthMmPerSec, false);
}

bool commandMissionGripDepthForwardForLayer(bool lower) {
  return commandMissionGripDepthMoveForLayer(
    lower, DRIVE_DIRECTION_FORWARD,
    lower
      ? F("[mission move] 하층 그립 깊이 추가 전진")
      : F("[mission move] 상층 그립 깊이 추가 전진"));
}

bool commandMissionGripDepthRetreatForLayer(bool lower) {
  return commandMissionGripDepthMoveForLayer(
    lower, DRIVE_DIRECTION_BACKWARD,
    lower
      ? F("[mission move] 하층 집기 후 깊이 복귀")
      : F("[mission move] 상층 집기 후 깊이 복귀"));
}

bool commandMissionGripDepthTuning(const String &input) {
  String mode = tokenAt(input, 2);
  mode.toLowerCase();

  if (mode == "run" || mode == "test" || mode == "실행" || mode == "테스트") {
    bool lower = missionPickLayer == "lower";
    if (parseGripAlignLayer(tokenAt(input, 3), &lower)) {}
    bool ok = commandMissionGripDepthForwardForLayer(lower);
    if (ok) ok = commandMissionGripDepthRetreatForLayer(lower);
    printMissionGripDepthStatus();
    return ok;
  }

  bool layerMode = false;
  bool lower = false;
  uint8_t argOffset = 0;
  if (parseGripAlignLayer(mode, &lower)) {
    layerMode = true;
    argOffset = 1;
  }

  if (tokenCount(input) < (layerMode ? 4 : 5)) {
    printMissionGripDepthStatus();
    DEBUG_SERIAL.println(F("사용법: mission gripdepth <upperMm> <lowerMm> [speedMm/s]"));
    DEBUG_SERIAL.println(F("       mission gripdepth upper|lower <mm> [speedMm/s]"));
    DEBUG_SERIAL.println(F("       mission gripdepth run [upper|lower]"));
    DEBUG_SERIAL.println(F("예시: mission gripdepth 8 5 60"));
    return true;
  }

  long speedMmPerSec = missionGripDepthMmPerSec;
  if (layerMode) {
    float distanceMm = 0.0;
    if (!parseFloatStrict(tokenAt(input, 3), &distanceMm)) {
      DEBUG_SERIAL.println(F("사용법: mission gripdepth upper|lower <mm> [speedMm/s]"));
      return false;
    }
    if (tokenCount(input) >= 5 &&
        !parseLongStrict(tokenAt(input, 4), &speedMmPerSec)) {
      DEBUG_SERIAL.println(F("사용법: mission gripdepth upper|lower <mm> [speedMm/s]"));
      return false;
    }
    if (!validateMissionGripDepth(distanceMm, speedMmPerSec)) return false;
    if (lower) missionLowerGripDepthMm = distanceMm;
    else missionUpperGripDepthMm = distanceMm;
    missionGripDepthMmPerSec = speedMmPerSec;
    printMissionGripDepthStatus();
    return true;
  }

  float upperMm = 0.0;
  float lowerMm = 0.0;
  if (!parseFloatStrict(tokenAt(input, 2), &upperMm) ||
      !parseFloatStrict(tokenAt(input, 3), &lowerMm)) {
    DEBUG_SERIAL.println(F("사용법: mission gripdepth <upperMm> <lowerMm> [speedMm/s]"));
    return false;
  }
  if (tokenCount(input) >= 5 &&
      !parseLongStrict(tokenAt(input, 4), &speedMmPerSec)) {
    DEBUG_SERIAL.println(F("사용법: mission gripdepth <upperMm> <lowerMm> [speedMm/s]"));
    return false;
  }
  if (!validateMissionGripDepth(upperMm, speedMmPerSec) ||
      !validateMissionGripDepth(lowerMm, speedMmPerSec)) {
    return false;
  }
  missionUpperGripDepthMm = upperMm;
  missionLowerGripDepthMm = lowerMm;
  missionGripDepthMmPerSec = speedMmPerSec;
  printMissionGripDepthStatus();
  return true;
}

bool commandMissionMoveToPlaceAlign() {
  if (!ensureMobileReady()) return false;

  DEBUG_SERIAL.println(F("[mission move] 미션수행존 배치 기준 SL PSD 정렬"));
  DEBUG_SERIAL.print(F("  target SL/FR/tol="));
  DEBUG_SERIAL.print(missionPlaceSl);
  DEBUG_SERIAL.print(F("/"));
  DEBUG_SERIAL.print(missionPlaceFr);
  DEBUG_SERIAL.print(F("/"));
  DEBUG_SERIAL.println(missionPlaceTolerance);
  DEBUG_SERIAL.print(F("  place align speed="));
  DEBUG_SERIAL.println(missionMotion.missionZonePlaceCorrectionSpeed);

  ChangeMobilebaseMode2VelocityControlMode(dxl);
  int16_t slVal = 0;
  int16_t flVal = 0;
  int16_t frVal = 0;
  int16_t frontError = 0;
  unsigned long startedAt = millis();
  while (true) {
    if (checkEmergencyStopInput()) {
      stopAll(F("[긴급정지] ! 입력"));
      return false;
    }
    readSideLeftPSDSensor(&slVal);
    readFrontPSDSensors(&flVal, &frVal);
    frontError = frVal - missionPlaceFr;
    if (!LocatingWithTwoSensors(dxl, slVal - missionPlaceSl,
                                frontError, missionPlaceTolerance,
                                DRIVE_DIRECTION_LEFT,
                                missionMotion.missionZonePlaceCorrectionSpeed)) {
      break;
    }
    if (millis() - startedAt > CFG.timeout.psdLoopMs) {
      DEBUG_SERIAL.println(F("  미션수행존 이동 타임아웃"));
      break;
    }
    delay(10);
  }
  stopMobilebase();
  DEBUG_SERIAL.print(F("  미션수행존 이동 종료 SL="));
  DEBUG_SERIAL.print(slVal);
  DEBUG_SERIAL.print(F(" FL="));
  DEBUG_SERIAL.print(flVal);
  DEBUG_SERIAL.print(F(" FR="));
  DEBUG_SERIAL.print(frVal);
  DEBUG_SERIAL.print(F(" FRerr="));
  DEBUG_SERIAL.println(frontError);
  clearMissionUndoCandidate();
  return interruptibleDelay(CFG.wait.driveSettleMs);
}

void printMissionFinishAlignStatus() {
  DEBUG_SERIAL.print(F("[mission finishalign] preAlign SL="));
  DEBUG_SERIAL.print(missionFinishPreAlignSl);
  DEBUG_SERIAL.print(F(", tolerance="));
  DEBUG_SERIAL.print(missionFinishPreAlignTolerance);
  DEBUG_SERIAL.print(F(", speed="));
  DEBUG_SERIAL.println(missionFinishPreAlignSpeed);
  DEBUG_SERIAL.print(F("{\"type\":\"mission-finishalign\",\"sl\":"));
  DEBUG_SERIAL.print(missionFinishPreAlignSl);
  DEBUG_SERIAL.print(F(",\"tolerance\":"));
  DEBUG_SERIAL.print(missionFinishPreAlignTolerance);
  DEBUG_SERIAL.print(F(",\"speed\":"));
  DEBUG_SERIAL.print(missionFinishPreAlignSpeed);
  DEBUG_SERIAL.println(F(",\"sideRightUsed\":false}"));
}

bool commandMissionMoveToFinishPreAlign() {
  if (!ensureMobileReady()) return false;

  DEBUG_SERIAL.println(F("[mission finishalign] 후진 전 미션수행존 SL 기준 위치로 보정"));
  DEBUG_SERIAL.print(F("  target SL/tol/speed="));
  DEBUG_SERIAL.print(missionFinishPreAlignSl);
  DEBUG_SERIAL.print(F("/"));
  DEBUG_SERIAL.print(missionFinishPreAlignTolerance);
  DEBUG_SERIAL.print(F("/"));
  DEBUG_SERIAL.println(missionFinishPreAlignSpeed);

  ChangeMobilebaseMode2VelocityControlMode(dxl);
  int16_t slVal = 0;
  unsigned long startedAt = millis();
  while (true) {
    if (checkEmergencyStopInput()) {
      stopAll(F("[긴급정지] ! 입력"));
      return false;
    }
    readSideLeftPSDSensor(&slVal);
    if (!DriveWithOneSensor(dxl,
                            slVal - missionFinishPreAlignSl,
                            missionFinishPreAlignTolerance,
                            DRIVE_DIRECTION_LEFT,
                            missionFinishPreAlignSpeed)) {
      break;
    }
    if (millis() - startedAt > CFG.timeout.psdLoopMs) {
      DEBUG_SERIAL.println(F("  finish 시작 SL 보정 타임아웃"));
      break;
    }
    delay(10);
  }
  stopMobilebase();
  DEBUG_SERIAL.print(F("  finish 시작 SL 보정 종료 SL="));
  DEBUG_SERIAL.println(slVal);
  clearMissionUndoCandidate();
  interruptibleDelay(CFG.wait.driveSettleMs);
  return abs(slVal - missionFinishPreAlignSl) <= missionFinishPreAlignTolerance;
}

bool commandMissionFinishAlignTuning(const String &input) {
  String mode = tokenAt(input, 2);
  mode.toLowerCase();
  if (mode == "run" || mode == "go" || mode == "move" || mode == "실행" || mode == "이동") {
    return commandMissionMoveToFinishPreAlign();
  }
  if (mode == "current" || mode == "now" || mode == "현재") {
    long tolerance = missionFinishPreAlignTolerance;
    long speed = missionFinishPreAlignSpeed;
    if (tokenCount(input) >= 4 &&
        !parseLongStrict(tokenAt(input, 3), &tolerance)) {
      DEBUG_SERIAL.println(F("사용법: mission finishalign current [tolerance] [speed]"));
      return false;
    }
    if (tokenCount(input) >= 5 &&
        !parseLongStrict(tokenAt(input, 4), &speed)) {
      DEBUG_SERIAL.println(F("사용법: mission finishalign current [tolerance] [speed]"));
      return false;
    }
    if (tolerance < 1 || tolerance > 80 || speed < 1 || speed > profile().maxMissionVelocityRaw) {
      DEBUG_SERIAL.println(F("[제한] tolerance는 1~80, speed는 현재 profile 속도 한도 안에서 지정하세요."));
      return false;
    }
    int16_t slVal = 0;
    readSideLeftPSDSensor(&slVal);
    missionFinishPreAlignSl = slVal;
    missionFinishPreAlignTolerance = (int16_t)tolerance;
    missionFinishPreAlignSpeed = (int32_t)speed;
    DEBUG_SERIAL.println(F("[mission finishalign] 현재 SL 값을 finish 시작 기준으로 적용했습니다."));
    printMissionFinishAlignStatus();
    printPsdStatus();
    return true;
  }

  if (tokenCount(input) < 3) {
    printMissionFinishAlignStatus();
    DEBUG_SERIAL.println(F("사용법: mission finishalign <targetSl> [tolerance] [speed]"));
    DEBUG_SERIAL.println(F("       mission finishalign current [tolerance] [speed]"));
    DEBUG_SERIAL.println(F("       mission finishalign run"));
    DEBUG_SERIAL.println(F("예시: mission finishalign 570 20 80"));
    DEBUG_SERIAL.println(F("finish 후진 직전에 SL만 먼저 맞춥니다. 배치용 placealign과 별도 기준입니다."));
    return true;
  }

  long targetSl = 0;
  long tolerance = missionFinishPreAlignTolerance;
  long speed = missionFinishPreAlignSpeed;
  if (!parseLongStrict(tokenAt(input, 2), &targetSl)) {
    DEBUG_SERIAL.println(F("사용법: mission finishalign <targetSl> [tolerance] [speed]"));
    return false;
  }
  if (tokenCount(input) >= 4 &&
      !parseLongStrict(tokenAt(input, 3), &tolerance)) {
    DEBUG_SERIAL.println(F("사용법: mission finishalign <targetSl> [tolerance] [speed]"));
    return false;
  }
  if (tokenCount(input) >= 5 &&
      !parseLongStrict(tokenAt(input, 4), &speed)) {
    DEBUG_SERIAL.println(F("사용법: mission finishalign <targetSl> [tolerance] [speed]"));
    return false;
  }
  if (targetSl < 1 || targetSl > 1023 ||
      tolerance < 1 || tolerance > 80 ||
      speed < 1 || speed > profile().maxMissionVelocityRaw) {
    DEBUG_SERIAL.println(F("[제한] targetSl은 1~1023, tolerance는 1~80, speed는 현재 profile 속도 한도 안에서 지정하세요."));
    return false;
  }

  missionFinishPreAlignSl = (int16_t)targetSl;
  missionFinishPreAlignTolerance = (int16_t)tolerance;
  missionFinishPreAlignSpeed = (int32_t)speed;
  printMissionFinishAlignStatus();
  return true;
}

bool commandMissionMoveToZoneAndPlace(uint8_t slot) {
  DEBUG_SERIAL.print(F("[mission move] 미션수행존 "));
  DEBUG_SERIAL.print(slot);
  DEBUG_SERIAL.println(F("번 칸 이동 후 배치"));
  if (!commandMissionMoveToPlaceAlign()) return false;

  bool ok = commandSeq(String("seq place ") + String(slot));
  clearMissionUndoCandidate();
  return ok;
}

bool commandMissionRealignStorage() {
  if (!ensureMobileReady()) return false;

  DEBUG_SERIAL.println(F("[mission move] 적재함 기준 위치로 재정렬"));
  DEBUG_SERIAL.print(F("  psd speed="));
  DEBUG_SERIAL.println(missionMotion.psdCorrectionSpeed);
  ChangeMobilebaseMode2VelocityControlMode(dxl);
  int16_t slVal = 0;
  int16_t flVal = 0;
  int16_t frVal = 0;
  int16_t frontError = 0;
  unsigned long startedAt = millis();
  while (true) {
    if (checkEmergencyStopInput()) {
      stopAll(F("[긴급정지] ! 입력"));
      return false;
    }
    readSideLeftPSDSensor(&slVal);
    readFrontPSDSensors(&flVal, &frVal);
    frontError = storageAlignFrontError(flVal, frVal);
    if (!LocatingWithTwoSensors(dxl, slVal - missionAlignSl,
                                frontError, missionAlignTolerance,
                                DRIVE_DIRECTION_LEFT,
                                missionMotion.psdCorrectionSpeed)) {
      break;
    }
    if (millis() - startedAt > CFG.timeout.psdLoopMs) {
      DEBUG_SERIAL.println(F("  재정렬 타임아웃"));
      break;
    }
    delay(10);
  }
  stopMobilebase();
  DEBUG_SERIAL.print(F("  재정렬 종료 SL="));
  DEBUG_SERIAL.print(slVal);
  DEBUG_SERIAL.print(F(" FL="));
  DEBUG_SERIAL.print(flVal);
  DEBUG_SERIAL.print(F(" FR="));
  DEBUG_SERIAL.print(frVal);
  DEBUG_SERIAL.print(F(" frontError="));
  DEBUG_SERIAL.println(frontError);
  missionStorageColumn = 1;
  clearMissionUndoCandidate();
  return interruptibleDelay(CFG.wait.driveSettleMs);
}

void driveBackwardWithLeftBoundaryCorrection(int16_t slVal) {
  int32_t correctionSpeed = 0;

  if (slVal <= CFG.finishReturn.boundaryAdc) {
    correctionSpeed = -CFG.finishReturn.openSideLeftBiasSpeed;
  } else {
    int16_t sideError = slVal - CFG.finishReturn.trackSl;
    if (abs(sideError) > CFG.finishReturn.trackTolerance) {
      correctionSpeed = (int32_t)sideError * CFG.finishReturn.correctionMaxSpeed /
                        (CFG.finishReturn.trackTolerance * 4);
      correctionSpeed = clampInt32(correctionSpeed,
                                   -CFG.finishReturn.correctionMaxSpeed,
                                   CFG.finishReturn.correctionMaxSpeed);
    }
  }

  int32_t baseSpeed = CFG.speed.returnSpeed;
  int32_t flSpeed = clampInt32(-baseSpeed + correctionSpeed,
                               -CFG.finishReturn.wheelMaxSpeed,
                               CFG.finishReturn.wheelMaxSpeed);
  int32_t frSpeed = clampInt32(-baseSpeed - correctionSpeed,
                               -CFG.finishReturn.wheelMaxSpeed,
                               CFG.finishReturn.wheelMaxSpeed);
  int32_t blSpeed = clampInt32(-baseSpeed - correctionSpeed,
                               -CFG.finishReturn.wheelMaxSpeed,
                               CFG.finishReturn.wheelMaxSpeed);
  int32_t brSpeed = clampInt32(-baseSpeed + correctionSpeed,
                               -CFG.finishReturn.wheelMaxSpeed,
                               CFG.finishReturn.wheelMaxSpeed);

  SetMobileGoalVelocityForSyncWrite(dxl, flSpeed, frSpeed, blSpeed, brSpeed);
}

bool runFinishReturnPhase(const __FlashStringHelper *label, bool waitForBoundary,
                          unsigned long timeoutMs) {
  DEBUG_SERIAL.println(label);
  unsigned long startedAt = millis();
  int16_t slVal = 0;
  while (millis() - startedAt < timeoutMs) {
    if (checkEmergencyStopInput()) {
      stopAll(F("[긴급정지] ! 입력"));
      return false;
    }
    readSideLeftPSDSensor(&slVal);
    driveBackwardWithLeftBoundaryCorrection(slVal);
    bool boundaryDetected = slVal > CFG.finishReturn.boundaryAdc;
    if (waitForBoundary == boundaryDetected) {
      stopMobilebase();
      DEBUG_SERIAL.print(F("  SL="));
      DEBUG_SERIAL.println(slVal);
      return interruptibleDelay(CFG.wait.driveSettleMs);
    }
    delay(10);
  }
  stopMobilebase();
  DEBUG_SERIAL.println(F("  타임아웃: 다음 finish phase로 넘어갑니다."));
  return interruptibleDelay(CFG.wait.driveSettleMs);
}

bool commandMissionFinish() {
  if (!ensureMobileReady()) return false;

  DEBUG_SERIAL.println(F("[mission finish] finish 시작 SL 보정 후 후진 finish 실행"));
  DEBUG_SERIAL.println(F("  적재함 재정렬 없이, SL을 미션수행존 finish 시작 기준에 맞춘 뒤 후진합니다."));
  DEBUG_SERIAL.print(F("{\"type\":\"mission-finish\",\"returnSpeed\":"));
  DEBUG_SERIAL.print(CFG.speed.returnSpeed);
  DEBUG_SERIAL.print(F(",\"boundaryAdc\":"));
  DEBUG_SERIAL.print(CFG.finishReturn.boundaryAdc);
  DEBUG_SERIAL.print(F(",\"preAlignSl\":"));
  DEBUG_SERIAL.print(missionFinishPreAlignSl);
  DEBUG_SERIAL.print(F(",\"preAlignTolerance\":"));
  DEBUG_SERIAL.print(missionFinishPreAlignTolerance);
  DEBUG_SERIAL.print(F(",\"preAlignSpeed\":"));
  DEBUG_SERIAL.print(missionFinishPreAlignSpeed);
  DEBUG_SERIAL.print(F(",\"trackSl\":"));
  DEBUG_SERIAL.print(CFG.finishReturn.trackSl);
  DEBUG_SERIAL.print(F(",\"finishExtraMs\":"));
  DEBUG_SERIAL.print(CFG.finishReturn.finishExtraMs);
  DEBUG_SERIAL.println(F("}"));

  if (!runPoseRequired(POSE_STORAGE, missionMotion.storagePoseMs, 0.0)) return false;
  if (!runPoseRequired(POSE_INITIAL, missionMotion.returnPoseMs, -360.0)) return false;
  pixy.setLamp(0, 0);
  clearMissionUndoCandidate();
  ChangeMobilebaseMode2VelocityControlMode(dxl);
  if (!commandMissionMoveToFinishPreAlign()) {
    if (checkEmergencyStopInput()) return false;
    DEBUG_SERIAL.println(F("  [주의] finish 시작 SL 보정이 목표 범위 밖에서 끝났습니다. 현재 위치 기준으로 후진을 계속합니다."));
  }

  if (!runFinishReturnPhase(F("  [finish-1] SL 장애물 없어질 때까지 후진"),
                            false, CFG.timeout.returnPhaseMs)) return false;
  if (!runFinishReturnPhase(F("  [finish-2] SL 장애물 다시 감지될 때까지 후진"),
                            true, CFG.timeout.returnPhaseMs)) return false;
  if (!runFinishReturnPhase(F("  [finish-3] SL 장애물 없어질 때까지 후진"),
                            false, CFG.timeout.returnPhaseMs)) return false;

  DEBUG_SERIAL.print(F("  [finish-4] 추가 후진 "));
  DEBUG_SERIAL.print(CFG.finishReturn.finishExtraMs);
  DEBUG_SERIAL.println(F("ms"));
  unsigned long startedAt = millis();
  int16_t slVal = 0;
  while (millis() - startedAt < CFG.finishReturn.finishExtraMs) {
    if (checkEmergencyStopInput()) {
      stopAll(F("[긴급정지] ! 입력"));
      return false;
    }
    readSideLeftPSDSensor(&slVal);
    driveBackwardWithLeftBoundaryCorrection(slVal);
    delay(10);
  }
  stopMobilebase();
  if (!runPoseRequired(POSE_INITIAL, missionMotion.returnPoseMs, -360.0)) return false;
  missionStage = MISSION_FINISHED;
  missionButtonMode = false;
  setRGBLEDOff();
  DEBUG_SERIAL.println(F("[mission finish] 완료"));
  return true;
}

const __FlashStringHelper *missionStageName(MissionStepperStage stage) {
  switch (stage) {
    case MISSION_IDLE: return F("idle");
    case MISSION_START_TO_INSTRUCTION: return F("start-to-instruction");
    case MISSION_INSTRUCTION_SCAN_HOLD: return F("instruction-scan-hold");
    case MISSION_GO_TO_STORAGE: return F("go-to-storage");
    case MISSION_COLUMN_MOVE_OR_SCAN: return F("column-move-or-scan");
    case MISSION_PICK_HOLD: return F("pick-hold");
    case MISSION_PLACE_HOLD: return F("place-hold");
    case MISSION_REALIGN_OR_NEXT: return F("realign-or-next");
    case MISSION_FINISH: return F("finish");
    case MISSION_FINISHED: return F("finished");
  }
  return F("unknown");
}

uint8_t missionSourceSlotForCurrentBlock() {
  if (missionQueueCount > 0) {
    if (missionBlockIndex < 1 || missionBlockIndex > missionQueueCount) return 0;
    return missionQueueSourceSlots[missionBlockIndex - 1];
  }
  if (missionBlockIndex < 1 || missionBlockIndex > CFG.storageRack.pickSlotCount) return 0;
  return CFG.storageRack.pickSlotOrder[missionBlockIndex - 1];
}

uint8_t missionPlaceSlotForCurrentBlock() {
  if (missionQueueCount > 0) {
    if (missionBlockIndex < 1 || missionBlockIndex > missionQueueCount) return 0;
    return missionQueueGoalSlots[missionBlockIndex - 1];
  }
  if (missionBlockIndex < 1 || missionBlockIndex > MissionConfig::MAX_MISSION_BLOCKS) return 0;
  uint8_t slot = CFG.mission.goalPositions[missionBlockIndex - 1];
  return (slot >= 1 && slot <= CFG.pose.missionZoneSlotCount) ? slot : 0;
}

void clearMissionQueue() {
  missionQueueCount = 0;
  memset(missionQueueSignatures, 0, sizeof(missionQueueSignatures));
  memset(missionQueueSourceSlots, 0, sizeof(missionQueueSourceSlots));
  memset(missionQueueGoalSlots, 0, sizeof(missionQueueGoalSlots));
  memset(missionQueueCompleted, 0, sizeof(missionQueueCompleted));
  missionStorageScanTargetColumn = 1;
}

void clearMissionColumnScanDecision() {
  missionColumnScanHasDecision = false;
  missionColumnSearchMissColumn = 0;
  missionDetectedSignature = 0;
  missionDetectedSourceSlot = 0;
  missionDetectedGoalSlot = 0;
  missionDetectedPickupRegion = 0;
  missionDetectedPixyColumn = 0;
  missionDetectedX = 0;
  missionDetectedY = 0;
  missionDetectedArea = 0;
}

void printMissionQueueJsonLine() {
  DEBUG_SERIAL.print(F("{\"type\":\"mission-queue\",\"count\":"));
  DEBUG_SERIAL.print(missionQueueCount);
  DEBUG_SERIAL.print(F(",\"items\":["));
  for (uint8_t i = 0; i < missionQueueCount; i++) {
    if (i > 0) DEBUG_SERIAL.print(F(","));
    DEBUG_SERIAL.print(F("{\"index\":"));
    DEBUG_SERIAL.print(i + 1);
    DEBUG_SERIAL.print(F(",\"sig\":"));
    DEBUG_SERIAL.print(missionQueueSignatures[i]);
    DEBUG_SERIAL.print(F(",\"sourceSlot\":"));
    DEBUG_SERIAL.print(missionQueueSourceSlots[i]);
    DEBUG_SERIAL.print(F(",\"goalSlot\":"));
    DEBUG_SERIAL.print(missionQueueGoalSlots[i]);
    DEBUG_SERIAL.print(F(",\"completed\":"));
    printJsonBool(missionQueueCompleted[i]);
    DEBUG_SERIAL.print(F("}"));
  }
  DEBUG_SERIAL.println(F("]}"));
}

bool commitMissionInstructionQueue(int16_t *bestX, uint8_t *bestSig, uint8_t bestCount) {
  if (bestCount == 0) {
    DEBUG_SERIAL.println(F("[mission scan] 감지된 블록이 없습니다."));
    turnOnLEDRed500ms();
    return false;
  }

  clearMissionQueue();
  sortMissionInstructionBlocksByX(bestX, bestSig, bestCount);
  missionQueueCount = CFG.mission.dynamicBlockCount
                        ? bestCount
                        : min(bestCount, CFG.mission.blockCount);
  missionQueueCount = min(missionQueueCount, CFG.storageRack.pickSlotCount);
  missionQueueCount = min(missionQueueCount, CFG.pose.missionZoneSlotCount);
  missionQueueCount = min(missionQueueCount, (uint8_t)MissionConfig::MAX_MISSION_BLOCKS);

  DEBUG_SERIAL.print(F("[mission queue] count="));
  DEBUG_SERIAL.println(missionQueueCount);
  for (uint8_t i = 0; i < missionQueueCount; i++) {
    missionQueueSignatures[i] = bestSig[i];
    missionQueueSourceSlots[i] = 0;
    missionQueueGoalSlots[i] = 0;
    missionQueueCompleted[i] = false;
    DEBUG_SERIAL.print(F("  #"));
    DEBUG_SERIAL.print(i + 1);
    DEBUG_SERIAL.print(F(" sig="));
    DEBUG_SERIAL.print(missionQueueSignatures[i]);
    DEBUG_SERIAL.print(F(", sourceSlot=scan"));
    DEBUG_SERIAL.println(F(", goalSlot=sourceSlot after storage survey/columnscan"));
  }
  printMissionQueueJsonLine();
  turnOnLEDGreen500ms();
  return missionQueueCount > 0;
}

bool buildMissionQueueFromInstructionScan() {
  clearMissionQueue();

  uint8_t signatureMap = missionInstructionSignatureMap();
  if (signatureMap == 0) {
    DEBUG_SERIAL.println(F("[mission scan] signature map이 비어 있습니다."));
    return false;
  }

  uint8_t lamp = CFG.cameraScan.missionInstructionLampOn ? 1 : 0;
  pixy.setLamp(lamp, lamp);
  if (!interruptibleDelay(CFG.wait.cameraLampMs)) return false;

  int16_t bestX[MissionConfig::MAX_MISSION_BLOCKS] = {0};
  uint8_t bestSig[MissionConfig::MAX_MISSION_BLOCKS] = {0};
  uint8_t bestCount = 0;
  uint32_t bestAreaScore = 0;

  DEBUG_SERIAL.print(F("[mission scan] signatureMap="));
  printSignatureMap(signatureMap);
  DEBUG_SERIAL.print(F(", minArea="));
  DEBUG_SERIAL.print(CFG.cameraScan.missionInstructionMinBlockArea);
  DEBUG_SERIAL.print(F(", scanMs="));
  DEBUG_SERIAL.print(missionInstructionScanMs);
  DEBUG_SERIAL.print(F(", sampleMs="));
  DEBUG_SERIAL.println(missionInstructionScanSampleMs);

  for (uint8_t pass = 0; pass < 2 && bestCount == 0; pass++) {
    uint16_t minArea = (pass == 0) ? CFG.cameraScan.missionInstructionMinBlockArea : 0;
    if (pass == 1 && CFG.cameraScan.missionInstructionMinBlockArea == 0) break;
    if (pass == 1) DEBUG_SERIAL.println(F("[mission scan] minArea=0 fallback"));

    unsigned long startedAt = millis();
    while (millis() - startedAt < missionInstructionScanMs) {
      if (checkEmergencyStopInput()) {
        stopAll(F("[긴급정지] ! 입력"));
        return false;
      }

      updateMissionInstructionBestFromPixy(signatureMap, minArea,
                                           bestX, bestSig,
                                           &bestCount, &bestAreaScore);
      if (!interruptibleDelay(missionInstructionScanSampleMs)) return false;
    }
  }

  return commitMissionInstructionQueue(bestX, bestSig, bestCount);
}

uint8_t storageColumnForSourceSlot(uint8_t sourceSlot) {
  for (uint8_t i = 0; i < 4; i++) {
    if (CFG.storageRack.upperRowSlots[i] == sourceSlot ||
        CFG.storageRack.lowerRowSlots[i] == sourceSlot) {
      return i + 1;
    }
  }
  return 0;
}

uint8_t missionTargetStorageColumn() {
  uint8_t sourceSlot = missionSourceSlotForCurrentBlock();
  if (sourceSlot == 0) {
    return (missionStorageScanTargetColumn >= 1 && missionStorageScanTargetColumn <= 4)
             ? missionStorageScanTargetColumn
             : 1;
  }
  return storageColumnForSourceSlot(sourceSlot);
}

uint8_t firstPendingMissionQueueIndex() {
  for (uint8_t i = 0; i < missionQueueCount; i++) {
    if (!missionQueueCompleted[i]) return i + 1;
  }
  return 0;
}

bool missionHasPendingQueueBlocks() {
  return firstPendingMissionQueueIndex() != 0;
}

uint8_t pendingMissionSignatureMap() {
  uint8_t map = 0;
  for (uint8_t i = 0; i < missionQueueCount; i++) {
    if (missionQueueCompleted[i]) continue;
    uint8_t sig = missionQueueSignatures[i];
    if (sig >= 1 && sig <= CFG.cameraScan.maxSignature) {
      map |= ((uint8_t)1 << (sig - 1));
    }
  }
  if (missionQueueCount == 0) {
    map = CFG.cameraScan.storageAllowedSignatureMap & 0x7F;
  }
  return map & CFG.cameraScan.storageAllowedSignatureMap & 0x7F;
}

uint8_t pendingMissionQueueIndexForSignature(uint8_t signature) {
  for (uint8_t i = 0; i < missionQueueCount; i++) {
    if (missionQueueCompleted[i]) continue;
    if (missionQueueSignatures[i] == signature) return i + 1;
  }
  return 0;
}

void selectFirstPendingMissionBlockForPrompt() {
  uint8_t nextIndex = firstPendingMissionQueueIndex();
  if (nextIndex != 0) missionBlockIndex = nextIndex;
}

void markCurrentMissionBlockCompleted() {
  if (missionQueueCount == 0) return;
  if (missionBlockIndex < 1 || missionBlockIndex > missionQueueCount) return;
  missionQueueCompleted[missionBlockIndex - 1] = true;
}

void advanceMissionStorageScanTargetColumn() {
  if (missionStorageScanTargetColumn < 4) {
    missionStorageScanTargetColumn++;
  }
}

uint8_t sourceSlotForStorageColumnAndLayer(uint8_t column, bool upperLayer) {
  if (column < 1 || column > 4) return 0;
  return upperLayer
           ? CFG.storageRack.upperRowSlots[column - 1]
           : CFG.storageRack.lowerRowSlots[column - 1];
}

bool missionPickLayerForSourceSlot(uint8_t sourceSlot, String *layer) {
  for (uint8_t i = 0; i < 4; i++) {
    if (CFG.storageRack.upperRowSlots[i] == sourceSlot) {
      *layer = "upper";
      return true;
    }
  }
  for (uint8_t i = 0; i < 4; i++) {
    if (CFG.storageRack.lowerRowSlots[i] == sourceSlot) {
      *layer = "lower";
      return true;
    }
  }
  return false;
}

uint8_t sourceSlotForStorageColumnAndRegion(uint8_t column, uint8_t pickupRegion) {
  if (pickupRegion == 0) return 0;
  return sourceSlotForStorageColumnAndLayer(
    column, storagePickupRegionUsesUpperGrip(pickupRegion));
}

uint8_t pendingMissionTargetCountForSignature(uint8_t signature) {
  uint8_t count = 0;
  for (uint8_t i = 0; i < missionQueueCount; i++) {
    if (missionQueueCompleted[i]) continue;
    if (missionQueueSignatures[i] == signature) count++;
  }
  return count;
}

uint8_t missionSurveyDetectionCountForSignature(uint8_t signature) {
  uint8_t count = 0;
  for (uint8_t i = 0; i < missionSurveyDetectionCount; i++) {
    if (missionSurveyDetections[i].signature == signature) count++;
  }
  return count;
}

bool missionSurveyCoversPendingTargets() {
  if (missionQueueCount == 0) return false;
  for (uint8_t i = 0; i < missionQueueCount; i++) {
    if (missionQueueCompleted[i]) continue;
    uint8_t signature = missionQueueSignatures[i];
    if (missionSurveyDetectionCountForSignature(signature) <
        pendingMissionTargetCountForSignature(signature)) {
      return false;
    }
  }
  return true;
}

bool upsertMissionSurveyDetection(uint8_t signature,
                                  uint8_t sourceSlot,
                                  uint8_t column,
                                  uint8_t pickupRegion,
                                  int16_t x,
                                  int16_t y,
                                  uint32_t area,
                                  const PsdSnapshot &psdSnapshot) {
  for (uint8_t i = 0; i < missionSurveyDetectionCount; i++) {
    MissionStorageSurveyDetection &detection = missionSurveyDetections[i];
    if (detection.signature == signature && detection.sourceSlot == sourceSlot) {
      if (area > detection.area) {
        detection.column = column;
        detection.pickupRegion = pickupRegion;
        detection.x = x;
        detection.y = y;
        detection.area = area;
        detection.psdFl = psdSnapshot.fl;
        detection.psdFr = psdSnapshot.fr;
        detection.psdSl = psdSnapshot.sl;
        detection.psdSr = psdSnapshot.sr;
      }
      return true;
    }
  }

  if (missionSurveyDetectionCount >= MissionConfig::MAX_MISSION_BLOCKS) {
    DEBUG_SERIAL.println(F("[mission survey] detection buffer가 가득 찼습니다."));
    return false;
  }

  MissionStorageSurveyDetection &detection =
    missionSurveyDetections[missionSurveyDetectionCount++];
  detection.signature = signature;
  detection.sourceSlot = sourceSlot;
  detection.column = column;
  detection.pickupRegion = pickupRegion;
  detection.x = x;
  detection.y = y;
  detection.area = area;
  detection.psdFl = psdSnapshot.fl;
  detection.psdFr = psdSnapshot.fr;
  detection.psdSl = psdSnapshot.sl;
  detection.psdSr = psdSnapshot.sr;
  detection.assigned = false;
  return true;
}

void printMissionSurveyDetectionJson(const MissionStorageSurveyDetection &detection) {
  DEBUG_SERIAL.print(F("{\"type\":\"mission-survey-detection\",\"sig\":"));
  DEBUG_SERIAL.print(detection.signature);
  DEBUG_SERIAL.print(F(",\"sourceSlot\":"));
  DEBUG_SERIAL.print(detection.sourceSlot);
  DEBUG_SERIAL.print(F(",\"goalSlot\":"));
  DEBUG_SERIAL.print(detection.sourceSlot);
  DEBUG_SERIAL.print(F(",\"poseId\":"));
  DEBUG_SERIAL.print(CFG.pose.missionZoneStartId + detection.sourceSlot);
  DEBUG_SERIAL.print(F(",\"column\":"));
  DEBUG_SERIAL.print(detection.column);
  DEBUG_SERIAL.print(F(",\"layer\":\""));
  DEBUG_SERIAL.print(storagePickupRegionUsesUpperGrip(detection.pickupRegion) ? F("upper") : F("lower"));
  DEBUG_SERIAL.print(F("\",\"x\":"));
  DEBUG_SERIAL.print(detection.x);
  DEBUG_SERIAL.print(F(",\"y\":"));
  DEBUG_SERIAL.print(detection.y);
  DEBUG_SERIAL.print(F(",\"area\":"));
  DEBUG_SERIAL.print(detection.area);
  DEBUG_SERIAL.print(F(",\"psd\":{\"fl\":"));
  DEBUG_SERIAL.print(detection.psdFl);
  DEBUG_SERIAL.print(F(",\"fr\":"));
  DEBUG_SERIAL.print(detection.psdFr);
  DEBUG_SERIAL.print(F(",\"sl\":"));
  DEBUG_SERIAL.print(detection.psdSl);
  DEBUG_SERIAL.print(F(",\"sr\":"));
  DEBUG_SERIAL.print(detection.psdSr);
  DEBUG_SERIAL.print(F("}"));
  DEBUG_SERIAL.println(F("}"));
}

bool commandMissionSurveyMoveToColumn(uint8_t targetColumn) {
  if (missionStorageColumn < 1 || missionStorageColumn > 4) {
    missionStorageColumn = 1;
    DEBUG_SERIAL.println(F("[mission survey] currentColumn이 비정상이라 1열 기준으로 보정합니다."));
  }
  while (missionStorageColumn != targetColumn) {
    uint8_t direction = (targetColumn > missionStorageColumn)
                          ? DRIVE_DIRECTION_RIGHT
                          : DRIVE_DIRECTION_LEFT;
    const __FlashStringHelper *label = (direction == DRIVE_DIRECTION_RIGHT)
                                         ? F("  survey 다음 열 우측 이동")
                                         : F("  survey 이전 열 좌측 이동");
    if (!commandMissionDriveDistanceStep(label,
                                         missionColumnStepMm,
                                         direction,
                                         missionColumnMoveMmPerSec)) {
      return false;
    }
    if (direction == DRIVE_DIRECTION_RIGHT) missionStorageColumn++;
    else missionStorageColumn--;
    missionStorageScanTargetColumn = missionStorageColumn;
  }
  return true;
}

bool commandMissionSurveyScanCurrentColumn(uint8_t signatureMap) {
  if (!pixyReady) {
    DEBUG_SERIAL.println(F("[주의] Pixy init이 실패했을 수 있습니다. 그래도 survey를 시도합니다."));
  }

  uint8_t frames = constrain(missionColumnScanFrames, 1, 20);
  uint16_t minArea = CFG.storageRack.scanMinBlockArea;
  bool foundAny = false;
  DEBUG_SERIAL.print(F("[mission survey] column="));
  DEBUG_SERIAL.print(missionStorageColumn);
  DEBUG_SERIAL.print(F(", signatureMap="));
  printSignatureMap(signatureMap);
  DEBUG_SERIAL.print(F(", frames="));
  DEBUG_SERIAL.print(frames);
  DEBUG_SERIAL.print(F(", settleMs="));
  DEBUG_SERIAL.print(missionColumnScanSettleMs);
  DEBUG_SERIAL.print(F(", sampleMs="));
  DEBUG_SERIAL.print(missionColumnScanSampleMs);
  DEBUG_SERIAL.print(F(", minArea="));
  DEBUG_SERIAL.println(minArea);
  if (missionColumnScanSettleMs > 0 &&
      !interruptibleDelay(missionColumnScanSettleMs)) return false;

  for (uint8_t frame = 0; frame < frames; frame++) {
    PsdSnapshot psdSnapshot;
    readAllPSDSensors(&psdSnapshot);

    pixy.ccc.getBlocks(true, signatureMap);
    DEBUG_SERIAL.print(F("  frame "));
    DEBUG_SERIAL.print(frame + 1);
    DEBUG_SERIAL.print(F(": count="));
    DEBUG_SERIAL.println(pixy.ccc.numBlocks);

    for (uint8_t i = 0; i < pixy.ccc.numBlocks; i++) {
      uint8_t sig = pixy.ccc.blocks[i].m_signature;
      if (sig < 1 || sig > CFG.cameraScan.maxSignature) continue;
      if ((((uint8_t)1 << (sig - 1)) & signatureMap) == 0) continue;
      uint32_t area = pixyBlockArea(i);
      if (area < minArea) continue;

      int16_t x = pixy.ccc.blocks[i].m_x;
      int16_t y = pixy.ccc.blocks[i].m_y;
      uint8_t pickupRegion = storagePickupRegionForPixyXY(x, y);
      uint8_t sourceSlot = sourceSlotForStorageColumnAndRegion(missionStorageColumn,
                                                               pickupRegion);
      DEBUG_SERIAL.print(F("    seen sig="));
      DEBUG_SERIAL.print(sig);
      DEBUG_SERIAL.print(F(", x="));
      DEBUG_SERIAL.print(x);
      DEBUG_SERIAL.print(F(", y="));
      DEBUG_SERIAL.print(y);
      DEBUG_SERIAL.print(F(", pickup="));
      DEBUG_SERIAL.print(storagePickupRegionName(pickupRegion));
      DEBUG_SERIAL.print(F(", area="));
      DEBUG_SERIAL.print(area);
      if (pickupRegion == 0 || sourceSlot == 0) {
        DEBUG_SERIAL.println(F(" -> reject"));
        continue;
      }
      DEBUG_SERIAL.println(F(" -> accept"));
      if (upsertMissionSurveyDetection(sig, sourceSlot, missionStorageColumn,
                                       pickupRegion, x, y, area,
                                       psdSnapshot)) {
        foundAny = true;
      }
    }
    if (!interruptibleDelay(missionColumnScanSampleMs)) return false;
  }

  for (uint8_t i = 0; i < missionSurveyDetectionCount; i++) {
    if (missionSurveyDetections[i].column == missionStorageColumn) {
      printMissionSurveyDetectionJson(missionSurveyDetections[i]);
    }
  }
  return foundAny;
}

bool commandMissionSurvey() {
  if (!ensureMobileReady()) return false;
  uint8_t signatureMap = pendingMissionSignatureMap();
  if (signatureMap == 0) signatureMap = CFG.cameraScan.storageAllowedSignatureMap & 0x7F;
  if (signatureMap == 0) {
    DEBUG_SERIAL.println(F("[mission survey] signatureMap이 비어 있습니다."));
    return false;
  }
  if (missionColumnStepMm <= 0.0 || missionColumnStepMm > profile().maxDriveMm ||
      missionColumnMoveMmPerSec <= 0 || missionColumnMoveMmPerSec > profile().maxDriveMmPerSec) {
    DEBUG_SERIAL.println(F("[mission columnstep] 현재 profile 한도를 벗어났습니다."));
    DEBUG_SERIAL.println(F("  mission columnstep <mm> <mm/s>로 다시 지정하거나 profile을 올리세요."));
    return false;
  }
  if (missionStorageColumn < 1 || missionStorageColumn > 4) {
    missionStorageColumn = 1;
  }

  missionSurveyDetectionCount = 0;
  missionSurveyHasResults = false;
  for (uint8_t i = 0; i < MissionConfig::MAX_MISSION_BLOCKS; i++) {
    missionSurveyDetections[i].assigned = false;
  }

  DEBUG_SERIAL.print(F("[mission survey] startColumn="));
  DEBUG_SERIAL.print(missionStorageColumn);
  DEBUG_SERIAL.print(F(", stepMm="));
  DEBUG_SERIAL.print(missionColumnStepMm, 2);
  DEBUG_SERIAL.print(F(", speedMmPerSec="));
  DEBUG_SERIAL.println(missionColumnMoveMmPerSec);

  for (uint8_t column = missionStorageColumn; column <= 4; column++) {
    if (!commandMissionSurveyMoveToColumn(column)) return false;
    commandMissionSurveyScanCurrentColumn(signatureMap);
    if (missionSurveyCoversPendingTargets()) {
      DEBUG_SERIAL.println(F("[mission survey] 미션 queue의 target signature를 모두 찾았습니다. 남은 열 스캔을 생략합니다."));
      break;
    }
  }

  missionSurveyEndColumn = missionStorageColumn;
  missionSurveyHasResults = missionSurveyDetectionCount > 0;
  DEBUG_SERIAL.print(F("{\"type\":\"mission-survey-summary\",\"count\":"));
  DEBUG_SERIAL.print(missionSurveyDetectionCount);
  DEBUG_SERIAL.print(F(",\"endColumn\":"));
  DEBUG_SERIAL.print(missionSurveyEndColumn);
  DEBUG_SERIAL.println(F("}"));
  if (!missionSurveyHasResults) {
    DEBUG_SERIAL.println(F("[mission survey] 감지된 target 블록이 없습니다."));
    turnOnLEDRed500ms();
    return false;
  }
  turnOnLEDGreen500ms();
  return true;
}

bool commandMissionColumnPsdAverage(const String &input) {
  long column = missionStorageColumn;
  long samples = 40;
  long intervalMs = 50;

  if (column < 1 || column > 4) column = 1;
  if (tokenCount(input) >= 3 &&
      !parseLongStrict(tokenAt(input, 2), &column)) {
    DEBUG_SERIAL.println(F("사용법: mission columnpsd [column] [samples] [intervalMs]"));
    DEBUG_SERIAL.println(F("예시: mission columnpsd 1 40 50"));
    return false;
  }
  if (tokenCount(input) >= 4 &&
      !parseLongStrict(tokenAt(input, 3), &samples)) {
    DEBUG_SERIAL.println(F("사용법: mission columnpsd [column] [samples] [intervalMs]"));
    return false;
  }
  if (tokenCount(input) >= 5 &&
      !parseLongStrict(tokenAt(input, 4), &intervalMs)) {
    DEBUG_SERIAL.println(F("사용법: mission columnpsd [column] [samples] [intervalMs]"));
    return false;
  }
  if (column < 1 || column > 4) {
    DEBUG_SERIAL.println(F("[제한] column은 1~4 범위입니다."));
    return false;
  }
  if (samples < 1 || samples > 300) {
    DEBUG_SERIAL.println(F("[제한] samples는 1~300 범위입니다."));
    return false;
  }
  if (intervalMs < 5 || intervalMs > 1000) {
    DEBUG_SERIAL.println(F("[제한] intervalMs는 5~1000ms 범위입니다."));
    return false;
  }

  int32_t sumFl = 0;
  int32_t sumFr = 0;
  int32_t sumSl = 0;
  int32_t sumSr = 0;
  int16_t minFl = 32767;
  int16_t minFr = 32767;
  int16_t minSl = 32767;
  int16_t minSr = 32767;
  int16_t maxFl = -1;
  int16_t maxFr = -1;
  int16_t maxSl = -1;
  int16_t maxSr = -1;

  DEBUG_SERIAL.print(F("[mission columnpsd] column="));
  DEBUG_SERIAL.print(column);
  DEBUG_SERIAL.print(F(", samples="));
  DEBUG_SERIAL.print(samples);
  DEBUG_SERIAL.print(F(", intervalMs="));
  DEBUG_SERIAL.println(intervalMs);

  for (long i = 0; i < samples; i++) {
    PsdSnapshot snapshot;
    readAllPSDSensors(&snapshot);
    sumFl += snapshot.fl;
    sumFr += snapshot.fr;
    sumSl += snapshot.sl;
    sumSr += snapshot.sr;
    if (snapshot.fl < minFl) minFl = snapshot.fl;
    if (snapshot.fr < minFr) minFr = snapshot.fr;
    if (snapshot.sl < minSl) minSl = snapshot.sl;
    if (snapshot.sr < minSr) minSr = snapshot.sr;
    if (snapshot.fl > maxFl) maxFl = snapshot.fl;
    if (snapshot.fr > maxFr) maxFr = snapshot.fr;
    if (snapshot.sl > maxSl) maxSl = snapshot.sl;
    if (snapshot.sr > maxSr) maxSr = snapshot.sr;
    if (i + 1 < samples && !interruptibleDelay((unsigned long)intervalMs)) {
      return false;
    }
  }

  DEBUG_SERIAL.print(F("  avg FL/FR/SL/SR="));
  DEBUG_SERIAL.print((float)sumFl / samples, 2);
  DEBUG_SERIAL.print(F("/"));
  DEBUG_SERIAL.print((float)sumFr / samples, 2);
  DEBUG_SERIAL.print(F("/"));
  DEBUG_SERIAL.print((float)sumSl / samples, 2);
  DEBUG_SERIAL.print(F("/"));
  DEBUG_SERIAL.println((float)sumSr / samples, 2);
  DEBUG_SERIAL.print(F("{\"type\":\"mission-column-psd\",\"column\":"));
  DEBUG_SERIAL.print(column);
  DEBUG_SERIAL.print(F(",\"currentColumn\":"));
  DEBUG_SERIAL.print(missionStorageColumn);
  DEBUG_SERIAL.print(F(",\"samples\":"));
  DEBUG_SERIAL.print(samples);
  DEBUG_SERIAL.print(F(",\"intervalMs\":"));
  DEBUG_SERIAL.print(intervalMs);
  DEBUG_SERIAL.print(F(",\"avg\":{\"fl\":"));
  DEBUG_SERIAL.print((float)sumFl / samples, 2);
  DEBUG_SERIAL.print(F(",\"fr\":"));
  DEBUG_SERIAL.print((float)sumFr / samples, 2);
  DEBUG_SERIAL.print(F(",\"sl\":"));
  DEBUG_SERIAL.print((float)sumSl / samples, 2);
  DEBUG_SERIAL.print(F(",\"sr\":"));
  DEBUG_SERIAL.print((float)sumSr / samples, 2);
  DEBUG_SERIAL.print(F("},\"min\":{\"fl\":"));
  DEBUG_SERIAL.print(minFl);
  DEBUG_SERIAL.print(F(",\"fr\":"));
  DEBUG_SERIAL.print(minFr);
  DEBUG_SERIAL.print(F(",\"sl\":"));
  DEBUG_SERIAL.print(minSl);
  DEBUG_SERIAL.print(F(",\"sr\":"));
  DEBUG_SERIAL.print(minSr);
  DEBUG_SERIAL.print(F("},\"max\":{\"fl\":"));
  DEBUG_SERIAL.print(maxFl);
  DEBUG_SERIAL.print(F(",\"fr\":"));
  DEBUG_SERIAL.print(maxFr);
  DEBUG_SERIAL.print(F(",\"sl\":"));
  DEBUG_SERIAL.print(maxSl);
  DEBUG_SERIAL.print(F(",\"sr\":"));
  DEBUG_SERIAL.print(maxSr);
  DEBUG_SERIAL.println(F("}}"));
  return true;
}

void printMissionColumnScanTimingStatus() {
  DEBUG_SERIAL.print(F("[mission columnscanrate] settleMs="));
  DEBUG_SERIAL.print(missionColumnScanSettleMs);
  DEBUG_SERIAL.print(F(", frames="));
  DEBUG_SERIAL.print(missionColumnScanFrames);
  DEBUG_SERIAL.print(F(", sampleMs="));
  DEBUG_SERIAL.println(missionColumnScanSampleMs);
  DEBUG_SERIAL.print(F("{\"type\":\"mission-columnscanrate\",\"settleMs\":"));
  DEBUG_SERIAL.print(missionColumnScanSettleMs);
  DEBUG_SERIAL.print(F(",\"frames\":"));
  DEBUG_SERIAL.print(missionColumnScanFrames);
  DEBUG_SERIAL.print(F(",\"sampleMs\":"));
  DEBUG_SERIAL.print(missionColumnScanSampleMs);
  DEBUG_SERIAL.println(F("}"));
}

bool commandMissionColumnScanTiming(const String &input) {
  if (tokenCount(input) < 5) {
    printMissionColumnScanTimingStatus();
    DEBUG_SERIAL.println(F("사용법: mission columnscanrate <settleMs> <frames> <sampleMs>"));
    DEBUG_SERIAL.println(F("예시: mission columnscanrate 220 10 25"));
    return true;
  }

  long settleMs = 0;
  long frames = 0;
  long sampleMs = 0;
  if (!parseLongStrict(tokenAt(input, 2), &settleMs) ||
      !parseLongStrict(tokenAt(input, 3), &frames) ||
      !parseLongStrict(tokenAt(input, 4), &sampleMs)) {
    DEBUG_SERIAL.println(F("사용법: mission columnscanrate <settleMs> <frames> <sampleMs>"));
    return false;
  }
  if (settleMs < 0 || settleMs > 2000 ||
      frames < 1 || frames > 20 ||
      sampleMs < 5 || sampleMs > 200) {
    DEBUG_SERIAL.println(F("[제한] settleMs=0~2000, frames=1~20, sampleMs=5~200 범위입니다."));
    return false;
  }

  missionColumnScanSettleMs = (uint16_t)settleMs;
  missionColumnScanFrames = (uint8_t)frames;
  missionColumnScanSampleMs = (uint16_t)sampleMs;
  printMissionColumnScanTimingStatus();
  return true;
}

uint8_t firstUnplannedMissionQueueIndexForSignature(bool targetAssigned[], uint8_t signature) {
  for (uint8_t i = 0; i < missionQueueCount; i++) {
    if (missionQueueCompleted[i] || targetAssigned[i]) continue;
    if (missionQueueSignatures[i] == signature) return i + 1;
  }
  return 0;
}

bool commandMissionPlan() {
  if (missionQueueCount == 0) {
    DEBUG_SERIAL.println(F("[mission plan] 미션지시존 queue가 없습니다. 먼저 mission rescan 또는 mission start/next로 스캔하세요."));
    return false;
  }
  if (!missionSurveyHasResults || missionSurveyDetectionCount == 0) {
    DEBUG_SERIAL.println(F("[mission plan] survey 결과가 없습니다. 먼저 mission survey를 실행하세요."));
    return false;
  }

  bool targetAssigned[MissionConfig::MAX_MISSION_BLOCKS] = {false};
  uint8_t plannedSignatures[MissionConfig::MAX_MISSION_BLOCKS] = {0};
  uint8_t plannedSourceSlots[MissionConfig::MAX_MISSION_BLOCKS] = {0};
  uint8_t plannedGoalSlots[MissionConfig::MAX_MISSION_BLOCKS] = {0};
  uint8_t plannedCount = 0;

  for (uint8_t i = 0; i < missionSurveyDetectionCount; i++) {
    missionSurveyDetections[i].assigned = false;
  }

  while (plannedCount < missionQueueCount) {
    uint8_t bestDetection = MissionConfig::MAX_MISSION_BLOCKS;
    uint8_t bestQueueIndex = 0;
    uint8_t bestColumn = 0;
    uint32_t bestArea = 0;

    for (uint8_t i = 0; i < missionSurveyDetectionCount; i++) {
      MissionStorageSurveyDetection &detection = missionSurveyDetections[i];
      if (detection.assigned) continue;
      uint8_t queueIndex =
        firstUnplannedMissionQueueIndexForSignature(targetAssigned,
                                                    detection.signature);
      if (queueIndex == 0) continue;
      if (bestDetection >= MissionConfig::MAX_MISSION_BLOCKS ||
          detection.column > bestColumn ||
          (detection.column == bestColumn && detection.area > bestArea)) {
        bestDetection = i;
        bestQueueIndex = queueIndex;
        bestColumn = detection.column;
        bestArea = detection.area;
      }
    }

    if (bestDetection >= MissionConfig::MAX_MISSION_BLOCKS) break;

    MissionStorageSurveyDetection &detection = missionSurveyDetections[bestDetection];
    detection.assigned = true;
    targetAssigned[bestQueueIndex - 1] = true;

    plannedSignatures[plannedCount] = detection.signature;
    plannedSourceSlots[plannedCount] = detection.sourceSlot;
    plannedGoalSlots[plannedCount] = detection.sourceSlot;
    plannedCount++;

    DEBUG_SERIAL.print(F("{\"type\":\"mission-plan-task\",\"index\":"));
    DEBUG_SERIAL.print(plannedCount);
    DEBUG_SERIAL.print(F(",\"queueIndex\":"));
    DEBUG_SERIAL.print(bestQueueIndex);
    DEBUG_SERIAL.print(F(",\"sig\":"));
    DEBUG_SERIAL.print(detection.signature);
    DEBUG_SERIAL.print(F(",\"sourceSlot\":"));
    DEBUG_SERIAL.print(detection.sourceSlot);
    DEBUG_SERIAL.print(F(",\"goalSlot\":"));
    DEBUG_SERIAL.print(detection.sourceSlot);
    DEBUG_SERIAL.print(F(",\"poseId\":"));
    DEBUG_SERIAL.print(CFG.pose.missionZoneStartId + detection.sourceSlot);
    DEBUG_SERIAL.print(F(",\"column\":"));
    DEBUG_SERIAL.print(detection.column);
    DEBUG_SERIAL.print(F(",\"layer\":\""));
    DEBUG_SERIAL.print(storagePickupRegionUsesUpperGrip(detection.pickupRegion) ? F("upper") : F("lower"));
    DEBUG_SERIAL.print(F("\",\"x\":"));
    DEBUG_SERIAL.print(detection.x);
    DEBUG_SERIAL.print(F(",\"y\":"));
    DEBUG_SERIAL.print(detection.y);
    DEBUG_SERIAL.print(F(",\"area\":"));
    DEBUG_SERIAL.print(detection.area);
    DEBUG_SERIAL.print(F(",\"psd\":{\"fl\":"));
    DEBUG_SERIAL.print(detection.psdFl);
    DEBUG_SERIAL.print(F(",\"fr\":"));
    DEBUG_SERIAL.print(detection.psdFr);
    DEBUG_SERIAL.print(F(",\"sl\":"));
    DEBUG_SERIAL.print(detection.psdSl);
    DEBUG_SERIAL.print(F(",\"sr\":"));
    DEBUG_SERIAL.print(detection.psdSr);
    DEBUG_SERIAL.print(F("}"));
    DEBUG_SERIAL.println(F("}"));
  }

  if (plannedCount == 0) {
    DEBUG_SERIAL.println(F("[mission plan] queue signature와 survey detection이 매칭되지 않았습니다."));
    turnOnLEDRed500ms();
    return false;
  }

  if (plannedCount < missionQueueCount) {
    DEBUG_SERIAL.print(F("[mission plan] 매칭되지 않은 queue 개수="));
    DEBUG_SERIAL.println(missionQueueCount - plannedCount);
  }

  missionQueueCount = plannedCount;
  for (uint8_t i = 0; i < missionQueueCount; i++) {
    missionQueueSignatures[i] = plannedSignatures[i];
    missionQueueSourceSlots[i] = plannedSourceSlots[i];
    missionQueueGoalSlots[i] = plannedGoalSlots[i];
    missionQueueCompleted[i] = false;
  }
  for (uint8_t i = missionQueueCount; i < MissionConfig::MAX_MISSION_BLOCKS; i++) {
    missionQueueSignatures[i] = 0;
    missionQueueSourceSlots[i] = 0;
    missionQueueGoalSlots[i] = 0;
    missionQueueCompleted[i] = false;
  }

  missionBlockIndex = 1;
  missionStorageScanTargetColumn = 1;
  missionColumnSearchMissColumn = 0;
  clearMissionColumnScanDecision();
  setMissionBranchDefaultsForCurrentBlock();
  DEBUG_SERIAL.println(F("[mission plan] 높은 열부터 sourceSlot=goalSlot으로 쓰는 실행 queue를 적용했습니다."));
  printMissionQueueJsonLine();
  turnOnLEDGreen500ms();
  return true;
}

bool commandMissionMoveOneStorageColumnTowardCurrentSlot() {
  if (!ensureMobileReady()) return false;

  uint8_t targetColumn = missionTargetStorageColumn();
  if (targetColumn == 0) {
    DEBUG_SERIAL.println(F("[mission] 현재 source slot의 적재함 열을 결정하지 못했습니다."));
    DEBUG_SERIAL.println(F("  MissionConfig.h의 upperRowSlots/lowerRowSlots/pickSlotOrder를 확인하세요."));
    return false;
  }

  DEBUG_SERIAL.print(F("[mission scan] target sourceSlot="));
  DEBUG_SERIAL.print(missionSourceSlotForCurrentBlock());
  DEBUG_SERIAL.print(F(", targetColumn="));
  DEBUG_SERIAL.print(targetColumn);
  DEBUG_SERIAL.print(F(", currentColumn="));
  DEBUG_SERIAL.println(missionStorageColumn);

  if (missionStorageColumn < 1 || missionStorageColumn > 4) {
    missionStorageColumn = 1;
    DEBUG_SERIAL.println(F("  currentColumn이 비정상이라 1번 열 기준으로 보정합니다."));
  }
  if (missionStorageColumn == targetColumn) {
    DEBUG_SERIAL.println(F("  이미 목표 열입니다. 이동하지 않고 다음 단계에서 스캔합니다."));
    stopMobilebase();
    return true;
  }
  if (missionColumnStepMm <= 0.0 || missionColumnStepMm > profile().maxDriveMm ||
      missionColumnMoveMmPerSec <= 0 || missionColumnMoveMmPerSec > profile().maxDriveMmPerSec) {
    DEBUG_SERIAL.println(F("[mission columnstep] 현재 profile 한도를 벗어났습니다."));
    DEBUG_SERIAL.println(F("  mission columnstep <mm> <mm/s>로 다시 지정하거나 profile을 올리세요."));
    return false;
  }

  uint8_t direction = (targetColumn > missionStorageColumn) ? DRIVE_DIRECTION_RIGHT : DRIVE_DIRECTION_LEFT;
  const __FlashStringHelper *label = (direction == DRIVE_DIRECTION_RIGHT)
                                      ? F("  다음 적재함 열로 우측 이동")
                                      : F("  이전 적재함 열로 좌측 이동");
  if (!commandMissionDriveDistanceStep(label,
                                       missionColumnStepMm,
                                       direction,
                                       missionColumnMoveMmPerSec)) {
    return false;
  }

  if (targetColumn > missionStorageColumn) missionStorageColumn++;
  else missionStorageColumn--;

  DEBUG_SERIAL.print(F("  이동 후 currentColumn="));
  DEBUG_SERIAL.println(missionStorageColumn);
  DEBUG_SERIAL.println(F("  여기서 멈춥니다. 다음 SW1에서 목표 열이면 Pixy 스캔을 실행합니다."));
  return true;
}

bool commandMissionMoveOneStorageColumnForSearch() {
  if (!ensureMobileReady()) return false;

  if (missionStorageColumn < 1 || missionStorageColumn > 4) {
    missionStorageColumn = 1;
    DEBUG_SERIAL.println(F("[mission scan] currentColumn이 비정상이라 1번 열 기준으로 보정합니다."));
  }
  if (missionStorageColumn >= 4 || missionStorageScanTargetColumn >= 4) {
    DEBUG_SERIAL.println(F("[mission scan] 1~4열을 모두 확인했지만 처리할 signature를 집기 위치에서 찾지 못했습니다."));
    DEBUG_SERIAL.println(F("  mission rescan, pixy storage all 10 0, 또는 mission column <1~4>로 기준을 다시 확인하세요."));
    return false;
  }
  if (missionColumnStepMm <= 0.0 || missionColumnStepMm > profile().maxDriveMm ||
      missionColumnMoveMmPerSec <= 0 || missionColumnMoveMmPerSec > profile().maxDriveMmPerSec) {
    DEBUG_SERIAL.println(F("[mission columnstep] 현재 profile 한도를 벗어났습니다."));
    DEBUG_SERIAL.println(F("  mission columnstep <mm> <mm/s>로 다시 지정하거나 profile을 올리세요."));
    return false;
  }

  DEBUG_SERIAL.println(F("[mission scan] 현재 열에서 처리할 signature를 못 찾아 다음 열로 이동합니다."));
  missionStorageScanTargetColumn = missionStorageColumn + 1;
  if (!commandMissionDriveDistanceStep(F("  다음 적재함 열로 우측 이동"),
                                       missionColumnStepMm,
                                       DRIVE_DIRECTION_RIGHT,
                                       missionColumnMoveMmPerSec)) {
    return false;
  }
  missionStorageColumn++;
  missionColumnSearchMissColumn = 0;
  clearMissionColumnScanDecision();
  DEBUG_SERIAL.print(F("  이동 후 currentColumn="));
  DEBUG_SERIAL.println(missionStorageColumn);
  DEBUG_SERIAL.println(F("  여기서 멈춥니다. 다음 SW1에서 현재 열 Pixy 스캔을 실행합니다."));
  return true;
}

bool commandMissionScanCurrentStorageColumn() {
  uint8_t sourceSlot = missionSourceSlotForCurrentBlock();
  uint8_t targetColumn = missionTargetStorageColumn();
  uint8_t goalSlot = missionPlaceSlotForCurrentBlock();
  uint8_t targetSignature = missionCurrentTargetSignature();
  bool dynamicColumnScan = sourceSlot == 0;
  uint8_t requiredColumn = (missionStorageColumn >= 1 && missionStorageColumn <= 4)
                             ? missionStorageColumn
                             : targetColumn;
  uint8_t signatureMap = dynamicColumnScan
                           ? pendingMissionSignatureMap()
                           : missionCurrentTargetSignatureMap();
  String expectedLayer = "";
  bool hasExpectedLayer = missionPickLayerForSourceSlot(sourceSlot, &expectedLayer);
  bool expectedUpper = expectedLayer == "upper";

  DEBUG_SERIAL.print(F("[mission scan] 적재함 열 스캔: sourceSlot="));
  DEBUG_SERIAL.print(sourceSlot);
  DEBUG_SERIAL.print(F(", column="));
  DEBUG_SERIAL.println(targetColumn);
  DEBUG_SERIAL.println(F("  현재 block의 signature/sourceSlot/goalSlot/pickLayer 판정 상태를 저장합니다."));
  DEBUG_SERIAL.print(F("{\"type\":\"mission-columnscan\",\"blockIndex\":"));
  DEBUG_SERIAL.print(missionBlockIndex);
  DEBUG_SERIAL.print(F(",\"sourceSlot\":"));
  DEBUG_SERIAL.print(sourceSlot);
  DEBUG_SERIAL.print(F(",\"goalSlot\":"));
  DEBUG_SERIAL.print(goalSlot);
  DEBUG_SERIAL.print(F(",\"targetSignature\":"));
  if (dynamicColumnScan) DEBUG_SERIAL.print(0);
  else DEBUG_SERIAL.print(targetSignature);
  DEBUG_SERIAL.print(F(",\"column\":"));
  DEBUG_SERIAL.print(targetColumn);
  DEBUG_SERIAL.print(F(",\"stepMm\":"));
  DEBUG_SERIAL.print(missionColumnStepMm, 2);
  DEBUG_SERIAL.print(F(",\"speedMmPerSec\":"));
  DEBUG_SERIAL.print(missionColumnMoveMmPerSec);
  DEBUG_SERIAL.println(F("}"));

  clearMissionColumnScanDecision();
  if (targetColumn == 0 || (!dynamicColumnScan && goalSlot == 0) || signatureMap == 0) {
    DEBUG_SERIAL.println(F("[mission scan] 현재 block의 goal/signature 기준을 만들 수 없습니다."));
    DEBUG_SERIAL.println(F("  먼저 mission rescan 또는 mission auto를 실행하세요."));
    turnOnLEDRed500ms();
    return false;
  }

  uint8_t frames = constrain(missionColumnScanFrames, 1, 20);
  uint16_t minArea = CFG.storageRack.scanMinBlockArea;
  bool found = false;
  uint8_t bestSig = 0;
  uint8_t bestRegion = 0;
  uint8_t bestPixyColumn = 0;
  uint8_t bestSourceSlot = sourceSlot;
  uint8_t bestQueueIndex = dynamicColumnScan ? 0 : missionBlockIndex;
  uint8_t bestGoalSlot = goalSlot;
  int16_t bestX = 0;
  int16_t bestY = 0;
  uint32_t bestArea = 0;
  bool sawMatchingSignature = false;
  bool sawMatchingSignatureInPickupWindow = false;
  bool sawPixyColumnMismatch = false;

  DEBUG_SERIAL.print(F("  signatureMap="));
  printSignatureMap(signatureMap);
  DEBUG_SERIAL.print(F(", expectedLayer="));
  if (hasExpectedLayer) DEBUG_SERIAL.print(expectedLayer);
  else DEBUG_SERIAL.print(F("(unknown)"));
  DEBUG_SERIAL.print(F(", frames="));
  DEBUG_SERIAL.print(frames);
  DEBUG_SERIAL.print(F(", settleMs="));
  DEBUG_SERIAL.print(missionColumnScanSettleMs);
  DEBUG_SERIAL.print(F(", sampleMs="));
  DEBUG_SERIAL.print(missionColumnScanSampleMs);
  DEBUG_SERIAL.print(F(", minArea="));
  DEBUG_SERIAL.println(minArea);
  DEBUG_SERIAL.print(F("  requiredColumn="));
  DEBUG_SERIAL.print(requiredColumn);
  DEBUG_SERIAL.println(F(" (현재 물리 column/sourceSlot 우선, Pixy x열은 경고/로그용)"));
  if (missionColumnScanSettleMs > 0 &&
      !interruptibleDelay(missionColumnScanSettleMs)) return false;

  for (uint8_t frame = 0; frame < frames; frame++) {
    pixy.ccc.getBlocks(true, signatureMap);
    DEBUG_SERIAL.print(F("  frame "));
    DEBUG_SERIAL.print(frame + 1);
    DEBUG_SERIAL.print(F(": count="));
    DEBUG_SERIAL.println(pixy.ccc.numBlocks);

    for (uint8_t i = 0; i < pixy.ccc.numBlocks; i++) {
      uint8_t sig = pixy.ccc.blocks[i].m_signature;
      if (sig < 1 || sig > CFG.cameraScan.maxSignature) continue;
      if ((((uint8_t)1 << (sig - 1)) & signatureMap) == 0) continue;

      uint32_t area = pixyBlockArea(i);
      if (area < minArea) continue;
      sawMatchingSignature = true;

      int16_t x = pixy.ccc.blocks[i].m_x;
      int16_t y = pixy.ccc.blocks[i].m_y;
      uint8_t region = storagePickupRegionForPixyXY(x, y);
      uint8_t detectedPixyColumn = storageColumnForPixyX(x);
      bool inRackBoundary = storagePixyInRackBoundary(x, y);
      bool inPickupWindow = region != 0;
      bool detectedUpper = storagePickupRegionUsesUpperGrip(region);
      bool layerMatches = !hasExpectedLayer || (inPickupWindow && detectedUpper == expectedUpper);
      uint8_t detectedSourceSlot = sourceSlot > 0
                                     ? sourceSlot
                                     : sourceSlotForStorageColumnAndLayer(requiredColumn, detectedUpper);
      uint8_t detectedQueueIndex = dynamicColumnScan
                                     ? pendingMissionQueueIndexForSignature(sig)
                                     : missionBlockIndex;
      uint8_t detectedGoalSlot = goalSlot;
      if (dynamicColumnScan) {
        detectedGoalSlot = detectedSourceSlot;
      }

      DEBUG_SERIAL.print(F("    seen sig="));
      DEBUG_SERIAL.print(sig);
      DEBUG_SERIAL.print(F(", x="));
      DEBUG_SERIAL.print(x);
      DEBUG_SERIAL.print(F(", y="));
      DEBUG_SERIAL.print(y);
      DEBUG_SERIAL.print(F(", pixyColumn="));
      if (detectedPixyColumn == 0) DEBUG_SERIAL.print(F("?"));
      else DEBUG_SERIAL.print(detectedPixyColumn);
      DEBUG_SERIAL.print(F(", yLayer="));
      DEBUG_SERIAL.print(inPickupWindow ? (detectedUpper ? F("upper") : F("lower")) : F("?"));
      DEBUG_SERIAL.print(F(", rack="));
      DEBUG_SERIAL.print(inRackBoundary ? F("inside") : F("outside"));
      DEBUG_SERIAL.print(F(", pickup="));
      DEBUG_SERIAL.print(storagePickupRegionName(region));
      DEBUG_SERIAL.print(F(", area="));
      DEBUG_SERIAL.print(area);
      bool pixyColumnMismatch = requiredColumn >= 1 && requiredColumn <= 4 &&
                                detectedPixyColumn > 0 &&
                                detectedPixyColumn != requiredColumn;
      if (pixyColumnMismatch) {
        sawPixyColumnMismatch = true;
        DEBUG_SERIAL.print(F(", columnWarn=pixy-current-mismatch"));
      }
      if (!inRackBoundary) {
        DEBUG_SERIAL.println(F(" -> reject: outside rack boundary"));
        continue;
      }
      if (!inPickupWindow) {
        DEBUG_SERIAL.println(F(" -> reject: outside upper/lower pickup boundary"));
        continue;
      }
      sawMatchingSignatureInPickupWindow = true;
      if (!layerMatches) {
        DEBUG_SERIAL.println(F(" -> reject: layer mismatch"));
        continue;
      }
      if (detectedSourceSlot == 0) {
        DEBUG_SERIAL.println(F(" -> reject: source slot unresolved"));
        continue;
      }
      if (detectedGoalSlot < 1 || detectedGoalSlot > CFG.pose.missionZoneSlotCount) {
        DEBUG_SERIAL.println(F(" -> reject: goal slot unresolved"));
        continue;
      }
      DEBUG_SERIAL.println(F(" -> accept"));

      DEBUG_SERIAL.print(F("    candidate sig="));
      DEBUG_SERIAL.print(sig);
      DEBUG_SERIAL.print(F(", x="));
      DEBUG_SERIAL.print(x);
      DEBUG_SERIAL.print(F(", y="));
      DEBUG_SERIAL.print(y);
      DEBUG_SERIAL.print(F(", pickup="));
      DEBUG_SERIAL.print(storagePickupRegionName(region));
      DEBUG_SERIAL.print(F(", area="));
      DEBUG_SERIAL.println(area);

      if (!found || area > bestArea) {
        found = true;
        bestSig = sig;
        bestRegion = region;
        bestPixyColumn = detectedPixyColumn;
        bestSourceSlot = detectedSourceSlot;
        bestQueueIndex = detectedQueueIndex;
        bestGoalSlot = detectedGoalSlot;
        bestX = x;
        bestY = y;
        bestArea = area;
      }
    }

    if (!interruptibleDelay(missionColumnScanSampleMs)) return false;
  }

  if (!found) {
    DEBUG_SERIAL.println(F("[mission scan] 현재 source slot 조건에 맞는 블록을 찾지 못했습니다."));
    missionColumnSearchMissColumn = missionStorageColumn;
    if (sawPixyColumnMismatch) {
      DEBUG_SERIAL.println(F("  Pixy x열과 현재 물리 column이 달랐지만, 이제 이 조건만으로는 탈락시키지 않습니다."));
      DEBUG_SERIAL.println(F("  그래도 실패했다면 upper/lower pickup boundary 또는 expectedLayer 조건을 확인하세요."));
    } else if (sawMatchingSignature && !sawMatchingSignatureInPickupWindow) {
      DEBUG_SERIAL.println(F("  signature는 보였지만 적재함 전체/upper/lower 집기 boundary 밖입니다."));
      DEBUG_SERIAL.println(F("  목표 블록이 다른 열에 있거나 Pixy boundary/mission column 기준이 틀린 상태입니다."));
    } else if (sawMatchingSignature) {
      DEBUG_SERIAL.println(F("  signature는 보였지만 expectedLayer 또는 upper/lower boundary 조건에서 탈락했습니다."));
      DEBUG_SERIAL.println(F("  MissionConfig.h의 storagePickupRegion / storageGripTarget 값을 확인하세요."));
    } else {
      DEBUG_SERIAL.println(F("  처리할 signature가 현재 Pixy frame에 없습니다."));
    }
    DEBUG_SERIAL.print(F("{\"type\":\"mission-columnscan-decision\",\"found\":false,\"blockIndex\":"));
    DEBUG_SERIAL.print(missionBlockIndex);
    DEBUG_SERIAL.print(F(",\"sourceSlot\":"));
    DEBUG_SERIAL.print(sourceSlot);
    DEBUG_SERIAL.println(F("}"));
    turnOnLEDRed500ms();
    return false;
  }

  missionColumnScanHasDecision = true;
  missionColumnSearchMissColumn = 0;
  if (bestQueueIndex >= 1 && bestQueueIndex <= missionQueueCount) {
    missionBlockIndex = bestQueueIndex;
  }
  missionDetectedSignature = bestSig;
  missionDetectedSourceSlot = bestSourceSlot;
  missionDetectedGoalSlot = bestGoalSlot;
  missionDetectedPickupRegion = bestRegion;
  missionDetectedPixyColumn = bestPixyColumn;
  missionDetectedX = bestX;
  missionDetectedY = bestY;
  missionDetectedArea = bestArea;
  if (hasExpectedLayer) {
    missionPickLayer = expectedLayer;
  } else {
    missionPickLayer = storagePickupRegionUsesUpperGrip(bestRegion) ? "upper" : "lower";
  }
  missionPlaceSlot = bestGoalSlot;
  if (missionQueueCount > 0 &&
      bestQueueIndex >= 1 &&
      bestQueueIndex <= missionQueueCount) {
    missionQueueSourceSlots[bestQueueIndex - 1] = bestSourceSlot;
    missionQueueGoalSlots[bestQueueIndex - 1] = bestGoalSlot;
  }

  DEBUG_SERIAL.print(F("[mission scan] 결정: sig="));
  DEBUG_SERIAL.print(missionDetectedSignature);
  DEBUG_SERIAL.print(F(", sourceSlot="));
  DEBUG_SERIAL.print(missionDetectedSourceSlot);
  DEBUG_SERIAL.print(F(", goalSlot="));
  DEBUG_SERIAL.print(missionDetectedGoalSlot);
  DEBUG_SERIAL.print(F(", pickLayer="));
  DEBUG_SERIAL.print(missionPickLayer);
  DEBUG_SERIAL.print(F(", pixyColumn="));
  DEBUG_SERIAL.print(missionDetectedPixyColumn);
  DEBUG_SERIAL.print(F(", x="));
  DEBUG_SERIAL.print(missionDetectedX);
  DEBUG_SERIAL.print(F(", y="));
  DEBUG_SERIAL.println(missionDetectedY);
  DEBUG_SERIAL.print(F("{\"type\":\"mission-columnscan-decision\",\"found\":true,\"blockIndex\":"));
  DEBUG_SERIAL.print(missionBlockIndex);
  DEBUG_SERIAL.print(F(",\"sig\":"));
  DEBUG_SERIAL.print(missionDetectedSignature);
  DEBUG_SERIAL.print(F(",\"sourceSlot\":"));
  DEBUG_SERIAL.print(missionDetectedSourceSlot);
  DEBUG_SERIAL.print(F(",\"goalSlot\":"));
  DEBUG_SERIAL.print(missionDetectedGoalSlot);
  DEBUG_SERIAL.print(F(",\"pickLayer\":\""));
  DEBUG_SERIAL.print(missionPickLayer);
  DEBUG_SERIAL.print(F("\",\"pickupRegion\":"));
  DEBUG_SERIAL.print(missionDetectedPickupRegion);
  DEBUG_SERIAL.print(F(",\"pixyColumn\":"));
  DEBUG_SERIAL.print(missionDetectedPixyColumn);
  DEBUG_SERIAL.print(F(",\"x\":"));
  DEBUG_SERIAL.print(missionDetectedX);
  DEBUG_SERIAL.print(F(",\"y\":"));
  DEBUG_SERIAL.print(missionDetectedY);
  DEBUG_SERIAL.print(F(",\"area\":"));
  DEBUG_SERIAL.print(missionDetectedArea);
  DEBUG_SERIAL.println(F("}"));
  turnOnLEDGreen500ms();
  return true;
}

void clearMissionBranchSelection() {
  missionPickLayer = "";
  missionPlaceSlot = 0;
  clearMissionColumnScanDecision();
}

void setMissionBranchDefaultsForCurrentBlock() {
  clearMissionBranchSelection();

  String layer;
  if (missionPickLayerForSourceSlot(missionSourceSlotForCurrentBlock(), &layer)) {
    missionPickLayer = layer;
  }

  missionPlaceSlot = missionPlaceSlotForCurrentBlock();
}

uint8_t missionCurrentTargetSignature() {
  if (missionQueueCount == 0) return 0;
  if (missionBlockIndex < 1 || missionBlockIndex > missionQueueCount) return 0;
  return missionQueueSignatures[missionBlockIndex - 1];
}

uint8_t missionCurrentTargetSignatureMap() {
  uint8_t signature = missionCurrentTargetSignature();
  if (signature >= 1 && signature <= CFG.cameraScan.maxSignature) {
    return ((uint8_t)1 << (signature - 1)) & CFG.cameraScan.storageAllowedSignatureMap;
  }
  return CFG.cameraScan.storageAllowedSignatureMap & 0x7F;
}

void printMissionPrompt() {
  printLine();
  DEBUG_SERIAL.print(F("[mission] stage="));
  DEBUG_SERIAL.print(missionStageName(missionStage));
  DEBUG_SERIAL.print(F(", block="));
  DEBUG_SERIAL.print(missionBlockIndex);
  DEBUG_SERIAL.print(F(", sourceSlot="));
  uint8_t sourceSlot = missionSourceSlotForCurrentBlock();
  if (sourceSlot > 0) {
    DEBUG_SERIAL.print(sourceSlot);
  } else {
    DEBUG_SERIAL.print(F("(none)"));
  }
  DEBUG_SERIAL.print(F(", column="));
  uint8_t targetColumn = missionTargetStorageColumn();
  if (targetColumn > 0) {
    DEBUG_SERIAL.print(missionStorageColumn);
    DEBUG_SERIAL.print(F("->"));
    DEBUG_SERIAL.print(targetColumn);
  } else {
    DEBUG_SERIAL.print(F("(none)"));
  }
  DEBUG_SERIAL.print(F(", pick="));
  DEBUG_SERIAL.print(missionPickLayer.length() ? missionPickLayer : F("(none)"));
  DEBUG_SERIAL.print(F(", slot="));
  if (missionPlaceSlot > 0) {
    DEBUG_SERIAL.println(missionPlaceSlot);
  } else {
    DEBUG_SERIAL.println(F("(none)"));
  }
  if (missionColumnScanHasDecision) {
    DEBUG_SERIAL.print(F("  scan decision: sig="));
    DEBUG_SERIAL.print(missionDetectedSignature);
    DEBUG_SERIAL.print(F(", sourceSlot="));
    DEBUG_SERIAL.print(missionDetectedSourceSlot);
    DEBUG_SERIAL.print(F(", goalSlot="));
    DEBUG_SERIAL.print(missionDetectedGoalSlot);
    DEBUG_SERIAL.print(F(", layer="));
    DEBUG_SERIAL.print(missionPickLayer);
    DEBUG_SERIAL.print(F(", pixyColumn="));
    DEBUG_SERIAL.print(missionDetectedPixyColumn);
    DEBUG_SERIAL.print(F(", x/y="));
    DEBUG_SERIAL.print(missionDetectedX);
    DEBUG_SERIAL.print(F("/"));
    DEBUG_SERIAL.println(missionDetectedY);
  }
  if (missionUndoAvailable) {
    DEBUG_SERIAL.print(F("  undo: mission undo 가능 ("));
    DEBUG_SERIAL.print(missionUndoDistanceMm, 1);
    DEBUG_SERIAL.print(F("mm @ "));
    DEBUG_SERIAL.print(missionUndoSpeedMmPerSec);
    DEBUG_SERIAL.println(F("mm/s 반대 이동)"));
  }

  if (missionStage == MISSION_IDLE) {
    DEBUG_SERIAL.println(F("  mission start : SW1 단계 진행 모드 시작"));
    DEBUG_SERIAL.println(F("  mission start quick : 미션지시존 접근/주행 중 스캔/적재함 접근까지 묶음 실행"));
  } else if (missionStage == MISSION_START_TO_INSTRUCTION) {
    DEBUG_SERIAL.println(F("  SW1 실행: status + pose verify + 1번 자세 + 최초 장애물 접근 + 미션지시존 좌측 정렬"));
    DEBUG_SERIAL.println(F("  정렬 후 finalForward로 선을 밟으며 Pixy를 주행 중 스캔하고 정지합니다."));
    DEBUG_SERIAL.println(F("  바로 적재함까지 묶어 가려면: mission start quick 또는 mission run storage"));
  } else if (missionStage == MISSION_INSTRUCTION_SCAN_HOLD) {
    DEBUG_SERIAL.println(F("  SW1/mission next: 미션지시존 Pixy 정지 스캔 후 계속 정지"));
    DEBUG_SERIAL.println(F("  quick 실행에서는 이 확인 단계를 자동 수락하고 적재함 접근으로 넘어갑니다."));
    DEBUG_SERIAL.println(F("  확인 가능: mission rescan, pixy watch all 30, pixy sweep all 4 40 140 20 12 80, pixy lamp off"));
    DEBUG_SERIAL.println(F("  결과가 괜찮으면: mission accept"));
  } else if (missionStage == MISSION_GO_TO_STORAGE) {
    DEBUG_SERIAL.println(F("  SW1 실행: 3번 적재함 보기/안전 자세 + 적재함 기준 위치 접근/정렬 후 정지"));
    DEBUG_SERIAL.println(F("  조정 가능: speed set front|slow|psd|depth|place|position, mission columnstep <mm> <mm/s>"));
  } else if (missionStage == MISSION_COLUMN_MOVE_OR_SCAN) {
    if (sourceSlot == 0) {
      DEBUG_SERIAL.println(F("  SW1 실행: 현재 열에서 아직 처리 안 된 signature를 스캔합니다. 없으면 다음 SW1에서 다음 열로 이동합니다."));
    } else {
      DEBUG_SERIAL.println(F("  SW1 실행: 목표 열까지 한 칸씩 이동합니다. 목표 열이면 현재 열 Pixy 스캔 후 멈춥니다."));
    }
    DEBUG_SERIAL.println(F("  1/5는 1열, 2/6은 2열, 3/7은 3열, 4/8은 4열입니다."));
    DEBUG_SERIAL.println(F("  조정 가능: mission column <1~4>, mission columnstep <mm> <mm/s>, mission columnscanrate, mission columnright/left, mission jog, mission columnscan, mission undo"));
  } else if (missionStage == MISSION_PICK_HOLD) {
    DEBUG_SERIAL.println(F("  SW1 실행: columnscan 판정 기준으로 Pixy 저속 micro-step 정렬 + PSD 깊이 재정렬 + 추가 깊이 집기"));
    DEBUG_SERIAL.println(F("  수동 변경: mission upper|lower, mission slot <1~8>, mission columnscan, pixy alignslow, mission gripdepth"));
  } else if (missionStage == MISSION_PLACE_HOLD) {
    DEBUG_SERIAL.println(F("  SW1 실행: 미션수행존으로 이동해 현재 slot에 배치하고 정지"));
  } else if (missionStage == MISSION_REALIGN_OR_NEXT) {
    DEBUG_SERIAL.println(F("  SW1 실행: 다음 블록이 있으면 적재함 기준 위치로 복귀, 없으면 finish 준비 상태로 전환"));
  } else if (missionStage == MISSION_FINISH) {
    DEBUG_SERIAL.println(F("  SW1 실행: finishalign으로 SL 보정 후 후진 finish"));
    DEBUG_SERIAL.println(F("  보조 명령: mission finish, mission finishalign <sl> [tol] [speed]"));
  } else if (missionStage == MISSION_FINISHED) {
    DEBUG_SERIAL.println(F("  완료 상태입니다. 다시 하려면 mission start"));
  }
  printLine();
}

void resetMissionBranchSelection() {
  clearMissionBranchSelection();
}

void resetMissionStartState() {
  missionStage = MISSION_START_TO_INSTRUCTION;
  missionBlockIndex = 1;
  missionStorageColumn = 1;
  clearMissionQueue();
  resetMissionBranchSelection();
  clearMissionUndoCandidate();
  missionButtonMode = true;
  missionButtonWasPressed = digitalRead(SW1_PIN) == LOW;
  missionButtonLastChangeMs = millis();
}

bool acceptMissionInstructionQueueForStorage(const __FlashStringHelper *message) {
  if (missionQueueCount == 0) {
    DEBUG_SERIAL.println(F("[mission] 아직 사용할 미션지시존 스캔 queue가 없습니다. 먼저 mission rescan 또는 mission next로 스캔하세요."));
    return false;
  }
  missionBlockIndex = 1;
  missionStorageColumn = 1;
  missionStorageScanTargetColumn = 1;
  setMissionBranchDefaultsForCurrentBlock();
  missionStage = MISSION_GO_TO_STORAGE;
  DEBUG_SERIAL.println(message);
  return true;
}

bool commandMissionStart(const String &input) {
  String mode = tokenAt(input, 2);
  mode.toLowerCase();
  bool quickToStorage = (mode == "quick" || mode == "fast" || mode == "storage" ||
                         mode == "go" || mode == "auto" || mode == "빠르게" ||
                         mode == "적재함" || mode == "묶음");

  resetMissionStartState();
  DEBUG_SERIAL.println(F("[mission] 단계별 미션 확인 모드를 시작합니다."));
  if (!quickToStorage) {
    DEBUG_SERIAL.println(F("첫 SW1은 미션지시존 도착까지 실행하고 멈춥니다."));
    DEBUG_SERIAL.println(F("미션지시존에서 멈추지 않고 적재함까지 묶으려면: mission start quick"));
    DEBUG_SERIAL.println(F("Serial mission next는 보조 명령으로만 남겨둡니다."));
    printMissionPrompt();
    return true;
  }

  DEBUG_SERIAL.println(F("[mission] quick: 미션지시존 접근 + 주행 중 스캔 + queue 수락 + 적재함 접근까지 묶어 실행합니다."));
  bool ok = commandMissionNext();
  if (ok && missionQueueCount == 0) {
    DEBUG_SERIAL.println(F("[mission] 주행 중 queue가 없어서 정지 스캔 fallback을 실행합니다."));
    ok = buildMissionQueueFromInstructionScan();
  }
  if (ok) {
    ok = acceptMissionInstructionQueueForStorage(F("[mission] quick: 스캔 queue를 자동 수락했습니다. 적재함 기준 위치로 이동합니다."));
  }
  if (ok) {
    ok = commandMissionNext();
  }
  printMissionPrompt();
  return ok;
}

bool commandMissionNext() {
  bool ok = false;

  if (missionStage == MISSION_IDLE) {
    DEBUG_SERIAL.println(F("[mission] 먼저 mission start를 입력하세요."));
    return false;
  }

  if (missionStage == MISSION_START_TO_INSTRUCTION) {
    commandStatus();
    ok = commandPoseVerify();
    if (ok) ok = commandSeq(String("seq initial"));
    if (ok) ok = commandMissionApproachInstructionZone();
    if (ok) {
      missionStage = MISSION_INSTRUCTION_SCAN_HOLD;
    }
  } else if (missionStage == MISSION_INSTRUCTION_SCAN_HOLD) {
    ok = buildMissionQueueFromInstructionScan();
    if (ok) {
      missionBlockIndex = 1;
      missionStorageColumn = 1;
      missionStorageScanTargetColumn = 1;
      setMissionBranchDefaultsForCurrentBlock();
      DEBUG_SERIAL.println(F("[mission] 스캔 결과를 확인하세요. 이동하려면 mission accept를 입력하세요."));
    }
  } else if (missionStage == MISSION_GO_TO_STORAGE) {
    ok = commandMissionStoragePose();
    if (ok) ok = commandMissionStorageDynamicApproach();
    if (ok) missionStage = MISSION_COLUMN_MOVE_OR_SCAN;
  } else if (missionStage == MISSION_COLUMN_MOVE_OR_SCAN) {
    uint8_t sourceSlot = missionSourceSlotForCurrentBlock();
    if (sourceSlot == 0 && missionColumnSearchMissColumn == missionStorageColumn) {
      ok = commandMissionMoveOneStorageColumnForSearch();
    } else if (missionStorageColumn == missionTargetStorageColumn()) {
      ok = commandMissionScanCurrentStorageColumn();
      if (ok) missionStage = MISSION_PICK_HOLD;
    } else {
      ok = commandMissionMoveOneStorageColumnTowardCurrentSlot();
    }
  } else if (missionStage == MISSION_PICK_HOLD) {
    if (!missionColumnScanHasDecision) {
      DEBUG_SERIAL.println(F("[mission] 먼저 현재 열 스캔 판정이 필요합니다. mission columnscan 또는 SW1을 한 번 실행하세요."));
      printMissionPrompt();
      return false;
    }
    if (missionPickLayer.length() == 0) {
      setMissionBranchDefaultsForCurrentBlock();
    }
    if (missionPickLayer.length() == 0) {
      DEBUG_SERIAL.println(F("[mission] 현재 block의 source slot에서 집기 층을 결정하지 못했습니다."));
      DEBUG_SERIAL.println(F("  MissionConfig.h의 storageRack.pickSlotOrder/upperRowSlots/lowerRowSlots를 확인하세요."));
      DEBUG_SERIAL.println(F("  임시 실행은 mission upper 또는 mission lower로 수동 지정하세요."));
      printMissionPrompt();
      return false;
    }
    String alignInput = String("pixy alignslow ") + missionPickLayer +
                        String(" ") + String(CFG.storageGripTarget.alignTimeoutMs) +
                        String(" ") + String(CFG.storageRack.scanMinBlockArea);
    if (missionDetectedSignature >= 1 && missionDetectedSignature <= CFG.cameraScan.maxSignature) {
      uint8_t alignSignatureMap = (uint8_t)1 << (missionDetectedSignature - 1);
      alignInput += String(" ");
      alignInput += String(alignSignatureMap);
    }
    ok = commandMissionStorageGripAlign();
    if (ok) ok = commandPixyAlignGripTargetSlow(alignInput);
    if (ok) {
      DEBUG_SERIAL.println(F("[mission move] Pixy 중심 정렬 후 그립 깊이 PSD 재확인"));
      ok = commandMissionStorageGripAlign();
    }
    bool gripDepthLower = missionPickLayer == "lower";
    if (ok) ok = commandMissionGripDepthForwardForLayer(gripDepthLower);
    if (ok) ok = commandSeq(String("seq pick ") + missionPickLayer);
    if (ok) ok = commandMissionGripDepthRetreatForLayer(gripDepthLower);
    if (ok) {
      if (missionPlaceSlot < 1 || missionPlaceSlot > CFG.pose.missionZoneSlotCount) {
        missionPlaceSlot = missionPlaceSlotForCurrentBlock();
      }
      if (missionPlaceSlot < 1 || missionPlaceSlot > CFG.pose.missionZoneSlotCount) {
        DEBUG_SERIAL.println(F("[mission] 현재 block의 배치 칸을 결정하지 못했습니다."));
        DEBUG_SERIAL.println(F("  MissionConfig.h의 mission.goalPositions 값을 확인하세요."));
        DEBUG_SERIAL.print(F("  임시 실행은 mission slot <1~"));
        DEBUG_SERIAL.print(CFG.pose.missionZoneSlotCount);
        DEBUG_SERIAL.println(F(">로 수동 지정하세요."));
        printMissionPrompt();
        return false;
      }
      missionStage = MISSION_PLACE_HOLD;
    }
  } else if (missionStage == MISSION_PLACE_HOLD) {
    if (missionPlaceSlot < 1 || missionPlaceSlot > CFG.pose.missionZoneSlotCount) {
      missionPlaceSlot = missionPlaceSlotForCurrentBlock();
    }
    if (missionPlaceSlot < 1 || missionPlaceSlot > CFG.pose.missionZoneSlotCount) {
      DEBUG_SERIAL.println(F("[mission] 현재 block의 배치 칸을 결정하지 못했습니다."));
      DEBUG_SERIAL.println(F("  MissionConfig.h의 mission.goalPositions 값을 확인하세요."));
      DEBUG_SERIAL.print(F("  임시 실행은 mission slot <1~"));
      DEBUG_SERIAL.print(CFG.pose.missionZoneSlotCount);
      DEBUG_SERIAL.println(F(">로 수동 지정하세요."));
      printMissionPrompt();
      return false;
    }
    ok = commandMissionMoveToZoneAndPlace(missionPlaceSlot);
    if (ok) {
      markCurrentMissionBlockCompleted();
      missionStage = MISSION_REALIGN_OR_NEXT;
    }
  } else if (missionStage == MISSION_REALIGN_OR_NEXT) {
    if (missionQueueCount > 0 && !missionHasPendingQueueBlocks()) {
      DEBUG_SERIAL.println(F("[mission] 감지된 블록 처리가 끝났습니다. 다음 SW1에서 finish 후진을 실행합니다."));
      missionStage = MISSION_FINISH;
      ok = true;
    } else if (missionQueueCount > 0 && missionStorageScanTargetColumn >= 4) {
      DEBUG_SERIAL.println(F("[mission] 4열까지 진행했습니다. 남은 signature가 있으면 mission rescan/pixy storage로 확인하세요."));
      missionStage = MISSION_FINISH;
      ok = true;
    } else {
      ok = commandMissionRealignStorage();
      if (ok) {
        advanceMissionStorageScanTargetColumn();
        selectFirstPendingMissionBlockForPrompt();
        setMissionBranchDefaultsForCurrentBlock();
        missionStage = MISSION_COLUMN_MOVE_OR_SCAN;
      }
    }
  } else if (missionStage == MISSION_FINISH) {
    ok = commandMissionFinish();
  } else if (missionStage == MISSION_FINISHED) {
    DEBUG_SERIAL.println(F("[mission] 이미 완료 상태입니다. 다시 하려면 mission start"));
    return false;
  }

  printMissionPrompt();
  return ok;
}

bool parseMissionStage(String text, MissionStepperStage *stage) {
  text.trim();
  text.toLowerCase();
  if (text == "start" || text == "start-to-instruction" || text == "instruction-start" ||
      text == "1" || text == "시작" || text == "접근") {
    *stage = MISSION_START_TO_INSTRUCTION;
  } else if (text == "scan" || text == "instruction" || text == "instruction-scan" ||
             text == "2" || text == "픽시" || text == "스캔") {
    *stage = MISSION_INSTRUCTION_SCAN_HOLD;
  } else if (text == "storage" || text == "go-storage" || text == "go-to-storage" ||
             text == "3" || text == "적재함") {
    *stage = MISSION_GO_TO_STORAGE;
  } else if (text == "column" || text == "column-move" || text == "column-scan" ||
             text == "4" || text == "열") {
    *stage = MISSION_COLUMN_MOVE_OR_SCAN;
  } else if (text == "pick" || text == "5" || text == "집기") {
    *stage = MISSION_PICK_HOLD;
  } else if (text == "place" || text == "6" || text == "배치") {
    *stage = MISSION_PLACE_HOLD;
  } else if (text == "realign" || text == "next" || text == "7" || text == "재정렬") {
    *stage = MISSION_REALIGN_OR_NEXT;
  } else if (text == "finish" || text == "8" || text == "완료") {
    *stage = MISSION_FINISH;
  } else if (text == "finished" || text == "done" || text == "9") {
    *stage = MISSION_FINISHED;
  } else {
    return false;
  }
  return true;
}

bool commandMission(const String &input) {
  String sub = tokenAt(input, 1);
  sub.toLowerCase();

  if (sub.length() == 0 || sub == "status" || sub == "상태") {
    printMissionPrompt();
    return true;
  }
  if (sub == "start" || sub == "시작") {
    return commandMissionStart(input);
  }
  if (sub == "next" || sub == "다음") {
    return commandMissionNext();
  }
  if (sub == "run" || sub == "go" || sub == "묶음실행") {
    String target = tokenAt(input, 2);
    target.toLowerCase();
    if (target.length() == 0 || target == "storage" || target == "quick" ||
        target == "fast" || target == "적재함") {
      return commandMissionStart(String("mission start quick"));
    }
    DEBUG_SERIAL.println(F("사용법: mission run storage"));
    return false;
  }
  if (sub == "rescan" || sub == "scan" || sub == "재스캔" || sub == "스캔") {
    missionStage = MISSION_INSTRUCTION_SCAN_HOLD;
    bool ok = buildMissionQueueFromInstructionScan();
    if (ok) {
      missionBlockIndex = 1;
      missionStorageColumn = 1;
      setMissionBranchDefaultsForCurrentBlock();
      DEBUG_SERIAL.println(F("[mission] 재스캔 완료. 결과가 괜찮으면 mission accept를 입력하세요."));
    }
    printMissionPrompt();
    return ok;
  }
  if (sub == "accept" || sub == "확정" || sub == "수락") {
    bool ok = acceptMissionInstructionQueueForStorage(F("[mission] 스캔 결과를 수락했습니다. 다음 SW1에서 적재함 기준 위치로 이동합니다."));
    printMissionPrompt();
    return ok;
  }
  if (sub == "survey" || sub == "전체스캔" || sub == "적재함스캔") {
    bool ok = commandMissionSurvey();
    printMissionPrompt();
    return ok;
  }
  if (sub == "plan" || sub == "계획" || sub == "큐계획") {
    bool ok = commandMissionPlan();
    printMissionPrompt();
    return ok;
  }
  if (sub == "auto" || sub == "자동") {
    setMissionBranchDefaultsForCurrentBlock();
    DEBUG_SERIAL.println(F("[mission] 현재 block의 기본 source slot/pick/배치 칸을 다시 적용했습니다."));
    printMissionPrompt();
    return true;
  }
  if (sub == "button" || sub == "버튼") {
    String mode = tokenAt(input, 2);
    mode.toLowerCase();
    if (mode == "off" || mode == "0" || mode == "끄기") {
      missionButtonMode = false;
      DEBUG_SERIAL.println(F("[mission button] OFF"));
      return true;
    }
    missionButtonMode = true;
    missionButtonWasPressed = digitalRead(SW1_PIN) == LOW;
    missionButtonLastChangeMs = millis();
    DEBUG_SERIAL.println(F("[mission button] ON: SW1을 누를 때마다 현재 mission step을 1회 실행합니다."));
    DEBUG_SERIAL.println(F("  끄기: mission button off"));
    printMissionPrompt();
    return true;
  }
  if (sub == "upper" || sub == "상층") {
    missionPickLayer = "upper";
    DEBUG_SERIAL.println(F("[mission] 집기 분기: 상층"));
    printMissionPrompt();
    return true;
  }
  if (sub == "lower" || sub == "하층") {
    missionPickLayer = "lower";
    DEBUG_SERIAL.println(F("[mission] 집기 분기: 하층"));
    printMissionPrompt();
    return true;
  }
  if (sub == "slot" || sub == "칸") {
    long slot = 0;
    if (!parseLongStrict(tokenAt(input, 2), &slot) ||
        slot < 1 || slot > CFG.pose.missionZoneSlotCount) {
      DEBUG_SERIAL.print(F("사용법: mission slot <1~"));
      DEBUG_SERIAL.print(CFG.pose.missionZoneSlotCount);
      DEBUG_SERIAL.println(F(">"));
      return false;
    }
    missionPlaceSlot = (uint8_t)slot;
    DEBUG_SERIAL.print(F("[mission] 배치 칸: "));
    DEBUG_SERIAL.println(missionPlaceSlot);
    printMissionPrompt();
    return true;
  }
  if (sub == "column" || sub == "col" || sub == "열") {
    long column = 0;
    if (!parseLongStrict(tokenAt(input, 2), &column) ||
        column < 1 || column > 4) {
      DEBUG_SERIAL.println(F("사용법: mission column <1~4>"));
      return false;
    }
    missionStorageColumn = (uint8_t)column;
    missionStorageScanTargetColumn = (uint8_t)column;
    missionColumnSearchMissColumn = 0;
    DEBUG_SERIAL.print(F("[mission] 현재 적재함 열 기준: "));
    DEBUG_SERIAL.println(missionStorageColumn);
    printMissionPrompt();
    return true;
  }
  if (sub == "columnpsd" || sub == "colpsd" || sub == "psdavg" ||
      sub == "열psd" || sub == "열평균") {
    bool ok = commandMissionColumnPsdAverage(input);
    printMissionPrompt();
    return ok;
  }
  if (sub == "columnscanrate" || sub == "columnscanwait" ||
      sub == "colscanrate" || sub == "열스캔시간" || sub == "열확인시간") {
    bool ok = commandMissionColumnScanTiming(input);
    printMissionPrompt();
    return ok;
  }
  if (sub == "columnstep" || sub == "colstep" || sub == "열이동") {
    float stepMm = 0.0;
    long speedMmPerSec = 0;
    if (!parseFloatStrict(tokenAt(input, 2), &stepMm) ||
        !parseLongStrict(tokenAt(input, 3), &speedMmPerSec)) {
      DEBUG_SERIAL.println(F("사용법: mission columnstep <mm> <mm/s>"));
      return false;
    }
    if (stepMm <= 0.0 || stepMm > profile().maxDriveMm) {
      DEBUG_SERIAL.print(F("[제한] 현재 profile의 열 이동량 한도는 "));
      DEBUG_SERIAL.print(profile().maxDriveMm);
      DEBUG_SERIAL.println(F("mm입니다."));
      return false;
    }
    if (speedMmPerSec <= 0 || speedMmPerSec > profile().maxDriveMmPerSec) {
      DEBUG_SERIAL.print(F("[제한] 현재 profile의 열 이동 속도 한도는 "));
      DEBUG_SERIAL.print(profile().maxDriveMmPerSec);
      DEBUG_SERIAL.println(F("mm/s입니다."));
      return false;
    }
    missionColumnStepMm = stepMm;
    missionColumnMoveMmPerSec = speedMmPerSec;
    DEBUG_SERIAL.print(F("[mission columnstep] stepMm="));
    DEBUG_SERIAL.print(missionColumnStepMm, 2);
    DEBUG_SERIAL.print(F(", speedMmPerSec="));
    DEBUG_SERIAL.println(missionColumnMoveMmPerSec);
    DEBUG_SERIAL.print(F("{\"type\":\"mission-columnstep\",\"stepMm\":"));
    DEBUG_SERIAL.print(missionColumnStepMm, 2);
    DEBUG_SERIAL.print(F(",\"speedMmPerSec\":"));
    DEBUG_SERIAL.print(missionColumnMoveMmPerSec);
    DEBUG_SERIAL.println(F("}"));
    printMissionPrompt();
    return true;
  }
  if (sub == "storagepath" || sub == "storageway" || sub == "storageapproach" ||
      sub == "적재함경로" || sub == "적재함접근") {
    bool ok = commandMissionStoragePath(input);
    printMissionPrompt();
    return ok;
  }
  if (sub == "storagegate" || sub == "storagedetect" || sub == "storagesl" ||
      sub == "적재함감지" || sub == "적재함게이트") {
    bool ok = commandMissionStorageGateTuning(input);
    printMissionPrompt();
    return ok;
  }
  if (sub == "approach" || sub == "frontapproach" || sub == "frontstep" ||
      sub == "장애물접근" || sub == "최초접근") {
    bool ok = commandMissionApproachTuning(input);
    printMissionPrompt();
    return ok;
  }
  if (sub == "instruction" || sub == "instructionpath" || sub == "missionzone" ||
      sub == "지시존" || sub == "미션지시존") {
    bool ok = commandMissionInstructionPath(input);
    printMissionPrompt();
    return ok;
  }
  if (sub == "scanrate" || sub == "scanspeed" || sub == "scantime" ||
      sub == "스캔속도" || sub == "스캔시간") {
    bool ok = commandMissionInstructionScanTiming(input);
    printMissionPrompt();
    return ok;
  }
  if (sub == "align" || sub == "storagealign" || sub == "정렬" ||
      sub == "적재함정렬") {
    bool ok = commandMissionAlignTuning(input);
    printMissionPrompt();
    return ok;
  }
  if (sub == "gripalign" || sub == "pickalign" || sub == "집기정렬" ||
      sub == "그립정렬") {
    bool ok = commandMissionGripAlignTuning(input);
    printMissionPrompt();
    return ok;
  }
  if (sub == "gripdepth" || sub == "pickdepth" || sub == "depth" ||
      sub == "집기깊이" || sub == "그립깊이") {
    bool ok = commandMissionGripDepthTuning(input);
    printMissionPrompt();
    return ok;
  }
  if (sub == "placealign" || sub == "dropalign" || sub == "zonealign" ||
      sub == "배치정렬" || sub == "내려놓기정렬") {
    bool ok = commandMissionPlaceAlignTuning(input);
    printMissionPrompt();
    return ok;
  }
  if (sub == "finishalign" || sub == "returnalign" || sub == "finishsl" ||
      sub == "복귀정렬" || sub == "후진정렬") {
    bool ok = commandMissionFinishAlignTuning(input);
    printMissionPrompt();
    return ok;
  }
  if (sub == "columnright" || sub == "colright" || sub == "rightcol" ||
      sub == "열오른쪽" || sub == "열우") {
    long steps = 1;
    if (tokenCount(input) >= 3 && !parseLongStrict(tokenAt(input, 2), &steps)) {
      DEBUG_SERIAL.println(F("사용법: mission columnright [steps]"));
      return false;
    }
    bool ok = commandMissionColumnNudge(DRIVE_DIRECTION_RIGHT, (uint8_t)constrain(steps, 1, 3));
    printMissionPrompt();
    return ok;
  }
  if (sub == "columnleft" || sub == "colleft" || sub == "leftcol" ||
      sub == "열왼쪽" || sub == "열좌") {
    long steps = 1;
    if (tokenCount(input) >= 3 && !parseLongStrict(tokenAt(input, 2), &steps)) {
      DEBUG_SERIAL.println(F("사용법: mission columnleft [steps]"));
      return false;
    }
    bool ok = commandMissionColumnNudge(DRIVE_DIRECTION_LEFT, (uint8_t)constrain(steps, 1, 3));
    printMissionPrompt();
    return ok;
  }
  if (sub == "jog" || sub == "move" || sub == "이동" || sub == "미세이동") {
    bool ok = commandMissionJog(input);
    printMissionPrompt();
    return ok;
  }
  if (sub == "columnscan" || sub == "colscan" || sub == "열스캔") {
    bool ok = commandMissionScanCurrentStorageColumn();
    if (ok && missionStage == MISSION_COLUMN_MOVE_OR_SCAN &&
        missionStorageColumn == missionTargetStorageColumn()) {
      missionStage = MISSION_PICK_HOLD;
    }
    printMissionPrompt();
    return ok;
  }
  if (sub == "undo" || sub == "원복" || sub == "되돌리기") {
    bool ok = commandMissionUndo();
    printMissionPrompt();
    return ok;
  }
  if (sub == "block" || sub == "블록") {
    String action = tokenAt(input, 2);
    action.toLowerCase();
    if (action != "next" && action != "다음") {
      DEBUG_SERIAL.println(F("사용법: mission block next"));
      return false;
    }
    if (missionStage != MISSION_REALIGN_OR_NEXT && missionStage != MISSION_FINISH) {
      DEBUG_SERIAL.println(F("[mission] block next는 배치 후 realign/finish 준비 단계에서만 사용하세요."));
      printMissionPrompt();
      return false;
    }
    if (missionStage == MISSION_FINISH) {
      DEBUG_SERIAL.println(F("[mission] 이미 finish 준비 상태입니다. mission next 또는 mission finish를 실행하세요."));
      printMissionPrompt();
      return false;
    }
    uint8_t totalBlocks = missionQueueCount > 0 ? missionQueueCount : CFG.storageRack.pickSlotCount;
    if (missionBlockIndex >= totalBlocks) {
      missionStage = MISSION_FINISH;
      DEBUG_SERIAL.println(F("[mission] 설정된 block을 모두 확인했습니다. 다음 SW1에서 finish를 실행합니다."));
      printMissionPrompt();
      return true;
    }
    missionBlockIndex++;
    setMissionBranchDefaultsForCurrentBlock();
    missionStage = MISSION_COLUMN_MOVE_OR_SCAN;
    DEBUG_SERIAL.print(F("[mission] 다음 블록으로 이동: "));
    DEBUG_SERIAL.println(missionBlockIndex);
    printMissionPrompt();
    return true;
  }
  if (sub == "goto" || sub == "이동") {
    MissionStepperStage target;
    if (!parseMissionStage(tokenAt(input, 2), &target)) {
      DEBUG_SERIAL.println(F("사용법: mission goto start|scan|storage|column|pick|place|realign|finish|finished"));
      return false;
    }
    missionStage = target;
    DEBUG_SERIAL.print(F("[mission] stage 이동: "));
    DEBUG_SERIAL.println(missionStageName(missionStage));
    printMissionPrompt();
    return true;
  }
  if (sub == "finish" || sub == "완료") {
    bool ok = commandMissionFinish();
    printMissionPrompt();
    return ok;
  }
  if (sub == "reset" || sub == "리셋") {
    missionStage = MISSION_IDLE;
    missionBlockIndex = 1;
    missionStorageColumn = 1;
    missionButtonMode = false;
    clearMissionQueue();
    resetMissionBranchSelection();
    clearMissionUndoCandidate();
    DEBUG_SERIAL.println(F("[mission] 상태를 초기화했습니다."));
    printMissionPrompt();
    return true;
  }

  DEBUG_SERIAL.println(F("사용법: mission start [quick]|run storage|next|rescan|accept|survey|plan|column <1~4>|columnpsd|columnscanrate|columnstep <mm> <mm/s>|columnscan|align|gripalign|placealign|finishalign|undo|finish|reset"));
  DEBUG_SERIAL.println(F("보조: mission button on/off|auto|upper|lower|slot <1~8>|block next|goto <stage>"));
  return false;
}

void commandStatus() {
  printLine();
  DEBUG_SERIAL.println(F("MissionRouteTuner 상태"));
  DEBUG_SERIAL.print(F("  profile: ")); DEBUG_SERIAL.println(profile().name);
  DEBUG_SERIAL.print(F("  mission stage: ")); DEBUG_SERIAL.println(missionStageName(missionStage));
  DEBUG_SERIAL.print(F("  mission speed: front="));
  DEBUG_SERIAL.print(missionMotion.frontCruiseSpeed);
  DEBUG_SERIAL.print(F("/"));
  DEBUG_SERIAL.print(missionMotion.frontSlowSpeed);
  DEBUG_SERIAL.print(F(" raw, psd="));
  DEBUG_SERIAL.print(missionMotion.psdCorrectionSpeed);
  DEBUG_SERIAL.print(F(", depth="));
  DEBUG_SERIAL.print(missionMotion.frontDepthCorrectionSpeed);
  DEBUG_SERIAL.print(F(", place="));
  DEBUG_SERIAL.print(missionMotion.missionZonePlaceCorrectionSpeed);
  DEBUG_SERIAL.print(F(" raw, position="));
  DEBUG_SERIAL.print(missionMotion.positionMoveMmPerSec);
  DEBUG_SERIAL.println(F("mm/s"));
  DEBUG_SERIAL.print(F("  column move/scan: "));
  DEBUG_SERIAL.print(missionColumnStepMm, 2);
  DEBUG_SERIAL.print(F("mm @ "));
  DEBUG_SERIAL.print(missionColumnMoveMmPerSec);
  DEBUG_SERIAL.print(F("mm/s, settle/frames/sample="));
  DEBUG_SERIAL.print(missionColumnScanSettleMs);
  DEBUG_SERIAL.print(F("ms/"));
  DEBUG_SERIAL.print(missionColumnScanFrames);
  DEBUG_SERIAL.print(F("/"));
  DEBUG_SERIAL.print(missionColumnScanSampleMs);
  DEBUG_SERIAL.println(F("ms"));
  DEBUG_SERIAL.print(F("  finish pre-align SL/tol/speed: "));
  DEBUG_SERIAL.print(missionFinishPreAlignSl);
  DEBUG_SERIAL.print(F("/"));
  DEBUG_SERIAL.print(missionFinishPreAlignTolerance);
  DEBUG_SERIAL.print(F("/"));
  DEBUG_SERIAL.println(missionFinishPreAlignSpeed);
  DEBUG_SERIAL.print(F("  wheel velocity trim FL/FR/BL/BR: "));
  DEBUG_SERIAL.print(missionWheelVelocityTrimFl, 3);
  DEBUG_SERIAL.print(F("/"));
  DEBUG_SERIAL.print(missionWheelVelocityTrimFr, 3);
  DEBUG_SERIAL.print(F("/"));
  DEBUG_SERIAL.print(missionWheelVelocityTrimBl, 3);
  DEBUG_SERIAL.print(F("/"));
  DEBUG_SERIAL.println(missionWheelVelocityTrimBr, 3);
  DEBUG_SERIAL.print(F("  manipulator: ")); DEBUG_SERIAL.println(manipulatorReady ? F("OK") : F("FAIL"));
  DEBUG_SERIAL.print(F("  mobilebase: ")); DEBUG_SERIAL.println(mobileReady ? F("OK") : F("FAIL"));
  DEBUG_SERIAL.print(F("  pixy/gripper: ")); DEBUG_SERIAL.println(pixyReady ? F("INIT CALLED") : F("NOT INIT"));
  DEBUG_SERIAL.println(F("  serial: USB 115200 primary, Bluetooth Serial2 input-only fallback"));
  DEBUG_SERIAL.print(F("  psd watch: "));
  DEBUG_SERIAL.print(psdWatchEnabled ? F("ON ") : F("OFF "));
  DEBUG_SERIAL.print(psdWatchIntervalMs);
  DEBUG_SERIAL.println(F("ms"));
  DEBUG_SERIAL.print(F("  last replay: "));
  if (lastReplayableCommand.length() > 0) {
    DEBUG_SERIAL.println(lastReplayableCommand);
  } else {
    DEBUG_SERIAL.println(F("(none)"));
  }
  printLine();
}

void printPsdSnapshotLine(const PsdSnapshot &snapshot, const __FlashStringHelper *label) {
  DEBUG_SERIAL.print(label);
  DEBUG_SERIAL.print(F(" FL="));
  DEBUG_SERIAL.print(snapshot.fl);
  DEBUG_SERIAL.print(F(" FR="));
  DEBUG_SERIAL.print(snapshot.fr);
  DEBUG_SERIAL.print(F(" SL="));
  DEBUG_SERIAL.print(snapshot.sl);
  DEBUG_SERIAL.print(F(" SR="));
  DEBUG_SERIAL.print(snapshot.sr);
  DEBUG_SERIAL.print(F(" | instructionSL="));
  DEBUG_SERIAL.print(missionInstructionSl);
  DEBUG_SERIAL.print(F(" scan SL/FL/FR="));
  DEBUG_SERIAL.print(missionAlignSl);
  DEBUG_SERIAL.print(F("/"));
  DEBUG_SERIAL.print(missionAlignFl);
  DEBUG_SERIAL.print(F("/"));
  DEBUG_SERIAL.print(missionAlignFr);
  DEBUG_SERIAL.print(F(" tol="));
  DEBUG_SERIAL.print(missionAlignTolerance);
  DEBUG_SERIAL.print(F(" upperGrip FL/FR(SLref)="));
  DEBUG_SERIAL.print(missionGripAlignFl);
  DEBUG_SERIAL.print(F("/"));
  DEBUG_SERIAL.print(missionGripAlignFr);
  DEBUG_SERIAL.print(F("("));
  DEBUG_SERIAL.print(missionGripAlignSl);
  DEBUG_SERIAL.print(F(")"));
  DEBUG_SERIAL.print(F(" tol="));
  DEBUG_SERIAL.print(missionGripAlignTolerance);
  DEBUG_SERIAL.print(F(" lowerGrip FL/FR(SLref)="));
  DEBUG_SERIAL.print(missionLowerGripAlignFl);
  DEBUG_SERIAL.print(F("/"));
  DEBUG_SERIAL.print(missionLowerGripAlignFr);
  DEBUG_SERIAL.print(F("("));
  DEBUG_SERIAL.print(missionLowerGripAlignSl);
  DEBUG_SERIAL.print(F(")"));
  DEBUG_SERIAL.print(F(" tol="));
  DEBUG_SERIAL.print(missionLowerGripAlignTolerance);
  DEBUG_SERIAL.print(F(" placeSL/FR="));
  DEBUG_SERIAL.print(missionPlaceSl);
  DEBUG_SERIAL.print(F("/"));
  DEBUG_SERIAL.print(missionPlaceFr);
  DEBUG_SERIAL.print(F(" tol="));
  DEBUG_SERIAL.print(missionPlaceTolerance);
  DEBUG_SERIAL.print(F(" finishSL="));
  DEBUG_SERIAL.print(missionFinishPreAlignSl);
  DEBUG_SERIAL.print(F(" tol="));
  DEBUG_SERIAL.print(missionFinishPreAlignTolerance);
  DEBUG_SERIAL.println(F(" (SR ignored)"));
}

void printPsdStatus() {
  PsdSnapshot snapshot;
  readAllPSDSensors(&snapshot);
  printPsdSnapshotLine(snapshot, F("[psd]"));
  DEBUG_SERIAL.print(F("{\"type\":\"psd-status\",\"watch\":"));
  printJsonBool(psdWatchEnabled);
  DEBUG_SERIAL.print(F(",\"intervalMs\":"));
  DEBUG_SERIAL.print(psdWatchIntervalMs);
  DEBUG_SERIAL.print(F(",\"snapshot\":"));
  printJsonPsdSnapshot(snapshot);
  DEBUG_SERIAL.println(F("}"));
}

void updatePsdWatch() {
  if (!psdWatchEnabled) return;
  unsigned long now = millis();
  if (now - psdWatchLastPrintMs < psdWatchIntervalMs) return;
  psdWatchLastPrintMs = now;

  PsdSnapshot snapshot;
  readAllPSDSensors(&snapshot);
  printPsdSnapshotLine(snapshot, F("[psd watch]"));
}

bool commandPsd(const String &input) {
  String sub = tokenAt(input, 1);
  sub.toLowerCase();

  if (sub.length() == 0 || sub == "status" || sub == "상태" ||
      sub == "read" || sub == "값") {
    printPsdStatus();
    return true;
  }

  if (sub == "watch" || sub == "on" || sub == "toggle" || sub == "토글" ||
      sub == "켜기") {
    long intervalMs = psdWatchIntervalMs;
    if (tokenCount(input) >= 3 &&
        !parseLongStrict(tokenAt(input, 2), &intervalMs)) {
      DEBUG_SERIAL.println(F("사용법: psd watch [intervalMs] | psd off"));
      return false;
    }
    if (intervalMs < 100 || intervalMs > 5000) {
      DEBUG_SERIAL.println(F("[제한] psd watch interval은 100~5000ms 범위입니다."));
      return false;
    }
    psdWatchIntervalMs = (uint16_t)intervalMs;
    psdWatchEnabled = (sub == "toggle" || sub == "토글") ? !psdWatchEnabled : true;
    psdWatchLastPrintMs = 0;
    DEBUG_SERIAL.print(F("[psd watch] "));
    DEBUG_SERIAL.print(psdWatchEnabled ? F("ON") : F("OFF"));
    DEBUG_SERIAL.print(F(", intervalMs="));
    DEBUG_SERIAL.println(psdWatchIntervalMs);
    printPsdStatus();
    return true;
  }

  if (sub == "off" || sub == "stop" || sub == "끄기") {
    psdWatchEnabled = false;
    DEBUG_SERIAL.println(F("[psd watch] OFF"));
    return true;
  }

  if (sub == "targets" || sub == "target" || sub == "기준") {
    printMissionInstructionPathStatus();
    printMissionAlignStatus();
    printPsdStatus();
    return true;
  }

  DEBUG_SERIAL.println(F("사용법: psd status | psd watch [ms] | psd off | psd targets"));
  return false;
}

void printTunerCalibrationStatus() {
  TunerCalibrationRecord record;
  bool storedValid = readTunerCalibrationRecord(&record);
  printLine();
  DEBUG_SERIAL.println(F("튜너 캘리브레이션 상태"));
  DEBUG_SERIAL.print(F("  EEPROM reserved: "));
  DEBUG_SERIAL.print(TUNER_CAL_EEPROM_ADDR);
  DEBUG_SERIAL.print(F("~"));
  DEBUG_SERIAL.print(TUNER_CAL_EEPROM_ADDR + (int)sizeof(TunerCalibrationRecord) - 1);
  DEBUG_SERIAL.print(F(" / length="));
  DEBUG_SERIAL.println(EEPROM.length());
  DEBUG_SERIAL.print(F("  pose EEPROM area: 0~"));
  DEBUG_SERIAL.print(TUNER_CAL_EEPROM_ADDR - 1);
  DEBUG_SERIAL.println(F(" (cal save/load는 이 영역을 쓰지 않습니다)"));
  DEBUG_SERIAL.print(F("  stored valid: "));
  DEBUG_SERIAL.println(storedValid ? F("YES") : F("NO"));
  DEBUG_SERIAL.print(F("  approach: firstDetect="));
  DEBUG_SERIAL.print(missionFrontFirstDetectAdc);
  DEBUG_SERIAL.print(F(", decelWindow="));
  DEBUG_SERIAL.print(missionFrontDecelWindowAdc);
  DEBUG_SERIAL.print(F(", afterDetect="));
  DEBUG_SERIAL.print(missionFrontAfterDetectMm, 2);
  DEBUG_SERIAL.print(F("mm @ "));
  DEBUG_SERIAL.print(missionFrontAfterDetectMmPerSec);
  DEBUG_SERIAL.println(F(" raw"));
  DEBUG_SERIAL.print(F("  instruction: SL="));
  DEBUG_SERIAL.print(missionInstructionSl);
  DEBUG_SERIAL.print(F(", finalForwardMs="));
  DEBUG_SERIAL.print(missionInstructionFinalForwardMs);
  DEBUG_SERIAL.print(F(", finalForwardRaw="));
  DEBUG_SERIAL.println(missionInstructionFinalForwardSpeed);
  DEBUG_SERIAL.print(F("  instruction scan: "));
  DEBUG_SERIAL.print(missionInstructionScanMs);
  DEBUG_SERIAL.print(F("ms / sample "));
  DEBUG_SERIAL.print(missionInstructionScanSampleMs);
  DEBUG_SERIAL.println(F("ms"));
  DEBUG_SERIAL.print(F("  storage path: "));
  DEBUG_SERIAL.print(missionStorageFirstForwardMm, 2);
  DEBUG_SERIAL.print(F(" / "));
  DEBUG_SERIAL.print(missionStorageExtraForwardMm, 2);
  DEBUG_SERIAL.print(F(" / "));
  DEBUG_SERIAL.print(missionStorageRightMm, 2);
  DEBUG_SERIAL.println(F("mm"));
  DEBUG_SERIAL.print(F("  column: "));
  DEBUG_SERIAL.print(missionColumnStepMm, 2);
  DEBUG_SERIAL.print(F("mm @ "));
  DEBUG_SERIAL.print(missionColumnMoveMmPerSec);
  DEBUG_SERIAL.println(F("mm/s"));
  DEBUG_SERIAL.print(F("  column scan: settle "));
  DEBUG_SERIAL.print(missionColumnScanSettleMs);
  DEBUG_SERIAL.print(F("ms / frames "));
  DEBUG_SERIAL.print(missionColumnScanFrames);
  DEBUG_SERIAL.print(F(" / sample "));
  DEBUG_SERIAL.print(missionColumnScanSampleMs);
  DEBUG_SERIAL.println(F("ms"));
  DEBUG_SERIAL.print(F("  scan align SL/FL/FR/tol: "));
  DEBUG_SERIAL.print(missionAlignSl);
  DEBUG_SERIAL.print(F(" / "));
  DEBUG_SERIAL.print(missionAlignFl);
  DEBUG_SERIAL.print(F(" / "));
  DEBUG_SERIAL.print(missionAlignFr);
  DEBUG_SERIAL.print(F(" / "));
  DEBUG_SERIAL.print(missionAlignTolerance);
  DEBUG_SERIAL.println(F(" (SR ignored)"));
  DEBUG_SERIAL.print(F("  grip depth FL/FR/tol (SL ref): "));
  DEBUG_SERIAL.print(missionGripAlignFl);
  DEBUG_SERIAL.print(F(" / "));
  DEBUG_SERIAL.print(missionGripAlignFr);
  DEBUG_SERIAL.print(F(" / "));
  DEBUG_SERIAL.print(missionGripAlignTolerance);
  DEBUG_SERIAL.print(F(" (SL ref="));
  DEBUG_SERIAL.print(missionGripAlignSl);
  DEBUG_SERIAL.println(F(", SL/SR ignored for motion)"));
  DEBUG_SERIAL.print(F("  lower grip depth FL/FR/tol (SL ref): "));
  DEBUG_SERIAL.print(missionLowerGripAlignFl);
  DEBUG_SERIAL.print(F(" / "));
  DEBUG_SERIAL.print(missionLowerGripAlignFr);
  DEBUG_SERIAL.print(F(" / "));
  DEBUG_SERIAL.print(missionLowerGripAlignTolerance);
  DEBUG_SERIAL.print(F(" (SL ref="));
  DEBUG_SERIAL.print(missionLowerGripAlignSl);
  DEBUG_SERIAL.println(F(", SL/SR ignored for motion)"));
  DEBUG_SERIAL.print(F("  grip depth upper/lower: "));
  DEBUG_SERIAL.print(missionUpperGripDepthMm, 2);
  DEBUG_SERIAL.print(F(" / "));
  DEBUG_SERIAL.print(missionLowerGripDepthMm, 2);
  DEBUG_SERIAL.print(F("mm @ "));
  DEBUG_SERIAL.print(missionGripDepthMmPerSec);
  DEBUG_SERIAL.println(F("mm/s"));
  DEBUG_SERIAL.print(F("  place align SL/FR/tol: "));
  DEBUG_SERIAL.print(missionPlaceSl);
  DEBUG_SERIAL.print(F(" / "));
  DEBUG_SERIAL.print(missionPlaceFr);
  DEBUG_SERIAL.print(F(" / "));
  DEBUG_SERIAL.println(missionPlaceTolerance);
  DEBUG_SERIAL.print(F("{\"type\":\"cal-status\",\"calibration\":"));
  printJsonTunerCalibration();
  DEBUG_SERIAL.println(F("}"));
  printLine();
}

bool saveTunerCalibrationToEEPROM() {
  if (!tunerCalibrationFitsEEPROM()) {
    DEBUG_SERIAL.println(F("[cal save] EEPROM 여유 영역이 부족합니다. 저장하지 않았습니다."));
    return false;
  }
  TunerCalibrationRecord record;
  fillTunerCalibrationRecord(&record);
  if (!tunerCalibrationRecordLooksSafe(record)) {
    DEBUG_SERIAL.println(F("[cal save] 현재 튜닝값 검증에 실패했습니다. 저장하지 않았습니다."));
    return false;
  }
  EEPROM.put(TUNER_CAL_EEPROM_ADDR, record);
  TunerCalibrationRecord verify;
  if (!readTunerCalibrationRecord(&verify)) {
    DEBUG_SERIAL.println(F("[cal save] 저장 후 검증 실패. 값을 다시 확인하세요."));
    return false;
  }
  DEBUG_SERIAL.print(F("[cal save] EEPROM "));
  DEBUG_SERIAL.print(TUNER_CAL_EEPROM_ADDR);
  DEBUG_SERIAL.print(F("번부터 "));
  DEBUG_SERIAL.print(sizeof(TunerCalibrationRecord));
  DEBUG_SERIAL.println(F("바이트에 튜너 보정값을 저장했습니다. pose EEPROM은 건드리지 않았습니다."));
  return true;
}

bool clearTunerCalibrationFromEEPROM() {
  if (!tunerCalibrationFitsEEPROM()) {
    DEBUG_SERIAL.println(F("[cal clear] EEPROM 여유 영역 확인 실패."));
    return false;
  }
  uint32_t zero = 0;
  EEPROM.put(TUNER_CAL_EEPROM_ADDR, zero);
  DEBUG_SERIAL.println(F("[cal clear] 튜너 보정 record magic만 지웠습니다. pose EEPROM은 건드리지 않았습니다."));
  return true;
}

bool commandCalibration(const String &input) {
  String sub = tokenAt(input, 1);
  sub.toLowerCase();

  if (sub.length() == 0 || sub == "status" || sub == "상태") {
    printTunerCalibrationStatus();
    return true;
  }
  if (sub == "save" || sub == "저장") {
    bool ok = saveTunerCalibrationToEEPROM();
    printTunerCalibrationStatus();
    return ok;
  }
  if (sub == "load" || sub == "불러오기") {
    bool ok = loadTunerCalibrationFromEEPROM(true);
    printTunerCalibrationStatus();
    return ok;
  }
  if (sub == "defaults" || sub == "default" || sub == "reset" || sub == "기본값") {
    resetTunerCalibrationToDefaults();
    DEBUG_SERIAL.println(F("[cal defaults] MissionConfig 기본값/profile preset으로 되돌렸습니다. EEPROM에는 쓰지 않았습니다."));
    printTunerCalibrationStatus();
    return true;
  }
  if (sub == "clear" || sub == "erase" || sub == "삭제") {
    bool ok = clearTunerCalibrationFromEEPROM();
    printTunerCalibrationStatus();
    return ok;
  }
  if (sub == "eeprom" || sub == "addr" || sub == "주소") {
    DEBUG_SERIAL.print(F("[cal eeprom] pose=0~"));
    DEBUG_SERIAL.print(TUNER_CAL_EEPROM_ADDR - 1);
    DEBUG_SERIAL.print(F(", tuner="));
    DEBUG_SERIAL.print(TUNER_CAL_EEPROM_ADDR);
    DEBUG_SERIAL.print(F("~"));
    DEBUG_SERIAL.print(TUNER_CAL_EEPROM_ADDR + (int)sizeof(TunerCalibrationRecord) - 1);
    DEBUG_SERIAL.print(F(", length="));
    DEBUG_SERIAL.println(EEPROM.length());
    return true;
  }

  DEBUG_SERIAL.println(F("사용법: cal status|save|load|defaults|clear|eeprom"));
  DEBUG_SERIAL.println(F("  cal save는 pose EEPROM(0~3999)이 아니라 튜너 전용 여유 영역만 씁니다."));
  return false;
}

void printProfileTable() {
  DEBUG_SERIAL.println(F("profile safe|normal|fast|max"));
  for (uint8_t i = 0; i < PROFILE_COUNT; i++) {
    DEBUG_SERIAL.print(F("  "));
    DEBUG_SERIAL.print(PROFILES[i].name);
    DEBUG_SERIAL.print(F(": poseMs="));
    DEBUG_SERIAL.print(PROFILES[i].minPoseMs);
    DEBUG_SERIAL.print(F("~"));
    DEBUG_SERIAL.print(PROFILES[i].maxPoseMs);
    DEBUG_SERIAL.print(F(", driveMax="));
    DEBUG_SERIAL.print(PROFILES[i].maxDriveMm);
    DEBUG_SERIAL.print(F("mm, speedMax="));
    DEBUG_SERIAL.print(PROFILES[i].maxDriveMmPerSec);
    DEBUG_SERIAL.print(F("mm/s, driveTimeMax="));
    DEBUG_SERIAL.print(PROFILES[i].maxDriveTimeMs);
    DEBUG_SERIAL.print(F("ms, missionScale="));
    DEBUG_SERIAL.print(PROFILES[i].missionVelocityScale, 2);
    DEBUG_SERIAL.print(F(", missionVelocityMax="));
    DEBUG_SERIAL.print(PROFILES[i].maxMissionVelocityRaw);
    DEBUG_SERIAL.println(F("raw"));
  }
}

void printMissionSpeedStatus() {
  printLine();
  DEBUG_SERIAL.println(F("미션 동작 속도"));
  DEBUG_SERIAL.print(F("  profile: "));
  DEBUG_SERIAL.println(profile().name);
  DEBUG_SERIAL.print(F("  front cruise/slow raw: "));
  DEBUG_SERIAL.print(missionMotion.frontCruiseSpeed);
  DEBUG_SERIAL.print(F(" / "));
  DEBUG_SERIAL.println(missionMotion.frontSlowSpeed);
  DEBUG_SERIAL.print(F("  PSD align raw: "));
  DEBUG_SERIAL.println(missionMotion.psdCorrectionSpeed);
  DEBUG_SERIAL.print(F("  front depth raw: "));
  DEBUG_SERIAL.println(missionMotion.frontDepthCorrectionSpeed);
  DEBUG_SERIAL.print(F("  mission-zone place align raw: "));
  DEBUG_SERIAL.println(missionMotion.missionZonePlaceCorrectionSpeed);
  DEBUG_SERIAL.print(F("  position move: "));
  DEBUG_SERIAL.print(missionMotion.positionMoveMmPerSec);
  DEBUG_SERIAL.println(F("mm/s"));
  DEBUG_SERIAL.print(F("  actuator pose/hold ms: fixed "));
  DEBUG_SERIAL.println(MISSION_ACTUATOR_MS);
  DEBUG_SERIAL.print(F("  column step: "));
  DEBUG_SERIAL.print(missionColumnStepMm, 2);
  DEBUG_SERIAL.print(F("mm @ "));
  DEBUG_SERIAL.print(missionColumnMoveMmPerSec);
  DEBUG_SERIAL.println(F("mm/s"));
  DEBUG_SERIAL.print(F("  column scan settle/frames/sample: "));
  DEBUG_SERIAL.print(missionColumnScanSettleMs);
  DEBUG_SERIAL.print(F("ms / "));
  DEBUG_SERIAL.print(missionColumnScanFrames);
  DEBUG_SERIAL.print(F(" / "));
  DEBUG_SERIAL.print(missionColumnScanSampleMs);
  DEBUG_SERIAL.println(F("ms"));
  DEBUG_SERIAL.print(F("  storage path first/extra/right: "));
  DEBUG_SERIAL.print(missionStorageFirstForwardMm, 2);
  DEBUG_SERIAL.print(F(" / "));
  DEBUG_SERIAL.print(missionStorageExtraForwardMm, 2);
  DEBUG_SERIAL.print(F(" / "));
  DEBUG_SERIAL.print(missionStorageRightMm, 2);
  DEBUG_SERIAL.println(F("mm"));
  DEBUG_SERIAL.println();
  DEBUG_SERIAL.println(F("사용법: speed status | speed reset | speed safe|normal|fast|max"));
  DEBUG_SERIAL.println(F("       speed set front|slow|psd|depth|place|position <value>"));
  DEBUG_SERIAL.println(F("       mission approach <afterDetectMm> <raw> [firstDetectAdc]"));
  DEBUG_SERIAL.println(F("       mission instruction <targetSl> <finalForwardMs> <raw>"));
  DEBUG_SERIAL.println(F("       mission scanrate <scanMs> <sampleMs>"));
  DEBUG_SERIAL.println(F("       mission align <targetSl> <targetFl> <targetFr> [tolerance]"));
  DEBUG_SERIAL.println(F("       mission align current [tolerance]"));
  DEBUG_SERIAL.println(F("       mission storagegate <slLeave> <slReenter> [forwardRaw] [samples] [ignoreMs]"));
  DEBUG_SERIAL.println(F("       mission gripalign upper|lower <targetFl> <targetFr> [tolerance]"));
  DEBUG_SERIAL.println(F("       mission gripalign upper|lower current|run [tolerance]"));
  DEBUG_SERIAL.println(F("       mission gripdepth <upperMm> <lowerMm> [speed]"));
  DEBUG_SERIAL.println(F("       mission placealign <targetSl> <targetFr> [tolerance]"));
  DEBUG_SERIAL.println(F("       mission placealign current|run [tolerance]"));
  DEBUG_SERIAL.println(F("       mission columnstep <mm> <mm/s>"));
  DEBUG_SERIAL.println(F("       mission columnscanrate <settleMs> <frames> <sampleMs>"));
  DEBUG_SERIAL.println(F("       mission columnright|columnleft, mission jog <dir> <mm> <mm/s>"));
  DEBUG_SERIAL.println(F("       mission storagepath <first> <extra> <right> [speed]"));
  printLine();
}

void commandProfile(const String &input) {
  String name = tokenAt(input, 1);
  if (name.length() == 0) {
    printProfileTable();
    return;
  }
  name.toLowerCase();
  for (uint8_t i = 0; i < PROFILE_COUNT; i++) {
    if (name.equals(PROFILES[i].name)) {
      profileIndex = i;
      applyMissionSpeedPresetForProfile();
      DEBUG_SERIAL.print(F("[profile] "));
      DEBUG_SERIAL.println(PROFILES[i].name);
      DEBUG_SERIAL.print(F("[speed] profile 기준 주행/정렬 속도 preset만 적용했습니다. actuator ms는 "));
      DEBUG_SERIAL.print(MISSION_ACTUATOR_MS);
      DEBUG_SERIAL.println(F("로 유지합니다."));
      printMissionSpeedStatus();
      return;
    }
  }
  DEBUG_SERIAL.println(F("[오류] profile은 safe/normal/fast/max 중 하나입니다."));
}

bool isProfileName(const String &name, uint8_t *index) {
  for (uint8_t i = 0; i < PROFILE_COUNT; i++) {
    if (name.equals(PROFILES[i].name)) {
      *index = i;
      return true;
    }
  }
  return false;
}

bool validateMissionVelocitySpeed(long value) {
  if (value <= 0 || value > profile().maxMissionVelocityRaw) {
    DEBUG_SERIAL.print(F("[제한] 현재 profile의 미션 velocity raw 한도는 "));
    DEBUG_SERIAL.print(profile().maxMissionVelocityRaw);
    DEBUG_SERIAL.println(F("입니다."));
    return false;
  }
  return true;
}

bool validateMissionPositionSpeed(long value) {
  if (value <= 0 || value > profile().maxDriveMmPerSec) {
    DEBUG_SERIAL.print(F("[제한] 현재 profile의 position 이동 속도 한도는 "));
    DEBUG_SERIAL.print(profile().maxDriveMmPerSec);
    DEBUG_SERIAL.println(F("mm/s입니다. 더 빠르게 보려면 profile fast 또는 max를 먼저 쓰세요."));
    return false;
  }
  return true;
}

bool validateMissionPoseMs(long value) {
  if (value < 100 || value > 500) {
    DEBUG_SERIAL.println(F("[제한] 시퀀스 pose/hold 시간은 100~500ms 범위만 허용합니다."));
    return false;
  }
  return true;
}

bool commandSpeedSet(const String &input) {
  String key = tokenAt(input, 2);
  key.toLowerCase();
  long value = 0;
  if (!parseLongStrict(tokenAt(input, 3), &value)) {
    DEBUG_SERIAL.println(F("사용법: speed set <front|slow|psd|depth|place|position> <value>"));
    return false;
  }

  if (key == "front" || key == "cruise" || key == "approach") {
    if (!validateMissionVelocitySpeed(value)) return false;
    missionMotion.frontCruiseSpeed = value;
  } else if (key == "slow") {
    if (!validateMissionVelocitySpeed(value)) return false;
    if (value >= missionMotion.frontCruiseSpeed) {
      DEBUG_SERIAL.println(F("[오류] slow는 front cruise보다 작아야 합니다."));
      return false;
    }
    missionMotion.frontSlowSpeed = value;
  } else if (key == "psd" || key == "align" || key == "correction") {
    if (!validateMissionVelocitySpeed(value)) return false;
    missionMotion.psdCorrectionSpeed = value;
  } else if (key == "depth" || key == "frontdepth" || key == "deep" ||
             key == "깊이") {
    if (!validateMissionVelocitySpeed(value)) return false;
    missionMotion.frontDepthCorrectionSpeed = value;
  } else if (key == "place" || key == "placealign" || key == "zone" ||
             key == "missionzone" || key == "drop" || key == "배치") {
    if (!validateMissionVelocitySpeed(value)) return false;
    missionMotion.missionZonePlaceCorrectionSpeed = value;
  } else if (key == "position" || key == "drive" || key == "move") {
    if (!validateMissionPositionSpeed(value)) return false;
    missionMotion.positionMoveMmPerSec = value;
  } else if (key == "initial" || key == "storage" || key == "pregrip" ||
             key == "grip" || key == "return" ||
             key == "hold" || key == "griphold" || key == "placehold") {
    DEBUG_SERIAL.print(F("[고정] mission actuator pose/hold 시간은 "));
    DEBUG_SERIAL.print(MISSION_ACTUATOR_MS);
    DEBUG_SERIAL.println(F("ms로 유지합니다."));
    DEBUG_SERIAL.println(F("       1회 자세 시간 테스트는 pose run/tune 또는 delaytest를 쓰세요."));
    return false;
  } else {
    DEBUG_SERIAL.println(F("[오류] speed key를 알 수 없습니다."));
    DEBUG_SERIAL.println(F("가능: front, slow, psd, depth, place, position"));
    return false;
  }

  DEBUG_SERIAL.print(F("[speed set] "));
  DEBUG_SERIAL.print(key);
  DEBUG_SERIAL.print(F("="));
  DEBUG_SERIAL.println(value);
  printMissionSpeedStatus();
  return true;
}

bool commandSpeed(const String &input) {
  String sub = tokenAt(input, 1);
  sub.toLowerCase();

  if (sub.length() == 0 || sub == "status" || sub == "상태") {
    printMissionSpeedStatus();
    return true;
  }
  if (sub == "reset" || sub == "리셋") {
    applyMissionSpeedPresetForProfile();
    DEBUG_SERIAL.println(F("[speed] 현재 profile 기준 preset으로 되돌렸습니다."));
    printMissionSpeedStatus();
    return true;
  }
  if (sub == "set" || sub == "설정") {
    return commandSpeedSet(input);
  }

  uint8_t newProfileIndex = 0;
  if (isProfileName(sub, &newProfileIndex)) {
    profileIndex = newProfileIndex;
    applyMissionSpeedPresetForProfile();
    DEBUG_SERIAL.print(F("[profile] "));
    DEBUG_SERIAL.println(PROFILES[profileIndex].name);
    DEBUG_SERIAL.print(F("[speed] profile preset 적용: 주행/정렬 속도만 변경, actuator ms는 "));
    DEBUG_SERIAL.print(MISSION_ACTUATOR_MS);
    DEBUG_SERIAL.println(F(" 유지"));
    printMissionSpeedStatus();
    return true;
  }

  DEBUG_SERIAL.println(F("사용법: speed status | speed reset | speed safe|normal|fast|max | speed set <key> <value>"));
  return false;
}

void printGuideHeader(const __FlashStringHelper *title) {
  printLine();
  DEBUG_SERIAL.println(title);
  DEBUG_SERIAL.println(F("그대로 한 줄씩 입력하세요. 움직이는 명령 전에는 ! 긴급정지를 준비하세요."));
  printLine();
}

void commandGuideMain() {
  printLine();
  DEBUG_SERIAL.println(F("현장 가이드: 최소 명령"));
  DEBUG_SERIAL.println(F("  mission start                  : SW1 단계 진행 시작"));
  DEBUG_SERIAL.println(F("  mission start quick            : 지시존 접근+주행 중 스캔+적재함 접근 묶음 실행"));
  DEBUG_SERIAL.println(F("  mission run storage            : mission start quick 별칭"));
  DEBUG_SERIAL.println(F("  mission next                   : Serial에서 현재 SW1 단계 1회 실행"));
  DEBUG_SERIAL.println(F("  mission rescan                 : 미션지시존 Pixy 재스캔"));
  DEBUG_SERIAL.println(F("  mission accept                 : 스캔 queue 수락 후 적재함 이동 준비"));
  DEBUG_SERIAL.println(F("  mission survey                 : 적재함 1~4열 전체 스캔/조기 종료"));
  DEBUG_SERIAL.println(F("  mission plan                   : survey 결과로 sourceSlot=goalSlot queue 적용"));
  DEBUG_SERIAL.println(F("  mission approach <mm> <raw>    : 최초 장애물 감지 후 이어서 갈 양/출력"));
  DEBUG_SERIAL.println(F("  mission instruction <sl> <ms> <raw>: 좌측 지시존 정렬/직진 보정"));
  DEBUG_SERIAL.println(F("  mission scanrate <scanMs> <sampleMs>: 미션지시존 스캔 시간/간격"));
  DEBUG_SERIAL.println(F("  mission align <sl> <fl> <fr> [tol]: 적재함 1열 스캔 정렬 기준"));
  DEBUG_SERIAL.println(F("  mission storagegate <slLeave> <slReenter> [forwardRaw] [samples] [ignoreMs]: SL 재감지 전진 조건"));
  DEBUG_SERIAL.println(F("  mission gripalign upper|lower <fl> <fr> [tol]: 집기 직전 FL/FR 깊이 기준"));
  DEBUG_SERIAL.println(F("  mission gripdepth <upperMm> <lowerMm> [speed]: 그립 직전 추가 전진 깊이"));
  DEBUG_SERIAL.println(F("  mission placealign <sl> <fr> [tol]: 미션수행존 배치 SL+FR 기준"));
  DEBUG_SERIAL.println(F("  mission columnstep <mm> <mm/s> : 열 이동량/속도 테스트값"));
  DEBUG_SERIAL.println(F("  mission columnscanrate <settle> <frames> <sample>: 열 도착 후 확인 시간"));
  DEBUG_SERIAL.println(F("  mission columnpsd [col] [n] [ms]: 현재 위치 PSD 평균/min/max JSON 출력"));
  DEBUG_SERIAL.println(F("  mission columnright/columnleft : columnstep 기준 한 칸 수동 이동"));
  DEBUG_SERIAL.println(F("  mission jog <dir> <mm> <mm/s>  : 정위치에서 임의 보정 이동"));
  DEBUG_SERIAL.println(F("  drive balance set/test         : velocity-mode 좌/우 바퀴 속도 보정"));
  DEBUG_SERIAL.println(F("  mission storagepath <f> <e> <r>: 적재함 접근 고정거리 테스트"));
  DEBUG_SERIAL.println(F("  mission columnscan             : 현재 열 Pixy 스캔/판정 저장"));
  DEBUG_SERIAL.println(F("  pixy alignslow upper|lower     : 저속 micro-step 중심 정렬 테스트"));
  DEBUG_SERIAL.println(F("  psd watch [ms] / psd off       : PSD 네 센서값 토글 출력"));
  DEBUG_SERIAL.println(F("  cal status|save|load           : 튜너 주행/PSD 보정값 저장/불러오기"));
  DEBUG_SERIAL.println(F("  status                         : 초기화/현재 stage 상태"));
  DEBUG_SERIAL.println(F("  pose verify                    : EEPROM 자세 1,3,4,5,7~14 확인"));
  DEBUG_SERIAL.println(F("  pixy scan all 5 | pixy watch all 30"));
  DEBUG_SERIAL.println(F("  stop 또는 !                     : 즉시 정지"));
  DEBUG_SERIAL.println();
  DEBUG_SERIAL.println(F("긴 도움말은 help advanced, 주제별은 help pose|pixy|drive|seq|speed를 사용하세요."));
  printLine();
}

void commandGuideFirst() {
  printGuideHeader(F("[가이드] 첫 연결 후 안전 확인"));
  DEBUG_SERIAL.println(F("1) 상태 확인"));
  DEBUG_SERIAL.println(F("status"));
  DEBUG_SERIAL.println(F("2) 현재 테스트 자세 1,3,4,5,7~14 누락 검사"));
  DEBUG_SERIAL.println(F("pose verify"));
  DEBUG_SERIAL.println(F("3) EEPROM 백업 출력"));
  DEBUG_SERIAL.println(F("pose backup"));
  DEBUG_SERIAL.println(F("4) 안전 프로파일"));
  DEBUG_SERIAL.println(F("profile safe"));
  DEBUG_SERIAL.println(F("5) 현재 팔 raw 확인"));
  DEBUG_SERIAL.println(F("pose present"));
  DEBUG_SERIAL.println(F("6) 1번 자세 이동 전 미리보기"));
  DEBUG_SERIAL.println(F("pose plan 1 300"));
  DEBUG_SERIAL.println(F("7) 안전하면 1번 자세 실행"));
  DEBUG_SERIAL.println(F("pose run 1 300"));
  DEBUG_SERIAL.println(F("문제 있으면: ! 또는 stop"));
  printLine();
}

void commandGuidePose() {
  printGuideHeader(F("[가이드] 자세/높이 조정"));
  DEBUG_SERIAL.println(F("1) 기준 자세와 현재 차이 확인"));
  DEBUG_SERIAL.println(F("pose present"));
  DEBUG_SERIAL.println(F("pose diff 3"));
  DEBUG_SERIAL.println(F("2) 조정 목표 미리보기"));
  DEBUG_SERIAL.println(F("pose tuneplan 3 300 0 +30 -30 0"));
  DEBUG_SERIAL.println(F("3) 안전하면 실제 1회 테스트"));
  DEBUG_SERIAL.println(F("pose tune 3 300 0 +30 -30 0"));
  DEBUG_SERIAL.println(F("4) 맞으면 저장 후보 확인 후 확정"));
  DEBUG_SERIAL.println(F("pose apply storage_height_test"));
  DEBUG_SERIAL.println(F("pose confirm"));
  DEBUG_SERIAL.println(F("팁: 높이는 m2/m3/m4가 같이 바뀌므로 20~50 raw씩 나눠 확인하세요."));
  printLine();
}

void commandGuidePixy() {
  printGuideHeader(F("[가이드] Pixy 인식 안정화"));
  DEBUG_SERIAL.println(F("1) 조명 끄고 전체 인식 확인"));
  DEBUG_SERIAL.println(F("pixy lamp off"));
  DEBUG_SERIAL.println(F("pixy scan all 5"));
  DEBUG_SERIAL.println(F("2) 목표 signature 안정도"));
  DEBUG_SERIAL.println(F("pixy watch 1 30"));
  DEBUG_SERIAL.println(F("pixy fps"));
  DEBUG_SERIAL.println(F("3) 자동 밝기 후보 탐색"));
  DEBUG_SERIAL.println(F("pixy sweep all 4 40 140 20 12 80"));
  DEBUG_SERIAL.println(F("4) 적재함 slot x/y 판정 확인"));
  DEBUG_SERIAL.println(F("pixy storage lower 10 0"));
  DEBUG_SERIAL.println(F("pixy storage all 10 80"));
  DEBUG_SERIAL.println(F("5) 그립 목표창까지 이동 시간 확인"));
  DEBUG_SERIAL.println(F("pose run 3 300"));
  DEBUG_SERIAL.println(F("pixy align lower 5000 0"));
  DEBUG_SERIAL.println(F("6) 추천 밝기로 다시 안정도 확인"));
  DEBUG_SERIAL.println(F("pixy watch all 30"));
  DEBUG_SERIAL.println(F("판정: 인식률 80% 미만이면 PixyMon 재학습/배경 차폐/조명 고정을 먼저 보세요."));
  DEBUG_SERIAL.println(F("주의: sweep은 brightness 추천만 합니다. 나쁜 signature 학습 자체를 고치지는 못합니다."));
  printLine();
}

void commandGuideGrip() {
  printGuideHeader(F("[가이드] 집게 확인"));
  DEBUG_SERIAL.println(F("1) 기본 열기/닫기 반복"));
  DEBUG_SERIAL.println(F("grip cycle 2 500"));
  DEBUG_SERIAL.println(F("2) 후보값 테스트"));
  DEBUG_SERIAL.println(F("grip test 100 650 2 500"));
  DEBUG_SERIAL.println(F("3) 직접 위치 확인"));
  DEBUG_SERIAL.println(F("grip set 600"));
  DEBUG_SERIAL.println(F("grip open"));
  DEBUG_SERIAL.println(F("주의: 실제로 잡혔는지 센서 피드백은 없으므로 눈으로 확인해야 합니다."));
  printLine();
}

void commandGuideDrive() {
  printGuideHeader(F("[가이드] 후진 휨 보정"));
  DEBUG_SERIAL.println(F("1) 짧은 기본 후진"));
  DEBUG_SERIAL.println(F("profile safe"));
  DEBUG_SERIAL.println(F("drive back 100 80"));
  DEBUG_SERIAL.println(F("drive back 200 100"));
  DEBUG_SERIAL.println(F("2) 좌/우 보정 비교"));
  DEBUG_SERIAL.println(F("drive trim 후진 200 100 1.00 0.95"));
  DEBUG_SERIAL.println(F("drive trim 후진 200 100 0.95 1.00"));
  DEBUG_SERIAL.println(F("3) PSD 정렬/접근처럼 velocity-mode가 휘면"));
  DEBUG_SERIAL.println(F("drive balance set 1.00 0.95"));
  DEBUG_SERIAL.println(F("drive balance test forward 120 1000"));
  DEBUG_SERIAL.println(F("drive balance status"));
  DEBUG_SERIAL.println(F("좋아지는 쪽을 기록한 뒤, Motor 반영은 별도 단계에서 합니다."));
  printLine();
}

void commandGuidePlace() {
  printGuideHeader(F("[가이드] 미션수행존 1~8칸 배치 자세 확인"));
  DEBUG_SERIAL.println(F("1) 자세 누락 검사"));
  DEBUG_SERIAL.println(F("pose verify"));
  DEBUG_SERIAL.println(F("2) 그리퍼 동작 없이 1~8칸 pose 확인"));
  DEBUG_SERIAL.println(F("seq placeall"));
  DEBUG_SERIAL.println(F("3) 특정 칸만 실제 배치 흐름 확인"));
  DEBUG_SERIAL.println(F("seq place 1"));
  DEBUG_SERIAL.println(F("seq place 8"));
  DEBUG_SERIAL.println(F("어긋나는 칸은 pose id 7~14를 pose tune으로 조정하세요."));
  printLine();
}

void commandGuideRace() {
  printGuideHeader(F("[가이드] 경기 직전 체크"));
  DEBUG_SERIAL.println(F("1) 안전/자세"));
  DEBUG_SERIAL.println(F("status"));
  DEBUG_SERIAL.println(F("profile normal"));
  DEBUG_SERIAL.println(F("speed status"));
  DEBUG_SERIAL.println(F("pose verify"));
  DEBUG_SERIAL.println(F("pose backup"));
  DEBUG_SERIAL.println(F("2) Pixy"));
  DEBUG_SERIAL.println(F("pixy lamp off"));
  DEBUG_SERIAL.println(F("pixy watch 1 30"));
  DEBUG_SERIAL.println(F("pixy watch 2 30"));
  DEBUG_SERIAL.println(F("pixy watch 3 30"));
  DEBUG_SERIAL.println(F("3) 집게/배치"));
  DEBUG_SERIAL.println(F("grip cycle 1 500"));
  DEBUG_SERIAL.println(F("seq storage"));
  DEBUG_SERIAL.println(F("seq placeall"));
  DEBUG_SERIAL.println(F("4) 주행 짧은 확인"));
  DEBUG_SERIAL.println(F("speed fast"));
  DEBUG_SERIAL.println(F("drive back 100 80"));
  DEBUG_SERIAL.println();
  DEBUG_SERIAL.println(F("미션 흐름을 한 단계씩 돌며 확인하려면:"));
  DEBUG_SERIAL.println(F("mission start"));
  DEBUG_SERIAL.println(F("mission approach 15 80  # 최초 장애물 감지 후 짧게 이어서 전진"));
  DEBUG_SERIAL.println(F("mission instruction 640 500 120  # SL 정렬 후 선을 밟으며 주행 중 Pixy scan"));
  DEBUG_SERIAL.println(F("mission scanrate 500 10  # 미션지시존 signature scan 빠르게"));
  DEBUG_SERIAL.println(F("mission scanrate 200 10  # 거의 멈추지 않는 빠른 후보"));
  DEBUG_SERIAL.println(F("SW1              # 미션지시존 도착 후 정지"));
  DEBUG_SERIAL.println(F("SW1              # Pixy scan 후 정지, 확인 후 mission accept"));
  DEBUG_SERIAL.println(F("SW1              # 적재함 기준 위치 접근/정렬"));
  DEBUG_SERIAL.println(F("mission storagegate 500 550 150 2 2000  # SL 이탈 후 2초 전진, 550 재감지 시 정렬"));
  DEBUG_SERIAL.println(F("SW1              # 1/5, 2/6 순서로 열 이동 또는 columnscan"));
  DEBUG_SERIAL.println(F("mission columnstep 72 150  # 한 칸 이동량/속도 조정"));
  DEBUG_SERIAL.println(F("mission columnscanrate 220 10 25  # 열 도착 후 충분히 보고 다음 열로 이동"));
  DEBUG_SERIAL.println(F("mission columnright       # columnstep 기준 한 칸 테스트"));
  DEBUG_SERIAL.println(F("mission jog right 20 80   # 정위치에서 임의 보정 이동"));
  DEBUG_SERIAL.println(F("필요할 때만 mission upper/lower, mission slot <1~8>, mission column <1~4>로 수동 변경"));
  DEBUG_SERIAL.println();
  DEBUG_SERIAL.println(F("미션지시존에서 멈추지 않고 적재함까지 묶어 보려면:"));
  DEBUG_SERIAL.println(F("mission start quick"));
  DEBUG_SERIAL.println(F("또는 mission run storage"));
  DEBUG_SERIAL.println(F("모두 안정적이면 Motor.ino 미션 업로드/실행으로 넘어갑니다."));
  printLine();
}

void commandGuideLimits() {
  printGuideHeader(F("[가이드] 제약/주의/아직 어려운 점"));
  DEBUG_SERIAL.println(F("절대 건드리지 않는 것:"));
  DEBUG_SERIAL.println(F("  - Pins.h 핀 정의"));
  DEBUG_SERIAL.println(F("  - 모바일베이스/팔 Dynamixel ID"));
  DEBUG_SERIAL.println(F("  - Motor.ino 미션 본체"));
  DEBUG_SERIAL.println();
  DEBUG_SERIAL.println(F("Pixy 제한:"));
  DEBUG_SERIAL.println(F("  - signature 자체 학습은 PixyMon에서 합니다."));
  DEBUG_SERIAL.println(F("  - Arduino는 brightness/lamp/sigmap/안정도만 조정합니다."));
  DEBUG_SERIAL.println(F("  - PRM은 경기 전 최종 1개를 Load하고, 경기 중 교체하지 않습니다."));
  DEBUG_SERIAL.println(F("  - pixy sweep은 추천값이며, 배경/반사/나쁜 학습은 별도로 해결해야 합니다."));
  DEBUG_SERIAL.println();
  DEBUG_SERIAL.println(F("집게/자세 제한:"));
  DEBUG_SERIAL.println(F("  - 집게는 실제 잡힘 센서가 없어서 눈으로 확인해야 합니다."));
  DEBUG_SERIAL.println(F("  - EEPROM 자세가 틀리면 코드가 맞아도 동작은 실패합니다."));
  DEBUG_SERIAL.println(F("  - pose apply/save/restore 전에는 pose backup을 먼저 남기세요."));
  DEBUG_SERIAL.println();
  DEBUG_SERIAL.println(F("주행 제한:"));
  DEBUG_SERIAL.println(F("  - drive trim은 후보 테스트이며 Motor 설정에 자동 반영하지 않습니다."));
  DEBUG_SERIAL.println(F("  - drive balance는 튜너 실행 중 velocity-mode에만 적용됩니다."));
  DEBUG_SERIAL.println(F("  - 후진 휨은 바닥/하중/바퀴 장착/배터리 영향을 받습니다."));
  DEBUG_SERIAL.println(F("  - 빠른 테스트는 빈 공간 또는 바퀴를 띄운 상태에서만 합니다."));
  DEBUG_SERIAL.println();
  DEBUG_SERIAL.println(F("다음 발전 방향:"));
  DEBUG_SERIAL.println(F("  - sweep/watch 결과로 장소별 brightness를 MissionConfig에 반영"));
  DEBUG_SERIAL.println(F("  - Motor.ino에 여러 프레임 안정 판정 후 집기 로직 추가"));
  DEBUG_SERIAL.println(F("  - 적재함 4x2 x/y 위치 검증으로 오인식 방지"));
  printLine();
}

void commandGuideImprove() {
  printGuideHeader(F("[가이드] 테스트 후 반영 기준"));
  DEBUG_SERIAL.println(F("1) 먼저 기록"));
  DEBUG_SERIAL.println(F("pose backup"));
  DEBUG_SERIAL.println(F("pixy watch all 30"));
  DEBUG_SERIAL.println(F("필요하면: pixy sweep all 4 40 140 20 12 80"));
  DEBUG_SERIAL.println();
  DEBUG_SERIAL.println(F("2) 튜너에서만 반복할 것"));
  DEBUG_SERIAL.println(F("  - pose 높이/각도 후보"));
  DEBUG_SERIAL.println(F("  - grip open/close 후보"));
  DEBUG_SERIAL.println(F("  - drive trim 후보"));
  DEBUG_SERIAL.println(F("  - Pixy brightness/lamp 후보"));
  DEBUG_SERIAL.println();
  DEBUG_SERIAL.println(F("3) Motor에 반영할 조건"));
  DEBUG_SERIAL.println(F("  - 같은 조건에서 3회 이상 같은 결과"));
  DEBUG_SERIAL.println(F("  - 실패 원인이 Pixy/자세/집게/주행 중 하나로 분리됨"));
  DEBUG_SERIAL.println(F("  - 기존 성공 루트를 깨지 않는 작은 변경"));
  DEBUG_SERIAL.println();
  DEBUG_SERIAL.println(F("4) 아직 반영하지 말 것"));
  DEBUG_SERIAL.println(F("  - 한 번만 성공한 빠른 값"));
  DEBUG_SERIAL.println(F("  - 조도/배경이 불안정한 상태의 signature 변경"));
  DEBUG_SERIAL.println(F("  - 핀/ID/방향 같은 하드웨어 정의 변경"));
  printLine();
}

void commandGuide(const String &input) {
  String topic = tokenAt(input, 1);
  topic.toLowerCase();

  if (topic.length() == 0 || topic == "목록" || topic == "menu") {
    commandGuideMain();
  } else {
    DEBUG_SERIAL.println(F("[guide] guide는 현장 최소 명령만 보여줍니다."));
    DEBUG_SERIAL.println(F("        자세한 내용은 help pose|pixy|drive|seq|speed|advanced를 사용하세요."));
    commandGuideMain();
  }
}

void commandHelpMain() {
  printLine();
  DEBUG_SERIAL.println(F("MissionRouteTuner 기본 명령"));
  DEBUG_SERIAL.println(F("  guide                         : 현장용 최소 명령"));
  DEBUG_SERIAL.println(F("  mission start                 : SW1 단계 진행 시작"));
  DEBUG_SERIAL.println(F("  mission start quick           : 지시존 접근+주행 중 스캔+적재함 접근 묶음 실행"));
  DEBUG_SERIAL.println(F("  mission run storage           : mission start quick 별칭"));
  DEBUG_SERIAL.println(F("  mission next                  : 현재 SW1 단계 Serial 실행"));
  DEBUG_SERIAL.println(F("  mission rescan                : 미션지시존 Pixy 재스캔"));
  DEBUG_SERIAL.println(F("  mission accept                : scan queue 수락 후 적재함 이동 준비"));
  DEBUG_SERIAL.println(F("  mission survey | mission plan : 적재함 전체 스캔 후 sourceSlot 기반 queue 적용"));
  DEBUG_SERIAL.println(F("  mission column <1~4>          : 현재 적재함 열 기준 지정"));
  DEBUG_SERIAL.println(F("  mission columnstep <mm> <mm/s>: 열 이동량/속도 테스트"));
  DEBUG_SERIAL.println(F("  mission columnscanrate <settle> <frames> <sample>: 열 도착 후 스캔 시간"));
  DEBUG_SERIAL.println(F("  mission columnpsd [c] [n] [ms]: 열별 PSD 평균 JSON 출력"));
  DEBUG_SERIAL.println(F("  mission columnscan            : 현재 열 Pixy 스캔/판정 저장"));
  DEBUG_SERIAL.println(F("  mission undo                  : 가능한 마지막 고정 거리 이동만 반대 실행"));
  DEBUG_SERIAL.println(F("  mission finish                : finishalign 후 후진 finish"));
  DEBUG_SERIAL.println(F("  psd status|watch [ms]|off     : PSD 네 센서값 확인/토글"));
  DEBUG_SERIAL.println(F("  cal status|save|load          : 튜너 보정값 전용 EEPROM 저장/불러오기"));
  DEBUG_SERIAL.println(F("  status | stop | !"));
  DEBUG_SERIAL.println(F("  pose verify|backup|present"));
  DEBUG_SERIAL.println(F("  pixy scan all 5 | pixy storage lower 10 0"));
  DEBUG_SERIAL.println(F("  drive balance status|set|test  : velocity-mode 바퀴 속도 보정"));
  DEBUG_SERIAL.println(F("  speed status | speed set front|slow|psd|depth|place|position <value>"));
  DEBUG_SERIAL.println();
  DEBUG_SERIAL.println(F("상세 도움말: help advanced 또는 help pose|pixy|drive|seq|speed|psd|cal"));
  printLine();
}

void commandHelpAdvanced() {
  printLine();
  DEBUG_SERIAL.println(F("고급/보조 명령"));
  DEBUG_SERIAL.println(F("  mission button on|off         : SW1 단계 진행 수동 토글"));
  DEBUG_SERIAL.println(F("  mission upper|lower           : 현재 블록 집기 층 수동 지정"));
  DEBUG_SERIAL.println(F("  mission slot <1~8>            : 현재 블록 배치 칸 수동 지정"));
  DEBUG_SERIAL.println(F("  mission block next            : 숨김 fallback, 보통 사용하지 않음"));
  DEBUG_SERIAL.println(F("  mission goto <stage>          : start|scan|storage|column|pick|place|realign|finish"));
  DEBUG_SERIAL.println(F("  mission align <sl> <fl> <fr> [tol]: 적재함 스캔 정렬 target 임시 변경"));
  DEBUG_SERIAL.println(F("  mission gripalign upper|lower <fl> <fr> [tol]: 집기 직전 FL/FR 깊이 target 변경"));
  DEBUG_SERIAL.println(F("  mission placealign <sl> <fr> [tol]: 미션수행존 배치 SL+FR target 임시 변경"));
  DEBUG_SERIAL.println(F("  mission finishalign <sl> [tol] [speed]: finish 후진 전 SL target 변경"));
  DEBUG_SERIAL.println(F("  pose run|plan|diff|tuneplan|tune|apply|save|restore|confirm|cancel"));
  DEBUG_SERIAL.println(F("  seq initial|storage|camera|pick upper|pick lower|place <1~8>|placeall"));
  DEBUG_SERIAL.println(F("  pixy sig|watch|align|sweep|brightness|fps|lamp"));
  DEBUG_SERIAL.println(F("  grip open|close|cycle|set|test"));
  DEBUG_SERIAL.println(F("  drive forward|back|left|right <mm> <mm/s>"));
  DEBUG_SERIAL.println(F("  drive trim <방향> <mm> <mm/s> <left> <right>"));
  DEBUG_SERIAL.println(F("  export json                    : EEPROM/config/queue/Pixy snapshot 한 줄 JSON"));
  DEBUG_SERIAL.println(F("  psd status|watch|off|targets   : PSD 네 센서값과 동적 target 확인"));
  DEBUG_SERIAL.println(F("  cal status|save|load|defaults|clear|eeprom"));
  DEBUG_SERIAL.println(F("  profile safe|normal|fast|max"));
  DEBUG_SERIAL.println(F("  replay"));
  printLine();
}

void commandHelpPose() {
  printLine();
  DEBUG_SERIAL.println(F("자세/매니퓰레이터 도움말"));
  DEBUG_SERIAL.println(F("  pose list"));
  DEBUG_SERIAL.println(F("    EEPROM에 저장된 전체 자세를 출력합니다."));
  DEBUG_SERIAL.println(F("  pose backup"));
  DEBUG_SERIAL.println(F("    1~14번 EEPROM 자세를 CSV처럼 출력합니다. 저장 전 먼저 남기세요."));
  DEBUG_SERIAL.println(F("  pose present"));
  DEBUG_SERIAL.println(F("    현재 m1~m4 raw 값을 출력합니다. 자세가 이상할 때 첫 확인 명령입니다."));
  DEBUG_SERIAL.println(F("  pose verify"));
  DEBUG_SERIAL.println(F("    현재 테스트 자세 1,3,4,5,7~14 누락 여부를 검사합니다."));
  DEBUG_SERIAL.println(F("  pose diff <id>"));
  DEBUG_SERIAL.println(F("    현재 자세와 EEPROM id 자세 차이를 dm1~dm4로 보여줍니다."));
  DEBUG_SERIAL.println(F("  pose plan <id> <ms> [m1angle]"));
  DEBUG_SERIAL.println(F("    실제 이동 없이 목표 raw, 변화량, 자동 보정 시간을 미리 봅니다."));
  DEBUG_SERIAL.println(F("  pose run <id> <ms> [m1angle]"));
  DEBUG_SERIAL.println(F("    EEPROM 자세를 실행합니다. ms가 작을수록 빠릅니다."));
  DEBUG_SERIAL.println(F("    m1angle은 하부 회전만 각도로 덮어씁니다. 권장 범위 -100~100도."));
  DEBUG_SERIAL.println(F("  pose tuneplan <id> <ms> <m1> <m2> <m3> <m4>"));
  DEBUG_SERIAL.println(F("    pose tune 전에 목표값과 위험도를 미리 봅니다. 실제 이동하지 않습니다."));
  DEBUG_SERIAL.println(F("  pose tune <id> <ms> <m1> <m2> <m3> <m4>"));
  DEBUG_SERIAL.println(F("    EEPROM 기준값을 읽어 1회 테스트합니다. 바로 저장하지 않습니다."));
  DEBUG_SERIAL.println(F("    +20/-20/0은 기준값 대비 증감, =2048은 절대 raw 목표값입니다."));
  DEBUG_SERIAL.println(F("  pose apply [desc]"));
  DEBUG_SERIAL.println(F("    마지막 pose tune 결과를 저장 후보로 올립니다. EEPROM 쓰기는 pose confirm 때 수행합니다."));
  DEBUG_SERIAL.println(F("  pose save <id> <desc>"));
  DEBUG_SERIAL.println(F("    현재 실제 모터 위치를 저장 후보로 올립니다. EEPROM 쓰기는 pose confirm 때 수행합니다."));
  DEBUG_SERIAL.println(F("  pose restore <id> <m1> <m2> <m3> <m4> [desc]"));
  DEBUG_SERIAL.println(F("    pose backup/스크린샷에 남은 raw 값으로 복구 후보를 만듭니다."));
  DEBUG_SERIAL.println(F("  pose confirm"));
  DEBUG_SERIAL.println(F("    저장 후보를 EEPROM에 실제로 씁니다."));
  DEBUG_SERIAL.println(F("  pose cancel"));
  DEBUG_SERIAL.println(F("    저장 후보를 버립니다."));
  DEBUG_SERIAL.println();
  DEBUG_SERIAL.println(F("미션 pose id:"));
  DEBUG_SERIAL.println(F("  1=초기/미션지시존, 2=예비/수동"));
  DEBUG_SERIAL.println(F("  3=적재함 보기/안전 이동, 4=상층 집기, 5=하층 집기, 6=예비, 7~14=미션수행존 1~8칸"));
  DEBUG_SERIAL.println();
  DEBUG_SERIAL.println(F("모터 의미와 raw 범위:"));
  DEBUG_SERIAL.println(F("  m1: 하부 좌우 회전, raw 0~4095"));
  DEBUG_SERIAL.println(F("  m2: 어깨 링크, raw 1024~3474"));
  DEBUG_SERIAL.println(F("  m3: 팔꿈치 링크, raw 211~2640"));
  DEBUG_SERIAL.println(F("  m4: 손목/말단 링크, raw 722~3161"));
  DEBUG_SERIAL.println(F("  1 raw는 약 0.088도입니다. 실제 높이는 m2~m4 조합으로 바뀝니다."));
  DEBUG_SERIAL.println(F("  높이만 조금 올릴 때도 m2/m3/m4를 20~50 raw 단위로 나누어 확인하세요."));
  DEBUG_SERIAL.println(F("예: pose tune 3 300 0 +30 -30 0"));
  DEBUG_SERIAL.println(F("예: pose tune 7 300 0 0 +20 -20"));
  printLine();
}

void commandHelpPixy() {
  printLine();
  DEBUG_SERIAL.println(F("Pixy 카메라 도움말"));
  DEBUG_SERIAL.println(F("  pixy scan [signatureMap|0b1111111|all] [frames]"));
  DEBUG_SERIAL.println(F("    현재 보이는 블록의 sig, x, y, width, height, area를 출력합니다."));
  DEBUG_SERIAL.println(F("    map 생략 시 signature 1~7 전체를 봅니다."));
  DEBUG_SERIAL.println(F("  pixy sig <1~7> [frames]"));
  DEBUG_SERIAL.println(F("    특정 signature 하나만 확인합니다."));
  DEBUG_SERIAL.println(F("  pixy watch <1~7|all|0b1111111> [frames] [intervalMs]"));
  DEBUG_SERIAL.println(F("    여러 프레임의 인식률, 평균 x/y/area, 흔들림 범위를 계산합니다."));
  DEBUG_SERIAL.println(F("  pixy storage [lower|all|upper] [frames] [minArea]"));
  DEBUG_SERIAL.println(F("    적재함 기준 위치에서 x/y가 upper/lower boundary 안에 들어오는지 출력합니다."));
  DEBUG_SERIAL.println(F("    storagePickupRegion x/y 범위와 margin 보정용입니다."));
  DEBUG_SERIAL.println(F("  pixy alignslow [lower|upper|all] [timeoutMs] [minArea] [signatureMap]"));
  DEBUG_SERIAL.println(F("    Motor와 같은 저속 1~6mm micro-step 방식으로 중심 정렬을 테스트합니다."));
  DEBUG_SERIAL.println(F("  pixy align [lower|upper|all] [timeoutMs] [minArea] [signatureMap]"));
  DEBUG_SERIAL.println(F("    기존 velocity 방식 비교용입니다."));
  DEBUG_SERIAL.println(F("  pixy sweep [1~7|all] [expected] [start] [end] [step] [frames] [minArea]"));
  DEBUG_SERIAL.println(F("    brightness를 훑어서 면적 기준을 넘은 블록 수가 가장 안정적인 값을 추천합니다."));
  DEBUG_SERIAL.println(F("    같은 signature 블록이 여러 개여도 각각 개수로 계산합니다."));
  DEBUG_SERIAL.println(F("    단, signature 학습 자체가 나쁘면 PixyMon에서 다시 조정해야 합니다."));
  DEBUG_SERIAL.println(F("  pixy lamp on|off"));
  DEBUG_SERIAL.println(F("    Pixy LED를 켜거나 끕니다."));
  DEBUG_SERIAL.println(F("  pixy brightness <0~255>"));
  DEBUG_SERIAL.println(F("    Pixy 카메라 노출/밝기 후보를 테스트합니다."));
  DEBUG_SERIAL.println(F("  pixy fps"));
  DEBUG_SERIAL.println(F("    현재 FPS를 읽습니다. 낮으면 조명 부족 가능성이 큽니다."));
  DEBUG_SERIAL.println();
  DEBUG_SERIAL.println(F("판단 기준:"));
  DEBUG_SERIAL.println(F("  count=0이면 색상 학습, 조명, 거리, 차폐, signature 필터를 봅니다."));
  DEBUG_SERIAL.println(F("  area가 작거나 프레임마다 크게 흔들리면 접근 거리/조명/배경을 조정합니다."));
  DEBUG_SERIAL.println(F("  x가 계속 한쪽으로 치우치면 정렬 또는 카메라 장착 각도를 확인합니다."));
  DEBUG_SERIAL.println(F("  Pixy2 CCC signature는 PixyMon에서 학습한 1~7번 범위 안에서만 씁니다."));
  printLine();
}

void commandHelpGrip() {
  printLine();
  DEBUG_SERIAL.println(F("집게 도움말"));
  DEBUG_SERIAL.println(F("  grip open  또는 집게 열기"));
  DEBUG_SERIAL.println(F("  grip close 또는 집게 닫기"));
  DEBUG_SERIAL.println(F("  grip cycle [repeat] [holdMs]"));
  DEBUG_SERIAL.println(F("    집게 열기->닫기 반복 후 다시 열어 둡니다."));
  DEBUG_SERIAL.println(F("  grip set <0~1000>"));
  DEBUG_SERIAL.println(F("    집게 서보 위치를 직접 보냅니다. EEPROM/상수는 바꾸지 않습니다."));
  DEBUG_SERIAL.println(F("  grip test [open close] [repeat] [holdMs]"));
  DEBUG_SERIAL.println(F("    열림/닫힘 후보값을 임시로 반복 테스트합니다."));
  DEBUG_SERIAL.print(F("  현재 열림값="));
  DEBUG_SERIAL.print(GRIP_ANGLE_OPEN);
  DEBUG_SERIAL.print(F(", 닫힘값="));
  DEBUG_SERIAL.println(GRIP_ANGLE_CLOSE);
  DEBUG_SERIAL.println();
  DEBUG_SERIAL.println(F("부족한 점: 현재 그리퍼는 위치 명령만 보내며 실제로 잡혔는지 힘/전류 피드백은 없습니다."));
  DEBUG_SERIAL.println(F("따라서 집기 실패는 Pixy 인식, 접근 자세, 그리퍼 닫힘값을 나눠서 확인해야 합니다."));
  DEBUG_SERIAL.println(F("grip test 결과는 후보값이며 Gripper.h 상수는 자동으로 바뀌지 않습니다."));
  printLine();
}

void commandHelpDrive() {
  printLine();
  DEBUG_SERIAL.println(F("주행 도움말"));
  DEBUG_SERIAL.println(F("  drive forward|back|left|right <mm> <mmPerSec>"));
  DEBUG_SERIAL.println(F("    mm는 이동 거리, mmPerSec는 초당 이동 거리입니다."));
  DEBUG_SERIAL.println(F("    예: drive back 300 120 -> 300mm 후진, 약 2.5초"));
  DEBUG_SERIAL.println(F("    예: drive forward 500 250 -> 500mm 전진, 약 2.0초"));
  DEBUG_SERIAL.println(F("  drive trim <방향> <mm> <mmPerSec> <leftScale> <rightScale>"));
  DEBUG_SERIAL.println(F("    좌/우 바퀴 목표 회전량을 비율로 보정합니다."));
  DEBUG_SERIAL.println(F("    예: drive trim 후진 200 100 1.00 0.95"));
  DEBUG_SERIAL.println(F("  drive trim <방향> <mm> <mmPerSec> <fl> <fr> <bl> <br>"));
  DEBUG_SERIAL.println(F("    개별 바퀴 보정입니다. 꼭 필요할 때만 사용하세요."));
  DEBUG_SERIAL.println(F("  drive balance set <leftScale> <rightScale>"));
  DEBUG_SERIAL.println(F("    velocity-mode 주행/PSD 정렬용 좌/우 속도 보정입니다."));
  DEBUG_SERIAL.println(F("    예: drive balance set 1.00 0.95"));
  DEBUG_SERIAL.println(F("  drive balance test <방향> <raw> <ms> [leftScale rightScale]"));
  DEBUG_SERIAL.println(F("    예: drive balance test forward 120 1000"));
  DEBUG_SERIAL.println();
  DEBUG_SERIAL.println(F("동작 방식:"));
  DEBUG_SERIAL.println(F("  1. 바퀴를 Extended Position Control + Time Based Profile로 전환"));
  DEBUG_SERIAL.println(F("  2. 거리(mm)를 바퀴 raw 회전량으로 변환"));
  DEBUG_SERIAL.println(F("  3. 예상 시간 distance/mmPerSec로 profile time을 설정"));
  DEBUG_SERIAL.println(F("  4. 도착 확인이 늦으면 타임아웃으로 정지"));
  DEBUG_SERIAL.println();
  DEBUG_SERIAL.println(F("후진 때 한쪽으로 휘면 먼저 짧게 테스트하세요:"));
  DEBUG_SERIAL.println(F("  profile safe"));
  DEBUG_SERIAL.println(F("  drive back 100 80"));
  DEBUG_SERIAL.println(F("  drive back 200 100"));
  DEBUG_SERIAL.println(F("  drive trim 후진 200 100 1.00 0.95"));
  DEBUG_SERIAL.println(F("drive trim은 position-mode 후보, drive balance는 velocity-mode 튜너 보정입니다."));
  DEBUG_SERIAL.println(F("둘 다 MissionConfig/Motor에는 자동 반영되지 않습니다."));
  DEBUG_SERIAL.println(F("미션 단계별 기본 속도는 speed status로 확인하고 speed set으로 임시 조정합니다."));
  printTrimHint();
  printLine();
}

void commandHelpSeq() {
  printLine();
  DEBUG_SERIAL.println(F("미션 부분 시퀀스 도움말"));
  DEBUG_SERIAL.println(F("  seq initial        : 1번 초기 자세만 실행"));
  DEBUG_SERIAL.println(F("  seq storage        : 3번 적재함 보기/안전 이동 자세만 실행"));
  DEBUG_SERIAL.println(F("  seq camera         : 3번 적재함 보기/안전 이동 자세만 실행"));
  DEBUG_SERIAL.println(F("  seq pick upper     : 3->4 staged(m4->m3->m2)->그리퍼 닫기->3 staged"));
  DEBUG_SERIAL.println(F("  seq pick lower     : 3->5->그리퍼 닫기->3"));
  DEBUG_SERIAL.println(F("  seq place <1~8>    : 3->미션수행존 칸 pose(7~14)->그리퍼 열기->3"));
  DEBUG_SERIAL.println(F("  seq placeall [start] [end]"));
  DEBUG_SERIAL.println(F("    미션수행존 pose 7~14를 연속 확인합니다. 그리퍼는 열거나 닫지 않습니다."));
  DEBUG_SERIAL.println();
  DEBUG_SERIAL.println(F("목적: Motor.ino 전체 미션을 돌리지 않고, 집기/배치 한 단계만 반복 검증합니다."));
  DEBUG_SERIAL.println(F("배치 칸 높이가 어긋나면 해당 칸 pose id(7~14)를 pose tune으로 조정하세요."));
  printLine();
}

void commandHelpProfile() {
  printLine();
  DEBUG_SERIAL.println(F("profile 도움말"));
  printProfileTable();
  DEBUG_SERIAL.println();
  DEBUG_SERIAL.println(F("safe   : 첫 연결, 바퀴/팔 방향 확인, 작은 높이 조정"));
  DEBUG_SERIAL.println(F("normal : 일반 캘리브레이션 기본값"));
  DEBUG_SERIAL.println(F("fast   : 반복 테스트 시간을 줄일 때"));
  DEBUG_SERIAL.println(F("max    : 속도 한도를 가장 크게 푼 모드. 바퀴를 띄우거나 빈 경기장에서만 사용"));
  DEBUG_SERIAL.println(F("profile은 주행 거리/속도/시간 한도와 미션 주행 속도 preset만 바꿉니다."));
  printLine();
}

void commandHelpSpeed() {
  printMissionSpeedStatus();
  DEBUG_SERIAL.println(F("권장 순서:"));
  DEBUG_SERIAL.println(F("  1. speed status로 현재 값 확인"));
  DEBUG_SERIAL.println(F("  2. 답답하면 profile fast 또는 speed fast"));
  DEBUG_SERIAL.println(F("  3. 한 단계만 더 올릴 때 speed set position 220, speed set psd 260처럼 부분 조정"));
  DEBUG_SERIAL.println(F("  4. 흔들리거나 오버슈트가 생기면 speed reset 또는 profile normal"));
}

void commandHelpPsd() {
  printLine();
  DEBUG_SERIAL.println(F("PSD 센서 도움말"));
  DEBUG_SERIAL.println(F("  psd status"));
  DEBUG_SERIAL.println(F("    FL/FR/SL/SR 현재값과 미션 지시존/적재함 정렬 target을 1회 출력합니다."));
  DEBUG_SERIAL.println(F("  psd watch [intervalMs]"));
  DEBUG_SERIAL.println(F("    비차단 토글 모드로 센서값을 계속 출력합니다. 예: psd watch 250"));
  DEBUG_SERIAL.println(F("  psd off"));
  DEBUG_SERIAL.println(F("    watch 출력 중지."));
  DEBUG_SERIAL.println(F("  psd targets"));
  DEBUG_SERIAL.println(F("    instruction/scan align/grip align 기준과 현재 센서값을 함께 출력합니다."));
  DEBUG_SERIAL.println();
  DEBUG_SERIAL.println(F("정렬 주의: 적재함 스캔 정렬은 SL+FL/FR, 집기 깊이는 FL/FR만 씁니다. SR은 값 확인용입니다."));
  printLine();
}

void commandHelpCalibration() {
  printLine();
  DEBUG_SERIAL.println(F("튜너 캘리브레이션 EEPROM 도움말"));
  DEBUG_SERIAL.println(F("  cal status"));
  DEBUG_SERIAL.println(F("    현재 동적 튜닝값과 저장 record 유효성을 확인합니다."));
  DEBUG_SERIAL.println(F("  cal save"));
  DEBUG_SERIAL.println(F("    pose EEPROM 뒤쪽 튜너 전용 영역에 주행/PSD/정렬 값을 저장합니다."));
  DEBUG_SERIAL.println(F("  cal load"));
  DEBUG_SERIAL.println(F("    저장된 튜너 값을 현재 세션에 적용합니다."));
  DEBUG_SERIAL.println(F("  cal defaults"));
  DEBUG_SERIAL.println(F("    EEPROM에 쓰지 않고 MissionConfig 기본값/profile preset으로 되돌립니다."));
  DEBUG_SERIAL.println(F("  cal clear"));
  DEBUG_SERIAL.println(F("    튜너 record magic만 삭제합니다. pose EEPROM은 건드리지 않습니다."));
  DEBUG_SERIAL.println(F("  cal eeprom"));
  DEBUG_SERIAL.println(F("    pose 영역과 튜너 영역 주소를 출력합니다."));
  DEBUG_SERIAL.println();
  DEBUG_SERIAL.println(F("pose 저장은 기존 pose save/apply -> pose confirm 흐름만 사용합니다."));
  printLine();
}

void commandHelpMath() {
  printLine();
  DEBUG_SERIAL.println(F("계산식 도움말"));
  DEBUG_SERIAL.println(F("자세 속도:"));
  DEBUG_SERIAL.println(F("  pose run: maxRawDelta = max(|목표 m1~m4 - 현재 m1~m4|)"));
  DEBUG_SERIAL.println(F("  pose tune: stepMaxRawDelta = max(|목표 m1~m4 - 현재 m1~m4|)"));
  DEBUG_SERIAL.println(F("             eepromMaxRawDelta는 저장 전 확인용으로만 출력합니다."));
  DEBUG_SERIAL.println(F("  safeMs = min(profile.maxPoseMs, max(요청ms, profile.minPoseMs + maxRawDelta * msPerRawTick))"));
  DEBUG_SERIAL.println(F("  지금 profile.maxPoseMs는 500ms라서 큰 raw 변화도 500ms 안에서 실행합니다."));
  DEBUG_SERIAL.println();
  DEBUG_SERIAL.println(F("Dynamixel raw:"));
  DEBUG_SERIAL.println(F("  4096 raw = 360도, 1 raw ~= 0.088도"));
  DEBUG_SERIAL.println();
  DEBUG_SERIAL.println(F("주행 거리 변환:"));
  DEBUG_SERIAL.println(F("  wheelRaw = distanceMm / 30mm * 4095 / (2*pi)"));
  DEBUG_SERIAL.println(F("  대략 1mm ~= 21.7 raw, 100mm ~= 2170 raw"));
  DEBUG_SERIAL.println(F("  driveMs = distanceMm * 1000 / mmPerSec"));
  DEBUG_SERIAL.println();
  DEBUG_SERIAL.println(F("옴니휠 X/Y 모델(향후 슬롯 좌표 보정에 사용 가능):"));
  DEBUG_SERIAL.println(F("  k = 4095 / (2*pi*30)"));
  DEBUG_SERIAL.println(F("  [fl,fr,bl,br] = k * [y+x, y-x, y-x, y+x]"));
  DEBUG_SERIAL.println(F("  x=오른쪽 mm, y=전진 mm입니다."));
  printLine();
}

void commandHelpRisk() {
  printLine();
  DEBUG_SERIAL.println(F("위험 기준 도움말"));
  DEBUG_SERIAL.println(F("  stop은 모바일베이스 속도를 0으로 합니다. 팔 torque는 떨어짐 방지를 위해 끄지 않습니다."));
  DEBUG_SERIAL.println(F("  동작 중 ! 를 보내면 모바일베이스를 멈추고 팔은 현재 위치로 hold 합니다."));
  DEBUG_SERIAL.println(F("  pose tune은 저장 전 테스트입니다. EEPROM 변경은 pose confirm 때만 발생합니다."));
  DEBUG_SERIAL.println(F("  큰 raw 변화 + 짧은 ms가 가장 위험합니다. 먼저 safe/normal에서 20~50 raw씩 확인하세요."));
  DEBUG_SERIAL.println(F("  profile max는 빠른 주행 확인용입니다. 블록 근처, 팔이 낮은 상태, 좁은 구간에서는 쓰지 마세요."));
  DEBUG_SERIAL.println(F("  drive는 한 번에 최대 5초 예상 주행까지만 허용합니다."));
  DEBUG_SERIAL.println(F("  replay는 편하지만 같은 충돌을 반복할 수 있으니 손은 항상 전원/정지 근처에 두세요."));
  DEBUG_SERIAL.println(F("  현재 테스트 자세 1,3,4,5,7~14가 비어 있으면 실행하지 않고 중단합니다."));
  DEBUG_SERIAL.println(F("  핀, 모터 ID, PRM 교체, Pixy signature 재학습은 이 튜너에서 하지 않습니다."));
  printLine();
}

void commandHelp(const String &input) {
  String topic = tokenAt(input, 1);
  topic.toLowerCase();

  if (topic.length() == 0 || topic == "main" || topic == "기본") {
    commandHelpMain();
  } else if (topic == "advanced" || topic == "고급" || topic == "all" || topic == "전체") {
    commandHelpAdvanced();
  } else if (topic == "pose" || topic == "자세" || topic == "arm") {
    commandHelpPose();
  } else if (topic == "drive" || topic == "주행" || topic == "mobile") {
    commandHelpDrive();
  } else if (topic == "seq" || topic == "sequence" || topic == "시퀀스") {
    commandHelpSeq();
  } else if (topic == "pixy" || topic == "픽시" || topic == "camera" || topic == "카메라") {
    commandHelpPixy();
  } else if (topic == "grip" || topic == "gripper" || topic == "집게") {
    commandHelpGrip();
  } else if (topic == "profile" || topic == "프로파일") {
    commandHelpProfile();
  } else if (topic == "speed" || topic == "속도") {
    commandHelpSpeed();
  } else if (topic == "psd" || topic == "sensor" || topic == "센서") {
    commandHelpPsd();
  } else if (topic == "cal" || topic == "calibration" || topic == "캘리브레이션" || topic == "보정") {
    commandHelpCalibration();
  } else if (topic == "math" || topic == "수학" || topic == "계산") {
    commandHelpMath();
  } else if (topic == "risk" || topic == "위험" || topic == "안전") {
    commandHelpRisk();
  } else {
    DEBUG_SERIAL.println(F("[오류] help advanced|pose|drive|seq|pixy|grip|profile|speed|psd|cal|math|risk 중 하나를 입력하세요."));
  }
}

void commandHelp() {
  commandHelp(String("help"));
}

bool isReplayable(const String &input) {
  String first = tokenAt(input, 0);
  first.toLowerCase();
  if (first == "pose" || first == "자세") {
    String second = tokenAt(input, 1);
    second.toLowerCase();
    return second == "run" || second == "실행" ||
           second == "tune" || second == "조정" ||
           second == "save" || second == "저장";
  }
  return first == "seq" || first == "시퀀스" ||
         first == "drive" || first == "주행" ||
         first == "delaytest" || first == "딜레이테스트" ||
         first == "grip" || first == "gripper" || first == "집게" ||
         first == "pixy" || first == "픽시";
}

bool handleCommand(String input, bool remember) {
  input.trim();
  if (input.length() == 0) return false;

  String first = tokenAt(input, 0);
  String second = tokenAt(input, 1);
  first.toLowerCase();
  second.toLowerCase();

  bool ok = false;
  if (first == "guide" || first == "가이드" || first == "절차") {
    commandGuide(input);
    return true;
  } else if (first == "mission" || first == "미션") {
    return commandMission(input);
  } else if (first == "help" || first == "도움말") {
    commandHelp(input);
    return true;
  } else if (first == "status" || first == "상태") {
    commandStatus();
    return true;
  } else if (first == "psd" || first == "sensor" || first == "센서") {
    return commandPsd(input);
  } else if (first == "cal" || first == "calibration" || first == "캘리브레이션" ||
             first == "보정") {
    return commandCalibration(input);
  } else if (first == "export" || first == "json" || first == "내보내기") {
    ok = (first == "json") ? commandExport(String("export json")) : commandExport(input);
  } else if (first == "profile" || first == "프로파일") {
    commandProfile(input);
    return true;
  } else if (first == "speed" || first == "속도") {
    return commandSpeed(input);
  } else if (first == "stop" || first == "정지") {
    stopAll(F("[사용자 정지]"));
    return true;
  } else if (first == "replay") {
    if (lastReplayableCommand.length() == 0) {
      DEBUG_SERIAL.println(F("[replay 불가] 아직 반복할 명령이 없습니다."));
      return false;
    }
    DEBUG_SERIAL.print(F("[replay] "));
    DEBUG_SERIAL.println(lastReplayableCommand);
    return handleCommand(lastReplayableCommand, false);
  } else if (first == "pose" || first == "자세") {
    if (second == "list" || second == "목록") {
      commandPoseList();
      return true;
    } else if (second == "backup" || second == "백업") {
      commandPoseBackup();
      return true;
    } else if (second == "present" || second == "현재") {
      ok = commandPosePresent();
    } else if (second == "verify" || second == "검증") {
      ok = commandPoseVerify();
    } else if (second == "diff" || second == "차이") {
      ok = commandPoseDiff(input);
    } else if (second == "plan" || second == "계획") {
      ok = commandPosePlan(input);
    } else if (second == "run" || second == "실행") {
      ok = commandPoseRun(input);
    } else if (second == "tuneplan" || second == "조정계획") {
      ok = commandPoseTunePlan(input);
    } else if (second == "tune" || second == "조정") {
      ok = commandPoseTune(input);
    } else if (second == "apply" || second == "적용") {
      ok = commandPoseApply(input);
    } else if (second == "save" || second == "저장") {
      ok = commandPoseSave(input);
    } else if (second == "restore" || second == "복구") {
      ok = commandPoseRestore(input);
    } else if (second == "confirm" || second == "확정") {
      ok = commandPoseConfirm();
    } else if (second == "cancel" || second == "취소") {
      ok = commandPoseCancel();
    } else {
      DEBUG_SERIAL.println(F("pose list|backup|present|verify|diff|plan|run|tuneplan|tune|apply|save|restore|confirm|cancel"));
      return false;
    }
  } else if (first == "seq" || first == "시퀀스") {
    ok = commandSeq(input);
  } else if (first == "drive" || first == "주행") {
    ok = commandDrive(input);
  } else if (first == "delaytest" || first == "딜레이테스트") {
    ok = commandDelayTest(input);
  } else if (first == "pixy" || first == "픽시" || first == "camera" || first == "카메라") {
    ok = commandPixy(input);
  } else if (first == "grip" || first == "gripper" || first == "집게") {
    if (!pixyReady) {
      DEBUG_SERIAL.println(F("[주의] Pixy init이 실패했을 수 있습니다. 그래도 명령을 보냅니다."));
    }
    if (second == "open" || second == "열기") {
      OpenGripper(pixy);
      DEBUG_SERIAL.println(F("[그리퍼] 열기"));
      ok = true;
    } else if (second == "close" || second == "닫기") {
      CloseGripper(pixy);
      DEBUG_SERIAL.println(F("[그리퍼] 닫기"));
      ok = true;
    } else if (second == "set" || second == "위치") {
      ok = commandGripSet(input);
    } else if (second == "test" || second == "각도테스트") {
      ok = commandGripTest(input);
    } else if (second == "cycle" || second == "test" || second == "테스트" || second == "반복") {
      ok = commandGripCycle(input);
    } else {
      DEBUG_SERIAL.println(F("사용법: grip open|close|cycle|set|test"));
      return false;
    }
  } else {
    DEBUG_SERIAL.print(F("[오류] 알 수 없는 명령: "));
    DEBUG_SERIAL.println(input);
    DEBUG_SERIAL.println(F("help를 입력하세요."));
    return false;
  }

  if (ok && remember && isReplayable(input)) {
    lastReplayableCommand = input;
  }
  return ok;
}

void flushDebugSerialData() {
  while (DEBUG_SERIAL.available()) DEBUG_SERIAL.read();
}

void clearReceiveBuffer() {
  memset(receiveBuffer, 0, RECEIVE_BUFFER_SIZE);
  receiveBufferIdx = 0;
}

void resetReceiveBuffer() {
  clearReceiveBuffer();
  flushDebugSerialData();
}

void handleMissionButtonMode() {
  if (!missionButtonMode) return;

  bool pressed = digitalRead(SW1_PIN) == LOW;
  if (pressed != missionButtonWasPressed) {
    missionButtonLastChangeMs = millis();
    missionButtonWasPressed = pressed;
  }

  if (!pressed) return;
  if (millis() - missionButtonLastChangeMs < 80) return;

  missionButtonWasPressed = true;
  missionButtonLastChangeMs = millis() + 500;
  DEBUG_SERIAL.println();
  DEBUG_SERIAL.println(F("[mission button] SW1 -> mission step"));
  commandMissionNext();
  DEBUG_SERIAL.print(F("> "));

  while (digitalRead(SW1_PIN) == LOW) {
    delay(10);
  }
  missionButtonWasPressed = false;
  missionButtonLastChangeMs = millis();
}

void setup() {
  initDebug();
  applyMissionSpeedPresetForProfile();
  initMotorCommunication();
  initPSDInputs();
  initRGBLED();
  pinMode(SW1_PIN, INPUT_PULLUP);
  setRGBLEDBlue();

  DEBUG_SERIAL.println();
  DEBUG_SERIAL.println(F("========================================"));
  DEBUG_SERIAL.println(F(" MissionRouteTuner"));
  DEBUG_SERIAL.println(F(" USB 115200 primary / Bluetooth Serial2 9600 input-only / Newline"));
  DEBUG_SERIAL.println(F("========================================"));
  DEBUG_SERIAL.println(F("초기화 중입니다. 자동 이동은 하지 않습니다."));
  if (loadTunerCalibrationFromEEPROM(false)) {
    DEBUG_SERIAL.print(F("[cal load] EEPROM "));
    DEBUG_SERIAL.print(TUNER_CAL_EEPROM_ADDR);
    DEBUG_SERIAL.println(F("번 이후의 튜너 보정값을 자동 적용했습니다."));
  } else {
    DEBUG_SERIAL.println(F("[cal] 저장된 튜너 보정값 없음. MissionConfig 기본 튜닝값으로 시작합니다."));
  }

  manipulatorReady = initManipulator();
  if (manipulatorReady) {
    DEBUG_SERIAL.println(F("[OK] 매니퓰레이터 초기화"));
  } else {
    DEBUG_SERIAL.println(F("[FAIL] 매니퓰레이터 초기화"));
  }

  mobileReady = InitMobilebase(dxl);
  if (mobileReady) {
    ChangeMobilebaseMode2VelocityControlMode(dxl);
    stopMobilebase();
    DEBUG_SERIAL.println(F("[OK] 모바일베이스 초기화"));
  } else {
    DEBUG_SERIAL.println(F("[FAIL] 모바일베이스 초기화"));
  }

  InitPixy(pixy);
  pixyReady = true;
  OpenGripper(pixy);
  setRGBLEDOff();

  commandStatus();
  DEBUG_SERIAL.println(F("[READY] 명령 대기 중입니다."));
  DEBUG_SERIAL.println(F("USB Serial을 기준으로 확인하세요. Bluetooth는 입력 보조 경로로만 둡니다."));
  DEBUG_SERIAL.println(F("현장 단계 확인은 mission start 후 SW1, 전체 도움말은 guide 또는 help를 입력하세요."));
  DEBUG_SERIAL.print(F("> "));
}

void loop() {
  while (DEBUG_SERIAL.available()) {
    char received = DEBUG_SERIAL.read();

    if (received == '\n' || received == '\r') {
      if (receiveBufferIdx == 0) {
        continue;
      }
      String input = String(receiveBuffer);
      clearReceiveBuffer();
      input.trim();
      handleCommand(input, true);
      DEBUG_SERIAL.print(F("> "));
      return;
    }

    if (received < 32 || received == 127) {
      continue;
    }

    if (receiveBufferIdx >= RECEIVE_BUFFER_SIZE - 1) {
      DEBUG_SERIAL.println(F("[입력 오류] 명령이 너무 깁니다. 짧게 입력하세요."));
      resetReceiveBuffer();
      DEBUG_SERIAL.print(F("> "));
      return;
    }
    receiveBuffer[receiveBufferIdx++] = received;
  }
  handleMissionButtonMode();
  updatePsdWatch();
}
