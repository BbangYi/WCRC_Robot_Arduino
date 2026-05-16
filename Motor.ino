/*
 * Motor.ino  (v2 - 15차시 레퍼런스 기반 전면 재설계)
 * MobileManipulator Blackberry - WCRC 물류로봇2-분류 일반부
 *
 * 미션 흐름 (6단계):
 *   1단계: 초기화 & 버튼 대기 (LED 점등)
 *   2단계: 장애물 접근 & 미션지시존 이동
 *   3단계: 블록 색상/순서 스캔
 *   4단계: 적재함 앞 접근 & 정밀 정렬
 *   5단계: 적재함 스캔 -> 집기 -> 미션수행존 배치 (루프)
 *   6단계: Finish Zone 복귀
 *
 * EEPROM 자세 번호 (사전 저장 필요):
 *   1  : INITIAL (초기/미션지시존 카메라 자세)
 *   2  : RESERVED / MANUAL
 *   3  : STORAGE_VIEW_SAFE (적재함 보기 + 안전 이동 자세)
 *   4  : GRIP_UPPER        (상층 집기)
 *   5  : GRIP_LOWER        (하층 집기)
 *   6  : RESERVED / MANUAL
 *   7~14: 미션수행존 1~8번 칸 배치 자세
 *
 * 하드웨어:
 *   - 메카넘 4WD (Dynamixel ID 1~4)
 *   - 4-DOF 매니퓰레이터 (Dynamixel ID 5~8)
 *   - Pixy2 카메라 (SPI)
 *   - PSD 센서: FL=A0, SL=A1, FR=A2, SR=A3
 *   - 그리퍼: Pixy2 서보 출력
 */

#include "Debug.h"
#include "Motor.h" // 전역 dxl 선언 포함
#include "Mobilebase.h"
#include "Manipulator.h" // 전역 dxl 사용 (dxl 파라미터 불필요)
#include "PSD.h"         // PSD 센서 읽기 함수
#include "Pixy.h"
#include "Gripper.h"
#include "RGBLED.h"
#include "Pins.h"
#include "MissionConfig.h"

// ============================================================
//  Pixy2 전역 객체
// ============================================================
Pixy2SPI_SS pixy;
const MissionConfig &CFG = MissionConfig::get();

// ============================================================
//  EEPROM 매니퓰레이터 자세 번호 (15차시 체계)
// ============================================================
enum ManipulatorPoseID
{
  INITIAL_AND_MISSION_INSTRUCTION = 1,
  STORAGE = 3,          // 3: 적재함 보기 + 안전 이동 자세
  GRIP_UPPER_BLOCK = 4, // 4: 상층 블록 집기
  GRIP_LOWER_BLOCK = 5  // 5: 하층 블록 집기
};

// 글로벌 미션 시작 시각
unsigned long missionStartTime = 0;

// ============================================================
//  블록 스캔 결과
// ============================================================
uint8_t targetSigs[MissionConfig::MAX_MISSION_BLOCKS];    // 순서별 시그니처 (step3에서 자동 채움)
uint8_t goalPositions[MissionConfig::MAX_MISSION_BLOCKS]; // 순서별 목표 칸 번호 (1~8)
uint8_t sourceSlots[MissionConfig::MAX_MISSION_BLOCKS];   // 순서별 적재함 물리 칸 번호 (1~8)
uint8_t totalBlocks = 0;                                  // 인식된 총 블록 수
bool storageGripAlignTimedOut = false;
uint8_t placedBlockCount = 0;
uint8_t skippedBlockCount = 0;
bool allDetectedBlocksPlaced = false;

struct StorageDetection
{
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

struct MissionPickTask
{
  uint8_t signature;
  uint8_t sourceSlot;
  uint8_t goalSlot;
  uint8_t column;
  uint8_t pickupRegion;
  int16_t x;
  int16_t y;
  uint32_t area;
  int16_t psdFl;
  int16_t psdFr;
  int16_t psdSl;
  int16_t psdSr;
  bool found;
};

int32_t clampInt32(int32_t value, int32_t minValue, int32_t maxValue)
{
  if (value < minValue)
    return minValue;
  if (value > maxValue)
    return maxValue;
  return value;
}

uint32_t pixyBlockArea(uint8_t blockIndex)
{
  return (uint32_t)pixy.ccc.blocks[blockIndex].m_width *
         (uint32_t)pixy.ccc.blocks[blockIndex].m_height;
}

void readPsdSnapshotFields(int16_t *flValue,
                           int16_t *frValue,
                           int16_t *slValue,
                           int16_t *srValue)
{
  GetValueFromFrontLeftPSDSensor(flValue);
  GetValueFromFrontRightPSDSensor(frValue);
  GetValueFromSideLeftPSDSensor(slValue);
  GetValueFromSideRightPSDSensor(srValue);
}

void printPsdSnapshotJsonFields(int16_t flValue,
                                int16_t frValue,
                                int16_t slValue,
                                int16_t srValue)
{
  DEBUG_SERIAL.print(F("\"psd\":{\"fl\":"));
  DEBUG_SERIAL.print(flValue);
  DEBUG_SERIAL.print(F(",\"fr\":"));
  DEBUG_SERIAL.print(frValue);
  DEBUG_SERIAL.print(F(",\"sl\":"));
  DEBUG_SERIAL.print(slValue);
  DEBUG_SERIAL.print(F(",\"sr\":"));
  DEBUG_SERIAL.print(srValue);
  DEBUG_SERIAL.print(F("}"));
}

bool pixyBlockPassesFilter(uint8_t blockIndex, uint8_t signatureMap, uint16_t minArea)
{
  signatureMap &= 0x7F;
  uint8_t signature = pixy.ccc.blocks[blockIndex].m_signature;
  if (signature < 1 || signature > CFG.cameraScan.maxSignature)
    return false;
  if (((1 << (signature - 1)) & signatureMap) == 0)
    return false;
  return pixyBlockArea(blockIndex) >= minArea;
}

bool readLargestTargetBlock(uint8_t targetSigmap, uint16_t minArea, int16_t *blockX, int16_t *blockY)
{
  pixy.ccc.getBlocks(true, targetSigmap);
  if (pixy.ccc.numBlocks == 0)
    return false;

  int8_t bestIndex = -1;
  uint32_t bestArea = 0;
  for (uint8_t i = 0; i < pixy.ccc.numBlocks; i++)
  {
    if (!pixyBlockPassesFilter(i, targetSigmap, minArea))
      continue;
    uint32_t area = pixyBlockArea(i);
    if (area > bestArea)
    {
      bestArea = area;
      bestIndex = i;
    }
  }

  if (bestIndex < 0)
    return false;

  *blockX = pixy.ccc.blocks[bestIndex].m_x;
  *blockY = pixy.ccc.blocks[bestIndex].m_y;
  return true;
}

uint8_t collectFilteredPixyBlocks(uint8_t signatureMap,
                                  uint16_t minArea,
                                  int16_t *blockX,
                                  uint8_t *blockSig,
                                  uint32_t *totalArea)
{
  uint8_t count = 0;
  *totalArea = 0;

  for (uint8_t i = 0; i < pixy.ccc.numBlocks && count < MissionConfig::MAX_MISSION_BLOCKS; i++)
  {
    if (!pixyBlockPassesFilter(i, signatureMap, minArea))
      continue;

    blockX[count] = pixy.ccc.blocks[i].m_x;
    blockSig[count] = pixy.ccc.blocks[i].m_signature;
    *totalArea += pixyBlockArea(i);
    count++;
  }

  return count;
}

uint8_t storageColumnForPixyX(int16_t blockX)
{
  uint8_t bestColumn = 0;
  int16_t bestError = 32767;
  for (uint8_t i = 0; i < 4; i++)
  {
    int16_t error = abs(blockX - CFG.storageRack.columnXCenters[i]);
    if (error < bestError)
    {
      bestError = error;
      bestColumn = i + 1;
    }
  }

  if (bestError > CFG.storageRack.columnXTolerance)
    return 0;
  return bestColumn;
}

uint8_t storageSlotForPixyXY(int16_t blockX, int16_t blockY)
{
  uint8_t column = storageColumnForPixyX(blockX);
  if (column == 0)
    return 0;
  return (blockY < CFG.cameraScan.storageYUpperLowerSplit)
             ? CFG.storageRack.upperRowSlots[column - 1]
             : CFG.storageRack.lowerRowSlots[column - 1];
}

bool isUpperStorageSlot(uint8_t targetSlot)
{
  for (uint8_t i = 0; i < 4; i++)
  {
    if (CFG.storageRack.upperRowSlots[i] == targetSlot)
      return true;
  }
  return false;
}

uint8_t storagePickupRegionBit(uint8_t regionId)
{
  if (regionId < 1 || regionId > MissionConfig::STORAGE_PICKUP_REGION_COUNT)
    return 0;
  return (uint8_t)1 << (regionId - 1);
}

uint8_t storagePickupRegionMaskForSlot(uint8_t targetSlot)
{
  uint8_t mask = 0;
  bool wantUpperGrip = isUpperStorageSlot(targetSlot);
  for (uint8_t i = 0; i < MissionConfig::STORAGE_PICKUP_REGION_COUNT; i++)
  {
    bool regionUsesUpperGrip = CFG.storagePickupRegion.useUpperGripPose[i] != 0;
    if (regionUsesUpperGrip == wantUpperGrip)
      mask |= storagePickupRegionBit(i + 1);
  }
  return mask;
}

uint8_t storagePickupRegionForPixyXY(int16_t blockX, int16_t blockY)
{
  for (uint8_t i = 0; i < MissionConfig::STORAGE_PICKUP_REGION_COUNT; i++)
  {
    int16_t xMin = CFG.storagePickupRegion.xMin[i] - CFG.storagePickupRegion.xMargin;
    int16_t xMax = CFG.storagePickupRegion.xMax[i] + CFG.storagePickupRegion.xMargin;
    int16_t yMin = CFG.storagePickupRegion.yMin[i] - CFG.storagePickupRegion.yMargin;
    int16_t yMax = CFG.storagePickupRegion.yMax[i] + CFG.storagePickupRegion.yMargin;
    if (blockX >= xMin && blockX <= xMax && blockY >= yMin && blockY <= yMax)
      return i + 1;
  }
  return 0;
}

bool storagePickupRegionUsesUpperGrip(uint8_t regionId)
{
  if (regionId < 1 || regionId > MissionConfig::STORAGE_PICKUP_REGION_COUNT)
    return false;
  return CFG.storagePickupRegion.useUpperGripPose[regionId - 1] != 0;
}

uint8_t storageGripTargetBit(uint8_t targetId)
{
  if (targetId < 1 || targetId > MissionConfig::STORAGE_GRIP_TARGET_COUNT)
    return 0;
  return (uint8_t)1 << (targetId - 1);
}

uint8_t storageGripTargetMaskForSlot(uint8_t targetSlot)
{
  uint8_t mask = 0;
  bool wantUpperGrip = isUpperStorageSlot(targetSlot);
  for (uint8_t i = 0; i < MissionConfig::STORAGE_GRIP_TARGET_COUNT; i++)
  {
    bool targetUsesUpperGrip = CFG.storageGripTarget.useUpperGripPose[i] != 0;
    if (targetUsesUpperGrip == wantUpperGrip)
      mask |= storageGripTargetBit(i + 1);
  }
  return mask;
}

bool storageGripTargetUsesUpperGrip(uint8_t targetId)
{
  if (targetId < 1 || targetId > MissionConfig::STORAGE_GRIP_TARGET_COUNT)
    return false;
  return CFG.storageGripTarget.useUpperGripPose[targetId - 1] != 0;
}

bool storageGripTargetContainsPixyXY(uint8_t targetId, int16_t blockX, int16_t blockY)
{
  if (targetId < 1 || targetId > MissionConfig::STORAGE_GRIP_TARGET_COUNT)
    return false;
  uint8_t i = targetId - 1;
  int16_t xMin = CFG.storageGripTarget.xMin[i] - CFG.storageGripTarget.xMargin;
  int16_t xMax = CFG.storageGripTarget.xMax[i] + CFG.storageGripTarget.xMargin;
  int16_t yMin = CFG.storageGripTarget.yMin[i] - CFG.storageGripTarget.yMargin;
  int16_t yMax = CFG.storageGripTarget.yMax[i] + CFG.storageGripTarget.yMargin;
  return blockX >= xMin && blockX <= xMax && blockY >= yMin && blockY <= yMax;
}

uint8_t storageGripTargetForPixyXY(uint8_t allowedTargetMask, int16_t blockX, int16_t blockY)
{
  for (uint8_t targetId = 1; targetId <= MissionConfig::STORAGE_GRIP_TARGET_COUNT; targetId++)
  {
    if ((allowedTargetMask & storageGripTargetBit(targetId)) == 0)
      continue;
    if (storageGripTargetContainsPixyXY(targetId, blockX, blockY))
      return targetId;
  }
  return 0;
}

int16_t storageGripTargetCenterX(uint8_t targetId)
{
  uint8_t i = targetId - 1;
  return (CFG.storageGripTarget.xMin[i] + CFG.storageGripTarget.xMax[i]) / 2;
}

int16_t storageGripTargetCenterY(uint8_t targetId)
{
  uint8_t i = targetId - 1;
  return (CFG.storageGripTarget.yMin[i] + CFG.storageGripTarget.yMax[i]) / 2;
}

uint8_t closestStorageGripTargetForPixyXY(uint8_t allowedTargetMask, int16_t blockX, int16_t blockY)
{
  uint8_t bestTarget = 0;
  int32_t bestScore = 2147483647L;
  for (uint8_t targetId = 1; targetId <= MissionConfig::STORAGE_GRIP_TARGET_COUNT; targetId++)
  {
    if ((allowedTargetMask & storageGripTargetBit(targetId)) == 0)
      continue;
    int32_t dx = blockX - storageGripTargetCenterX(targetId);
    int32_t dy = blockY - storageGripTargetCenterY(targetId);
    int32_t score = dx * dx + dy * dy;
    if (score < bestScore)
    {
      bestScore = score;
      bestTarget = targetId;
    }
  }
  return bestTarget;
}

bool readBestBlockInStoragePickupRegions(uint8_t allowedRegionMask,
                                         uint8_t targetSignatureMap,
                                         int16_t *blockX,
                                         int16_t *blockY,
                                         uint8_t *blockSig,
                                         uint8_t *pickupRegion)
{
  pixy.ccc.getBlocks(true, targetSignatureMap);
  if (pixy.ccc.numBlocks == 0)
    return false;

  int8_t bestIndex = -1;
  uint8_t bestRegion = 0;
  int16_t bestXError = 32767;
  uint32_t bestArea = 0;
  for (uint8_t i = 0; i < pixy.ccc.numBlocks; i++)
  {
    if (!pixyBlockPassesFilter(i, targetSignatureMap,
                               CFG.cameraScan.storageMinBlockArea))
      continue;

    int16_t x = pixy.ccc.blocks[i].m_x;
    int16_t y = pixy.ccc.blocks[i].m_y;
    uint8_t region = storagePickupRegionForPixyXY(x, y);
    if (region == 0)
      continue;
    if ((allowedRegionMask & storagePickupRegionBit(region)) == 0)
      continue;

    int16_t xError = abs(x - CFG.cameraScan.storageXSetpoint);
    uint32_t area = pixyBlockArea(i);
    if (xError < bestXError || (xError == bestXError && area > bestArea))
    {
      bestXError = xError;
      bestArea = area;
      bestRegion = region;
      bestIndex = i;
    }
  }

  if (bestIndex < 0)
    return false;

  *blockX = pixy.ccc.blocks[bestIndex].m_x;
  *blockY = pixy.ccc.blocks[bestIndex].m_y;
  *blockSig = pixy.ccc.blocks[bestIndex].m_signature;
  *pickupRegion = bestRegion;
  return true;
}

bool readBestBlockForStorageGripTargets(uint8_t allowedTargetMask,
                                        uint8_t targetSignatureMap,
                                        int16_t *blockX,
                                        int16_t *blockY,
                                        uint8_t *blockSig,
                                        uint8_t *pickupRegion,
                                        uint8_t *targetId,
                                        bool *targetReached)
{
  pixy.ccc.getBlocks(true, targetSignatureMap);
  if (pixy.ccc.numBlocks == 0)
    return false;

  int8_t bestIndex = -1;
  uint8_t bestPickupRegion = 0;
  uint8_t bestTarget = 0;
  bool bestReached = false;
  int32_t bestScore = 2147483647L;
  uint32_t bestArea = 0;

  for (uint8_t i = 0; i < pixy.ccc.numBlocks; i++)
  {
    if (!pixyBlockPassesFilter(i, targetSignatureMap,
                               CFG.cameraScan.storageMinBlockArea))
      continue;

    int16_t x = pixy.ccc.blocks[i].m_x;
    int16_t y = pixy.ccc.blocks[i].m_y;
    uint8_t reached = storageGripTargetForPixyXY(allowedTargetMask, x, y);
    uint8_t closest = reached ? reached : closestStorageGripTargetForPixyXY(allowedTargetMask, x, y);
    if (closest == 0)
      continue;

    int32_t dx = x - storageGripTargetCenterX(closest);
    int32_t dy = y - storageGripTargetCenterY(closest);
    int32_t score = dx * dx + dy * dy;
    uint32_t area = pixyBlockArea(i);
    if (reached)
      score = 0;

    if (score < bestScore || (score == bestScore && area > bestArea))
    {
      bestScore = score;
      bestArea = area;
      bestIndex = i;
      bestPickupRegion = storagePickupRegionForPixyXY(x, y);
      bestTarget = closest;
      bestReached = reached != 0;
    }
  }

  if (bestIndex < 0)
    return false;

  *blockX = pixy.ccc.blocks[bestIndex].m_x;
  *blockY = pixy.ccc.blocks[bestIndex].m_y;
  *blockSig = pixy.ccc.blocks[bestIndex].m_signature;
  *pickupRegion = bestPickupRegion;
  *targetId = bestTarget;
  *targetReached = bestReached;
  return true;
}

bool readLargestBlockInStorageSlot(uint8_t targetSlot,
                                   int16_t *blockX,
                                   int16_t *blockY,
                                   uint8_t *blockSig)
{
  pixy.ccc.getBlocks(true, CFG.cameraScan.storageAllowedSignatureMap);
  if (pixy.ccc.numBlocks == 0)
    return false;

  int8_t bestIndex = -1;
  uint32_t bestArea = 0;
  for (uint8_t i = 0; i < pixy.ccc.numBlocks; i++)
  {
    if (!pixyBlockPassesFilter(i, CFG.cameraScan.storageAllowedSignatureMap,
                               CFG.cameraScan.storageMinBlockArea))
      continue;

    uint8_t slot = storageSlotForPixyXY(pixy.ccc.blocks[i].m_x,
                                        pixy.ccc.blocks[i].m_y);
    if (slot != targetSlot)
      continue;

    uint32_t area = pixyBlockArea(i);
    if (area > bestArea)
    {
      bestArea = area;
      bestIndex = i;
    }
  }

  if (bestIndex < 0)
    return false;

  *blockX = pixy.ccc.blocks[bestIndex].m_x;
  *blockY = pixy.ccc.blocks[bestIndex].m_y;
  *blockSig = pixy.ccc.blocks[bestIndex].m_signature;
  return true;
}

bool readClosestBlockInStorageRow(bool upperRow,
                                  int16_t *blockX,
                                  int16_t *blockY,
                                  uint8_t *blockSig)
{
  pixy.ccc.getBlocks(true, CFG.cameraScan.storageAllowedSignatureMap);
  if (pixy.ccc.numBlocks == 0)
    return false;

  int8_t bestIndex = -1;
  int16_t bestXError = 32767;
  uint32_t bestArea = 0;
  for (uint8_t i = 0; i < pixy.ccc.numBlocks; i++)
  {
    if (!pixyBlockPassesFilter(i, CFG.cameraScan.storageAllowedSignatureMap,
                               CFG.cameraScan.storageMinBlockArea))
      continue;

    bool blockUpperRow = (int16_t)pixy.ccc.blocks[i].m_y <
                         CFG.cameraScan.storageYUpperLowerSplit;
    if (blockUpperRow != upperRow)
      continue;

    int16_t xError = abs(pixy.ccc.blocks[i].m_x - CFG.cameraScan.storageXSetpoint);
    uint32_t area = pixyBlockArea(i);
    if (xError < bestXError || (xError == bestXError && area > bestArea))
    {
      bestXError = xError;
      bestArea = area;
      bestIndex = i;
    }
  }

  if (bestIndex < 0)
    return false;

  *blockX = pixy.ccc.blocks[bestIndex].m_x;
  *blockY = pixy.ccc.blocks[bestIndex].m_y;
  *blockSig = pixy.ccc.blocks[bestIndex].m_signature;
  return true;
}

uint8_t getMissionInstructionSignatureMap()
{
  return CFG.cameraScan.missionInstructionAllowedSignatureMap &
         CFG.mission.blockSignatureMap &
         0x7F;
}

uint8_t getStorageTargetSignatureMap(uint8_t targetSig)
{
  if (targetSig < 1 || targetSig > CFG.cameraScan.maxSignature)
    return 0;
  return ((uint8_t)1 << (targetSig - 1)) &
         CFG.cameraScan.storageAllowedSignatureMap &
         0x7F;
}

void printPixyRecognitionMode(const __FlashStringHelper *label, uint8_t signatureMap)
{
  signatureMap &= 0x7F;
  DEBUG_SERIAL.print(label);
  DEBUG_SERIAL.print(F(" Pixy signature filter=0b"));
  for (int bit = 6; bit >= 0; bit--)
  {
    DEBUG_SERIAL.print((signatureMap >> bit) & 0x01);
  }
  DEBUG_SERIAL.println();
}

void printMissionInstructionPixyRawDebug(uint8_t filteredSignatureMap)
{
  DEBUG_SERIAL.println(F("  [Pixy raw debug] 미션지시존 원본 프레임 확인"));
  DEBUG_SERIAL.print(F("  filter=0b"));
  for (int bit = 6; bit >= 0; bit--)
    DEBUG_SERIAL.print((filteredSignatureMap & ((uint8_t)1 << bit)) ? '1' : '0');
  DEBUG_SERIAL.print(F(", minArea="));
  DEBUG_SERIAL.println(CFG.cameraScan.missionInstructionMinBlockArea);

  for (uint8_t frame = 0; frame < 5; frame++)
  {
    pixy.ccc.getBlocks(true, 0x7F);
    DEBUG_SERIAL.print(F("    frame "));
    DEBUG_SERIAL.print(frame + 1);
    DEBUG_SERIAL.print(F(": rawCount="));
    DEBUG_SERIAL.println(pixy.ccc.numBlocks);
    for (uint8_t i = 0; i < pixy.ccc.numBlocks; i++)
    {
      uint8_t sig = pixy.ccc.blocks[i].m_signature;
      uint32_t area = pixyBlockArea(i);
      bool passMap = sig >= 1 && sig <= CFG.cameraScan.maxSignature &&
                     ((filteredSignatureMap & ((uint8_t)1 << (sig - 1))) != 0);
      bool passArea = area >= CFG.cameraScan.missionInstructionMinBlockArea;
      DEBUG_SERIAL.print(F("      sig="));
      DEBUG_SERIAL.print(sig);
      DEBUG_SERIAL.print(F(" x="));
      DEBUG_SERIAL.print(pixy.ccc.blocks[i].m_x);
      DEBUG_SERIAL.print(F(" y="));
      DEBUG_SERIAL.print(pixy.ccc.blocks[i].m_y);
      DEBUG_SERIAL.print(F(" area="));
      DEBUG_SERIAL.print(area);
      DEBUG_SERIAL.print(F(" map="));
      DEBUG_SERIAL.print(passMap ? F("OK") : F("NO"));
      DEBUG_SERIAL.print(F(" area="));
      DEBUG_SERIAL.println(passArea ? F("OK") : F("NO"));
    }
    delay(CFG.wait.scanSampleMs);
  }
}

void driveBackwardWithLeftBoundaryCorrection(int16_t slVal)
{
  int32_t correctionSpeed = 0;

  if (slVal <= CFG.finishReturn.boundaryAdc)
  {
    correctionSpeed = -CFG.finishReturn.openSideLeftBiasSpeed;
  }
  else
  {
    int16_t sideError = slVal - CFG.finishReturn.trackSl;
    if (abs(sideError) > CFG.finishReturn.trackTolerance)
    {
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

void haltWithRedBlink(const __FlashStringHelper *title, const __FlashStringHelper *detail)
{
  SetMobileGoalVelocityForSyncWrite(dxl, 0, 0, 0, 0);
  pixy.setLamp(0, 0);

  DEBUG_SERIAL.println(F(""));
  DEBUG_SERIAL.println(title);
  DEBUG_SERIAL.println(detail);
  DEBUG_SERIAL.println(F("  안전을 위해 코드를 멈춥니다. 원인을 수정한 뒤 전원을 다시 켜세요."));

  while (1)
  {
    setRGBLEDRed();
    delay(500);
    setRGBLEDOff();
    delay(500);
  }
}

void haltForMissingManipulatorPose(uint8_t poseId, const __FlashStringHelper *context)
{
  DEBUG_SERIAL.print(F("  [오류] EEPROM 자세 "));
  DEBUG_SERIAL.print(poseId);
  DEBUG_SERIAL.println(F("번 데이터가 없습니다."));
  DEBUG_SERIAL.println(context);
  haltWithRedBlink(F("  매니퓰레이터 자세 누락"), F("  manageManipulatorPose에서 필요한 자세를 저장한 뒤 다시 실행하세요."));
}

ManipulatorPose runRequiredManipulatorPose(uint8_t poseId,
                                           int32_t operatingTimeMillis,
                                           float motor1Angle,
                                           const __FlashStringHelper *context)
{
  ManipulatorPose pose = RunManipulatorPoseWithPoseDataInEEPROM(poseId,
                                                                operatingTimeMillis,
                                                                motor1Angle);
  if (!pose.isTherePoseData)
  {
    haltForMissingManipulatorPose(poseId, context);
  }
  return pose;
}

ManipulatorPose runTimedManipulatorPose(uint8_t poseId,
                                        uint16_t operatingTimeMillis,
                                        float motor1Angle,
                                        const __FlashStringHelper *context)
{
  ManipulatorPose pose = runRequiredManipulatorPose(poseId,
                                                    operatingTimeMillis,
                                                    motor1Angle,
                                                    context);
  delay((unsigned long)operatingTimeMillis + CFG.wait.poseSettleMs);
  return pose;
}

int32_t manipulatorMotor1GoalForPose(ManipulatorPose pose, float motor1Angle)
{
  if (motor1Angle == -360.0)
    return pose.manipulatorMotor1Value;
  return map(constrain(round(motor1Angle * 100), -10000, 10000),
             -18000, 18000, 0, 4095);
}

void readPresentManipulatorMotorValues(int32_t *motorValues)
{
  motorValues[0] = dxl.readControlTableItem(PRESENT_POSITION, ARM_DXL_IDS[0]);
  motorValues[1] = dxl.readControlTableItem(PRESENT_POSITION, ARM_DXL_IDS[1]);
  motorValues[2] = dxl.readControlTableItem(PRESENT_POSITION, ARM_DXL_IDS[2]);
  motorValues[3] = dxl.readControlTableItem(PRESENT_POSITION, ARM_DXL_IDS[3]);
}

ManipulatorPose runStagedManipulatorPose(uint8_t poseId,
                                         uint16_t stageMs,
                                         float motor1Angle,
                                         const uint8_t *motorOrder,
                                         uint8_t motorOrderCount,
                                         const __FlashStringHelper *context)
{
  ManipulatorPose pose = ReadManipulatorPresentPoseToEEPROM(poseId);
  if (!pose.isTherePoseData)
  {
    haltForMissingManipulatorPose(poseId, context);
  }

  int32_t current[4] = {0, 0, 0, 0};
  int32_t target[4] = {
      manipulatorMotor1GoalForPose(pose, motor1Angle),
      pose.manipulatorMotor2Value,
      pose.manipulatorMotor3Value,
      pose.manipulatorMotor4Value};
  readPresentManipulatorMotorValues(current);

  // m1은 보통 0도 유지라서 각 단계에 같이 맞춰 둔다. 충돌 회피 순서는 m4 -> m3 -> m2가 담당한다.
  current[0] = target[0];
  for (uint8_t i = 0; i < motorOrderCount; i++)
  {
    uint8_t motorIndex = motorOrder[i];
    if (motorIndex > 3)
      continue;
    current[motorIndex] = target[motorIndex];
    SetManipulatorForwardMoveWithMotorValueForSyncWrite(current[0],
                                                        current[1],
                                                        current[2],
                                                        current[3],
                                                        stageMs);
    delay((unsigned long)stageMs + CFG.wait.poseSettleMs);
  }

  return pose;
}

ManipulatorPose runUpperGripPoseWithRfidClearance()
{
  static const uint8_t upperGripOrder[3] = {3, 2, 1}; // m4 -> m3 -> m2
  DEBUG_SERIAL.println(F("    상층 RFID 간섭 회피 그립: m4 -> m3 -> m2"));
  return runStagedManipulatorPose(GRIP_UPPER_BLOCK,
                                  CFG.poseTiming.upperGripStageMs,
                                  0.0,
                                  upperGripOrder,
                                  3,
                                  F("  4번 상층 블록 집기 자세가 필요합니다."));
}

ManipulatorPose runUpperLiftToStorageWithRfidClearance()
{
  static const uint8_t upperLiftOrder[3] = {3, 2, 1}; // m4 -> m3 -> m2
  DEBUG_SERIAL.println(F("    상층 RFID 간섭 회피 복귀: m4 -> m3 -> m2 -> storage"));
  return runStagedManipulatorPose(STORAGE,
                                  CFG.poseTiming.upperLiftStageMs,
                                  0.0,
                                  upperLiftOrder,
                                  3,
                                  F("  집은 블록을 들고 이동하기 전 3번 안전 이동 자세가 필요합니다."));
}

uint8_t missionZonePoseIdForGoal(uint8_t goalPos)
{
  if (goalPos < 1 || goalPos > CFG.pose.missionZoneSlotCount)
  {
    haltWithRedBlink(F("  [설정 오류] 미션 수행존 번호가 보정된 범위를 벗어났습니다."),
                     F("  MissionConfig.h의 goalPositions 값을 확인하세요."));
  }

  return CFG.pose.missionZoneStartId + goalPos;
}

void validateMissionConfig()
{
  const uint8_t validSignatureBits = 0x7F;
  if (CFG.pose.missionZoneSlotCount < 1 ||
      CFG.pose.missionZoneSlotCount > MissionConfig::MAX_MISSION_BLOCKS)
  {
    haltWithRedBlink(F("  [설정 오류] 미션수행존 slot 개수 오류"),
                     F("  CFG.pose.missionZoneSlotCount는 1~MAX_MISSION_BLOCKS 사이여야 합니다."));
  }

  if (CFG.mission.blockCount < 1 ||
      CFG.mission.blockCount > MissionConfig::MAX_MISSION_BLOCKS ||
      CFG.mission.blockCount > CFG.pose.missionZoneSlotCount)
  {
    haltWithRedBlink(F("  [설정 오류] blockCount 범위 오류"),
                     F("  CFG.mission.blockCount는 보정된 미션수행존 slot 개수 이하여야 합니다."));
  }
  if (CFG.storageRack.pickSlotCount < 1 || CFG.storageRack.pickSlotCount > 8)
  {
    haltWithRedBlink(F("  [설정 오류] pickSlotCount 범위 오류"),
                     F("  CFG.storageRack.pickSlotCount는 1~8 사이여야 합니다."));
  }
  if (CFG.storageRack.perSlotScanMs == 0)
  {
    haltWithRedBlink(F("  [설정 오류] perSlotScanMs 범위 오류"),
                     F("  CFG.storageRack.perSlotScanMs는 0보다 커야 합니다."));
  }

  if (CFG.mission.blockSignatureMap == 0 ||
      CFG.cameraScan.missionInstructionAllowedSignatureMap == 0 ||
      CFG.cameraScan.storageAllowedSignatureMap == 0)
  {
    haltWithRedBlink(F("  [설정 오류] Pixy signature map이 비어 있습니다."),
                     F("  MissionConfig.h의 signature map은 최소 하나 이상의 signature를 포함해야 합니다."));
  }

  if ((CFG.mission.blockSignatureMap & ~validSignatureBits) != 0 ||
      (CFG.cameraScan.missionInstructionAllowedSignatureMap & ~validSignatureBits) != 0 ||
      (CFG.cameraScan.storageAllowedSignatureMap & ~validSignatureBits) != 0)
  {
    haltWithRedBlink(F("  [설정 오류] Pixy signature map 범위 오류"),
                     F("  Pixy2 CCC signature map은 1~7번 bit만 사용해야 합니다."));
  }

  if (getMissionInstructionSignatureMap() == 0)
  {
    haltWithRedBlink(F("  [설정 오류] 미션지시존 signature 교집합 없음"),
                     F("  blockSignatureMap과 missionInstructionAllowedSignatureMap을 확인하세요."));
  }

  if (CFG.cameraScan.maxSignature < 1 || CFG.cameraScan.maxSignature > 7)
  {
    haltWithRedBlink(F("  [설정 오류] Pixy maxSignature 범위 오류"),
                     F("  Pixy2 CCC signature는 1~7 범위만 사용합니다."));
  }

  uint16_t usedGoalMask = 0;
  for (uint8_t i = 0; i < CFG.pose.missionZoneSlotCount; i++)
  {
    uint8_t goal = CFG.mission.goalPositions[i];
    if (goal < 1 || goal > CFG.pose.missionZoneSlotCount)
    {
      haltWithRedBlink(F("  [설정 오류] goalPositions 범위 오류"),
                       F("  CFG.mission.goalPositions 값은 보정된 미션수행존 slot 범위 안이어야 합니다."));
    }
    uint16_t goalBit = (uint16_t)1 << goal;
    if ((usedGoalMask & goalBit) != 0)
    {
      haltWithRedBlink(F("  [설정 오류] goalPositions 중복"),
                       F("  같은 미션수행존 칸에 두 번 배치하지 않도록 설정을 확인하세요."));
    }
    usedGoalMask |= goalBit;
  }

  uint16_t usedSourceSlotMask = 0;
  for (uint8_t i = 0; i < CFG.storageRack.pickSlotCount; i++)
  {
    uint8_t slot = CFG.storageRack.pickSlotOrder[i];
    if (slot < 1 || slot > 8)
    {
      haltWithRedBlink(F("  [설정 오류] pickSlotOrder 범위 오류"),
                       F("  CFG.storageRack.pickSlotOrder 값은 1~8이어야 합니다."));
    }
    uint16_t slotBit = (uint16_t)1 << slot;
    if ((usedSourceSlotMask & slotBit) != 0)
    {
      haltWithRedBlink(F("  [설정 오류] pickSlotOrder 중복"),
                       F("  같은 적재함 칸을 두 번 집지 않도록 설정을 확인하세요."));
    }
    usedSourceSlotMask |= slotBit;
  }

  if (CFG.pose.missionZoneStartId < 1 ||
      CFG.pose.missionZoneStartId + CFG.pose.missionZoneSlotCount > MANIPULATOR_POSE_ID_MAX_CNT)
  {
    haltWithRedBlink(F("  [설정 오류] 미션수행존 pose ID 범위 오류"),
                     F("  missionZoneStartId + slotCount가 EEPROM pose 범위 안에 있어야 합니다."));
  }

  if (CFG.psd.missionTolerance <= 0 ||
      CFG.psd.alignTolerance <= 0 ||
      CFG.psd.missionZoneTolerance <= 0 ||
      CFG.psd.gripAlignTolerance <= 0 ||
      CFG.psd.lowerGripAlignTolerance <= 0 ||
      CFG.psd.scanSlTolerance <= 0 ||
      CFG.psd.storageApproachSlLeaveAdc <= 0 ||
      CFG.psd.storageApproachSlReenterAdc <= 0 ||
      CFG.psd.storageApproachSlReenterConfirmSamples == 0 ||
      CFG.psd.storageApproachIgnoreReentryMs == 0 ||
      CFG.speed.storageApproachForwardSpeed <= 0 ||
      CFG.cameraScan.storageXTolerance <= 0 ||
      CFG.storageRack.columnXTolerance <= 0 ||
      CFG.storageRack.scanColumnStepMm <= 0.0 ||
      CFG.storageRack.scanColumnMoveMmPerSec <= 0 ||
      CFG.storageRack.scanFramesPerStop == 0 ||
      CFG.finishReturn.trackTolerance <= 0)
  {
    haltWithRedBlink(F("  [설정 오류] tolerance 값 오류"),
                     F("  PSD/Pixy/복귀/적재함 열 이동 값은 0보다 커야 합니다."));
  }

  if (CFG.storagePickupRegion.xMargin < 0 ||
      CFG.storagePickupRegion.yMargin < 0)
  {
    haltWithRedBlink(F("  [설정 오류] pickup region margin 오류"),
                     F("  storagePickupRegion x/y margin은 0 이상이어야 합니다."));
  }

  for (uint8_t i = 0; i < MissionConfig::STORAGE_PICKUP_REGION_COUNT; i++)
  {
    if (CFG.storagePickupRegion.xMin[i] > CFG.storagePickupRegion.xMax[i] ||
        CFG.storagePickupRegion.yMin[i] > CFG.storagePickupRegion.yMax[i])
    {
      haltWithRedBlink(F("  [설정 오류] pickup region 범위 오류"),
                       F("  xMin<=xMax, yMin<=yMax가 되도록 MissionConfig.h를 확인하세요."));
    }
  }

  if (CFG.storageGripTarget.xMargin < 0 ||
      CFG.storageGripTarget.yMargin < 0 ||
      CFG.storageGripTarget.centerToleranceX <= 0 ||
      CFG.storageGripTarget.centerToleranceY <= 0 ||
      CFG.storageGripTarget.alignTimeoutMs == 0 ||
      CFG.storageGripTarget.alignStepMs == 0 ||
      CFG.storageGripTarget.alignFullSpeedPixelError <= CFG.storageGripTarget.centerToleranceX ||
      CFG.storageGripTarget.alignFullSpeedPixelError <= CFG.storageGripTarget.centerToleranceY ||
      CFG.storageGripTarget.upperExtraForwardMm < 0.0 ||
      CFG.storageGripTarget.lowerExtraForwardMm < 0.0 ||
      CFG.storageGripTarget.upperExtraForwardMm > 80.0 ||
      CFG.storageGripTarget.lowerExtraForwardMm > 80.0 ||
      CFG.storageGripTarget.extraForwardMmPerSec <= 0 ||
      CFG.storageGripTarget.fineAlignGain <= 0.0 ||
      CFG.storageGripTarget.fineAlignMinStepMm <= 0.0 ||
      CFG.storageGripTarget.fineAlignMaxStepMm < CFG.storageGripTarget.fineAlignMinStepMm ||
      CFG.storageGripTarget.fineAlignForwardMaxStepMm < 0.0 ||
      CFG.storageGripTarget.fineAlignSpeedMmPerSec <= 0)
  {
    haltWithRedBlink(F("  [설정 오류] grip target 정렬값 오류"),
                     F("  storageGripTarget margin/timeout/step/depth/fineAlign 값을 확인하세요."));
  }

  if (CFG.poseTiming.upperGripStageMs == 0 ||
      CFG.poseTiming.upperLiftStageMs == 0)
  {
    haltWithRedBlink(F("  [설정 오류] 상층 staged pose 시간 오류"),
                     F("  upperGripStageMs/upperLiftStageMs는 0보다 커야 합니다."));
  }

  for (uint8_t i = 0; i < MissionConfig::STORAGE_GRIP_TARGET_COUNT; i++)
  {
    if (CFG.storageGripTarget.xMin[i] > CFG.storageGripTarget.xMax[i] ||
        CFG.storageGripTarget.yMin[i] > CFG.storageGripTarget.yMax[i])
    {
      haltWithRedBlink(F("  [설정 오류] grip target 범위 오류"),
                       F("  xMin<=xMax, yMin<=yMax가 되도록 MissionConfig.h를 확인하세요."));
    }
  }

  DEBUG_SERIAL.println(F("  MissionConfig 사전 검사 OK"));
}

// ============================================================
//  1단계: 초기화 & 버튼 대기
// ============================================================
void step1_init()
{
  DEBUG_SERIAL.println(F(""));
  DEBUG_SERIAL.println(F("========================================"));
  DEBUG_SERIAL.println(F(" [1단계] 초기화"));
  DEBUG_SERIAL.println(F("========================================"));

  initDebug();
  initMotorCommunication();

  while (!InitMobilebase(dxl))
  {
    DEBUG_SERIAL.println(F("  모바일베이스 초기화 실패..."));
    delay(1000);
  }

  while (!initManipulator())
  {
    DEBUG_SERIAL.println(F("  매니퓰레이터 초기화 실패..."));
    delay(1000);
  }

  InitPSD();
  InitPixy(pixy);
  initRGBLED();
  setRGBLEDOff();

  validateMissionConfig();

  // 그리퍼 열기, INITIAL 자세
  OpenGripper(pixy);
  runTimedManipulatorPose(INITIAL_AND_MISSION_INSTRUCTION, CFG.poseTiming.startupInitialMs,
                          -360.0,
                          F("  1번 자세는 초기 자세와 미션지시존 카메라 자세로 사용됩니다."));
  SetMobileGoalVelocityForSyncWrite(dxl, 0, 0, 0, 0);

  // 스캔 결과 초기화
  totalBlocks = 0;
  placedBlockCount = 0;
  skippedBlockCount = 0;
  allDetectedBlocksPlaced = false;
  memset(targetSigs, 0, sizeof(targetSigs));
  memset(goalPositions, 0, sizeof(goalPositions));
  memset(sourceSlots, 0, sizeof(sourceSlots));

  // 버튼 핀 설정
  pinMode(SW1_PIN, INPUT_PULLUP);

  // 대기 표시: 파란 LED
  setRGBLEDBlue();
  DEBUG_SERIAL.println(F("  >> SW1 버튼을 누르면 미션 시작 <<"));

  while (digitalRead(SW1_PIN) == HIGH)
  {
    delay(10);
  } // PULLUP: 안 누르면 HIGH
  delay(CFG.wait.buttonDebounceMs);
  while (digitalRead(SW1_PIN) == LOW)
  {
    delay(10);
  } // 버튼 뗄 때까지 대기

  // ── 출발 LED 시퀀스 (규정 8점) ──
  // 빨간 LED 3번 점멸 → 초록(파란) LED 1번 점멸 → 출발
  // "점멸 간격은 심판진이 확실히 구별할 수 있도록"
  pinMode(RED_LED_PIN, OUTPUT);

  DEBUG_SERIAL.println(F("  LED 시퀀스 시작..."));

  // 빨간 LED 3번 점멸
  for (int i = 0; i < 3; i++)
  {
    setRGBLEDRed();
    digitalWrite(RED_LED_PIN, HIGH);
    delay(400);
    setRGBLEDOff();
    digitalWrite(RED_LED_PIN, LOW);
    delay(400);
  }

  // 초록 LED 1번 점멸
  setRGBLEDGreen();
  digitalWrite(RED_LED_PIN, LOW);
  delay(CFG.wait.cameraLampMs);
  setRGBLEDOff();
  delay(CFG.wait.buttonDebounceMs);

  DEBUG_SERIAL.println(F("  >> 미션 시작! <<"));
}

// ============================================================
//  2단계: 장애물 접근 & 미션지시존 이동
// ============================================================
void step2_approach()
{
  DEBUG_SERIAL.println(F(""));
  DEBUG_SERIAL.println(F("========================================"));
  DEBUG_SERIAL.println(F(" [2단계] 장애물 접근 & 미션지시존 이동"));
  DEBUG_SERIAL.println(F("========================================"));

  // --- FL/FR 첫 감지 기반 전방 접근 + 감속 정지 ---
  DEBUG_SERIAL.print(F("  [2-1] FL/FR 첫 감지("));
  DEBUG_SERIAL.print(CFG.front.firstDetectAdc);
  DEBUG_SERIAL.println(F(") 기반 전방 접근..."));
  int16_t flVal = 0, frVal = 0;
  int32_t currentForwardSpeed = CFG.front.cruiseSpeed;
  bool frontDetected = false;

  unsigned long t0 = millis();
  while (1)
  {
    GetValueFromFrontPSDSensors(&flVal, &frVal);
    int16_t frontDetectVal = max(flVal, frVal);

    if (frontDetectVal >= CFG.front.firstDetectAdc)
    {
      frontDetected = true;
      DEBUG_SERIAL.print(F("  첫 감지 발생(front="));
      DEBUG_SERIAL.print(frontDetectVal);
      DEBUG_SERIAL.println(F(")"));
      break;
    }

    // 임계값에 가까워지면 저속으로 바꿔 튕김을 줄임
    if (frontDetectVal >= CFG.front.firstDetectAdc - CFG.front.decelWindowAdc)
      currentForwardSpeed = CFG.front.slowSpeed;
    else
      currentForwardSpeed = CFG.front.cruiseSpeed;

    SetMobileGoalVelocityForSyncWrite(dxl, currentForwardSpeed, currentForwardSpeed,
                                      currentForwardSpeed, currentForwardSpeed);

    if (millis() - t0 > CFG.timeout.psdLoopMs)
    {
      DEBUG_SERIAL.println(F("  [2-1] 타임아웃"));
      break;
    }
    delay(10);
  }

  // 감지/타임아웃 이후 즉시 정지가 아니라 램프다운 정지
  int32_t decelStartSpeed = max(currentForwardSpeed, CFG.front.slowSpeed);
  for (int32_t speed = decelStartSpeed; speed > 0; speed -= CFG.front.brakeStep)
  {
    SetMobileGoalVelocityForSyncWrite(dxl, speed, speed, speed, speed);
    delay(CFG.front.brakeDelayMs / 2);
  }
  SetMobileGoalVelocityForSyncWrite(dxl, 0, 0, 0, 0);

  if (!frontDetected)
  {
    DEBUG_SERIAL.println(F("  [2-1] 첫 감지 없이 정지 (타임아웃/환경요인)"));
  }

  DEBUG_SERIAL.print(F("  접근 완료 FL="));
  DEBUG_SERIAL.print(flVal);
  DEBUG_SERIAL.print(F(" FR="));
  DEBUG_SERIAL.println(frVal);
  delay(CFG.wait.driveSettleMs);

  // --- SL PSD로 미션지시존 열 정렬 (좌측벽 기준 거리 조정) ---
  DEBUG_SERIAL.println(F("  [2-2] SL PSD 미션지시존 정렬..."));
  DEBUG_SERIAL.print(F("  목표 SL="));
  DEBUG_SERIAL.print(CFG.psd.missionSl);
  DEBUG_SERIAL.print(F(", tolerance="));
  DEBUG_SERIAL.println(CFG.psd.missionTolerance);
  int16_t slVal;
  t0 = millis();
  while (1)
  {
    GetValueFromSideLeftPSDSensor(&slVal);
    if (!DriveWithOneSensor(dxl, slVal - CFG.psd.missionSl,
                            CFG.psd.missionTolerance, DRIVE_DIRECTION_LEFT,
                            CFG.speed.psdCorrectionSpeed))
      break;
    if (millis() - t0 > CFG.timeout.psdLoopMs)
    {
      DEBUG_SERIAL.println(F("  [2-2] 타임아웃"));
      break;
    }
  }
  SetMobileGoalVelocityForSyncWrite(dxl, 0, 0, 0, 0);
  DEBUG_SERIAL.print(F("  미션지시존 도착 SL="));
  DEBUG_SERIAL.println(slVal);
  delay(CFG.wait.driveSettleMs);
}

// ============================================================
//  3단계: 블록 색상/순서 스캔
// ============================================================
void step3_scan()
{
  DEBUG_SERIAL.println(F(""));
  DEBUG_SERIAL.println(F("========================================"));
  DEBUG_SERIAL.println(F(" [3단계] 블록 스캔"));
  DEBUG_SERIAL.println(F("========================================"));

  totalBlocks = 0;
  placedBlockCount = 0;
  skippedBlockCount = 0;
  allDetectedBlocksPlaced = false;
  memset(targetSigs, 0, sizeof(targetSigs));
  memset(goalPositions, 0, sizeof(goalPositions));
  memset(sourceSlots, 0, sizeof(sourceSlots));

  // INITIAL 자세에서 미션지시존을 카메라로 촬영
  // (step2 후 이미 INITIAL 자세이므로 확인용)
  runTimedManipulatorPose(INITIAL_AND_MISSION_INSTRUCTION, CFG.poseTiming.missionInstructionMs,
                          -360.0,
                          F("  미션지시존 스캔 전에 1번 초기 자세가 필요합니다."));
  uint8_t missionInstructionLamp = CFG.cameraScan.missionInstructionLampOn ? 1 : 0;
  pixy.setLamp(missionInstructionLamp, missionInstructionLamp);
  delay(CFG.wait.cameraLampMs);
  DEBUG_SERIAL.println(CFG.cameraScan.missionInstructionLampOn ? F("  Pixy lamp ON") : F("  Pixy lamp OFF"));

  // CFG.mission.blockSignatureMap으로 관심 시그니처만 필터링하여 스캔
  // 여러 프레임 중 최다 블록 인식 결과 사용
  int16_t bestBlockX[MissionConfig::MAX_MISSION_BLOCKS];
  uint8_t bestBlockSig[MissionConfig::MAX_MISSION_BLOCKS];
  uint8_t bestCount = 0;
  uint32_t bestAreaScore = 0;
  uint8_t missionInstructionSignatureMap = getMissionInstructionSignatureMap();
  printPixyRecognitionMode(F("  [미션지시존 signature 필터]"), missionInstructionSignatureMap);
  DEBUG_SERIAL.print(F("  최소 블록 면적="));
  DEBUG_SERIAL.println(CFG.cameraScan.missionInstructionMinBlockArea);

  for (uint8_t pass = 0; pass < 2 && bestCount == 0; pass++)
  {
    uint16_t activeMinArea = (pass == 0) ? CFG.cameraScan.missionInstructionMinBlockArea : 0;
    if (pass == 1 && CFG.cameraScan.missionInstructionMinBlockArea == 0)
      break;
    if (pass == 1)
    {
      DEBUG_SERIAL.println(F("  [fallback] 면적 필터를 0으로 낮춰 재스캔합니다."));
    }

    for (int attempt = 0; attempt < 20; attempt++)
    {
      pixy.ccc.getBlocks(true, missionInstructionSignatureMap);
      int16_t candidateBlockX[MissionConfig::MAX_MISSION_BLOCKS];
      uint8_t candidateBlockSig[MissionConfig::MAX_MISSION_BLOCKS];
      uint32_t candidateAreaScore = 0;
      uint8_t candidateCount = collectFilteredPixyBlocks(missionInstructionSignatureMap,
                                                         activeMinArea,
                                                         candidateBlockX,
                                                         candidateBlockSig,
                                                         &candidateAreaScore);
      if (candidateCount > bestCount ||
          (candidateCount == bestCount && candidateAreaScore > bestAreaScore))
      {
        bestCount = candidateCount;
        bestAreaScore = candidateAreaScore;
        for (int i = 0; i < bestCount; i++)
        {
          bestBlockX[i] = candidateBlockX[i];
          bestBlockSig[i] = candidateBlockSig[i];
        }
      }
      delay(CFG.wait.scanSampleMs);
    }
  }

  if (bestCount == 0)
  {
    DEBUG_SERIAL.println(F("  [스킵] 미션지시존 블록을 인식하지 못했습니다."));
    printMissionInstructionPixyRawDebug(missionInstructionSignatureMap);
    DEBUG_SERIAL.println(F("  같은 스캔을 반복하지 않고 적재함/집기 단계를 건너뛰어 복귀합니다."));
    totalBlocks = 0;
    return;
  }

  // X좌표 기준 버블정렬 (왼→오른 = 미션 순서)
  for (int i = 0; i < bestCount - 1; i++)
  {
    for (int j = 0; j < bestCount - 1 - i; j++)
    {
      if (bestBlockX[j] > bestBlockX[j + 1])
      {
        int16_t tmpX = bestBlockX[j];
        bestBlockX[j] = bestBlockX[j + 1];
        bestBlockX[j + 1] = tmpX;
        uint8_t tmpSig = bestBlockSig[j];
        bestBlockSig[j] = bestBlockSig[j + 1];
        bestBlockSig[j + 1] = tmpSig;
      }
    }
  }

  // 동적 모드에서는 카메라 인식 수를 실제 처리 개수로 사용한다.
  // 고정 모드에서는 대회 설정 blockCount까지만 처리한다.
  totalBlocks = CFG.mission.dynamicBlockCount
                    ? min(bestCount, (uint8_t)MissionConfig::MAX_MISSION_BLOCKS)
                    : min(bestCount, (uint8_t)CFG.mission.blockCount);
  totalBlocks = min(totalBlocks, CFG.storageRack.pickSlotCount);
  totalBlocks = min(totalBlocks, CFG.pose.missionZoneSlotCount);

  // 결과 저장: 미션지시존은 처리할 signature 목록만 만든다.
  // 실제 source/goal slot은 적재함 전체 survey에서 발견한 위치를 사용한다.
  for (int i = 0; i < totalBlocks; i++)
  {
    targetSigs[i] = bestBlockSig[i];
    goalPositions[i] = 0;
    sourceSlots[i] = 0;
  }

  // 스캔 결과 출력
  DEBUG_SERIAL.print(F("  블록 수: "));
  DEBUG_SERIAL.println(totalBlocks);
  for (int i = 0; i < totalBlocks; i++)
  {
    DEBUG_SERIAL.print(F("  ["));
    DEBUG_SERIAL.print(i);
    DEBUG_SERIAL.print(F("] Sig"));
    DEBUG_SERIAL.print(targetSigs[i]);
    DEBUG_SERIAL.println(F(", source/zone=pending storage survey"));
  }
}

// ============================================================
//  4단계: 적재함 앞 접근 & 정밀 정렬
// ============================================================
int16_t storageAlignFrontErrorFor(int16_t flVal, int16_t frVal,
                                  int16_t targetFl, int16_t targetFr,
                                  int16_t tolerance)
{
  int16_t flError = flVal - targetFl;
  int16_t frError = frVal - targetFr;
  bool flOk = abs(flError) <= tolerance;
  bool frOk = abs(frError) <= tolerance;
  if (flOk && frOk)
    return 0;
  if (flOk)
    return frError;
  if (frOk)
    return flError;
  if ((flError < 0 && frError > 0) || (flError > 0 && frError < 0))
    return abs(flError) >= abs(frError) ? flError : frError;
  return (flError + frError) / 2;
}

bool alignStorageWithPsdTarget(const __FlashStringHelper *title,
                               int16_t targetSl,
                               int16_t targetFl,
                               int16_t targetFr,
                               int16_t tolerance)
{
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
  ChangeMobilebaseMode2VelocityControlMode(dxl);

  int16_t slVal = 0;
  int16_t flVal = 0;
  int16_t frVal = 0;
  int16_t frontError = 0;
  unsigned long t0 = millis();
  while (1)
  {
    GetValueFromSideLeftPSDSensor(&slVal);
    GetValueFromFrontLeftPSDSensor(&flVal);
    GetValueFromFrontRightPSDSensor(&frVal);
    frontError = storageAlignFrontErrorFor(flVal, frVal, targetFl, targetFr, tolerance);
    if (!LocatingWithTwoSensors(dxl, slVal - targetSl,
                                frontError, tolerance,
                                DRIVE_DIRECTION_LEFT,
                                CFG.speed.psdCorrectionSpeed))
      break;
    if (millis() - t0 > CFG.timeout.psdLoopMs)
    {
      DEBUG_SERIAL.println(F("  PSD 정렬 타임아웃"));
      break;
    }
    delay(10);
  }

  SetMobileGoalVelocityForSyncWrite(dxl, 0, 0, 0, 0);
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
  delay(CFG.wait.driveSettleMs);
  return true;
}

bool alignStorageFrontDepthWithPsdTarget(const __FlashStringHelper *title,
                                         int16_t targetFl,
                                         int16_t targetFr,
                                         int16_t tolerance)
{
  DEBUG_SERIAL.println(title);
  DEBUG_SERIAL.print(F("  target FL/FR/tol="));
  DEBUG_SERIAL.print(targetFl);
  DEBUG_SERIAL.print(F("/"));
  DEBUG_SERIAL.print(targetFr);
  DEBUG_SERIAL.print(F("/"));
  DEBUG_SERIAL.print(tolerance);
  DEBUG_SERIAL.println(F(" (SL/SR ignored for motion)"));
  ChangeMobilebaseMode2VelocityControlMode(dxl);

  int16_t slVal = 0;
  int16_t flVal = 0;
  int16_t frVal = 0;
  int16_t flError = 0;
  int16_t frError = 0;
  unsigned long t0 = millis();
  while (1)
  {
    GetValueFromSideLeftPSDSensor(&slVal);
    GetValueFromFrontLeftPSDSensor(&flVal);
    GetValueFromFrontRightPSDSensor(&frVal);
    flError = flVal - targetFl;
    frError = frVal - targetFr;
    if (!GoForwardWithTwoSensors(dxl, flError, frError, tolerance,
                                 CFG.speed.psdCorrectionSpeed))
      break;
    if (millis() - t0 > CFG.timeout.psdLoopMs)
    {
      DEBUG_SERIAL.println(F("  PSD 전후 깊이 정렬 타임아웃"));
      break;
    }
    delay(10);
  }

  SetMobileGoalVelocityForSyncWrite(dxl, 0, 0, 0, 0);
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
  delay(CFG.wait.driveSettleMs);
  return true;
}

bool alignStorageSideLeftWithPsdTarget(const __FlashStringHelper *title,
                                       int16_t targetSl,
                                       int16_t tolerance)
{
  DEBUG_SERIAL.println(title);
  DEBUG_SERIAL.print(F("  target SL/tol="));
  DEBUG_SERIAL.print(targetSl);
  DEBUG_SERIAL.print(F("/"));
  DEBUG_SERIAL.println(tolerance);
  ChangeMobilebaseMode2VelocityControlMode(dxl);

  int16_t slVal = 0;
  unsigned long t0 = millis();
  while (1)
  {
    GetValueFromSideLeftPSDSensor(&slVal);
    if (!DriveWithOneSensor(dxl, slVal - targetSl, tolerance,
                            DRIVE_DIRECTION_LEFT,
                            CFG.speed.psdCorrectionSpeed))
      break;
    if (millis() - t0 > CFG.timeout.psdLoopMs)
    {
      DEBUG_SERIAL.println(F("  SL 정렬 타임아웃"));
      break;
    }
    delay(10);
  }

  SetMobileGoalVelocityForSyncWrite(dxl, 0, 0, 0, 0);
  DEBUG_SERIAL.print(F("  SL 정렬 종료 SL="));
  DEBUG_SERIAL.print(slVal);
  DEBUG_SERIAL.print(F(" err="));
  DEBUG_SERIAL.println(slVal - targetSl);
  delay(CFG.wait.driveSettleMs);
  return true;
}

bool alignStorageScanPosition(const __FlashStringHelper *title)
{
  DEBUG_SERIAL.println(title);
  bool ok = alignStorageSideLeftWithPsdTarget(
    F("  [scan align A] SL 단독 정렬"),
    CFG.psd.alignSl,
    CFG.psd.alignTolerance);
  if (ok)
  {
    ok = alignStorageFrontDepthWithPsdTarget(
      F("  [scan align B] FL/FR 전방 깊이 정렬"),
      CFG.psd.alignFl,
      CFG.psd.alignFr,
      CFG.psd.alignTolerance);
  }
  return ok;
}

bool alignStorageReturnPosition(const __FlashStringHelper *title)
{
  DEBUG_SERIAL.println(title);
  bool ok = alignStorageSideLeftWithPsdTarget(
    F("  [return align A] SL 단독 정렬"),
    CFG.psd.returnAlignSl,
    CFG.psd.returnAlignTolerance);
  if (ok)
  {
    ok = alignStorageFrontDepthWithPsdTarget(
      F("  [return align B] FL/FR 전방 깊이 정렬"),
      CFG.psd.returnAlignFl,
      CFG.psd.returnAlignFr,
      CFG.psd.returnAlignTolerance);
  }
  return ok;
}

bool driveForwardUntilStorageSlReentry()
{
  DEBUG_SERIAL.println(F("  [4-1] 지시존 이탈 후 다음 SL 박스 감지까지 전진"));
  DEBUG_SERIAL.print(F("    SL leave<="));
  DEBUG_SERIAL.print(CFG.psd.storageApproachSlLeaveAdc);
  DEBUG_SERIAL.print(F(", reenter>="));
  DEBUG_SERIAL.print(CFG.psd.storageApproachSlReenterAdc);
  DEBUG_SERIAL.print(F(", confirm samples="));
  DEBUG_SERIAL.print(CFG.psd.storageApproachSlReenterConfirmSamples);
  DEBUG_SERIAL.print(F(", ignore reentry ms="));
  DEBUG_SERIAL.print(CFG.psd.storageApproachIgnoreReentryMs);
  DEBUG_SERIAL.print(F(", forward raw="));
  DEBUG_SERIAL.println(CFG.speed.storageApproachForwardSpeed);

  ChangeMobilebaseMode2VelocityControlMode(dxl);
  int16_t slVal = 0;
  bool sawLeave = false;
  uint8_t reenterSamples = 0;
  unsigned long ignoreReentryUntil = 0;
  unsigned long t0 = millis();
  while (1)
  {
    GetValueFromSideLeftPSDSensor(&slVal);
    if (!sawLeave)
    {
      if (slVal <= CFG.psd.storageApproachSlLeaveAdc)
      {
        sawLeave = true;
        ignoreReentryUntil = millis() + CFG.psd.storageApproachIgnoreReentryMs;
        DEBUG_SERIAL.print(F("    SL 이탈 감지 SL="));
        DEBUG_SERIAL.println(slVal);
        DEBUG_SERIAL.print(F("    이후 "));
        DEBUG_SERIAL.print(CFG.psd.storageApproachIgnoreReentryMs);
        DEBUG_SERIAL.println(F("ms 동안 재감지 무시하고 전진"));
      }
    }
    else
    {
      if ((long)(millis() - ignoreReentryUntil) < 0)
      {
        reenterSamples = 0;
      }
      else if (slVal >= CFG.psd.storageApproachSlReenterAdc)
      {
        if (reenterSamples < CFG.psd.storageApproachSlReenterConfirmSamples)
          reenterSamples++;
      }
      else
      {
        reenterSamples = 0;
      }
      if (reenterSamples >= CFG.psd.storageApproachSlReenterConfirmSamples)
        break;
    }

    SetMobileGoalVelocityForSyncWrite(dxl,
                                      CFG.speed.storageApproachForwardSpeed,
                                      CFG.speed.storageApproachForwardSpeed,
                                      CFG.speed.storageApproachForwardSpeed,
                                      CFG.speed.storageApproachForwardSpeed);
    if (millis() - t0 > CFG.timeout.psdLoopMs)
    {
      DEBUG_SERIAL.println(F("    SL 재감지 전진 타임아웃"));
      break;
    }
    delay(10);
  }

  SetMobileGoalVelocityForSyncWrite(dxl, 0, 0, 0, 0);
  DEBUG_SERIAL.print(F("    SL 재감지 전진 종료 SL="));
  DEBUG_SERIAL.print(slVal);
  DEBUG_SERIAL.print(F(" sawLeave="));
  DEBUG_SERIAL.print(sawLeave ? F("yes") : F("no"));
  DEBUG_SERIAL.print(F(" samples="));
  DEBUG_SERIAL.println(reenterSamples);
  delay(CFG.wait.driveSettleMs);
  return sawLeave &&
         reenterSamples >= CFG.psd.storageApproachSlReenterConfirmSamples;
}

bool alignStorageGripPosition(bool lower)
{
  return alignStorageFrontDepthWithPsdTarget(
    lower
      ? F("    집기 직전: 하층 FL/FR 전후 깊이 정렬")
      : F("    집기 직전: 상층 FL/FR 전후 깊이 정렬"),
    lower ? CFG.psd.lowerGripAlignFl : CFG.psd.gripAlignFl,
    lower ? CFG.psd.lowerGripAlignFr : CFG.psd.gripAlignFr,
    lower ? CFG.psd.lowerGripAlignTolerance : CFG.psd.gripAlignTolerance);
}

bool alignStorageGripPositionForSlot(uint8_t sourceSlot)
{
  return alignStorageGripPosition(sourceSlot >= 5);
}

uint8_t storageColumnForSourceSlot(uint8_t sourceSlot)
{
  for (uint8_t i = 0; i < 4; i++)
  {
    if (CFG.storageRack.upperRowSlots[i] == sourceSlot ||
        CFG.storageRack.lowerRowSlots[i] == sourceSlot)
      return i + 1;
  }
  return 0;
}

bool driveOneStorageColumn(uint8_t direction)
{
  ChangeMobilebaseMode2ExtendedPositionControlWithTimeBasedProfileMode(dxl);
  DriveDistanceAndMmPerSecAndDirection(dxl,
                                       CFG.storageRack.scanColumnStepMm,
                                       direction,
                                       CFG.storageRack.scanColumnMoveMmPerSec);
  bool ok = waitForMobilePositionOrTimeout(CFG.timeout.positionMoveMs,
                                           F("  적재함 열 이동 타임아웃"));
  ChangeMobilebaseMode2VelocityControlMode(dxl);
  SetMobileGoalVelocityForSyncWrite(dxl, 0, 0, 0, 0);
  return ok;
}

bool moveStorageToColumn(uint8_t *currentColumn, uint8_t targetColumn)
{
  if (targetColumn < 1 || targetColumn > 4)
    return false;
  if (*currentColumn < 1 || *currentColumn > 4)
    *currentColumn = 1;

  while (*currentColumn != targetColumn)
  {
    uint8_t direction = (targetColumn > *currentColumn)
                          ? DRIVE_DIRECTION_RIGHT
                          : DRIVE_DIRECTION_LEFT;
    DEBUG_SERIAL.print(F("  적재함 열 이동 "));
    DEBUG_SERIAL.print(*currentColumn);
    DEBUG_SERIAL.print(F(" -> "));
    DEBUG_SERIAL.print((direction == DRIVE_DIRECTION_RIGHT) ? *currentColumn + 1 : *currentColumn - 1);
    DEBUG_SERIAL.print(F(", step="));
    DEBUG_SERIAL.print(CFG.storageRack.scanColumnStepMm, 2);
    DEBUG_SERIAL.print(F("mm @ "));
    DEBUG_SERIAL.print(CFG.storageRack.scanColumnMoveMmPerSec);
    DEBUG_SERIAL.println(F("mm/s"));

    if (!driveOneStorageColumn(direction))
      return false;
    if (direction == DRIVE_DIRECTION_RIGHT)
      (*currentColumn)++;
    else
      (*currentColumn)--;
  }
  return true;
}

bool scanCurrentStorageColumnForTarget(uint8_t sourceSlot,
                                       uint8_t targetSignatureMap,
                                       uint8_t requiredColumn,
                                       int16_t *blockX,
                                       int16_t *blockY,
                                       uint8_t *blockSig,
                                       uint8_t *pickupRegion)
{
  uint8_t allowedRegionMask = storagePickupRegionMaskForSlot(sourceSlot);
  uint8_t frames = CFG.storageRack.scanFramesPerStop;
  if (frames < 1)
    frames = 1;
  if (frames > 20)
    frames = 20;

  DEBUG_SERIAL.print(F("  현재 열 Pixy 스캔: sourceSlot="));
  DEBUG_SERIAL.print(sourceSlot);
  DEBUG_SERIAL.print(F(", column="));
  DEBUG_SERIAL.print(requiredColumn);
  DEBUG_SERIAL.print(F(", frames="));
  DEBUG_SERIAL.print(frames);
  DEBUG_SERIAL.print(F(", minArea="));
  DEBUG_SERIAL.println(CFG.storageRack.scanMinBlockArea);

  int8_t bestIndex = -1;
  uint8_t bestRegion = 0;
  uint8_t bestSig = 0;
  int16_t bestX = 0;
  int16_t bestY = 0;
  uint32_t bestArea = 0;
  bool sawSignature = false;
  bool sawPickupWindow = false;

  for (uint8_t frame = 0; frame < frames; frame++)
  {
    pixy.ccc.getBlocks(true, targetSignatureMap);
    DEBUG_SERIAL.print(F("    frame "));
    DEBUG_SERIAL.print(frame + 1);
    DEBUG_SERIAL.print(F(": count="));
    DEBUG_SERIAL.println(pixy.ccc.numBlocks);

    for (uint8_t i = 0; i < pixy.ccc.numBlocks; i++)
    {
      if (!pixyBlockPassesFilter(i, targetSignatureMap,
                                 CFG.storageRack.scanMinBlockArea))
        continue;
      sawSignature = true;

      int16_t x = pixy.ccc.blocks[i].m_x;
      int16_t y = pixy.ccc.blocks[i].m_y;
      uint32_t area = pixyBlockArea(i);
      uint8_t region = storagePickupRegionForPixyXY(x, y);
      uint8_t pixyColumn = storageColumnForPixyX(x);
      if (pixyColumn > 0 &&
          requiredColumn >= 1 &&
          requiredColumn <= 4 &&
          abs((int)pixyColumn - (int)requiredColumn) > 1)
      {
        DEBUG_SERIAL.print(F("      reject column mismatch sig="));
        DEBUG_SERIAL.print(pixy.ccc.blocks[i].m_signature);
        DEBUG_SERIAL.print(F(", x/y="));
        DEBUG_SERIAL.print(x);
        DEBUG_SERIAL.print(F("/"));
        DEBUG_SERIAL.print(y);
        DEBUG_SERIAL.print(F(", pixyColumn="));
        DEBUG_SERIAL.println(pixyColumn);
        continue;
      }
      if (region == 0)
      {
        DEBUG_SERIAL.print(F("      reject pickup boundary sig="));
        DEBUG_SERIAL.print(pixy.ccc.blocks[i].m_signature);
        DEBUG_SERIAL.print(F(", x/y="));
        DEBUG_SERIAL.print(x);
        DEBUG_SERIAL.print(F("/"));
        DEBUG_SERIAL.println(y);
        continue;
      }
      sawPickupWindow = true;
      if ((allowedRegionMask & storagePickupRegionBit(region)) == 0)
      {
        DEBUG_SERIAL.print(F("      reject layer sig="));
        DEBUG_SERIAL.print(pixy.ccc.blocks[i].m_signature);
        DEBUG_SERIAL.print(F(", region="));
        DEBUG_SERIAL.println(region);
        continue;
      }

      if (bestIndex < 0 || area > bestArea)
      {
        bestIndex = i;
        bestRegion = region;
        bestSig = pixy.ccc.blocks[i].m_signature;
        bestX = x;
        bestY = y;
        bestArea = area;
      }
    }
    delay(CFG.wait.scanSampleMs);
  }

  if (bestIndex < 0)
  {
    DEBUG_SERIAL.println(F("  현재 열에서 목표 블록을 찾지 못했습니다."));
    if (sawSignature && !sawPickupWindow)
      DEBUG_SERIAL.println(F("  signature는 보였지만 upper/lower pickup boundary 밖입니다."));
    else if (sawSignature)
      DEBUG_SERIAL.println(F("  signature는 보였지만 sourceSlot의 상/하층 조건과 맞지 않습니다."));
    return false;
  }

  *blockX = bestX;
  *blockY = bestY;
  *blockSig = bestSig;
  *pickupRegion = bestRegion;
  DEBUG_SERIAL.print(F("  현재 열 스캔 결정 sig="));
  DEBUG_SERIAL.print(*blockSig);
  DEBUG_SERIAL.print(F(", x/y="));
  DEBUG_SERIAL.print(*blockX);
  DEBUG_SERIAL.print(F("/"));
  DEBUG_SERIAL.print(*blockY);
  DEBUG_SERIAL.print(F(", region="));
  DEBUG_SERIAL.print(*pickupRegion);
  DEBUG_SERIAL.print(F(", area="));
  DEBUG_SERIAL.println(bestArea);
  return true;
}

uint8_t sourceSlotForStorageColumnAndRegion(uint8_t column, uint8_t pickupRegion)
{
  if (column < 1 || column > 4 || pickupRegion == 0)
    return 0;
  return storagePickupRegionUsesUpperGrip(pickupRegion)
           ? CFG.storageRack.upperRowSlots[column - 1]
           : CFG.storageRack.lowerRowSlots[column - 1];
}

uint8_t missionTargetSignatureMap()
{
  uint8_t signatureMap = 0;
  for (uint8_t i = 0; i < totalBlocks; i++)
  {
    signatureMap |= getStorageTargetSignatureMap(targetSigs[i]);
  }
  return signatureMap;
}

uint8_t neededMissionTargetCountForSignature(uint8_t signature)
{
  uint8_t count = 0;
  for (uint8_t i = 0; i < totalBlocks; i++)
  {
    if (targetSigs[i] == signature)
      count++;
  }
  return count;
}

uint8_t storageDetectionCountForSignature(StorageDetection detections[],
                                          uint8_t detectionCount,
                                          uint8_t signature)
{
  uint8_t count = 0;
  for (uint8_t i = 0; i < detectionCount; i++)
  {
    if (detections[i].signature == signature)
      count++;
  }
  return count;
}

bool storageSurveyCoversMissionTargets(StorageDetection detections[],
                                       uint8_t detectionCount)
{
  for (uint8_t i = 0; i < totalBlocks; i++)
  {
    uint8_t signature = targetSigs[i];
    if (storageDetectionCountForSignature(detections, detectionCount, signature) <
        neededMissionTargetCountForSignature(signature))
      return false;
  }
  return totalBlocks > 0;
}

bool upsertStorageDetection(StorageDetection detections[],
                            uint8_t *detectionCount,
                            uint8_t signature,
                            uint8_t sourceSlot,
                            uint8_t column,
                            uint8_t pickupRegion,
                            int16_t x,
                            int16_t y,
                            uint32_t area,
                            int16_t psdFl,
                            int16_t psdFr,
                            int16_t psdSl,
                            int16_t psdSr)
{
  for (uint8_t i = 0; i < *detectionCount; i++)
  {
    if (detections[i].signature == signature &&
        detections[i].sourceSlot == sourceSlot)
    {
      if (area > detections[i].area)
      {
        detections[i].pickupRegion = pickupRegion;
        detections[i].x = x;
        detections[i].y = y;
        detections[i].area = area;
        detections[i].psdFl = psdFl;
        detections[i].psdFr = psdFr;
        detections[i].psdSl = psdSl;
        detections[i].psdSr = psdSr;
      }
      return true;
    }
  }

  if (*detectionCount >= MissionConfig::MAX_MISSION_BLOCKS)
    return false;

  StorageDetection &detection = detections[*detectionCount];
  detection.signature = signature;
  detection.sourceSlot = sourceSlot;
  detection.column = column;
  detection.pickupRegion = pickupRegion;
  detection.x = x;
  detection.y = y;
  detection.area = area;
  detection.psdFl = psdFl;
  detection.psdFr = psdFr;
  detection.psdSl = psdSl;
  detection.psdSr = psdSr;
  detection.assigned = false;
  (*detectionCount)++;
  return true;
}

void printStorageDetectionJson(const StorageDetection &detection)
{
  DEBUG_SERIAL.print(F("{\"type\":\"storage-survey-detection\",\"sig\":"));
  DEBUG_SERIAL.print(detection.signature);
  DEBUG_SERIAL.print(F(",\"sourceSlot\":"));
  DEBUG_SERIAL.print(detection.sourceSlot);
  DEBUG_SERIAL.print(F(",\"goalSlot\":"));
  DEBUG_SERIAL.print(detection.sourceSlot);
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
  DEBUG_SERIAL.print(F(","));
  printPsdSnapshotJsonFields(detection.psdFl, detection.psdFr,
                             detection.psdSl, detection.psdSr);
  DEBUG_SERIAL.println(F("}"));
}

bool surveyCurrentStorageColumn(StorageDetection detections[],
                                uint8_t *detectionCount,
                                uint8_t column,
                                uint8_t signatureMap)
{
  uint8_t frames = constrain(CFG.storageRack.scanFramesPerStop, 1, 20);
  bool foundAny = false;

  DEBUG_SERIAL.print(F("[storage survey] column="));
  DEBUG_SERIAL.print(column);
  DEBUG_SERIAL.print(F(", frames="));
  DEBUG_SERIAL.print(frames);
  DEBUG_SERIAL.print(F(", minArea="));
  DEBUG_SERIAL.println(CFG.storageRack.scanMinBlockArea);

  for (uint8_t frame = 0; frame < frames; frame++)
  {
    int16_t psdFl = 0;
    int16_t psdFr = 0;
    int16_t psdSl = 0;
    int16_t psdSr = 0;
    readPsdSnapshotFields(&psdFl, &psdFr, &psdSl, &psdSr);

    pixy.ccc.getBlocks(true, signatureMap);
    DEBUG_SERIAL.print(F("  frame "));
    DEBUG_SERIAL.print(frame + 1);
    DEBUG_SERIAL.print(F(": count="));
    DEBUG_SERIAL.println(pixy.ccc.numBlocks);

    for (uint8_t i = 0; i < pixy.ccc.numBlocks; i++)
    {
      if (!pixyBlockPassesFilter(i, signatureMap,
                                 CFG.storageRack.scanMinBlockArea))
        continue;

      int16_t x = pixy.ccc.blocks[i].m_x;
      int16_t y = pixy.ccc.blocks[i].m_y;
      uint8_t pickupRegion = storagePickupRegionForPixyXY(x, y);
      if (pickupRegion == 0)
        continue;

      uint8_t sourceSlot = sourceSlotForStorageColumnAndRegion(column, pickupRegion);
      if (sourceSlot == 0)
        continue;

      uint8_t signature = pixy.ccc.blocks[i].m_signature;
      uint32_t area = pixyBlockArea(i);
      if (upsertStorageDetection(detections, detectionCount, signature,
                                 sourceSlot, column, pickupRegion, x, y, area,
                                 psdFl, psdFr, psdSl, psdSr))
      {
        foundAny = true;
      }
    }
    delay(CFG.wait.scanSampleMs);
  }

  for (uint8_t i = 0; i < *detectionCount; i++)
  {
    if (detections[i].column == column)
      printStorageDetectionJson(detections[i]);
  }
  return foundAny;
}

bool runStorageSurvey(StorageDetection detections[],
                      uint8_t *detectionCount,
                      uint8_t *currentColumn)
{
  *detectionCount = 0;
  uint8_t signatureMap = missionTargetSignatureMap();
  if (signatureMap == 0)
    return false;

  printPixyRecognitionMode(F("[storage survey] target"), signatureMap);

  for (uint8_t column = 1; column <= 4; column++)
  {
    if (!moveStorageToColumn(currentColumn, column))
      return false;
    surveyCurrentStorageColumn(detections, detectionCount, column, signatureMap);
    if (storageSurveyCoversMissionTargets(detections, *detectionCount))
    {
      DEBUG_SERIAL.println(F("[storage survey] 목표 signature를 모두 찾았습니다. 남은 열 스캔을 생략합니다."));
      return true;
    }
  }

  DEBUG_SERIAL.println(F("[storage survey] 4열까지 스캔 완료"));
  return *detectionCount > 0;
}

int16_t taskDistanceScore(uint8_t currentColumn, const StorageDetection &detection)
{
  return abs((int)detection.column - (int)currentColumn);
}

uint8_t firstUnassignedTargetIndexForSignature(bool targetAssigned[], uint8_t signature)
{
  for (uint8_t i = 0; i < totalBlocks; i++)
  {
    if (!targetAssigned[i] && targetSigs[i] == signature)
      return i;
  }
  return MissionConfig::MAX_MISSION_BLOCKS;
}

uint8_t buildMissionPickTasks(StorageDetection detections[],
                              uint8_t detectionCount,
                              MissionPickTask tasks[],
                              uint8_t startColumn)
{
  bool targetAssigned[MissionConfig::MAX_MISSION_BLOCKS] = {false};
  uint8_t taskCount = 0;
  (void)startColumn;

  for (uint8_t i = 0; i < detectionCount; i++)
    detections[i].assigned = false;

  while (taskCount < totalBlocks)
  {
    uint8_t bestDetection = MissionConfig::MAX_MISSION_BLOCKS;
    uint8_t bestTarget = MissionConfig::MAX_MISSION_BLOCKS;
    uint8_t bestColumn = 0;
    uint32_t bestArea = 0;

    for (uint8_t i = 0; i < detectionCount; i++)
    {
      if (detections[i].assigned)
        continue;
      uint8_t targetIndex = firstUnassignedTargetIndexForSignature(targetAssigned,
                                                                   detections[i].signature);
      if (targetIndex >= MissionConfig::MAX_MISSION_BLOCKS)
        continue;

      if (bestDetection >= MissionConfig::MAX_MISSION_BLOCKS ||
          detections[i].column > bestColumn ||
          (detections[i].column == bestColumn && detections[i].area > bestArea))
      {
        bestDetection = i;
        bestTarget = targetIndex;
        bestColumn = detections[i].column;
        bestArea = detections[i].area;
      }
    }

    if (bestDetection >= MissionConfig::MAX_MISSION_BLOCKS)
      break;

    StorageDetection &detection = detections[bestDetection];
    detection.assigned = true;
    targetAssigned[bestTarget] = true;

    MissionPickTask &task = tasks[taskCount++];
    task.signature = detection.signature;
    task.sourceSlot = detection.sourceSlot;
    task.goalSlot = detection.sourceSlot;
    task.column = detection.column;
    task.pickupRegion = detection.pickupRegion;
    task.x = detection.x;
    task.y = detection.y;
    task.area = detection.area;
    task.psdFl = detection.psdFl;
    task.psdFr = detection.psdFr;
    task.psdSl = detection.psdSl;
    task.psdSr = detection.psdSr;
    task.found = true;

    DEBUG_SERIAL.print(F("{\"type\":\"mission-pick-task\",\"index\":"));
    DEBUG_SERIAL.print(taskCount);
    DEBUG_SERIAL.print(F(",\"sig\":"));
    DEBUG_SERIAL.print(task.signature);
    DEBUG_SERIAL.print(F(",\"sourceSlot\":"));
    DEBUG_SERIAL.print(task.sourceSlot);
    DEBUG_SERIAL.print(F(",\"goalSlot\":"));
    DEBUG_SERIAL.print(task.goalSlot);
    DEBUG_SERIAL.print(F(",\"poseId\":"));
    DEBUG_SERIAL.print(missionZonePoseIdForGoal(task.goalSlot));
    DEBUG_SERIAL.print(F(",\"column\":"));
    DEBUG_SERIAL.print(task.column);
    DEBUG_SERIAL.print(F(",\"layer\":\""));
    DEBUG_SERIAL.print(storagePickupRegionUsesUpperGrip(task.pickupRegion) ? F("upper") : F("lower"));
    DEBUG_SERIAL.print(F("\",\"x\":"));
    DEBUG_SERIAL.print(task.x);
    DEBUG_SERIAL.print(F(",\"y\":"));
    DEBUG_SERIAL.print(task.y);
    DEBUG_SERIAL.print(F(",\"area\":"));
    DEBUG_SERIAL.print(task.area);
    DEBUG_SERIAL.print(F(","));
    printPsdSnapshotJsonFields(task.psdFl, task.psdFr, task.psdSl, task.psdSr);
    DEBUG_SERIAL.println(F("}"));

    // Survey 기준으로 가장 먼 열부터 처리해 후반 이동/재정렬 부담을 줄인다.
  }

  return taskCount;
}

bool approachStorageScanPositionDynamically()
{
  bool ok = driveForwardUntilStorageSlReentry();
  if (!ok)
  {
    DEBUG_SERIAL.println(F("  [주의] SL 이탈/재감지가 완전하지 않습니다. 현재 위치 기준으로 SL/FL/FR 정렬을 시도합니다."));
  }
  return alignStorageScanPosition(F("  [4-3] 최종 스캔 기준 정렬: SL 먼저, FL/FR 나중"));
}

void step4_alignToStorage()
{
  DEBUG_SERIAL.println(F(""));
  DEBUG_SERIAL.println(F("========================================"));
  DEBUG_SERIAL.println(F(" [4단계] 적재함 접근 & 정렬"));
  DEBUG_SERIAL.println(F("========================================"));

  // STORAGE 자세 (팔 접기, 이동 안전)
  runTimedManipulatorPose(STORAGE, CFG.poseTiming.storageMs, 0.0,
                          F("  적재함으로 이동하기 전 3번 안전 이동 자세가 필요합니다."));
  DEBUG_SERIAL.println(F("  STORAGE 자세 완료"));

  approachStorageScanPositionDynamically();

  // 카메라 램프 ON
  pixy.setLamp(1, 1);
  delay(CFG.wait.cameraLampMs);
  DEBUG_SERIAL.println(F("  카메라 램프 ON"));
}

bool waitForMobilePositionOrTimeout(unsigned long timeoutMs,
                                    const __FlashStringHelper *timeoutMessage)
{
  unsigned long startedAt = millis();
  while (!CheckIfMobilebaseIsInPosition(dxl))
  {
    if (millis() - startedAt > timeoutMs)
    {
      DEBUG_SERIAL.println(timeoutMessage);
      SetMobileGoalVelocityForSyncWrite(dxl, 0, 0, 0, 0);
      return false;
    }
    delay(10);
  }
  return true;
}

// ============================================================
//  5단계 헬퍼: 적재함 survey, 저속 중심 정렬, 집기
// ============================================================

/*
 * 카메라에 목표 적재함 slot 블록이 잡혔을 때 미세조정 → 상/하층 판별 → 집기까지 수행
 * 반환: true=집기 성공, false=실패
 */
float fineAlignMmPerPixelX()
{
  int32_t pixelSum = 0;
  uint8_t gapCount = 0;
  for (uint8_t i = 0; i < 3; i++)
  {
    int16_t gap = abs(CFG.storageRack.columnXCenters[i + 1] -
                      CFG.storageRack.columnXCenters[i]);
    if (gap > 0)
    {
      pixelSum += gap;
      gapCount++;
    }
  }
  if (gapCount == 0 || pixelSum == 0)
    return 1.0;
  float avgPixelGap = (float)pixelSum / (float)gapCount;
  return CFG.storageRack.scanColumnStepMm / avgPixelGap;
}

float clampFineAlignDistance(float value, float minValue, float maxValue)
{
  if (value < minValue)
    return minValue;
  if (value > maxValue)
    return maxValue;
  return value;
}

float fineAlignDistanceForPixelError(int16_t error, float maxStepMm)
{
  float distanceMm = (float)abs(error) *
                     fineAlignMmPerPixelX() *
                     CFG.storageGripTarget.fineAlignGain;
  return clampFineAlignDistance(distanceMm,
                                CFG.storageGripTarget.fineAlignMinStepMm,
                                maxStepMm);
}

bool driveFineAlignDistance(uint8_t direction, float distanceMm)
{
  if (distanceMm <= 0.0)
    return true;
  ChangeMobilebaseMode2ExtendedPositionControlWithTimeBasedProfileMode(dxl);
  DriveDistanceAndMmPerSecAndDirection(dxl, distanceMm, direction,
                                       CFG.storageGripTarget.fineAlignSpeedMmPerSec);
  bool ok = waitForMobilePositionOrTimeout(CFG.timeout.positionMoveMs,
                                           F("    fine align 이동 타임아웃"));
  ChangeMobilebaseMode2VelocityControlMode(dxl);
  SetMobileGoalVelocityForSyncWrite(dxl, 0, 0, 0, 0);
  return ok;
}

bool stepCameraGripAlignment(int16_t xError, int16_t yError)
{
  if (abs(xError) > CFG.storageGripTarget.centerToleranceX)
  {
    float sideStepMm = fineAlignDistanceForPixelError(
      xError, CFG.storageGripTarget.fineAlignMaxStepMm);
    uint8_t direction = (xError > 0) ? DRIVE_DIRECTION_RIGHT : DRIVE_DIRECTION_LEFT;
    DEBUG_SERIAL.print(F("    fine align X "));
    DEBUG_SERIAL.print(sideStepMm, 2);
    DEBUG_SERIAL.println(direction == DRIVE_DIRECTION_RIGHT ? F("mm right") : F("mm left"));
    return driveFineAlignDistance(direction, sideStepMm);
  }

  if (CFG.storageGripTarget.fineAlignUseY &&
      abs(yError) > CFG.storageGripTarget.centerToleranceY)
  {
    float forwardStepMm = fineAlignDistanceForPixelError(
      yError, CFG.storageGripTarget.fineAlignForwardMaxStepMm);
    bool forward = (yError > 0) == CFG.storageGripTarget.yErrorUsesForwardDirection;
    uint8_t direction = forward ? DRIVE_DIRECTION_FORWARD : DRIVE_DIRECTION_BACKWARD;
    DEBUG_SERIAL.print(F("    fine align Y "));
    DEBUG_SERIAL.print(forwardStepMm, 2);
    DEBUG_SERIAL.println(forward ? F("mm forward") : F("mm backward"));
    return driveFineAlignDistance(direction, forwardStepMm);
  }

  return true;
}

float storageGripDepthMmForTarget(uint8_t targetId)
{
  return storageGripTargetUsesUpperGrip(targetId)
           ? CFG.storageGripTarget.upperExtraForwardMm
           : CFG.storageGripTarget.lowerExtraForwardMm;
}

bool driveStorageGripDepth(uint8_t targetId, uint8_t direction)
{
  float distanceMm = storageGripDepthMmForTarget(targetId);
  if (distanceMm <= 0.0)
    return true;

  ChangeMobilebaseMode2ExtendedPositionControlWithTimeBasedProfileMode(dxl);
  DriveDistanceAndMmPerSecAndDirection(dxl, distanceMm, direction,
                                       CFG.storageGripTarget.extraForwardMmPerSec);
  bool ok = waitForMobilePositionOrTimeout(CFG.timeout.positionMoveMs,
                                           F("    그립 깊이 이동 타임아웃"));
  ChangeMobilebaseMode2VelocityControlMode(dxl);
  SetMobileGoalVelocityForSyncWrite(dxl, 0, 0, 0, 0);
  return ok;
}

bool fineTuneAndPickStorageSlot(uint8_t targetSlot, uint8_t targetSignatureMap)
{
  SetMobileGoalVelocityForSyncWrite(dxl, 0, 0, 0, 0);
  storageGripAlignTimedOut = false;
  DEBUG_SERIAL.print(F("    적재함 Slot "));
  DEBUG_SERIAL.print(targetSlot);
  DEBUG_SERIAL.println(F(" 영역 감지 → 그립 목표창 정렬 시작"));

  uint8_t allowedRegionMask = storagePickupRegionMaskForSlot(targetSlot);
  uint8_t allowedTargetMask = storageGripTargetMaskForSlot(targetSlot);
  if (allowedRegionMask == 0)
  {
    DEBUG_SERIAL.println(F("    [오류] target slot에 맞는 pickup region이 없습니다."));
    return false;
  }
  if (allowedTargetMask == 0)
  {
    DEBUG_SERIAL.println(F("    [오류] target slot에 맞는 grip target이 없습니다."));
    return false;
  }

  runTimedManipulatorPose(STORAGE, CFG.poseTiming.preGripMs, 0.0,
                          F("  그립 목표창 정렬 전 3번 적재함 보기 자세가 필요합니다."));
  if (!alignStorageGripPositionForSlot(targetSlot))
    return false;

  unsigned long t0 = millis();
  while (1)
  {
    int16_t blockX = 0;
    int16_t blockY = 0;
    uint8_t blockSig = 0;
    uint8_t pickupRegion = 0;
    uint8_t targetId = 0;
    bool targetReached = false;

    bool foundBlock = readBestBlockForStorageGripTargets(allowedTargetMask,
                                                         targetSignatureMap,
                                                         &blockX, &blockY,
                                                         &blockSig, &pickupRegion,
                                                         &targetId, &targetReached);

    if (foundBlock)
    {
      if (targetId == 0)
      {
        DEBUG_SERIAL.println(F("    [오류] 선택 가능한 grip target이 없습니다."));
        return false;
      }

      int16_t xError = blockX - storageGripTargetCenterX(targetId);
      int16_t yError = blockY - storageGripTargetCenterY(targetId);

      if (targetReached)
      {
        // 그립 목표창 도달 → 집기 실행
        SetMobileGoalVelocityForSyncWrite(dxl, 0, 0, 0, 0);
        DEBUG_SERIAL.println(F("    그립 목표창 정렬 완료"));
        DEBUG_SERIAL.print(F("    선택 블록 sig="));
        DEBUG_SERIAL.print(blockSig);
        DEBUG_SERIAL.print(F(", x="));
        DEBUG_SERIAL.print(blockX);
        DEBUG_SERIAL.print(F(", y="));
        DEBUG_SERIAL.print(blockY);
        DEBUG_SERIAL.print(F(", region="));
        DEBUG_SERIAL.print(pickupRegion);
        DEBUG_SERIAL.print(F(", target="));
        DEBUG_SERIAL.println(targetId);

        // grip target 기준 상/하층 판별
        bool upperTarget = storageGripTargetUsesUpperGrip(targetId);
        if (!alignStorageGripPosition(!upperTarget))
          return false;
        if (!driveStorageGripDepth(targetId, DRIVE_DIRECTION_FORWARD))
          return false;

        if (upperTarget)
        {
          // 상층 블록
          DEBUG_SERIAL.println(F("    상층 target 블록 집기"));
          runUpperGripPoseWithRfidClearance();
        }
        else
        {
          // 하층 블록
          DEBUG_SERIAL.println(F("    하층 target 블록 집기"));
          runTimedManipulatorPose(GRIP_LOWER_BLOCK, CFG.poseTiming.gripMs, 0.0,
                                  F("  5번 하층 블록 집기 자세가 필요합니다."));
        }

        // 그리퍼 닫기
        CloseGripper(pixy);
        delay(CFG.wait.gripperActionMs);
        DEBUG_SERIAL.println(F("    그리퍼 닫기 완료"));

        // STORAGE 자세로 접기 (이동 안전)
        if (upperTarget)
        {
          runUpperLiftToStorageWithRfidClearance();
        }
        else
        {
          runTimedManipulatorPose(STORAGE, CFG.poseTiming.storageWithBlockMs, 0.0,
                                  F("  집은 블록을 들고 이동하기 전 3번 안전 이동 자세가 필요합니다."));
        }

        if (!driveStorageGripDepth(targetId, DRIVE_DIRECTION_BACKWARD))
          return false;

        return true;
      }

      DEBUG_SERIAL.print(F("    target "));
      DEBUG_SERIAL.print(targetId);
      DEBUG_SERIAL.print(F(" 정렬 중 x="));
      DEBUG_SERIAL.print(blockX);
      DEBUG_SERIAL.print(F(", y="));
      DEBUG_SERIAL.print(blockY);
      DEBUG_SERIAL.print(F(", dx="));
      DEBUG_SERIAL.print(xError);
      DEBUG_SERIAL.print(F(", dy="));
      DEBUG_SERIAL.println(yError);
      stepCameraGripAlignment(xError, yError);
    }
    else
    {
      // 블록을 놓침 → 탈출
      DEBUG_SERIAL.println(F("    블록 소실, 미세조정 중단"));
      SetMobileGoalVelocityForSyncWrite(dxl, 0, 0, 0, 0);
      return false;
    }

    if (millis() - t0 > CFG.storageGripTarget.alignTimeoutMs)
    {
      DEBUG_SERIAL.println(F("    그립 목표창 정렬 타임아웃"));
      SetMobileGoalVelocityForSyncWrite(dxl, 0, 0, 0, 0);
      storageGripAlignTimedOut = true;
      return false;
    }
  }
}

/*
 * 블록을 미션수행존 goalPos번 칸에 배치
 */
bool placeAtZone(uint8_t goalPos)
{
  DEBUG_SERIAL.print(F("    미션수행존 "));
  DEBUG_SERIAL.print(goalPos);
  DEBUG_SERIAL.println(F("번 칸에 배치..."));

  // SL+FR PSD로 미션수행존 위치까지 좌측 이동. FL/SR은 이 위치에서 불안정해서 쓰지 않는다.
  int16_t slVal;
  int16_t frVal;
  unsigned long t0 = millis();
  while (1)
  {
    GetValueFromSideLeftPSDSensor(&slVal);
    GetValueFromFrontRightPSDSensor(&frVal);
    if (!LocatingWithTwoSensors(dxl, slVal - CFG.psd.missionZoneSl,
                                frVal - CFG.psd.missionZoneFr,
                                CFG.psd.missionZoneTolerance,
                                DRIVE_DIRECTION_LEFT,
                                CFG.speed.psdCorrectionSpeed))
      break;
    if (millis() - t0 > CFG.timeout.psdLoopMs)
    {
      DEBUG_SERIAL.println(F("    미션존 이동 타임아웃"));
      break;
    }
  }
  SetMobileGoalVelocityForSyncWrite(dxl, 0, 0, 0, 0);
  delay(CFG.wait.driveSettleMs);

  // STORAGE 자세에서 motor1을 -90도로 회전 (미션수행존 방향)
  runTimedManipulatorPose(STORAGE, CFG.poseTiming.missionZoneTurnMs, -90.0,
                          F("  미션 수행존 방향으로 팔을 돌리기 위해 3번 안전 이동 자세가 필요합니다."));

  // 미션수행존 배치 자세 (기본값: 7번=1번 칸, 14번=8번 칸)
  uint8_t placePoseId = missionZonePoseIdForGoal(goalPos);
  DEBUG_SERIAL.print(F("    배치 자세 EEPROM #"));
  DEBUG_SERIAL.println(placePoseId);
  runTimedManipulatorPose(placePoseId, CFG.poseTiming.missionZonePlaceMs,
                          -360.0,
                          F("  미션 수행존 배치 자세가 없으면 그리퍼를 열지 않습니다."));

  // 그리퍼 열기
  OpenGripper(pixy);
  delay(CFG.wait.gripperActionMs);
  DEBUG_SERIAL.println(F("    블록 배치 완료"));

  // STORAGE 자세로 복귀 (정면 0도)
  runTimedManipulatorPose(STORAGE, CFG.poseTiming.storageMs, 0.0,
                          F("  배치 후 복귀를 위해 3번 안전 이동 자세가 필요합니다."));
  return true;
}

/*
 * 적재함 앞 SL+FL/FR 재정렬
 */
void realignToStorage()
{
  DEBUG_SERIAL.println(F("    적재함 재정렬..."));
  alignStorageReturnPosition(F("    미션수행존 -> 적재함 복귀 PSD 재정렬: SL 먼저, FL/FR 나중"));
}

// ============================================================
//  5단계: 적재함 스캔 -> 집기 -> 배치 (핵심 루프)
// ============================================================
void step5_pickAndPlace()
{
  DEBUG_SERIAL.println(F(""));
  DEBUG_SERIAL.println(F("========================================"));
  DEBUG_SERIAL.println(F(" [5단계] 적재함 전체 스캔 -> 집기 & 배치"));
  DEBUG_SERIAL.println(F("========================================"));

  if (totalBlocks == 0)
  {
    DEBUG_SERIAL.println(F("  블록 없음, 건너뜀"));
    return;
  }

  placedBlockCount = 0;
  skippedBlockCount = 0;
  allDetectedBlocksPlaced = false;
  ChangeMobilebaseMode2VelocityControlMode(dxl);

  // STORAGE 자세 + 카메라 램프 확인
  runTimedManipulatorPose(STORAGE, CFG.poseTiming.storageMs, 0.0,
                          F("  적재함 스캔 전 3번 안전 이동 자세가 필요합니다."));
  pixy.setLamp(1, 1);
  delay(CFG.wait.cameraLampMs);

  uint8_t currentStorageColumn = 1;
  static StorageDetection detections[MissionConfig::MAX_MISSION_BLOCKS];
  static MissionPickTask tasks[MissionConfig::MAX_MISSION_BLOCKS];
  uint8_t detectionCount = 0;
  uint8_t requestedBlockCount = totalBlocks;

  if (!runStorageSurvey(detections, &detectionCount, &currentStorageColumn))
  {
    skippedBlockCount = requestedBlockCount;
    DEBUG_SERIAL.println(F("  [스킵] 적재함 survey에서 목표 signature를 찾지 못했습니다."));
    return;
  }

  uint8_t taskCount = buildMissionPickTasks(detections, detectionCount,
                                            tasks, currentStorageColumn);
  if (taskCount < requestedBlockCount)
  {
    skippedBlockCount += requestedBlockCount - taskCount;
    DEBUG_SERIAL.print(F("  [주의] 미션지시존 target 중 survey에서 못 찾은 개수="));
    DEBUG_SERIAL.println(requestedBlockCount - taskCount);
  }
  if (taskCount == 0)
  {
    DEBUG_SERIAL.println(F("  [스킵] 실행 가능한 집기 task가 없습니다."));
    return;
  }

  for (uint8_t taskIdx = 0; taskIdx < taskCount; taskIdx++)
  {
    MissionPickTask &task = tasks[taskIdx];
    uint8_t targetSig = task.signature;
    uint8_t sourceSlot = task.sourceSlot;
    uint8_t goalPos = task.goalSlot;
    uint8_t targetSigmap = getStorageTargetSignatureMap(targetSig);

    if (targetSigmap == 0)
    {
      DEBUG_SERIAL.print(F("  [정지] 스토리지 signature 필터에서 Sig"));
      DEBUG_SERIAL.print(targetSig);
      DEBUG_SERIAL.println(F("이 비활성화되어 있거나 잘못된 signature입니다."));
      haltWithRedBlink(F("  [설정 오류] 스토리지 signature 필터 오류"),
                       F("  MissionConfig.h의 storageAllowedSignatureMap/Pixy signature를 확인하세요."));
    }

    printPixyRecognitionMode(F("  [스토리지 signature 필터]"), targetSigmap);

    DEBUG_SERIAL.print(F("\n  === 블록 "));
    DEBUG_SERIAL.print(taskIdx + 1);
    DEBUG_SERIAL.print(F("/"));
    DEBUG_SERIAL.print(taskCount);
    DEBUG_SERIAL.print(F(": Sig"));
    DEBUG_SERIAL.print(targetSig);
    DEBUG_SERIAL.print(F(", StorageSlot "));
    DEBUG_SERIAL.print(sourceSlot);
    DEBUG_SERIAL.print(F(", Column "));
    DEBUG_SERIAL.print(task.column);
    DEBUG_SERIAL.print(F(" -> Zone "));
    DEBUG_SERIAL.print(goalPos);
    DEBUG_SERIAL.println(F(" ==="));

    bool found = false;
    bool skipCurrentBlock = false;
    if (!moveStorageToColumn(&currentStorageColumn, task.column))
    {
      skipCurrentBlock = true;
      DEBUG_SERIAL.println(F("  [스킵] 목표 열 이동 실패"));
    }
    else
    {
      int16_t blockX = 0;
      int16_t blockY = 0;
      uint8_t blockSig = 0;
      uint8_t pickupRegion = 0;
      if (scanCurrentStorageColumnForTarget(sourceSlot,
                                            targetSigmap,
                                            currentStorageColumn,
                                            &blockX, &blockY,
                                            &blockSig, &pickupRegion))
      {
        DEBUG_SERIAL.print(F("  목표 영역 감지 sig="));
        DEBUG_SERIAL.print(blockSig);
        DEBUG_SERIAL.print(F(" X="));
        DEBUG_SERIAL.print(blockX);
        DEBUG_SERIAL.print(F(" Y="));
        DEBUG_SERIAL.print(blockY);
        DEBUG_SERIAL.print(F(" region="));
        DEBUG_SERIAL.println(pickupRegion);

        bool picked = fineTuneAndPickStorageSlot(sourceSlot, targetSigmap);
        if (!picked && storageGripAlignTimedOut)
          skipCurrentBlock = true;
        if (picked && placeAtZone(goalPos))
          found = true;
      }
    }

    if (found)
    {
      placedBlockCount++;
      DEBUG_SERIAL.println(F("  블록 처리 완료!"));
      setRGBLEDGreen();
      delay(CFG.wait.blockFeedbackMs);
      setRGBLEDOff();
    }
    else if (skipCurrentBlock)
    {
      skippedBlockCount++;
      DEBUG_SERIAL.println(F("  [스킵] 열 이동 또는 그립 정렬 실패로 다음 블록으로 넘어갑니다."));
      setRGBLEDRed();
      delay(CFG.wait.blockFeedbackMs);
      setRGBLEDOff();
    }
    else
    {
      skippedBlockCount++;
      DEBUG_SERIAL.print(F("  [스킵] 적재함 Slot "));
      DEBUG_SERIAL.print(sourceSlot);
      DEBUG_SERIAL.println(F("에 해당하는 pickup region 블록을 찾지 못했습니다."));
      DEBUG_SERIAL.println(F("  같은 블록을 반복하지 않고 다음 source slot으로 넘어갑니다."));
      setRGBLEDRed();
      delay(CFG.wait.blockFeedbackMs);
      setRGBLEDOff();
    }

    // 글로벌 시간 체크: 복귀 시간 확보
    if (millis() - missionStartTime > CFG.timeout.missionTimeLimitMs)
    {
      DEBUG_SERIAL.println(F("  [시간] 제한 임박! 남은 블록 건너뛰고 복귀"));
      break;
    }

    // 다음 블록을 위해 적재함 재정렬
    if (taskIdx < taskCount - 1)
    {
      realignToStorage();
      currentStorageColumn = 1;
    }
  }

  allDetectedBlocksPlaced = (placedBlockCount == requestedBlockCount && requestedBlockCount > 0);
  DEBUG_SERIAL.println(F(""));
  if (allDetectedBlocksPlaced)
  {
    DEBUG_SERIAL.println(F("  감지했던 signature 블록을 모두 배치했습니다."));
    DEBUG_SERIAL.println(F("  적재함으로 재정렬하지 않고 현재 미션수행존 위치에서 바로 복귀 후진을 시작합니다."));
  }
  else
  {
    DEBUG_SERIAL.print(F("  배치 완료 수="));
    DEBUG_SERIAL.print(placedBlockCount);
    DEBUG_SERIAL.print(F("/"));
    DEBUG_SERIAL.print(requestedBlockCount);
    DEBUG_SERIAL.print(F(", 스킵="));
    DEBUG_SERIAL.println(skippedBlockCount);
    DEBUG_SERIAL.println(F("  일부 블록은 미배치 상태입니다. 현재 위치 기준으로 안전 복귀합니다."));
  }
}

// ============================================================
//  6단계: Finish Zone 복귀
// ============================================================
void step6_return()
{
  DEBUG_SERIAL.println(F(""));
  DEBUG_SERIAL.println(F("========================================"));
  DEBUG_SERIAL.println(F(" [6단계] 복귀"));
  DEBUG_SERIAL.println(F("========================================"));
  if (allDetectedBlocksPlaced)
  {
    DEBUG_SERIAL.println(F("  전체 배치 완료 상태: 현재 미션수행존 위치에서 바로 후진 복귀합니다."));
  }
  else
  {
    DEBUG_SERIAL.println(F("  전체 배치 미완료/시간초과 가능성 있음: 현재 위치 기준으로 복귀합니다."));
  }

  // STORAGE 자세 → INITIAL 자세
  runTimedManipulatorPose(STORAGE, CFG.poseTiming.finishStorageMs, 0.0,
                          F("  복귀 전 3번 안전 이동 자세가 필요합니다."));
  runTimedManipulatorPose(INITIAL_AND_MISSION_INSTRUCTION, CFG.poseTiming.finishInitialMs,
                          -360.0,
                          F("  복귀 전 1번 초기 자세가 필요합니다."));

  // 카메라 램프 OFF
  pixy.setLamp(0, 0);
  DEBUG_SERIAL.println(F("  카메라 램프 OFF"));

  ChangeMobilebaseMode2VelocityControlMode(dxl);

  int16_t slVal;
  unsigned long t0;

  // ─── 복귀 1단계: 장애물 없어질 때까지 후진 ───
  DEBUG_SERIAL.println(F("  [6-1] SL 장애물 없어질 때까지 후진..."));
  t0 = millis();
  while (1)
  {
    GetValueFromSideLeftPSDSensor(&slVal);
    if (slVal <= CFG.finishReturn.boundaryAdc)
      break;
    driveBackwardWithLeftBoundaryCorrection(slVal);
    if (millis() - t0 > CFG.timeout.returnPhaseMs)
    {
      DEBUG_SERIAL.println(F("  [6-1] 타임아웃"));
      break;
    }
    delay(10);
  }
  SetMobileGoalVelocityForSyncWrite(dxl, 0, 0, 0, 0);
  delay(CFG.wait.driveSettleMs);

  // ─── 복귀 2단계: 장애물 다시 감지될 때까지 후진 ───
  DEBUG_SERIAL.println(F("  [6-2] SL 장애물 다시 감지될 때까지 후진..."));
  t0 = millis();
  while (1)
  {
    GetValueFromSideLeftPSDSensor(&slVal);
    driveBackwardWithLeftBoundaryCorrection(slVal);
    if (slVal > CFG.finishReturn.boundaryAdc)
    {
      SetMobileGoalVelocityForSyncWrite(dxl, 0, 0, 0, 0);
      DEBUG_SERIAL.println(F("  장애물 다시 감지"));
      break;
    }
    if (millis() - t0 > CFG.timeout.returnPhaseMs)
    {
      SetMobileGoalVelocityForSyncWrite(dxl, 0, 0, 0, 0);
      DEBUG_SERIAL.println(F("  [6-2] 타임아웃"));
      break;
    }
    delay(10);
  }
  delay(CFG.wait.driveSettleMs);

  // ─── 복귀 3단계: 장애물 다시 없어질 때까지 후진 ───
  DEBUG_SERIAL.println(F("  [6-3] SL 장애물 없어질 때까지 후진..."));
  t0 = millis();
  while (1)
  {
    GetValueFromSideLeftPSDSensor(&slVal);
    if (slVal <= CFG.finishReturn.boundaryAdc)
      break;
    driveBackwardWithLeftBoundaryCorrection(slVal);
    if (millis() - t0 > CFG.timeout.returnPhaseMs)
    {
      DEBUG_SERIAL.println(F("  [6-3] 타임아웃"));
      break;
    }
    delay(10);
  }
  SetMobileGoalVelocityForSyncWrite(dxl, 0, 0, 0, 0);
  delay(CFG.wait.driveSettleMs);

  // ─── 복귀 4단계: Finish Zone 진입 추가 후진 ───
  DEBUG_SERIAL.print(F("  [6-4] Finish Zone 진입 ("));
  DEBUG_SERIAL.print(CFG.finishReturn.finishExtraMs);
  DEBUG_SERIAL.println(F("ms)..."));
  t0 = millis();
  while (millis() - t0 < CFG.finishReturn.finishExtraMs)
  {
    GetValueFromSideLeftPSDSensor(&slVal);
    driveBackwardWithLeftBoundaryCorrection(slVal);
    delay(10);
  }
  SetMobileGoalVelocityForSyncWrite(dxl, 0, 0, 0, 0);

  // 최종 정지, INITIAL 자세
  runTimedManipulatorPose(INITIAL_AND_MISSION_INSTRUCTION, CFG.poseTiming.finishInitialMs,
                          -360.0,
                          F("  미션 완료 시 1번 초기 자세로 복귀해야 합니다."));

  // LED 끄기
  setRGBLEDOff();
  digitalWrite(RED_LED_PIN, LOW);

  DEBUG_SERIAL.println(F(""));
  DEBUG_SERIAL.println(F("========================================"));
  DEBUG_SERIAL.println(F(" 미션 완료!"));
  DEBUG_SERIAL.println(F("========================================"));

  // 완료 표시
  for (int i = 0; i < 5; i++)
  {
    setRGBLEDGreen();
    delay(300);
    setRGBLEDOff();
    delay(300);
  }
}

// ============================================================
//  Arduino setup() & loop()
// ============================================================
// 남은 시간이 부족하면 블록 수를 줄여 복귀 시간 확보
void adjustBlockCountByTime()
{
  unsigned long elapsed = millis() - missionStartTime;
  if (elapsed > 100000)
  { // 100초 초과: 1개만
    totalBlocks = min(totalBlocks, (uint8_t)1);
  }
  else if (elapsed > 80000)
  { // 80초 초과: 2개만
    totalBlocks = min(totalBlocks, (uint8_t)2);
  }
  else if (elapsed > 60000)
  { // 60초 초과: 3개만
    totalBlocks = min(totalBlocks, (uint8_t)3);
  }
  DEBUG_SERIAL.print(F("  시간 경과: "));
  DEBUG_SERIAL.print(elapsed / 1000);
  DEBUG_SERIAL.print(F("초, 처리할 블록: "));
  DEBUG_SERIAL.println(totalBlocks);
}

void setup()
{
  step1_init();
  missionStartTime = millis();

  step2_approach();
  step3_scan();
  if (totalBlocks > 0)
  {
    step4_alignToStorage();

    // 시간 체크 → 블록 수 조정
    adjustBlockCountByTime();

    step5_pickAndPlace();
  }
  else
  {
    DEBUG_SERIAL.println(F("  처리할 블록이 없어 적재함 이동/집기를 건너뜁니다."));
  }
  step6_return();
}

void loop()
{
  delay(1000);
}
