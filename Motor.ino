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
 *   2  : STORAGE (팔 접힘, 안전 이동 자세)
 *   3  : PRE_GRIP_UPPER (상층 집기 준비)
 *   4  : GRIP_UPPER     (상층 집기)
 *   5  : PRE_GRIP_LOWER (하층 집기 준비)
 *   6  : GRIP_LOWER     (하층 집기)
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
  STORAGE,              // 2: 팔 접힘 (이동 안전, -90도 회전 기반)
  PRE_GRIP_UPPER_BLOCK, // 3: 상층 블록 집기 준비
  GRIP_UPPER_BLOCK,     // 4: 상층 블록 집기
  PRE_GRIP_LOWER_BLOCK, // 5: 하층 블록 집기 준비
  GRIP_LOWER_BLOCK      // 6: 하층 블록 집기
};

// 글로벌 미션 시작 시각
unsigned long missionStartTime = 0;

// ============================================================
//  블록 스캔 결과
// ============================================================
uint8_t targetSigs[MissionConfig::MAX_MISSION_BLOCKS];    // 순서별 시그니처 (step3에서 자동 채움)
uint8_t goalPositions[MissionConfig::MAX_MISSION_BLOCKS]; // 순서별 목표 칸 번호 (1~8)
uint8_t totalBlocks = 0;                                  // 인식된 총 블록 수

int32_t clampInt32(int32_t value, int32_t minValue, int32_t maxValue)
{
  if (value < minValue)
    return minValue;
  if (value > maxValue)
    return maxValue;
  return value;
}

bool readLargestTargetBlock(uint8_t targetSigmap, int16_t *blockX, int16_t *blockY)
{
  pixy.ccc.getBlocks(true, targetSigmap);
  if (pixy.ccc.numBlocks == 0)
    return false;

  uint8_t bestIndex = 0;
  uint32_t bestArea = 0;
  for (uint8_t i = 0; i < pixy.ccc.numBlocks; i++)
  {
    uint32_t area = (uint32_t)pixy.ccc.blocks[i].m_width * (uint32_t)pixy.ccc.blocks[i].m_height;
    if (area > bestArea)
    {
      bestArea = area;
      bestIndex = i;
    }
  }

  *blockX = pixy.ccc.blocks[bestIndex].m_x;
  *blockY = pixy.ccc.blocks[bestIndex].m_y;
  return true;
}

uint8_t getMissionInstructionSignatureMap()
{
  uint8_t signatureMap = CFG.cameraScan.missionInstructionAllowedSignatureMap &
                         CFG.mission.blockSignatureMap;
  if (signatureMap == 0)
    signatureMap = CFG.mission.blockSignatureMap;
  return signatureMap;
}

uint8_t getStorageTargetSignatureMap(uint8_t targetSig)
{
  if (targetSig < 1 || targetSig > CFG.cameraScan.maxSignature)
    return 0;
  return (1 << (targetSig - 1)) & CFG.cameraScan.storageAllowedSignatureMap;
}

void printPixyRecognitionMode(const __FlashStringHelper *label, uint8_t signatureMap)
{
  DEBUG_SERIAL.print(label);
  DEBUG_SERIAL.print(F(" Pixy signature filter=0b"));
  for (int bit = 7; bit >= 0; bit--)
  {
    DEBUG_SERIAL.print((signatureMap >> bit) & 0x01);
  }
  DEBUG_SERIAL.println();
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

void waitForSW1Continue(const __FlashStringHelper *message)
{
  SetMobileGoalVelocityForSyncWrite(dxl, 0, 0, 0, 0);
  setRGBLEDBlue();
  unsigned long pauseStartTime = millis();

  DEBUG_SERIAL.println(F(""));
  DEBUG_SERIAL.println(message);
  DEBUG_SERIAL.println(F("  SW1 버튼을 누르면 계속 진행합니다."));

  while (digitalRead(SW1_PIN) == HIGH)
  {
    delay(10);
  }
  delay(CFG.wait.buttonDebounceMs);
  while (digitalRead(SW1_PIN) == LOW)
  {
    delay(10);
  }
  delay(CFG.wait.buttonDebounceMs);
  setRGBLEDOff();

  if (missionStartTime > 0)
  {
    missionStartTime += millis() - pauseStartTime;
  }
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

uint8_t missionZonePoseIdForGoal(uint8_t goalPos)
{
  if (goalPos < 1 || goalPos > 8)
  {
    haltWithRedBlink(F("  [설정 오류] 미션 수행존 번호가 1~8 범위를 벗어났습니다."),
                     F("  MissionConfig.h의 goalPositions 값을 확인하세요."));
  }

  return CFG.pose.missionZoneStartId + goalPos;
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

  // 그리퍼 열기, INITIAL 자세
  OpenGripper(pixy);
  runRequiredManipulatorPose(INITIAL_AND_MISSION_INSTRUCTION, 2000,
                             -360.0,
                             F("  1번 자세는 초기 자세와 미션지시존 카메라 자세로 사용됩니다."));
  SetMobileGoalVelocityForSyncWrite(dxl, 0, 0, 0, 0);
  delay(2500);

  // 스캔 결과 초기화
  totalBlocks = 0;
  memset(targetSigs, 0, sizeof(targetSigs));
  memset(goalPositions, 0, sizeof(goalPositions));

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

  waitForSW1Continue(F("  [일시정지] 미션지시존에 정지했습니다. 블록 스캔 전 위치를 확인하세요."));
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
  memset(targetSigs, 0, sizeof(targetSigs));
  memset(goalPositions, 0, sizeof(goalPositions));

  // INITIAL 자세에서 미션지시존을 카메라로 촬영
  // (step2 후 이미 INITIAL 자세이므로 확인용)
  runRequiredManipulatorPose(INITIAL_AND_MISSION_INSTRUCTION, 1000,
                             -360.0,
                             F("  미션지시존 스캔 전에 1번 초기 자세가 필요합니다."));
  delay(CFG.wait.pose1000Ms);

  // CFG.mission.blockSignatureMap으로 관심 시그니처만 필터링하여 스캔
  // 여러 프레임 중 최다 블록 인식 결과 사용
  int16_t bestBlockX[MissionConfig::MAX_MISSION_BLOCKS];
  uint8_t bestBlockSig[MissionConfig::MAX_MISSION_BLOCKS];
  uint8_t bestCount = 0;
  uint8_t missionInstructionSignatureMap = getMissionInstructionSignatureMap();
  printPixyRecognitionMode(F("  [미션지시존 signature 필터]"), missionInstructionSignatureMap);

  while (bestCount == 0)
  {
    for (int attempt = 0; attempt < 20; attempt++)
    {
      pixy.ccc.getBlocks(true, missionInstructionSignatureMap);
      if (pixy.ccc.numBlocks > 0 && pixy.ccc.numBlocks >= bestCount)
      {
        bestCount = min((uint8_t)pixy.ccc.numBlocks, (uint8_t)MissionConfig::MAX_MISSION_BLOCKS);
        for (int i = 0; i < bestCount; i++)
        {
          bestBlockX[i] = pixy.ccc.blocks[i].m_x;
          bestBlockSig[i] = pixy.ccc.blocks[i].m_signature;
        }
      }
      delay(CFG.wait.scanSampleMs);
    }

    if (bestCount == 0)
    {
      DEBUG_SERIAL.println(F("  [정지] 미션지시존 블록을 인식하지 못했습니다."));
      DEBUG_SERIAL.println(F("  조명/카메라/블록 위치를 확인한 뒤 다시 스캔하세요."));
      waitForSW1Continue(F("  블록 미인식 상태입니다. SW1을 누르면 미션지시존 스캔을 다시 시도합니다."));
    }
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

  // 카메라 인식 수를 대회 설정 블록 수로 제한
  totalBlocks = min(bestCount, (uint8_t)CFG.mission.blockCount);

  // 결과 저장: i번째 블록 → 대회 당일 발표된 칸에 배치
  for (int i = 0; i < totalBlocks; i++)
  {
    targetSigs[i] = bestBlockSig[i];
    goalPositions[i] = CFG.mission.goalPositions[i]; // 사전 설정된 칸 번호
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
    DEBUG_SERIAL.print(F(" -> Zone "));
    DEBUG_SERIAL.println(goalPositions[i]);
  }
}

// ============================================================
//  4단계: 적재함 앞 접근 & 정밀 정렬
// ============================================================
void step4_alignToStorage()
{
  DEBUG_SERIAL.println(F(""));
  DEBUG_SERIAL.println(F("========================================"));
  DEBUG_SERIAL.println(F(" [4단계] 적재함 접근 & 정렬"));
  DEBUG_SERIAL.println(F("========================================"));

  // STORAGE 자세 (팔 접기, 이동 안전)
  runRequiredManipulatorPose(STORAGE, 1000, 0.0,
                             F("  적재함으로 이동하기 전 2번 안전 이동 자세가 필요합니다."));
  delay(CFG.wait.pose1000Ms);
  DEBUG_SERIAL.println(F("  STORAGE 자세 완료"));

  // --- 거리 기반 전진: 위치제어 모드 ---
  DEBUG_SERIAL.print(F("  [4-1] 위치제어 "));
  DEBUG_SERIAL.print(CFG.storageDrive.firstForwardMm);
  DEBUG_SERIAL.println(F("mm 전진..."));
  ChangeMobilebaseMode2ExtendedPositionControlWithTimeBasedProfileMode(dxl);

  DriveDistanceAndMmPerSecAndDirection(dxl, CFG.storageDrive.firstForwardMm,
                                       DRIVE_DIRECTION_FORWARD,
                                       CFG.speed.positionMoveMmPerSec);
  {
    unsigned long pt = millis();
    while (!CheckIfMobilebaseIsInPosition(dxl))
    {
      if (millis() - pt > CFG.timeout.positionMoveMs)
      {
        DEBUG_SERIAL.println(F("  전진 타임아웃"));
        break;
      }
    }
  }
  DEBUG_SERIAL.println(F("  전진 완료"));

  // --- 추가 전진 ---
  DEBUG_SERIAL.println(F("  [4-2] 추가 전진..."));
  DriveDistanceAndMmPerSecAndDirection(dxl, CFG.storageDrive.extraForwardMm,
                                       DRIVE_DIRECTION_FORWARD,
                                       CFG.speed.positionMoveMmPerSec);
  {
    unsigned long pt = millis();
    while (!CheckIfMobilebaseIsInPosition(dxl))
    {
      if (millis() - pt > CFG.timeout.positionMoveMs)
      {
        DEBUG_SERIAL.println(F("  전진 타임아웃"));
        break;
      }
    }
  }
  DEBUG_SERIAL.println(F("  추가 전진 완료"));

  // --- 우측 이동 ---
  DEBUG_SERIAL.println(F("  [4-3] 우측 이동..."));
  DriveDistanceAndMmPerSecAndDirection(dxl, CFG.storageDrive.rightMm,
                                       DRIVE_DIRECTION_RIGHT,
                                       CFG.speed.positionMoveMmPerSec);
  {
    unsigned long pt = millis();
    while (!CheckIfMobilebaseIsInPosition(dxl))
    {
      if (millis() - pt > CFG.timeout.positionMoveMs)
      {
        DEBUG_SERIAL.println(F("  우측 타임아웃"));
        break;
      }
    }
  }
  DEBUG_SERIAL.println(F("  우측 이동 완료"));

  // --- SL+FR 2축 정밀 정렬: 속도제어 모드로 전환 ---
  DEBUG_SERIAL.println(F("  [4-4] SL+FR 2축 정밀 정렬..."));
  ChangeMobilebaseMode2VelocityControlMode(dxl);

  int16_t slVal, frVal;
  unsigned long t0 = millis();
  while (1)
  {
    GetValueFromSideLeftPSDSensor(&slVal);
    GetValueFromFrontRightPSDSensor(&frVal);
    if (!LocatingWithTwoSensors(dxl, slVal - CFG.psd.alignSl,
                                frVal - CFG.psd.alignFr, CFG.psd.alignTolerance,
                                DRIVE_DIRECTION_LEFT,
                                CFG.speed.psdCorrectionSpeed))
      break;
    if (millis() - t0 > CFG.timeout.psdLoopMs)
    {
      DEBUG_SERIAL.println(F("  [4-3] 타임아웃"));
      break;
    }
  }
  SetMobileGoalVelocityForSyncWrite(dxl, 0, 0, 0, 0);
  DEBUG_SERIAL.print(F("  정렬 완료 SL="));
  DEBUG_SERIAL.print(slVal);
  DEBUG_SERIAL.print(F(" FR="));
  DEBUG_SERIAL.println(frVal);
  delay(CFG.wait.driveSettleMs);

  // 카메라 램프 ON
  pixy.setLamp(1, 1);
  delay(CFG.wait.cameraLampMs);
  DEBUG_SERIAL.println(F("  카메라 램프 ON"));
}

// ============================================================
//  5단계 헬퍼: 적재함에서 블록 집기 (2페이즈 스캔)
// ============================================================

/*
 * 카메라에 목표 signature 블록이 잡혔을 때 미세조정 → 상/하층 판별 → 집기까지 수행
 * 반환: true=집기 성공, false=실패
 */
bool fineTuneAndPick(uint8_t targetSigmap)
{
  SetMobileGoalVelocityForSyncWrite(dxl, 0, 0, 0, 0);
  DEBUG_SERIAL.println(F("    목표 블록 감지 → 미세조정 시작"));

  unsigned long t0 = millis();
  while (1)
  {
    int16_t blockX = 0;
    int16_t blockY = 0;

    if (readLargestTargetBlock(targetSigmap, &blockX, &blockY))
    {
      int16_t blockXError = blockX - CFG.cameraScan.storageXSetpoint;

      if (!DriveWithOneSensor(dxl, blockXError, CFG.cameraScan.storageXTolerance,
                              DRIVE_DIRECTION_LEFT,
                              CFG.speed.cameraFineTuneSpeed))
      {
        // 미세조정 완료 → 집기 실행
        SetMobileGoalVelocityForSyncWrite(dxl, 0, 0, 0, 0);
        DEBUG_SERIAL.println(F("    미세조정 완료"));

        // 상/하층 판별
        if (blockY < CFG.cameraScan.storageYUpperLowerSplit)
        {
          // 상층 블록
          DEBUG_SERIAL.println(F("    상층 블록 집기"));
          runRequiredManipulatorPose(PRE_GRIP_UPPER_BLOCK, 1000, 0.0,
                                     F("  상층 블록 집기 준비 자세가 필요합니다."));
          delay(CFG.wait.pose1000Ms);
          runRequiredManipulatorPose(GRIP_UPPER_BLOCK, 1000, 0.0,
                                     F("  상층 블록 집기 자세가 필요합니다."));
          delay(CFG.wait.pose1500Ms);
        }
        else
        {
          // 하층 블록
          DEBUG_SERIAL.println(F("    하층 블록 집기"));
          runRequiredManipulatorPose(PRE_GRIP_LOWER_BLOCK, 1000, 0.0,
                                     F("  하층 블록 집기 준비 자세가 필요합니다."));
          delay(CFG.wait.pose1000Ms);
          runRequiredManipulatorPose(GRIP_LOWER_BLOCK, 1000, 0.0,
                                     F("  하층 블록 집기 자세가 필요합니다."));
          delay(CFG.wait.pose1500Ms);
        }

        // 그리퍼 닫기
        CloseGripper(pixy);
        delay(CFG.wait.gripperActionMs);
        DEBUG_SERIAL.println(F("    그리퍼 닫기 완료"));

        // STORAGE 자세로 접기 (이동 안전)
        runRequiredManipulatorPose(STORAGE, 1000, 0.0,
                                   F("  집은 블록을 들고 이동하기 전 2번 안전 이동 자세가 필요합니다."));
        delay(CFG.wait.pose1500Ms);

        return true;
      }
      // else: 미세조정 진행 중
    }
    else
    {
      // 블록을 놓침 → 탈출
      DEBUG_SERIAL.println(F("    블록 소실, 미세조정 중단"));
      SetMobileGoalVelocityForSyncWrite(dxl, 0, 0, 0, 0);
      return false;
    }

    if (millis() - t0 > CFG.timeout.fineTuneMs)
    {
      DEBUG_SERIAL.println(F("    미세조정 타임아웃"));
      SetMobileGoalVelocityForSyncWrite(dxl, 0, 0, 0, 0);
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

  // SL PSD로 미션수행존 위치까지 좌측 이동
  int16_t slVal;
  unsigned long t0 = millis();
  while (1)
  {
    GetValueFromSideLeftPSDSensor(&slVal);
    if (!DriveWithOneSensor(dxl, slVal - CFG.psd.missionZoneSl,
                            CFG.psd.missionZoneTolerance, DRIVE_DIRECTION_LEFT,
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
  runRequiredManipulatorPose(STORAGE, 800, -90.0,
                             F("  미션 수행존 방향으로 팔을 돌리기 위해 2번 안전 이동 자세가 필요합니다."));
  delay(CFG.wait.pose800Ms);

  // 미션수행존 배치 자세 (기본값: 7번=1번 칸, 14번=8번 칸)
  uint8_t placePoseId = missionZonePoseIdForGoal(goalPos);
  DEBUG_SERIAL.print(F("    배치 자세 EEPROM #"));
  DEBUG_SERIAL.println(placePoseId);
  runRequiredManipulatorPose(placePoseId, 1000,
                             -360.0,
                             F("  미션 수행존 배치 자세가 없으면 그리퍼를 열지 않습니다."));
  delay(CFG.wait.pose1300Ms);

  // 그리퍼 열기
  OpenGripper(pixy);
  delay(CFG.wait.gripperActionMs);
  DEBUG_SERIAL.println(F("    블록 배치 완료"));

  // STORAGE 자세로 복귀 (정면 0도)
  runRequiredManipulatorPose(STORAGE, 1000, 0.0,
                             F("  배치 후 복귀를 위해 2번 안전 이동 자세가 필요합니다."));
  delay(CFG.wait.pose1500Ms);
  return true;
}

/*
 * 적재함 앞 SL+FR 재정렬
 */
void realignToStorage()
{
  DEBUG_SERIAL.println(F("    적재함 재정렬..."));
  int16_t slVal, frVal;
  unsigned long t0 = millis();
  while (1)
  {
    GetValueFromSideLeftPSDSensor(&slVal);
    GetValueFromFrontRightPSDSensor(&frVal);
    if (!LocatingWithTwoSensors(dxl, slVal - CFG.psd.alignSl,
                                frVal - CFG.psd.alignFr, CFG.psd.alignTolerance,
                                DRIVE_DIRECTION_LEFT,
                                CFG.speed.psdCorrectionSpeed))
      break;
    if (millis() - t0 > CFG.timeout.psdLoopMs)
    {
      DEBUG_SERIAL.println(F("    재정렬 타임아웃"));
      break;
    }
  }
  SetMobileGoalVelocityForSyncWrite(dxl, 0, 0, 0, 0);
  delay(CFG.wait.driveSettleMs);
}

// ============================================================
//  5단계: 적재함 스캔 -> 집기 -> 배치 (핵심 루프)
// ============================================================
void step5_pickAndPlace()
{
  DEBUG_SERIAL.println(F(""));
  DEBUG_SERIAL.println(F("========================================"));
  DEBUG_SERIAL.println(F(" [5단계] 집기 & 배치 루프"));
  DEBUG_SERIAL.println(F("========================================"));

  if (totalBlocks == 0)
  {
    DEBUG_SERIAL.println(F("  블록 없음, 건너뜀"));
    return;
  }

  ChangeMobilebaseMode2VelocityControlMode(dxl);

  for (int blockIdx = 0; blockIdx < totalBlocks; blockIdx++)
  {
    uint8_t targetSig = targetSigs[blockIdx];
    uint8_t goalPos = goalPositions[blockIdx];

    uint8_t targetSigmap = getStorageTargetSignatureMap(targetSig);
    if (targetSigmap == 0)
    {
      DEBUG_SERIAL.print(F("  [정지] 스토리지 signature 필터에서 Sig"));
      DEBUG_SERIAL.println(targetSig);
      DEBUG_SERIAL.println(F("  이 비활성화되어 있거나 잘못된 signature입니다."));
      waitForSW1Continue(F("  MissionConfig.h의 storageAllowedSignatureMap을 확인한 뒤 SW1을 누르면 다시 시도합니다."));
      blockIdx--;
      continue;
    }

    printPixyRecognitionMode(F("  [스토리지 signature 필터]"), targetSigmap);

    DEBUG_SERIAL.print(F("\n  === 블록 "));
    DEBUG_SERIAL.print(blockIdx + 1);
    DEBUG_SERIAL.print(F("/"));
    DEBUG_SERIAL.print(totalBlocks);
    DEBUG_SERIAL.print(F(": Sig"));
    DEBUG_SERIAL.print(targetSig);
    DEBUG_SERIAL.print(F(" -> Zone "));
    DEBUG_SERIAL.print(goalPos);
    DEBUG_SERIAL.println(F(" ==="));

    // STORAGE 자세 + 카메라 램프 확인
    runRequiredManipulatorPose(STORAGE, 800, 0.0,
                               F("  적재함 스캔 전 2번 안전 이동 자세가 필요합니다."));
    delay(CFG.wait.pose800Ms);
    pixy.setLamp(1, 1);
    delay(CFG.wait.cameraLampMs);

    bool found = false;

    // ─── Phase 1: FR PSD로 우측 이동하며 탐색 ───
    DEBUG_SERIAL.println(F("  [Phase 1] FR PSD 우측 스캔..."));
    int16_t frVal;
    unsigned long t0 = millis();
    do
    {
      // 카메라로 목표 시그니처 블록 확인
      int16_t blockX = 0;
      int16_t blockY = 0;
      if (readLargestTargetBlock(targetSigmap, &blockX, &blockY))
      {
        DEBUG_SERIAL.print(F("  목표 블록 감지 X="));
        DEBUG_SERIAL.print(blockX);
        DEBUG_SERIAL.print(F(" Y="));
        DEBUG_SERIAL.println(blockY);
        if (fineTuneAndPick(targetSigmap) && placeAtZone(goalPos))
        {
          found = true;
          break;
        }
      }

      GetValueFromFrontRightPSDSensor(&frVal);

      if (millis() - t0 > CFG.timeout.scanPhaseMs)
      {
        DEBUG_SERIAL.println(F("  [Phase 1] 타임아웃"));
        SetMobileGoalVelocityForSyncWrite(dxl, 0, 0, 0, 0);
        break;
      }
    } while (DriveUntilNoObstacleWithOneSensor(dxl, frVal, CFG.psd.storageScanFrNoObstacle,
                                               DRIVE_DIRECTION_RIGHT,
                                               CFG.speed.storageScanSpeed));
    SetMobileGoalVelocityForSyncWrite(dxl, 0, 0, 0, 0);

    // ─── Phase 2: SL PSD로 좌측 이동하며 탐색 ───
    if (!found)
    {
      DEBUG_SERIAL.println(F("  [Phase 2] SL PSD 좌측 스캔..."));
      int16_t slVal;
      t0 = millis();
      do
      {
        int16_t blockX = 0;
        int16_t blockY = 0;
        if (readLargestTargetBlock(targetSigmap, &blockX, &blockY))
        {
          DEBUG_SERIAL.print(F("  목표 블록 감지 X="));
          DEBUG_SERIAL.print(blockX);
          DEBUG_SERIAL.print(F(" Y="));
          DEBUG_SERIAL.println(blockY);
          if (fineTuneAndPick(targetSigmap) && placeAtZone(goalPos))
          {
            found = true;
            break;
          }
        }

        GetValueFromSideLeftPSDSensor(&slVal);

        if (millis() - t0 > CFG.timeout.scanPhaseMs)
        {
          DEBUG_SERIAL.println(F("  [Phase 2] 타임아웃"));
          SetMobileGoalVelocityForSyncWrite(dxl, 0, 0, 0, 0);
          break;
        }
      } while (DriveWithOneSensor(dxl, slVal - CFG.psd.scanSl,
                                  CFG.psd.scanSlTolerance, DRIVE_DIRECTION_LEFT,
                                  CFG.speed.storageScanSpeed));
      SetMobileGoalVelocityForSyncWrite(dxl, 0, 0, 0, 0);
    }

    if (found)
    {
      DEBUG_SERIAL.println(F("  블록 처리 완료!"));
      setRGBLEDGreen();
      delay(CFG.wait.blockFeedbackMs);
      setRGBLEDOff();
    }
    else
    {
      DEBUG_SERIAL.print(F("  [정지] Sig"));
      DEBUG_SERIAL.print(targetSig);
      DEBUG_SERIAL.println(F(" 블록을 찾지 못했습니다."));
      waitForSW1Continue(F("  스토리지 목표 블록 미발견 상태입니다. SW1을 누르면 같은 블록을 다시 탐색합니다."));
      blockIdx--;
      continue;
    }

    // 글로벌 시간 체크: 복귀 시간 확보
    if (millis() - missionStartTime > CFG.timeout.missionTimeLimitMs)
    {
      DEBUG_SERIAL.println(F("  [시간] 제한 임박! 남은 블록 건너뛰고 복귀"));
      break;
    }

    // 다음 블록을 위해 적재함 재정렬
    if (blockIdx < totalBlocks - 1)
    {
      realignToStorage();
    }
  }

  DEBUG_SERIAL.println(F("\n  모든 블록 처리 완료"));
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

  // STORAGE 자세 → INITIAL 자세
  runRequiredManipulatorPose(STORAGE, 800, 0.0,
                             F("  복귀 전 2번 안전 이동 자세가 필요합니다."));
  delay(CFG.wait.pose800Ms);
  runRequiredManipulatorPose(INITIAL_AND_MISSION_INSTRUCTION, 1000,
                             -360.0,
                             F("  복귀 전 1번 초기 자세가 필요합니다."));
  delay(CFG.wait.pose1000Ms);

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
  runRequiredManipulatorPose(INITIAL_AND_MISSION_INSTRUCTION, 1000,
                             -360.0,
                             F("  미션 완료 시 1번 초기 자세로 복귀해야 합니다."));
  delay(CFG.wait.pose1000Ms);

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
  step4_alignToStorage();

  // 시간 체크 → 블록 수 조정
  adjustBlockCountByTime();

  step5_pickAndPlace();
  step6_return();
}

void loop()
{
  delay(1000);
}
