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
#include "Camera.h" // PIXY2_X_SETPOINT, PIXY2_Y_SETPOINT, PIXY2_X_TOLERANCE
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
  ManipulatorPose testPose = RunManipulatorPoseWithPoseDataInEEPROM(INITIAL_AND_MISSION_INSTRUCTION, 2000);
  SetMobileGoalVelocityForSyncWrite(dxl, 0, 0, 0, 0);

  // EEPROM 자세 데이터 검증
  if (!testPose.isTherePoseData)
  {
    DEBUG_SERIAL.println(F("  [오류] EEPROM 자세 1번 데이터 없음!"));
    DEBUG_SERIAL.println(F("  manageManipulatorPose로 자세를 먼저 저장하세요."));
    while (1)
    {
      setRGBLEDRed();
      delay(500);
      setRGBLEDOff();
      delay(500);
    }
  }
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

  // INITIAL 자세에서 미션지시존을 카메라로 촬영
  // (step2 후 이미 INITIAL 자세이므로 확인용)
  RunManipulatorPoseWithPoseDataInEEPROM(INITIAL_AND_MISSION_INSTRUCTION, 1000);
  delay(CFG.wait.pose1000Ms);

  // CFG.mission.blockSignatureMap으로 관심 시그니처만 필터링하여 스캔
  // 여러 프레임 중 최다 블록 인식 결과 사용
  int16_t bestBlockX[MissionConfig::MAX_MISSION_BLOCKS];
  uint8_t bestBlockSig[MissionConfig::MAX_MISSION_BLOCKS];
  uint8_t bestCount = 0;

  for (int attempt = 0; attempt < 20; attempt++)
  {
    pixy.ccc.getBlocks(true, CFG.mission.blockSignatureMap);
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

  // 카메라 인식 수를 대회 설정 블록 수로 제한
  totalBlocks = min(bestCount, (uint8_t)CFG.mission.blockCount);

  if (totalBlocks == 0)
  {
    DEBUG_SERIAL.println(F("  [경고] 블록 미인식! 기본 1개 설정"));
    // 최소 1개는 처리 시도
    totalBlocks = 1;
    targetSigs[0] = 1;
    goalPositions[0] = 1;
    return;
  }

  // X좌표 기준 버블정렬 (왼→오른 = 미션 순서)
  for (int i = 0; i < totalBlocks - 1; i++)
  {
    for (int j = 0; j < totalBlocks - 1 - i; j++)
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
  RunManipulatorPoseWithPoseDataInEEPROM(STORAGE, 1000, 0.0);
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
 * 카메라에 블록이 MissionConfig.cameraScan 범위로 잡혔을 때
 * 미세조정 → 상/하층 판별 → 집기까지 수행
 * 반환: true=집기 성공, false=실패
 */
bool fineTuneAndPick(uint8_t targetSigmap)
{
  SetMobileGoalVelocityForSyncWrite(dxl, 0, 0, 0, 0);
  DEBUG_SERIAL.println(F("    블록 중앙 감지 → 미세조정 시작"));

  unsigned long t0 = millis();
  while (1)
  {
    pixy.ccc.getBlocks(true, targetSigmap);

    if (pixy.ccc.numBlocks)
    {
      int16_t blockXError = pixy.ccc.blocks[0].m_x - PIXY2_X_SETPOINT;

      if (!DriveWithOneSensor(dxl, blockXError, PIXY2_X_TOLERANCE,
                              DRIVE_DIRECTION_LEFT,
                              CFG.speed.cameraFineTuneSpeed))
      {
        // 미세조정 완료 → 집기 실행
        SetMobileGoalVelocityForSyncWrite(dxl, 0, 0, 0, 0);
        DEBUG_SERIAL.println(F("    미세조정 완료"));

        // 상/하층 판별
        if (pixy.ccc.blocks[0].m_y < PIXY2_Y_SETPOINT)
        {
          // 상층 블록
          DEBUG_SERIAL.println(F("    상층 블록 집기"));
          RunManipulatorPoseWithPoseDataInEEPROM(PRE_GRIP_UPPER_BLOCK, 1000, 0.0);
          delay(CFG.wait.pose1000Ms);
          RunManipulatorPoseWithPoseDataInEEPROM(GRIP_UPPER_BLOCK, 1000, 0.0);
          delay(CFG.wait.pose1500Ms);
        }
        else
        {
          // 하층 블록
          DEBUG_SERIAL.println(F("    하층 블록 집기"));
          RunManipulatorPoseWithPoseDataInEEPROM(PRE_GRIP_LOWER_BLOCK, 1000, 0.0);
          delay(CFG.wait.pose1000Ms);
          RunManipulatorPoseWithPoseDataInEEPROM(GRIP_LOWER_BLOCK, 1000, 0.0);
          delay(CFG.wait.pose1500Ms);
        }

        // 그리퍼 닫기
        CloseGripper(pixy);
        delay(CFG.wait.gripperActionMs);
        DEBUG_SERIAL.println(F("    그리퍼 닫기 완료"));

        // STORAGE 자세로 접기 (이동 안전)
        RunManipulatorPoseWithPoseDataInEEPROM(STORAGE, 1000, 0.0);
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
void placeAtZone(uint8_t goalPos)
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
  RunManipulatorPoseWithPoseDataInEEPROM(STORAGE, 800, -90.0);
  delay(CFG.wait.pose800Ms);

  // 미션수행존 배치 자세 (CFG.pose.missionZoneStartId + goalPos)
  uint8_t placePoseId = CFG.pose.missionZoneStartId + goalPos;
  DEBUG_SERIAL.print(F("    배치 자세 EEPROM #"));
  DEBUG_SERIAL.println(placePoseId);
  RunManipulatorPoseWithPoseDataInEEPROM(placePoseId, 1000);
  delay(CFG.wait.pose1300Ms);

  // 그리퍼 열기
  OpenGripper(pixy);
  delay(CFG.wait.gripperActionMs);
  DEBUG_SERIAL.println(F("    블록 배치 완료"));

  // STORAGE 자세로 복귀 (정면 0도)
  RunManipulatorPoseWithPoseDataInEEPROM(STORAGE, 1000, 0.0);
  delay(CFG.wait.pose1500Ms);
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
    uint8_t targetSigmap = (1 << (targetSig - 1));
    uint8_t goalPos = goalPositions[blockIdx];

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
    RunManipulatorPoseWithPoseDataInEEPROM(STORAGE, 800, 0.0);
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
      pixy.ccc.getBlocks(true, targetSigmap);

      if (pixy.ccc.numBlocks && pixy.ccc.blocks[0].m_x > CFG.cameraScan.centerXMin && pixy.ccc.blocks[0].m_x < CFG.cameraScan.centerXMax)
      {
        // 블록이 카메라 중앙 부근 → 정지 후 미세조정+집기
        if (fineTuneAndPick(targetSigmap))
        {
          placeAtZone(goalPos);
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
        pixy.ccc.getBlocks(true, targetSigmap);

        if (pixy.ccc.numBlocks && pixy.ccc.blocks[0].m_x > CFG.cameraScan.centerXMin && pixy.ccc.blocks[0].m_x < CFG.cameraScan.centerXMax)
        {
          if (fineTuneAndPick(targetSigmap))
          {
            placeAtZone(goalPos);
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
      DEBUG_SERIAL.print(F("  [경고] Sig"));
      DEBUG_SERIAL.print(targetSig);
      DEBUG_SERIAL.println(F(" 블록 미발견, 다음으로"));
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
  RunManipulatorPoseWithPoseDataInEEPROM(STORAGE, 800, 0.0);
  delay(CFG.wait.pose800Ms);
  RunManipulatorPoseWithPoseDataInEEPROM(INITIAL_AND_MISSION_INSTRUCTION, 1000);
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
    if (!DriveUntilNoObstacleWithOneSensor(dxl, slVal, CFG.finishReturn.boundaryAdc,
                                           DRIVE_DIRECTION_BACKWARD,
                                           CFG.speed.returnSpeed))
      break;
    if (millis() - t0 > CFG.timeout.returnPhaseMs)
    {
      DEBUG_SERIAL.println(F("  [6-1] 타임아웃"));
      break;
    }
  }
  SetMobileGoalVelocityForSyncWrite(dxl, 0, 0, 0, 0);
  delay(CFG.wait.driveSettleMs);

  // ─── 복귀 2단계: 장애물 다시 감지될 때까지 후진 ───
  DEBUG_SERIAL.println(F("  [6-2] SL 장애물 다시 감지될 때까지 후진..."));
  t0 = millis();
  while (1)
  {
    GetValueFromSideLeftPSDSensor(&slVal);
    SetMobileGoalVelocityForSyncWrite(dxl, -CFG.speed.returnSpeed,
                                      -CFG.speed.returnSpeed,
                                      -CFG.speed.returnSpeed,
                                      -CFG.speed.returnSpeed); // 후진
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
    if (!DriveUntilNoObstacleWithOneSensor(dxl, slVal, CFG.finishReturn.boundaryAdc,
                                           DRIVE_DIRECTION_BACKWARD,
                                           CFG.speed.returnSpeed))
      break;
    if (millis() - t0 > CFG.timeout.returnPhaseMs)
    {
      DEBUG_SERIAL.println(F("  [6-3] 타임아웃"));
      break;
    }
  }
  SetMobileGoalVelocityForSyncWrite(dxl, 0, 0, 0, 0);
  delay(CFG.wait.driveSettleMs);

  // ─── 복귀 4단계: Finish Zone 진입 추가 후진 ───
  DEBUG_SERIAL.print(F("  [6-4] Finish Zone 진입 ("));
  DEBUG_SERIAL.print(CFG.finishReturn.finishExtraMs);
  DEBUG_SERIAL.println(F("ms)..."));
  SetMobileGoalVelocityForSyncWrite(dxl, -CFG.speed.returnSpeed,
                                    -CFG.speed.returnSpeed,
                                    -CFG.speed.returnSpeed,
                                    -CFG.speed.returnSpeed); // 후진
  delay(CFG.finishReturn.finishExtraMs);
  SetMobileGoalVelocityForSyncWrite(dxl, 0, 0, 0, 0);

  // 최종 정지, INITIAL 자세
  RunManipulatorPoseWithPoseDataInEEPROM(INITIAL_AND_MISSION_INSTRUCTION, 1000);
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
