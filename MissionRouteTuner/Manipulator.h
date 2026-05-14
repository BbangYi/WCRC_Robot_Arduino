#ifndef MANIPULATOR_H
#define MANIPULATOR_H

#include "Debug.h"
#include "Motor.h"
#include <EEPROM.h>

//////////////  모터 구동용 변수/상수
#define ARM_DXL_ID_1            0x05 // 매니퓰레이터 1번 모터 아이디 (가장 아래)
#define ARM_DXL_ID_2            0x06 // 매니퓰레이터 2번 모터 아이디
#define ARM_DXL_ID_3            0x07 // 매니퓰레이터 3번 모터 아이디
#define ARM_DXL_ID_4            0x08 // 매니퓰레이터 4번 모터 아이디 (가장 위)

#define ARM_DXL_1_POSITION_MIN  0
#define ARM_DXL_1_POSITION_MAX  4095
#define ARM_DXL_2_POSITION_MIN  1024
#define ARM_DXL_2_POSITION_MAX  3474
#define ARM_DXL_3_POSITION_MIN  211
#define ARM_DXL_3_POSITION_MAX  2640
#define ARM_DXL_4_POSITION_MIN  722
#define ARM_DXL_4_POSITION_MAX  3161

#define ARM_DXL_1_OFFSET        0
#define ARM_DXL_2_OFFSET        238
#define ARM_DXL_3_OFFSET        799
#define ARM_DXL_4_OFFSET        0


// FinManipulatorServos 함수에서 찾을 모터 아이디들, 값이 중복 없이 정렬되어있어야 함
const uint8_t ARM_DXL_ID_CNT = 4;
const uint8_t ARM_DXL_IDS[ARM_DXL_ID_CNT] = {ARM_DXL_ID_1,
                                             ARM_DXL_ID_2,
                                             ARM_DXL_ID_3,
                                             ARM_DXL_ID_4};


// arm sync write 용 상수, 구조체 정의 및 인스턴스화
const uint16_t SW_JOINT_POSITION_START_ADDR = 112; // sync write start 주소
const uint16_t SW_JOINT_POSITION_DATA_SIZE = 8; // sync write data 길이
typedef struct sw_joint_position_data{ // 모터에 데이터를 쓰기 위한 구조체 정의
  int32_t profile_velocity;
  int32_t goal_position;
} __attribute__((packed)) sw_joint_position_data_t;

sw_joint_position_data_t sw_joint_position_data[ARM_DXL_ID_CNT]; // 모터 개수만큼 인스턴스 생성
DYNAMIXEL::InfoSyncWriteInst_t sw_arm_position_infos; // syncwrite 정보 인스턴스 생성
// syncWrite할 모터들 정보 인스턴스를 모터 개수만큼 생성
DYNAMIXEL::XELInfoSyncWrite_t info_joint_position_xels_sw[ARM_DXL_ID_CNT];


// arm position control mode sync read 용 상수, 구조체 정의 및 인스턴스화
const uint16_t joint_joint_position_pkt_buf_cap = 128;
uint8_t joint_position_pkt_buf[joint_joint_position_pkt_buf_cap];

const uint16_t SR_JOINT_POSITION_START_ADDR = 132; // sync read start 주소
const uint16_t SR_JOINT_POSITION_DATA_SIZE = 4; // sync read data 길이
typedef struct sr_joint_position_data{ // 모터에서 데이터를 읽기 위한 구조체 정의
  int32_t present_position;
} __attribute__((packed)) sr_joint_position_data_t;

sr_joint_position_data_t sr_joint_position_data[ARM_DXL_ID_CNT]; // 모터 개수만큼 인스턴스 생성
DYNAMIXEL::InfoSyncReadInst_t sr_arm_position_infos; // syncread 정보 인스턴스 생성
// syncRead할 모터들 정보 인스턴스를 모터 개수만큼 생성
DYNAMIXEL::XELInfoSyncRead_t info_joint_position_xels_sr[ARM_DXL_ID_CNT];


//////////////  매니퓰레이터 계산용 변수/상수
const float d = 79.75; // 바닥에서 매니퓰레이터 2번 모터 회전축까지의 거리
const float L1 = 109.21; // 2번과 3번 모터의 회전축간 거리
const float L2 = 86.4; // 3번과 4번 모터의 회전축간 거리
const float L3 = 97.2; // 4번 모터 회전축과 그리퍼 끝부분 사이의 거리


//////////////  매니퓰레이터 포즈 저장용
#define MANIPULATOR_POSE_ID_MAX_CNT           100
#define MANIPULATOR_POSE_DATA_SIZE            40
#define MANIPULATOR_POSE_DESCRIPTION_SIZE     30

typedef struct _ManipulatorPose {
  bool isTherePoseData;
  uint8_t id;
  int16_t manipulatorMotor1Value;
  int16_t manipulatorMotor2Value;
  int16_t manipulatorMotor3Value;
  int16_t manipulatorMotor4Value;
  char description[MANIPULATOR_POSE_DESCRIPTION_SIZE];
} __attribute__((packed)) ManipulatorPose;


//////////////  매니퓰레이터 함수 선언
bool initManipulator();
bool FindManipulatorServos();
void turnOnManipulatorTorque();
void turnOffManipulatorTorque();
int32_t GetArmServoGoalPositionWithAngle(float angle);
void SetManipulatorForwardMoveWithAngleForSyncWrite( float a1,
                                                     float a2,
                                                     float a3,
                                                     float a4,
                                                     int32_t operatingTime);
void SetManipulatorForwardMoveWithMotorValueForSyncWrite( int32_t mv1,
                                                          int32_t mv2,
                                                          int32_t mv3,
                                                          int32_t mv4,
                                                          int32_t operatingTime);
void SetManipulatorInverseMoveForSyncWrite( int16_t x, int16_t y, int16_t z,
                                            int16_t angle,
                                            int32_t operatingTime );
bool IsManipulatorPoseDataValid(uint8_t requestedId, ManipulatorPose manipulatorPose);
void PrintManipulatorPoseListFromEEPROM();
void WriteManipulatorPresentPoseToEEPROM(uint8_t id, String description);
ManipulatorPose ReadManipulatorPresentPoseToEEPROM(uint8_t id);
void RemoveManipulatorPresentPoseFromEEPROM(uint8_t id);
ManipulatorPose RunManipulatorPoseWithPoseDataInEEPROM(uint8_t id, int32_t operatingTimeMillis, float motor1Angle = -360.0);


//////////////  매니퓰레이터 함수 정의
/* 
 * 매니퓰레이터 모터가 통신 가능한지 확인하고 초기화 하는 함수
 * return :
 *    true  : 통신 체크 및 초기화 성공
 *    false : 통신 체크 및 초기화 실패
 */
bool initManipulator() {
  // 매니퓰레이터 모터가 모두 있는지 확인
  if (!FindManipulatorServos()) return false;
  
  for (int i = 0 ; i < ARM_DXL_ID_CNT ; i++) {
    // 토크 끄기
    dxl.writeControlTableItem(TORQUE_ENABLE, ARM_DXL_IDS[i], TORQUE_OFF);
  
    // 드라이브 모드 설정
    // 방향은 Reverse Mode, Profile Configuration은 Time-based Profile
    dxl.writeControlTableItem(DRIVE_MODE, ARM_DXL_IDS[i],
                              REVERSE_MODE + TIME_BASED_PROFILE);
    // 오퍼레이팅 모드를 위치 제어 모드로 설정
    dxl.writeControlTableItem(OPERATING_MODE, ARM_DXL_IDS[i], POSITION_CONTROL_MODE);

    switch (ARM_DXL_IDS[i]) { // 모터에 따라 position limit, homing offset 설정
      case ARM_DXL_ID_1:
        dxl.writeControlTableItem(MIN_POSITION_LIMIT,
                                  ARM_DXL_IDS[i], ARM_DXL_1_POSITION_MIN);
        dxl.writeControlTableItem(MAX_POSITION_LIMIT,
                                  ARM_DXL_IDS[i], ARM_DXL_1_POSITION_MAX);
        dxl.writeControlTableItem(HOMING_OFFSET,
                                  ARM_DXL_IDS[i], ARM_DXL_1_OFFSET);
        break;
      case ARM_DXL_ID_2:
        dxl.writeControlTableItem(MIN_POSITION_LIMIT, 
                                  ARM_DXL_IDS[i], ARM_DXL_2_POSITION_MIN);
        dxl.writeControlTableItem(MAX_POSITION_LIMIT, 
                                  ARM_DXL_IDS[i], ARM_DXL_2_POSITION_MAX);
        dxl.writeControlTableItem(HOMING_OFFSET, 
                                  ARM_DXL_IDS[i], ARM_DXL_2_OFFSET);
        break;
      case ARM_DXL_ID_3:
        dxl.writeControlTableItem(MIN_POSITION_LIMIT, 
                                  ARM_DXL_IDS[i], ARM_DXL_3_POSITION_MIN);
        dxl.writeControlTableItem(MAX_POSITION_LIMIT, 
                                  ARM_DXL_IDS[i], ARM_DXL_3_POSITION_MAX);
        dxl.writeControlTableItem(HOMING_OFFSET, 
                                  ARM_DXL_IDS[i], ARM_DXL_3_OFFSET);
        break;
      case ARM_DXL_ID_4:
        dxl.writeControlTableItem(MIN_POSITION_LIMIT, 
                                  ARM_DXL_IDS[i], ARM_DXL_4_POSITION_MIN);
        dxl.writeControlTableItem(MAX_POSITION_LIMIT, 
                                  ARM_DXL_IDS[i], ARM_DXL_4_POSITION_MAX);
        dxl.writeControlTableItem(HOMING_OFFSET, 
                                  ARM_DXL_IDS[i], ARM_DXL_4_OFFSET);
        break;
    }

    // 토크 켜기
    dxl.writeControlTableItem(TORQUE_ENABLE, ARM_DXL_IDS[i], TORQUE_ON);
  }
  
  // manipulator sync write 준비
  sw_arm_position_infos.packet.p_buf = nullptr; // nullptr을 전달하면 내부 버퍼를 사용
  sw_arm_position_infos.packet.is_completed = false; // false로 초기화
  sw_arm_position_infos.addr = SW_JOINT_POSITION_START_ADDR; // 컨트롤 테이블에 sync write를 시작주소
  sw_arm_position_infos.addr_length = SW_JOINT_POSITION_DATA_SIZE; // sync write하는 데이터 길이
  sw_arm_position_infos.p_xels = info_joint_position_xels_sw; // sync write 할 모터 정보
  sw_arm_position_infos.xel_count = 0; // sync write 할 모터 개수

  sw_joint_position_data[0].profile_velocity = 1000; // 모터에 write 할 데이터 초기화
  sw_joint_position_data[0].goal_position = 2048;
  sw_joint_position_data[1].profile_velocity = 1000;
  sw_joint_position_data[1].goal_position = 2048;
  sw_joint_position_data[2].profile_velocity = 1000;
  sw_joint_position_data[2].goal_position = 2048;
  sw_joint_position_data[3].profile_velocity = 1000;
  sw_joint_position_data[3].goal_position = 2048;

  // 모터 정보 리스트에 모터 아이디 설정, 데이터 포인터 지정
  for(int i = 0; i < ARM_DXL_ID_CNT; i++) {
    info_joint_position_xels_sw[i].id = ARM_DXL_IDS[i];
    info_joint_position_xels_sw[i].p_data = (uint8_t*)&sw_joint_position_data[i];
    sw_arm_position_infos.xel_count++;
  }
  
  sw_arm_position_infos.is_info_changed = true; // sync write 정보가 변경됨을 설정
  
  // arm position control mode sync read 준비
  sr_arm_position_infos.packet.buf_capacity = joint_joint_position_pkt_buf_cap;
  sr_arm_position_infos.packet.p_buf = joint_position_pkt_buf;
  sr_arm_position_infos.packet.is_completed = false;
  sr_arm_position_infos.addr = SR_JOINT_POSITION_START_ADDR;
  sr_arm_position_infos.addr_length = SR_JOINT_POSITION_DATA_SIZE;
  sr_arm_position_infos.p_xels = info_joint_position_xels_sr;
  sr_arm_position_infos.xel_count = 0;

  // 모터 정보 리스트에 모터 아이디 설정, 데이터 포인터 지정
  for(int i = 0; i < ARM_DXL_ID_CNT; i++) {
    info_joint_position_xels_sr[i].id = ARM_DXL_IDS[i];
    info_joint_position_xels_sr[i].p_recv_buf = (uint8_t*)&sr_joint_position_data[i];
    sr_arm_position_infos.xel_count++;
  }
  sr_arm_position_infos.is_info_changed = true; // sync read 정보가 변경됨을 설정
  return true;
}

/*
 * ARM_DXL_IDS 배열에 있는 아이디의 모터들이 모두 통신 가능한지
 * return :
 *    true  : 통신 체크 성공
 *    false : 통신 체크 실패
 */
bool FindManipulatorServos() {
  uint8_t ids_pinged[10] = {0,};
  bool is_each_motor_found = true;
  if (uint8_t count_pinged = dxl.ping(DXL_BROADCAST_ID, ids_pinged, 
    sizeof(ids_pinged)/sizeof(ids_pinged[0]), 100)) {
    if (count_pinged >= ARM_DXL_ID_CNT) {
      uint8_t arm_dxl_ids_idx = 0;
      uint8_t ids_pinged_idx = 0;
      while(1) {
        if (ARM_DXL_IDS[arm_dxl_ids_idx]
            == ids_pinged[ids_pinged_idx++]) {
          arm_dxl_ids_idx ++;

          if (arm_dxl_ids_idx
              == sizeof(ARM_DXL_IDS)/sizeof(uint8_t)) {
            // 찾으려는 모터를 모두 찾은 경우
            break;
          }
        } else {
          if (ids_pinged_idx == count_pinged) {
             // 통신가능한 모터가 더이상 없는 경우
             is_each_motor_found = false;
             break;
          }
        }
      }
      
      if (!is_each_motor_found) {
#ifdef DEBUG
        DEBUG_SERIAL.print("Motor IDs does not match : ");
        DEBUG_SERIAL.println(dxl.getLastLibErrCode());
#endif
      }
    } else {
#ifdef DEBUG
      DEBUG_SERIAL.print("Motor count does not match : ");
      DEBUG_SERIAL.println(dxl.getLastLibErrCode());
#endif
      is_each_motor_found = false;
    }
  } else{
#ifdef DEBUG
    DEBUG_SERIAL.print("Broadcast returned no items : ");
    DEBUG_SERIAL.println(dxl.getLastLibErrCode());
#endif
    is_each_motor_found = false;
  }
  return is_each_motor_found;
}

/*
 * 매니퓰레이터의 토크를 켜는 함수
 */
void turnOnManipulatorTorque() {
  for (int i = 0 ; i < ARM_DXL_ID_CNT ; i++) {
    // 토크 켜기
    dxl.writeControlTableItem(TORQUE_ENABLE, ARM_DXL_IDS[i], TORQUE_ON);
  }
}

/*
 * 매니퓰레이터의 토크를 끄는 함수
 */
void turnOffManipulatorTorque() {
  for (int i = 0 ; i < ARM_DXL_ID_CNT ; i++) {
    // 토크 끄기
    dxl.writeControlTableItem(TORQUE_ENABLE, ARM_DXL_IDS[i], TORQUE_OFF);
  }
}

/* 
 * 각도를 입력받아 모터의 위치 값을 반환하는 함수
 * argument : 
 *    angle : 모터의 목표 각도
 * return :
 *    대응하는 모터의 목표 위치 값. -1은 변환 실패
 */
int32_t GetArmServoGoalPositionWithAngle(float angle) {
  if (!isnan(angle)) {
    // 입력받은 angle 값을 -180~180 범위 내의 값으로 제한하고
    // 0~4095 범위로 매핑
    return (int32_t)map(constrain((int32_t)angle, -180, 180),
                        -180, 180, 0, 4095);
  } else {
    return (int32_t)-1;
  }
}

/* 
 * 매니퓰레이터를 네 모터의 각도 값과 시간으로 제어하는 함수
 * argument : 
 *    a1 : 5번 모터 각도
 *    a2 : 6번 모터 각도
 *    a3 : 7번 모터 각도
 *    a4 : 8번 모터 각도
 *    operatingTime : 매니퓰레이터 동작이 완료되기까지 소요될 시간
 */
void SetManipulatorForwardMoveWithAngleForSyncWrite( float a1,
                                                     float a2,
                                                     float a3,
                                                     float a4,
                                                     int32_t operatingTime) {
  int32_t motor1GoalPosition = GetArmServoGoalPositionWithAngle( a1 );
  int32_t motor2GoalPosition = GetArmServoGoalPositionWithAngle( a2 );
  int32_t motor3GoalPosition = GetArmServoGoalPositionWithAngle( a3 );
  int32_t motor4GoalPosition = GetArmServoGoalPositionWithAngle( a4 );
  
  if (motor1GoalPosition != -1
      && motor2GoalPosition != -1
      && motor3GoalPosition != -1
      && motor4GoalPosition != -1) {
    SetManipulatorForwardMoveWithMotorValueForSyncWrite( motor1GoalPosition,
                                                         motor2GoalPosition,
                                                         motor3GoalPosition,
                                                         motor4GoalPosition,
                                                         operatingTime);
  }
}

/* 
 * 매니퓰레이터를 네 모터의 값과 시간으로 제어하는 함수
 * argument : 
 *    mv1 : 5번 모터 값
 *    mv2 : 6번 모터 값
 *    mv3 : 7번 모터 값
 *    mv4 : 8번 모터 값
 *    operatingTime : 매니퓰레이터 동작이 완료되기까지 소요될 시간
 */
void SetManipulatorForwardMoveWithMotorValueForSyncWrite( int32_t mv1,
                                                          int32_t mv2,
                                                          int32_t mv3,
                                                          int32_t mv4,
                                                          int32_t operatingTime) {
                                                            
  sw_joint_position_data[0].goal_position = constrain(mv1, ARM_DXL_1_POSITION_MIN, ARM_DXL_1_POSITION_MAX);
  sw_joint_position_data[0].profile_velocity = operatingTime;
  sw_joint_position_data[1].goal_position = constrain(mv2, ARM_DXL_2_POSITION_MIN, ARM_DXL_2_POSITION_MAX);
  sw_joint_position_data[1].profile_velocity = operatingTime;
  sw_joint_position_data[2].goal_position = constrain(mv3, ARM_DXL_3_POSITION_MIN, ARM_DXL_3_POSITION_MAX);
  sw_joint_position_data[2].profile_velocity = operatingTime;
  sw_joint_position_data[3].goal_position = constrain(mv4, ARM_DXL_4_POSITION_MIN, ARM_DXL_4_POSITION_MAX);
  sw_joint_position_data[3].profile_velocity = operatingTime;
  
  sw_arm_position_infos.is_info_changed = true;
  
  while(!dxl.syncWrite(&sw_arm_position_infos)) {}
}

/*
 * 그리퍼의 위치와 그리퍼의 피치 각도로 4-DOF 매니퓰레이터를 제어하는 함수
 * params :
 *    x : 그리퍼 좌우 위치(mm)
 *    y : 그리퍼 전후 위치(mm)
 *    z : 그리퍼 높이(mm)
 *    angle : 그리퍼 피치 각도(degree)
 *    operatingTime : 매니퓰레이터 동작이 완료되기까지 소요될 시간
 */
void SetManipulatorInverseMoveForSyncWrite( int16_t x, int16_t y, int16_t z,
                                            int16_t angle,
                                            int32_t operatingTime ) {
  if (y == 0) {
#ifdef DEBUG
    DEBUG_SERIAL.println(F("Inverse move skipped: y must not be 0."));
#endif
    return;
  }

  int16_t zFromMotor2 = z - d; // 계산의 편의를 위해 2번 모터 관절과 지면 사이의 거리를
                               // 무시한 z축(위/아래) 거리
  // 매니퓰레이터를 측면에서 봤을 때 전진 거리 yd
  float yd = sqrt((int32_t)x*x + (int32_t)y*y) * (y < 0 ? -1.0 : 1.0);
  // 매니퓰레이터를 측면에서 봤을 때 2번째 링크가 끝나는 점의 위치 (y2, z2)
  float y2 = yd - L3*cos(radians(angle));
  float z2 = zFromMotor2 - L3*sin(radians(angle));
  float reachRatio = (y2*y2 + z2*z2 - L1*L1 - L2*L2)/(2*L1*L2);
  if (reachRatio < -1.0 || reachRatio > 1.0) {
#ifdef DEBUG
    DEBUG_SERIAL.println(F("Inverse move skipped: target is out of manipulator reach."));
#endif
    return;
  }
  float rootTerm = 1 - reachRatio*reachRatio;
  if (rootTerm < 0) rootTerm = 0;

  // 세타1을 구하기 위해 사용되는 변수
  float k1 = L1 + L2*reachRatio;
  float k2 = L2*-1*sqrt(rootTerm);
  float theta1 = 0, theta2 = 0, theta3 = 0, theta4 = 0; // 모터별 세타 변수

  theta1 = degrees(atan2(y, x)); // atan2는 -pi~pi 범위로 결과를 반환 함
  theta2 = degrees(atan2(z2, y2)) - degrees(atan2(k2, k1));
  theta3 = degrees(atan2(-1*sqrt(rootTerm), reachRatio));
  theta4 = angle - theta2 - theta3;

  float a1 = 90 - theta1; // 세타 값을 모터 제어를 위한 각도로 변환
  if (a1 > 180) // 범위를 -180~180으로 변환
    a1 -= 360;
  float a2 = -90 + theta2;
  if (a2 < -180)
    a2 += 360;
  float a3 = theta3;
  float a4 = theta4;

  // 계산된 모터 각도로 매니퓰레이터 동작 준비
  SetManipulatorForwardMoveWithAngleForSyncWrite( a1, a2, a3, a4, operatingTime);

#ifdef DEBUG
  DEBUG_SERIAL.println("Result");
  DEBUG_SERIAL.print("    theta1 = ");
  DEBUG_SERIAL.println(theta1);
  DEBUG_SERIAL.print("    theta2 = ");
  DEBUG_SERIAL.println(theta2);
  DEBUG_SERIAL.print("    theta3 = ");
  DEBUG_SERIAL.println(theta3);
  DEBUG_SERIAL.print("    theta4 = ");
  DEBUG_SERIAL.println(theta4);
  DEBUG_SERIAL.println();

  DEBUG_SERIAL.println("Result motor angle");
  DEBUG_SERIAL.print("    m1angle = ");
  DEBUG_SERIAL.println(a1);
  DEBUG_SERIAL.print("    m2angle = ");
  DEBUG_SERIAL.println(a2);
  DEBUG_SERIAL.print("    m3angle = ");
  DEBUG_SERIAL.println(a3);
  DEBUG_SERIAL.print("    m4angle = ");
  DEBUG_SERIAL.println(a4);
#endif
}

/*
 * EEPROM에서 읽은 포즈 데이터가 실제로 저장된 정상 데이터인지 확인
 * 초기화되지 않은 EEPROM은 0xFF로 채워져 id=255, motor=-1처럼 보일 수 있음
 */
bool IsManipulatorPoseDataValid(uint8_t requestedId, ManipulatorPose manipulatorPose) {
  if (!manipulatorPose.isTherePoseData) return false;
  if (manipulatorPose.id != requestedId) return false;

  // EEPROM 유효성 검사는 쓰레기값(-1 등)만 걸러낸다.
  // 실제 모터별 안전 범위 제한은 실행 직전 SetManipulatorForwardMove...에서 처리한다.
  if (manipulatorPose.manipulatorMotor1Value < 0 ||
      manipulatorPose.manipulatorMotor1Value > 4095) return false;
  if (manipulatorPose.manipulatorMotor2Value < 0 ||
      manipulatorPose.manipulatorMotor2Value > 4095) return false;
  if (manipulatorPose.manipulatorMotor3Value < 0 ||
      manipulatorPose.manipulatorMotor3Value > 4095) return false;
  if (manipulatorPose.manipulatorMotor4Value < 0 ||
      manipulatorPose.manipulatorMotor4Value > 4095) return false;

  for (int i = 0 ; i < MANIPULATOR_POSE_DESCRIPTION_SIZE ; i++) {
    uint8_t descriptionByte = (uint8_t)manipulatorPose.description[i];
    if (descriptionByte == '\0') return true;
    if (descriptionByte < 32 || descriptionByte == 127 || descriptionByte == 255) return false;
  }

  return false;
}

/*
 * 매니퓰레이터 포즈 리스트를 출력
 */
void PrintManipulatorPoseListFromEEPROM() {
  uint8_t savedPoseCount = 0;
  DEBUG_SERIAL.println(F("[목록] 저장된 매니퓰레이터 자세"));
  DEBUG_SERIAL.println(F("--------------------------------"));

  for (int i = 1 ; i <= MANIPULATOR_POSE_ID_MAX_CNT ; i++) {
      ManipulatorPose manipulatorPose = ReadManipulatorPresentPoseToEEPROM(i);
      if (manipulatorPose.isTherePoseData) {
        savedPoseCount++;
        DEBUG_SERIAL.print(F("ID "));
        DEBUG_SERIAL.print(manipulatorPose.id);
        DEBUG_SERIAL.print(F(" : "));
        if (manipulatorPose.description[0] == '\0') {
          DEBUG_SERIAL.println(F("(설명 없음)"));
        } else {
          DEBUG_SERIAL.println(manipulatorPose.description);
        }
        DEBUG_SERIAL.print(F("  모터값 m1="));
        DEBUG_SERIAL.print(manipulatorPose.manipulatorMotor1Value);
        DEBUG_SERIAL.print(F(", m2="));
        DEBUG_SERIAL.print(manipulatorPose.manipulatorMotor2Value);
        DEBUG_SERIAL.print(F(", m3="));
        DEBUG_SERIAL.print(manipulatorPose.manipulatorMotor3Value);
        DEBUG_SERIAL.print(F(", m4="));
        DEBUG_SERIAL.println(manipulatorPose.manipulatorMotor4Value);
        DEBUG_SERIAL.println();
      }
  }

  if (savedPoseCount == 0) {
    DEBUG_SERIAL.println(F("저장된 자세가 없습니다."));
    DEBUG_SERIAL.println(F("팔을 원하는 위치로 맞춘 뒤 manageManipulatorPose에서 먼저 저장하세요."));
  }

  DEBUG_SERIAL.println(F("[목록 끝]"));
  DEBUG_SERIAL.println();
}

/*
 * 매니퓰레이터 네 모터의 위치를 읽어서 EEPROM에 저장
 * params :
 *    id : 저장할 포즈의 아이디
 *    description : 포즈에 대한 설명(최대 30byte)
 */
void WriteManipulatorPresentPoseToEEPROM(uint8_t id, String description) {
  ManipulatorPose manipulatorPose;
  memset(&manipulatorPose, 0, sizeof(manipulatorPose));
  manipulatorPose.isTherePoseData = true;
  manipulatorPose.id = id;
  manipulatorPose.manipulatorMotor1Value = dxl.readControlTableItem(PRESENT_POSITION, ARM_DXL_IDS[0]);
  manipulatorPose.manipulatorMotor2Value = dxl.readControlTableItem(PRESENT_POSITION, ARM_DXL_IDS[1]);
  manipulatorPose.manipulatorMotor3Value = dxl.readControlTableItem(PRESENT_POSITION, ARM_DXL_IDS[2]);
  manipulatorPose.manipulatorMotor4Value = dxl.readControlTableItem(PRESENT_POSITION, ARM_DXL_IDS[3]);
  description.toCharArray(manipulatorPose.description, MANIPULATOR_POSE_DESCRIPTION_SIZE);

  EEPROM.put(MANIPULATOR_POSE_DATA_SIZE*(id-1), manipulatorPose);
}

/*
 * 매니퓰레이터 포즈 데이터를 EEPROM에서 읽어 반환
 * return :
 *    ManipulatorPose 객체 : 해당 아이디를 가진 ManipulatorPose 객체를 반환
 *                          isTherePoseData로 데이터가 유효한지 확인한 후 사용해야 함
 */
ManipulatorPose ReadManipulatorPresentPoseToEEPROM(uint8_t id) {
  ManipulatorPose manipulatorPose;
  memset(&manipulatorPose, 0, sizeof(manipulatorPose));
  manipulatorPose.isTherePoseData = false;
  manipulatorPose.id = id;

  if (id >= 1 && id <= MANIPULATOR_POSE_ID_MAX_CNT) {
    EEPROM.get(MANIPULATOR_POSE_DATA_SIZE*(id-1), manipulatorPose);
    manipulatorPose.description[MANIPULATOR_POSE_DESCRIPTION_SIZE-1] = '\0';
    if (!IsManipulatorPoseDataValid(id, manipulatorPose)) {
      memset(&manipulatorPose, 0, sizeof(manipulatorPose));
      manipulatorPose.isTherePoseData = false;
      manipulatorPose.id = id;
    }
  }

  return manipulatorPose;
}

/*
 * EEPROM 에서 지정된 id의 매니퓰레이터 포즈 데이터를 삭제
 * isTherePoseData만 false로 변경하기 때문에 데이터가 실제로 지워지지는 않음
 * params :
 *    id : 삭제할 포즈의 아이디
 */
void RemoveManipulatorPresentPoseFromEEPROM(uint8_t id) {
  if (id >= 1 && id <= MANIPULATOR_POSE_ID_MAX_CNT) {
    ManipulatorPose manipulatorPose;
    memset(&manipulatorPose, 0, sizeof(manipulatorPose));
    manipulatorPose.id = id;
    EEPROM.put(MANIPULATOR_POSE_DATA_SIZE*(id-1), manipulatorPose);
  }
}

/*
 * 매니퓰레이터 포즈 데이터를 EEPROM에서 읽어 실행
 * params :
 *    id : 실행할 포즈의 아이디
 *    operatingTimeMillis : 포즈 실행에 사용할 시간
 *    motor1Angle(-100~100) : 1번 모터 회전 각도(기본 값은 -360.0이며, 각도 값을 주지 않으면 포즈 데이터의 값을 그대로 사용)
 *                            과한 회전을 방지하기 위해 범위를 -100~100도로 하였음
 * return :
 *    ManipulatorPose 객체 : 실행한 ManipulatorPose 객체를 반환
 *                          isTherePoseData로 데이터가 유효한지 확인한 후 사용해야 함
 */
ManipulatorPose RunManipulatorPoseWithPoseDataInEEPROM(uint8_t id, int32_t operatingTimeMillis, float motor1Angle) {
  ManipulatorPose manipulatorPose = ReadManipulatorPresentPoseToEEPROM(id);

  if (manipulatorPose.isTherePoseData) {
    int32_t manipulatorMotor1Value = (motor1Angle == -360.0
                                      ? manipulatorPose.manipulatorMotor1Value
                                      : map(constrain(round(motor1Angle*100), -10000, 10000), -18000, 18000, 0, 4095));

    SetManipulatorForwardMoveWithMotorValueForSyncWrite( manipulatorMotor1Value,
                                                         manipulatorPose.manipulatorMotor2Value,
                                                         manipulatorPose.manipulatorMotor3Value,
                                                         manipulatorPose.manipulatorMotor4Value,
                                                         operatingTimeMillis);
  }

  return manipulatorPose;
}

#endif
