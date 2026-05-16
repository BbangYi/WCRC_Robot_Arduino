#ifndef MOBILEBASE_H
#define MOBILEBASE_H

#include "Motor.h"

//////////////  모터 구동용 변수/상수
#define MOBILE_DXL_ID_FL 0x01 // 좌측 전방 모터 아이디
#define MOBILE_DXL_ID_FR 0x02 // 우측 전방 모터 아이디
#define MOBILE_DXL_ID_BL 0x03 // 좌측 후방 모터 아이디
#define MOBILE_DXL_ID_BR 0x04 // 우측 후방 모터 아이디

#define S_TO_MILLIS_RATIO 1000

// const int32_t MOBILEBASE_DEFAULT_DRIVING_SPEED = 150;
const int32_t MOBILEBASE_DEFAULT_DRIVING_SPEED = 200;
const int32_t MOBILEBASE_DEFAULT_DRIVING_MM_PER_S = 150;

#define MOBILEBASE_DEFAULT_OPERATING_MODE VELOCITY_CONTROL_MODE
#define MOBILEBASE_DEFAULT_DRIVE_MODE VELOCITY_BASED_PROFILE

#define RADIANS_2_MOTOR_VALUE_RATIO (4095 / (2 * PI))
#define RADIANS_2_MOTOR_VALUE(X) (int32_t)((X) * RADIANS_2_MOTOR_VALUE_RATIO)

//////////////  모바일베이스 주행 방향
#define DRIVE_DIRECTION_FORWARD 0x01  // 전진
#define DRIVE_DIRECTION_BACKWARD 0x02 // 후진
#define DRIVE_DIRECTION_LEFT 0x03     // 좌진
#define DRIVE_DIRECTION_RIGHT 0x04    // 우진

//////////////  휠
#define WHEEL_RADIUS_MM (60 / 2.0)

// FindMobileBaseServos 함수에서 찾을 모터 아이디들, 값이 중복 없이 정렬되어있어야 함
const uint8_t MOBILE_DXL_ID_CNT = 4;
const uint8_t MOBILE_DXL_IDS[MOBILE_DXL_ID_CNT] = {MOBILE_DXL_ID_FL,
                                                   MOBILE_DXL_ID_FR,
                                                   MOBILE_DXL_ID_BL,
                                                   MOBILE_DXL_ID_BR};

// mobile velocity control mode sync write 용 상수, 구조체 정의 및 인스턴스화
const uint16_t SW_WHEEL_VELOCITY_START_ADDR = 104; // sync write start 주소
const uint16_t SW_WHEEL_VELOCITY_DATA_SIZE = 4;    // sync write data 길이
typedef struct sw_wheel_velocity_data
{ // 모터에 데이터를 쓰기 위한 구조체 정의
  int32_t goal_velocity;
} __attribute__((packed)) sw_wheel_velocity_data_t;

// mobile extended position control mode sync write 용 상수, 구조체 정의 및 인스턴스화
const uint16_t SW_WHEEL_POSITION_START_ADDR = 112; // sync write start 주소
const uint16_t SW_WHEEL_POSITION_DATA_SIZE = 8;    // sync write data 길이
typedef struct sw_wheel_position_data
{ // 모터에 데이터를 쓰기 위한 구조체 정의
  int32_t profile_velocity;
  int32_t goal_position;
} __attribute__((packed)) sw_wheel_position_data_t;

// mobile extended position control mode sync read 용 상수, 구조체 정의 및 인스턴스화
const uint16_t wheel_position_pkt_buf_cap = 128;

const uint16_t SR_WHEEL_POSITION_START_ADDR = 132; // sync read start 주소
const uint16_t SR_WHEEL_POSITION_DATA_SIZE = 4;    // sync read data 길이
typedef struct sr_wheel_position_data
{ // 모터에서 데이터를 읽기 위한 구조체 정의
  int32_t present_position;
} __attribute__((packed)) sr_wheel_position_data_t;

// is moving sync read 용 상수, 구조체 정의 및 인스턴스화
const uint16_t moving_pkt_buf_cap = 128;

const uint16_t SR_WHEEL_MOVING_START_ADDR = 122; // sync write start 주소
const uint16_t SR_WHEEL_MOVING_DATA_SIZE = 2;    // sync write data 길이
typedef struct sr_wheel_moving_data
{ // 모터에 데이터를 쓰기 위한 구조체 정의
  uint8_t moving;
  uint8_t moving_status;
} __attribute__((packed)) sr_wheel_moving_data_t;

//////////////  모바일베이스 함수 선언
/*
 * 모바일베이스 모터가 통신 가능한지 확인하고 초기화 하는 함수
 * return :
 *    true  : 통신 체크 및 초기화 성공
 *    false : 통신 체크 및 초기화 실패
 */
bool InitMobilebase(Dynamixel2Arduino dxl);

/*
 * 모바일베이스를 속도제어 모드로 변경
 */
void ChangeMobilebaseMode2VelocityControlMode(Dynamixel2Arduino dxl);

/*
 * 모바일베이스를 확장 위치제어, 시간기반 프로파일 모드로 변경
 */
void ChangeMobilebaseMode2ExtendedPositionControlWithTimeBasedProfileMode(Dynamixel2Arduino dxl);

/*
 * MOBILE_DXL_IDS 배열에 있는 아이디의 모터들이 모두 통신 가능한지
 * return :
 *    true  : 통신 체크 및 초기화 성공
 *    false : 통신 체크 및 초기화 실패
 */
bool FindMobileBaseServos(Dynamixel2Arduino dxl);

/*
 * 모바일베이스가 goal position에 도착했는지 체크하는 함수
 * (sr_wheel_moving_data[i].moving_status)&0x01가 1일 때 arrived, 0일 때 not arrived
 * return :
 *    true  : 도착
 *    false : 이동중
 */
bool CheckIfMobilebaseIsInPosition(Dynamixel2Arduino dxl);

/*
 * 모바일베이스가 움직이는 중인지 체크하는 함수
 * (sr_wheel_moving_data[i].moving_status)&0x01가 1일 때 arrived, 0일 때 not arrived
 * return :
 *    true  : 이동중
 *    false : 정지
 */
bool CheckIfMobilebaseIsMoving(Dynamixel2Arduino dxl);

/*
 * 멀티턴모드, 시간기반 프로파일 사용하여 모바일베이스를 정해진 방향으로 정해진 거리만큼 이동시키는 함수
 * params :
 *    distance : 이동할 거리(mm 단위)
 *    drivingDirection : 주행방향
 *    mmPerSec : 1초에 얼마나 주행할 것인지 속도
 */
void DriveDistanceAndMmPerSecAndDirection(Dynamixel2Arduino dxl, float distance,
                                          uint8_t drivingDirection = DRIVE_DIRECTION_FORWARD,
                                          int32_t mmPerSec = MOBILEBASE_DEFAULT_DRIVING_MM_PER_S);

/*
 * 멀티턴모드, 시간기반 프로파일 사용하여 모바일베이스를 정해진 X,Y 거리만큼 이동시키는 함수
 * params :
 *    xMm : 수평으로 이동할 거리(mm 단위), 양수는 우측
 *    yMm : 수직으로 이동할 거리(mm 단위), 양수는 전방
 *    mmPerSec : 1초에 얼마나 주행할 것인지 속도
 */
void DriveXYDistanceAndMmPerSec(Dynamixel2Arduino dxl, float xMm, float yMm,
                                int32_t mmPerSec = MOBILEBASE_DEFAULT_DRIVING_MM_PER_S);

/*
 * 두 개의 센서 에러 값을 사용하여 전방으로 주행하는 함수
 * params :
 *    leftSensorError : 좌측 센서 오차
 *    rightSensorError : 우측 센서 오차
 *    toleranceValue : 센서 오차 허용 범위. 센서 오차가 -toleranceValue ~ +toleranceValue 사이에 들면 모바일베이스 정지
 *    drivingSpeed : 주행 속도
 * return :
 *    true : 제어중
 *    false : 제어 완료
 */
bool GoForwardWithTwoSensors(Dynamixel2Arduino dxl, int16_t leftSensorError, int16_t rightSensorError,
                             int16_t toleranceValue, int32_t drivingSpeed = MOBILEBASE_DEFAULT_DRIVING_SPEED);

/*
 * 두 개의 센서 에러 값을 사용하여 위치를 조정하는 함수
 * params :
 *    sideSensorError : 측면 센서 오차
 *    forwardSensorError : 전방 센서 오차
 *    toleranceValue : 센서 오차 허용 범위. 센서 오차가 -toleranceValue ~ +toleranceValue 사이에 들면 모바일베이스 정지
 *    drivingDirectionLeftOrRight : 주행 방향 DRIVE_DIRECTION_LEFT 또는 DRIVE_DIRECTION_RIGHT 이어야 함
 *    drivingSpeed : 주행 속도
 * return :
 *    true : 제어중
 *    false : 제어 완료
 */
bool LocatingWithTwoSensors(Dynamixel2Arduino dxl, int16_t sideSensorError, int16_t forwardSensorError, int16_t toleranceValue,
                            uint8_t drivingDirectionLeftOrRight, int32_t drivingSpeed = MOBILEBASE_DEFAULT_DRIVING_SPEED);

/*
 * 한 개의 센서 에러 값을 사용하여 정해진 방향으로 주행하는 함수
 * params :
 *    SensorError : 센서 오차
 *    toleranceValue : 센서 오차 허용 범위. 센서 오차가 -toleranceValue ~ +toleranceValue 사이에 들면 모바일베이스 정지
 *    drivingDirection : 주행 방향. 사용하려는 PSD 센서 방향(후방은 없음) 또는 센싱값이 목표값보다 작을 때 즉 오차가 음수일 때 주행할 방향을 의미
 *    drivingSpeed : 주행 속도
 * return :
 *    true : 제어중
 *    false : 제어 완료
 */
bool DriveWithOneSensor(Dynamixel2Arduino dxl, int16_t sensorError, int16_t toleranceValue,
                        uint8_t drivingDirection = DRIVE_DIRECTION_FORWARD,
                        int32_t drivingSpeed = MOBILEBASE_DEFAULT_DRIVING_SPEED);

/*
 * 센서에 장애물이 감지되지 않을 때 까지 주행하는 함수
 * params :
 *    sensorValue : 센서 값
 *    thresholdValue : 센서 값이 이 값보다 작아지면 장애물이 없음으로 판단
 *    drivingDirection : 주행 방향
 *    drivingSpeed : 주행 속도
 * return :
 *    true : 제어중
 *    false : 제어 완료
 */
bool DriveUntilNoObstacleWithOneSensor(Dynamixel2Arduino dxl, int16_t sensorValue, int16_t thresholdValue,
                                       uint8_t drivingDirection = DRIVE_DIRECTION_FORWARD,
                                       int32_t drivingSpeed = MOBILEBASE_DEFAULT_DRIVING_SPEED);

/*
 * 멀티턴모드를 사용하여 정해진 바퀴를 모터 값 만큼 회전시키는 함수
 * params :
 *    fl_relative_position : 좌측전방 바퀴 모터 회전 값
 *    fr_relative_position : 우측전방 바퀴 모터 회전 값
 *    bl_relative_position : 좌측후방 바퀴 모터 회전 값
 *    br_relative_position : 우측전방 바퀴 모터 회전 값
 *    drivingTime : 이동에 사용할 시간
 */
void SetMobileRelativePositionForSyncWrite(Dynamixel2Arduino dxl,
                                           int32_t fl_relative_position, int32_t fr_relative_position,
                                           int32_t bl_relative_position, int32_t br_relative_position,
                                           int32_t drivingTime);

/*
 * sync write 하기 위한 모바일베이스의 goal velocity 값들을 설정하는 함수
 * params :
 *    fl_goal_velocity : 1번 모터 속도
 *    fr_goal_velocity : 2번 모터 속도
 *    bl_goal_velocity : 3번 모터 속도
 *    br_goal_velocity : 4번 모터 속도
 */
void SetMobileGoalVelocityForSyncWrite(Dynamixel2Arduino dxl,
                                       int32_t fl_goal_velocity, int32_t fr_goal_velocity,
                                       int32_t bl_goal_velocity, int32_t br_goal_velocity);

/*
 * Velocity-mode wheel trim. Defaults are all 1.0, so existing mission behavior
 * is unchanged unless a tuner sketch explicitly sets trim values.
 */
void SetMobileWheelVelocityTrim(float fl_scale, float fr_scale,
                                float bl_scale, float br_scale);
void GetMobileWheelVelocityTrim(float *fl_scale, float *fr_scale,
                                float *bl_scale, float *br_scale);

#endif
