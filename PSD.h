#ifndef PSD_H
#define PSD_H

#include "Debug.h"
#include "Pins.h"


//////////////  PSD 함수
/*
 * PSD를 초기화 하는 함수
 */
void InitPSD();

/*
 * 전방 두 개의 PSD 센서에서 값을 받아 업데이트하는 함수. 필터링 없음
 * params :
 *    flPSDValuePtr : 전방좌측 PSD 센서값을 업데이트할 변수 포인터
 *    frPSDValuePtr : 전방우측 PSD 센서값을 업데이트할 변수 포인터
 */
void GetValueFromFrontPSDSensors(int16_t* flPSDValuePtr, int16_t* frPSDValuePtr);

/*
 * 전방좌측 PSD 센서에서 값을 받아 업데이트하는 함수. 필터링 없음
 */
void GetValueFromFrontLeftPSDSensor(int16_t* flPSDValuePtr);

/*
 * 전방우측 PSD 센서에서 값을 받아 업데이트하는 함수. 필터링 없음
 */
void GetValueFromFrontRightPSDSensor(int16_t* frPSDValuePtr);

/*
 * 좌측면 PSD 센서에서 값을 받아 업데이트하는 함수. 필터링 없음
 */
void GetValueFromSideLeftPSDSensor(int16_t* slPSDValuePtr);

/*
 * 우측면 PSD 센서에서 값을 받아 업데이트하는 함수. 필터링 없음
 */
void GetValueFromSideRightPSDSensor(int16_t* srPSDValuePtr);

#endif
