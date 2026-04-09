#include "Arduino.h"
#include "Debug.h"
#include "Pins.h"
#include "PSD.h"


//////////////  PSD 함수 정의
void InitPSD() {
  // 아날로그 핀은 핀모드 설정 필요 없음
}

void GetValueFromFrontPSDSensors(int16_t* flPSDValuePtr, int16_t* frPSDValuePtr) {
  *flPSDValuePtr = analogRead(PIN_FRONT_LEFT_PSD);
  *frPSDValuePtr = analogRead(PIN_FRONT_RIGHT_PSD);
}

void GetValueFromFrontLeftPSDSensor(int16_t* flPSDValuePtr) {
  *flPSDValuePtr = analogRead(PIN_FRONT_LEFT_PSD);
}

void GetValueFromFrontRightPSDSensor(int16_t* frPSDValuePtr) {
  *frPSDValuePtr = analogRead(PIN_FRONT_RIGHT_PSD);
}

void GetValueFromSideLeftPSDSensor(int16_t* slPSDValuePtr) {
  *slPSDValuePtr = analogRead(PIN_SIDE_LEFT_PSD);
}

void GetValueFromSideRightPSDSensor(int16_t* srPSDValuePtr) {
  *srPSDValuePtr = analogRead(PIN_SIDE_RIGHT_PSD);
}
