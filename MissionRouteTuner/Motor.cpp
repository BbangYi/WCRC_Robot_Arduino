#include "Arduino.h"
#include "Motor.h"

// dxl 전역 객체 정의 (여기서 1번만 정의)
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

void initMotorCommunication() {
  dxl.begin(DXL_BAUDRATE);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
}
