#include "Arduino.h"
#include "Pixy.h"
#include "Gripper.h"

//////////////  Gripper 함수 정의
void OpenGripper(Pixy2SPI_SS pixy) {
  pixy.setServos(0, GRIP_ANGLE_OPEN);
}

void CloseGripper(Pixy2SPI_SS pixy) {
  pixy.setServos(0, GRIP_ANGLE_CLOSE);
}
