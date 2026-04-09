#ifndef GRIPPER_H
#define GRIPPER_H

#define GRIP_ANGLE_CLOSE          650  // 그리퍼 반 정도 닫히는 값(1000이 최대)
#define GRIP_ANGLE_OPEN           100  // 그리퍼 열리는 값

//////////////  Gripper 함수 선언
void OpenGripper(Pixy2SPI_SS pixy);

void CloseGripper(Pixy2SPI_SS pixy);

#endif
