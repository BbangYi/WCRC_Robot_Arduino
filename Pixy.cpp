#include "Arduino.h"
#include "Pixy.h"

#include <Pixy2SPI_SS.h> // SPI 통신하는 Pixy2 라이브러리를 포함시킴

//////////////  Pixy 함수 정의
void InitPixy(Pixy2SPI_SS pixy) {
  pixy.init(); // pixy객체 초기화
}
