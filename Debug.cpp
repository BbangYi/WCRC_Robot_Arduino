#include "Arduino.h"
#include "Debug.h"

void initDebug() {
#ifdef DEBUG
  DEBUG_SERIAL.begin(SERIAL_BAUD_RATE);
#endif
}
