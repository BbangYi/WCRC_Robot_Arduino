#include "Arduino.h"
#include "Debug.h"

DebugSerialFanout DEBUG_SERIAL;

void DebugSerialFanout::begin(unsigned long usbBaudRate, unsigned long bluetoothBaudRate) {
  Serial.begin(usbBaudRate);
#if ENABLE_BLUETOOTH_SERIAL
  BLUETOOTH_SERIAL.begin(bluetoothBaudRate);
#endif
}

int DebugSerialFanout::available() {
  int usbAvailable = Serial.available();
  if (usbAvailable > 0) return usbAvailable;

#if ENABLE_BLUETOOTH_SERIAL
  return BLUETOOTH_SERIAL.available();
#else
  return 0;
#endif
}

int DebugSerialFanout::read() {
  if (Serial.available() > 0) return Serial.read();

#if ENABLE_BLUETOOTH_SERIAL
  if (BLUETOOTH_SERIAL.available() > 0) return BLUETOOTH_SERIAL.read();
#endif

  return -1;
}

void initDebug() {
#ifdef DEBUG
  DEBUG_SERIAL.begin(SERIAL_BAUD_RATE, BLUETOOTH_BAUD_RATE);
#endif
}
