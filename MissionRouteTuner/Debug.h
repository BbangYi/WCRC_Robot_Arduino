#ifndef DEBUG_H
#define DEBUG_H

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "Arduino.h"

//////////////  디버그 모드 설정 관련
#define DEBUG

#define DEBUG_OUTPUT_LEVEL_INFO
#define DEBUG_OUTPUT_LEVEL_DEBUG
#define DEBUG_OUTPUT_LEVEL_WARN
#define DEBUG_OUTPUT_LEVEL_ERROR

#define SERIAL_BAUD_RATE              115200
#define ENABLE_BLUETOOTH_SERIAL       1
#define ENABLE_BLUETOOTH_SERIAL_OUTPUT 0
#define BLUETOOTH_SERIAL              Serial2
#define BLUETOOTH_BAUD_RATE           9600

class DebugSerialFanout {
public:
  void begin(unsigned long usbBaudRate, unsigned long bluetoothBaudRate);
  int available();
  int read();

  template <typename T>
  size_t print(const T &value) {
    size_t written = Serial.print(value);
#if ENABLE_BLUETOOTH_SERIAL && ENABLE_BLUETOOTH_SERIAL_OUTPUT
    written += BLUETOOTH_SERIAL.print(value);
#endif
    return written;
  }

  template <typename T, typename U>
  size_t print(const T &value, const U &format) {
    size_t written = Serial.print(value, format);
#if ENABLE_BLUETOOTH_SERIAL && ENABLE_BLUETOOTH_SERIAL_OUTPUT
    written += BLUETOOTH_SERIAL.print(value, format);
#endif
    return written;
  }

  size_t println() {
    size_t written = Serial.println();
#if ENABLE_BLUETOOTH_SERIAL && ENABLE_BLUETOOTH_SERIAL_OUTPUT
    written += BLUETOOTH_SERIAL.println();
#endif
    return written;
  }

  template <typename T>
  size_t println(const T &value) {
    size_t written = Serial.println(value);
#if ENABLE_BLUETOOTH_SERIAL && ENABLE_BLUETOOTH_SERIAL_OUTPUT
    written += BLUETOOTH_SERIAL.println(value);
#endif
    return written;
  }

  template <typename T, typename U>
  size_t println(const T &value, const U &format) {
    size_t written = Serial.println(value, format);
#if ENABLE_BLUETOOTH_SERIAL && ENABLE_BLUETOOTH_SERIAL_OUTPUT
    written += BLUETOOTH_SERIAL.println(value, format);
#endif
    return written;
  }
};

extern DebugSerialFanout DEBUG_SERIAL;


//////////////  Debug 함수 (정의는 Debug.cpp)
void initDebug();

#endif
