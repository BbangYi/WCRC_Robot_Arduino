#ifndef RGBLED_H
#define RGBLED_H

//////////////  RGB LED 관련 선언
#include "Pins.h"

#include <Adafruit_NeoPixel.h>
#define NUM_PIXELS  1
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUM_PIXELS,
                                             RGB_LED_PIN,
                                             NEO_GRBW + NEO_KHZ800);

//////////////  RGB LED 함수
/*
 * RGB LED를 초기화 하는 함수
 */
void initRGBLED() {
  pixels.begin(); // RGB LED 설정
  pixels.setBrightness(100); // 밝기 설정
  pixels.show(); // 변경사항을 보이기
}

void setRGBLEDRed() {
  pixels.setPixelColor(0, pixels.Color(255, 0, 0)); // 빨강
  pixels.show();
}

void setRGBLEDGreen() {
  pixels.setPixelColor(0, pixels.Color(0, 255, 0)); // 초록
  pixels.show();
}

void setRGBLEDBlue() {
  pixels.setPixelColor(0, pixels.Color(0, 0, 255)); // 파랑
  pixels.show();
}

void setRGBLEDOff() {
  pixels.setPixelColor(0, pixels.Color(0, 0, 0)); // 소등
  pixels.show();
}

#endif
