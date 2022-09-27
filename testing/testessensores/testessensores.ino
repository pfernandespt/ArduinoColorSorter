#include <Wire.h>
#include <FastLED.h>
#include "DFRobot_TCS34725.h"

byte gammatable[256];
DFRobot_TCS34725 tcs = DFRobot_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
int r,g,b,r1,b1,g1;
CRGB leds[21];

void setup() {
  FastLED.addLeds<WS2852, 2, RGB>(leds, 21);
  Serial.begin(9600);
  Serial.println("TESTE");
  if (tcs.begin()) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1); // halt!
  }

  for (int i=0; i<256; i++) {
    float x = i;
    x /= 255;
    x = pow(x, 2.5);
    x *= 255;
    gammatable[i] = x; 
   
}
}

void loop() {
  uint16_t clear, red, green, blue;
  tcs.getRGBC(&red, &green, &blue, &clear);
  tcs.lock();  // turn off LED

  uint32_t sum = clear;
  float r, g, b;
  r = red; r /= sum;
  g = green; g /= sum;
  b = blue; b /= sum;
  r *= 256 * 3; g *= 256 * 3; b *= 256 * 3;
  r1 = gammatable[(int)r];
  g1 = gammatable[(int)g];
  b1 = gammatable[(int)b];
  Serial.print("(");
  Serial.print(r1);
  Serial.print(";");
  Serial.print(g1);
  Serial.print(";");
  Serial.print(b1);
  Serial.println(")");
  
  // put your main code here, to run repeatedly:

}
