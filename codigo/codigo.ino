//bibliotecas e variaveis para o sensor de cor //////////////////////////////////
#include <Wire.h>
#include "DFRobot_TCS34725.h"

uint16_t sensor_clear, sensor_red, sensor_green, sensor_blue;
uint8_t real_r,real_g,real_b, max_color, min_color, hue, sat, lum;

DFRobot_TCS34725 tcs = DFRobot_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_16X);

//bibliotecas e variaveis para os motores stepper ///////////////////////////////
#include <AFMotor.h>
#define StepsPerRevolution1 2048 //revoluções por volta motor 1 (cima)
#define StepsPerRevolution2 200 //revoluções por volta motor 2 (baixo)

AF_Stepper motor1(StepsPerRevolution1,1); //inicialização motor 1
AF_Stepper motor2(StepsPerRevolution2,2); //inicialização motor 2

//bibliotecas e variaveis para os leds endereçaveis /////////////////////////////
#include <FastLED.h>
#define NumLeds 32 //numero de leds da fita 
#define LedsPin A0 //pino onde a fita esta ligada
#define Brightness 200 //brilho dos leds 0 - 255

CRGB leds[NumLeds];

void setup() { /////////////////////////////////////////////////////////////////
  Serial.begin(9600);
  Serial.println("Programa Inicializado!");

  FastLED.addLeds<WS2812B, LedsPin, GRB>(leds, NumLeds);
  FastLED.setBrightness(Brightness);

  if(tcs.begin()){ //tentar encontrar o sensor de cor no I2C
    Serial.println("Sensor de Cor encontrado!");
  } else {
    Serial.println("Erro! Sensor de Cor desconectado!");
    while(true); //se ocorrer um erro a encotrar o sensor o programa para a execução
  }

}

void loop() { /////////////////////////////////////////////////////////////////
  tcs.getRGBC(&sensor_red, &sensor_green, &sensor_blue, &sensor_clear); //leitura da cor do sensor
  tcs.lock();
  delay(1000);
  
  real_r = sensor_red   * 255.0/sensor_clear; //Calculo dos Valores R na gama 0-255
  real_g = sensor_green * 255.0/sensor_clear; //Calculo dos Valores G na gama 0-255
  real_b = sensor_blue  * 255.0/sensor_clear; //Calculo dos Valores B na gama 0-255

  Serial.print("Debug: ("); 
  Serial.print(real_r);
  Serial.print(';');
  Serial.print(real_g);
  Serial.print(';');
  Serial.print(real_b);
  Serial.print(") ->");


  max_color = max(max(real_r,real_g),real_b);
  min_color = min(min(real_r,real_g),real_b);

if(abs(real_r - min_color) <= 10) real_r *= 0.1;
if(abs(real_g - min_color) <= 10) real_g *= 0.1;
if(abs(real_b - min_color) <= 10) real_b *= 0.1;

  real_r = map(real_r, 0, max_color, 0, 255);
  real_g = map(real_g, 0, max_color, 0, 255);
  real_b = map(real_b, 0, max_color, 0, 255);

  Serial.print(" ("); 
  Serial.print(real_r);
  Serial.print(';');
  Serial.print(real_g);
  Serial.print(';');
  Serial.print(real_b);
  Serial.println(')');

  for(int i = 0; i < NumLeds; ++i){
    leds[i] = CRGB(real_r,real_g,real_b);
  }
  FastLED.show();
}
