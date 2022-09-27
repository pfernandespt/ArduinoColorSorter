//bibliotecas e variaveis para o sensor de cor //////////////////////////////////

#include <Wire.h>
#include "DFRobot_TCS34725.h"

uint16_t sensor_clear, sensor_red, sensor_green, sensor_blue;

/* Esquema de fios
 *  VERMELHO 5V
 *  PRETO GND
 *  AZUL SCL ou A5
 *  VERDE SDA ou A4
 */

DFRobot_TCS34725 tcs = DFRobot_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_16X); //inicialização sensor de cor

//bibliotecas e variaveis para os motores stepper ///////////////////////////////
#include <AFMotor.h>
#define StepsPerRevolution1 2048 //revoluções por volta motor 1 (cima)
#define StepsPerRevolution2 200 //revoluções por volta motor 2 (baixo)

AF_Stepper motor1(StepsPerRevolution1,1); //inicialização motor 1
AF_Stepper motor2(StepsPerRevolution2,2); //inicialização motor 2

//bibliotecas e variaveis para os leds endereçaveis /////////////////////////////
#include <FastLED.h>
#define NumLeds 20 //numero de leds da fita 
#define LedsPin A0 //pino onde a fita esta ligada
#define Brightness 255 //brilho dos leds 0 - 255

CRGB leds[NumLeds];

//Processamento dos dados //////////////////////////////////////////////////////
byte r,g,b, max_color, min_color; //processamento das cores para ficarem mais vivas

struct color{  
  byte r;
  byte g;
  byte b;
  char name[3];
};

const color cor[7] = { //cores padrao para a comparacao
  {255,255,255, "NOT"},
  {255, 10, 10, "RED"}, //cor 1
  {255,130,  8, "ORG"}, //cor 2
  {255,243, 10, "YEL"}, //cor 3
  { 10,255,170, "GRN"}, //cor 4
  {  0,125,255, "BLU"}, //cor 5
  {255, 20, 20, "BRW"}  //cor 6
  };

void setup() { /////////////////////////////////////////////////////////////////
  Serial.begin(9600);
  Serial.println("Programa Inicializado!");

  FastLED.addLeds<WS2812B, LedsPin, GRB>(leds, NumLeds); //inicialização da fita de leds
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
  
  r = sensor_red   * 255.0/sensor_clear; //Calculo dos Valores R na gama 0-255
  g = sensor_green * 255.0/sensor_clear; //Calculo dos Valores G na gama 0-255
  b = sensor_blue  * 255.0/sensor_clear; //Calculo dos Valores B na gama 0-255

  Serial.print("Debug: ("); 
  Serial.print(r);
  Serial.print(';');
  Serial.print(g);
  Serial.print(';');
  Serial.print(b);
  Serial.print(") ->");

  // processamento dos valores para obter cores mais vivas
  
  max_color = max(max(r,g),b);
  min_color = min(min(r,g),b);

  if(abs(r - min_color) <= 10) r *= 0.1;
  if(abs(g - min_color) <= 10) g *= 0.1;
  if(abs(b - min_color) <= 10) b *= 0.1;

  r = map(r, 0, max_color, 0, 255);
  g = map(g, 0, max_color, 0, 255);
  b = map(b, 0, max_color, 0, 255);

  Serial.print(" ("); 
  Serial.print(r);
  Serial.print(';');
  Serial.print(g);
  Serial.print(';');
  Serial.print(b);
  Serial.print(") Nearest color is ");
  Serial.println(cor[aprox_color()].name);

  for(int i = 0; i < NumLeds; ++i){
    leds[i] = CRGB(r,g,b);
  }
  FastLED.show();
  delay(1000);
}
//funções necessarias //////////////////////////////////////////////////////////////////////////

int aprox_color(){ //devolve o numero da cor mais proxima a que foi lida no sensor
	int color_number = 0;
	float min_distance = distance(0);
  for(int i = 0; i < 7; ++i){
    float cur_distance = distance(i);
    if(cur_distance < min_distance){
      color_number = i;
      min_distance = cur_distance;
    }
  }
  return color_number;
}

float distance(int i){ // calcula a distancia da cor lida a cor indicada no argumento
  float dist = sqrt(pow((r-cor[i].r),2) + pow((g-cor[i].g),2) + pow((b-cor[i].b),2));
  return dist;
}
