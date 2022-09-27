/*  
 *    Descrição: O seguinte codigo tem por objetivo através da utilização de um sensor de cor e de motores 
 *    stepper executar a separação de coisas de até 6 cores diferentes tendo por base valores padrão indicados.
 *    
 *    Universidade: UAveiro | Departamento: DETI | Curso: EEC | Ano: 2022 | Cadeira: IEE | Professor: Pedro Cabral
 *    Grupo nº8: Paulo Fernandes, Pedro Cunha
 */
 
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
#define StepsPerRevolution1 200 //revoluções por volta motor 1 (cima)
#define StepsPerRevolution2 200 //revoluções por volta motor 2 (baixo)
#define Motor1Speed 50          //RPM maximo do motor 1
#define Motor2Speed 50          //RPM maximo do motor 2
#define Motor1Angle 45          //amplitude que o motor de cima tem que rodar para pegar mais um M&M
#define Motor2Freedom 280       //amplitude maxima que o motor2 pode rodar

AF_Stepper motor1(StepsPerRevolution1,1); //inicialização motor 1
AF_Stepper motor2(StepsPerRevolution2,2); //inicialização motor 2

int last_pos = 1, next_pos;
const int rotation_step = map(Motor1Angle,0,360,0,StepsPerRevolution1);
//const int rotation_step = 25;
const float  sector_amplitude = map(Motor2Freedom, 0, 360, 0, StepsPerRevolution2)/ 6;

//bibliotecas e variaveis para os leds endereçaveis /////////////////////////////
#include <FastLED.h>
#define NumLeds 60 //numero de leds da fita 
#define LedsPin A0 //pino onde a fita esta ligada
#define Brightness 255 //brilho dos leds 0 - 255

CRGB leds[NumLeds];

//Processamento dos dados //////////////////////////////////////////////////////
byte r,g,b, max_color, min_color, nearest_color; //processamento das cores para ficarem mais vivas

struct color{  
  byte r;
  byte g;
  byte b;
  byte pos; //posição no recipiente
  char c;   //letra representativa
};

const color cor[7] = { //cores padrao para a comparacao
  {255,255,255, 0, 'N'}, //nada detetado
  {255, 10, 10, 1, 'R'}, //cor 1
  {255,125, 10, 2, 'O'}, //cor 2
  {255,243, 10, 3, 'Y'}, //cor 3
  { 10,255,170, 4, 'G'}, //cor 4
  {  0,125,255, 5, 'B'}, //cor 5
  {255, 20, 20, 6, 'C'}  //cor 6
  };

int contador[7];

//Controlo /////////////////////////////////////////////////////////////////////
#define Button A1

void setup() { /////////////////////////////////////////////////////////////////
  pinMode(Button, INPUT_PULLUP);
  
  Serial.begin(9600);
  Serial.println("Comunicacao Serie Inicializada!");

  Serial.print("Motors:  M1: ");
  Serial.print(sector_amplitude);
  Serial.print("   M2: ");
  Serial.println(rotation_step);
  
  motor1.setSpeed(Motor1Speed);
  motor2.setSpeed(Motor2Speed);

  for(int i = 0; i < 7; ++i) contador[i] = 0; //inicializa o contador de cores a 0;

  FastLED.addLeds<WS2812B, LedsPin, GRB>(leds, NumLeds); //inicialização da fita de leds
  FastLED.setBrightness(Brightness);

  if(tcs.begin()){ //tentar encontrar o sensor de cor no I2C
    Serial.println("Sensor de Cor encontrado!");
  } else {
    Serial.println("Erro! Sensor de Cor desconectado!");
    while(true); //se ocorrer um erro a encotrar o sensor o programa para a execução
  }

  Serial.println("Tubo Seletor na Primeira posicao? Confirme...");
  while(digitalRead(Button) == HIGH);
  Serial.println("Tudo Certo! Iniciando Programa!");
  while(digitalRead(Button) == LOW);
}

void loop() { /////////////////////////////////////////////////////////////////
  
  delay(2000);
  //ação do motor1
  motor1.step(rotation_step, FORWARD, DOUBLE);
  
  //ação do sensor de cor
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

  nearest_color = aprox_color(); //chama a funcao
  
  Serial.print(" ("); 
  Serial.print(r);
  Serial.print(';');
  Serial.print(g);
  Serial.print(';');
  Serial.print(b);
  Serial.print(") Nearest color is ");
  Serial.println(cor[nearest_color].c);
  contador[nearest_color]++;
  
  for(int i = 1; i < 7; ++i){
    Serial.print(cor[i].c);
    Serial.print(": ");
    Serial.print(contador[i]);
    Serial.print("   ");
  }
  
  //transmisao dos dados de cor para a fita de leds

  for(int i = 0; i < NumLeds; ++i){
    leds[i] = CRGB(r,g,b);
  }
  FastLED.show();

  //ação do motor2
  delay(500);
  next_pos = cor[nearest_color].pos;
  int movement = abs(next_pos - last_pos);
  movement *= sector_amplitude;
  Serial.print("movimento motor baixo:");
  Serial.println(movement);
  if(next_pos > last_pos) {
    motor2.step(movement, FORWARD, DOUBLE);
    delay(100);
  }
  else {
    motor2.step(movement, BACKWARD, DOUBLE);
    delay(100);
  }
  
  last_pos = next_pos;

  //opcao de parar o programa 
  if(digitalRead(Button) == LOW){
    Serial.println("Programa em Pausa! Para retomar clique novamente no botao!");
    while(digitalRead(Button) == LOW) delay(100);

    next_pos = 1;
  int movement = abs(next_pos - last_pos);
  movement *= sector_amplitude;
  Serial.print("movimento motor baixo:");
  Serial.println(movement);
  if(next_pos > last_pos) {
    motor2.step(movement, FORWARD, DOUBLE);
    delay(100);
  }
  else {
    motor2.step(movement, BACKWARD, DOUBLE);
    delay(100);
  }
  
  last_pos = next_pos;
    
    while(digitalRead(Button) == HIGH) delay(50);
    Serial.println("Programa sera retomado! Por favor largue o botao.");
    while(digitalRead(Button) == LOW) delay(100);
  }
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
