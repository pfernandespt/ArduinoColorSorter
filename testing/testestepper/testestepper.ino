// Adafruit Motor shield library
// copyright Adafruit Industries LLC, 2009
// this code is public domain, enjoy!

#include <AFMotor.h>

// Connect a stepper motor with 48 steps per revolution (7.5 degree)
// to motor port #2 (M3 and M4)
AF_Stepper motor(200, 2);
int last_pos = 1, next_pos;
void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Stepper test!");

  motor.setSpeed(100);  // 10 rpm   
}

void loop() {
  next_pos = random(1,7);
    int movement = abs(next_pos - last_pos);
  movement *= 150/6;
  Serial.print("movimento motor baixo:");
  Serial.println(movement);
  if(next_pos > last_pos) motor.step(movement, FORWARD, MICROSTEP);
  else motor.step(movement, BACKWARD, MICROSTEP);
  
  last_pos = next_pos; 
}
