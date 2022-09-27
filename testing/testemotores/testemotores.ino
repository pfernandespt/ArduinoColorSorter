#include <AFMotor.h>

// Number of steps per output rotation
// Change this as per your motor's specification
const int stepsPerRevolution = 200;

// connect motor to port #2 (M3 and M4)
//AF_Stepper motor(stepsPerRevolution, 2);
AF_Stepper motor(stepsPerRevolution, 1);

void setup() {
  Serial.begin(9600);
  Serial.println("Stepper test!");

  motor.setSpeed(500);  // 10 rpm   

}
void loop() {

  Serial.println("Double coil steps");
  motor.step(20000, FORWARD, DOUBLE);
  delay(2000);
  motor.step(20000, BACKWARD, DOUBLE);
}
