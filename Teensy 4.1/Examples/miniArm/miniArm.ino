#include <Servo.h>

// Declare objects for the servos
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

// create names for the controller pins
int control1 = A0;
int control2 = A1;
int control3 = A2;
int control4 = A3;

// variables for the servo angle
int angle1;
int angle2;
int angle3;
int angle4;


// setup the servos by attaching the objects to the arduino pins
void setup() {
  Serial.begin(9600);
  servo1.attach(6);
  servo2.attach(9);
  servo3.attach(10);
  servo4.attach(11);
}


void loop() 
{ 
// constantly update the servo positions with the value given by the controllers

// analogRead will return a decimal 0-1, multiplied by 180 gives a range from 0-180 for the servo angle.
  angle1 = (analogRead(control1) * 180.0) / 1023.0;
  angle2 = (analogRead(control2) * 180.0) / 1023.0;
  angle3 = (analogRead(control3) * 180.0) / 1023.0;
  angle4 = (analogRead(control4) * 180.0) / 1023.0;
//  Serial.println(angle1);

// send the angle given by the controller to the servo
  servo1.write(angle1);
  delay(5); // 5 milliseconds of wait time to improve stability
  servo2.write(angle2);
  delay(5);
  servo3.write(angle3);
  delay(5);
  servo4.write(angle4);
  delay(5);
  
}
