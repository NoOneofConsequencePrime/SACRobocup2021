#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"

typedef double db;

// Components
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *RB = AFMS.getMotor(1);
Adafruit_DCMotor *LB = AFMS.getMotor(2);
Adafruit_DCMotor *RF = AFMS.getMotor(3);
Adafruit_DCMotor *LF = AFMS.getMotor(4);

const int maxSpd = 255;

void setup() {
  Serial.begin(115200);
  Serial.println("Initializing...");
  
  // Init
  AFMS.begin();
  LF -> setSpeed(0); LF -> run(RELEASE);
  LB -> setSpeed(0); LB -> run(RELEASE);
  RF -> setSpeed(0); RF -> run(RELEASE);
  RB -> setSpeed(0); RB -> run(RELEASE);

  Serial.println("---Startup Complete---");
}

void moveForward(db inpDist, db inpSpd) {
  int spd = round(inpSpd*255);
  LF -> setSpeed(spd);
  LB -> setSpeed(spd);
  RF -> setSpeed(spd);
  RB -> setSpeed(spd);
  if (inpDist > 0) {
    LF -> run(FORWARD);
    LB -> run(FORWARD);
    RF -> run(FORWARD);
    RB -> run(FORWARD);
  } else if (inpDist < 0) {
    LF -> run(BACKWARD);
    LB -> run(BACKWARD);
    RF -> run(BACKWARD);
    RB -> run(BACKWARD);
  }
  
  delay(1000);
  LF -> run(RELEASE);
  LB -> run(RELEASE);
  RF -> run(RELEASE);
  RB -> run(RELEASE);
}

void turn(db inpRot, db inpSpd) {
  int spd = round(inpSpd);
  LF -> setSpeed(spd);
  LB -> setSpeed(spd);
  RF -> setSpeed(spd);
  RB -> setSpeed(spd);
  if (inpRot > 0) {
    LF -> run(FORWARD);
    LB -> run(FORWARD);
    RF -> run(BACKWARD);
    RB -> run(BACKWARD);
  } else if (inpRot < 0) {
    LF -> run(BACKWARD);
    LB -> run(BACKWARD);
    RF -> run(FORWARD);
    RB -> run(FORWARD);
  }
  
  delay(1000);
  LF -> run(RELEASE);
  LB -> run(RELEASE);
  RF -> run(RELEASE);
  RB -> run(RELEASE);
}

void loop() {
  Serial.println("running");
//  moveForward(10, 150);
//  moveForward(-10, 200);
//  turn(90, 250);
//  turn(-90, 100);
  LF -> setSpeed(255);
  LB -> setSpeed(255);
  RF -> setSpeed(255);
  RB -> setSpeed(255);
  LF -> run(FORWARD);
  LB -> run(FORWARD);
  RF -> run(FORWARD);
  RB -> run(FORWARD);
//  delay(1000);
}
