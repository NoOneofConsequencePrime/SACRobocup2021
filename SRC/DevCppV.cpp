#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"
#define S0 3
#define S1 4
#define S2 5
#define S3 6
#define trigPin 12
#define echoPin 11
#include "HCSR04.h"

HCSR04 sr04 = HCSR04(echoPin, trigPin);

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

// Select which 'port' M1, M2, M3 or M4.
Adafruit_DCMotor *myMotor1= AFMS.getMotor(1);
Adafruit_DCMotor *myMotor2 = AFMS.getMotor(2);

// Variable
float duration, distance;

// Setup
void setup() {
  Serial.begin(9600);
  AFMS.begin();  // create with the default frequency 1.6KHz
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  myMotor1->setSpeed(0);
  myMotor1->run(RELEASE);

  myMotor2->setSpeed(0);
  myMotor2->run(RELEASE);
  return;
}

// Functions
float getDistInst() {
  delay(5);
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10
  );
  digitalWrite(trigPin, LOW);
  return pulseIn(echoPin, HIGH) * 0.0348 / 2;
}

float getDist() {
  float distArr[12];
  for (int i = 0; i < 12; i++) {
    distArr[i] = getDistInst();
  }
  qsort(distArr, 12, sizeof(distArr[0]), sort_ascd_float);
  float avg = 0;
  for (int i = 3; i < 9; i++) {
    avg += distArr[i];
  }
  return avg/6;
}

void forward(float spd){
  int s = anaConv(spd);
  myMotor1->setSpeed(s);
  myMotor1->run(BACKWARD);

  myMotor2->setSpeed(s);  
  myMotor2->run(FORWARD);
}

void backward(float spd){
  int s = anaConv(spd);
  myMotor1->setSpeed(s);  
  myMotor1->run(FORWARD);

  myMotor2->setSpeed(s);  
  myMotor2->run(BACKWARD);
}

void stopMotor() {
  myMotor1->run(RELEASE);
  myMotor2->run(RELEASE);
}

void mov(int pos, float spd) {
  float initDist = getDist();
  Serial.println("InitDist: " + String(initDist));
  // Initial Movement
  if (pos > 0) {// Positive -> Forward movement
  	Serial.println("Going forward");
    forward(spd);
    while (initDist-getDistInst() < pos) {
      Serial.println(initDist-getDistInst());
    }
  } else if (pos < 0) {// Negative -> Backward movement
    Serial.println("Going backward");
    backward(spd);
    while (initDist-getDistInst() > pos) {
      Serial.println(initDist-getDistInst());
    }
  }
  // Minory Fixation
  if (initDist-getDistInst() > pos) {// Greater -> Go back
  	Serial.println("Fixing back");
    backward(spd/3);
    while (initDist-getDistInst() > pos) {
      Serial.println(initDist-getDistInst());
    }
  } else if (initDist-getDistInst() < pos) {// Smaller -> Go front
    Serial.println("Fixing front");
    forward(spd/3);
    while (initDist-getDistInst() < pos) {
      Serial.println(initDist-getDistInst());
    }
  }
  stopMotor();
  Serial.println("Motor Stopped");
  return;
}

// Support Functions
int anaConv(float spd) {
  return (int)(spd*255);
}

int sort_ascd_float(const void *cmp1, const void *cmp2) {
  float a = *((float *)cmp1);
  float b = *((float *)cmp2);
  return a < b ? -1 : (a > b ? 1 : 0);
}

// Main Loop
void loop() {
  mov(20, 0.5);
  delay(5000);
//  forward(0.8);
//  while (true) {
//    Serial.println("testing");
//  }
//  Serial.println("stopped---------");
//  delay(3000);
}
