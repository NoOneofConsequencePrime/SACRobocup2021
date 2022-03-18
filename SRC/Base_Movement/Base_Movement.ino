#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
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

// Components
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_MPU6050 mpu;// Accelerometer
//HCSR04 sr04 = HCSR04(echoPin, trigPin);// Ultrasonic sensor

// Select which 'port' M1, M2, M3 or M4.
Adafruit_DCMotor *BL= AFMS.getMotor(1);
Adafruit_DCMotor *BR = AFMS.getMotor(2);
Adafruit_DCMotor *FR = AFMS.getMotor(3);
Adafruit_DCMotor *FL = AFMS.getMotor(4);

// Variable
float gx, gy, gz;
int wasteDelay = 5;// Millisecond
//float duration, distance;

// Setup
void setup() {
  Serial.begin(115200);
  
// Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  // set accelerometer range to +-8G
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);

  // set gyro range to +- 500 deg/s
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);

  // set filter bandwidth to 21 Hz
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  
  AFMS.begin();  // create with the default frequency 1.6KHz
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  BL->setSpeed(0);
  BL->run(RELEASE);

  BR->setSpeed(0);
  BR->run(RELEASE);

  FR->setSpeed(0);
  FR->run(RELEASE);

  FL->setSpeed(0);
  FL->run(RELEASE);

  delay(100);
}

// Functions
void getData() {
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  gz -= round(perSecConv(g.gyro.z)*100)/100.0;

  delay(wasteDelay);
}

float perSecConv(float num) {
  return num*(wasteDelay/10.0);
}

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

void forward(float spd) {
  int s = anaConv(spd);
  FL->setSpeed(s);
  FL->run(FORWARD);
  FR->setSpeed(s);
  FR->run(FORWARD);
  BL->setSpeed(s);
  BL->run(FORWARD);
  BR->setSpeed(s);
  BR->run(FORWARD);
}

void backward(float spd) {
  int s = anaConv(spd);
  FL->setSpeed(s);
  FL->run(BACKWARD);
  FR->setSpeed(s);
  FR->run(BACKWARD);
  BL->setSpeed(s);
  BL->run(BACKWARD);
  BR->setSpeed(s);
  BR->run(BACKWARD);
}

void rotateSimple(float spd) {
  int s = -anaConv(spd);
  if (s > 0) {
    FL->setSpeed(s);
    FL->run(BACKWARD);
    FR->setSpeed(s);
    FR->run(FORWARD);
    BL->setSpeed(s);
    BL->run(BACKWARD);
    BR->setSpeed(s);
    BR->run(FORWARD);
  } else if (s < 0) {
    s *= -1.0;
    FL->setSpeed(s);
    FL->run(FORWARD);
    FR->setSpeed(s);
    FR->run(BACKWARD);
    BL->setSpeed(s);
    BL->run(FORWARD);
    BR->setSpeed(s);
    BR->run(BACKWARD);
  }
}

void stopMotor() {
  FL->run(RELEASE);
  FR->run(RELEASE);
  BL->run(RELEASE);
  BR->run(RELEASE);
}

void turn(float deg, float spd) {
  float initDeg = gz;
  Serial.println("InitDeg: " + String(initDeg));
  //Initial Movement
  if (deg > 0) {// right(pos)
    Serial.println("Turn Right");
    rotateSimple(spd);
    while (gz-initDeg < deg) {
      getData();
      Serial.println(gz-initDeg);
    }
  } else if (deg < 0) {// left(neg)
    Serial.println("Turn Lef");
    rotateSimple(-spd);
    while (gz-initDeg > deg) {
      getData();
      Serial.println(gz-initDeg);
    }
  }
  // Minor Fixation
  float pError = 0.2;
  if (gz-initDeg > deg+pError) {
    Serial.println("Fixing right");
    rotateSimple(-spd/3);
    while (gz-initDeg > deg) {
      getData();
      Serial.println(gz-initDeg);
    }
  } else if (gz-initDeg < deg-pError) {
    Serial.println("Fixing left");
    rotateSimple(spd/3);
    while (gz-initDeg < deg) {
      getData();
      Serial.println(gz-initDeg);
    }
  }
  stopMotor();
  Serial.println("Motor Stopped");
  return;
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
  // Minor Fixation
  if (initDist-getDistInst() > pos) {// Greater -> Go back
    Serial.println("Fixing back");
    backward(spd/5);
    while (initDist-getDistInst() > pos) {
      Serial.println(initDist-getDistInst());
    }
  } else if (initDist-getDistInst() < pos) {// Smaller -> Go front
    Serial.println("Fixing front");
    forward(spd/5);
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
  // Update
  getData();
  
  // Move
  turn(90, 0.7);
  delay(800);
  turn(-90, 0.7);
  delay(800);
  turn(-180, 0.5);
  delay(1000);

  // Debug
  Serial.println("GZ: " + String(gz));
}
