#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"

// Setup

// Components
//Adafruit_MotorShield AFMS = Adafruit_MotorShield();
//Adafruit_DCMotor *motor = AFMS.getMotor(2);
//const int maxSpd = 255;
Adafruit_MPU6050 mpu;// Accelerometer

// Parameters
const double gyroOffset = -0.0151442;

// Variable
double gx, gy, gz;
const int wasteDelay = 1000;
unsigned long curTime;

void setup() {
  Serial.begin(115200);
  Serial.println("Initializing...");
  
  // Init
//  AFMS.begin();
//  motor -> setSpeed(0); motor -> run(RELEASE);
//  accelgyro.initialize();
  
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  // set accelerometer range to +-8G
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  // set gyro range to +- 500 deg/s
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  // set filter bandwidth to 21 Hz
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println("---Startup Complete---");
}

// Functions
double perInstConv(double num) {
  num -= gyroOffset;
  double timePassed = micros()-curTime;
  curTime = micros();
  double tmp = 1000000;
  return num*(timePassed/tmp);
}

void getData() {
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  gz -= perInstConv(g.gyro.z)*180.0/M_PI;
}

int cnt = 0;

void loop() {
  // Update
  getData();
  
  Serial.println(gz, 4);
  
//  motor -> run(FORWARD);
//  Serial.println("running");
//  motor -> setSpeed(0);
//  delay(500);
//  motor -> setSpeed(50);
//  delay(500);
//  motor -> setSpeed(100);
//  delay(500);
//  motor -> setSpeed(150);
//  delay(500);
//  motor -> setSpeed(200);
//  delay(500);
//  motor -> setSpeed(250);
//  delay(500);

//  delay(wasteDelay);
}
