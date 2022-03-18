#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"
// Components
//Adafruit_MotorShield AFMS = Adafruit_MotorShield();
//Adafruit_DCMotor *motor = AFMS.getMotor(2);
//const int maxSpd = 255;
Adafruit_MPU6050 mpu;// Accelerometer

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
  
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  // set accelerometer range to +-8G
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  // set gyro range to +- 500 deg/s
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  // set filter bandwidth to 21 Hz
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println("---Startup Complete---");
}

// Functions
double perSecConv(double num) {
  double timePassed = millis()-curTime;
  curTime = millis();
  double tmp = 1000;
  return num*(timePassed/tmp);
}

void getData() {
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  if (g.gyro.z < -0.02 || g.gyro.z > -0.01) gz -= perSecConv(g.gyro.z);
}

void loop() {
  // Update
//  getData();
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  Serial.println(g.gyro.z);
  
//  Serial.println(gz, 4);
  
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

  delay(wasteDelay);
}
