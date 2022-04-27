#include "Adafruit_MPU6050.h"
#include "Adafruit_Sensor.h"
#include "Adafruit_VL53L0X.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#include "Adafruit_MotorShield.h"
#include "utility/Adafruit_PWMServoDriver.h"
#include "ArduinoSort.h"

// Setup
Adafruit_MPU6050 mpu;// Accelerometer
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

// Parameters
const double gyroOffset = -0.0160901;
const int dofCnts = 6;

// Datas
double gx, gy, gz;// x and z
VL53L0X_RangingMeasurementData_t measure;
unsigned long curTime;
int dt;

// Variables

void setup() {
  Serial.begin(115200);
  Serial.println("Initializing...");
  
  // Init
//  AFMS.begin();
//  motor -> setSpeed(0); motor -> run(RELEASE);
//  accelgyro.initialize();

  while (!Serial) {delay(1);}
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
//  if (!lox.begin()) {
//    Serial.println("Failed to boot VL53L0X");
//    while(1) {delay(10);}
//  }
//  Serial.println("VL53L0X Found!"); 

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

  // mpu6050 (gyro)
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  gz -= perInstConv(g.gyro.z)*180.0/M_PI;

  // vl53l0x (DoF distance laser)
//  int arr[dofCnts];
//  for (int i = 0; i < dofCnts; i++) {
//    lox.rangingTest(&measure, false);
//    if (measure.RangeStatus != 4) arr[i] = measure.RangeMilliMeter;
//    else i--;
//  }
//  sortArray(arr, dofCnts);
//  dt = (arr[dofCnts/2-1]+arr[dofCnts/2]+1)/2;
//  Serial.println(dt);
}

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

//  delay(1000);
}
