// Convenience
typedef double db;
typedef long long ll;

// Program Setup
#include "Wire.h"
#include "Adafruit_Sensor.h"
#include "Adafruit_VL53L0X.h"
#include "I2Cdev.h"
#include "Adafruit_MotorShield.h"
#include "utility/Adafruit_PWMServoDriver.h"
#include "ArduinoSort.h"
#include "MPU6050_tockn.h"
#include "LiquidCrystal.h"

// DoF pins
#define LOXF_ADDRESS 0x30
#define LOXL_ADDRESS 0x31
#define LOXR_ADDRESS 0x32
#define SHT_LOXL 6
#define SHT_LOXF 5
#define SHT_LOXR 4

// Settings
const int maxSpd = 255;
const int wallSafeDist = 80;

// Data
db gz;
int dF, dL, dR;

// Hardware
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *RB = AFMS.getMotor(4);
Adafruit_DCMotor *LB = AFMS.getMotor(3);
Adafruit_DCMotor *RF = AFMS.getMotor(1);
Adafruit_DCMotor *LF = AFMS.getMotor(2);

Adafruit_VL53L0X loxF = Adafruit_VL53L0X();
Adafruit_VL53L0X loxL = Adafruit_VL53L0X();
Adafruit_VL53L0X loxR = Adafruit_VL53L0X();
VL53L0X_RangingMeasurementData_t measureF;
VL53L0X_RangingMeasurementData_t measureL;
VL53L0X_RangingMeasurementData_t measureR;

MPU6050 mpu6050(Wire);
LiquidCrystal lcd(7, 8, 12, 11, 10, 9);

void setID() {
  // LCD
  lcd.begin(16, 2);
  
  // AFMS: shield & motors
  AFMS.begin();
  LF -> setSpeed(0); LF -> run(RELEASE);
  LB -> setSpeed(0); LB -> run(RELEASE);
  RF -> setSpeed(0); RF -> run(RELEASE);
  RB -> setSpeed(0); RB -> run(RELEASE);

  // DoF: all reset
  digitalWrite(SHT_LOXF, LOW);    
  digitalWrite(SHT_LOXL, LOW);
  digitalWrite(SHT_LOXR, LOW);
  delay(10);

  // DoF: procedural activation
  digitalWrite(SHT_LOXF, HIGH);
  delay(10);
  if(!loxF.begin(LOXF_ADDRESS)) {
    Serial.println(F("Failed to boot front VL53L0X"));
  }
  delay(10);
  
  digitalWrite(SHT_LOXL, HIGH);
  delay(10);
  if(!loxL.begin(LOXL_ADDRESS)) {
    Serial.println(F("Failed to boot left VL53L0X"));
  }
  delay(10);

  digitalWrite(SHT_LOXR, HIGH);
  delay(10);
  if(!loxR.begin(LOXR_ADDRESS)) {
    Serial.println(F("Failed to boot right VL53L0X"));
  }
  delay(10);

  lcd.print("Complete");
  delay(300);
}

void getDataMPU() {
  mpu6050.update();
  gz = -(mpu6050.getAngleZ());
}

void getDataDoF(char c) {// F, A
  // DoF sensors
  loxF.rangingTest(&measureF, false);
  if (c != 'F') loxL.rangingTest(&measureL, false);
  if (c != 'F') loxR.rangingTest(&measureR, false);

  if (measureF.RangeStatus != 4) {
    dF = measureF.RangeMilliMeter;
  } else dF = 9999;
  if (c != 'F' && measureL.RangeStatus != 4) {
    dL = measureL.RangeMilliMeter;
  } else dL = 9999;
  if (c != 'F' && measureR.RangeStatus != 4) {
    dR = measureR.RangeMilliMeter;
  } else dR = 9999;

  // https://pastebin.com/WcuVfiu5
}

void debug() {
//  Serial.println("dF: "+String(dF));
//  Serial.println("dL: "+String(dL));
//  Serial.println("dR: "+String(dR));
  Serial.println(gz, 4);
  Serial.println("\n");
  lcd.clear();
  lcd.print(analogRead(8));
}

void setup() {
  Serial.begin(115200);
  Serial.println("Initializing...");
  while (!Serial) {delay(1);}
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(1);
  pinMode(SHT_LOXF, OUTPUT);
  pinMode(SHT_LOXL, OUTPUT);
  pinMode(SHT_LOXR, OUTPUT);

  setID();
  delay(100);

  Serial.println("---Startup Complete---");
}

void loop() {
  
//  getDataDoF('A');
//  Serial.println(dL);
//  Serial.println(dF);
//  Serial.println(dR);
//  Serial.println();
  
//  lcd.clear();
//  lcd.setCursor(0, 1);
//  lcd.print(millis()/1000);
  
//  getDataMPU();
  debug();
  
//  LF -> setSpeed(200);
//  LB -> setSpeed(200);
//  RF -> setSpeed(200);
//  RB -> setSpeed(200);
//  LF -> run(FORWARD);
//  LB -> run(FORWARD);
//  RF -> run(BACKWARD);
//  RB -> run(BACKWARD);
}
