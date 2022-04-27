typedef double db;
typedef long long ll;

#include "Adafruit_MPU6050.h"
#include "Adafruit_Sensor.h"
#include "Adafruit_VL53L0X.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#include "Adafruit_MotorShield.h"
#include "utility/Adafruit_PWMServoDriver.h"
#include "ArduinoSort.h"

// DoF pins
#define LOXF_ADDRESS 0x30
#define LOXL_ADDRESS 0x31
#define LOXR_ADDRESS 0x32
#define SHT_LOXF 6
#define SHT_LOXL 5
#define SHT_LOXR 7

/* 
 * Setup
 */

// Settings
const db gyroOffset = -0.0164267;
const int dofCnts = 3;
const int maxSpd = 255;
const int wasteDelay = 3;

// Data
db gx, gy, gz;// x and z
unsigned long curTime;
int dF, dL, dR;

// Hardware components
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *RB = AFMS.getMotor(1);
Adafruit_DCMotor *LB = AFMS.getMotor(2);
Adafruit_DCMotor *RF = AFMS.getMotor(3);
Adafruit_DCMotor *LF = AFMS.getMotor(4);

Adafruit_MPU6050 mpu;// mpu (accelgyro)

Adafruit_VL53L0X loxF = Adafruit_VL53L0X();
Adafruit_VL53L0X loxL = Adafruit_VL53L0X();
Adafruit_VL53L0X loxR = Adafruit_VL53L0X();
VL53L0X_RangingMeasurementData_t measureF;
VL53L0X_RangingMeasurementData_t measureL;
VL53L0X_RangingMeasurementData_t measureR;

void setup() {
  Serial.begin(115200);
  Serial.println("Initializing...");
  while (!Serial) {delay(1);}
  
  pinMode(SHT_LOXF, OUTPUT);
  pinMode(SHT_LOXL, OUTPUT);
  pinMode(SHT_LOXR, OUTPUT);
  
  setID();
  
  Serial.println("---Startup Complete---");
}

void loop() {
//  getData();
//  debug();
//  turn(90, 0.5);
  moveForward(100, 0.5);
  delay(10000);
}

void setID() {
  // AFMS: shield & motors
  AFMS.begin();
  LF -> setSpeed(0); LF -> run(RELEASE);
  LB -> setSpeed(0); LB -> run(RELEASE);
  RF -> setSpeed(0); RF -> run(RELEASE);
  RB -> setSpeed(0); RB -> run(RELEASE);
  
  // MPU
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  
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
    while(1);
  }
  delay(10);
  
  digitalWrite(SHT_LOXL, HIGH);
  delay(10);
  if(!loxL.begin(LOXL_ADDRESS)) {
    Serial.println(F("Failed to boot left VL53L0X"));
    while(1);
  }
  delay(10);

  digitalWrite(SHT_LOXR, HIGH);
  delay(10);
  if(!loxR.begin(LOXR_ADDRESS)) {
    Serial.println(F("Failed to boot right VL53L0X"));
    while(1);
  }
  delay(10);
}

db perInstConv(db num) {
  num -= gyroOffset;
  db timePassed = micros()-curTime;
  curTime = micros();
  db tmp = 1000000;
  return num*(timePassed/tmp);
}

void getDataMPU() {
  // MPU6050 (accelgyro)
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  gz -= perInstConv(g.gyro.z)*180.0/M_PI;
}

void getDataDoF() {
  // DoF sensors
  loxF.rangingTest(&measureF, false);
  loxL.rangingTest(&measureL, false);
  loxR.rangingTest(&measureR, false);

  if (measureF.RangeStatus != 4) {
    dF = measureF.RangeMilliMeter;
  } else dF = 9999;
  if (measureL.RangeStatus != 4) {
    dL = measureL.RangeMilliMeter;
  } else dL = 9999;
  if (measureR.RangeStatus != 4) {
    dR = measureR.RangeMilliMeter;
  } else dR = 9999;

  // https://pastebin.com/WcuVfiu5
}

void moveForward(db inpDist, db inpSpd) {
  int spd = round(inpSpd*maxSpd);
  LF -> setSpeed(spd);
  LB -> setSpeed(spd);
  RF -> setSpeed(spd);
  RB -> setSpeed(spd);
  
  if (inpDist > 0) {
    LF -> run(FORWARD);
    LB -> run(FORWARD);
    RF -> run(FORWARD);
    RB -> run(FORWARD);
    getDataDoF();
    db curDF = dF;
    while (curDF-inpDist < dF) {
      getDataDoF();
      delay(wasteDelay);
      Serial.println("dF :"+String(dF));
      Serial.println("curDF: "+String(curDF));
      Serial.println();
    }
  } else if (inpDist < 0) {
    LF -> run(BACKWARD);
    LB -> run(BACKWARD);
    RF -> run(BACKWARD);
    RB -> run(BACKWARD);
    getDataDoF();
    db curDF = dF;
    while (curDF-inpDist > dF) {
      getDataDoF();
      delay(wasteDelay);
    }
  }

  LF -> run(RELEASE);
  LB -> run(RELEASE);
  RF -> run(RELEASE);
  RB -> run(RELEASE);
}

void turn(db inpRot, db inpSpd) {
  int spd = round(inpSpd*maxSpd);
  LF -> setSpeed(spd);
  LB -> setSpeed(spd);
  RF -> setSpeed(spd);
  RB -> setSpeed(spd);

  if (inpRot > 0) {// right (cw)
    LF -> run(FORWARD);
    LB -> run(FORWARD);
    RF -> run(BACKWARD);
    RB -> run(BACKWARD);
    getDataMPU();
    db curGZ = gz;
    while (curGZ+inpRot > gz) {
      getDataMPU();
      delay(wasteDelay);
    }
  } else if (inpRot < 0) {// left (ccw)
    LF -> run(BACKWARD);
    LB -> run(BACKWARD);
    RF -> run(FORWARD);
    RB -> run(FORWARD);
    getDataMPU();
    db curGZ = gz;
    while (curGZ+inpRot < gz) {
      getDataMPU();
      delay(wasteDelay);
    }
  }
    
  LF -> run(RELEASE);
  LB -> run(RELEASE);
  RF -> run(RELEASE);
  RB -> run(RELEASE);
}

void debug() {
  Serial.println("dF: "+String(dF));
  Serial.println("dL: "+String(dL));
  Serial.println("dR: "+String(dR));
  Serial.println(gz, 4);
  Serial.println("\n");
}
