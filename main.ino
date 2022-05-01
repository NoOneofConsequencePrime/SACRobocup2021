typedef double db;
typedef long long ll;

#include "Adafruit_Sensor.h"
#include "Adafruit_VL53L0X.h"
#include "I2Cdev.h"
#include "Wire.h"
#include "Adafruit_MotorShield.h"
#include "utility/Adafruit_PWMServoDriver.h"
#include "ArduinoSort.h"
#include "MPU6050_tockn.h"
#include "Wire.h"
// DoF pins
#define LOXF_ADDRESS 0x30
#define LOXL_ADDRESS 0x31
#define LOXR_ADDRESS 0x32
#define SHT_LOXF 6
#define SHT_LOXL 5
#define SHT_LOXR 7
#define LEDPIN 9

/* 
 * Setup
 */

// Settings
//const db gyroOffset = -0.017884;
//const db gyroOffset = -0.00663502;
//const db gyroOffset = -0.1;
const int dofCnts = 3;
const int maxSpd = 255;
const int wasteDelay = 100;
const int moveWait = 300;
const int blockDelay = 1400;
const int distFromWall = 80;
const int turnWait90 = 960;
MPU6050 mpu6050(Wire);
const int MFAmt = 235;
const int turnAmt = 151;

// Data
db z;
unsigned long curTime;
int dF, dL, dR;
int wallDist;

// Hardware components
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *RB = AFMS.getMotor(1);
Adafruit_DCMotor *LB = AFMS.getMotor(2);
Adafruit_DCMotor *RF = AFMS.getMotor(3);
Adafruit_DCMotor *LF = AFMS.getMotor(4);

Adafruit_VL53L0X loxF = Adafruit_VL53L0X();
Adafruit_VL53L0X loxL = Adafruit_VL53L0X();
Adafruit_VL53L0X loxR = Adafruit_VL53L0X();
VL53L0X_RangingMeasurementData_t measureF;
VL53L0X_RangingMeasurementData_t measureL;
VL53L0X_RangingMeasurementData_t measureR;
int rng;

void setID() {
  rng = random(2);
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
//    while(1);
  }
  delay(10);
  
  digitalWrite(SHT_LOXL, HIGH);
  delay(10);
  if(!loxL.begin(LOXL_ADDRESS)) {
    Serial.println(F("Failed to boot left VL53L0X"));
//    while(1);
  }
  delay(10);

  digitalWrite(SHT_LOXR, HIGH);
  delay(10);
  if(!loxR.begin(LOXR_ADDRESS)) {
    Serial.println(F("Failed to boot right VL53L0X"));
//    while(1);
  }
  delay(10);
}

void getDataMPU() {
  mpu6050.update();
  z = -(mpu6050.getAngleZ());
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

void dumbForward(db inpSpd, int wallD) {
  int spd = round(inpSpd*maxSpd);
  LF -> setSpeed(spd);
  LB -> setSpeed(spd);
  RF -> setSpeed(spd);
  RB -> setSpeed(spd);
  db distReduc = (db)spd*spd/400.0;

  LF -> run(FORWARD);
  LB -> run(FORWARD);
  RF -> run(FORWARD);
  RB -> run(FORWARD);
  
  getDataDoF();
  db curDF = dF;
  while (dF-distReduc > wallD) {
    getDataDoF();
    delay(wasteDelay);
    if (analogRead(0) > 400) {
      LF -> setSpeed(0);
      LB -> setSpeed(0);
      RF -> setSpeed(0);
      RB -> setSpeed(0);
      delay(500);
      exit(0);
    }
  }
  LF -> setSpeed(0);
  LB -> setSpeed(0);
  RF -> setSpeed(0);
  RB -> setSpeed(0);
  delay(moveWait);
}

void moveForward(db inpDist, db inpSpd) {
  int spd = round(inpSpd*maxSpd);
  LF -> setSpeed(spd);
  LB -> setSpeed(spd);
  RF -> setSpeed(spd);
  RB -> setSpeed(spd);

  db distReduc = (db)spd*spd/400.0;
  if (inpDist > 0) {
    LF -> run(FORWARD);
    LB -> run(FORWARD);
    RF -> run(FORWARD);
    RB -> run(FORWARD);
//    delay(blockDelay*inpDist/300);
    getDataDoF();
    db curDF = dF;
//      Serial.println(String(curDF)+" "+String(dF)+" "+String(distReduc)+" "+String(inpDist));
//    while (curDF-dF+distReduc < inpDist) {// curDF-inpDist+distReduc < dF
//      getDataDoF();
//      delay(wasteDelay);
//    }
    unsigned long tmpTime = millis();
    while (curDF-dF+distReduc < inpDist && dF-distReduc > distFromWall) {// curDF-inpDist+distReduc < dF
      getDataDoF();
      delay(wasteDelay);
      if (analogRead(0) > 400) {
        LF -> run(BACKWARD);
        LB -> run(BACKWARD);
        RF -> run(BACKWARD);
        RB -> run(BACKWARD);
        delay(millis()-tmpTime);
        
        LF -> setSpeed(0);
        LB -> setSpeed(0);
        RF -> setSpeed(0);
        RB -> setSpeed(0);
        delay(moveWait);
        turn(-turnAmt*2-10, 0.5);
        break;
      }
    }
  } else if (inpDist < 0) {
    LF -> run(BACKWARD);
    LB -> run(BACKWARD);
    RF -> run(BACKWARD);
    RB -> run(BACKWARD);
//    delay(blockDelay*(-inpDist)/300);
    getDataDoF();
    db curDF = dF;
    while (curDF-inpDist+distReduc > dF) {
      getDataDoF();
      delay(wasteDelay);
    }
  }

//  LF -> run(RELEASE);
//  LB -> run(RELEASE);
//  RF -> run(RELEASE);
//  RB -> run(RELEASE);

  LF -> setSpeed(0);
  LB -> setSpeed(0);
  RF -> setSpeed(0);
  RB -> setSpeed(0);
  delay(moveWait);
}

void turn(db inpRot, db inpSpd) {
  int spd = round(inpSpd*maxSpd);
  LF -> setSpeed(spd);
  LB -> setSpeed(spd);
  RF -> setSpeed(spd);
  RB -> setSpeed(spd);

  int turnReduc = (db)spd*spd/2750.0;
  if (inpRot > 0) {// right (cw)
    
    LF -> run(FORWARD);
    LB -> run(FORWARD);
    RF -> run(BACKWARD);
    RB -> run(BACKWARD);
    
//    delay(turnWait90 * inpRot/90);
    getDataMPU();
    db curZ = z;
    while (z-curZ+turnReduc < inpRot) {// +turnReduc
//      digitalWrite(LEDPIN, HIGH);
      Serial.println(String(z)+" "+String(curZ)+" "+String(z-curZ));
      getDataMPU();
      delay(wasteDelay);
    }
////    digitalWrite(LEDPIN, LOW);
  } else if (inpRot < 0) {// left (ccw)
    LF -> run(BACKWARD);
    LB -> run(BACKWARD);
    RF -> run(FORWARD);
    RB -> run(FORWARD);
//    delay(turnWait90 * -inpRot/90);
    getDataMPU();
    db curZ = z;
    while (z-curZ+turnReduc > inpRot) {// +turnReduc
      if (z-curZ < inpRot || z-curZ > -inpRot) break;
      getDataMPU();
      delay(wasteDelay);
    }
  }
    
//  LF -> run(RELEASE);
//  LB -> run(RELEASE);
//  RF -> run(RELEASE);
//  RB -> run(RELEASE);

  LF -> setSpeed(0);
  LB -> setSpeed(0);
  RF -> setSpeed(0);
  RB -> setSpeed(0);
  delay(moveWait);
}

void setupData() {
  // inp
  getDataDoF();

  // setting
  wallDist = min(dF, min(dL, dR));
}

void debug() {
  Serial.println("dF: "+String(dF));
  Serial.println("dL: "+String(dL));
  Serial.println("dR: "+String(dR));
  Serial.println(z, 4);
  Serial.println("\n");
}

void setup() {
  delay(5000);
  Serial.begin(115200);
  Serial1.begin(115200);
  Serial2.begin(115200);
  Serial.println("Initializing...");
  while (!Serial) {delay(1);}
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  pinMode(13, OUTPUT);
  pinMode(SHT_LOXF, OUTPUT);
  pinMode(SHT_LOXL, OUTPUT);
  pinMode(SHT_LOXR, OUTPUT);
  pinMode(LEDPIN, OUTPUT);
  
  setID();
  setupData();
  delay(200);
  
  Serial.println("---Startup Complete---");

  getDataDoF();
  while (dR > 350) {
    getDataDoF();
    delay(5);
  }
}

void dispense(int value, int receiving, int camera) {
  //do the dispensing and then move away, after write Done back to camera
  if (receiving == 0) {
    delay(1000);
  }
  else if (receiving == 1) {
    delay(1000);
  }
  else if (receiving == 2) {
    delay(1000);
  }
  
  //telling camera so it knows to start detecting again
  
  if (camera==0) {
    Serial1.write(value);
  }
  else if (camera==1) {
    Serial2.write(value);
  }
  digitalWrite(13, HIGH);
  delay(500);
  digitalWrite(13, LOW);
}

void loop() {
//  int val;
//  bolo
//  val=   //connect grayscale sensor to Analog 0
//  Serial.print("Color: ");
//  Serial.println(val);//print the value to serial
  
//  getDataDoF();
//  if (dF > wallDist+200) {
//    moveForward(330, 0.8);
//  } else {
//    int rng = random(3);
//    if (rng == 0) {
//      
//    } else if (rng == 1) {
//      turn(150, 0.5);
//    } else if (rng == 2) {
//      turn(-150, 0.5);
//    }
//  }
  
//  moveForward(200, 1);
//  turn(180, 0.6);
//  turn(-180, 0.6);
//  turn(360, 0.6);
//  delay(1000);
//  for (int i = 0; i < 4; i++) {
//    moveForward(200, 1);
//    delay(100000);
//  }
//  delay(200000);
  if (Serial1.available()) {
    int data_rcvd = Serial1.read();
    Serial.print("Received: ");
    Serial.println(data_rcvd);
    if (data_rcvd == 48) Serial1.write(2);
    if (data_rcvd == 49) Serial1.write(2);
    if (data_rcvd == 50) Serial1.write(2);
  } else {
//    Serial.println("Unavailable");
  }
  if (Serial2.available()) {
    int data_rcvd = Serial2.read();
    Serial.print("Received: ");
    Serial.println(data_rcvd);
    if (data_rcvd == 48) Serial2.write(2);
    if (data_rcvd == 49) Serial2.write(2);
    if (data_rcvd == 50) Serial2.write(2);
  } else {
//    Serial.println("Unavailable");
  }
//  moveForward(150, 1);
//  delay(3000);

//moveForward(283, 1);
//moveForward(1000, 0.8);
//delay(99999);

//turn(150, 0.5);
//delay(99999);

//  turn(-turnAmt*2-10, 0.5);
//  delay(9999);

  getDataDoF();
  if (analogRead(0) > 400) {
    LF -> setSpeed(0);
    LB -> setSpeed(0);
    RF -> setSpeed(0);
    RB -> setSpeed(0);
    delay(500);
    exit(0);
  }
  if (dL > 250) {
    turn(-turnAmt, 0.5);// -150
    dumbForward(1, 90);
//    moveForward(MFAmt, 0.7);
  } else if (dF > 170) {
    dumbForward(1, 90);
//    moveForward(MFAmt, 0.7);// 330
  } else if (dR > 250) {
    turn(turnAmt, 0.5);// 150
    dumbForward(1, 90);
//    moveForward(MFAmt, 0.7);// 330
  } else {
    moveForward(-MFAmt, 0.4);// -330
    turn(-turnAmt*2-10, 0.5);
//    turn(345, 0.5);
  }
  delay(100);

//  getDataDoF();
//  if (rng == 1) {
//    if (dR > wallDist+150) {
//      turn(turnAmt, 0.5);// 150
//      moveForward(MFAmt, 0.7);// 330
//    } else if (dF > wallDist+150) {
//      moveForward(MFAmt, 0.7);// 330
//    } else if (dL > wallDist+150) {
//      turn(-turnAmt, 0.5);// -150
//      moveForward(MFAmt, 0.7);
//    } else {
//      moveForward(MFAmt, 0.4);// -330
//      turn(-turnAmt, 0.5);
//  //    turn(345, 0.5);
//    } 
//  } else {
//    if (dL > wallDist+150) {
//      turn(-turnAmt, 0.5);// -150
//      moveForward(MFAmt, 0.7);
//    } else if (dF > wallDist+150) {
//      moveForward(MFAmt, 0.7);// 330
//    } else if (dR > wallDist+150) {
//      turn(turnAmt, 0.5);// 150
//      moveForward(MFAmt, 0.7);// 330
//    } else {
//      moveForward(-MFAmt, 0.4);// -330
//      turn(-turnAmt, 0.5);
//  //    turn(345, 0.5);
//    }
//  }
//    delay(100);
}
