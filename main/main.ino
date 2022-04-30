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
#define LEDPIN 9

/* 
 * Setup
 */

// Settings
//const db gyroOffset = -0.017884;
const db gyroOffset = -0.00713494;
//const db gyroOffset = -0.1;
const int dofCnts = 3;
const int maxSpd = 255;
const int wasteDelay = 20;
const int moveWait = 300;
const int blockDelay = 1400;
const int distFromWall = 80;
const int turnWait90 = 960;

// Data
db gx, gy, gz;// x and z
unsigned long curTime;
int dF, dL, dR;
int wallDist;

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
//    while (1) {
//      delay(10);
//    }
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

  int turnReduc = (db)spd*spd/5700.0;
  if (inpRot > 0) {// right (cw)
    
    LF -> run(FORWARD);
    LB -> run(FORWARD);
    RF -> run(BACKWARD);
    RB -> run(BACKWARD);
    
//    delay(turnWait90 * inpRot/90);
    getDataMPU();
    db curGZ = gz;
    while (gz-curGZ+turnReduc < inpRot) {// +turnReduc
//      digitalWrite(LEDPIN, HIGH);
      Serial.println(String(gz)+" "+String(curGZ)+" "+String(gz-curGZ));
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
    db curGZ = gz;
    while (gz-curGZ+turnReduc > inpRot) {// +turnReduc
      if (gz-curGZ < inpRot || gz-curGZ > -inpRot) break;
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
  Serial.println(gz, 4);
  Serial.println("\n");
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);
  Serial.println("Initializing...");
  while (!Serial) {delay(1);}
  
  pinMode(13, OUTPUT);
  pinMode(SHT_LOXF, OUTPUT);
  pinMode(SHT_LOXL, OUTPUT);
  pinMode(SHT_LOXR, OUTPUT);
  pinMode(LEDPIN, OUTPUT);
  
  setID();
  setupData();
  delay(200);
  
  Serial.println("---Startup Complete---");
}

void dispense(int value, int receiving) {
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
  Serial2.write(value);
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
//  turn(90, 0.7);
//  turn(-90, 0.7);
//  turn(180, 0.7);
//  delay(99999);
//  for (int i = 0; i < 4; i++) {
//    moveForward(200, 1);
//    delay(100000);
//  }
//  delay(200000);
  
//  if (Serial2.available()) {
//    int data_rcvd = Serial2.read();
//    Serial.print("Received: ");
//    Serial.println(data_rcvd);
//    if (data_rcvd == 48) dispense(2, 0);
//    if (data_rcvd == 49) dispense(2, 1);
//    if (data_rcvd == 50) dispense(2, 2);
//  } else {
//    Serial.println("Unavailable");
//  }
//  moveForward(150, 1);
//  delay(3000);

//moveForward(283, 1);
//moveForward(1000, 0.8);
//delay(99999);

//turn(150, 0.5);
//delay(99999);

  getDataDoF();
  if (dL > wallDist+200) {
    turn(-90, 0.5);// -150
    moveForward(240, 0.7);
  } else if (dF > wallDist+200) {
    moveForward(240, 0.7);// 330
  } else if (dR > wallDist+200) {
    turn(90, 0.5);// 150
    moveForward(240, 0.7);// 330
  } else {
    moveForward(-300, 0.4);// -330
//    turn(300, 0.5);
  }
  delay(100);
}
