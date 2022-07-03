// Convenience
typedef double db;
typedef long long ll;
typedef unsigned long ul;

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
#include "SR04.h"
#include "Servo.h"

// DoF Pins
#define LOXF_ADDRESS 0x30
#define LOXL_ADDRESS 0x31
#define LOXR_ADDRESS 0x32
#define SHT_LOXL 39
#define SHT_LOXF 41
#define SHT_LOXR 43

#define GRAYSCALE_PIN 8
#define TRIG_PIN 5
#define ECHO_PIN 4
#define SERVO_PIN 25
#define ID_PIN 53

// Camera Pins
#define P0L 26
#define P1L 24
#define P2L 22
#define P0R 33
#define P1R 31
#define P2R 29

// Settings
const int wasteDelay = 2;// ms
const int maxSpd = 255;
const int moveWait = 250;
const int moveDist = 300;
const db turnDist = 89;// degrees
const int wallDist = 80;// mm
const int sonicDist = 450;// mm
const int sonicLim = 50;// mm
const int wallDetect = 250;// mm
const db P_coeff = 0.7;
const int blackTile = 300, silverTile = 150;
const int ltr_H = 7, ltr_S = 3, ltr_RY = 5, ltr_UG = 6;
const int moveMargin = 0;// 5
const db turnMargin = 0;// 0.1
const int kitOpen = 50, kitClose = 95;
const int kitDelay = 300;
const int fwdCrCnt = 4;
const int preRampDelay = 800;
const int minRampDist = 2000;// mm
//const int bwdCrDist = 10;// mm

// Data
db gz;
int dF, dL, dR, sonicDF;
int gsVal;
String camL, camR;

// Hardware
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *RB = AFMS.getMotor(4);
Adafruit_DCMotor *LB = AFMS.getMotor(3);
Adafruit_DCMotor *RF = AFMS.getMotor(1);
Adafruit_DCMotor *LF = AFMS.getMotor(2);

Adafruit_VL53L0X loxL = Adafruit_VL53L0X();
Adafruit_VL53L0X loxF = Adafruit_VL53L0X();
Adafruit_VL53L0X loxR = Adafruit_VL53L0X();
VL53L0X_RangingMeasurementData_t measureL;
VL53L0X_RangingMeasurementData_t measureF;
VL53L0X_RangingMeasurementData_t measureR;

MPU6050 mpu6050(Wire);
LiquidCrystal lcd(7, 8, 9, 10, 11, 12);
SR04 sr04 = SR04(ECHO_PIN, TRIG_PIN);
Servo kitServo;

void setID() {
  // Servo
  kitServo.attach(SERVO_PIN);

  // ID LED
  pinMode(ID_PIN, OUTPUT);
  
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

  // LCD
  lcd.begin(16, 2);
}

void getDataMPU() {
  mpu6050.update();
  gz = -(mpu6050.getAngleZ())/2;
}

void getDataDoF(char c) {// F, A, S
  delay(30);
  // DoF sensors
  if (c != 'S') {
    loxF.rangingTest(&measureF, false);
    getDataSonic();
  }
  if (c != 'F') loxL.rangingTest(&measureL, false);
  if (c != 'F') loxR.rangingTest(&measureR, false);

  if (c != 'S' && measureF.RangeStatus != 4) {
    dF = measureF.RangeMilliMeter;
    if (dF < sonicDist || sonicDF < sonicLim) dF = sonicDF;
//    if (dF < sonicDist || abs(dF-sonicDF) > sonicThreshold) dF = sonicDF;
  } else dF = 9999;
  if (c != 'F' && measureL.RangeStatus != 4) {
    dL = measureL.RangeMilliMeter;
  } else dL = 9999;
  if (c != 'F' && measureR.RangeStatus != 4) {
    dR = measureR.RangeMilliMeter;
  } else dR = 9999;
  
  // https://pastebin.com/WcuVfiu5
}

void getDataSonic() {
  sonicDF = sr04.Distance()*10;
}

void getDataAmbient() {
  gsVal = analogRead(GRAYSCALE_PIN);
}

void getDataCamera() {
  int pL = (digitalRead(P0L)<<2)|(digitalRead(P1L)<<1)|digitalRead(P2L);
  int pR = (digitalRead(P0R)<<2)|(digitalRead(P1R)<<1)|digitalRead(P2R);
  getDataDoF('S');
  if (pL == ltr_H && dL < wallDetect) camL = "H";
  else if (pL == ltr_S && dL < wallDetect) camL = "S";
  else if (pL == ltr_RY && dL < wallDetect) camL = "RY";
  else if (pL == ltr_UG && dL < wallDetect) camL = "UG";
  else camL = "N";
  if (pR == ltr_H && dR < wallDetect) camR = "H";
  else if (pR == ltr_S && dR < wallDetect) camR = "S";
  else if (pR == ltr_RY && dR < wallDetect) camR = "RY";
  else if (pR == ltr_UG && dR < wallDetect) camR = "UG";
  else camR = "N";
//  lcd.clear();
//  lcd.setCursor(0, 0);
//  lcd.print(camL);
//  lcd.setCursor(0, 1);
//  lcd.print(camR);
//  Serial.println(camL+"; "+camR);
}

void setup() {
  delay(300);
  Serial.begin(115200);
  while (!Serial) {delay(1);}
  Serial.println("Initializing...");
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(1);
  pinMode(SHT_LOXF, OUTPUT);
  pinMode(SHT_LOXL, OUTPUT);
  pinMode(SHT_LOXR, OUTPUT);

  pinMode(P0L, INPUT);
  pinMode(P1L, INPUT);
  pinMode(P2L, INPUT);
  pinMode(P0R, INPUT);
  pinMode(P1R, INPUT);
  pinMode(P2R, INPUT);
  
  setID();

  Serial.println("---Startup Complete---");
  lcd.print("---Complete---");
  delay(200);
  lcd.clear();
}

void setMotorSpd(int spd) {
  LF -> setSpeed(spd);
  LB -> setSpeed(spd);
  RF -> setSpeed(spd);
  RB -> setSpeed(spd);
}

void setMotorDir(bool l, bool r) {
  if (l) {
    LF -> run(FORWARD);
    LB -> run(FORWARD);
  } else {
    LF -> run(BACKWARD);
    LB -> run(BACKWARD);
  }
  if (r) {
    RF -> run(FORWARD);
    RB -> run(FORWARD);
  } else {
    RF -> run(BACKWARD);
    RB -> run(BACKWARD);
  }
}

void setMotorRel() {
  LF -> run(RELEASE);
  LB -> run(RELEASE);
  RF -> run(RELEASE);
  RB -> run(RELEASE);
}

void turn(db inpRot, db inpSpd, db errorM, int fixCnt) {
//  if (abs(inpRot) < errorM) return;
  if (fixCnt >= 2) return;
  
  getDataMPU();
  db curZ = gz;
  int spd = round(inpSpd*maxSpd);
  setMotorRel();
  setMotorSpd(spd);
  
  if (inpRot > 0) {
    setMotorDir(1, 0);
    while (abs(gz-curZ) < abs(inpRot)) {
      setMotorDir(1, 0);
      getDataMPU();
      delay(wasteDelay);
    }
  } else if (inpRot < 0) {
    setMotorDir(0, 1);
    while (abs(gz-curZ) < abs(inpRot)) {
      setMotorDir(0, 1);
      getDataMPU();
      delay(wasteDelay);
    }
  }

  setMotorSpd(0);
  getDataMPU();
  turn(curZ+inpRot-gz, inpSpd*P_coeff, errorM, fixCnt+1);
  setMotorSpd(0);
  if (fixCnt == 0) delay(moveWait);
  setMotorRel();
}

int fwdCnt = 0;
void moveForward(int inpDist, db inpSpd, int errorM, int fixCnt) {
  if (abs(inpDist) < errorM || inpSpd < 0.3 || (dF < wallDist && inpDist > 0)) return;
//  if (fixCnt >= 2) return;

  getDataDoF('F');
  int curDF = dF;
  int spd = round(inpSpd*maxSpd);
  setMotorRel();
  setMotorSpd(spd);

  if (inpDist > 0) {
    getDataDoF('F');
    if (fixCnt == 0) {
      setMotorDir(1, 1);
      ul stMS = millis();
      while (millis()-stMS < preRampDelay) {
        getDataAmbient();
        if (gsVal > blackTile) {
          setMotorSpd(0);
          delay(moveWait);
          getDataDoF('F');
          moveForward(dF-curDF, inpSpd, errorM, 0);
          turn(2*turnDist, 1, 0.1, 0);
          return;
        }
        delay(wasteDelay);
      }
    }
    getDataDoF('F');
    setMotorDir(1, 1);
    if (dF > minRampDist && fixCnt == 0) {
      while (dF > wallDist) {
        getDataDoF('F');
        getDataAmbient();
        if (gsVal > blackTile) {
          setMotorSpd(0);
          delay(moveWait);
          getDataDoF('F');
          moveForward(dF-curDF, inpSpd, errorM, 0);
          turn(2*turnDist, 1, 0.1, 0);
          return;
        }
        delay(wasteDelay);
      }
      setMotorSpd(0);
      if (fixCnt == 0) delay(moveWait);
      return;
    }
    if (dF < wallDetect*2 && fixCnt == 0) {
      while (dF > wallDist) {
        getDataDoF('F');
        getDataAmbient();
        if (gsVal > blackTile) {
          setMotorSpd(0);
          delay(moveWait);
          getDataDoF('F');
          moveForward(dF-curDF, inpSpd, errorM, 0);
          turn(2*turnDist, 1, 0.1, 0);
          return;
        }
        delay(wasteDelay);
      }
      if (fwdCnt == fwdCrCnt) {
        fwdCnt = 0;
        delay(250);// Move to touch front wall
//        setMotorSpd(0);
//        delay(moveWait);
//        getDataDoF('F');
//        moveForward(-bwdCrDist, inpSpd, errorM, 0);
      }
      setMotorSpd(0);
      if (fixCnt == 0) delay(moveWait);
      setMotorRel();
      fwdCnt++;
      return;
    } else {
      while (curDF-dF < inpDist) {
        getDataDoF('F');
        getDataAmbient();
        if (gsVal > blackTile) {
          setMotorSpd(0);
          delay(moveWait);
          getDataDoF('F');
          moveForward(dF-curDF, inpSpd, errorM, 0);
          turn(2*turnDist, 1, 0.1, 0);
          return;
        } else if (gsVal > silverTile) {
//          lcd.clear();
//          lcd.print("Silver Detected");
        }
        if (dF < wallDist) {
          setMotorSpd(0);
          if (fixCnt == 0) delay(moveWait);
          setMotorRel();
          return;
        }
        delay(wasteDelay);
      }
    }
  } else if (inpDist < 0) {
    setMotorDir(0, 0);
    while (curDF-dF > inpDist) {
      getDataDoF('F');
      delay(wasteDelay);
    }
  }

  moveForward(-(curDF-inpDist-dF), inpSpd*P_coeff, errorM, fixCnt+1);
  setMotorSpd(0);
  if (fixCnt == 0) delay(moveWait);
  setMotorRel();
}

void deployKit() {
  kitServo.write(kitOpen);
  delay(kitDelay);
  kitServo.write(kitClose);
  delay(kitDelay);
}

void flashLED() {
  digitalWrite(ID_PIN, HIGH);
  delay(5000);
  digitalWrite(ID_PIN, LOW);
}

void chkKit() {
  getDataCamera();
  if (camL != "N") flashLED();
  if (camL == "H") {
    turn(turnDist, 1, turnMargin, 0);
    deployKit(); deployKit();// deployKit();
    turn(-turnDist, 1, turnMargin, 0);
  } else if (camL == "S") {
    turn(turnDist, 1, turnMargin, 0);
    deployKit(); deployKit();
    turn(-turnDist, 1, turnMargin, 0);
  } else if (camL == "RY") {
    turn(turnDist, 1, turnMargin, 0);
    deployKit();
    turn(-turnDist, 1, turnMargin, 0);
  }
  
  getDataCamera();
  if (camR != "N") flashLED();
  if (camR == "H") {
    turn(-turnDist, 1, turnMargin, 0);
    deployKit(); deployKit();// deployKit();
    turn(turnDist, 1, turnMargin, 0);
  } else if (camR == "S") {
    turn(-turnDist, 1, turnMargin, 0);
    deployKit(); deployKit();
    turn(turnDist, 1, turnMargin, 0);
  } else if (camR == "RY") {
    turn(-turnDist, 1, turnMargin, 0);
    deployKit();
    turn(turnDist, 1, turnMargin, 0);
  } 
}

void debug() {
//  lcd.clear();
//  lcd.setCursor(0, 0);
//  lcd.print(dF);
  getDataMPU();
//  getDataDoF('F');
  Serial.println(gz);
//  getDataCamera();
//  delay(500);
//  Serial.println("dL: "+String(dL));
//  Serial.println("dF: "+String(dF));
//  Serial.println("dR: "+String(dR));
//  Serial.println(gz, 4);

//  Serial.println(String(p0l)+"; "+String(p1l)+"; "+String(p2l));
//  Serial.println(String(p0r)+"; "+String(p1r)+"; "+String(p2r));
//  lcd.clear();
//  lcd.setCursor(0, 0);
//  lcd.print(String(camL)+"; "+String(camR));
//  lcd.setCursor(0, 1);
//  lcd.print(camR);
//  Serial.println();
}

void simpleWallFollow() {
  chkKit();
  getDataDoF('A');
  if (dL > wallDetect) {
    turn(-turnDist, 1, turnMargin, 0);
    chkKit();
  } else if (dF > wallDetect) {
  } else if (dR > wallDetect) {
    turn(turnDist, 1, turnMargin, 0);
    chkKit();
  } else {
    turn(turnDist, 1, turnMargin, 0);
    chkKit();
    turn(turnDist, 1, turnMargin, 0);
    chkKit();
  }
  getDataDoF('F');
  moveForward(moveDist, 0.7, moveMargin, 0);
  delay(wasteDelay);
}

void loop() {
  simpleWallFollow();
//  getDataCamera();
//  getDataDoF('F');
//  getDataSonic();
//  getDataMPU();
//  moveForward(moveDist, 0.7, 8, 0);
//  moveForward(moveDist, 0.7, 8, 0);
//  delay(99999);
  
//  debug();
//  turn((random(5)-2)*90, 1, 0.1, 0);
}
