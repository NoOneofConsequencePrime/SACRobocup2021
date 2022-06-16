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

// DoF Pins
#define LOXF_ADDRESS 0x30
#define LOXL_ADDRESS 0x31
#define LOXR_ADDRESS 0x32
#define SHT_LOXL 6
#define SHT_LOXF 5
#define SHT_LOXR 4

#define GRAYSCALE_PIN 8

// Camera Pins
#define P0L 29
#define P1L 30
#define P2L 31
#define P0R 23
#define P1R 24
#define P2R 25

// Settings
const int maxSpd = 255;
const int wasteDelay = 5;
const int moveWait = 250;
const int moveDist = 275;
const db turnDist = 90;
const int wallDist = 80;// mm
const int wallDetect = 120;// mm
const db P_coeff = 0.6;
const int blackTile = 300, silverTile = 150;
const int ltr_H = 3, ltr_S = 5, ltr_R = 6, ltr_Y = 1, ltr_U = 2, ltr_G = 4;

// Data
db gz;
int dF, dL, dR;
int gsVal;
char camL, camR;

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

void setID() {
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

void getDataAmbient() {
  gsVal = analogRead(GRAYSCALE_PIN);
}

void getDataCamera() {
  int pL = (digitalRead(P0L)<<2)|(digitalRead(P1L)<<1)|digitalRead(P2L);
  int pR = (digitalRead(P0R)<<2)|(digitalRead(P1R)<<1)|digitalRead(P2R);
  if (pR == ltr_H) camR = 'H';
  else if (pR == ltr_S) camR = 'S';
  else if (pR == ltr_R) camR = 'R';
  else if (pR == ltr_Y) camR = 'Y';
  else if (pR == ltr_U) camR = 'U';
  else if (pR == ltr_G) camR = 'G';
  else camR = 'N';
  if (pL == ltr_H) camL = 'H';
  else if (pL == ltr_S) camL = 'S';
  else if (pL == ltr_R) camL = 'R';
  else if (pL == ltr_Y) camL = 'Y';
  else if (pL == ltr_U) camL = 'U';
  else if (pL == ltr_G) camL = 'G';
  else camL = 'N';
//  lcd.setCursor(0, 0);
////  lcd.print(pL);
//  lcd.print(digitalRead(P0L)*100+digitalRead(P1L)*10+digitalRead(P2L));
  lcd.setCursor(0, 0);
//  lcd.print(pR);
  lcd.print(digitalRead(P0R)*100+digitalRead(P1R)*10+digitalRead(P2R));
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

void turn(db inpRot, db inpSpd, db errorM, int fixCnt) {
//  if (abs(inpRot) < errorM) return;
  if (fixCnt >= 2) return;
  
  getDataMPU();
  db curZ = gz;
  int spd = round(inpSpd*maxSpd);
  setMotorSpd(spd);
  
  if (inpRot > 0) {
    setMotorDir(1, 0);
    while (gz-curZ < inpRot) {
//      lcd.clear(); lcd.setCursor(0, 1);
//      lcd.println(String(gz-curZ)+"; "+String(inpRot));
//      lcd.print(gz);
      getDataMPU();
    }
  } else if (inpRot < 0) {
    setMotorDir(0, 1);
    while (gz-curZ > inpRot) {
//      lcd.clear(); lcd.setCursor(0, 1);
//      lcd.println(String(gz-curZ)+"; "+String(inpRot));
//      lcd.print(gz);
      getDataMPU();
    }
  }

//  lcd.print(String(curZ+inpRot)+"; "+String(gz));
  turn(curZ+inpRot-gz, inpSpd*P_coeff, errorM, fixCnt+1);
  setMotorSpd(0);
  if (fixCnt == 0) delay(moveWait);
}

void moveForward(int inpDist, db inpSpd, int errorM, int fixCnt) {
  if (abs(inpDist) < errorM || inpSpd < 0.3 || dF < wallDist) return;
//  if (fixCnt >= 2) return;

  getDataDoF('F');
  int curDF = dF;
  int spd = round(inpSpd*maxSpd);
  setMotorSpd(spd);

  if (inpDist > 0) {
    setMotorDir(1, 1);
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
//        lcd.clear();
//        lcd.print("Silver Detected");
      }
      if (dF < wallDist) {
        setMotorSpd(0);
        if (fixCnt == 0) delay(moveWait);
        
        return;
      }
    }
  } else if (inpDist < 0) {
    setMotorDir(0, 0);
    while (curDF-dF > inpDist) {
      getDataDoF('F');
    }
  }

  moveForward(-(curDF-inpDist-dF), inpSpd*P_coeff, errorM, fixCnt+1);
  setMotorSpd(0);
  if (fixCnt == 0) delay(moveWait);
}

void debug() {
//  getDataMPU();
//  getDataDoF('F');
  getDataCamera();
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
  getDataDoF('A');
  if (dL > wallDetect) {
    turn(-turnDist, 1, 0.1, 0);
  } else if (dF > wallDetect) {
  } else if (dR > wallDetect) {
    turn(turnDist, 1, 0.1, 0);
  } else {
    turn(2*turnDist, 1, 0.1, 0);
  }
  moveForward(moveDist, 0.7, 8, 0);
}

void loop() {
//  moveForward(moveDist, 0.7, 8, 0);
//  moveForward(moveDist, 0.7, 8, 0);
//  delay(99999);
  
  debug();
//  turn((random(5)-2)*90, 1, 0.1, 0);
}
