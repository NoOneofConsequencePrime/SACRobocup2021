#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"
#define S0 3
#define S1 4
#define S2 5
#define S3 6
#define sensorOut 11
// distance sensors
#include "HCSR04.h"

#define TRIG_PIN_Left 12
#define ECHO_PIN_Left 11
#define TRIG_PIN_Mid 12
#define ECHO_PIN_Mid 11
#define TRIG_PIN_Right 12
#define ECHO_PIN_Right 11

HCSR04 sr04 = HCSR04(ECHO_PIN_Left,TRIG_PIN_Left);

long distLeft, distRight, distMid;




// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

// Select which 'port' M1, M2, M3 or M4.
Adafruit_DCMotor *myMotor1 = AFMS.getMotor(3);
Adafruit_DCMotor *myMotor2 = AFMS.getMotor(4);
Adafruit_DCMotor *myMotor3 = AFMS.getMotor(1);
Adafruit_DCMotor *myMotor4 = AFMS.getMotor(2);

const int maxSpeed = 150;

int frequencyG, frequencyR;

void setup() {
  Serial.begin(9600);
  delay(1000);
  Serial.begin(9600);           // set up Serial library at 115200 bps
  Serial.println("Adafruit Motorshield v2 - DC Motor test!");

  AFMS.begin();  // create with the default frequency 1.6KHz
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(sensorOut, INPUT);
  
  // Setting frequency-scaling to 20%
  digitalWrite(S0,HIGH);
  digitalWrite(S1,LOW);
  myMotor1->setSpeed(0);
  myMotor1->run(RELEASE);

  myMotor2->setSpeed(0);
  myMotor2->run(RELEASE);

  myMotor3->setSpeed(0);
  myMotor3->run(RELEASE);

  myMotor4->setSpeed(0);
  myMotor4->run(RELEASE);
  Serial.begin(115200);
}

void loop() {
  Serial.print("Distance Left = ");
  distLeft = distanceDetect(ECHO_PIN_Left, TRIG_PIN_Left);
  Serial.print("Distance Mid = ");
  distMid = distanceDetect(ECHO_PIN_Mid, TRIG_PIN_Mid);
  Serial.print("Distance Right = ");
  distRight = distanceDetect(ECHO_PIN_Right, TRIG_PIN_Right);

  if(distMid <= 50){
    
    if(distLeft > distRight){
      goLeft();
      }
     else{
      goRight();
        }
    
    }
    else{
      forward(maxSpeed);
      }

//  forward(maxSpeed);
//  delay(3000);
//  stop();
//  backwards(maxSpeed);
//  delay(3000);
//  stop();
//  goLeft();
//  delay(3000);
//  stop();
//  goRight();
//  delay(3000);
//  stop();
  
  

  
  // Setting red filtered photodiodes to be read
  frequencyR = getColor(LOW, LOW); //get amount of RED
  // Printing the value on the serial monitor
  Serial.print("R= ");//printing name
  Serial.print(frequencyR);//printing RED color frequency
  Serial.println("  ");
  delay(100);

  
  frequencyG = getColor(HIGH, HIGH); //get amount of GREEN
  // Printing the value on the serial monitor
  Serial.print("G= ");//printing name
  Serial.print(frequencyG);//printing GREEN color frequency
  Serial.print("  ");
  delay(100);

  if(frequencyR >= 100 && frequencyR <=270 && frequencyG >=70 && frequencyG <= 250){ // silver
    Serial.print("      Silver      ");
    delay(100);
    }
    else if(frequencyR < 100 && frequencyG < 70){ // black
    Serial.print("      Black       ");
    delay(100);
    }
    else if(frequencyR > 270 & frequencyG > 250){ // white
    Serial.print("      White       ");
    delay(100);
    }
}

long distanceDetect(int ECHO, int TRIG){ // distance method
sr04 = HCSR04(ECHO,TRIG);
Serial.print(sr04.dist());
  Serial.println("cm");
return sr04.dist();
}

int getColor(int filter2, int filter3){
  int amount;
  // Setting colour filtered photodiodes to be read
  digitalWrite(S2,filter2);
  digitalWrite(S3, filter3);
  // Reading the output frequency
  amount = pulseIn(sensorOut, LOW);
  //Remaping the value of the frequency to the RGB Model of 0 to 255
  amount = map(amount, 25,90,255,0);
  return amount;
}

void forward(int speed){
  myMotor1->setSpeed(speed);  
  myMotor1->run(FORWARD);

  myMotor2->setSpeed(speed);  
  myMotor2->run(FORWARD);

  myMotor3->setSpeed(speed);  
  myMotor3->run(FORWARD);

  myMotor4->setSpeed(speed);  
  myMotor4->run(FORWARD);
}

void backwards(int speed){
  myMotor1->setSpeed(speed);  
  myMotor1->run(BACKWARD);

  myMotor2->setSpeed(speed);  
  myMotor2->run(BACKWARD);

  myMotor3->setSpeed(speed);  
  myMotor3->run(BACKWARD);

  myMotor4->setSpeed(speed);  
  myMotor4->run(BACKWARD);
}

void stop(){
  myMotor1->run(RELEASE);
  myMotor2->run(RELEASE);
  myMotor3->run(RELEASE);
  myMotor4->run(RELEASE);
}
void goRight(){
  myMotor1->setSpeed(maxSpeed);  
  myMotor1->run(BACKWARD);

  myMotor2->setSpeed(maxSpeed);  
  myMotor2->run(FORWARD);

  myMotor3->setSpeed(maxSpeed);  
  myMotor3->run(FORWARD);

  myMotor4->setSpeed(maxSpeed);  
  myMotor4->run(BACKWARD);
}
void goLeft(){
  myMotor1->setSpeed(maxSpeed);  
  myMotor1->run(FORWARD);

  myMotor2->setSpeed(maxSpeed);  
  myMotor2->run(BACKWARD);

  myMotor3->setSpeed(maxSpeed);  
  myMotor3->run(BACKWARD);

  myMotor4->setSpeed(maxSpeed);  
  myMotor4->run(FORWARD);
}

  
