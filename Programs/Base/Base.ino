

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"
#include "SR04.h"
#include <Servo.h>


#define TrigPinFront 29
#define EchoPinFront 28
#define TrigPinLeft  25
#define EchoPinLeft 24
#define TrigPinRight 27
#define EchoPinRight 26
SR04 sr04Front = SR04(EchoPinFront, TrigPinFront); 
SR04 sr04Left = SR04(EchoPinLeft, TrigPinLeft);
SR04 sr04Right = SR04(EchoPinRight, TrigPinRight);
long frontDist, leftDist, rightDist; 




#define S0 37
#define S1 34
#define S2 38
#define S3 36
#define sensorOut 39
int frequency = 0;





Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *myMotor = AFMS.getMotor(1);
Adafruit_DCMotor *myMotor2 = AFMS.getMotor(2);
Adafruit_DCMotor *myMotor3 = AFMS.getMotor(3);
Adafruit_DCMotor *myMotor4 = AFMS.getMotor(4);
int speedLeft = 145;
int speedRight = 155;
int incr = 10;



int p0 = 44;
int p1 = 45;



Servo myservo;  
int pos = 0;   
int j = 0;

void setup() {
  
  Serial.begin(115200);           // set up Serial library at 115200 bps

  AFMS.begin();  // create with the default frequency 1.6KHz
  
pinMode(S0, OUTPUT);
pinMode(S1, OUTPUT);
pinMode(S2, OUTPUT);
pinMode(S3, OUTPUT);
pinMode(sensorOut, INPUT);

digitalWrite(S0, HIGH);
digitalWrite(S1, LOW);



pinMode(p0, INPUT);
pinMode(p1, INPUT);


myservo.attach(7);
 
}

void loop() {


  
distDetect();

colorsensor();


 if (rightDist < 25 && frontDist > 15 ){
  drive();
 
 } else if (rightDist > 25){
      delay(100);
      anhalten();
      turnright();
     
      delay(500); 
    } else if (frontDist < 15 ){
      
      anhalten();

      if (leftDist > rightDist){
        turnleft();
       // anhalten();
      //  camera();
      }
      if (rightDist > leftDist){
        turnright();
       // anhalten();
      //  camera();
      }
  
    }    


} 




void distDetect(){
 
  frontDist=sr04Front.Distance(); 
  leftDist=sr04Left.Distance();
  rightDist=sr04Right.Distance();
  
  
  Serial.print(" \tFront: "); 
  Serial.print(frontDist); 
  Serial.print(" \tLeft: "); 
  Serial.print(leftDist);
  Serial.print(" \tRight: "); 
  Serial.println(rightDist);  


  
}

void colorsensor(){

digitalWrite(S2,LOW);
digitalWrite(S3,HIGH);

frequency = pulseIn (sensorOut, LOW);

Serial.println(frequency);

frequency = map (frequency, 25,70,255,0);

if (frequency < 0) {
 // backward();
  anhalten();
  delay(10000);
/*if (leftDist > rightDist){
        turnleft();
        delay(800);
         //     }
   /*  if (rightDist > leftDist){
        turnright();
        delay(1000);
      }
  
}*/
  
}
}

void camera(){

int valP0 = digitalRead(p0);
int valP1 = digitalRead(p1);
  
Serial.println(valP0);
Serial.println(valP1);
Serial.println("hi");  

  
if (rightDist < 7){     
    if (valP0 != 0 || valP1 != 0 ) { 
        tworescue();
    } 
      } 
   
}

void drive(){



 /* if (rightDist > leftDist && rightDist < 15 && leftDist < 15){
            speedLeft = speedLeft + incr;
            speedRight = speedRight - incr;
            Serial.println("right");
        }
        
    if (leftDist > rightDist && rightDist < 15 && leftDist < 15){
                 speedRight = speedRight + incr;
                 speedLeft = speedLeft - incr;
                 Serial.println("left");
          }
delay(200);

      if (speedRight >= 90||  speedLeft >= 90 || speedRight <= 50 || speedLeft <= 50 ){
          speedLeft = 70;
          speedRight = 70;
                Serial.println("reset");
        } */
  myMotor->setSpeed(speedLeft);  
  myMotor->run(BACKWARD);

  myMotor2->setSpeed(speedRight);  
  myMotor2->run(BACKWARD);

  myMotor3->setSpeed(speedRight);  
  myMotor3->run(FORWARD);

  myMotor4->setSpeed(speedLeft);  
  myMotor4->run(FORWARD);

         
}

void anhalten(){

  myMotor->setSpeed(0);
  myMotor->run(RELEASE);

  myMotor2->setSpeed(0);
  myMotor2->run(RELEASE);

  myMotor3->setSpeed(0);
  myMotor3->run(RELEASE);

  myMotor4->setSpeed(0);
  myMotor4->run(RELEASE);

  delay (100);
  Serial.println("stop");
  
}

void turnleft(){
  
  
  myMotor->setSpeed(90);  
  myMotor->run(FORWARD);

  myMotor2->setSpeed(90);  
  myMotor2->run(BACKWARD);

  myMotor3->setSpeed(90);  
  myMotor3->run(FORWARD);

  myMotor4->setSpeed(90);  
  myMotor4->run(BACKWARD);

  delay (600);

  drive();
  Serial.println("left");
  
  
}

void turnright(){


  myMotor->setSpeed(90);  
  myMotor->run(BACKWARD);

  myMotor2->setSpeed(90);  
  myMotor2->run(FORWARD);

  myMotor3->setSpeed(90);  
  myMotor3->run(BACKWARD);

  myMotor4->setSpeed(90);  
  myMotor4->run(FORWARD);


  delay(600);

  drive();
  Serial.println("right");
  
}

void backward(){

  myMotor->setSpeed(speedLeft);  
  myMotor->run(FORWARD);

  myMotor2->setSpeed(speedRight);  
  myMotor2->run(FORWARD);

  myMotor3->setSpeed(speedRight);  
  myMotor3->run(BACKWARD);

  myMotor4->setSpeed(speedLeft);  
  myMotor4->run(BACKWARD);

Serial.println("back");

  delay(500);

}

void threerescue(){

Serial.println("drei");

  anhalten();

   for (j = 0;j < 3; j += 1){

      for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
         myservo.write(pos);
         }

      delay(1000);
          
      for (pos = 180; pos >= 90; pos -= 1) { // goes from 180 degrees to 0 degrees
          myservo.write(pos);   
         }
  
      delay(1000);
   }
   

pos = 0;
drive();

delay(20);
   
}


void tworescue(){

Serial.println("zwei");
 
anhalten();
 
  for (j = 0;j < 2; j += 1){
    
    for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
         myservo.write(pos);
      }
      delay(1000);

      for (pos = 180; pos >= 90; pos -= 1) { // goes from 180 degrees to 0 degrees
          myservo.write(pos);              
     
      }
      delay(1000);
    
  }
  
 pos = 0;
 drive();


 delay(20);  
}
