#define S0 4
#define S1 5
#define S2 6
#define S3 7
#define sensorOut 8

bool black_tile = false;
bool checkpoint = true;

void setup() {
  Serial.begin(115200);      // Serial monitor
  Serial2.begin(115200);     // baud rate set at 115200, the rest of the parameters are default, USE PINS 16, 17 NOT 18, 19
  pinMode(13, OUTPUT);
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(sensorOut, INPUT);

  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);

}

void Dispense(int value, int receiving) {
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
  
  //writing done so camera knows to start detecting again
  Serial2.write(value);
  digitalWrite(13, HIGH);
  delay(500);
  digitalWrite(13, LOW);  
}

void check_colour() {
  digitalWrite(S2, LOW);
  digitalWrite(S3, LOW);
  int redcount = pulseIn(sensorOut, LOW);
  digitalWrite(S2, HIGH);
  digitalWrite(S3, HIGH);
  int greencount = pulseIn(sensorOut, LOW);
  digitalWrite(S2, LOW);  
  digitalWrite(S3, HIGH);
  int bluecount = pulseIn(sensorOut, LOW);
  
  if (redcount > 75 && greencount > 75 && bluecount > 75) {
    Serial.println("Black");
    black_tile = true;
    checkpoint = false;
  }
  else if (redcount < 30 && greencount < 30 && bluecount < 30) {
    Serial.println("White");
    black_tile = false;
    checkpoint = false;
  }
  else if (redcount > 35 && redcount < 50 && greencount > 35 && greencount < 50 && bluecount > 35 && bluecount < 50) {
    Serial.println("Silver");
    black_tile = false;
    checkpoint = true;
  }

}

void loop() {
  check_colour();
  if (Serial2.available()) {
    int data_rcvd = Serial2.read();
    Serial.print("Received: ");
    Serial.println(data_rcvd);
    if (data_rcvd == 48) Dispense(2, 0);
    if (data_rcvd == 49) Dispense(2, 1);
    if (data_rcvd == 50) Dispense(2, 2);
  }
  else {
//    Serial.println("Unavailable");
  }
}
