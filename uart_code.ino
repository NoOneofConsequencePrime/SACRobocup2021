void setup() {
  Serial.begin(9600);      // Serial monitor
  Serial2.begin(115200);     // baud rate set at 115200, the rest of the parameters are default
  pinMode(13, OUTPUT);
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

void loop() {
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
