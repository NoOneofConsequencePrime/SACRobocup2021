#define S0 4
#define S1 5
#define S2 6
#define S3 7
#define sensorOut 8

void setup() {
  // put your setup code here, to run once:
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(sensorOut, INPUT);

  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);

  
}

void loop() {
  // red number check
  digitalWrite(S2, LOW);
  digitalWrite(S3, LOW);
  redcount = pulseIn(sensorOut, LOW);
  digitalWrite(S2, HIGH);
  digitalWrite(S3, HIGH);
  greencount = pulseIn(sensorOut, LOW);
  digitalWrite(S2, LOW);  
  digitalWrite(S3, HIGH);
  bluecount = pulseIn(sensorOut, LOW);

}
