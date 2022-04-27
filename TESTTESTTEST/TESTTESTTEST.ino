#include <ArduinoSort.h>

void setup() {
  Serial.begin(9600);
  while (!Serial) {delay(1);}
  delay(100);
}

void loop() {

  // Set up array of 5 numbers
  int arr[3][6];
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 6; j++) {
      arr[i][j] = random(100);
    }
  }

  sortArray(arr[0], 6);
  sortArray(arr[1], 6);
  sortArray(arr[2], 6);

  for (int i = 0; i < 3; i++) {
    Serial.println(String(i)+": ");
    for (int j = 0; j < 6; j++) {
      Serial.print(String(arr[i][j])+", ");
    }
    Serial.println();
  }
}
