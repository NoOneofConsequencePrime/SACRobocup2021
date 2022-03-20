#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();

  // initialize serial communication
  Serial.begin(115200);
}

void loop() {
  
}
