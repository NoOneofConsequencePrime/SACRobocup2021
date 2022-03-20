#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// Components
Adafruit_MPU6050 mpu;// Accelerometer

// Parameters
const double gyroOffset = -0.0144806;

void setup() {
  Serial.begin(115200);
  Serial.println("Initializing...");
  
  // Init
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  // set accelerometer range to +-8G
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  // set gyro range to +- 500 deg/s
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  // set filter bandwidth to 21 Hz
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println("---Startup Complete---");
}

int cnt = 0;

void loop() {
  // Update
//  getData();
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  Serial.println(g.gyro.z, 8);
  cnt++;
  if (cnt >= 50000) exit(0);
}
