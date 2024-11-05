#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>

#define WIFI_SSID "wifi_name"  // wi-fi network name
#define WIFI_PASSWORD "123456" // password

// Timer variables
unsigned long lastGyroTime = 0;
unsigned long lastAccTime = 0;
unsigned long lastTempTime = 0;
unsigned long gyroDelay = 10;
unsigned long temperatureDelay = 1000;
unsigned long accelerometerDelay = 200;

Adafruit_MPU6050 mpu; // create a sensor object
sensors_event_t a, g, temp;

float gyroX = 0, gyroY = 0, gyroZ = 0;
float accX = 0, accY = 0, accZ = 0;
float temperature = 0;

// To get the sensor offset,
//     go to File > Examples > Adafruit MPU6050 >
//         basic_readings.With the sensor in a static position,
//     check the gyroscope X, Y, and Z values.Then,
//     add those values to the gyroXerror,
//     gyroYerror and gyroZerror variables.
//     // Gyroscope sensor offset on all axis
    float gyroXerror = 0.07;
float gyroYerror = 0.03;
float gyroZerror = 0.01;

void MPU_initialize() {
  Serial.println("MPU6050 test!");

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  // setup motion detection
  mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
  mpu.setMotionDetectionThreshold(1);
  mpu.setMotionDetectionDuration(20);
  mpu.setInterruptPinLatch(
      true); // Keep it latched.  Will turn off when reinitialized.
  mpu.setInterruptPinPolarity(true);
  mpu.setMotionInterrupt(true);

  delay(100);
}

void getGyroReadings() {
  mpu.getEvent(&a, &g, &temp);

  float gyroX_temp = g.gyro.x;
  if (abs(gyroX_temp) > gyroXerror) {
    gyroX += gyroX_temp / 50.00;
  }

  float gyroY_temp = g.gyro.y;
  if (abs(gyroY_temp) > gyroYerror) {
    gyroY += gyroY_temp / 70.00;
  }

  float gyroZ_temp = g.gyro.z;
  if (abs(gyroZ_temp) > gyroZerror) {
    gyroZ += gyroZ_temp / 90.00;
  }
}

void getAccReadings() {
  mpu.getEvent(&a, &g, &temp);
  accX = a.acceleration.x;
  accY = a.acceleration.y;
  accZ = a.acceleration.z;
}

void getTemperature() {
  mpu.getEvent(&a, &g, &temp);
  temperature = temp.temperature;
}

void sendDataSerial() {
  if (mpu.getMotionInterruptStatus()) {
    mpu.getEvent(&a, &g, &temp);

    Serial.print("AccelX:");
    Serial.print(a.acceleration.x);
    Serial.print(",");
    Serial.print("AccelY:");
    Serial.print(a.acceleration.y);
    Serial.print(",");
    Serial.print("AccelZ:");
    Serial.print(a.acceleration.z);
    Serial.print(", ");
    Serial.print("GyroX:");
    Serial.print(g.gyro.x);
    Serial.print(",");
    Serial.print("GyroY:");
    Serial.print(g.gyro.y);
    Serial.print(",");
    Serial.print("GyroZ:");
    Serial.print(g.gyro.z);
    Serial.print(", ");
    Serial.print("Temperature:");
    Serial.print(temp.temperature);
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);

  // Initialize I2C
  Wire.begin();

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  MPU_initialize();
  Serial.println("Starting");
}

bool isConnected = false;

void loop() {
  if (WiFi.status() == WL_CONNECTED && !isConnected) {
    Serial.println("Connected");
    digitalWrite(LED_BUILTIN, HIGH);
    isConnected = true;
  }
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println(".");
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(1000);
    isConnected = false;
  }

  unsigned long currentTime = millis();

  if (currentTime - lastGyroTime >= gyroDelay) {
    lastGyroTime = currentTime;
    getGyroReadings();
  }

  if (currentTime - lastAccTime >= accelerometerDelay) {
    lastAccTime = currentTime;
    getAccReadings();
  }

  if (currentTime - lastTempTime >= temperatureDelay) {
    lastTempTime = currentTime;
    getTemperature();
  }

  sendDataSerial();
}