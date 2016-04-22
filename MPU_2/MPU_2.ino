// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#include "Wire.h"
// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"
// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;

// 加速度
int16_t ax, ay, az;
//陀螺仪
int16_t gx, gy, gz;
 
#define LED_PIN 13
bool blinkState = false;
 
void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();
 
  // initialize serial communication
  // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
  // it's really up to you depending on your project)
  Serial.begin(38400);
 
  // initialize device
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();
 
  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
 
  // configure Arduino LED for
  pinMode(LED_PIN, OUTPUT);
}
 
void loop() {
  // read raw accel/gyro measurements from device
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
 
  // these methods (and a few others) are also available
  //accelgyro.getAcceleration(&ax, &ay, &az);
  //accelgyro.getRotation(&gx, &gy, &gz);
 
  // display tab-separated accel/gyro x/y/z values
    Serial.print("a/g:\t");
    //单位都有默认的倍率，所以除上一些常数，当然可以修改倍率，但增大倍率精度会降低
    Serial.print(ax/16384); Serial.print("\t");
    Serial.print(ay/16384); Serial.print("\t");
    Serial.print(az/16384); Serial.print("\t");
    Serial.print(gx/131); Serial.print("\t");
    Serial.print(gy/131); Serial.print("\t");
    Serial.println(gz/131);
  // blink LED to indicate activity
  blinkState = !blinkState;
  digitalWrite(LED_PIN, blinkState);
}

