#include "Command.h"
#include <Wire.h>
#include <WiFi.h>
#include <AsyncUDP.h> //引用以使用异步UDP
#include "Kalman.h" // Source: https://github.com/TKJElectronics/KalmanFilter
#include "EEPROM.h"
Kalman kalmanZ;
#define gyroZ_OFF 0.9
/* ----IMU Data---- */

double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;
bool stable = 0;
uint32_t last_unstable_time;

double gyroZangle; // Angle calculate using the gyro only
double compAngleZ; // Calculated angle using a complementary filter
double kalAngleZ;  // Calculated angle using a Kalman filter

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data

void setup() {
  Serial.begin(115200);
  // kalman mpu6050 init
  Wire.begin(19, 18, 400000); // Set I2C frequency to 400kHz
  i2cData[0] = 7;    // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cData, 4, false))
    ; // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true))
    ; // PLL with X axis gyroscope reference and disable sleep mode

  while (i2cRead(0x75, i2cData, 1))
    ;
  if (i2cData[0] != 0x68)
  { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    while (1)
      ;
  }

  delay(100); // Wait for sensor to stabilize

  /* Set kalman and gyro starting angle */
  while (i2cRead(0x3B, i2cData, 6))
    ;
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
  double pitch = acc2rotation(accX, accY);

  kalmanZ.setAngle(pitch);
  gyroZangle = pitch;

  timer = micros();
  Serial.println("kalman mpu6050 init");
}

void loop() {
  while (i2cRead(0x3B, i2cData, 14));
    accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
    accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
    accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
    tempRaw = (int16_t)((i2cData[6] << 8) | i2cData[7]);
    gyroX = (int16_t)((i2cData[8] << 8) | i2cData[9]);
    gyroY = (int16_t)((i2cData[10] << 8) | i2cData[11]);
    gyroZ = (int16_t)((i2cData[12] << 8) | i2cData[13]);

    double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
    timer = micros();

    double pitch = acc2rotation(accX, accY);
    double gyroZrate = gyroZ / 131.0; // Convert to deg/s

    kalAngleZ = kalmanZ.getAngle(pitch, gyroZrate + gyroZ_OFF, dt);
    gyroZangle += (gyroZrate + gyroZ_OFF) * dt;
    compAngleZ = 0.93 * (compAngleZ + (gyroZrate + gyroZ_OFF) * dt) + 0.07 * pitch;

//    // Reset the gyro angle when it has drifted too much
//    if (gyroZangle < -180 || gyroZangle > 180)
//      gyroZangle = kalAngleZ;

   Serial.print(pitch); Serial.print("\t");
//   Serial.print(gyroZrate); Serial.print("\t");
//   Serial.print(kalAngleZ); Serial.print("\t");
//   Serial.print(gyroZangle); Serial.print("\t");
   Serial.print(compAngleZ); Serial.print("\t");
   Serial.print("\r\n");
}
/* mpu6050加速度转换为角度
            acc2rotation(ax, ay)
            acc2rotation(az, ay) */
double acc2rotation(double x, double y)
{
  if (y < 0)
  {
    return atan(x / y) / 1.570796 * 90 + 180;
  }
  else if (x < 0)
  {
    return (atan(x / y) / 1.570796 * 90 + 360);
  }
  else
  {
    return (atan(x / y) / 1.570796 * 90);
  }
}

// function constraining the angle in between -60~60
float constrainAngle(float x)
{
  float a = 0;
  if (x < 0)
  {
    a = 120 + x;
    if (a < abs(x))
      return a;
  }
  return x;
}
