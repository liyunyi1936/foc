
#include <Arduino.h>
#include <Wire.h>
#include <Kalman.h> // Source: https://github.com/TKJElectronics/KalmanFilter
#define gyroZ_OFF -0.22

//#define stable_angle 178.2
//#define stable_angle 58.8

//#define stable_angle 301.75
#define stable_angle 60.0
Kalman kalmanZ;

/* IMU Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;

double gyroZangle; // Angle calculate using the gyro only
double compAngleZ; // Calculated angle using a complementary filter
double kalAngleZ;  // Calculated angle using a Kalman filter

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data

/********************************************************************************/
#include <SimpleFOC.h>
//#include "common/foc_utils.h"
#define swing_up_voltage 1.5 //V
#define balance_voltage 10   //V
#define min_voltage 9.5      //V
/*
  #define PID_P 0  //
  #define PID_I 0   //
  #define PID_D 1   //
  #define LQR_K1 1  //
  #define LQR_K2 0  //
  #define LQR_K3 0.0  //
*/
float PID_P = 1; //
float PID_I = 0; //
float PID_D = 0; //
/*
//稳定器参数
float LQR_K1 = 50;  //摇摆到平衡
float LQR_K2 = 2;   //
float LQR_K3 = 0.30; //

float LQR_K1_1 = 50;   //平衡态
float LQR_K2_1 = 2;   //
float LQR_K3_1 = 0.15; //
*/
//倒立摆参数
float LQR_K1 = 200;  //摇摆到平衡
float LQR_K2 = 40;   //
float LQR_K3 = 0.30; //

float LQR_K1_1 = 200;   //平衡态
float LQR_K2_1 = 15;   //
float LQR_K3_1 = 0.15; //

/*
float LQR_K1 = 200;   //
float LQR_K2 = 40;   //
float LQR_K3 = 0.30; //
*/
/*单角度稳定

float LQR_K1 = 80;  //平衡完成
float LQR_K2 = 15;  //
float LQR_K3 = 0.15;  //
*/
float OFFSET = 0;
bool stable = 0, battery_low = 0;
uint32_t last_unstable_time;
//output=LQR_K1*PID+LQR_K2*p_vel + LQR_K3 * m_vel

MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
PIDController angle_pid = PIDController(PID_P, PID_I, PID_D, balance_voltage * 0.7, 20000);
LowPassFilter lpf_throttle{0.00};
// BLDC motor init
BLDCMotor motor = BLDCMotor(5);
// driver instance
BLDCDriver3PWM driver = BLDCDriver3PWM(9, 10, 11, 8, 3);
double rotationshift(double origin, double theta, double shift, bool y);
double acc2rotation(double x, double y);
float controllerLQR(float p_angle, float p_vel, float m_vel);
float constrainAngle(float x);

// instantiate the commander
Commander command = Commander(Serial);
//void onp(char *cmd) { command.scalar(&PID_P, cmd); }
//void oni(char *cmd) { command.scalar(&PID_I, cmd); }
//void ond(char *cmd) { command.scalar(&PID_D, cmd); }
void onj(char *cmd) { command.scalar(&LQR_K1, cmd); }
void onk(char *cmd) { command.scalar(&LQR_K2, cmd); }
void onl(char *cmd) { command.scalar(&LQR_K3, cmd); }

/********************************************************************************/
void setup()
{
  Serial.begin(115200);
  Wire.begin();

  Wire.setClock(400000UL); // Set I2C frequency to 400kHz
  Serial.println(((analogRead(A3) / 41.5)));
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

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
  // Eq. 25 and 26

  double pitch = acc2rotation(accX, accY);

  kalmanZ.setAngle(pitch);
  gyroZangle = pitch;

  timer = micros();

  pinMode(4, OUTPUT);
  digitalWrite(4, 1);
  sensor.init(&Wire);
  motor.linkSensor(&sensor);
  // driver
  driver.voltage_power_supply = 12;
  driver.init();

  // link the driver and the motor
  motor.linkDriver(&driver);

  // aligning voltage
  motor.voltage_sensor_align = 3;

  // choose FOC modulation (optional)
  //motor.foc_modulation = FOCModulationType::SinePWM;
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

  // set control loop type to be used
  motor.controller = MotionControlType::torque;
  //motor.controller = TorqueControlType::voltage;

  motor.voltage_limit = balance_voltage;

  motor.useMonitoring(Serial);
  // initialize motor
  motor.init();
  // align encoder and start FOC
  //motor.initFOC(4.5,Direction::CW);
  //motor.initFOC(4.05, Direction::CCW);
  motor.initFOC();
  //motor.initFOC(2.6492,Direction::CW);
  //command.add('p', onp, "p");
  //command.add('i', oni, "i");
  //command.add('d', ond, "d");
  command.add('j', onj, "newj:");
  command.add('k', onk, "newk:");
  command.add('l', onl, "newl:");

  digitalWrite(4, 0);
}
long loop_count = 0;
float target_voltage;
void loop()
{

  motor.loopFOC();

  if (loop_count++ == 10)
  {
    /* Update all the values */
    while (i2cRead(0x3B, i2cData, 14))
      ;
    accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
    accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
    accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
    tempRaw = (int16_t)((i2cData[6] << 8) | i2cData[7]);
    gyroX = (int16_t)((i2cData[8] << 8) | i2cData[9]);
    gyroY = (int16_t)((i2cData[10] << 8) | i2cData[11]);
    gyroZ = (int16_t)((i2cData[12] << 8) | i2cData[13]);
    ;

    double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
    timer = micros();

    // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
    // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
    // It is then converted from radians to degrees
    // Eq. 25 and 26

    double pitch = acc2rotation(accX, accY);
    double gyroZrate = gyroZ / 131.0; // Convert to deg/s

    kalAngleZ = kalmanZ.getAngle(pitch, gyroZrate + gyroZ_OFF, dt);

    gyroZangle += (gyroZrate + gyroZ_OFF) * dt;
    //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
    //gyroYangle += kalmanY.getRate() * dt;

    compAngleZ = 0.93 * (compAngleZ + (gyroZrate + gyroZ_OFF) * dt) + 0.07 * pitch;

    // Reset the gyro angle when it has drifted too much
    if (gyroZangle < -180 || gyroZangle > 180)
      gyroZangle = kalAngleZ;

      /* Print Data */
#if 0 // Set to 1 to activate
  Serial.print(accX); Serial.print("\t");
  Serial.print(accY); Serial.print("\t");
  Serial.print(accZ); Serial.print("\t");

  Serial.print(gyroX); Serial.print("\t");
  Serial.print(gyroY); Serial.print("\t");
  Serial.print(gyroZ); Serial.print("\t");

  Serial.print("\t");
#endif
#if 0
    Serial.print(pitch);
    Serial.print("\t");
    Serial.print(gyroZangle);
    Serial.print("\t");
    Serial.print(compAngleZ);
    Serial.print("\t");
    Serial.print(kalAngleZ);
    Serial.print("\t");

    //Serial.print("\r\n");
#endif
    // calculate the pendulum angle
    //LQR_K1 = analogRead(A3) / 10.0;
    digitalWrite(3, 1);
    //float pendulum_angle = constrainAngle(rotationshift(kalAngleZ * 3, 180.0, -180.0+OFFSET, 0.0) / 57.29578 + M_PI);
    //float pendulum_angle = constrainAngle((kalAngleZ - stable_angle ) / 57.29578);
    float pendulum_angle = constrainAngle((fmod(kalAngleZ * 3, 360.0) / 3.0 - stable_angle) / 57.29578);
    if (abs(pendulum_angle) < 0.6) // if angle small enough stabilize 0.5~30°,1.5~90°
    {
      //target_voltage = controllerLQR(pendulum_angle, g.gyro.z, motor.shaftVelocity());

      target_voltage = controllerLQR(angle_pid(pendulum_angle), gyroZrate / 57.29578, motor.shaftVelocity());

      //digitalWrite(4, 1);
    }
    else // else do swing-up
    {    // sets 1.5V to the motor in order to swing up
      target_voltage = -_sign(gyroZrate) * swing_up_voltage;
      digitalWrite(4, 0);
    }

    // set the target voltage to the motor
    if (accZ < -13000 && ((accX * accX + accY * accY) > (14000 * 14000)))
    {
      motor.move(0);
    }
    else
    {
      motor.move(lpf_throttle(target_voltage));
    }
    command.run();
    // restart the counter
    loop_count = 0;
    //Serial.print("kangle:");
    driver.voltage_power_supply = analogRead(A3) / 41.5;
    //Serial.println(driver.voltage_power_supply);
    if ((analogRead(A3) / 41.5) < min_voltage && !battery_low)
    {
      battery_low = 1;
      Serial.println("battery_low!!");
      while (battery_low)
      {
        motor.loopFOC();
        motor.move(0);
        if (millis() % 500 < 250)
          digitalWrite(4, 1);
        else
          digitalWrite(4, 0);
      }
    }
    //Serial.print(",fangle:");
    //Serial.print(constrainAngle(rotationshift(kalAngleZ * 3, 180.0, -180.0+OFFSET, 0.0) / 57.29578 + M_PI));

    //Serial.println(fmod(kalAngleZ * 3, 360.0) / 3.0);
    //Serial.print(",pid:");
    //Serial.println(accX);
    //Serial.print(angle_pid(pendulum_angle));
    //Serial.print(",voltage:");
    //Serial.print(target_voltage);
    //Serial.print(",lpf_throttle:");
    //Serial.println(lpf_throttle(target_voltage));
    //Serial.print(",E_gle:");
    //Serial.print(sensor.getAngle());
    //Serial.print(",vel:");
    //Serial.println(sensor.getVelocity());
  }
  
    
    
  
}

// function constraining the angle in between -pi and pi, in degrees -180 and 180
float constrainAngle(float x)
{
  x = fmod(x + M_PI, _2PI);
  if (x < 0)
    x += _2PI;
  return x - M_PI;
}
// LQR stabilization controller functions
// calculating the voltage that needs to be set to the motor in order to stabilize the pendulum
float controllerLQR(float p_angle, float p_vel, float m_vel)
{
  // if angle controllable
  // calculate the control law
  // LQR controller u = k*x
  //  - k = [40, 7, 0.3]
  //  - k = [13.3, 21, 0.3]
  //  - x = [pendulum angle, pendulum velocity, motor velocity]'
  if (abs(p_angle) > 0.05)
  {
    last_unstable_time = millis();
    stable = 0;
    digitalWrite(4, 0);
  }
  if ((millis() - last_unstable_time) > 1000)
  {
    stable = 1;
    digitalWrite(4, 1);
  }

  //Serial.println(stable);
  float u;
  if (!stable)
  {
    u = LQR_K1 * p_angle + LQR_K2 * p_vel + LQR_K3 * m_vel;
  }
  else
  {
    //u = LQR_K1 * p_angle + LQR_K2 * p_vel + LQR_K3 * m_vel;
    u = LQR_K1_1 * p_angle + LQR_K2_1 * p_vel + LQR_K3_1 * m_vel;
  }

  // limit the voltage set to the motor
  if (abs(u) > motor.voltage_limit * 0.7)
    u = _sign(u) * motor.voltage_limit * 0.7;

  return u;
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

/* mpu6050加速度转换为角度
       rotationshift(original angle,+θ,shiftθ,0 is normal,1 is reverse)
       rotationshift(0,30)=30
       rotationshift(20,30)=50
       rotationshift(0,30,1)=330
       rotationshift(20,30,1)=310
       rotationshift(0,0,-180,0)=-180
*/
double rotationshift(double origin, double theta, double shift = 0, bool y = false)
{
  static float origin_old;
  if (abs(origin - origin_old) > 0.1)
    origin_old += _sign(origin - origin_old) * 0.01;
  else
    origin_old = origin;

  if (y == 0)
  {
    if (origin + theta > 360)
      return origin + theta - 360 + shift;
    else
    {
      return origin + theta + shift;
    }
  }
  else
  {

    if (-(origin + theta) + 360 < 0)
      return -(origin + theta) + 360 + 360 + shift;
    else
    {
      return -(origin + theta) + 360 + shift;
    }
  }
}
