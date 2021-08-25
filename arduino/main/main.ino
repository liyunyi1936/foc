  /**
Deng's FOC 闭环速度控制例程 测试库：SimpleFOC 2.1.1 测试硬件：灯哥开源FOC V1.0
在串口窗口中输入：T+速度，就可以使得两个电机闭环转动
比如让两个电机都以 10rad/s 的速度转动，则输入：T10
在使用自己的电机时，请一定记得修改默认极对数，即 BLDCMotor(7) 中的值，设置为自己的极对数数字
程序默认设置的供电电压为 16.8V,用其他电压供电请记得修改 voltage_power_supply , voltage_limit 变量中的值
默认PID针对的电机是 GB6010 ，使用自己的电机需要修改PID参数，才能实现更好效果
 */
#include <SimpleFOC.h>
#include <WiFi.h>
#include <AsyncUDP.h> //引用以使用异步UDP
#include <Kalman.h> // Source: https://github.com/TKJElectronics/KalmanFilter
Kalman kalmanZ;
#define gyroZ_OFF -0.72
#define swing_up_voltage 10 //V
#define balance_voltage 10   //V
/* ----IMU Data---- */
float PID_P = 1; //
float PID_I = 0; //
float PID_D = 0; //
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
/* ----FOC Data---- */

// driver instance
double acc2rotation(double x, double y);
float constrainAngle(float x);
const char *ssid = "esp32";
const char *password = "12345678";


AsyncUDP udp;                     //创建UDP对象
unsigned int localUdpPort = 2333; //本地端口号
unsigned int broadcastPort = localUdpPort;

MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

TwoWire I2Ctwo = TwoWire(1);
PIDController angle_pid = PIDController(PID_P, PID_I, PID_D, balance_voltage * 0.7, 20000);
LowPassFilter lpf_throttle{0.00};
//倒立摆参数
float LQR_K1 = 500;  //摇摆到平衡
float LQR_K2 = 40;   //
float LQR_K3 = 0.30; //

float LQR_K1_1 = 200;   //平衡态
float LQR_K2_1 = 15;   //
float LQR_K3_1 = 0.15; //

//电机参数
BLDCMotor motor = BLDCMotor(5);
BLDCDriver3PWM driver = BLDCDriver3PWM(32, 33, 25, 22);


//命令设置
double target_velocity = 0;
double target_angle = 31;
double target_voltage = 0;
void onPacketCallBack(AsyncUDPPacket packet)
{
  target_velocity = atoi((char*)(packet.data()));
  Serial.print("数据内容: ");
  Serial.println(target_velocity);
//  packet.print("reply data");
}
// instantiate the commander
Commander command = Commander(Serial);
void onp(char* cmd) { command.scalar(&PID_P, cmd); }
void oni(char* cmd) { command.scalar(&PID_I, cmd); }
void ond(char* cmd) { command.scalar(&PID_D, cmd); }
void setup() {
   Serial.begin(115200);


    // kalman mpu6050 init
  Wire.begin(19, 18,400000);// Set I2C frequency to 400kHz
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

    //wifi初始化
  WiFi.mode(WIFI_AP);
   while(!WiFi.softAP(ssid, password)){}; //启动AP
    Serial.println("AP启动成功");
  while (!udp.listen(localUdpPort)) //等待udp监听设置成功
  {
  }
  udp.onPacket(onPacketCallBack); //注册收到数据包事件
  
  I2Ctwo.begin(23, 5, 400000);   //SDA,SCL
  sensor.init(&I2Ctwo);

  //连接motor对象与传感器对象
  motor.linkSensor(&sensor);

  //供电电压设置 [V]
  driver.voltage_power_supply = 12;
  driver.init();

  //连接电机和driver对象
  motor.linkDriver(&driver);

  //FOC模型选择
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

  //运动控制模式设置
  motor.controller = MotionControlType::torque;

//  //速度PI环设置
//  motor.PID_velocity.P = 2;
//  motor.PID_velocity.I = 20;

  //最大电机限制电机
  motor.voltage_limit = 12;

  //速度低通滤波时间常数
  motor.LPF_velocity.Tf = 0.01;

  //设置最大速度限制
  motor.velocity_limit = 40;

  motor.useMonitoring(Serial);
  
  //初始化电机
  motor.init();

  //初始化 FOC
  motor.initFOC();

      command.add('P', onp, "newKP");
    command.add('I', oni, "newKI");
    command.add('D', ond, "newKD");
  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target velocity using serial terminal:"));


}
char s[255];
int t_v;
int lim_v = 30;
long loop_count = 0;
void loop() {
    motor.loopFOC();
  if (loop_count++ == 10)
  {
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

    // Reset the gyro angle when it has drifted too much
    if (gyroZangle < -180 || gyroZangle > 180)
      gyroZangle = kalAngleZ;
      
  sprintf(s, "%.2f",kalAngleZ); //将100转为16进制表示的字符串
  float pendulum_angle = constrainAngle((fmod(kalAngleZ * 3, 360.0) / 3.0 - target_angle) / 57.29578);
   if (abs(pendulum_angle) < 0.6) // if angle small enough stabilize 0.5~30°,1.5~90°
   {
     target_voltage = controllerLQR(angle_pid(pendulum_angle), gyroZrate / 57.29578, motor.shaftVelocity());
   }
    else // else do swing-up
    {    // sets 1.5V to the motor in order to swing up
      target_voltage = -_sign(gyroZrate) * swing_up_voltage;
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
  Serial.print(motor.shaft_velocity);Serial.print("\t");
  Serial.print(target_voltage);Serial.print("\t");
  Serial.print(target_angle);Serial.print("\t");
  Serial.print(pendulum_angle+target_angle);Serial.print("\t");
  Serial.print(gyroZrate);Serial.print("\t");
  Serial.print("\r\n");
  motor.move(lpf_throttle(target_voltage));
//  motor.move(target_velocity);
  //可以使用该方法广播信息
  IPAddress broadcastAddr((~(uint32_t)WiFi.subnetMask())|((uint32_t)WiFi.localIP())); //计算广播地址
  udp.writeTo((const unsigned char*)s, strlen(s), broadcastAddr, localUdpPort); //广播数据
  loop_count = 0;
  }
  command.run();
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
  }
  if ((millis() - last_unstable_time) > 1000)
  {
    stable = 1;
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
