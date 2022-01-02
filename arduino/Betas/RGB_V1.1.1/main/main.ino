/**
   自平衡莱洛三角形 RGB版  HW:Ver 1.5  FW:Ver 1.1.1
   立创EDA https://oshwhub.com/muyan2020/zi-ping-heng-di-lai-luo-san-jiao_10-10-ban-ben_copy
   RGB版本程序 https://gitee.com/muyan3000/RGBFOC 基于45°(https://gitee.com/coll45/foc/)程序修改
   arduino开发环境-灯哥开源FOChttps://gitee.com/ream_d/Deng-s-foc-controller，并安装Kalman。

  FOC引脚32, 33, 25, 22    22为enable
  AS5600霍尔传感器 SDA-23 SCL-5  MPU6050六轴传感器 SDA-19 SCL-18
  本程序有两种平衡方式， FLAG_V为1时使用电压控制，为0时候速度控制。电压控制时LQR参数使用K1和K2，速度控制时LQR参数使用K3和K4
  在wifi上位机窗口中输入：TA+角度，就可以修改平衡角度
  比如让平衡角度为90度，则输入：TA90，并且会存入eeprom的位置0中 注：wifi发送命令不能过快，因为每次都会保存进eeprom
  在使用自己的电机时，请一定记得修改默认极对数，即 BLDCMotor(5) 中的值，设置为自己的极对数数字，磁铁数量/2
  程序默认设置的供电电压为 12V,用其他电压供电请记得修改 voltage_power_supply , voltage_limit 变量中的值
  默认PID针对的电机是 GB2204 ，使用自己的电机需要修改PID参数，才能实现更好效果
*/
#include <SimpleFOC.h>
#include "Command.h"
#include <WiFi.h>
#include <AsyncUDP.h> //引用以使用异步UDP
#include "Kalman.h" // Source: https://github.com/TKJElectronics/KalmanFilter
#include "EEPROM.h"

#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include <WebServer.h>
#include <FS.h>
#include "SPIFFS.h"
#include <time.h>
#define timezone 8

#include <FastLED.h>
#define DATA_PIN    16    //RGB pin
#define LED_TYPE    WS2812B
#define COLOR_ORDER GRB
#define NUM_LEDS    21  //LED数量
int rgb_brightness = 25;  //初始化亮度
CRGB leds[NUM_LEDS];

unsigned long TenthSecondsSinceStart = 0;
void TenthSecondsSinceStartTask();
void OnTenthSecond();
void OnSecond();
void StartWebServer();

#define ACTIVE_PIN 4  //状态灯
#define BAT_VOLTAGE_SENSE_PIN 34  //电池电压检测ADC，如果旧版PCB无电压检测电路，则注释掉此行
const double R1_VOLTAGE = 68000; //68K
const double R2_VOLTAGE = 10000; //10K
const double min_voltage  = 9.5;  //电池检测最低电压
double bat_voltage;

const int threshold_top = 20;   //触摸顶部阈值
const int threshold_bottom = 1;   //触摸底部阈值，越接近数值越小
const int threshold_count = 4;   //触摸计数器有效值，通常会有意外的自动触发

int touchread[4] = {100, 100, 100, 100}; //初始化触摸读取值为100，无触摸
int touchDetected[4] = {}; //通过touchdetected持续计数判断是否按键，防止无触碰触发

bool touch_touched[4] = {};   //单击判断
int touch_touched_times[4] = {};  //单击次数，单击切换模式，双击
int touch_touching_time[4] = {}; //持续触摸秒数，用于判断长按事件，长按关闭，长按开启，开启状态长按调光，
bool touch_STATE[4] = {1, 1, 1, 1}; // 定义按键触发对象状态变量初始值为true默认开启

const char* username = "admin";     //web用户名
const char* userpassword = "12345678"; //web用户密码
const char* ServerName = "ESP32-LELO-RGB";
char DateTimeStr[20]  = "1970-01-01 00:00:00";
char Debug_Log[255][255];
uint32_t loop_time_begin = millis();
int debug_times;
bool log_control = 0, debug_log_control = 0;

WebServer ESP32Server(80);

Kalman kalmanZ;
#define gyroZ_OFF -0.19
/* ----IMU Data---- */

double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;
bool stable = 0 , battery_low = 0;
uint32_t last_unstable_time;
uint32_t last_stable_time;

double gyroZangle; // Angle calculate using the gyro only
double compAngleZ; // Calculated angle using a complementary filter
double kalAngleZ;  // Calculated angle using a Kalman filter
float pendulum_angle;

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data
/* ----FOC Data---- */

// driver instance
double acc2rotation(double x, double y);
float constrainAngle(float x);
const char *ssid = "esp32";
const char *password = "12345678";

bool wifi_flag = 0;
AsyncUDP udp;                     //创建UDP对象
unsigned int localUdpPort = 2333; //本地端口号
void wifi_print(char * s, double num);

MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
TwoWire I2Ctwo = TwoWire(1);
LowPassFilter lpf_throttle{0.00};

//倒立摇摆参数
//3和4是速度控制稳定前和后
float LQR_K3_1 = 10;   //速度控制摇摆到平衡
float LQR_K3_2 = 1.7;   //
float LQR_K3_3 = 1.75; //

float LQR_K4_1 = 2.4;   //速度控制平衡态
float LQR_K4_2 = 1.5;   //
float LQR_K4_3 = 1.42; //

//电机参数
BLDCMotor motor = BLDCMotor(5);
BLDCDriver3PWM driver = BLDCDriver3PWM(32, 33, 25, 22);
float target_velocity = 0;	//目标速度
float target_angle = 90;	//平衡角度 例如TA89.3 设置平衡角度89.3
float target_voltage = 0;	//目标电压
float swing_up_voltage = 1.8;	//摇摆电压 左右摇摆的电压，越大越快到平衡态，但是过大会翻过头
float swing_up_angle = 20;	//摇摆角度 离平衡角度还有几度时候，切换到自平衡控制
float v_i_1 = 20;	//非稳态速度环I
float v_p_1 = 0.7;	//非稳态速度环P
float v_i_2 = 10;	//稳态速度环I
float v_p_2 = 0.3;	//稳态速度环P
//命令设置
Command comm;
bool Motor_enable_flag = 0;
int test_flag = 0;
void do_TA(char* cmd) {
  comm.scalar(&target_angle, cmd);
  EEPROM.writeFloat(0, target_angle);
}
void do_SV(char* cmd) {
  comm.scalar(&swing_up_voltage, cmd);
  EEPROM.writeFloat(4, swing_up_voltage);
}
void do_SA(char* cmd) {
  comm.scalar(&swing_up_angle, cmd);
  EEPROM.writeFloat(8, swing_up_angle);
}

void do_START(char* cmd) {
  wifi_flag = !wifi_flag;
}
void do_MOTOR(char* cmd)
{
  if (Motor_enable_flag)
    motor.enable();
  else
    motor.disable();
  Motor_enable_flag = !Motor_enable_flag;
}

void do_TVQ(char* cmd)
{
  if (test_flag == 1)
    test_flag = 0;
  else
    test_flag = 1;
}
void do_TVV(char* cmd)
{
  if (test_flag == 2)
    test_flag = 0;
  else
    test_flag = 2;
}
void do_VV(char* cmd) {
  comm.scalar(&target_velocity, cmd);
}
void do_VQ(char* cmd) {
  comm.scalar(&target_voltage, cmd);
}

void do_vp1(char* cmd) {
  comm.scalar(&v_p_1, cmd);
  EEPROM.writeFloat(12, v_p_1);
}
void do_vi1(char* cmd) {
  comm.scalar(&v_i_1, cmd);
  EEPROM.writeFloat(16, v_i_1);
}
void do_vp2(char* cmd) {
  comm.scalar(&v_p_2, cmd);
  EEPROM.writeFloat(20, v_p_2);
}
void do_vi2(char* cmd) {
  comm.scalar(&v_i_2, cmd);
  EEPROM.writeFloat(24, v_i_2);
}
void do_tv(char* cmd) {
  comm.scalar(&target_velocity, cmd);
}
void do_K31(char* cmd) {
  comm.scalar(&LQR_K3_1, cmd);
}
void do_K32(char* cmd) {
  comm.scalar(&LQR_K3_2, cmd);
}
void do_K33(char* cmd) {
  comm.scalar(&LQR_K3_3, cmd);
}
void do_K41(char* cmd) {
  comm.scalar(&LQR_K4_1, cmd);
}
void do_K42(char* cmd) {
  comm.scalar(&LQR_K4_2, cmd);
}
void do_K43(char* cmd) {
  comm.scalar(&LQR_K4_3, cmd);
}

void Debug_Log_func(String debuglog, bool debug_begin = 0) {
  uint32_t loop_time_end;
  if (debug_log_control) {
    if (debug_begin) {
      loop_time_begin = millis();
      sprintf(Debug_Log[debug_times], "%s Begin time:%d", debuglog.c_str(), loop_time_begin);
    } else {
      loop_time_end = millis();
      sprintf(Debug_Log[debug_times], "%s\t%s End time:%d\tProcessed in %d ms\tFreeHeap:%d\r\n", Debug_Log[debug_times], debuglog.c_str(), loop_time_end, (loop_time_end - loop_time_begin), ESP.getFreeHeap());
      debug_times++;
    }
  }
}

void onPacketCallBack(AsyncUDPPacket packet)
{
  char* da;
  da = (char*)(packet.data());
  Serial.println(da);
  comm.run(da);
  EEPROM.commit();
  //  packet.print("reply data");
}
// instantiate the commander
void setup() {
  Debug_Log_func("Before setup");
  Debug_Log_func("setup", 1);
  Serial.begin(115200);

  pinMode(ACTIVE_PIN, OUTPUT);
  digitalWrite(ACTIVE_PIN, HIGH);
  uint32_t chipId = 0;
  for (int i = 0; i < 17; i = i + 8) {
    chipId |= ((ESP.getEfuseMac() >> (40 - i)) & 0xff) << i;
  }

  Serial.printf("ESP32 Chip model = %s Rev %d\n", ESP.getChipModel(), ESP.getChipRevision());
  Serial.printf("This chip has %d cores\n", ESP.getChipCores());
  Serial.print("Chip ID: "); Serial.println(chipId);

  //获取IDF版本
  printf("SDK version:%s\n", esp_get_idf_version());
  //获取芯片内存
  Serial.print("Max Free Heap: ");
  Serial.println(ESP.getMaxAllocHeap());
  //获取芯片可用内存
  printf("esp_get_free_heap_size : %d  \n", esp_get_free_heap_size());
  printf("getFreeHeap : %d  \n", ESP.getFreeHeap());
  //获取从未使用过的最小内存
  printf("esp_get_minimum_free_heap_size : %d  \n", esp_get_minimum_free_heap_size());

  if (!EEPROM.begin(1000)) {
    Serial.println("Failed to initialise EEPROM");
    Serial.println("Restarting...");
    delay(1000);
    ESP.restart();
  }
  // eeprom 读取
  int k, j;
  j = 0;
  for (k = 0; k <= 24; k = k + 4)
  {
    float nan = EEPROM.readFloat(k);
    if (isnan(nan))
    {
      j = 1;
      Serial.println("frist write");
      EEPROM.writeFloat(0, target_angle);       delay(10); EEPROM.commit();
      EEPROM.writeFloat(4, swing_up_voltage);      delay(10); EEPROM.commit();
      EEPROM.writeFloat(8, swing_up_angle);      delay(10); EEPROM.commit();
      EEPROM.writeFloat(12, v_p_1);      delay(10); EEPROM.commit();
      EEPROM.writeFloat(16, v_i_1);      delay(10); EEPROM.commit();
      EEPROM.writeFloat(20, v_p_2);      delay(10); EEPROM.commit();
      EEPROM.writeFloat(24, v_i_2);       delay(10); EEPROM.commit();
    }
  }
  if (j == 0)
  {
    target_angle = EEPROM.readFloat(0);
    swing_up_voltage = EEPROM.readFloat(4);
    swing_up_angle = EEPROM.readFloat(8);
    v_p_1 = EEPROM.readFloat(12);
    v_i_1 = EEPROM.readFloat(16);
    v_p_2 = EEPROM.readFloat(20);
    v_i_2 = EEPROM.readFloat(24);
    motor.PID_velocity.P = v_p_1;
    motor.PID_velocity.I = v_i_1;
  }

  //命令设置
  comm.add("TA", do_TA);
  comm.add("START", do_START);
  comm.add("MOTOR", do_MOTOR);
  comm.add("SV", do_SV);
  comm.add("SA", do_SA);
  comm.add("TVQ", do_TVQ);
  comm.add("TVV", do_TVV);
  comm.add("VV", do_VV);
  comm.add("VQ", do_VQ);
  //速度环参数
  comm.add("VP1", do_vp1);
  comm.add("VI1", do_vi1);
  comm.add("VP2", do_vp2);
  comm.add("VI2", do_vi2);
  comm.add("TV", do_tv);
  comm.add("K31", do_K31);
  comm.add("K32", do_K32);
  comm.add("K33", do_K33);
  comm.add("K41", do_K41);
  comm.add("K42", do_K42);
  comm.add("K43", do_K43);

  // tell FastLED about the LED strip configuration
  FastLED.addLeds<LED_TYPE, DATA_PIN, COLOR_ORDER>(leds, NUM_LEDS)
  .setCorrection(TypicalLEDStrip)
  .setDither(rgb_brightness < 255);
  // set master brightness control
  FastLED.setBrightness(rgb_brightness);

  CRGB c_rgb[5];
  c_rgb[0]  = CRGB::White;
  c_rgb[2]  = CRGB::Red;
  c_rgb[1]  = CRGB::Green;
  c_rgb[3]  = CRGB::Blue;
  c_rgb[4]  = CRGB::Purple;

  for ( int j = 0; j < 5; j++) {
    for ( int i = 0; i < NUM_LEDS; i++) {
      leds[i] = c_rgb[j];
      FastLED.show();
      delay(15);
    }
    delay(500);
  }

  //wifi初始化
  WiFi.mode(WIFI_AP);
  while (!WiFi.softAP(ssid, password)) {}; //启动AP
  Serial.println("AP启动成功");
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.softAPIP());

  while (!udp.listen(localUdpPort)) //等待udp监听设置成功
  {
  }
  udp.onPacket(onPacketCallBack); //注册收到数据包事件

  Serial.print("\nMax Free Heap: ");
  Serial.println(ESP.getMaxAllocHeap());
  Serial.println("");

  ArduinoOTA.setHostname("ESP32-Reuleaux");
  //以下是启动OTA，可以通过WiFi刷新固件
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_SPIFFS
      type = "filesystem";
    }

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });
  ArduinoOTA.begin();

  // kalman mpu6050 init
  Wire.begin(19, 18, 400000); // Set I2C frequency to 400kHz
  i2cData[0] = 7;    // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode
  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68)
  { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    while (1);
  }

  delay(100); // Wait for sensor to stabilize

  /* Set kalman and gyro starting angle */
  while (i2cRead(0x3B, i2cData, 6));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
  double pitch = acc2rotation(accX, accY);
  kalmanZ.setAngle(pitch); // Set starting angle
  gyroZangle = pitch;
  timer = micros();
  Serial.println("kalman mpu6050 init");

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
  motor.controller = MotionControlType::velocity;

  //速度PI环设置
  motor.PID_velocity.P = v_p_1;
  motor.PID_velocity.I = v_i_1;

  //最大电机限制电压
  motor.voltage_limit = 6;

  //速度低通滤波时间常数
  motor.LPF_velocity.Tf = 0.01f;

  // angle P controller
  motor.P_angle.P = 20;

  //设置最大速度限制
  motor.velocity_limit = 40;

  motor.useMonitoring(Serial);

  //初始化电机
  motor.init();

  //初始化 FOC
  motor.initFOC();

  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target velocity using serial terminal:"));


  // 启动闪存文件系统
  if (SPIFFS.begin())
  {
    Serial.println("SPIFFS Started.");
  }
  else
  {
    Serial.println("SPIFFS Failed to Start.");
  }

  StartWebServer();

  Serial.println("System is ready");
  Serial.println("-----------------------------------------------");

  Debug_Log_func("setup");
}

char buf[255];
long loop_count = 0;
double last_pitch;
void loop() {
  Debug_Log_func("loop", 1);
  ArduinoOTA.handle();
  motor.loopFOC();

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
  //double pitch2 = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
  double gyroZrate = gyroZ / 131.0; // Convert to deg/s
  if (abs(pitch - last_pitch) > 100)
    kalmanZ.setAngle(pitch);

  kalAngleZ = kalmanZ.getAngle(pitch, gyroZrate + gyroZ_OFF, dt);
  last_pitch = pitch;
  gyroZangle += (gyroZrate + gyroZ_OFF) * dt;	// Calculate gyro angle without any filter
  compAngleZ = 0.93 * (compAngleZ + (gyroZrate + gyroZ_OFF) * dt) + 0.07 * pitch; // Calculate the angle using a Complimentary filter

  // Reset the gyro angle when it has drifted too much
  if (gyroZangle < -180 || gyroZangle > 180)
    gyroZangle = kalAngleZ;

  pendulum_angle = constrainAngle(fmod(kalAngleZ, 120) - target_angle);	//摆角计算

  //   pendulum_angle当前角度与期望角度差值，在差值大的时候进行摇摆，差值小的时候LQR控制电机保持平衡
  if (test_flag == 0) //正常控制
  {
    if (abs(pendulum_angle) < swing_up_angle) // if angle small enough stabilize 0.5~30°,1.5~90°
    {
      target_velocity = controllerLQR(pendulum_angle, gyroZrate, motor.shaftVelocity());
      if (abs(target_velocity) > 140)
        //target_velocity = _sign(target_velocity) * 140;

        motor.controller = MotionControlType::velocity;
      motor.move(target_velocity);
    }
    else // else do swing-up
    { // sets swing_up_voltage to the motor in order to swing up
      motor.controller = MotionControlType::torque;
      target_voltage = -_sign(gyroZrate) * swing_up_voltage;
      motor.move(target_voltage);
    }
  }
  else if (test_flag == 1)
  {
    motor.controller = MotionControlType::torque;
    motor.move(target_voltage);
  }
  else
  {
    motor.controller = MotionControlType::velocity;
    motor.move(target_velocity);
  }

  //串口输出数据部分，不需要的情况可以改为0
#if 0

  Serial.print(accX); Serial.print("\t");
  Serial.print(accY); Serial.print("\t");
  Serial.print(atan(accX / accY) / 1.570796 * 90); Serial.print("\t");
  Serial.print(pitch); Serial.print("\t");
  Serial.print(gyroZangle); Serial.print("\t");
  Serial.print(compAngleZ); Serial.print("\t");
  Serial.print(kalAngleZ); Serial.print("\t");

  Serial.print(target_voltage); Serial.print("\t");
  //  Serial.print(target_velocity);Serial.print("\t");
  Serial.print(motor.shaft_velocity); Serial.print("\t");
  Serial.print(target_angle); Serial.print("\t");
  Serial.print(pendulum_angle); Serial.print("\t");
  Serial.print(gyroZrate); Serial.print("\t");
  Serial.print("\r\n");
#endif
  //  motor.move(target_velocity);
  //可以使用该方法wifi发送udp信息
  if (wifi_flag)
  {
    digitalWrite(ACTIVE_PIN, LOW);
    memset(buf, 0, strlen(buf));

    wifi_print("v", motor.shaftVelocity());
    wifi_print("vq", motor.voltage.q);
    wifi_print("p", pendulum_angle);
    wifi_print("t", target_angle);
    wifi_print("k", kalAngleZ);
    wifi_print("g", gyroZrate);
    wifi_print("BAT", driver.voltage_power_supply);

    udp.writeTo((const unsigned char*)buf, strlen(buf), IPAddress(192, 168, 4, 2), localUdpPort); //广播数据
    digitalWrite(ACTIVE_PIN, HIGH);
  }

  //触摸感应处理
  touchAttach(1, T2);
  touchAttach(2, T3);
  touchAttach(3, T4);


  TenthSecondsSinceStartTask();

  //单击事件处理
  if (touch_touched[1]) {
    //Serial.print("\nLight1 touched ");
    //Serial.println(touch_touched_times[1]);
    touch_touched[1] = false;
  }

  if (touch_touched[2]) {
    //Serial.print("\nLight2 touched ");
    //Serial.println(touch_touched_times[2]);

    touch_touched[2] = false;
  }

  //灯光及按键处理
  if ( touch_STATE[1] ) {
    pride();
    addGlitter(15);
    FastLED.show();
  } else {
    FastLED.clearData();
    FastLED.show();
  }

  Debug_Log_func("loop");
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

  if (abs(p_angle) > 3)	//摆角大于2.5则进入非稳态，记录非稳态时间
  {
    last_unstable_time = millis();
    if (stable)	//如果是稳态进入非稳态则调整为目标角度
    {
      target_angle = EEPROM.readFloat(0) - p_angle;
      stable = 0;
    }
  }
  if ((millis() - last_unstable_time) > 500 && !stable)	//非稳态进入稳态超过500ms检测，更新目标角为目标角+摆角，假设进入稳态
  {
    target_angle  -= _sign(target_velocity) * 0.4;
    stable = 1;
  }

  if ((millis() - last_stable_time) > 2500 && stable) { //稳态超过2000ms检测，更新目标角
    if (abs(target_velocity) > 3 && abs(target_velocity) < 10) { //稳态速度偏大校正
      last_stable_time = millis();
      target_angle  -= _sign(target_velocity) * 0.2;
    }
  }

  //Serial.println(stable);
  float u;

  if (!stable)	//非稳态计算
  {
    motor.PID_velocity.P = v_p_1;
    motor.PID_velocity.I = v_i_1;
    u = LQR_K3_1 * p_angle + LQR_K3_2 * p_vel + LQR_K3_3 * m_vel;
  }
  else
  {
    motor.PID_velocity.P = v_p_2;
    motor.PID_velocity.I = v_i_2;
    u = LQR_K4_1 * p_angle + LQR_K4_2 * p_vel + LQR_K4_3 * m_vel;
  }

  return u;
}
void wifi_print(char * s, double num)
{
  char str[255];
  char n[255];
  sprintf(n, "%.2f", num);
  strcpy(str, s);
  strcat(str, n);
  strcat(buf + strlen(buf), str);
  strcat(buf, ",\0");
}


unsigned long LastMillis = 0;
void TenthSecondsSinceStartTask() //100ms
{
  unsigned long CurrentMillis = millis();
  ESP32Server.handleClient();
  delay(1);//allow the cpu to switch to other tasks
  if (abs(int(CurrentMillis - LastMillis)) > 100)
  {
    LastMillis = CurrentMillis;
    TenthSecondsSinceStart++;
    OnTenthSecond();
  }
}

void OnSecond()
{
  time_t now = time(nullptr); //获取当前时间

  //转换成年月日的数字，可以更加自由的显示。
  struct   tm* timenow;
  timenow = localtime(&now);
  unsigned char tempHour = timenow->tm_hour;
  unsigned char tempMinute = timenow->tm_min;
  unsigned char tempSecond = timenow->tm_sec;
  unsigned char tempDay = timenow->tm_mday;
  unsigned char tempMonth = timenow->tm_mon + 1;
  unsigned int tempYear = timenow->tm_year + 1900;
  unsigned char tempWeek = timenow->tm_wday;


  //生成  年月日时分秒 字符串。
  sprintf(DateTimeStr, "%d-%02d-%02d %02d:%02d:%02d"
          , tempYear
          , tempMonth
          , tempDay
          , tempHour
          , tempMinute
          , tempSecond
         );

  //Serial.println(DateTimeStr);

#if defined(BAT_VOLTAGE_SENSE_PIN)  //电池电压检测
  //driver.voltage_power_supply = return_voltage_value(BAT_VOLTAGE_SENSE_PIN);
  bat_voltage = return_voltage_value(BAT_VOLTAGE_SENSE_PIN);
  //Serial.println(driver.voltage_power_supply);
  if (bat_voltage < min_voltage && !battery_low)
  {
    battery_low = 1;
    Serial.print(driver.voltage_power_supply);
    Serial.println("V ");
    Serial.print(bat_voltage);
    Serial.println("V battery_low!!");
    while (battery_low)
    {
      FastLED.clearData();
      FastLED.show();
      //motor.loopFOC();
      //motor.move(0);
      motor.disable();

      bat_voltage = return_voltage_value(BAT_VOLTAGE_SENSE_PIN);
      if (bat_voltage >= (min_voltage + 0.5)) {
        Serial.print(driver.voltage_power_supply);
        Serial.println("V");
        Serial.print(bat_voltage);
        Serial.println("V battery ok");
        digitalWrite(ACTIVE_PIN, 0);  //电池电压恢复则常亮，需reset重启
        //battery_low = 0;
      } else {  //电池电压低闪灯
        if (millis() % 500 < 250)
          digitalWrite(ACTIVE_PIN, 1);
        else
          digitalWrite(ACTIVE_PIN, 0);
      }
    }
  }
#endif

  if (touchDetected[1] > 0) { //检测到触摸中，一秒计数一次，未触摸则清零
    touch_touching_time[1]++;

    //Serial.print("\nLight1 touching ");
    //Serial.println(touch_touching_time[1]);
    //长按事件处理
    if (touch_touching_time[1] % 2 == 0) { //按住大于2秒关灯或者开灯
      touch_STATE[1] = !touch_STATE[1]; //灯光状态反处理
    }

  }
  if (touchDetected[2] > 0) { //检测到触摸中，一秒计数一次，未触摸则清零
    touch_touching_time[2]++;
    //Serial.print("\nLight2 touching ");
    //Serial.println(touch_touching_time[2]);

    if (touch_touching_time[2] % 2 == 0) { //按住大于2秒事件处理

    }
  }
  if (touchDetected[3] > 0) { //检测到触摸中，一秒计数一次，未触摸则清零
    touch_touching_time[3]++;
    //Serial.print("\nLight2 touching ");
    //Serial.println(touch_touching_time[2]);

    if (touch_touching_time[3] % 2 == 0) { //按住大于2秒事件处理

    }
  }
}

void OnTenthSecond()  // 100ms 十分之一秒
{

  if (TenthSecondsSinceStart % 3 == 0) //0.3S刷新
  {
    if ( touch_touching_time[2] > 1) {	//按键2长按大于1秒处理调光
      if ( touch_touched_times[2] == 0 || touch_touched_times[2] % 2 == 0 ) { //第0，2,4,6..次按加亮度，1,3,5...则减
        rgb_brightness = rgb_brightness + 5;
      } else {
        rgb_brightness = rgb_brightness - 5;
      }
      //Serial.println(rgb_brightness);
      FastLED.setBrightness(rgb_brightness);
    }
  }

  if (TenthSecondsSinceStart % 10 == 0) //10次为1秒
  {
    OnSecond();
  }
}

String TimeString(int TimeMillis) {
  char stringTime[10];
  int sec = TimeMillis;
  int min = sec / 60;
  int hr = min / 60;

  sprintf(stringTime, "%02d:%02d:%02d",
          hr, min % 60, sec % 60
         );
  return stringTime;
}

String ProcessUpdate()    //页面更新
{
  //自动生成一串用“,”隔开的字符串。
  //HTML脚本会按照“, ”分割，形成一个字符串数组。
  //并把这个数组填个表格的相应部分。
  String ReturnString;
  ReturnString = DateTimeStr;

  ReturnString += ",";
  ReturnString += TimeString(millis() / 1000);

  ReturnString += ",";
  ReturnString += log_control;
  ReturnString += ",";
  ReturnString += debug_log_control;
  ReturnString += ",";
  ReturnString += bat_voltage;
  if (log_control) {
    ReturnString += ",";
    ReturnString += motor.shaftVelocity();
    ReturnString += ",";
    ReturnString += motor.voltage.q;
    ReturnString += ",";
    ReturnString += target_velocity;
    ReturnString += ",";
    ReturnString += pendulum_angle;
    ReturnString += ",";
    ReturnString += target_angle;
    ReturnString += ",";
    ReturnString += kalAngleZ;
    ReturnString += ",";
    ReturnString += gyroZangle;
  } else {
    ReturnString += ",,,,,,,";
  }

  ReturnString += ",";
  ReturnString += EEPROM.readFloat(0);
  ReturnString += ",";
  ReturnString += swing_up_voltage;
  ReturnString += ",";
  ReturnString += swing_up_angle;
  ReturnString += ",";
  ReturnString += v_i_1;
  ReturnString += ",";
  ReturnString += v_p_1;
  ReturnString += ",";
  ReturnString += v_i_2;
  ReturnString += ",";
  ReturnString += v_p_2;
  ReturnString += ",";
  if (debug_log_control) {
    for (int i = 0; i < debug_times; i++) {
      if (String(Debug_Log[i]) != "") {
        //Serial.println(Debug_Log[i]);
        ReturnString += Debug_Log[i];
        memset( Debug_Log[debug_times], 0, sizeof(Debug_Log[debug_times]) );
      }
      if (i == debug_times - 1)
        debug_times = 0;
    }
  }
  //Serial.println(ReturnString);
  return ReturnString;
}

/*
  DeviceType  =0
  DeviceType  =1

  OPERATION_ON      0,3,6,9
  OPERATION_OFF     1,4,7,10
  OPERATION_ON_OFF  2,5,8,11
*/
void PocessControl(int DeviceType, int DeviceIndex, int Operation, float Operation2)
{
  String ReturnString;
  char do_commd[20];
  int SysIndex = 6;

  if (DeviceType == 0)  //系统操作：开关灯，调节亮度，重启
  {
    if (DeviceIndex == 0)
    {

      if (Operation % SysIndex == 0)
      {
        touch_STATE[1] = true;
        ReturnString += "开灯 亮度 ";
        ReturnString += String(rgb_brightness);
      }
      else if (Operation % SysIndex == 3)  //操作off
      {
        touch_STATE[1] = false;
        ReturnString += "关灯";
      }
      else if (Operation % SysIndex == 1)  //操作+
      {
        rgb_brightness = (rgb_brightness + 5) % 260;
        FastLED.setBrightness(rgb_brightness);
        ReturnString += "亮度增加至 ";
        ReturnString += String(rgb_brightness);
        if (!touch_STATE[1])
          ReturnString += " 【灯光已关闭】";
      }
      else if (Operation % SysIndex == 2)  //操作-
      {
        if (rgb_brightness == 0)
          rgb_brightness = 255;
        else
          rgb_brightness = rgb_brightness - 5;
        FastLED.setBrightness(rgb_brightness);
        ReturnString += "亮度降低至 ";
        ReturnString += String(rgb_brightness);
        if (!touch_STATE[1])
          ReturnString += " 【灯光已关闭】";
      }
      else if (Operation % SysIndex == 4)
      {
        ReturnString += "系统重启，请等待重新连接";
        ESP32Server.send(200, "text/plain", ReturnString);
        printf("Reboot...");
        esp_restart();
      }
    } else if (DeviceIndex == 5) {  //参数记录输出控制
      if (Operation % SysIndex == 0)
        log_control = 0;
      else if (Operation % SysIndex == 1)
        log_control = 1;
    } else if (DeviceIndex == 6) {  //DEBUG输出控制
      if (Operation % SysIndex == 0)
        debug_log_control = 0;
      else if (Operation % SysIndex == 1)
        debug_log_control = 1;
    }
  }

  if (DeviceType == 1)  //调参
  {
    if (Operation == 0)
    {
      sprintf(do_commd, "%.2f", Operation2);
      //Serial.println(do_commd);
      if (DeviceIndex == 0)  //期望角度TA
      {
        do_TA(do_commd);
      } else if (DeviceIndex == 1)  //摇摆电压SV
      {
        do_SV(do_commd);
      } else if (DeviceIndex == 2)  //摇摆角度SA
      {
        do_SA(do_commd);
      } else if (DeviceIndex == 3)  //速度环P1
      {
        do_vp1(do_commd);
      } else if (DeviceIndex == 4)  //速度环I1
      {
        do_vi1(do_commd);
      } else if (DeviceIndex == 5)  //速度环P2
      {
        do_vp2(do_commd);
      } else if (DeviceIndex == 6)  //速度环I2
      {
        do_vi2(do_commd);
      } else if (DeviceIndex == 99)  //电机启停
      {
        do_MOTOR(do_commd);
        if (!Motor_enable_flag)
          ReturnString += "电机启动...";
        else
          ReturnString += "电机停机...";
      }
      EEPROM.commit();

    }
  }
  ESP32Server.send(200, "text/plain", ReturnString);
}


bool handleFileRead(String path) {            //处理主页访问
  String contentType = "text/html";

  if (SPIFFS.exists(path)) {                       // 如果访问的文件可以在SPIFFS中找到
    File file = SPIFFS.open(path, "r");          // 则尝试打开该文件
    ESP32Server.streamFile(file, contentType);   // 并且将该文件返回给浏览器
    file.close();                                // 并且关闭文件
    return true;                                 // 返回true
  }
  return false;                                    // 如果文件未找到，则返回false
}

void handleNotFound()
{
  // 获取用户请求网址信息
  String webAddress = ESP32Server.uri();
  int AutheTimes  = 0;

  if (!ESP32Server.authenticate(username, userpassword)) //校验用户是否登录
  {
    if (AutheTimes == 3) {
      ESP32Server.send(404, "text/plain", "Bye");
    } else {
      AutheTimes++;
      return ESP32Server.requestAuthentication(); //请求进行用户登录认证
    }
  }

  //打印出请求
  if (webAddress != "/update")
  {
    printf("%s\n", webAddress.c_str());
  }

  //如果是主页请求，则发送FLASH中的index.html文件
  if (webAddress.endsWith("/")) {                   // 如果访问地址以"/"为结尾
    webAddress = "/index.html";                     // 则将访问地址修改为/index.html便于SPIFFS访问

    // 通过handleFileRead函数处处理用户访问
    handleFileRead(webAddress);
  }
  else if (webAddress.endsWith("update"))
  {
    ESP32Server.send(200, "text/plain", ProcessUpdate());
  }
  else if (webAddress.startsWith("/Control"))
  {
    if (ESP32Server.args() == 3)
    {
      int DeviceType = ESP32Server.arg(0).toInt();
      int DeviceIndex = ESP32Server.arg(1).toInt();
      int Operation = ESP32Server.arg(2).toInt();
      float Operation2 = ESP32Server.arg(2).toFloat();
      if (DeviceType == 1) {
        Operation = 0;
      }

      //printf("DeviceType:%d DeviceIndex:%d Operation:%d Operation2:%.2f\n", DeviceType, DeviceIndex, Operation, Operation2 );

      PocessControl(DeviceType, DeviceIndex, Operation, Operation2);
    }
    else
    {
      ESP32Server.send(404, "text/plain", "404 Not Found");
    }
  }
  else
  {
    ESP32Server.send(404, "text/plain", "404 Not Found");
  }
}

void StartWebServer()
{
  ESP32Server.begin();
  ESP32Server.onNotFound(handleNotFound);//将所有请求导向自己处理的代码
}


// This function draws rainbows with an ever-changing,
// widely-varying set of parameters.
void pride()
{
  static uint16_t sPseudotime = 0;
  static uint16_t sLastMillis = 0;
  static uint16_t sHue16 = 0;

  uint8_t sat8 = beatsin88( 87, 220, 250);
  uint8_t brightdepth = beatsin88( 341, 96, 224);
  uint16_t brightnessthetainc16 = beatsin88( 203, (25 * 256), (40 * 256));
  uint8_t msmultiplier = beatsin88(147, 23, 60);

  uint16_t hue16 = sHue16;//gHue * 256;
  uint16_t hueinc16 = beatsin88(113, 1, 3000);

  uint16_t ms = millis();
  uint16_t deltams = ms - sLastMillis ;
  sLastMillis  = ms;
  sPseudotime += deltams * msmultiplier;
  sHue16 += deltams * beatsin88( 400, 5, 9);
  uint16_t brightnesstheta16 = sPseudotime;

  for ( uint16_t i = 0 ; i < NUM_LEDS; i++) {
    hue16 += hueinc16;
    uint8_t hue8 = hue16 / 256;

    brightnesstheta16  += brightnessthetainc16;
    uint16_t b16 = sin16( brightnesstheta16  ) + 32768;

    uint16_t bri16 = (uint32_t)((uint32_t)b16 * (uint32_t)b16) / 65536;
    uint8_t bri8 = (uint32_t)(((uint32_t)bri16) * brightdepth) / 65536;
    bri8 += (255 - brightdepth);

    CRGB newcolor = CHSV( hue8, sat8, bri8);

    uint16_t pixelnumber = i;
    pixelnumber = (NUM_LEDS - 1) - pixelnumber;

    nblend( leds[pixelnumber], newcolor, 64);
  }
}

void addGlitter( fract8 chanceOfGlitter)
{
  if ( random8() < chanceOfGlitter) {
    leds[ random16(NUM_LEDS) ] += CRGB::White;
  }
}

double return_voltage_value(int pin_no)
{
  double tmp = 0;
  double ADCVoltage = 0;
  double inputVoltage = 0;
  double avg = 0;

  for (int i = 0; i < 150; i++)
  {
    tmp = tmp + analogRead(pin_no);
  }
  avg = tmp / 150;

  //avg = analogRead(pin_no);
  ADCVoltage = ((avg * 3.3) / (4095)) + 0.101;
  inputVoltage = ADCVoltage / (R2_VOLTAGE / (R1_VOLTAGE + R2_VOLTAGE)); // formula for calculating voltage in i.e. GND
  return inputVoltage;
}

//触摸感应处理
void touchAttach(int touchID, uint8_t touchPin) {
  touchread[touchID] = touchRead(touchPin);
  if ( touchread[touchID] <= threshold_top && touchread[touchID] > threshold_bottom ) { //达到触发值的计数
    //delay(38);  // 0.038秒
    touchDetected[touchID]++; //持续触摸计数
    if ( (touchDetected[touchID] >= threshold_count) && digitalRead(ACTIVE_PIN) == HIGH  ) {  //达到触发值的，灯不亮则亮灯
      digitalWrite(ACTIVE_PIN, LOW);
    }
  } else if (touchread[touchID] > threshold_top) { //无触摸处理
    if ( digitalRead(ACTIVE_PIN) == LOW ) { //灭触摸灯
      digitalWrite(ACTIVE_PIN, HIGH);
    }
    if ( touchDetected[touchID] >= threshold_count ) {  //检测无触摸之前的有效计数，触摸过则标记
      touch_touched[touchID] = true;
      touch_touched_times[touchID]++;  //触摸计数+1
    }
    touch_touching_time[touchID] = 0;  //持续触摸时间清零
    touchDetected[touchID]    = 0;  //持续触摸计数清零
  }
}
