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

const char *ssid = "esp32";
const char *password = "12345678";

AsyncUDP udp;                     //创建UDP对象
unsigned int localUdpPort = 2333; //本地端口号

MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

TwoWire I2Cone = TwoWire(0);


//电机参数
BLDCMotor motor = BLDCMotor(5);
BLDCDriver3PWM driver = BLDCDriver3PWM(32, 33, 25, 22);


//命令设置
int target_velocity = 0;
void onPacketCallBack(AsyncUDPPacket packet)
{
  target_velocity = atoi((char*)(packet.data()));
  Serial.print("数据内容: ");
  Serial.println(target_velocity);
//  packet.print("reply data");
}

void setup() {
   Serial.begin(115200);
  //wifi初始化
  WiFi.mode(WIFI_AP);
   while(!WiFi.softAP(ssid, password)){}; //启动AP
    Serial.println("AP启动成功");
  while (!udp.listen(localUdpPort)) //等待udp监听设置成功
  {
  }
  udp.onPacket(onPacketCallBack); //注册收到数据包事件
  I2Cone.begin(23, 5, 400000);   //SDA,SCL

  sensor.init(&I2Cone);

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
  motor.PID_velocity.P = 1.5;

  motor.PID_velocity.I = 20;

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


  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target velocity using serial terminal:"));
  
}



void loop() {
  motor.loopFOC();


  motor.move(target_velocity);

}
