#include <WiFi.h>
#include <AsyncUDP.h> //引用以使用异步UDP
#include<Wire.h>

const char *ssid = "esp32";
const char *password = "12345678";

const int MPU_addr=0x68; // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

AsyncUDP udp;                     //创建UDP对象
unsigned int localUdpPort = 2333; //本地端口号


unsigned int broadcastPort = localUdpPort;
//const char *broadcastData = "broadcast data";
 const uint8_t broadcastData[] = {"broadcast data"};

void onPacketCallBack(AsyncUDPPacket packet)
{
  Serial.print("UDP数据包来源类型: ");
  Serial.println(packet.isBroadcast() ? "广播数据" : (packet.isMulticast() ? "组播" : "单播"));
  Serial.print("远端地址及端口号: ");
  Serial.print(packet.remoteIP());
  Serial.print(":");
  Serial.println(packet.remotePort());
  Serial.print("目标地址及端口号: ");
  Serial.print(packet.localIP());
  Serial.print(":");
  Serial.println(packet.localPort());
  Serial.print("数据长度: ");
  Serial.println(packet.length());
  Serial.print("数据内容: ");
  Serial.write(packet.data(), packet.length());
  Serial.println();

//  packet.print("reply data");
//  broadcastPort = packet.remotePort();
}

void setup()
{
     Serial.begin(115200);
    // mpu6050初始化
     Wire.begin(19, 18, 400000);
 Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  //wifi初始化
  WiFi.mode(WIFI_AP);
   while(!WiFi.softAP(ssid, password)){}; //启动AP
    Serial.println("AP启动成功");
  while (!udp.listen(localUdpPort)) //等待udp监听设置成功
  {
  }
  udp.onPacket(onPacketCallBack); //注册收到数据包事件
}
char s[255];
void loop()
{
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true); // request a total of 14 registers
//  AcX=Wire.read()<<8|Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY=Wire.read()<<8|Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
//  AcZ=Wire.read()<<8|Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
//  Tmp=Wire.read()<<8|Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
//  GyX=Wire.read()<<8|Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
//  GyY=Wire.read()<<8|Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
//  GyZ=Wire.read()<<8|Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  
//  Serial.print("AcX = "); Serial.print(AcX);
  Serial.print(" | AcY = "); Serial.println(AcY);
//  itoa(AcY,s,10);

   sprintf(s, "%d", AcY); //将100转为16进制表示的字符串
   Serial.println(strlen(s));
//  Serial.print(" | AcZ = "); Serial.print(AcZ);
//  Serial.print(" | Tmp = "); Serial.print(Tmp/340.00+36.53); //equation for temperature in degrees C from datasheet
//  Serial.print(" | GyX = "); Serial.print(GyX);
//  Serial.print(" | GyY = "); Serial.print(GyY);
//  Serial.print(" | GyZ = "); Serial.println(GyZ);
  delay(100);

//  udp.broadcastTo(broadcastData, broadcastPort); //可以使用该方法广播信息

   IPAddress broadcastAddr((~(uint32_t)WiFi.subnetMask())|((uint32_t)WiFi.localIP())); //计算广播地址
   udp.writeTo((const unsigned char*)s, strlen(s), broadcastAddr, localUdpPort); //广播数据
}
