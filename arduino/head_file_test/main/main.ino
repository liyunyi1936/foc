#include "Command.h"


//命令设置
float target_velocity = 0;
Command  comm;//声明一个自己的该类的对象
void doTarget(char* cmd) { comm.scalar(&target_velocity, cmd); }

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  comm.add("T",doTarget);
}

void loop() {
  // put your main code here, to run repeatedly:
  comm.run("T233.2548");
  Serial.println(target_velocity);
}
