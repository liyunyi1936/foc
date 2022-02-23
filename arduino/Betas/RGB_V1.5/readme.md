本版本是慕炎RGB版本程序 基于45°(https://gitee.com/coll45/foc/)程序修改
同步更新 https://gitee.com/muyan3000/RGBFOC

本版本虽然能在原版本pcb上运行，但完整功能需要对应的硬件支持才能获得

【硬件基于45°工程文件修改】
https://oshwhub.com/muyan2020/zi-ping-heng-di-lai-luo-san-jiao_10-10-ban-ben_copy
1、去掉ch340和自动下载电路更换为rx tx插针，使用时不要连接3.3v插针，烧录前按住boot不放，再按一下reset，然后放掉所有键（如果打开串口监视发现数据乱码，把rx和tx线对换）

2、TPS54331更改为5V输出供电给RGB灯，5V输入到ams1117-3.3

3、预留2个触摸区和增加esp32状态灯，KEY3触摸开关灯，KEY4长按调光


【程序功能】增加如下：
1、OTA
2、RGB灯控制程序
3、触摸控制程序
4、基于webserver的基础调参功能

webserver需要使用【ESP32 SPIFFS】文件上传

下载插件复制到C:\Program Files (x86)\Arduino\tools\ESP32FS\tool
重启Arduino即可

arduino-esp32fs-plugin
https://github.com/me-no-dev/arduino-esp32fs-plugin/releases/tag/1.0

使用时在arduino界面点“工具”-“ESP32 Sketch Data Upload”