## IIC简介
  IIC(Inter－Integrated Circuit)总线是一种由 PHILIPS 公司开发的两线式串行总线，用于连接微控制器及其外围设备。
  数据线SDA + 时钟线SCL，可发送和接收数据
  IIC总线传输过程3种类型信号：开始信号，结束信号，应答信号
  开始信号：SCL为高，SDA由高电平向低电平跳变，开始传送数据。
  结束信号：SCL为高，SDA由低电平向高电平跳变，结束传送数据。
  应答信号：接收数据的 IC 在接收到 8bit 数据后，向发送数据的 IC 发出特定的低电平脉冲，表示已收到数据。

IIC总线时序图如下：
![这是图片](D:/Notes/STM32_Learn/7-IIC/IIC总线时序图.png "Magic Gardens")