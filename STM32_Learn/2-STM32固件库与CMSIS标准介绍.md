  stm32固件库就是函数集合。
  ARM：做芯片标准的公司，负责芯片内科架构设计
  TI、ST：芯片公司，根据ARM公司提供的芯片内核设计自己的芯片。
  如下图：
  ![这是图片](D:/Notes/STM32_Learn/2-固件库介绍/Cortex-m3芯片结构图.png "Magic Gardens")
  
  ARM公司提出CMSIS标准(Cortex Microcontroller Software Interface Standard)：ARM Cortex™ 微控制器软件接口标准。
  CMSIS应用程序基本结构：
    ![这是图片](D:/Notes/STM32_Learn/2-固件库介绍/CMSIS应用基本结构.png "Magic Gardens")

CMSIS分3个基本功能层：
     1）核内外设访问层：定义处理内部寄存器地址及功能函数
     2）中间件访问层：定义访问中间件的通用 API,也是 ARM 公司提供
     3）外设访问层：定义硬件寄存器的地址以及外设的访问函数

### STM32官方库包介绍
  ![这是图片](D:/Notes/STM32_Learn/2-固件库介绍/官方库目录列表.png "Magic Gardens")

CMSIS：存放启动文件
STM32F10x_StdPeriph_Driver：STM32 固件库源码文件。inc：stm32f10x_xxx.h 头文件； src：stm32f10x_xxx.c 格式的固件库源码文件
Project：实例
Utilities：官方评估板源码


STM32F10x_StdPeriph_Lib_V3.5.0\Libraries\CMSIS\CM3\CoreSupport中core_cm3.c和core_cm3.h：CMSIS内核文件。
STM32F10x_StdPeriph_Lib_V3.5.0\Libraries\CMSIS\CM3\DeviceSupport\ST\STM32F10x\startup：启动文件，基础寄存器定义，中断向量定义

system_stm32f10x.c 和对应的头文件 system_stm32f10x.h 文件的功能是设置系统以及总线时钟：的 SystemInit()函数，用来设置系统的整个时钟系统

stm32f10x.h：结构体以及宏定义，系统寄存器定义申明以及包装内存操作




