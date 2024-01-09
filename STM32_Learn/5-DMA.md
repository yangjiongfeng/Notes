DMA：全称为Direct Memory Access，即直接访问寄存器，DMA传输将数据从一个地址空间复制到另外地址空间。

5-1. DMA中断寄存器（DMA_ISR），只读寄存器，该寄存器的各位描述如下：  
![这是图片](D:/Notes/STM32_Learn/5-DMA/DMA_ISR寄存器各位描述.png "Magic Gardens")


5-2. DMA中断标志清除寄存器（DMA_IFCR），该寄存器各位描述，如下：
![这是图片](D:/Notes/STM32_Learn/5-DMA/DMA_IFCR寄存器各位描述.png "Magic Gardens")

DMA_IFCR各位用来清除AMD_ISR对应位，通过写0清除。

5-3. DMA通道x配置寄存器（DMA_CCRx）

      数据宽度、外设及存储器宽度、通道优先级、增量模式、传输方向、中断允许、使能


5-4. DMA通道x传输数据量寄存器（DMA_CNDTRx）
这个寄存器控制 DMA 通道 x 的每次传输所要传输的数据量。其设置范围为 0~65535。并且该寄存器的值会随着传输的进行而减少，当该寄存器的值为 0 的时候就代表此次数据传输已经全部发送完成了。所以可以通过这个寄存器的值来知道当前 DMA 传输的进度。

5-5. DMA通道x外设地址寄存器（DMA_CPARx）
该寄存器用来存储 STM32 外设的地址，比如我们使用串口 1，那么该寄存器必须写入 0x40013804（其实就是&USART1_DR）。如果使用其他外设，就修改成相应外设的地址就行了。

5-6. DMA通道x存储器地址寄存器（DMA_CMARx）
该寄存器和 DMA_CPARx 差不多，但是是用来放存储器的地址的。比如我们使用 SendBuf[5200]数组来做存储器，那么我们在DMA_CMARx 中写入&SendBuff 就可以了。


## DMA配置步骤
1. 使能DMA时钟
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE); //使能 DMA 时钟
2. 初始化DMA通道4参数
    void DMA_Init(DMA_Channel_TypeDef* DMAy_Channelx, DMA_InitTypeDef* DMA_InitStruct)
    
    typedef struct
	{
		uint32_t DMA_PeripheralBaseAddr;
		uint32_t DMA_MemoryBaseAddr;
		uint32_t DMA_DIR;
		uint32_t DMA_BufferSize;
		uint32_t DMA_PeripheralInc;
		uint32_t DMA_MemoryInc;
		uint32_t DMA_PeripheralDataSize;
		uint32_t DMA_MemoryDataSize;
		uint32_t DMA_Mode;
		uint32_t DMA_Priority;
		uint32_t DMA_M2M;
	}DMA_InitTypeDef;

	实例代码：
	DMA_InitTypeDef DMA_InitStructure;
	DMA_InitStructure.DMA_PeripheralBaseAddr = &USART1->DR;                        //DMA 外设 ADC 基地址
	DMA_InitStructure.DMA_MemoryBaseAddr = cmar;                                           //DMA 内存基地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;                                  //从内存读取发送到外设
	DMA_InitStructure.DMA_BufferSize = 64;                                                           //DMA 通道的 DMA 缓存的大小
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;              //外设地址不变
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                    //内存地址递增
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //8 位
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;      // 8 位
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                                     //工作在正常缓存模式
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;                               //DMA 通道 x 拥有中优先级
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                                       //非内存到内存传输
	DMA_Init(DMA_CHx, &DMA_InitStructure);                                                        //根据指定的参数初始化

3. 使能串口DMA发送
    USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);
    如果是要使能串口 DMA 接受，那么第二个参数修改为 USART_DMAReq_Rx 即可

4. 使能DMA1通道4，启动传输
    DMA_Cmd(DMA_CHx, ENABLE);
    
 5. 查询DMA传输状态
     FlagStatus DMA_GetFlagStatus(uint32_t DMAy_FLAG)
     
     比如：要查询 DMA 通道 4 传输是否完成的方法：
     DMA_GetFlagStatus(DMA2_FLAG_TC4);

     获取当前剩余数据量大小的函数：
     uint16_t DMA_GetCurrDataCounter(DMA_Channel_TypeDef* DMAy_Channelx)

     获取 DMA 通道 4 还有多少个数据没有传输的函数：
     DMA_GetCurrDataCounter(DMA1_Channel4);






































