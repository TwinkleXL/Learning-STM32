# STM32初级篇学习报告

        考虑到自身能力水平的限制和综合学长的一些建议，我选择了STM32单片机来进行学习。之前也有接触过STM32，但没有系统和层进式地学习STM32，对STM32还仅停留在似懂非懂的层面上，没办法达到真正掌握的程度。于是，结合网络教学视频、教材和STM32F103开发板来进行了STM32初级学习。

## STM32之命名方法：

![在这里插入图片描述](https://img-blog.csdnimg.cn/20201208003321343.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzUxMzkyMDc2,size_16,color_FFFFFF,t_70)

## STM32之内核与外设：

        STM32芯片是已经封装好的成品，主要由内核和片上外设组成。STM32F103采用的是Cortex-M3内核，内核即CPU，由ARM公司设计。ARM公司并不生产芯片，而是出售其芯片技术授权。芯片生产厂商(SOC)如 ST、 TI、 Freescale，负责在内核之外设计部件并生产整个芯片，这些内核之外的部件被称为核外外设或片上外设。如GPIO、USART（串口）、I2C、SPI等都叫做片上外设。
![STM32芯片](https://img-blog.csdnimg.cn/20201208003647822.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzUxMzkyMDc2,size_16,color_FFFFFF,t_70)![在这里插入图片描述](https://img-blog.csdnimg.cn/20201208003820380.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzUxMzkyMDc2,size_16,color_FFFFFF,t_70)

## STM32之存储器映射&寄存器映射：

        所谓**存储器映射**即给存储器分配地址的过程。如果给存储器再分配一个地址就叫**存储器重映射**。如下图所示：
![在这里插入图片描述](https://img-blog.csdnimg.cn/20201208003857914.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzUxMzkyMDc2,size_16,color_FFFFFF,t_70)
        

        给已经分配好地址的有特定功能的内存单元取别名的过程就叫**寄存器映射**。比如GPIOA寄存器是对描述PA0-PA15输出输出端口特性功能的内存单元取了一个能描述其相应功能的名称，在代码中通过该寄存器来实现对该内存单元的相关操作，而无须对其绝对地址直接操作从而增加了代码的可读性。

## STM32之外设基地址与地址偏移：

        外设寄存器分配在一个线性地址空间中，每个外设寄存器都有相应的起始地址称为**外设基地址**；寄存器内部相应功能寄存器对基地址的偏移称为**地址偏移**。比如：外设基地址为(0x40000000),APB2总线的基地址为(0x40000000 + 0x10000),GPIOA挂载在APB2总线上，GPIOA的基地址为(0x40000000 + 0x10000 + 0x0800),GPIOA->ODR的地址为(0x40000000 + 0x10000 + 0x0800 + 0x0C)，其中偏移地址为(0x0C)。

## STM32之寄存器封装：

        将基地址用相应的宏定义起来以便于理解和记忆(#define)，将寄存器用C语言的结构体封装起来以便于访问寄存器(typedef struct)。通过结构体针指针来访问相应寄存器。

## STM32之标准库：

        库是架设在寄存器与用户驱动层之间的代码，向下处理与寄存器直接相关的配置，向上为用户提供配置寄存器的接口。库帮我们实现了寄存器配置的工作，使我们脱离了底层操作绝对地址而通过函数接口来实现对相应地址的操作。
![在这里插入图片描述](https://img-blog.csdnimg.cn/20201208004414506.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzUxMzkyMDc2,size_16,color_FFFFFF,t_70)

## THE END:

在用固件库或HAL库编程时，我们并不用考虑以上内容,所以也并不是需要熟练掌握的内容。可能是因为我在用库编程时一直疑惑代码具体是如何实现的，所以才选择对这部分内容做些了解与学习吧。这一阶段学习内容并没有什么技术性进展，之后还要学习&练习。
