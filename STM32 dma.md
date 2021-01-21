# STM32之dma篇

## 1.DMA简述

        直接存储器存取(Direct Memory Access)用来提供在<u>外设和存储器之间</u>或者<u>存储器和存储器之间</u>的高速数据传输。无须CPU干预，数据可以通过DMA快速地移动，这就节省了CPU的资源来做其他操作。两个DMA控制器有12个通道(DMA1有7个通道， DMA2有5个通道)，每个通道专门用来管理来自于一个或多个外设对存储器访问的请求。还有一个仲裁器来协调各个DMA请求的优先权。

### 1.1 DMA通道

        不同外设可以通过DMA1控制器的通道[7:1]或者DMA2控制器的通道[5:1]发出DMA请求，进而与存储器之间进行DMA数据传输。不同的外设DMA请求分别对应着DMA的12个通道，对应关系(请求映像)如下：

DMA1请求映像：

![](C:\Users\30187\Desktop\dma1映像.png)

DMA2请求映像：

![](C:\Users\30187\Desktop\dma2映像.png)

注：以上请求映像是对外设与存储器之间的DMA数据传输而言的，对存储器与存储器之间的DMA数据传输而言，DMA的12个通道均可以使用，且在存储器到存储器模式下，DMA通道的操作可以在没有外设请求的情况下进行。

### 1.2 DMA通道优先级

        DMA通过一个仲裁器来协调各个DMA请求的优先权。仲裁器根据通道请求的优先级来启动外设/存储器的访问。通道优先级可分为软件优先级和硬件优先级。其中软件优先级可以配置为以下四个等级：

| 软件优先级 |
| ----- |
| 最高优先级 |
| 高优先级  |
| 中等优先级 |
| 低优先级  |

如果2个请求有相同的软件优先级，则比较它们的硬件优先级。

硬件优先级原则是：DMA1>DMA2；通道x>通道y，其中x<y。

### 1.3 DMA传输数据对齐方式

        我们称数据发送端的数据宽度（如uint8_t、uint16_t 等）为源端宽度，数据接收端的数据宽度为目标宽度的话，那么我自己总结的它们的对齐方式：读取源端数据的全部数据位，写入目标数据时要考虑：①当源端宽度大于目标宽度时，从源端数据的低位开始写入目标数据存储器直到写入数据的数据宽度与目标宽度相等；②当源端宽度不大于目标宽度时，远端数据全部写入到目标数据存储器中。例如：

| 源端宽度 | 目标宽度 | 源端数据/地址    | 目标数据/地址    |
| ---- | ---- | ---------- | ---------- |
| 16   | 8    | 0xA1B2     | 0xB2       |
| 16   | 16   | 0xA1B2     | 0xA1B2     |
| 16   | 32   | 0xA1B2     | 0xA1B2     |
| 32   | 32   | 0xA1B2C3D4 | 0xA1B2C3D4 |
| 32   | 16   | 0xA1B2C3D4 | 0xC3D4     |

## 2.DMA初始化配置

        对于DMA初始化配置，我们可以在stm32f10x_dma.h文件中找到一个库函数来实现，该函数声明如下：

```c
void DMA_Init(DMA_Channel_TypeDef* DMAy_Channelx, DMA_InitTypeDef* DMA_InitStruct);
```

可以看到该函数需要我们选择DMA通道和DMA配置结构体类型中的各成员变量。

其中该结构体类型定义如下：

```c
typedef struct
{
  /*解决数据来源和去路*/
  uint32_t DMA_PeripheralBaseAddr; //外设寄存器地址
  uint32_t DMA_MemoryBaseAddr; //数据存储器地址    
  uint32_t DMA_DIR; //DMA数据传输方向（从外设地址读取or写入外设地址）

  /*设置数据传输数目 源端数据宽度 目标数据宽度 地址增量模式*/          
  uint32_t DMA_BufferSize; //一次传输数据的数据量        
  uint32_t DMA_PeripheralInc; //传输一个数据单元后外设地址是否加一    
  uint32_t DMA_MemoryInc; //传输一个数据单元后存储器地址是否加一         
  uint32_t DMA_PeripheralDataSize; //外设传输数据宽度
  uint32_t DMA_MemoryDataSize; //存储器传输数据宽度 

  /*设置传输模式 优先级 M2M模式*/
  uint32_t DMA_Mode; //数据传输单次/循环传输              
  uint32_t DMA_Priority; //DMA通道软件优先级    
  uint32_t DMA_M2M; //使能/使能存储器到存储器模式

} DMA_InitTypeDef;
```

以上要填入各成员变量的值均可可以按照需求在"stm32f10x_dma.h"文件中找到对应的值写入。

## 3.DMA存储器到存储器模式下传输数据（MtoM）

### 3.1 DMA配置与初始化

        DMA作为一个外设，首先应该使能该外设时钟；在完成DMA相关寄存器配置，即调用DMA配置库函数；最后使能DMA，使其开始正常运行。相关宏定义和代码如下：

宏定义如下：

```c
/*要发送的数据大小*/
#define Buffer_Size    10
/*数据发送和接收的存储空间*/
extern const uint32_t aSRC_Const_Buffer[Buffer_Size];
extern uint32_t aDST_Buffer[Buffer_Size];

#define DMA_INIT_CLK               RCC_AHBPeriph_DMA1
#define DMA_CHANNEL                     DMA1_Channel2
#define DMA_FLAG_TC                  DMA1_FLAG_TC2
#define ERROR                       RESET
#define TRUE                       SET
```

初始化函数如下：

```c
/***DMA初始化***/
void DMA_Init_Config(void)
{
    DMA_InitTypeDef DMA_InitStructure;

    //DMA外设时钟使能
    RCC_AHBPeriphClockCmd(DMA_INIT_CLK, ENABLE);
    //设置数据从哪来到哪去 传输方向
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)aSRC_Const_Buffer;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)aDST_Buffer;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;                          //从外设读
    //设置数据传输数目 源端数据宽度 目标数据宽度 地址增量模式
    DMA_InitStructure.DMA_BufferSize = Buffer_Size;                             //传输数目
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Enable;             //外设地址增量模式
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                     //存储器地址增量模式
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;     //源端宽度：全字
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;             //目标宽度：全字     
    //设置传输模式 优先级 M2M模式 
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                               //进行一次DMA传输
    DMA_InitStructure.DMA_Priority =  DMA_Priority_High;                        //高优先级
    DMA_InitStructure.DMA_M2M = DMA_M2M_Enable;                                 //M2M模式
    //完成DMA初始化配置
    DMA_Init(DMA_CHANNEL, &DMA_InitStructure);

    //清空传输完成标志位
    DMA_ClearFlag(DMA_FLAG_TC);

    //DMA外设使能
    DMA_Cmd(DMA_CHANNEL, ENABLE);
}
```

注：

1.

```c
DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)aSRC_Const_Buffer;
DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)aDST_Buffer;
```

此处的地址来源为以下代码：

```c
/***定义数据的来源和去处***/

/* 定义aSRC_Const_Buffer数组为DMA传输的数据源
 * const关键字将aSRC_Const_Buffer数组变量定义为常量类型
 * 表示数组存放在内部的FLASH中
 */
const uint32_t aSRC_Const_Buffer[Buffer_Size] = 
{
    0x00000000,0x11111111,0x22222222,0x33333333,0x44444444,
    0x55555555,0x66666666,0x77777777,0x88888888,0x99999999
};

/* 定义DMA传输数据的目标存储器
 *存储在内部的SRAM中
 */
uint32_t aDST_Buffer[Buffer_Size];
```

`const`关键字 将`uint32_t aSRC_Const_Buffer[Buffer_Size]`定义为常量类型，存储在内部的FLASH中。

`uint32_t aDST_Buffer[Buffer_Size]`存储在内部的SRAM中，表示此DMA传输工作在存储器到存储器之间的数据传输模式。

2.

```c
//清空传输完成标志位
DMA_ClearFlag(DMA_FLAG_TC);
```

此处代码是清零DMA传输完成标志位，以便可以通过检测该标志位是否被置 ‘1’ 来判断DMA传输是否完成。

### 3.1 传输数据比较函数

        由于DMA工作在MtoM模式下，我们无法直观地判断数据传输的正确性，因此，我们需要编写一个传输数据比较函数来判断每个数据传输的正确性。相关代码如下：

```c
FlagStatus BufferCmp(const uint32_t* SRC_Const_Buffer, uint32_t* DST_Buffer, uint32_t number)
{
    uint32_t i;
    for(i=0;i<number;i++)    //比较每个数据的传输正确性
    {
        if( *(SRC_Const_Buffer) != *(DST_Buffer) )
            return RESET;
        SRC_Const_Buffer++;
        DST_Buffer++;
    }
    return SET;
}
```

同时，在main函数中我们可以通过LED的不同状态来反应DMA传输的不同阶段，如开始、正在传输和传输正确或错误。相关代码如下：

```c
int main(void)
{
    uint32_t i;
    FlagStatus status = RESET;

    GPIO_Init_Config();

    //灯亮表示传输未完成
    LED(ON);

    //开始传输
    DMA_Init_Config();

    //等待传输完成
    while(DMA_GetFlagStatus(DMA_FLAG_TC) == RESET);

    status = BufferCmp(aSRC_Const_Buffer, aDST_Buffer, Buffer_Size);

    if(status == ERROR)
    {
        LED(OFF);         //灯灭表示传输失败
    }
    else
    {
        for(i=0;i<0x0f;i++)   //灯交替闪烁一段时间表示传输成功
        {
            LED(ON);
            delay(0XFFFFF);
            LED(OFF);
            delay(0XFFFFF);
        }
    }
}
```

## 3.DMA存储器到外设模式下传输数据（MtoP）

## （举例为串口）

### 3.1 DMA和UASART初始化配置

        由于我们用到了DMA和UASRT两个外设，所以要对二者进行初始化配置。对于USART可参考之前串口部分初始化代码；对于DMA可以参考是上面MtoM模式下相关配置，只需修改部分参数即可。该部分宏定义和函数定义代码如下：

宏定义如下：

```c
//传输数目
#define SendBuff_Size                   10000

//串口GPIO宏定义
#define USART_GPIO_INIT_CLK             RCC_APB2Periph_GPIOA
#define USART_TX_INIT_PORT              GPIOA
#define USART_TX_INIT_PIN               GPIO_Pin_9
#define USART_RX_INIT_PORT              GPIOA
#define USART_RX_INIT_PIN               GPIO_Pin_10
//串口初始化宏定义
#define USART_INIT_CLK                  RCC_APB2Periph_USART1
#define DEBUG_USARTx                    USART1
#define DEBUG_USARTx_BAUDRATE           115200
#define DEBUG_USARTx_IRQn               USART1_IRQn

//DMA初始化宏定义
#define USART_TX_DMA_INIT_CLK           RCC_AHBPeriph_DMA1
#define USART_TX_DMA_CHANNEL            DMA1_Channel4
#define USART_TX_DMA_FLAG_TC            DMA1_FLAG_TC4

//串口DMA宏定义
#define USARTx_DMA                      USART1
#define USARTx_DMAReq                   USART_DMAReq_Tx
//串口数据寄存器地址
#define USARTx_DR_Buffer                (USART1_BASE+0x04)
```

初始化函数如下：

```c
uint8_t Send_Buffer[SendBuff_Size] = {0,1,2,3,4,5,6,7,8,9};

/***串口初始化***/
void Usart_Init_Comfig(void)
{
    GPIO_InitTypeDef    GPIO_InitStructure;
    USART_InitTypeDef   UASRT_InitStructure;

    //使能串口GPIO时钟 以及 串口时钟
    RCC_APB2PeriphClockCmd(USART_GPIO_INIT_CLK|USART_INIT_CLK, ENABLE);

    /*串口GPIO初始化*/
    //TX的GPIO初始化：推挽复用模式
    GPIO_InitStructure.GPIO_Pin = USART_TX_INIT_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(USART_TX_INIT_PORT, &GPIO_InitStructure);
    //RX的GPIO初始化：浮空输入模式
    GPIO_InitStructure.GPIO_Pin = USART_RX_INIT_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(USART_RX_INIT_PORT, &GPIO_InitStructure);

    /*USART1串口初始化*/
    //设置波特率：115200
    UASRT_InitStructure.USART_BaudRate = DEBUG_USARTx_BAUDRATE;
    //设置帧字长：8位
    UASRT_InitStructure.USART_WordLength = USART_WordLength_8b;
    //设置停止位：1位
    UASRT_InitStructure.USART_StopBits = USART_StopBits_1;
    //设置校验位：无
    UASRT_InitStructure.USART_Parity = USART_Parity_No;
    //设置串口模式；输入输出
    UASRT_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    //设置硬件流控制
    UASRT_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    //完成串口初始化配置
    USART_Init(DEBUG_USARTx, &UASRT_InitStructure);

    /*使能串口*/
    USART_Cmd(DEBUG_USARTx, ENABLE);
}

/***DMA初始化***/
void DMA_Init_Config(void)
{
    DMA_InitTypeDef DMA_InitStructure;

    //DMA外设时钟使能
    RCC_AHBPeriphClockCmd(USART_TX_DMA_INIT_CLK, ENABLE);

    //设置数据从哪来到哪去 传输方向
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)USARTx_DR_Buffer;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)Send_Buffer;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;                          //外设为目标
    //设置数据传输数目 源端数据宽度 目标数据宽度 地址增量模式
    DMA_InitStructure.DMA_BufferSize = SendBuff_Size;                           //传输数目
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;            //外设地址不增 DR寄存器
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                     //存储器地址增量模式
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;     //源端宽度：一字节
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;             //目标宽度：一字节    
    //设置传输模式 优先级 M2M模式 
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                               //进行一次DMA传输
    DMA_InitStructure.DMA_Priority =  DMA_Priority_High;                        //高优先级
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                                //失能M2M模式
    //完成DMA初始化配置
    DMA_Init(USART_TX_DMA_CHANNEL, &DMA_InitStructure);

    //清空传输完成标志位
    DMA_ClearFlag(USART_TX_DMA_FLAG_TC);

    //DMA外设使能
    DMA_Cmd(USART_TX_DMA_CHANNEL, ENABLE);
}
```

注：

1.`uint8_t Send_Buffer[SendBuff_Size] = {0,1,2,3,4,5,6,7,8,9};`存储在SRAM中，为需要通过DMA方式发送到USART1的数据。

2.传输数目为10000：`#define SendBuff_Size   10000`

### 3.2 串口传输&LED闪烁

        在main函数中，我们可以实现一边串口打印数据，一边闪烁LED灯，好像一个人在同一时间做两件不同的事一样。相关代码如下：

```c
int main(void)
{
    #if 0

    GPIO_Init_Config();
    while(1)
    {
        LED(ON);
        //GPIO_SetBits(GPIO_INIT_PORT, GPIO_INIT_PIN);
        delay(0XFFFFF);
        LED(OFF);
        //GPIO_ResetBits(GPIO_INIT_PORT, GPIO_INIT_PIN);
        delay(0XFFFFF);
    }

    #elif 1

    GPIO_Init_Config();
    Usart_Init_Comfig();
    DMA_Init_Config();

    USART_DMACmd(USARTx_DMA, USARTx_DMAReq, ENABLE);

    while(1)
    {
        LED_TOGGLE;
        delay(0xFFfFF);
    }

    #endif
}
```

TNE END...
