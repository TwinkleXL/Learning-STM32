# STM32之usart篇

## 1.usart简述

        通用同步异步收发器（usart）是一种全双工、同步/异步、采用串行数据格式的通信方式。其接口通过接收数据输入（RX）和发送数据输出 （TX）分别与其他设备的TX和RX连接在一起进行通信。

## 2.串口初始化

        使用任何一个外设均要首先对该外设进行初始化。在使用库函数编程时，我们均可以在所需外设的头文件中找到外设初始化配置函数声明，在源文件中找到对应函数的定义。因此我们可以在库文件“stm32f10x__usart.h”中找到usart初始化函数，该函数如下：

```c
void USART_Init(USART_TypeDef* USARTx, USART_InitTypeDef* USART_InitStruct);
```

可以看到我们需要指定所用串口和设定串口初始化结构体变量的各成员值来完成串口配置。

## 3.发送数据

### 3.1 发送数据初始化

相关宏定义如下：包含GPIO和串口时钟宏定义、TX/RX端口和引脚宏定义、串口初始化配置相关宏定义和串口中断宏定义。

```c
//串口GPIO宏定义
#define USART_GPIO_INIT_CLK             RCC_APB2Periph_GPIOA
#define USART_TX_INIT_PORT              GPIOA
#define USART_TX_INIT_PIN               GPIO_Pin_9
#define USART_RX_INIT_PORT              GPIOA
#define USART_RX_INIT_PIN               GPIO_Pin_10
//串口宏定义
#define USART_INIT_CLK                  RCC_APB2Periph_USART1
#define DEBUG_USARTx                    USART1
#define DEBUG_USARTx_BAUDRATE           115200
//串口中断源宏定义
#define DEBUG_USARTx_IRQn               USART1_IRQn
```

串口发送初始化函数如下：

```c
void Usart_Init_Comfig(void)
{
    GPIO_InitTypeDef    GPIO_InitStructure;
    USART_InitTypeDef   UASRT_InitStructure;

    //使能串口GPIO时钟
    RCC_APB2PeriphClockCmd(USART_GPIO_INIT_CLK, ENABLE);

    //串口时钟使能
    RCC_APB2PeriphClockCmd(USART_INIT_CLK, ENABLE);

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
    //设置串口模式；输出
    UASRT_InitStructure.USART_Mode = USART_Mode_Tx;
    //设置硬件流控制
    UASRT_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    //完成串口初始化配置
    USART_Init(DEBUG_USARTx, &UASRT_InitStructure);
    /*使能串口*/
    USART_Cmd(DEBUG_USARTx, ENABLE);
}
```

### 3.2 发送数据相关代码

```c
/*发送一个字节*/
void Usart_SendByte(USART_TypeDef* pUSARTx,uint8_t data)
{
    USART_SendData(pUSARTx, data);
    while( USART_GetFlagStatus(pUSARTx, USART_FLAG_TC) == RESET );
}

/*发送两个字节*/
void Usart_SendHalfWord(USART_TypeDef* pUSARTx,uint16_t data)
{
    uint8_t temp_h,temp_l;

    temp_h = (data & 0xFF00) >> 8;    //高8位
    temp_l = data & 0x00FF;           //低8位

    USART_SendData(pUSARTx, temp_h);
    while( USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET );

    USART_SendData(pUSARTx, temp_l);
    while( USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET );
}

/*发送一个8位数据的数组*/
void Usart_SendArray(USART_TypeDef* pUSARTx,uint8_t *array,uint8_t num)
{
    uint8_t i;

    for(i=0;i<num;i++)
    {
        USART_SendData(pUSARTx, array[i]);
        while( USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET );  
            //经测试 将该代码放置在循环内 \
                   //且FLAG为USART_FLAG_TXE时 串口输出才正常
    }
}

/*发送一个字符串*/
void Usart_SendStr(USART_TypeDef* pUSARTx,uint8_t *str)
{
    uint8_t i=0;

    while( *(str+i) != '\0')
    {
        USART_SendData(pUSARTx, *(str+i));
        while( USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET );  
            //经测试 将该代码放置在循环内 \
                //且FLAG为USART_FLAG_TXE时 串口输出才正常
        i++;
    }
}
```

在这些函数中主要是调用以下两个串口库函数：

```c
/*串口发送数据函数*/
void USART_SendData(USART_TypeDef* USARTx, uint16_t Data);
/*获取串口标志位函数*/
FlagStatus USART_GetFlagStatus(USART_TypeDef* USARTx, uint16_t USART_FLAG);
```

## 4.接收数据

        串口接收数据与串口发送数据的不同之处在于接收数据时需要用到串口中断，因此需要配置串口的中断优先级即配置NVIC，并使能串口中断，同时编写串口中断服务程序。

### 4.1 串口接收数据初始化

相关宏定义如下：

```c
//串口GPIO宏定义
#define USART_GPIO_INIT_CLK             RCC_APB2Periph_GPIOA
#define USART_TX_INIT_PORT              GPIOA
#define USART_TX_INIT_PIN               GPIO_Pin_9
#define USART_RX_INIT_PORT              GPIOA
#define USART_RX_INIT_PIN               GPIO_Pin_10
//串口宏定义
#define USART_INIT_CLK                  RCC_APB2Periph_USART1
#define DEBUG_USARTx                    USART1
#define DEBUG_USARTx_BAUDRATE           115200
//串口中断源宏定义
#define DEBUG_USARTx_IRQn               USART1_IRQn
//串口中断函数名宏定义
#define DEBUG_USARTx_IRQHandler         USART1_IRQHandler
```

串口接收初始化函数如下：

```c
static void UASART_NVIC_Init_Config(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    //嵌套向量中断控制器组选择
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    //配置UASRT为中断源
    NVIC_InitStructure.NVIC_IRQChannel = DEBUG_USARTx_IRQn;
    //抢占优先级
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  
    //子优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  
    //使能中断    
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;     
    //初始化配置NVIC
    NVIC_Init(&NVIC_InitStructure);
}

void Usart_Init_Comfig(void)
{
    GPIO_InitTypeDef    GPIO_InitStructure;
    USART_InitTypeDef   UASRT_InitStructure;

    //使能串口GPIO时钟
    RCC_APB2PeriphClockCmd(USART_GPIO_INIT_CLK, ENABLE);

    //串口时钟使能
    RCC_APB2PeriphClockCmd(USART_INIT_CLK, ENABLE);

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

    /*串口中断优先级配置*/
    UASART_NVIC_Init_Config();

    /*串口接收中断初始化*/
    USART_ITConfig(DEBUG_USARTx, USART_IT_RXNE, ENABLE);

    /*使能串口*/
    USART_Cmd(DEBUG_USARTx, ENABLE);
}
```

与之前的串口发送初始化函数不同的是：串口接收初始化函数增加了如下代码行用来配置串口接收中断：

```c
/*串口中断优先级配置*/
UASART_NVIC_Init_Config();

/*串口接收中断初始化*/
USART_ITConfig(DEBUG_USARTx, USART_IT_RXNE, ENABLE);
```

### 4.2 串口接收中断服务函数

该部分代码主要包含：

①获取串口中断标志位；

②实现所需要的功能；

③清楚串口中断挂起位。

```c
void DEBUG_USARTx_IRQHandler(void)
{
    uint16_t temp;

    if( USART_GetITStatus(DEBUG_USARTx, USART_IT_RXNE) != RESET )
    {
        temp = USART_ReceiveData(DEBUG_USARTx);
        USART_SendData(DEBUG_USARTx, temp);
        while( USART_GetFlagStatus(DEBUG_USARTx, USART_FLAG_TXE) == RESET );
        if(temp == '1')
            LED(ON)
        else
            LED(OFF)
    }
    USART_ClearITPendingBit(DEBUG_USARTx, USART_IT_RXNE);
}
```

## 5.学习串口遇到的问题：

1.时钟使能：串口GPIO时钟和串口时钟分开使能时，串口输出总是先输出一个“FF” 再输出业务代码中的要输出的内容。解决办法：将串口GPIO时钟和串口时钟用”|“或运算一次性使能。

2.FLAG标志位检测：

  2.1 发送一个字节，检测

`while( USART_GetFlagStatus(pUSARTx,USART_FLAG_TC) == RESET );`

和检测

`while( USART_GetFlagStatus(pUSARTx,USART_FLAG_TXE) == RESET );`无影响。

  2.2 发送两个字节，高字节发送时检测

`while(USART_GetFlagStatus(pUSARTx,USART_FLAG_TXE)==RESET);`

**且**低字节发送时检测

`while( USART_GetFlagStatus(pUSARTx, USART_FLAG_TC) == RESET ); `

**或**低字节发送时检测

`while( USART_GetFlagStatus(pUSARTx, USART_FLAG_TC) == RESET );`

时串口显示正常。其余检测情况不正常。

  2.3 发送数组和字符串时：检测语句应在**循环内**，且检测**USART_FLAG_TXE**位，串口发送才正常。

*总结为：检测时最好检测USART_FLAG_TXE位电平高低。且要注意检测代码的位置。*（为什么我没弄明白，实践出来的结果hhh）

3  Printf函数和scanf函数重定向问题：

```c
/*使用printf进行串口打印*/
//重定向C库函数printf到串口，重定向后可使用printf、putchar函数
int fputc(int ch,FILE *f)
{
    //发送一个字节到串口
    USART_SendData(DEBUG_USARTx, (uint8_t)ch);

    //等待发送完毕
    while( USART_GetFlagStatus(DEBUG_USARTx, USART_FLAG_TXE) == RESET );

    return (ch);
}

/*使用scanf进行串口写入数据*/
//重定向C库函数scanf到串口，重定向后可以使用scanf、getchar函数
int fgetc(FILE *f)
{
    while( USART_GetFlagStatus(DEBUG_USARTx, USART_FLAG_RXNE) == RESET );

    return (int)USART_ReceiveData(DEBUG_USARTx);
}
```

自认为上述函数没什么问题，可是printf和scanf函数还是没办法使用。后续还要琢磨琢磨~~~
