# STM32之中断篇

## 1.NVIC简述

         说到中断便离不开NVIC，它的全称为“Nested Vectored Interrupt Controller”,即：嵌套向量中断控制器。通俗地讲其作用就是控制中断（或称为“异常”）过程。NVIC根据中断的优先级来控制内核和外设的所有中断，因此每个中断都会首先设置其中断优先级。

        在NVIC中有一个专门的寄存器：中断优先级寄存器 NVIC_IPRx 用来配置外部中断的优先级。IPR 宽度为 8bit，原则上每个外部中断可配置的优先级为 0~255，数值越小，优先级越高。但是绝大多数 CM3芯片都会精简设计，以致实际上支持的优先级数减少，在 F103中，只使用了高 4bit来配置外部中断的优先级。而这 4bits 又被分组为抢占优先级和子优先级。

优先原则为：如果有多个中断同时响应，抢占优先级高的就会 抢占 抢占优先级低的优先得到执行，如果抢占优先级相同，就比较子优先级。如果抢占优先级和子优先级都相同的话，就比较他们的硬件中断编号，编号越小，优先级越高。

具体优先级分组为：

```c
  *     @arg NVIC_PriorityGroup_0: 0 位 for 抢占优先级
  *                                4 位 for 子优先级
  *     @arg NVIC_PriorityGroup_1: 1 位 for 抢占优先级
  *                                3 位 for 子优先级
  *     @arg NVIC_PriorityGroup_2: 2 位 for 抢占优先级
  *                                2 位 for 子优先级
  *     @arg NVIC_PriorityGroup_3: 3 b位 for 抢占优先级
  *                                1 位 for 子优先级
  *     @arg NVIC_PriorityGroup_4: 4 位 for 抢占优先级
  *                                0 位 for 子优先级
```

而优先级组选择可以通过调用一个专门的库函数来实现，在misc.h中可以找到这个函数，该库函数声明如下：

`void NVIC_PriorityGroupConfig(uint32_t NVIC_PriorityGroup);`

例如：选择优先级分组二`NVIC_PriorityGroupConfig( NVIC_PriorityGroup_1 );`，则优先级寄存器中抢占优先级可以在[1:0]中选择，如选择抢占优先级为0；子优先级可以在[7:0]中选择，如选择子优先级为1。优先级分组真值表如下（主优先级即抢占优先级）：

![](C:\Users\30187\Desktop\优先级分组真值表.png)

在库函数编程中，首先选择优先级分组，再通过一个NVIC初始化库函数来配置NVIC。在misc.h中可以找到这个函数，该库函数声明如下：

`void NVIC_Init(NVIC_InitTypeDef* NVIC_InitStruct);`

        可以看到我们主要是要对NVIC__InitTypeDef这类结构体类型中的成员写入相应值来配置NVIC的相关寄存器。该结构体类型定义如下：

```c
typedef struct

{
 uint8_t NVIC_IRQChannel;//指定中断通道。在“stm32f10x.h“中查找。
                            //其中含有所有中断的编号，在里面查找所需要的
                             //中断编号赋给此成员变量。

 uint8_t NVIC_IRQChannelPreemptionPriority;//抢占优先级

 uint8_t NVIC_IRQChannelSubPriority;//子优先级

 FunctionalState NVIC_IRQChannelCmd;//中断使能/失能位

} NVIC_InitTypeDef;
```

例如：配置一个外部中断的NVIC函数如下：

```c
 void EXTI_NVIC_Init_Config(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);//优先级分组
    NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;//中断通道选择
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;//抢占优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;//子优先级
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//中断使能
    NVIC_Init(&NVIC_InitStructure);//实现中断NVIC配置
}
```

## 2.外部中断

        外部中断由EXTI（External Interrupt /Event Controller）即外部 中断/事件 控制器控制。对于互联型产品，外部中断/事件控制器由20个产生事件/中断请求的边沿检测器组成，对于其它产品，则有19个能产生事件/中断请求的边沿检测器。每个输入线可以独立地配置输入类型(脉冲或挂起)和对应的触发事件(上升沿或下降沿或者双边沿都触发)。每个输入线都可以独立地被屏蔽。挂起寄存器保持着状态线的中断请求。

### 2.1 外部中断控制器框图如下（图中红色虚线即为中断产生的流程线）：

![](C:\Users\30187\Desktop\外部中断控制器框图.png)

编写外部中断程序时需要清楚的是：

①输入线选择：EXTI有20个输入中断线，分别为EXTI[19:0]。其中GPIO输入输出端口通过EXTI[15:0]连接到16个外部中断/事件线上，比如PA0-PG0均连接到EXTI0上，PA1-PG1均连接到EXTI1上，依次类推。库函数编程时，我们可以在"stm32f10x_gpio.h"头文件中找到

`void GPIO_EXTILineConfig(uint8_t GPIO_PortSource,uint8_t GPIO_PinSource);`

来选择我们所需要的GPIO端口作为外部中断的输入线。

②外部中断初始化配置寄存器：库函数编程时，我们可以在"stm32f10x_exti.h"头文件中找到EXTI初始化函数声明：

`void EXTI_Init(EXTI_InitTypeDef* EXTI_InitStruct);`

可以看到该函数也是通过配置EXTI_InitTypeDef这类结构体类型的成员变量来初始化EXTI，我们只需往结构体成员中写入相应值，配置EXTI的寄存器使该外部中断工作在我们所需要的模式下。该结构体类型定义如下：

```c
typedef struct
{
  uint32_t EXTI_Line;//使能/失能外部中断输入线。
                        //可以在该文件下找到所需外部中断线填入。
                         //注：此处作用是确定初始化时要写入的寄存器是EXTIx的寄存器
                          x=0~19
  EXTIMode_TypeDef EXTI_Mode;//EXTI工作模式：中断/事件   
  EXTITrigger_TypeDef EXTI_Trigger;//EXTI触发方式：上升沿/下降沿/上升沿和下降沿
  FunctionalState EXTI_LineCmd;//使能/失能EXTI
} EXTI_InitTypeDef;
```

配置好结构体中成员变量值后再调用上述初始化函数即可完成EXTI工作模式的初始化。

③NVIC初始化配置：即上述配置外部中断的NVIC函数以设置该外部中断的优先级和使能总中断。

例如：配置一个“与按键IO端口相连接的输入线的EXTI初始化函数”为：

exti.h文件：

```c
#ifndef __EXTI_H__
#define __EXTI_H__

#include "stm32f10x.h"

#define EXTI_KEY_INIT_CLK             RCC_APB2Periph_GPIOC
#define EXTI_INIT_CLK                 RCC_APB2Periph_AFIO
#define EXTI_KEY_INIT_PORT            GPIOC
#define EXTI_KEY_INIT_PIN             GPIO_Pin_13
#define EXTI_LINE_GOIO_PORT           GPIO_PortSourceGPIOC
#define EXTI_LINE_GOIO_PIN            GPIO_PinSource13

void EXTI_Init_Config(void);

#endif 
```

exti.c文件：

```c
#include "exti.h"

static void EXTI_NVIC_Init_Config(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);   //优先级分组
    NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;//中断通道选择
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;//抢占优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;//子优先级
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//中断使能
    NVIC_Init(&NVIC_InitStructure);//实现中断NVIC配置
}

void EXTI_Init_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;

    //配置中断优先级
    EXTI_NVIC_Init_Config();

    //初始化GPIO_KEY
    RCC_APB2PeriphClockCmd(EXTI_KEY_INIT_CLK, ENABLE);
    GPIO_InitStructure.GPIO_Pin = EXTI_KEY_INIT_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(EXTI_KEY_INIT_PORT, &GPIO_InitStructure);

    //初始化EXTI
    RCC_APB2PeriphClockCmd(EXTI_INIT_CLK, ENABLE);
    GPIO_EXTILineConfig(EXTI_LINE_GOIO_PORT, EXTI_LINE_GOIO_PIN);
    EXTI_InitStructure.EXTI_Line = EXTI_Line13;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
}
```

注：① 因为使用到GPIO外设，故要首先使能GPIO外设时钟以及配置GPIO工作模式。

        ② EXTI作为一个外设也需打开对应时钟。查询系统结构图可知EXTI外设挂在在APB2总线上且在库函数中是使能 RCC_APB2PeripAFIO （复用IO）时钟来使能EXTI外设时钟。

        ③根据原理图可知，按键未按下时IO口为高电平，因此检测按键被按下来作为中断请求时应设置外部中断为“检测下降沿”作为中断请求。

最后，在stm32f10x.c文件下编写中断服务函数来实现我们所需要的功能。比如当按键被按下后让LED灯翻转一次。

```c
void EXTI15_10_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line13) == SET) //调用中断库函数检测是否为所需中断
    {
        LED_TOGGLE; //实现功能，如led翻转
    }
    EXTI_ClearITPendingBit(EXTI_Line13); //清除该中断标志位
}
//注：函数名应为stm32f10x.h文件内对应中断的指定函数名称
```

        main函数中调用GPIO和EXTI初始化函数，然后while(1)无限循环以等待按键被按下进入中断实现LED翻转即可。

## 3. SysTick系统定时器

        SysTick定时器被捆绑在NVIC中，是存在于Cortex-M3处理器内部的一个简单定时器，用于产生SYSTICK异常（异常号：15）。其作用是提供操作系统所需的周期性的滴答中断以维持系统的节律。它也可用于测量时间。

        有4个寄存器控制SysTick定时器，分别为：

SysTick控制及状态寄存器：计数标志位、是时钟来源位、是否产生异常请求位、使能/失能位

SysTick重装载数值寄存器：顾名思义，当倒数到零时，被重装载的值，也即为计数初值。SysTick当前数值寄存器：读取时返回当前倒计数的值，写它则使之清零，同时还会清除在 SysTick 控制及状态寄存器中的COUNTFLAG 标志

SysTick校准数值寄存器：决定外部参考时钟是否可用、校准值是否是准确的10ms以及10ms的时间内倒计数的格数。    

        以上寄存器的配置由SysTick配置函数来完成，该函数在库文件“core.cm3.h”可以找到其定义，该函数如下：

```c
static __INLINE uint32_t SysTick_Config(uint32_t ticks)
{ 
  if (ticks > SysTick_LOAD_RELOAD_Msk)  return (1); //检查计时初值的合法性
                                                        //最高24位计数初值         

  SysTick->LOAD  = (ticks & SysTick_LOAD_RELOAD_Msk) - 1;  //装载计数初值 
  NVIC_SetPriority (SysTick_IRQn,(1<<__NVIC_PRIO_BITS) - 1);//设置中断优先级 
  SysTick->VAL   = 0; //归零当前倒计数的值                                         
  SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk |   //内核时钟作为时钟源
                   SysTick_CTRL_TICKINT_Msk   |   //允许产生异常请求
                   SysTick_CTRL_ENABLE_Msk;       //使能SysTick定时器             
  return (0);   //配置成功！
}
```

应用Systick定时器实现us延时：

```c
void SysTick_Delay_us(uint32_t us)
{
    uint32_t i;

    SysTick_Config(72);     //计时周期 T = 1/72MHz 乘以 72 = 1us

    for(i=0;i<us;i++)
    {
        while( !(SysTick->CTRL) & (1<<16) );
    }
    SysTick->CTRL &= ~(SysTick_CTRL_ENABLE_Msk);
}
```

应用Systick定时器实现ms延时：

```c
void SysTick_Delay_ms(uint32_t ms)
{
    uint32_t i;

    SysTick_Config(72000);     //计时周期 T = 1/72MHz 乘以 72000 = 1ms

    for(i=0;i<ms;i++)
    {
        while( !(SysTick->CTRL) & (1<<16) );
    }
    SysTick->CTRL &= ~(SysTick_CTRL_ENABLE_Msk);
}
```

THE END...
