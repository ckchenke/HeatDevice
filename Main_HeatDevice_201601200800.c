/*
 * File:   Main_HeatDevice.c
 * Author: lenovo
 *
 * Created on 2015年12月31日, 下午3:41
 */          
#include <xc.h>
#include <pic.h>
#include <pic16f1503.h>
// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

// CONFIG1
#pragma config FOSC = INTOSC    // Oscillator Selection Bits (INTOSC oscillator: I/O function on CLKIN pin)
#pragma config WDTE = SWDTEN    // Watchdog Timer Enable (WDT controlled by the SWDTEN bit in the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable (PWRT disabled)
#pragma config MCLRE = OFF      // MCLR Pin Function Select (MCLR/VPP pin function is digital input)
#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable (Brown-out Reset enabled)
#pragma config CLKOUTEN = OFF   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)

// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
#pragma config LPBOR = OFF      // Low-Power Brown Out Reset (Low-Power BOR is disabled)
#pragma config LVP = OFF        // Low-Voltage Programming Enable (High-voltage on MCLR/VPP must be used for programming)

/* 74HC595_SOP */
#define SH_CP_HIGH  (PORTCbits.RC3 = 1)
#define SH_CP_LOW   (PORTCbits.RC3 = 0)
#define ST_CP_HIGH  (PORTCbits.RC4 = 1)
#define ST_CP_LOW   (PORTCbits.RC4 = 0)
#define DS_HIGH     (PORTCbits.RC5 = 1)
#define DS_LOW      (PORTCbits.RC5 = 0)

/* 按键阵列标记，用于记录哪个按键被按下了 */
typedef union
{
    struct {
    unsigned char ucTimer      :1;
    unsigned char ucSwitch     :1;
    unsigned char ucTemSub     :1;
    unsigned char ucMode       :1;
    unsigned char ucTemAdd     :1;
    unsigned char ucLock       :1;
    };
    struct {
    unsigned char ucKeyBoardPressFlag;        
    };
} KeyBoardPress_t;

unsigned char LedDisplayMap[17] = {0xe7,0x82,0xd5,0xd3,0xb2,0x73,0x77,0xc2,0xf7,0xf3,0xfe,0x3f,0x65,0x9f,0x7d,0x7c,0x10};
unsigned short g_uiLeftLedDisplayStatuValue = 0;    /* 十进制位的数码管的状态值记录 */
unsigned short g_uiRightLedDisplayStatuValue = 0;   /* 个位的数码管的状态值记录 */
unsigned short g_uiFunLedDisplayStatuValue = 0;     /* 功能状态灯的状态值记录 */
unsigned char g_ucLedDisplayConFlag =0;             /* 灯轮询点亮标记，0十进制数码管，1个位数码管，2功能状态灯 */
unsigned char g_ucInfraRedFlag = 0;                 /* 红外信号触发标记 */
unsigned short g_uiInfraRedCounter = 0;             /* 红外信号时长相对于定时器上报的次数，用于估算一个红外信号的时长 */
unsigned char g_ucStartFlagValid = 0;               /* 前导码有效标记，用作判断条件，只有在前导码有效的情况下，后续的数据才有意义 */
unsigned char g_ucDataIndex = 31;                   /* 红外有效数据下标 */
unsigned int g_InfraData = 0;                       /* 存储红外数据 */
unsigned char g_ucTimeCounter = 0;                  /* 去抖消除需要延时20ms左右，这个变量用来记录timer定时器中实际进入的次数，以此来估算延时时间 */
unsigned char g_ucPowerKeyPress = 0;                /* 电源键是否按下，用于标识系统是否工作 */
unsigned char g_ucModeChoosedKeyPressCount = 0;     /* 用于记录模式选择按钮的按下次数，用于控制当前的工作模式 */
unsigned char g_ucInConstantTempe = 0;              /* 恒温模式标记 */
unsigned char g_ucCurrentTempValue = 0;             /* 用于记录当前恒温状态设置的温度值 */
unsigned char g_ucSetTimerValue = 0;                /* 定时时间设置值 */
unsigned char g_ucLedFlashFlag = 0;                 /* 数码管闪烁标记，当温度加减和定时时间改变时需要有闪烁现象 */
unsigned short g_usLedFlashCounter = 0;             /* 数码管闪烁定时计数，设置500ms闪烁一次 */
unsigned short g_usCountDownCounter = 0;            /* 1秒定时 */
//unsigned char g_ucCountDownFlag = 0;                /* 倒计时标记 */
unsigned char g_ucLedFlashCounterFlag = 0;          /* 用于记录数码管闪烁的标记 */
unsigned char g_ucTimerKeySwitchFlag = 0;           /* 定时按钮切换标记，当按下其他按键时需要将这个标记置上 */
unsigned char g_ucChildLockFunEnableFlag = 0;       /* 童锁功能开启标记，当这个标记置上时，其他按键功能关闭 */
unsigned char g_ucCountDownValue = 59;              /* 倒计时初始值 */
unsigned char g_ucPowerDownFlag = 0;                /* 电源键被按下时设置该标记，用来使能倒计时操作 */
unsigned char g_ucHighTempOpenFlag = 0;             /* 高温开启标记 */
unsigned short g_usHighTempOpenCount = 0;           /* 用于高温定时计数 */
unsigned char g_ucHighTempFirstChooseFlag = 1;      /* 当开电源首次被按下时，需要用5秒的延时，其他不需要 */
unsigned short g_usAdcCoverTimerCount = 0;          /* Adc转换定时器计数，每10发起一次adc读取操作，用于监控温度 */
unsigned short g_usTimerFuncCount = 0;              /* 定时功能计数，每10秒加一次 */
unsigned short g_ausSenserRValueMap[11] = {801, 786, 770, 754, 738, 722, 703, 687, 668, 651, 631};    /* 10-30都之间按10档来进行温度控制 */         
KeyBoardPress_t g_unPressFlag;                      /* 按键标记，用于记录哪个按键被按下 */

#define PICF1503_ADC_TEMP_OFFSET              (8)   /* 因为ADC采样的值会不停的浮动，需要增加一个 */

void PicF1503_SystemTime_Config(void)
{
    /* 配置为内部时钟源，且时钟频率为16兆 */
    OSCCONbits.IRCF = 0xF;
    OSCCONbits.SCS = 0x3;
}
void Pic16F1503_Adc_Init(void)
{
    /* 配置结果输出格式为右对齐，ADC输入时钟为系统时钟的4分频 */
    ADCON1bits.ADFM = 1;
    ADCON1bits.ADCS = 4;
    /* 配置基准电压 */
    ADCON1bits.ADPREF = 0;

    /* 选择ADC通道，并使能ADC功能 */
    ADCON0bits.CHS = 0;
    ADCON0bits.ADON = 1;
    
    /* 清除中断标记，使能ADC中断 */
    PIR1bits.ADIF = 0;
    PIE1bits.ADIE = 1;
    
}
void Pic16F1503_Gpio_Init(void)
{
#if 0
    OPTION_REG &= 0x7F; //清零OPTION_REG 寄存器的全局WPUEN 位，从而使能各个上拉功能
    /* 配置porta0,1,2为数字信号输入,弱上拉，中断触发方式为下降沿触发,清除中断标记 */
    TRISA0 = 1;
    ANSELAbits.ANSA0 = 0;
    WPUAbits.WPUA0 = 1;
    IOCAFbits.IOCAF0 = 0;
    IOCANbits.IOCAN0 = 1;
    TRISA1 = 1;
    ANSELAbits.ANSA1 = 0;
    WPUAbits.WPUA1 = 1;
    IOCAFbits.IOCAF1 = 0;
    IOCANbits.IOCAN1 = 1;
    TRISA2 = 1;
    ANSELAbits.ANSA2 = 0;
    WPUAbits.WPUA2 = 1;
    IOCAFbits.IOCAF2 = 0;
    IOCANbits.IOCAN2 = 1;
    /* 弱上拉，中断触发方式为下降沿触发，清除中断标记 */
    TRISA3 = 1;
    WPUAbits.WPUA3 = 1;
    IOCAFbits.IOCAF3 = 0;
    IOCANbits.IOCAN3 = 1;
    /* 配置porta4,5数字信号输出,置管脚为低电平 */
    TRISA4 = 0;
    ANSELAbits.ANSA4 = 0;
    TRISA5 = 0;
    PORTAbits.RA4 = 0;
    PORTAbits.RA5 = 0;
    
    /* 配置portc3,4,5数字信号输出 */
    TRISC3 = 0;
    ANSELCbits.ANSC3 = 0;
    TRISC4 = 0;
    TRISC5 = 0;
    /* 配置portc2为模拟信号输入 */
    TRISC2 = 1;
    ANSELCbits.ANSC2 = 1;
    
    /* 使能GPIO中断 */
    INTCONbits.IOCIE = 1;
#else
    OPTION_REG &= 0x7F; //清零OPTION_REG 寄存器的全局WPUEN 位，从而使能各个上拉功能
    /* 配置porta0,1,2,3为数字信号输入,弱上拉，中断触发方式为下降沿触发,清除中断标记,port4,5为数字信号输出，
     * 管脚默认初始化为低电平，portc3,4,5为数字信号输出，portc2为模拟信号输入 */
    TRISA = 0xf;
    ANSELA = 0x0;
    WPUA |= 0xf;
    IOCAF &= 0xf0;
    IOCAN |= 0xf;
    PORTA &= 0xcf;
    TRISC &= 0xc7;
    TRISC |= 0x4;
    ANSELC &= 0xf7;
    ANSELC |= 0x4;
    
    /* 使能GPIO中断 */
    INTCONbits.IOCIE = 1;
#endif
    
    return;
}

/* 初始化i2c总线 */
void Pic16F1503_I2c_Init(void)
{
    /* 配置SCL和SDA为输入模式 */
    TRISC0 = 1;
    TRISC0 = 1;
    
    /* 配置工作频率,配置成50K，工作在主模式 */
    SSP1CON1bits.SSPM =0x8;
    SSP1ADDbits.ADD = 0x50;
    
    /* 开启I2C通信 */
    SSP1CON1bits.SSPEN = 1;
    
     return;   
}

/* 发送数据 */
void Pic16F1503_I2c_Send(unsigned char ucDeviceAddr, unsigned char ucSendData)
{
    /* 发送一个起始信号 */
    SSP1CON2bits.SEN = 1;//Start condition
    while(0 == PIR1bits.SSP1IF);//waiting for Start condition completed.
    PIR1bits.SSP1IF = 0;
    
    /* 发送的第一个字节包含7位的设备地址和一位的写标志 */
    ucDeviceAddr = (ucDeviceAddr << 1) & 0xFE;
    SSP1BUF = ucDeviceAddr;//Device Address
    while(0 == PIR1bits.SSP1IF);
    PIR1bits.SSP1IF = 0;
    // ~ACK
    
    SSP1BUF = ucSendData;
    while(0 == PIR1bits.SSP1IF);
    PIR1bits.SSP1IF = 0;
   //  ~ACK
    
    SSP1CON2bits.PEN = 1;//Stop condition
    while(0 == PIR1bits.SSP1IF);
    PIR1bits.SSP1IF = 0;
   //  ~ACK
}

/* 接收数据 */
void Pic16F1503_I2c_Recive(unsigned char ucDeviceAddr, unsigned char *pucReciveData)
{
    /* 发送一个起始信号 */
    SSP1CON2bits.SEN = 1;//Start condition
    while(0 == PIR1bits.SSP1IF);//waiting for Start condition completed.
    PIR1bits.SSP1IF = 0;
    
    /* 发送的第一个字节包含7位的设备地址和一位的读标志 */
    ucDeviceAddr = (ucDeviceAddr << 1) | 0x1;
    SSP1BUF = ucDeviceAddr;//Device Address
    while(0 == PIR1bits.SSP1IF);
    PIR1bits.SSP1IF = 0;
    // ~ACK
    
    SSP1CON2bits.RCEN = 1;
    
    while(0 == PIR1bits.SSP1IF);
    PIR1bits.SSP1IF = 0;
    // ~ACK
    
    *pucReciveData = SSP1BUF;
    
    SSP1CON2bits.ACKEN = 1;
    SSP1CON2bits.ACKDT = 0;
    
    while(0 == PIR1bits.SSP1IF);
    PIR1bits.SSP1IF = 0;
    
    SSP1CON2bits.PEN = 1;//Stop condition
    while(0 == PIR1bits.SSP1IF);
    PIR1bits.SSP1IF = 0;
   //  ~ACK
    
}

void Pic16F1503_Timer1_Init(void)
{
    /* 配置时钟为16M */
    T1CONbits.TMR1CS = 1;    
    T1CONbits.T1CKPS = 0;
    /* 配置时钟同步，打开定时器Timer1 */
   // T1CONbits.nT1SYNC = 0;
    T1CONbits.TMR1ON = 1;
    
    /* 配置定时器1时间为0.25MS上报一次 */
    TMR1Hbits.TMR1H = 0xF0;
    TMR1Lbits.TMR1L = 0x5F;
    
    /* 清除中断标记，使能定时器中断Timer1 */
    PIR1bits.TMR1IF = 0;
    PIE1bits.TMR1IE = 1;
    
    return;

}

void Pic16F1503_Timer2_Init(void)
{
    /* 输入时钟为4分频的系统时钟，对输入时钟配置4分频,定时器时间为255us */
    PR2bits.PR2 = 0xFF;
    T2CONbits.T2CKPS = 1;
    T2CONbits.T2OUTPS = 0;
    /* 开启定时器Timer2 */
    T2CONbits.TMR2ON = 1;
    /* 清除定时器Timer2中断标记 */
    PIR1bits.TMR2IF = 0;
    /* 使能定时器Timer2中断 */
    PIE1bits.TMR2IE = 1;
    
    return;
}

void Pic16F1503_Int_Init(void)
{
    /* 开中断总开关 */
    INTCONbits.PEIE = 1;
    INTCONbits.GIE = 1;
    
    return;
}
void Pic16F1503_74HC595_SOP_SendData(unsigned short uiSendData)
{
    unsigned char i;
    
    SH_CP_LOW;
    ST_CP_LOW;
    DS_LOW;
    
    __nop();
    __nop();
    __nop();
    __nop();
    __nop();
	
    for (i = 0; i < 16; i++)
    {
        if (0x8000 == (uiSendData & 0x8000))
        {
            DS_HIGH;
        }
        else
        {
            DS_LOW;
        }
        uiSendData = uiSendData << 1;
        SH_CP_LOW;
        __nop();
        __nop();
        __nop();
        __nop();
        __nop();
        SH_CP_HIGH;
        __nop();
        __nop();
		__nop();
        __nop();
        __nop();
    }
    
    ST_CP_HIGH;
    __nop();
    __nop();
    __nop();
    __nop();
    __nop();
    ST_CP_LOW;
    
    return;
}

void HeatDevice_Set_Device_status(unsigned char ucLeftLedValue, unsigned char ucRightLedValue, unsigned char ucFuncLedValue, unsigned char ucOtherFuncValue)
{
    g_uiLeftLedDisplayStatuValue = (LedDisplayMap[ucLeftLedValue] << 8) | ucOtherFuncValue;
    g_uiRightLedDisplayStatuValue = (LedDisplayMap[ucRightLedValue] << 8) | ucOtherFuncValue;
    g_uiFunLedDisplayStatuValue = (ucFuncLedValue << 8) | ucOtherFuncValue;
    
    return;
}

/* Temp */
#if 0
void Pic16F1503_Display_Debug()
{
    /* 初始化一下，调试用 */
   /* g_uiLeftLedDisplayStatuValue = (LedDisplayMap[16] << 8);
	g_uiRightLedDisplayStatuValue = (LedDisplayMap[16] << 8);
	g_uiFunLedDisplayStatuValue = (0 << 8);*/
    HeatDevice_Set_Device_status(16, 16, 0, 0);
}
#endif
/* Temp */

void HeatDevice_Display_Led(unsigned short uiSendData)
{
    Pic16F1503_74HC595_SOP_SendData(uiSendData);
    
    return;
}

/* 实现童锁功能 */
void HeatDevice_Child_Lock_Operation(void)
{
    /* 童锁功能，键盘上其他按键功能被关闭，只有关闭童锁功能后才能继续使用，点亮童锁功能指示灯 */
    if (1 == g_ucChildLockFunEnableFlag)
    {
        g_uiFunLedDisplayStatuValue = g_uiFunLedDisplayStatuValue & (~((unsigned short)(0x4000)));
        
        g_ucChildLockFunEnableFlag = 0;
        g_ucTimerKeySwitchFlag = 1;
        
        return;
    }
    
    g_uiFunLedDisplayStatuValue = g_uiFunLedDisplayStatuValue | 0x4000;
    g_ucChildLockFunEnableFlag = 1;
    g_ucTimerKeySwitchFlag = 1;
    return;
}

/* 电源开关功能实现 */
void HeatDevice_Control_Switch_Operation(void)
{
    if (1 == g_ucPowerKeyPress)
    {
        /* 如果上一次已打开，则这次为关闭，需要重新初始化数码管显示和灯等相关功能 */
        g_ucPowerKeyPress = 0;
        g_ucModeChoosedKeyPressCount = 0;
        g_ucInConstantTempe = 0;
        g_ucSetTimerValue = 0;
        g_ucCurrentTempValue = 30;
        g_ucPowerDownFlag = 1;
     //   if (0 == g_ucCountDownFlag)
     //   {
            /* 定时标记没置上时直接显示默认态 */
     //       HeatDevice_Set_Device_status(16, 16, 0, 0);
     //   }
         
        return;
    }
    
    switch (g_ucInConstantTempe)
    {
        case 0:
            /* 开启电源开关后显示：温度值默认显示为30，电源指示灯，负离子灯，温度指示灯点亮,蜂鸣器叫一声，1秒后关蜂鸣器,输出低速风 */
            HeatDevice_Set_Device_status(3, 0, 0x8a, 0x11);
            break;
        case 1:
            HeatDevice_Set_Device_status(3, 0, 0x8e, 0x30);
            break;
        case 2:
            HeatDevice_Set_Device_status(3, 0, 0x9a, 0x30);
            break;
        case 3:
            HeatDevice_Set_Device_status(3, 0, 0xaa, 0x30);
            break;
        default:
            break;
    }
    
    g_ucPowerKeyPress = 1;
    g_ucTimerKeySwitchFlag = 1;
   // g_ucCountDownFlag = 0;
    g_ucPowerDownFlag = 0;
    g_usCountDownCounter = 0;
    g_ucCountDownValue = 59;
    g_ucHighTempFirstChooseFlag = 1;
    
    
    return;
}

/* 模式选择功能实现 */
void HeatDevice_WorkMode_Choose_Operation(void)
{
    switch (g_ucModeChoosedKeyPressCount)
    {
        case 0: 
            /* 高温档输出,数码管显示30，电源灯，负离子灯，温度灯和高温灯点亮，风机切换到高速档，第一次选择需要5秒后开启高温加热，以后的每次都直接开启高温加热不需要延时 */
            g_ucInConstantTempe = 2;
           // HeatDevice_Set_Device_status(3, 0, 0x9a, 0x50);
            if (1 == g_ucPowerKeyPress)
            {
                if (1 == g_ucHighTempFirstChooseFlag)
                {
                    HeatDevice_Set_Device_status(3, 0, 0x9a, 0x20);
                }
                else
                {
                    HeatDevice_Set_Device_status(3, 0, 0x9a, 0xe0);
                }
            }
            else
            {
                HeatDevice_Set_Device_status(3, 0, 0x90, 0x30);
            }
            g_ucModeChoosedKeyPressCount ++;
            g_ucHighTempOpenFlag = 1;
            break;
        case 1:
            /* 低温档输出,数码管显示30，电源灯，负离子灯，温度灯和低温灯点亮，切换到低温和高速 */
            g_ucInConstantTempe = 3;
            if (1 == g_ucPowerKeyPress)
            {
                //HeatDevice_Set_Device_status(3, 0, 0xaa, 0xa0);
                HeatDevice_Set_Device_status(3, 0, 0xaa, 0xa0);
            }
            else
            {
                HeatDevice_Set_Device_status(3, 0, 0xa0, 0x30);
            }
            g_ucModeChoosedKeyPressCount ++;
            break;

        case 2:
            /* 恒温档输出，数码管显示用户调，电源灯，负离子灯，温度灯和恒温灯点亮，切换到温度自动控制模式 */
            g_ucInConstantTempe = 1;
            if (1 == g_ucPowerKeyPress)
            {
                //HeatDevice_Set_Device_status(3, 0, 0x8e, 0xa0);
                HeatDevice_Set_Device_status(3, 0, 0x8e, 0xe0);
            }
            else
            {
                HeatDevice_Set_Device_status(3, 0, 0x84, 0x30);
            }
            g_ucCurrentTempValue = 30;
            g_ucModeChoosedKeyPressCount = 0;
            
            break;
        default:
            
            break;
    }
    
    g_ucTimerKeySwitchFlag = 1;
    
    return;
}

/* 温度加减操作 */
void HeatDevice_AddOrSub_Temp_Operation(unsigned char ucOperatType)
{
    /* 温度设置，最高30度，最低10度 */
    if (1 == ucOperatType)
    {
        /* 温度加操作 */
        if (29 <= g_ucCurrentTempValue)
        {
            g_ucCurrentTempValue = 29;
        }
        
        g_ucCurrentTempValue ++;
        
    }
    else if (0 == ucOperatType)
    {
        /* 温度减操作 */
        if (11 > g_ucCurrentTempValue)
        {
            g_ucCurrentTempValue = 11;
        }
        g_ucCurrentTempValue --;
    }
    
    //HeatDevice_Set_Device_status((g_ucCurrentTempValue / 10), (g_ucCurrentTempValue % 10), 0x8e, 0xa0);
    if (20 < g_ucCurrentTempValue)
    {
        g_uiLeftLedDisplayStatuValue = (LedDisplayMap[g_ucCurrentTempValue / 10] << 8) | 0xe0;
        g_uiRightLedDisplayStatuValue = (LedDisplayMap[g_ucCurrentTempValue % 10] << 8) | 0xe0;
        g_uiFunLedDisplayStatuValue = 0x8ea0;
    }
    else
    {
        g_uiLeftLedDisplayStatuValue = (LedDisplayMap[g_ucCurrentTempValue / 10] << 8) | 0x90;
        g_uiRightLedDisplayStatuValue = (LedDisplayMap[g_ucCurrentTempValue % 10] << 8) | 0x90;
        g_uiFunLedDisplayStatuValue = 0x8e90;
    }

    g_ucLedFlashFlag = 1;
    
    return;
}

/* 定时器操作 */
void HeatDevice_Timer_Operation(void)
{
    /* g_ucTimerKeySwitchFlag这个标记在电源按钮、模式切换按钮和童锁按钮按下时置上 */
    if (0 == g_ucTimerKeySwitchFlag)
    {
        g_ucSetTimerValue ++;
    }
    
    if (12 < g_ucSetTimerValue)
    {
        g_ucSetTimerValue = 0;
    }

    g_uiLeftLedDisplayStatuValue = (LedDisplayMap[g_ucSetTimerValue / 10] << 8) | (g_uiLeftLedDisplayStatuValue & 0xff);
    g_uiRightLedDisplayStatuValue = (LedDisplayMap[g_ucSetTimerValue % 10] << 8) | (g_uiRightLedDisplayStatuValue & 0xff);
    g_uiFunLedDisplayStatuValue = g_uiFunLedDisplayStatuValue | (0x1 << 8);
   // HeatDevice_Set_Device_status(g_ucSetTimerValue / 10, g_ucSetTimerValue % 10, g_uiFunLedDisplayStatuValue | 0x1, g_uiLeftLedDisplayStatuValue & 0xff);
    
    g_ucLedFlashFlag = 1;
    g_ucTimerKeySwitchFlag = 0;
 //   g_ucCountDownFlag = 1;
}
/* 矩阵按键功能分发处理 */
void HeatDevice_KeyBoard_Function_disapatch(void)
{
    if (1 == IOCAFbits.IOCAF0)
    {
        PORTAbits.RA4 = 1;
        __nop();
        __nop();
        __nop();
        if (0 == PORTAbits.RA0)
        {        
            g_unPressFlag.ucTimer = 1;
        }
        else
        {       
            g_unPressFlag.ucSwitch = 1;
        }      
        PORTAbits.RA4 = 0;
        
    }
    else if (1 == IOCAFbits.IOCAF1)
    {
        PORTAbits.RA4 = 1;
        __nop();
        __nop();
        __nop();
        if (0 == PORTAbits.RA1)
        {
            g_unPressFlag.ucTemSub = 1;
            
        }
        else
        {           
            g_unPressFlag.ucMode = 1;
        }      
        PORTAbits.RA4 = 0;
    }
    else if (1 == IOCAFbits.IOCAF2)
    {
        PORTAbits.RA4 = 1;
        __nop();
        __nop();
        __nop();
        if (0 == PORTAbits.RA2)
        {            
            g_unPressFlag.ucTemAdd = 1;
        }
        else
        {
            g_unPressFlag.ucLock = 1;
        }      
        PORTAbits.RA4 = 0;
    }
}

/* 矩阵按键扫描去抖确认函数 */
void HeatDevice_KeyBoard_Confirm(void)
{   
    if (1 == g_unPressFlag.ucLock)
    {
        PORTAbits.RA5 = 1;
        
        if (0 == PORTAbits.RA2)
        {
            /* 童锁 */        
            HeatDevice_Child_Lock_Operation();
        }
        PORTAbits.RA5 = 0;
    }
    else if (1 == g_unPressFlag.ucMode)
    {
        PORTAbits.RA5 = 1;
        if (0 == PORTAbits.RA1)
        {
            /* 模式选择，电源开关开启时才有效*/
            if (((1 == g_ucPowerKeyPress) && (!g_ucChildLockFunEnableFlag)) ||
                (0 != g_ucSetTimerValue) && (!g_ucChildLockFunEnableFlag))
            {
                HeatDevice_WorkMode_Choose_Operation();
            }
        }
        PORTAbits.RA5 = 0;
    }
    else if (1 == g_unPressFlag.ucSwitch)
    {
        PORTAbits.RA5 = 1;
        if (0 == PORTAbits.RA0)
        {
            /* 开关操作*/
            if (!g_ucChildLockFunEnableFlag)
            {
                HeatDevice_Control_Switch_Operation();
            }
            
        }
        PORTAbits.RA5 = 0;
    }
    else if (1 == g_unPressFlag.ucTemAdd)
    {        
        PORTAbits.RA4 = 1;
        if (0 == PORTAbits.RA2)
        {
            /* 温度加，电源开关开启时才有效*/
            if ((1 == g_ucPowerKeyPress) && (1 == g_ucInConstantTempe) && (!g_ucChildLockFunEnableFlag))
            {
                HeatDevice_AddOrSub_Temp_Operation(1);
            }
        }
        PORTAbits.RA4 = 0;
    }
    else if (1 == g_unPressFlag.ucTemSub)
    {
        PORTAbits.RA4 = 1;
        if (0 == PORTAbits.RA1)
        {
            /* 温度减，电源开关开启时才有效*/
            if ((1 == g_ucPowerKeyPress) && (1 == g_ucInConstantTempe) && (!g_ucChildLockFunEnableFlag))
            {
                HeatDevice_AddOrSub_Temp_Operation(0);
            }
        }
        PORTAbits.RA4 = 0;
    }
    else if (1 == g_unPressFlag.ucTimer)
    {
        PORTAbits.RA4 = 1;
        if (0 == PORTAbits.RA0)
        {
            /* 定时操作 */
            if (!g_ucChildLockFunEnableFlag)
            {
                HeatDevice_Timer_Operation();
            }
        }
        PORTAbits.RA4 = 0;
    }

    return;
}

/* 红外线指令功能分发 */
void HeatDevice_Infra_Red_Function_Disapatch(void)
{
    //unsigned char ucUserCodeH = 0;
    //unsigned char ucUserCodeL = 0;
    unsigned char ucFunPositiveCode = 0;
    unsigned char ucFunNegativeCode = 0;
    
    /* 从红外序列码中获取用户码，功能编码和反编码 */
   // ucUserCodeH = (unsigned char)(g_InfraData >> 24);
   // ucUserCodeL = (unsigned char)((g_InfraData & 0xff0000) >> 16);
    ucFunPositiveCode = (g_InfraData & 0xff00) >> 8;
    ucFunNegativeCode = g_InfraData & 0xff;
    
    /* 对功能编码进行校验检查 */
    if (0 != ucFunPositiveCode & ucFunNegativeCode)
    {
        /*接收到的数据有误 */
    }
    
    /* 根据功能编码进行分发执行 */
    switch (ucFunPositiveCode)
    {
        case 0x30:
            /* 定时操作 */
            if (!g_ucChildLockFunEnableFlag)
            {
                HeatDevice_Timer_Operation();
            }
            break;
        case 0x70:
            /* 温度加，电源开关开启时才有效*/
            if ((1 == g_ucPowerKeyPress) && (1 == g_ucInConstantTempe) && (!g_ucChildLockFunEnableFlag))
            {
                HeatDevice_AddOrSub_Temp_Operation(1);
            }
            break;
        case 0x60:
            /* 温度减，电源开关开启时才有效*/
            if ((1 == g_ucPowerKeyPress) && (1 == g_ucInConstantTempe) && (!g_ucChildLockFunEnableFlag))
            {
                HeatDevice_AddOrSub_Temp_Operation(0);
            }
            break;
        case 0x20:
            /* 童锁 */        
            HeatDevice_Child_Lock_Operation();
            break;
        case 0x78:
            /* 模式选择，电源开关开启时才有效*/
           // if ((1 == g_ucPowerKeyPress) && (!g_ucChildLockFunEnableFlag))
          //  {
            if (((1 == g_ucPowerKeyPress) && (!g_ucChildLockFunEnableFlag)) ||
                (0 != g_ucSetTimerValue) && (!g_ucChildLockFunEnableFlag))
            {
                HeatDevice_WorkMode_Choose_Operation();
            }
          //}
            break;
        case 0x38:
            /* 开关操作*/
            if (!g_ucChildLockFunEnableFlag)
            {
                HeatDevice_Control_Switch_Operation();
            }
            break;
        default:
            /* 编码异常 */
            break;
    }
    
    return;
}

/* 红外线信号处理 */
void HeatDevice_Infra_red_Operation(void)
{
    /* 如果是非法时长的信号则复位计数变量和红外标记 */
    if ((g_uiInfraRedCounter > 54) || (g_uiInfraRedCounter < 3) ||
        ((g_uiInfraRedCounter > 10) && (g_uiInfraRedCounter < 51)))
    {        
        g_uiInfraRedCounter = 0;
        g_ucStartFlagValid = 0;
        g_ucDataIndex = 31;
               
        return;
    }
	
    if (g_uiInfraRedCounter >= 51 && g_uiInfraRedCounter <= 54)
    {
        /* 收到一个前导码 */
        g_uiInfraRedCounter = 0;
        /* 置前导码有效标记位 */
        g_ucStartFlagValid = 1;
        g_ucDataIndex = 31;
        
        return;      
    }
    
    if (g_ucStartFlagValid)
    {
        if ((g_uiInfraRedCounter >= 3) && (g_uiInfraRedCounter <= 6))
        {
            /* 数据为0 */
            g_InfraData &= ~(1 << g_ucDataIndex);
            g_uiInfraRedCounter = 0;
            g_ucDataIndex --;
            
        }
        else if ((g_uiInfraRedCounter >= 7) && (g_uiInfraRedCounter <= 10))
        {
            /* 数据为1 */
            g_InfraData |= (1 << g_ucDataIndex);
            g_uiInfraRedCounter = 0;
            g_ucDataIndex --;
            
        }

    }

	if (g_ucDataIndex == 0xff)
	{
        HeatDevice_Infra_Red_Function_Disapatch();
	    g_ucInfraRedFlag = 0;
	}
	
    return;
}

/* 电源关闭后倒计时处理函数 */
void HeatDevice_Count_Down_Operation(void)
{
  //  if ((1 == g_ucCountDownFlag) && (1 == g_ucPowerDownFlag))
    if (1 == g_ucPowerDownFlag)
    {
        if (4000 == g_usCountDownCounter)
        {
            if (0 == g_ucCountDownValue)
            {
                HeatDevice_Set_Device_status(16, 16, 0, 0x30);
        //        g_ucCountDownFlag = 0;
                g_usCountDownCounter = 0;
                g_ucCountDownValue = 59;
                g_ucPowerDownFlag = 0;
                
                return;
            }
            /*g_uiLeftLedDisplayStatuValue = ((LedDisplayMap[g_ucCountDownValue / 10] << 8) | 0x80);
            g_uiRightLedDisplayStatuValue = ((LedDisplayMap[g_ucCountDownValue % 10] << 8) | 0x80);
            g_uiFunLedDisplayStatuValue = 0x80;*/
            /* 风机高速转动 */
            HeatDevice_Set_Device_status(g_ucCountDownValue / 10, g_ucCountDownValue % 10, 0x0, 0x20);
            
            g_ucCountDownValue --;
            g_usCountDownCounter = 0;
        }
    }
    
    return;
}

/* 定时器Timer1中断处理 */
void Timer1_Isr(void)
{
    unsigned short uiLedStatu = 0;
    g_ucLedDisplayConFlag ++; 
     
    if (0 != g_unPressFlag.ucKeyBoardPressFlag)
    {
        g_ucTimeCounter ++;
    }
    
    switch ((g_ucLedDisplayConFlag % 4))
    {
        case 1:
             uiLedStatu = g_uiLeftLedDisplayStatuValue;
             /* 开十进制位数码管 */           
             uiLedStatu |= 0xe;
             if (1 != g_ucLedFlashCounterFlag % 2)
             {
                 uiLedStatu &= ~((unsigned short)(0x2)); 
             }
        break;
        case 2:
             uiLedStatu = g_uiRightLedDisplayStatuValue;
             /* 开个位数码管 */
             uiLedStatu |= 0xe;
             if (1 != g_ucLedFlashCounterFlag % 2)
             {
                 uiLedStatu &= ~((unsigned short)(0x4)); 
             }
        break;
        case 3:
              uiLedStatu = g_uiFunLedDisplayStatuValue;
              /* 开功能状态灯 */
              uiLedStatu |= 0xe;
              uiLedStatu &= ~((unsigned short)(0x8));
              g_ucLedDisplayConFlag = 0;
        break;
        default:
            break;
    }
    
    HeatDevice_Display_Led(uiLedStatu);
    
    return;
}

/* 定时器Timer2中断处理函数 */
void Timer2_Isr(void)
{  
    /* ADC转化定时计数 */
    g_usAdcCoverTimerCount ++;
    
    if (g_ucInfraRedFlag)
    {
        g_uiInfraRedCounter ++;
    }
    
    /* led灯闪烁时间计数 */
    if (1 == g_ucLedFlashFlag)
    {
        g_usLedFlashCounter ++;
    }
    
    if (g_ucPowerDownFlag)
    {
        g_usCountDownCounter ++;
    }
    
    if ((1 == g_ucHighTempOpenFlag) && (1 == g_ucHighTempFirstChooseFlag))
    {
        /* 5秒定时计数 */
        g_usHighTempOpenCount ++;
    }
}

/* Adc中断处理函数 */
void Adc_Isr(void)
{
    unsigned short uiAdcResult = 0;
    unsigned char ucTempMapIndex = 0;
    
    /* 获取结果 */
    uiAdcResult = (ADRESLbits.ADRESL | (ADRESHbits.ADRESH << 8));

    if (1 == g_ucPowerKeyPress)
    {
        if (1 == g_ucInConstantTempe)
        {
            /* 在恒温模式时需要根据当前温度来控制加热管的开关 */       
            ucTempMapIndex = (g_ucCurrentTempValue - 10) / 2;
            if (g_ausSenserRValueMap[ucTempMapIndex] < (uiAdcResult + PICF1503_ADC_TEMP_OFFSET))
            {
                /* 加热 */
                if (g_ucCurrentTempValue > 20)
                {
                    g_uiLeftLedDisplayStatuValue = (g_uiLeftLedDisplayStatuValue & 0xff0f)  | 0xe0;
                    g_uiRightLedDisplayStatuValue = (g_uiRightLedDisplayStatuValue & 0xff0f) | 0xe0;
                    g_uiFunLedDisplayStatuValue = (g_uiFunLedDisplayStatuValue & 0xff0f) | 0xe0;
                }
                else
                {
                    g_uiLeftLedDisplayStatuValue = (g_uiLeftLedDisplayStatuValue & 0xff0f)  | 0x90;
                    g_uiRightLedDisplayStatuValue = (g_uiRightLedDisplayStatuValue & 0xff0f) | 0x90;
                    g_uiFunLedDisplayStatuValue = (g_uiFunLedDisplayStatuValue & 0xff0f) | 0x90;
                }
            }
            else if (g_ausSenserRValueMap[ucTempMapIndex + 1] > (uiAdcResult - PICF1503_ADC_TEMP_OFFSET))
            {
                /* 关闭 */
                g_uiLeftLedDisplayStatuValue = (g_uiLeftLedDisplayStatuValue & 0xff0f)  | 0x10;
                g_uiRightLedDisplayStatuValue = (g_uiRightLedDisplayStatuValue & 0xff0f) | 0x10;
                g_uiFunLedDisplayStatuValue = (g_uiFunLedDisplayStatuValue & 0xff0f) | 0x10;
            }
        }
        else
        {
            if (631 < (uiAdcResult + PICF1503_ADC_TEMP_OFFSET))
            {
                /* 加热 */
                if (2 == g_ucInConstantTempe)
                {
                    /* 温度高档 */
                    g_uiLeftLedDisplayStatuValue = (g_uiLeftLedDisplayStatuValue & 0xff0f)  | 0xe0;
                    g_uiRightLedDisplayStatuValue = (g_uiRightLedDisplayStatuValue & 0xff0f) | 0xe0;
                    g_uiFunLedDisplayStatuValue = (g_uiFunLedDisplayStatuValue & 0xff0f) | 0xe0;
                }
                else if (3 == g_ucInConstantTempe)
                {
                    /*温度低档 */
                    g_uiLeftLedDisplayStatuValue = (g_uiLeftLedDisplayStatuValue & 0xff0f)  | 0xa0;
                    g_uiRightLedDisplayStatuValue = (g_uiRightLedDisplayStatuValue & 0xff0f) | 0xa0;
                    g_uiFunLedDisplayStatuValue = (g_uiFunLedDisplayStatuValue & 0xff0f) | 0xa0;
                }
            }
            else
            {
                /* 关闭 */
                g_uiLeftLedDisplayStatuValue = (g_uiLeftLedDisplayStatuValue & 0xff0f)  | 0x10;
                g_uiRightLedDisplayStatuValue = (g_uiRightLedDisplayStatuValue & 0xff0f) | 0x10;
                g_uiFunLedDisplayStatuValue = (g_uiFunLedDisplayStatuValue & 0xff0f) | 0x10;            
            }
        }
    }
    else
    {
        
    }
    
    return;
}

/* Gpio中断处理函数 */
void Gpio_Isr(void)
{
    if ((1 == IOCAFbits.IOCAF0) || (1 == IOCAFbits.IOCAF1) || (1 == IOCAFbits.IOCAF2))
    {
        /* 去使能中断 */
        IOCANbits.IOCAN &= 0xf8; 
        /* 调用按键分发函数进行处理 */
        HeatDevice_KeyBoard_Function_disapatch();
        /* 清除中断标记，使能中断 */
        IOCAFbits.IOCAF &= 0xf8;
        IOCANbits.IOCAN |= 0x7;

    }
    else if ((1 == IOCAFbits.IOCAF3))
    {
        /* 去使能中断 */
        IOCANbits.IOCAN3 = 0;

		/* 红外信号处理函数 */
        HeatDevice_Infra_red_Operation();
        g_ucInfraRedFlag = 1;
		/* 清除中断标记，使能中断 */
        IOCAFbits.IOCAF3 = 0;
        IOCANbits.IOCAN3 = 1;	       
    }
    else
    {
        /* 如果是非期望的GPIO中断上报则不处理，直接清除中断标记 */
        IOCAFbits.IOCAF &= 0xf;
    }
    
    return;
}
/* 中断总入口 */
void interrupt HeatDevice_Interrupt_Isr(void)
{
    if (1 == PIR1bits.TMR1IF)
    {
        /* 去使能定时器Timer1中断 */
        PIE1bits.TMR1IE = 0;
        /* 清除中断标记 */
        PIR1bits.TMR1IF = 0;
        Timer1_Isr();
        /* 使能定时器Timer1中断 */
       PIE1bits.TMR1IE = 1;
    }
    else if (1 == PIR1bits.ADIF)
    {
        /* 去使能Adc中断 */
        PIE1bits.ADIE = 0;
        /* 清除中断标记 */
        PIR1bits.ADIF = 0;
        Adc_Isr();
        /* 使能Adc中断 */
        PIE1bits.ADIE = 1;
    }
    else if (1 == INTCONbits.IOCIF)
    {
        Gpio_Isr();
    }
    else if (1 == PIR1bits.TMR2IF)
    {
        /* 去使能定时器Timer2中断 */
        PIE1bits.TMR2IE = 0;
        /* 清除中断标记 */
        PIR1bits.TMR2IF = 0;
        Timer2_Isr();
        /* 使能定时器Timer2中断 */
       PIE1bits.TMR2IE = 1;
    }
    
    return;
}

void main(void)
{   
    unsigned char ucLastTimerValue = 0;
    /* 标记初始化 */
    g_unPressFlag.ucKeyBoardPressFlag = 0;
    
    /* 系统时钟配置 */
    PicF1503_SystemTime_Config();
    /* 初始化ADC */
    Pic16F1503_Adc_Init();
    
    /* 初始化GPIO */
    Pic16F1503_Gpio_Init();
    
    /* 初始化I2C */
   Pic16F1503_I2c_Init();
    
    /* 显示值初始化，调试用 */
	HeatDevice_Set_Device_status(16, 16, 0, 0x30);
    
    /* 初始化Timer1 */
    Pic16F1503_Timer1_Init();
    
    /* 初始化Timer2 */
    Pic16F1503_Timer2_Init();
    
    /* 中断初始化 */
    Pic16F1503_Int_Init();
   
    while(1)
    {
        if (40 == g_ucTimeCounter)
        {
            /* 调用按键扫描确认函数 */
            HeatDevice_KeyBoard_Confirm();      
           
            /* 清除中断标记，并复位计数变量,按键标记 */
            g_unPressFlag.ucKeyBoardPressFlag = 0;
            g_ucTimeCounter = 0;
        }
        
        if (1 == g_ucLedFlashFlag)
        {
            /* 当闪烁标记等于1时，需要控制灯闪烁 */
            if (2000 == g_usLedFlashCounter)
            {
                g_ucLedFlashCounterFlag ++;
                
                if (8 == g_ucLedFlashCounterFlag)
                {
                    g_ucLedFlashFlag = 0;
                    g_ucLedFlashCounterFlag = 0;
                }
                g_usLedFlashCounter = 0;
            }
        }
        
        if ((1 == g_ucHighTempOpenFlag) && (1 == g_ucHighTempFirstChooseFlag) && (1 == g_ucPowerKeyPress))
        {
            if(20000 == g_usHighTempOpenCount)
            {
                g_uiLeftLedDisplayStatuValue = (g_uiLeftLedDisplayStatuValue & 0xff00) | 0xe0;
                g_uiRightLedDisplayStatuValue = (g_uiRightLedDisplayStatuValue & 0xff00) | 0xe0;
                g_uiFunLedDisplayStatuValue = (g_uiFunLedDisplayStatuValue & 0xff00) | 0xe0;
                g_ucHighTempOpenFlag = 0;
                g_usHighTempOpenCount = 0;
                g_ucHighTempFirstChooseFlag = 0;
            }
        }
        
        if (40000 == g_usAdcCoverTimerCount)
        {
            /* 每10秒发起一次ADC转化，读取温感温度值，这位会在转化完成后被自动清除 */
            ADCON0bits.ADGO = 1;
            g_usAdcCoverTimerCount = 0;
            g_usTimerFuncCount ++;
        }
        
        if (1 == g_ucChildLockFunEnableFlag)
        {
            if (0 == g_usAdcCoverTimerCount % 2000)
            {
                g_uiFunLedDisplayStatuValue = g_uiFunLedDisplayStatuValue | 0x4000;
            }
            else
            {
                g_uiFunLedDisplayStatuValue = g_uiFunLedDisplayStatuValue & (~((unsigned short)(0x4000)));
            }
        }
        
        /* 定时时间到执行相应操作 */
        if (0 != g_ucSetTimerValue)
        {
            if (ucLastTimerValue == g_ucSetTimerValue)
            {
                if (g_usTimerFuncCount == (g_ucSetTimerValue * 360))
                {
                    g_usTimerFuncCount = 0;
                    g_ucSetTimerValue = 0;
                    /* 执行相应的开关操作 */
                    HeatDevice_Control_Switch_Operation();
                }
            }
            else
            {
                g_usTimerFuncCount = 0;
                ucLastTimerValue = g_ucSetTimerValue;
            }
        }
        
        /* 倒计时1分钟 */
        HeatDevice_Count_Down_Operation();
    };
    return;
}
