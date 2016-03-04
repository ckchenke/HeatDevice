/*
 * File:   Main_HeatDevice.c
 * Author: lenovo
 *
 * Created on 2015��12��31��, ����3:41
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

/* �������б�ǣ����ڼ�¼�ĸ������������� */
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
unsigned short g_uiLeftLedDisplayStatuValue = 0;    /* ʮ����λ������ܵ�״ֵ̬��¼ */
unsigned short g_uiRightLedDisplayStatuValue = 0;   /* ��λ������ܵ�״ֵ̬��¼ */
unsigned short g_uiFunLedDisplayStatuValue = 0;     /* ����״̬�Ƶ�״ֵ̬��¼ */
unsigned char g_ucLedDisplayConFlag =0;             /* ����ѯ������ǣ�0ʮ��������ܣ�1��λ����ܣ�2����״̬�� */
unsigned char g_ucInfraRedFlag = 0;                 /* �����źŴ������ */
unsigned short g_uiInfraRedCounter = 0;             /* �����ź�ʱ������ڶ�ʱ���ϱ��Ĵ��������ڹ���һ�������źŵ�ʱ�� */
unsigned char g_ucStartFlagValid = 0;               /* ǰ������Ч��ǣ������ж�������ֻ����ǰ������Ч������£����������ݲ������� */
unsigned char g_ucDataIndex = 31;                   /* ������Ч�����±� */
unsigned int g_InfraData = 0;                       /* �洢�������� */
unsigned char g_ucTimeCounter = 0;                  /* ȥ��������Ҫ��ʱ20ms���ң��������������¼timer��ʱ����ʵ�ʽ���Ĵ������Դ���������ʱʱ�� */
unsigned char g_ucPowerKeyPress = 0;                /* ��Դ���Ƿ��£����ڱ�ʶϵͳ�Ƿ��� */
unsigned char g_ucModeChoosedKeyPressCount = 0;     /* ���ڼ�¼ģʽѡ��ť�İ��´��������ڿ��Ƶ�ǰ�Ĺ���ģʽ */
unsigned char g_ucInConstantTempe = 0;              /* ����ģʽ��� */
unsigned char g_ucCurrentTempValue = 0;             /* ���ڼ�¼��ǰ����״̬���õ��¶�ֵ */
unsigned char g_ucSetTimerValue = 0;                /* ��ʱʱ������ֵ */
unsigned char g_ucLedFlashFlag = 0;                 /* �������˸��ǣ����¶ȼӼ��Ͷ�ʱʱ��ı�ʱ��Ҫ����˸���� */
unsigned short g_usLedFlashCounter = 0;             /* �������˸��ʱ����������500ms��˸һ�� */
unsigned short g_usCountDownCounter = 0;            /* 1�붨ʱ */
//unsigned char g_ucCountDownFlag = 0;                /* ����ʱ��� */
unsigned char g_ucLedFlashCounterFlag = 0;          /* ���ڼ�¼�������˸�ı�� */
unsigned char g_ucTimerKeySwitchFlag = 0;           /* ��ʱ��ť�л���ǣ���������������ʱ��Ҫ������������ */
unsigned char g_ucChildLockFunEnableFlag = 0;       /* ͯ�����ܿ�����ǣ�������������ʱ�������������ܹر� */
unsigned char g_ucCountDownValue = 59;              /* ����ʱ��ʼֵ */
unsigned char g_ucPowerDownFlag = 0;                /* ��Դ��������ʱ���øñ�ǣ�����ʹ�ܵ���ʱ���� */
unsigned char g_ucHighTempOpenFlag = 0;             /* ���¿������ */
unsigned short g_usHighTempOpenCount = 0;           /* ���ڸ��¶�ʱ���� */
unsigned char g_ucHighTempFirstChooseFlag = 1;      /* ������Դ�״α�����ʱ����Ҫ��5�����ʱ����������Ҫ */
unsigned short g_usAdcCoverTimerCount = 0;          /* Adcת����ʱ��������ÿ10����һ��adc��ȡ���������ڼ���¶� */
unsigned short g_usTimerFuncCount = 0;              /* ��ʱ���ܼ�����ÿ10���һ�� */
unsigned short g_ausSenserRValueMap[11] = {801, 786, 770, 754, 738, 722, 703, 687, 668, 651, 631};    /* 10-30��֮�䰴10���������¶ȿ��� */         
KeyBoardPress_t g_unPressFlag;                      /* ������ǣ����ڼ�¼�ĸ����������� */

#define PICF1503_ADC_TEMP_OFFSET              (8)   /* ��ΪADC������ֵ�᲻ͣ�ĸ�������Ҫ����һ�� */

void PicF1503_SystemTime_Config(void)
{
    /* ����Ϊ�ڲ�ʱ��Դ����ʱ��Ƶ��Ϊ16�� */
    OSCCONbits.IRCF = 0xF;
    OSCCONbits.SCS = 0x3;
}
void Pic16F1503_Adc_Init(void)
{
    /* ���ý�������ʽΪ�Ҷ��룬ADC����ʱ��Ϊϵͳʱ�ӵ�4��Ƶ */
    ADCON1bits.ADFM = 1;
    ADCON1bits.ADCS = 4;
    /* ���û�׼��ѹ */
    ADCON1bits.ADPREF = 0;

    /* ѡ��ADCͨ������ʹ��ADC���� */
    ADCON0bits.CHS = 0;
    ADCON0bits.ADON = 1;
    
    /* ����жϱ�ǣ�ʹ��ADC�ж� */
    PIR1bits.ADIF = 0;
    PIE1bits.ADIE = 1;
    
}
void Pic16F1503_Gpio_Init(void)
{
#if 0
    OPTION_REG &= 0x7F; //����OPTION_REG �Ĵ�����ȫ��WPUEN λ���Ӷ�ʹ�ܸ�����������
    /* ����porta0,1,2Ϊ�����ź�����,���������жϴ�����ʽΪ�½��ش���,����жϱ�� */
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
    /* ���������жϴ�����ʽΪ�½��ش���������жϱ�� */
    TRISA3 = 1;
    WPUAbits.WPUA3 = 1;
    IOCAFbits.IOCAF3 = 0;
    IOCANbits.IOCAN3 = 1;
    /* ����porta4,5�����ź����,�ùܽ�Ϊ�͵�ƽ */
    TRISA4 = 0;
    ANSELAbits.ANSA4 = 0;
    TRISA5 = 0;
    PORTAbits.RA4 = 0;
    PORTAbits.RA5 = 0;
    
    /* ����portc3,4,5�����ź���� */
    TRISC3 = 0;
    ANSELCbits.ANSC3 = 0;
    TRISC4 = 0;
    TRISC5 = 0;
    /* ����portc2Ϊģ���ź����� */
    TRISC2 = 1;
    ANSELCbits.ANSC2 = 1;
    
    /* ʹ��GPIO�ж� */
    INTCONbits.IOCIE = 1;
#else
    OPTION_REG &= 0x7F; //����OPTION_REG �Ĵ�����ȫ��WPUEN λ���Ӷ�ʹ�ܸ�����������
    /* ����porta0,1,2,3Ϊ�����ź�����,���������жϴ�����ʽΪ�½��ش���,����жϱ��,port4,5Ϊ�����ź������
     * �ܽ�Ĭ�ϳ�ʼ��Ϊ�͵�ƽ��portc3,4,5Ϊ�����ź������portc2Ϊģ���ź����� */
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
    
    /* ʹ��GPIO�ж� */
    INTCONbits.IOCIE = 1;
#endif
    
    return;
}

/* ��ʼ��i2c���� */
void Pic16F1503_I2c_Init(void)
{
    /* ����SCL��SDAΪ����ģʽ */
    TRISC0 = 1;
    TRISC0 = 1;
    
    /* ���ù���Ƶ��,���ó�50K����������ģʽ */
    SSP1CON1bits.SSPM =0x8;
    SSP1ADDbits.ADD = 0x50;
    
    /* ����I2Cͨ�� */
    SSP1CON1bits.SSPEN = 1;
    
     return;   
}

/* �������� */
void Pic16F1503_I2c_Send(unsigned char ucDeviceAddr, unsigned char ucSendData)
{
    /* ����һ����ʼ�ź� */
    SSP1CON2bits.SEN = 1;//Start condition
    while(0 == PIR1bits.SSP1IF);//waiting for Start condition completed.
    PIR1bits.SSP1IF = 0;
    
    /* ���͵ĵ�һ���ֽڰ���7λ���豸��ַ��һλ��д��־ */
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

/* �������� */
void Pic16F1503_I2c_Recive(unsigned char ucDeviceAddr, unsigned char *pucReciveData)
{
    /* ����һ����ʼ�ź� */
    SSP1CON2bits.SEN = 1;//Start condition
    while(0 == PIR1bits.SSP1IF);//waiting for Start condition completed.
    PIR1bits.SSP1IF = 0;
    
    /* ���͵ĵ�һ���ֽڰ���7λ���豸��ַ��һλ�Ķ���־ */
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
    /* ����ʱ��Ϊ16M */
    T1CONbits.TMR1CS = 1;    
    T1CONbits.T1CKPS = 0;
    /* ����ʱ��ͬ�����򿪶�ʱ��Timer1 */
   // T1CONbits.nT1SYNC = 0;
    T1CONbits.TMR1ON = 1;
    
    /* ���ö�ʱ��1ʱ��Ϊ0.25MS�ϱ�һ�� */
    TMR1Hbits.TMR1H = 0xF0;
    TMR1Lbits.TMR1L = 0x5F;
    
    /* ����жϱ�ǣ�ʹ�ܶ�ʱ���ж�Timer1 */
    PIR1bits.TMR1IF = 0;
    PIE1bits.TMR1IE = 1;
    
    return;

}

void Pic16F1503_Timer2_Init(void)
{
    /* ����ʱ��Ϊ4��Ƶ��ϵͳʱ�ӣ�������ʱ������4��Ƶ,��ʱ��ʱ��Ϊ255us */
    PR2bits.PR2 = 0xFF;
    T2CONbits.T2CKPS = 1;
    T2CONbits.T2OUTPS = 0;
    /* ������ʱ��Timer2 */
    T2CONbits.TMR2ON = 1;
    /* �����ʱ��Timer2�жϱ�� */
    PIR1bits.TMR2IF = 0;
    /* ʹ�ܶ�ʱ��Timer2�ж� */
    PIE1bits.TMR2IE = 1;
    
    return;
}

void Pic16F1503_Int_Init(void)
{
    /* ���ж��ܿ��� */
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
    /* ��ʼ��һ�£������� */
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

/* ʵ��ͯ������ */
void HeatDevice_Child_Lock_Operation(void)
{
    /* ͯ�����ܣ������������������ܱ��رգ�ֻ�йر�ͯ�����ܺ���ܼ���ʹ�ã�����ͯ������ָʾ�� */
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

/* ��Դ���ع���ʵ�� */
void HeatDevice_Control_Switch_Operation(void)
{
    if (1 == g_ucPowerKeyPress)
    {
        /* �����һ���Ѵ򿪣������Ϊ�رգ���Ҫ���³�ʼ���������ʾ�͵Ƶ���ع��� */
        g_ucPowerKeyPress = 0;
        g_ucModeChoosedKeyPressCount = 0;
        g_ucInConstantTempe = 0;
        g_ucSetTimerValue = 0;
        g_ucCurrentTempValue = 30;
        g_ucPowerDownFlag = 1;
     //   if (0 == g_ucCountDownFlag)
     //   {
            /* ��ʱ���û����ʱֱ����ʾĬ��̬ */
     //       HeatDevice_Set_Device_status(16, 16, 0, 0);
     //   }
         
        return;
    }
    
    switch (g_ucInConstantTempe)
    {
        case 0:
            /* ������Դ���غ���ʾ���¶�ֵĬ����ʾΪ30����Դָʾ�ƣ������ӵƣ��¶�ָʾ�Ƶ���,��������һ����1���ط�����,������ٷ� */
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

/* ģʽѡ����ʵ�� */
void HeatDevice_WorkMode_Choose_Operation(void)
{
    switch (g_ucModeChoosedKeyPressCount)
    {
        case 0: 
            /* ���µ����,�������ʾ30����Դ�ƣ������ӵƣ��¶ȵƺ͸��µƵ���������л������ٵ�����һ��ѡ����Ҫ5��������¼��ȣ��Ժ��ÿ�ζ�ֱ�ӿ������¼��Ȳ���Ҫ��ʱ */
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
            /* ���µ����,�������ʾ30����Դ�ƣ������ӵƣ��¶ȵƺ͵��µƵ������л������º͸��� */
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
            /* ���µ�������������ʾ�û�������Դ�ƣ������ӵƣ��¶ȵƺͺ��µƵ������л����¶��Զ�����ģʽ */
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

/* �¶ȼӼ����� */
void HeatDevice_AddOrSub_Temp_Operation(unsigned char ucOperatType)
{
    /* �¶����ã����30�ȣ����10�� */
    if (1 == ucOperatType)
    {
        /* �¶ȼӲ��� */
        if (29 <= g_ucCurrentTempValue)
        {
            g_ucCurrentTempValue = 29;
        }
        
        g_ucCurrentTempValue ++;
        
    }
    else if (0 == ucOperatType)
    {
        /* �¶ȼ����� */
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

/* ��ʱ������ */
void HeatDevice_Timer_Operation(void)
{
    /* g_ucTimerKeySwitchFlag�������ڵ�Դ��ť��ģʽ�л���ť��ͯ����ť����ʱ���� */
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
/* ���󰴼����ַܷ����� */
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

/* ���󰴼�ɨ��ȥ��ȷ�Ϻ��� */
void HeatDevice_KeyBoard_Confirm(void)
{   
    if (1 == g_unPressFlag.ucLock)
    {
        PORTAbits.RA5 = 1;
        
        if (0 == PORTAbits.RA2)
        {
            /* ͯ�� */        
            HeatDevice_Child_Lock_Operation();
        }
        PORTAbits.RA5 = 0;
    }
    else if (1 == g_unPressFlag.ucMode)
    {
        PORTAbits.RA5 = 1;
        if (0 == PORTAbits.RA1)
        {
            /* ģʽѡ�񣬵�Դ���ؿ���ʱ����Ч*/
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
            /* ���ز���*/
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
            /* �¶ȼӣ���Դ���ؿ���ʱ����Ч*/
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
            /* �¶ȼ�����Դ���ؿ���ʱ����Ч*/
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
            /* ��ʱ���� */
            if (!g_ucChildLockFunEnableFlag)
            {
                HeatDevice_Timer_Operation();
            }
        }
        PORTAbits.RA4 = 0;
    }

    return;
}

/* ������ָ��ַܷ� */
void HeatDevice_Infra_Red_Function_Disapatch(void)
{
    //unsigned char ucUserCodeH = 0;
    //unsigned char ucUserCodeL = 0;
    unsigned char ucFunPositiveCode = 0;
    unsigned char ucFunNegativeCode = 0;
    
    /* �Ӻ����������л�ȡ�û��룬���ܱ���ͷ����� */
   // ucUserCodeH = (unsigned char)(g_InfraData >> 24);
   // ucUserCodeL = (unsigned char)((g_InfraData & 0xff0000) >> 16);
    ucFunPositiveCode = (g_InfraData & 0xff00) >> 8;
    ucFunNegativeCode = g_InfraData & 0xff;
    
    /* �Թ��ܱ������У���� */
    if (0 != ucFunPositiveCode & ucFunNegativeCode)
    {
        /*���յ����������� */
    }
    
    /* ���ݹ��ܱ�����зַ�ִ�� */
    switch (ucFunPositiveCode)
    {
        case 0x30:
            /* ��ʱ���� */
            if (!g_ucChildLockFunEnableFlag)
            {
                HeatDevice_Timer_Operation();
            }
            break;
        case 0x70:
            /* �¶ȼӣ���Դ���ؿ���ʱ����Ч*/
            if ((1 == g_ucPowerKeyPress) && (1 == g_ucInConstantTempe) && (!g_ucChildLockFunEnableFlag))
            {
                HeatDevice_AddOrSub_Temp_Operation(1);
            }
            break;
        case 0x60:
            /* �¶ȼ�����Դ���ؿ���ʱ����Ч*/
            if ((1 == g_ucPowerKeyPress) && (1 == g_ucInConstantTempe) && (!g_ucChildLockFunEnableFlag))
            {
                HeatDevice_AddOrSub_Temp_Operation(0);
            }
            break;
        case 0x20:
            /* ͯ�� */        
            HeatDevice_Child_Lock_Operation();
            break;
        case 0x78:
            /* ģʽѡ�񣬵�Դ���ؿ���ʱ����Ч*/
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
            /* ���ز���*/
            if (!g_ucChildLockFunEnableFlag)
            {
                HeatDevice_Control_Switch_Operation();
            }
            break;
        default:
            /* �����쳣 */
            break;
    }
    
    return;
}

/* �������źŴ��� */
void HeatDevice_Infra_red_Operation(void)
{
    /* ����ǷǷ�ʱ�����ź���λ���������ͺ����� */
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
        /* �յ�һ��ǰ���� */
        g_uiInfraRedCounter = 0;
        /* ��ǰ������Ч���λ */
        g_ucStartFlagValid = 1;
        g_ucDataIndex = 31;
        
        return;      
    }
    
    if (g_ucStartFlagValid)
    {
        if ((g_uiInfraRedCounter >= 3) && (g_uiInfraRedCounter <= 6))
        {
            /* ����Ϊ0 */
            g_InfraData &= ~(1 << g_ucDataIndex);
            g_uiInfraRedCounter = 0;
            g_ucDataIndex --;
            
        }
        else if ((g_uiInfraRedCounter >= 7) && (g_uiInfraRedCounter <= 10))
        {
            /* ����Ϊ1 */
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

/* ��Դ�رպ󵹼�ʱ������ */
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
            /* �������ת�� */
            HeatDevice_Set_Device_status(g_ucCountDownValue / 10, g_ucCountDownValue % 10, 0x0, 0x20);
            
            g_ucCountDownValue --;
            g_usCountDownCounter = 0;
        }
    }
    
    return;
}

/* ��ʱ��Timer1�жϴ��� */
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
             /* ��ʮ����λ����� */           
             uiLedStatu |= 0xe;
             if (1 != g_ucLedFlashCounterFlag % 2)
             {
                 uiLedStatu &= ~((unsigned short)(0x2)); 
             }
        break;
        case 2:
             uiLedStatu = g_uiRightLedDisplayStatuValue;
             /* ����λ����� */
             uiLedStatu |= 0xe;
             if (1 != g_ucLedFlashCounterFlag % 2)
             {
                 uiLedStatu &= ~((unsigned short)(0x4)); 
             }
        break;
        case 3:
              uiLedStatu = g_uiFunLedDisplayStatuValue;
              /* ������״̬�� */
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

/* ��ʱ��Timer2�жϴ����� */
void Timer2_Isr(void)
{  
    /* ADCת����ʱ���� */
    g_usAdcCoverTimerCount ++;
    
    if (g_ucInfraRedFlag)
    {
        g_uiInfraRedCounter ++;
    }
    
    /* led����˸ʱ����� */
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
        /* 5�붨ʱ���� */
        g_usHighTempOpenCount ++;
    }
}

/* Adc�жϴ����� */
void Adc_Isr(void)
{
    unsigned short uiAdcResult = 0;
    unsigned char ucTempMapIndex = 0;
    
    /* ��ȡ��� */
    uiAdcResult = (ADRESLbits.ADRESL | (ADRESHbits.ADRESH << 8));

    if (1 == g_ucPowerKeyPress)
    {
        if (1 == g_ucInConstantTempe)
        {
            /* �ں���ģʽʱ��Ҫ���ݵ�ǰ�¶������Ƽ��ȹܵĿ��� */       
            ucTempMapIndex = (g_ucCurrentTempValue - 10) / 2;
            if (g_ausSenserRValueMap[ucTempMapIndex] < (uiAdcResult + PICF1503_ADC_TEMP_OFFSET))
            {
                /* ���� */
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
                /* �ر� */
                g_uiLeftLedDisplayStatuValue = (g_uiLeftLedDisplayStatuValue & 0xff0f)  | 0x10;
                g_uiRightLedDisplayStatuValue = (g_uiRightLedDisplayStatuValue & 0xff0f) | 0x10;
                g_uiFunLedDisplayStatuValue = (g_uiFunLedDisplayStatuValue & 0xff0f) | 0x10;
            }
        }
        else
        {
            if (631 < (uiAdcResult + PICF1503_ADC_TEMP_OFFSET))
            {
                /* ���� */
                if (2 == g_ucInConstantTempe)
                {
                    /* �¶ȸߵ� */
                    g_uiLeftLedDisplayStatuValue = (g_uiLeftLedDisplayStatuValue & 0xff0f)  | 0xe0;
                    g_uiRightLedDisplayStatuValue = (g_uiRightLedDisplayStatuValue & 0xff0f) | 0xe0;
                    g_uiFunLedDisplayStatuValue = (g_uiFunLedDisplayStatuValue & 0xff0f) | 0xe0;
                }
                else if (3 == g_ucInConstantTempe)
                {
                    /*�¶ȵ͵� */
                    g_uiLeftLedDisplayStatuValue = (g_uiLeftLedDisplayStatuValue & 0xff0f)  | 0xa0;
                    g_uiRightLedDisplayStatuValue = (g_uiRightLedDisplayStatuValue & 0xff0f) | 0xa0;
                    g_uiFunLedDisplayStatuValue = (g_uiFunLedDisplayStatuValue & 0xff0f) | 0xa0;
                }
            }
            else
            {
                /* �ر� */
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

/* Gpio�жϴ����� */
void Gpio_Isr(void)
{
    if ((1 == IOCAFbits.IOCAF0) || (1 == IOCAFbits.IOCAF1) || (1 == IOCAFbits.IOCAF2))
    {
        /* ȥʹ���ж� */
        IOCANbits.IOCAN &= 0xf8; 
        /* ���ð����ַ��������д��� */
        HeatDevice_KeyBoard_Function_disapatch();
        /* ����жϱ�ǣ�ʹ���ж� */
        IOCAFbits.IOCAF &= 0xf8;
        IOCANbits.IOCAN |= 0x7;

    }
    else if ((1 == IOCAFbits.IOCAF3))
    {
        /* ȥʹ���ж� */
        IOCANbits.IOCAN3 = 0;

		/* �����źŴ����� */
        HeatDevice_Infra_red_Operation();
        g_ucInfraRedFlag = 1;
		/* ����жϱ�ǣ�ʹ���ж� */
        IOCAFbits.IOCAF3 = 0;
        IOCANbits.IOCAN3 = 1;	       
    }
    else
    {
        /* ����Ƿ�������GPIO�ж��ϱ��򲻴���ֱ������жϱ�� */
        IOCAFbits.IOCAF &= 0xf;
    }
    
    return;
}
/* �ж������ */
void interrupt HeatDevice_Interrupt_Isr(void)
{
    if (1 == PIR1bits.TMR1IF)
    {
        /* ȥʹ�ܶ�ʱ��Timer1�ж� */
        PIE1bits.TMR1IE = 0;
        /* ����жϱ�� */
        PIR1bits.TMR1IF = 0;
        Timer1_Isr();
        /* ʹ�ܶ�ʱ��Timer1�ж� */
       PIE1bits.TMR1IE = 1;
    }
    else if (1 == PIR1bits.ADIF)
    {
        /* ȥʹ��Adc�ж� */
        PIE1bits.ADIE = 0;
        /* ����жϱ�� */
        PIR1bits.ADIF = 0;
        Adc_Isr();
        /* ʹ��Adc�ж� */
        PIE1bits.ADIE = 1;
    }
    else if (1 == INTCONbits.IOCIF)
    {
        Gpio_Isr();
    }
    else if (1 == PIR1bits.TMR2IF)
    {
        /* ȥʹ�ܶ�ʱ��Timer2�ж� */
        PIE1bits.TMR2IE = 0;
        /* ����жϱ�� */
        PIR1bits.TMR2IF = 0;
        Timer2_Isr();
        /* ʹ�ܶ�ʱ��Timer2�ж� */
       PIE1bits.TMR2IE = 1;
    }
    
    return;
}

void main(void)
{   
    unsigned char ucLastTimerValue = 0;
    /* ��ǳ�ʼ�� */
    g_unPressFlag.ucKeyBoardPressFlag = 0;
    
    /* ϵͳʱ������ */
    PicF1503_SystemTime_Config();
    /* ��ʼ��ADC */
    Pic16F1503_Adc_Init();
    
    /* ��ʼ��GPIO */
    Pic16F1503_Gpio_Init();
    
    /* ��ʼ��I2C */
   Pic16F1503_I2c_Init();
    
    /* ��ʾֵ��ʼ���������� */
	HeatDevice_Set_Device_status(16, 16, 0, 0x30);
    
    /* ��ʼ��Timer1 */
    Pic16F1503_Timer1_Init();
    
    /* ��ʼ��Timer2 */
    Pic16F1503_Timer2_Init();
    
    /* �жϳ�ʼ�� */
    Pic16F1503_Int_Init();
   
    while(1)
    {
        if (40 == g_ucTimeCounter)
        {
            /* ���ð���ɨ��ȷ�Ϻ��� */
            HeatDevice_KeyBoard_Confirm();      
           
            /* ����жϱ�ǣ�����λ��������,������� */
            g_unPressFlag.ucKeyBoardPressFlag = 0;
            g_ucTimeCounter = 0;
        }
        
        if (1 == g_ucLedFlashFlag)
        {
            /* ����˸��ǵ���1ʱ����Ҫ���Ƶ���˸ */
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
            /* ÿ10�뷢��һ��ADCת������ȡ�¸��¶�ֵ����λ����ת����ɺ��Զ���� */
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
        
        /* ��ʱʱ�䵽ִ����Ӧ���� */
        if (0 != g_ucSetTimerValue)
        {
            if (ucLastTimerValue == g_ucSetTimerValue)
            {
                if (g_usTimerFuncCount == (g_ucSetTimerValue * 360))
                {
                    g_usTimerFuncCount = 0;
                    g_ucSetTimerValue = 0;
                    /* ִ����Ӧ�Ŀ��ز��� */
                    HeatDevice_Control_Switch_Operation();
                }
            }
            else
            {
                g_usTimerFuncCount = 0;
                ucLastTimerValue = g_ucSetTimerValue;
            }
        }
        
        /* ����ʱ1���� */
        HeatDevice_Count_Down_Operation();
    };
    return;
}
