#line 1 "C:/Users/dinh quoc/Desktop/LMD driver/Control Velocity/motorv2.c"
#line 1 "c:/users/public/documents/mikroelektronika/mikroc pro for pic/include/stdint.h"




typedef signed char int8_t;
typedef signed int int16_t;
typedef signed long int int32_t;


typedef unsigned char uint8_t;
typedef unsigned int uint16_t;
typedef unsigned long int uint32_t;


typedef signed char int_least8_t;
typedef signed int int_least16_t;
typedef signed long int int_least32_t;


typedef unsigned char uint_least8_t;
typedef unsigned int uint_least16_t;
typedef unsigned long int uint_least32_t;



typedef signed char int_fast8_t;
typedef signed int int_fast16_t;
typedef signed long int int_fast32_t;


typedef unsigned char uint_fast8_t;
typedef unsigned int uint_fast16_t;
typedef unsigned long int uint_fast32_t;


typedef signed int intptr_t;
typedef unsigned int uintptr_t;


typedef signed long int intmax_t;
typedef unsigned long int uintmax_t;
#line 1 "c:/users/dinh quoc/desktop/lmd driver/control velocity/motorv2.h"
#line 1 "c:/users/public/documents/mikroelektronika/mikroc pro for pic/include/stdint.h"
#line 6 "c:/users/dinh quoc/desktop/lmd driver/control velocity/motorv2.h"
void Read_Encoder_InputCapture_ChannelAB(void);
void Timer1_RunCounter(void);
void PWM2_Config();
void PWM2_Set_Duty(uint16_t dc);

void Timer0_Config(void);
void Delay0_ms(unsigned int t);
void qUART_TX_Config(void);
void qUART_Transmit(unsigned char dataUART);
void qUART_PutString(unsigned char* s);
unsigned char qUART_ReadData();
void qUART_RX_Config(void);


uint16_t TMR1 = 0;

void Timer0_Config(void)
{
 OPTION_REG.T0CS = 0;
 OPTION_REG.PSA = 0;

 OPTION_REG.PS2 = 1;
 OPTION_REG.PS1 = 0;
 OPTION_REG.PS0 = 0;
 TMR0= 0;
 INTCON.TMR0IF = 0;
 INTCON.TMR0IE = 1;
 INTCON.GIE = 1;
 }
void Delay0_ms(unsigned int t)
{

 while(t--)
 {
 INTCON.TMR0IF=0;
 while(!INTCON.TMR0IF);
 }
}




void PWM2_Config()
{
 CCP2CON.CCP2M3= 1 ;
 CCP2CON.CCP2M2= 1 ;
 TRISC.B1= 0;
 PR2 = 155;

 T2CON.T2CKPS0 = 1;
 T2CON.T2CKPS1 = 1;
 T2CON.TMR2ON = 1;
 ADCON1 |=0x07;
 TRISA.B1 = 0;
 TRISA.B2 = 0;
 PORTA.B1=0;
 PORTA.B2=0;
 CCP2CON.CCP2Y = (0) & 1;
 CCP2CON.CCP2X = (0) & 2;
 CCPR2L = 0 >> 2;
}

void PWM2_Set_Duty(uint16_t dc)
{

 if(dc<1024)
 {
 CCP2CON.CCP2Y = (dc) & 1;
 CCP2CON.CCP2X = (dc) & 2;
 CCPR2L = dc >> 2;
 }
}
void Read_Encoder_InputCapture_ChannelAB(void)
{
 TMR1H= TMR1L=0;
 TMR1= TMR1H;
 TMR1<<=8;
 TMR1|=TMR1L;
 T1CON.T1CKPS0 = 0;
 T1CON.T1CKPS1 = 0;
 T1CON.TMR1CS = 1;
 T1CON.T1OSCEN = 0;
 T1CON.T1SYNC = 0;
 T1CON.TMR1ON = 1;
 CCPR1H=CCPR1L=0;
 CCP1CON.CCP1M0 = 1;
 CCP1CON.CCP1M1 = 0;
 CCP1CON.CCP1M2 = 1;
 CCP1CON.CCP1M3 = 0;
 TRISC.B2=1;
 TRISC.B0=1;
 TRISB.B4 = 1;
 PIE1.CCP1IE = 1;
 INTCON.PEIE = 1;
 INTCON.GIE = 1;
}

void Timer1_RunCounter(void)
{
 TMR1= TMR1H;
 TMR1<<=8;
 TMR1|=TMR1L;

}

 void qUART_TX_Config(void)
{

 SPBRG = 129;
 TXSTA.BRGH = 1;


 TXSTA.SYNC = 0;
 RCSTA.SPEN= 1;

 TRISC.B6=1;
 TRISC.B7=1;








 TXSTA.TXEN = 1;





}

void qUART_RX_Config(void)
{

 SPBRG = 129;
 TXSTA.BRGH = 1;

 TXSTA.SYNC = 0;
 RCSTA.SPEN= 1;

 TRISC.B6=1;
 TRISC.B7=1;

 INTCON.GIE = 1;
 INTCON.PEIE = 1;
 PIE1.RCIE= 1;


 RCSTA.CREN = 1;
}


void qUART_Transmit(unsigned char dataUART)
{
 while(!TXSTA.TRMT);
 TXREG = dataUART;
}
void qUART_PutString(unsigned char* s)
{
 while(*s)
 {
 qUART_Transmit(*s++);
 }
}

unsigned char qUART_ReadData()
{
 while(!PIR1.RCIF);
 return RCREG;
}
#line 4 "C:/Users/dinh quoc/Desktop/LMD driver/Control Velocity/motorv2.c"
uint8_t direct =0;
int32_t count = 0,rotary=0;

uint16_t Output_int;
uint16_t Pulse_per_second=0,cnt=0,sum_count=0;
uint16_t Velocity_PV=0,Velocity_SP=250;
double PID=0.0,Error=0.0,P_part=0.0,pre_Error=0.0,pre_I_Part=0.0,I_part=0.0,D_part=0.0,Output=0.0;
double Kp=0.01,Ki=0.001,Kd =0.0,T=0.001;
char positionUart[8];

void Motor_Control_Velocity(uint16_t SP , uint16_t PV);
void Calibrate_Motor(void);
void main() {

 ADCON1 |=0x07;
 TRISA.B3 =0;
 TRISA.B4 =0;
 TRISA.B5 =0;
 PORTA.B3 = 1;
 PORTA.B4 = 1;
 PORTA.B5 = 1;


 Calibrate_Motor();
 qUART_TX_Config();
 Timer0_Config();
 PWM2_Config();




 Read_Encoder_InputCapture_ChannelAB();



 while(1)
 {
 Motor_Control_Velocity(Velocity_SP,Velocity_PV);
 Delay_ms(1);


 }
}

void Calibrate_Motor(void)
{
 PID=0.0;
 Error=0.0;
 P_part=0.0;
 pre_Error=0.0;
 pre_I_Part=0.0;
 I_part=0.0;
 D_part=0.0;
 Output=0.0;
 Velocity_PV=0;
 Pulse_per_second=0;
 sum_count=0;
 cnt=0;
}
void Motor_Control_Velocity(uint16_t SP , uint16_t PV)
{


 Velocity_PV =(uint16_t)((double)(Pulse_per_second/330.0)*60.0) ;
 WordToStr(Velocity_PV,positionUart);
 qUART_PutString(positionUart);
 qUART_PutString(" ");

 PV = Velocity_PV;
 Error = (double)SP-(double)PV;

 I_part = pre_I_Part + Error * T;
 D_part = (Error - pre_Error)/T;
 PID = Kp*Error + Ki*I_part + Kd*D_part;
 pre_Error = Error;
 pre_I_Part = I_part;



 Output=(double)(Output+PID);

 if (Output >=333.0)
 {
 Output_int = 624;
 }
 else
 {
 Output_int = (uint16_t)((double)(Output/333.0)*624.0 );
 }
 PORTA.B1=1;
 PWM2_Set_Duty(Output_int);



 }


void Interrupt ()
{
 if (INTCON.TMR0IF == 1)
 {

 INTCON.TMR0IF = 0;
 }
 if (PIR1.CCP1IF == 1)
 {
 PORTA.B3 = !PORTA.B3;
 if (CCPR1 >=65535)
 {
 CCPR1=0;
 TMR1H= TMR1L=0;
 }

 if(PORTB.B4 == 0)
 {
 PORTA.B4 = !PORTA.B4;
 direct = 0;
 count++;
 }

 else if(PORTB.B4 == 1)
 {
 PORTA.B5 = !PORTA.B5;
 direct = 1;
 count--;
 }


 TMR0 =0;
 while(PORTC.B2 == 1)
 while(PORTC.B2 == 0)

 TMR0 =0;
 while(PORTC.B2 == 1)
 {
 if (TMR0 >= 255)
 {
 sum_count = sum_count + 256;
 TMR0 = 0;
 }
 }
 while(PORTC.B2 == 0)
 {
 if (TMR0 >= 255)
 {
 sum_count = sum_count + 256;
 TMR0 = 0;
 }
 }
 sum_count = sum_count + TMR0;

 Pulse_per_second = 156250/sum_count;


 sum_count=0;
 TMR0 = 0;


 PIR1.CCP1IF = 0;
 }


}
