#line 1 "C:/Users/dinh quoc/Desktop/LMD driver/PIC control position/Math MikroC/Motorv1.c"
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
#line 1 "c:/users/dinh quoc/desktop/lmd driver/pic control position/math mikroc/motorv1.h"
#line 1 "c:/users/public/documents/mikroelektronika/mikroc pro for pic/include/stdint.h"
#line 6 "c:/users/dinh quoc/desktop/lmd driver/pic control position/math mikroc/motorv1.h"
void Read_Encoder_InputCapture_ChannelAB(void);
void Timer1_RunCounter(void);
void PWM2_Config();
void PWM2_Set_Duty(uint16_t dc);
void Calculate_Position(double* Angles);
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
 TMR0= 100;
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
#line 4 "C:/Users/dinh quoc/Desktop/LMD driver/PIC control position/Math MikroC/Motorv1.c"
uint8_t direct =0;
int32_t count = 0,rotary=0;
int32_t Position_Angles_PV=0;
int32_t Position_PV=0,Position_SP=90;
uint16_t Output_int;

double Error=0.0,P_part=0.0,pre_Error=0.0,pre_I_Part=0.0,I_part=0.0,D_part=0.0,Output=0.0;
double Kp=1.5,Ki=0.001,Kd =0.2,T=0.001;
char positionUart[5];


void Motor_Control_Position(int32_t SP , int32_t PV);

void main() {

 ADCON1 |=0x07;
 TRISA.B3 =0;
 TRISA.B4 =0;
 TRISA.B5 =0;
 PORTA.B3 = 1;
 PORTA.B4 = 1;
 PORTA.B5 = 1;



 qUART_TX_Config();
 Timer0_Config();
 PWM2_Config();




 Read_Encoder_InputCapture_ChannelAB();



 while(1)
 {

 }
}



void Motor_Control_Position(int32_t SP , int32_t PV)
{


 Error = (double)(SP-PV);
 I_part = pre_I_Part + Error * T;
 D_part = (Error - pre_Error)/T;
 Output = Kp*Error + Ki*I_part + Kd*D_part;
 pre_Error = Error;
 pre_I_Part = I_part;


 if ((Output >=624) || (Output <= -624))
 {
 Output_int = 624;
 }
 else
 {
 Output_int = (uint16_t)(fabs(Output))+210;
 }
 if ( Error >1)
 {
 PORTA.B1=1;

 PWM2_Set_Duty(Output_int);
 }
 else if (Error<-1)
 {
 PORTA.B1=0;

 PWM2_Set_Duty(Output_int);
 }
 else
 {
 PORTA.B1=0;
 PORTA.B2=0;
 PWM2_Set_Duty(1);
 }

 }


void Interrupt ()
{

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

 if (count >= 330)
 {
 rotary++;
 count=0;
#line 125 "C:/Users/dinh quoc/Desktop/LMD driver/PIC control position/Math MikroC/Motorv1.c"
 }
 else if (count <= -330)
 {
 rotary--;
 count=0;
#line 135 "C:/Users/dinh quoc/Desktop/LMD driver/PIC control position/Math MikroC/Motorv1.c"
 }
 Position_Angles_PV =(count*360)/330;








 PIR1.CCP1IF = 0;
 }

 if (INTCON.TMR0IF == 1)
 {
 TMR0= 100;

 Motor_Control_Position(Position_SP,Position_Angles_PV);

 INTCON.TMR0IF = 0;
 }
}
