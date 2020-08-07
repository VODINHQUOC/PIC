#line 1 "C:/Users/dinh quoc/Desktop/LMD driver/Control Torque/Control Velocity/motorv2.c"
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
#line 1 "c:/users/dinh quoc/desktop/lmd driver/control torque/control velocity/motorv2.h"
#line 1 "c:/users/public/documents/mikroelektronika/mikroc pro for pic/include/stdint.h"
#line 6 "c:/users/dinh quoc/desktop/lmd driver/control torque/control velocity/motorv2.h"
void Read_current_sensor(void);
void calculate_torque(void);
void Read_Encoder_InputCapture_ChannelAB(void);
void Timer1_RunCounter(void);
void PWM2_Config();
void PWM2_Set_Duty(uint16_t dc);
void Motor_Control_Velocity(uint16_t SP , uint16_t PV);
void Calibrate_Motor(void);
void Timer0_Config(void);
void Delay0_ms(unsigned int t);
void qUART_TX_Config(void);
void qUART_Transmit(unsigned char dataUART);
void qUART_PutString(unsigned char* s);
unsigned char qUART_ReadData();
void qUART_RX_Config(void);
void qADC_Init(void);
uint16_t ADC_Read(uint8_t ADC_channel);
void Motor_Torque_Control(double SP,double PV);


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


 TRISA.B1 = 0;
 PORTA.B1=0;
 TRISA.B2 = 0;
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


void qADC_Init(void)
{
 ADCON0.ADCS1 = 1;
 ADCON0.ADCS0 = 0;
 ADCON0.CHS2 = 0;
 ADCON0.CHS1 = 0;
 ADCON0.CHS0 = 0;
 ADCON0.GO_DONE = 0;
 ADCON0.ADON = 1;

 PIR1.ADIF = 0;
 PIE1.ADIE = 1;
 INTCON.PEIE = 1;
 INTCON.GIE = 1;

 Delay_us(30);

}
uint16_t ADC_Read(uint8_t ADC_channel)
{
 if(ADC_Channel <0 || ADC_Channel >7)
 { return 0;}
 ADCON0 &= 0b11000101;
 ADCON0 |= ADC_Channel<<3;

 Delay_us(30);
 ADCON0.GO_DONE = 1;
 while(ADCON0.GO_DONE);

 return ((ADRESH << 8) + ADRESL);
}
#line 4 "C:/Users/dinh quoc/Desktop/LMD driver/Control Torque/Control Velocity/motorv2.c"
uint8_t direct =0;
int32_t count = 0,rotary=0;
uint16_t adc_value = 0;

uint16_t Pulse_per_second=0,Pulse_per_second_pre=0,cnt=0,sum_count=0;

double torque_sp=1.0,torque_pv=0.0;
double PID=0.0,Error=0.0,P_part=0.0,pre_Error=0.0,pre_I_Part=0.0,I_part=0.0,D_part=0.0,Output=0.0;
double Kp=0.025,Ki=0.0,Kd =0.000,T=0.001;
double voltage=0.0;
char positionUart[8];
char current_uart[20];
uint16_t PWM_direction =0;
double omega = 0.0;

uint16_t x=0;
void main() {

 ADCON1.PCFG3 = 1;
 ADCON1.PCFG2 = 1;
 ADCON1.PCFG1 = 1;
 ADCON1.PCFG0 = 0;

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
 PORTA.B1=1;
 PORTA.B2=0;

 PWM_direction=312;
 PWM2_Set_Duty(PWM_direction);
 Read_Encoder_InputCapture_ChannelAB();
 qADC_Init();

 while(1)
 {

 calculate_torque();
 Motor_Torque_Control(torque_sp,torque_pv);

 Delay_ms(1);

 }
}

void Read_current_sensor(void)
{
 adc_value=ADC_Read(0);

}
void calculate_torque(void)
{
 voltage = (double)((double)adc_value/65536.0)*5.0;
 omega = (double)(Pulse_per_second*2.0*3.14/330.0);
 torque_pv = (double)((voltage/4700.0)/0.000377)*(double)((12.0*((double)PWM_direction-312.0)/312.0)/omega);

 FloatToStr((float)torque_pv,current_uart);
 qUART_PutString(current_uart);
 qUART_PutString(" ");






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
 Pulse_per_second=0;
 sum_count=0;
 cnt=0;
 PWM_direction=312;
 omega = 1.0;
 torque_pv=0.0;
}


void Motor_Torque_Control(double SP,double PV)
{

 Error = SP-PV;
 I_part = pre_I_Part + Error * T;
 D_part = (Error - pre_Error)/T;
 PID = Kp*Error + Ki*I_part + Kd*D_part;
 pre_Error = Error;
 pre_I_Part = I_part;

 Output=(double)(Output+PID);
 if (Output >=3.5)
 {
 PWM_direction = 624;
 PWM2_Set_Duty(PWM_direction);
 }
 else if ( Output <= 0.0)
 {
 PWM_direction = 312;
 PWM2_Set_Duty(PWM_direction);
 }
 else
 {
 PWM_direction = 312+(uint16_t)(Output*312.0/3.5);
 PWM2_Set_Duty(PWM_direction);





 }
}


void Interrupt ()
{

 if (INTCON.TMR0IF == 1)
 {

 INTCON.TMR0IF = 0;
 }

 if (PIR1.ADIF == 1)
 {




 PIR1.ADIF = 0;
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
 Pulse_per_second_pre = Pulse_per_second;
 Pulse_per_second = 156250/sum_count;
 if (Pulse_per_second >2000 )
 {
 Pulse_per_second = Pulse_per_second_pre;
 }




 sum_count=0;
 TMR0 = 0;

 Read_current_sensor();
 PIR1.CCP1IF = 0;
 }

}
