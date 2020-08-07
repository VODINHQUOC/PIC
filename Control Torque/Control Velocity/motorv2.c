#include <stdint.h>
#include "motorv2.h"

uint8_t direct =0;
int32_t count = 0,rotary=0;
uint16_t adc_value = 0;

uint16_t Pulse_per_second=0,Pulse_per_second_pre=0,cnt=0,sum_count=0;

double torque_sp=1.0,torque_pv=0.0;
double  PID=0.0,Error=0.0,P_part=0.0,pre_Error=0.0,pre_I_Part=0.0,I_part=0.0,D_part=0.0,Output=0.0;
double  Kp=0.025,Ki=0.0,Kd =0.000,T=0.001; //Reccommended value
double  voltage=0.0;
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
    PORTA.B1=1;  //PWM
    PORTA.B2=0;

    PWM_direction=312;
    PWM2_Set_Duty(PWM_direction); //Direction
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
  voltage =  (double)((double)adc_value/65536.0)*5.0;
  omega = (double)(Pulse_per_second*2.0*3.14/330.0);
  torque_pv =  (double)((voltage/4700.0)/0.000377)*(double)((12.0*((double)PWM_direction-312.0)/312.0)/omega);   //N.m
   // Max torque =1.8/4700/0.000377*12*34.87 =0.49 kq/cm
  FloatToStr((float)torque_pv,current_uart);
  qUART_PutString(current_uart);
  qUART_PutString(" ");
 // qUART_PutString("\n");
  //---------Show Value--------------
  //FloatToStr((float)omega,current_uart);
  //qUART_PutString(current_uart);
  //qUART_PutString(" ");
  
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
        if (Output >=3.5) // kg/cm
        {
          PWM_direction = 624;
          PWM2_Set_Duty(PWM_direction);
        }         Output
        else if (  <= 0.0)
        {
          PWM_direction = 312;
          PWM2_Set_Duty(PWM_direction);
        } // N/m
        else
        {
          PWM_direction = 312+(uint16_t)(Output*312.0/3.5);
          PWM2_Set_Duty(PWM_direction);
          
          
         // WordToStr(PWM_direction,positionUart);
        //  qUART_PutString(positionUart);
         // qUART_PutString(" ");
        }
}


void Interrupt ()
{

  if (INTCON.TMR0IF == 1)
  {

     INTCON.TMR0IF = 0;
  }//TMR0

  if (PIR1.ADIF == 1)
  {
    //PORTA.B3 = !PORTA.B3;
    //PORTA.B4 = !PORTA.B4;

    
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
     //====================Clockwise=================
     if(PORTB.B4 == 0)
     {
        PORTA.B4 = !PORTA.B4;
        direct = 0;
        count++;
     }
     //====================CounterClockwise==========
     else if(PORTB.B4 == 1)
     {
        PORTA.B5 = !PORTA.B5;
        direct = 1;
        count--;
     }

     //---------------------
    TMR0 =0;
    while(PORTC.B2 ==  1)
    while(PORTC.B2 ==  0)

    TMR0 =0;
    while(PORTC.B2 ==  1)
    {
     if (TMR0 >= 255)
     {
       sum_count = sum_count + 256;
       TMR0 = 0;
     }
    }
    while(PORTC.B2 ==  0)
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
       Pulse_per_second =  Pulse_per_second_pre;
     }
     //WordToStr(Pulse_per_second,positionUart);
     //qUART_PutString(positionUart);
     //qUART_PutString(" ");

     sum_count=0;
     TMR0 = 0;
    //-------------------------------
     Read_current_sensor();
     PIR1.CCP1IF = 0; // Clear Flag Capture
  }//CCP1

}//Interrupt