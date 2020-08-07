#include <stdint.h>
#include "motorv2.h"

uint8_t direct =0;
int32_t count = 0,rotary=0;

uint16_t Output_int;
uint16_t Pulse_per_second=0,cnt=0,sum_count=0;
uint16_t Velocity_PV=0,Velocity_SP=250;
double  PID=0.0,Error=0.0,P_part=0.0,pre_Error=0.0,pre_I_Part=0.0,I_part=0.0,D_part=0.0,Output=0.0;
double  Kp=0.01,Ki=0.001,Kd =0.0,T=0.001; //Reccommended value
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
  //  PORTA.B1=1; //Direction

  //
  //  PWM2_Set_Duty(400);
    Read_Encoder_InputCapture_ChannelAB();



  while(1)
  {
    Motor_Control_Velocity(Velocity_SP,Velocity_PV);
    Delay_ms(1);
     //PORTA.B4 = !PORTA.B4;

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
void Motor_Control_Velocity(uint16_t SP /* rpm */ , uint16_t PV)
{

        //---------PID Algorthim-------------
         Velocity_PV =(uint16_t)((double)(Pulse_per_second/330.0)*60.0)  ;
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
        //-------------------------------------------
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
         //Delay(T) if use while(1)
         

 }


void Interrupt ()
{
     if (INTCON.TMR0IF == 1)
     {

       INTCON.TMR0IF = 0;
     }//TMR0
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

     Pulse_per_second = 156250/sum_count;
     

     sum_count=0;
     TMR0 = 0;
    //-------------------------------

       PIR1.CCP1IF = 0; // Clear Flag Capture
  }//CCP1


}//Interrupt