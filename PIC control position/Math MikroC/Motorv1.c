#include <stdint.h>
#include "Motorv1.h"

uint8_t direct =0;
int32_t count = 0,rotary=0;
int32_t Position_Angles_PV=0;
int32_t Position_PV=0,Position_SP=90;
uint16_t Output_int;

double  Error=0.0,P_part=0.0,pre_Error=0.0,pre_I_Part=0.0,I_part=0.0,D_part=0.0,Output=0.0;
double  Kp=1.5,Ki=0.001,Kd =0.2,T=0.001; //Reccommended value
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
  //  PORTA.B1=1;
  //  PORTA.B2=0;

  //  PWM2_Set_Duty(210);
    Read_Encoder_InputCapture_ChannelAB();



  while(1)
  {
    //Motor_Control_Position(Position_SP,Position_Angles_PV);
  }
}



void Motor_Control_Position(int32_t SP , int32_t PV)
{
        //---------PID Algorthim-------------

         Error = (double)(SP-PV);
         I_part = pre_I_Part + Error * T;
         D_part = (Error - pre_Error)/T;
         Output = Kp*Error + Ki*I_part + Kd*D_part;
         pre_Error = Error;
         pre_I_Part = I_part;

        //-------------------------------------------
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
           // PORTA.B2=0;
            PWM2_Set_Duty(Output_int);
        }
        else if  (Error<-1)  //2 degree
        {
            PORTA.B1=0;
          //  PORTA.B2=1;
            PWM2_Set_Duty(Output_int);
        }
        else
        {
            PORTA.B1=0;
            PORTA.B2=0;
            PWM2_Set_Duty(1);
        }
         //Delay(T) if use while(1)
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

     if (count >= 330)
     {
        rotary++;
        count=0;
        /*
        IntToStr(rotary,positionUart);
        qUART_PutString(positionUart);
        qUART_PutString("\n");
        */
     }
     else if (count <= -330)
     {
        rotary--;
        count=0;
        /*
        IntToStr(rotary,positionUart);
        qUART_PutString(positionUart);
        qUART_PutString("\n");
        */
     }
     Position_Angles_PV =(count*360)/330;

    // IntToStr(Position_Angles_PV,positionUart);
    // qUART_PutString(positionUart);
    // qUART_PutString("\n");




     PIR1.CCP1IF = 0; // Clear Flag Capture
  }//CCP1

   if (INTCON.TMR0IF == 1)
     {
       TMR0= 100;
       //PORTA.B4 = !PORTA.B4;
        Motor_Control_Position(Position_SP,Position_Angles_PV);

       INTCON.TMR0IF = 0;
     }//TMR0
}//Interrupt