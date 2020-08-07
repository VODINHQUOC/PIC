#include <stdint.h>
double a;
int b =20;

int16_t Rotary=10,count = 4;
double Position_PV = 0;
double  Error=0.0,P_part=0.0,pre_Error=0.0,pre_I_Part=0.0,I_part=0.0,D_part=0.0,Output=0.0;
int32_t SP,PV;
double Ki=0,Kp=0,Kd=0,T=0.001;

void Maths(void);
void main() {
   a=1/(double)b*9;
   Maths();
}
void Maths(void)
{
   //Position_PV = ((double)(Rotary) + (double)(count)/334)*360;
    // Position_PV =(double)(Rotary+count);
   //Position_PV =( error  - pre_error)/T;

   while(1)
        {

         Error = (double)(SP-PV);
         I_part = pre_I_Part + Error * T;
         D_part = (Error - pre_Error)/T;
         Output = Kp*Error + Ki*I_part + Kd*D_part;
         pre_Error = Error;
         pre_I_Part = I_part;
         }
}