#line 1 "C:/Users/dinh quoc/Desktop/Math MikroC/math.c"
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
#line 2 "C:/Users/dinh quoc/Desktop/Math MikroC/math.c"
double a;
int b =20;

int16_t Rotary=10,count = 4;
double Position_PV = 0;
double Error=0.0,P_part=0.0,pre_Error=0.0,pre_I_Part=0.0,I_part=0.0,D_part=0.0,Output=0.0;
int32_t SP,PV;
double Ki=0,Kp=0,Kd=0,T=0.001;

void Maths(void);
void main() {
 a=1/(double)b*9;
 Maths();
}
void Maths(void)
{




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
