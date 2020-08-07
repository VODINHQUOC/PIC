#ifndef QMOTOR
#define QMOTOR

#include <stdint.h>

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
        
        OPTION_REG.PS2 = 1; // prescaler = 32
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
   //----------------
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
  // Check The DC Value To Make Sure it's Within 10-Bit Range
  if(dc<1024)
  {
    CCP2CON.CCP2Y = (dc) & 1;  // Get bit-0 (The LSB)
    CCP2CON.CCP2X = (dc) & 2;  // Get bit-1
    CCPR2L = dc >> 2;        // Move The 8 MSBs To CCPR1L register
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
  //Initialize the SPBRG register for the appropriate baud rate. If a high-speed baud rate is desired, set the bit BRGH
    SPBRG = 129;   // set Baurate = 9600 mbps
    TXSTA.BRGH = 1;   //High Speed

   // Enable the asynchronous serial port by clearing bit SYNC and setting bit SPEN.
    TXSTA.SYNC = 0;
    RCSTA.SPEN=  1;
   //Bits TRISC<7:6> have to be set in order to configure pins RC6/TX/CK and RC7/RX/DT as the Universal Synchronous Asynchronous Receiver Transmitter.
    TRISC.B6=1;
    TRISC.B7=1;
   //If interrupts are desired, then set the enable bit TXIE.
  //  INTCON.GIE = 1;  //Enable Global Interrupt
  //  INTCON.PEIE = 1;   //Enable PEIE Interrupt
  //  PIE1.TXIE= 1;      // Enable TX UART interrupt

    //If the 9-bit transmission is desired, then set transmit bit TX9
      //-- TXSTA.TX9 = 1;
    //Enable the transmission by setting bit TXEN, which will also set bit TXIF.
    TXSTA.TXEN = 1;
    //If the 9-bit transmission is selected, the 9th bit should be loaded in bit TX9D.
    //Load data to the TXREG register (this step automatically starts the transmission)
    //If using interrupts, ensure that GIE and PEIE (bits 7 and 6) of the INTCON register are set
   // TXREG = 0xAC;  // Load data to TXREG => Data auto load to  TSR register
  //--------------------
}

void qUART_RX_Config(void)
{
  // write to the SPBRG register in order to set the desired baud rate
  SPBRG = 129;    //Baudrate 9600 Mbps
  TXSTA.BRGH = 1;
  // Enable the asynchronous serial port
  TXSTA.SYNC = 0;
  RCSTA.SPEN=  1;
  //Set The RX-TX Pins Data Direction
  TRISC.B6=1;
  TRISC.B7=1;
  // Enable Interrupt
  INTCON.GIE = 1;  //Enable Global Interrupt
  INTCON.PEIE = 1;   //Enable PEIE Interrupt
  PIE1.RCIE= 1;      // Enable RX UART interrupt

  // Enable UART Data Reception
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

#endif