
_Timer0_Config:

;motorv2.h,28 :: 		void Timer0_Config(void)
;motorv2.h,30 :: 		OPTION_REG.T0CS = 0;
	BCF        OPTION_REG+0, 5
;motorv2.h,31 :: 		OPTION_REG.PSA = 0;
	BCF        OPTION_REG+0, 3
;motorv2.h,33 :: 		OPTION_REG.PS2 = 1; // prescaler = 32
	BSF        OPTION_REG+0, 2
;motorv2.h,34 :: 		OPTION_REG.PS1 = 0;
	BCF        OPTION_REG+0, 1
;motorv2.h,35 :: 		OPTION_REG.PS0 = 0;
	BCF        OPTION_REG+0, 0
;motorv2.h,36 :: 		TMR0= 0;
	CLRF       TMR0+0
;motorv2.h,37 :: 		INTCON.TMR0IF = 0;
	BCF        INTCON+0, 2
;motorv2.h,38 :: 		INTCON.TMR0IE = 1;
	BSF        INTCON+0, 5
;motorv2.h,39 :: 		INTCON.GIE = 1;
	BSF        INTCON+0, 7
;motorv2.h,40 :: 		}
L_end_Timer0_Config:
	RETURN
; end of _Timer0_Config

_Delay0_ms:

;motorv2.h,41 :: 		void Delay0_ms(unsigned int t)
;motorv2.h,44 :: 		while(t--)
L_Delay0_ms0:
	MOVF       FARG_Delay0_ms_t+0, 0
	MOVWF      R0+0
	MOVF       FARG_Delay0_ms_t+1, 0
	MOVWF      R0+1
	MOVLW      1
	SUBWF      FARG_Delay0_ms_t+0, 1
	BTFSS      STATUS+0, 0
	DECF       FARG_Delay0_ms_t+1, 1
	MOVF       R0+0, 0
	IORWF      R0+1, 0
	BTFSC      STATUS+0, 2
	GOTO       L_Delay0_ms1
;motorv2.h,46 :: 		INTCON.TMR0IF=0;
	BCF        INTCON+0, 2
;motorv2.h,47 :: 		while(!INTCON.TMR0IF);
L_Delay0_ms2:
	BTFSC      INTCON+0, 2
	GOTO       L_Delay0_ms3
	GOTO       L_Delay0_ms2
L_Delay0_ms3:
;motorv2.h,48 :: 		}
	GOTO       L_Delay0_ms0
L_Delay0_ms1:
;motorv2.h,49 :: 		}
L_end_Delay0_ms:
	RETURN
; end of _Delay0_ms

_PWM2_Config:

;motorv2.h,54 :: 		void PWM2_Config()
;motorv2.h,56 :: 		CCP2CON.CCP2M3= 1 ;
	BSF        CCP2CON+0, 3
;motorv2.h,57 :: 		CCP2CON.CCP2M2= 1 ;
	BSF        CCP2CON+0, 2
;motorv2.h,58 :: 		TRISC.B1= 0;
	BCF        TRISC+0, 1
;motorv2.h,59 :: 		PR2 = 155;
	MOVLW      155
	MOVWF      PR2+0
;motorv2.h,61 :: 		T2CON.T2CKPS0 = 1;
	BSF        T2CON+0, 0
;motorv2.h,62 :: 		T2CON.T2CKPS1 = 1;
	BSF        T2CON+0, 1
;motorv2.h,63 :: 		T2CON.TMR2ON = 1;
	BSF        T2CON+0, 2
;motorv2.h,66 :: 		TRISA.B1 = 0;
	BCF        TRISA+0, 1
;motorv2.h,67 :: 		PORTA.B1=0;
	BCF        PORTA+0, 1
;motorv2.h,68 :: 		TRISA.B2 = 0;
	BCF        TRISA+0, 2
;motorv2.h,69 :: 		PORTA.B2=0;
	BCF        PORTA+0, 2
;motorv2.h,70 :: 		CCP2CON.CCP2Y = (0) & 1;
	BCF        CCP2CON+0, 4
;motorv2.h,71 :: 		CCP2CON.CCP2X = (0) & 2;
	BCF        CCP2CON+0, 5
;motorv2.h,72 :: 		CCPR2L = 0 >> 2;
	CLRF       CCPR2L+0
;motorv2.h,73 :: 		}
L_end_PWM2_Config:
	RETURN
; end of _PWM2_Config

_PWM2_Set_Duty:

;motorv2.h,75 :: 		void PWM2_Set_Duty(uint16_t dc)
;motorv2.h,78 :: 		if(dc<1024)
	MOVLW      4
	SUBWF      FARG_PWM2_Set_Duty_dc+1, 0
	BTFSS      STATUS+0, 2
	GOTO       L__PWM2_Set_Duty48
	MOVLW      0
	SUBWF      FARG_PWM2_Set_Duty_dc+0, 0
L__PWM2_Set_Duty48:
	BTFSC      STATUS+0, 0
	GOTO       L_PWM2_Set_Duty4
;motorv2.h,80 :: 		CCP2CON.CCP2Y = (dc) & 1;  // Get bit-0 (The LSB)
	MOVLW      1
	ANDWF      FARG_PWM2_Set_Duty_dc+0, 0
	MOVWF      R0+0
	BTFSC      R0+0, 0
	GOTO       L__PWM2_Set_Duty49
	BCF        CCP2CON+0, 4
	GOTO       L__PWM2_Set_Duty50
L__PWM2_Set_Duty49:
	BSF        CCP2CON+0, 4
L__PWM2_Set_Duty50:
;motorv2.h,81 :: 		CCP2CON.CCP2X = (dc) & 2;  // Get bit-1
	MOVLW      2
	ANDWF      FARG_PWM2_Set_Duty_dc+0, 0
	MOVWF      R0+0
	BTFSC      R0+0, 0
	GOTO       L__PWM2_Set_Duty51
	BCF        CCP2CON+0, 5
	GOTO       L__PWM2_Set_Duty52
L__PWM2_Set_Duty51:
	BSF        CCP2CON+0, 5
L__PWM2_Set_Duty52:
;motorv2.h,82 :: 		CCPR2L = dc >> 2;        // Move The 8 MSBs To CCPR1L register
	MOVF       FARG_PWM2_Set_Duty_dc+0, 0
	MOVWF      R0+0
	MOVF       FARG_PWM2_Set_Duty_dc+1, 0
	MOVWF      R0+1
	RRF        R0+1, 1
	RRF        R0+0, 1
	BCF        R0+1, 7
	RRF        R0+1, 1
	RRF        R0+0, 1
	BCF        R0+1, 7
	MOVF       R0+0, 0
	MOVWF      CCPR2L+0
;motorv2.h,83 :: 		}
L_PWM2_Set_Duty4:
;motorv2.h,84 :: 		}
L_end_PWM2_Set_Duty:
	RETURN
; end of _PWM2_Set_Duty

_Read_Encoder_InputCapture_ChannelAB:

;motorv2.h,85 :: 		void Read_Encoder_InputCapture_ChannelAB(void)
;motorv2.h,87 :: 		TMR1H= TMR1L=0;
	CLRF       TMR1L+0
	MOVF       TMR1L+0, 0
	MOVWF      TMR1H+0
;motorv2.h,88 :: 		TMR1= TMR1H;
	MOVF       TMR1H+0, 0
	MOVWF      _TMR1+0
	CLRF       _TMR1+1
;motorv2.h,89 :: 		TMR1<<=8;
	MOVF       _TMR1+0, 0
	MOVWF      _TMR1+1
	CLRF       _TMR1+0
;motorv2.h,90 :: 		TMR1|=TMR1L;
	MOVF       TMR1L+0, 0
	IORWF      _TMR1+0, 1
	MOVLW      0
	IORWF      _TMR1+1, 1
;motorv2.h,91 :: 		T1CON.T1CKPS0 = 0;
	BCF        T1CON+0, 4
;motorv2.h,92 :: 		T1CON.T1CKPS1 = 0;
	BCF        T1CON+0, 5
;motorv2.h,93 :: 		T1CON.TMR1CS = 1;
	BSF        T1CON+0, 1
;motorv2.h,94 :: 		T1CON.T1OSCEN = 0;
	BCF        T1CON+0, 3
;motorv2.h,95 :: 		T1CON.T1SYNC = 0;
	BCF        T1CON+0, 2
;motorv2.h,96 :: 		T1CON.TMR1ON = 1;
	BSF        T1CON+0, 0
;motorv2.h,97 :: 		CCPR1H=CCPR1L=0;
	CLRF       CCPR1L+0
	MOVF       CCPR1L+0, 0
	MOVWF      CCPR1H+0
;motorv2.h,98 :: 		CCP1CON.CCP1M0 = 1;
	BSF        CCP1CON+0, 0
;motorv2.h,99 :: 		CCP1CON.CCP1M1 = 0;
	BCF        CCP1CON+0, 1
;motorv2.h,100 :: 		CCP1CON.CCP1M2 = 1;
	BSF        CCP1CON+0, 2
;motorv2.h,101 :: 		CCP1CON.CCP1M3 = 0;
	BCF        CCP1CON+0, 3
;motorv2.h,102 :: 		TRISC.B2=1;
	BSF        TRISC+0, 2
;motorv2.h,103 :: 		TRISC.B0=1;
	BSF        TRISC+0, 0
;motorv2.h,105 :: 		TRISB.B4 = 1;
	BSF        TRISB+0, 4
;motorv2.h,108 :: 		PIE1.CCP1IE = 1;
	BSF        PIE1+0, 2
;motorv2.h,109 :: 		INTCON.PEIE = 1;
	BSF        INTCON+0, 6
;motorv2.h,110 :: 		INTCON.GIE = 1;
	BSF        INTCON+0, 7
;motorv2.h,111 :: 		}
L_end_Read_Encoder_InputCapture_ChannelAB:
	RETURN
; end of _Read_Encoder_InputCapture_ChannelAB

_Timer1_RunCounter:

;motorv2.h,113 :: 		void Timer1_RunCounter(void)
;motorv2.h,115 :: 		TMR1= TMR1H;
	MOVF       TMR1H+0, 0
	MOVWF      _TMR1+0
	CLRF       _TMR1+1
;motorv2.h,116 :: 		TMR1<<=8;
	MOVF       _TMR1+0, 0
	MOVWF      _TMR1+1
	CLRF       _TMR1+0
;motorv2.h,117 :: 		TMR1|=TMR1L;
	MOVF       TMR1L+0, 0
	IORWF      _TMR1+0, 1
	MOVLW      0
	IORWF      _TMR1+1, 1
;motorv2.h,119 :: 		}
L_end_Timer1_RunCounter:
	RETURN
; end of _Timer1_RunCounter

_qUART_TX_Config:

;motorv2.h,121 :: 		void qUART_TX_Config(void)
;motorv2.h,124 :: 		SPBRG = 129;   // set Baurate = 9600 mbps
	MOVLW      129
	MOVWF      SPBRG+0
;motorv2.h,125 :: 		TXSTA.BRGH = 1;   //High Speed
	BSF        TXSTA+0, 2
;motorv2.h,128 :: 		TXSTA.SYNC = 0;
	BCF        TXSTA+0, 4
;motorv2.h,129 :: 		RCSTA.SPEN=  1;
	BSF        RCSTA+0, 7
;motorv2.h,131 :: 		TRISC.B6=1;
	BSF        TRISC+0, 6
;motorv2.h,132 :: 		TRISC.B7=1;
	BSF        TRISC+0, 7
;motorv2.h,141 :: 		TXSTA.TXEN = 1;
	BSF        TXSTA+0, 5
;motorv2.h,147 :: 		}
L_end_qUART_TX_Config:
	RETURN
; end of _qUART_TX_Config

_qUART_RX_Config:

;motorv2.h,149 :: 		void qUART_RX_Config(void)
;motorv2.h,152 :: 		SPBRG = 129;    //Baudrate 9600 Mbps
	MOVLW      129
	MOVWF      SPBRG+0
;motorv2.h,153 :: 		TXSTA.BRGH = 1;
	BSF        TXSTA+0, 2
;motorv2.h,155 :: 		TXSTA.SYNC = 0;
	BCF        TXSTA+0, 4
;motorv2.h,156 :: 		RCSTA.SPEN=  1;
	BSF        RCSTA+0, 7
;motorv2.h,158 :: 		TRISC.B6=1;
	BSF        TRISC+0, 6
;motorv2.h,159 :: 		TRISC.B7=1;
	BSF        TRISC+0, 7
;motorv2.h,161 :: 		INTCON.GIE = 1;  //Enable Global Interrupt
	BSF        INTCON+0, 7
;motorv2.h,162 :: 		INTCON.PEIE = 1;   //Enable PEIE Interrupt
	BSF        INTCON+0, 6
;motorv2.h,163 :: 		PIE1.RCIE= 1;      // Enable RX UART interrupt
	BSF        PIE1+0, 5
;motorv2.h,166 :: 		RCSTA.CREN = 1;
	BSF        RCSTA+0, 4
;motorv2.h,167 :: 		}
L_end_qUART_RX_Config:
	RETURN
; end of _qUART_RX_Config

_qUART_Transmit:

;motorv2.h,170 :: 		void qUART_Transmit(unsigned char dataUART)
;motorv2.h,172 :: 		while(!TXSTA.TRMT);
L_qUART_Transmit5:
	BTFSC      TXSTA+0, 1
	GOTO       L_qUART_Transmit6
	GOTO       L_qUART_Transmit5
L_qUART_Transmit6:
;motorv2.h,173 :: 		TXREG = dataUART;
	MOVF       FARG_qUART_Transmit_dataUART+0, 0
	MOVWF      TXREG+0
;motorv2.h,174 :: 		}
L_end_qUART_Transmit:
	RETURN
; end of _qUART_Transmit

_qUART_PutString:

;motorv2.h,175 :: 		void qUART_PutString(unsigned char* s)
;motorv2.h,177 :: 		while(*s)
L_qUART_PutString7:
	MOVF       FARG_qUART_PutString_s+0, 0
	MOVWF      FSR
	MOVF       INDF+0, 0
	BTFSC      STATUS+0, 2
	GOTO       L_qUART_PutString8
;motorv2.h,179 :: 		qUART_Transmit(*s++);
	MOVF       FARG_qUART_PutString_s+0, 0
	MOVWF      FSR
	MOVF       INDF+0, 0
	MOVWF      FARG_qUART_Transmit_dataUART+0
	CALL       _qUART_Transmit+0
	INCF       FARG_qUART_PutString_s+0, 1
;motorv2.h,180 :: 		}
	GOTO       L_qUART_PutString7
L_qUART_PutString8:
;motorv2.h,181 :: 		}
L_end_qUART_PutString:
	RETURN
; end of _qUART_PutString

_qUART_ReadData:

;motorv2.h,183 :: 		unsigned char qUART_ReadData()
;motorv2.h,185 :: 		while(!PIR1.RCIF);
L_qUART_ReadData9:
	BTFSC      PIR1+0, 5
	GOTO       L_qUART_ReadData10
	GOTO       L_qUART_ReadData9
L_qUART_ReadData10:
;motorv2.h,186 :: 		return RCREG;
	MOVF       RCREG+0, 0
	MOVWF      R0+0
;motorv2.h,187 :: 		}
L_end_qUART_ReadData:
	RETURN
; end of _qUART_ReadData

_qADC_Init:

;motorv2.h,190 :: 		void qADC_Init(void)
;motorv2.h,192 :: 		ADCON0.ADCS1 = 1;  // T sample = 64Tosc
	BSF        ADCON0+0, 7
;motorv2.h,193 :: 		ADCON0.ADCS0 = 0;
	BCF        ADCON0+0, 6
;motorv2.h,194 :: 		ADCON0.CHS2 = 0;
	BCF        ADCON0+0, 5
;motorv2.h,195 :: 		ADCON0.CHS1 = 0;    // Channel 0 AN0
	BCF        ADCON0+0, 4
;motorv2.h,196 :: 		ADCON0.CHS0 = 0;
	BCF        ADCON0+0, 3
;motorv2.h,197 :: 		ADCON0.GO_DONE = 0;  //Stop conversion
	BCF        ADCON0+0, 2
;motorv2.h,198 :: 		ADCON0.ADON = 1;   // start adc
	BSF        ADCON0+0, 0
;motorv2.h,200 :: 		PIR1.ADIF = 0;  // Clear ADC Interrupt Flag
	BCF        PIR1+0, 6
;motorv2.h,201 :: 		PIE1.ADIE = 1;  // ADC Interrupt Enable Bit
	BSF        PIE1+0, 6
;motorv2.h,202 :: 		INTCON.PEIE = 1;  // Peripherals Interrupt Enable Bit
	BSF        INTCON+0, 6
;motorv2.h,203 :: 		INTCON.GIE = 1;   // Global Interrupts Enable Bit
	BSF        INTCON+0, 7
;motorv2.h,205 :: 		Delay_us(30); //wait 30 us
	MOVLW      49
	MOVWF      R13+0
L_qADC_Init11:
	DECFSZ     R13+0, 1
	GOTO       L_qADC_Init11
	NOP
	NOP
;motorv2.h,207 :: 		}
L_end_qADC_Init:
	RETURN
; end of _qADC_Init

_ADC_Read:

;motorv2.h,208 :: 		uint16_t ADC_Read(uint8_t ADC_channel)
;motorv2.h,210 :: 		if(ADC_Channel <0 || ADC_Channel >7)    // Check Channel Number Validity
	MOVLW      0
	SUBWF      FARG_ADC_Read_ADC_channel+0, 0
	BTFSS      STATUS+0, 0
	GOTO       L__ADC_Read43
	MOVF       FARG_ADC_Read_ADC_channel+0, 0
	SUBLW      7
	BTFSS      STATUS+0, 0
	GOTO       L__ADC_Read43
	GOTO       L_ADC_Read14
L__ADC_Read43:
;motorv2.h,211 :: 		{ return 0;}
	CLRF       R0+0
	CLRF       R0+1
	GOTO       L_end_ADC_Read
L_ADC_Read14:
;motorv2.h,212 :: 		ADCON0 &= 0b11000101; // Clear The Channel Selection Bits
	MOVLW      197
	ANDWF      ADCON0+0, 1
;motorv2.h,213 :: 		ADCON0 |= ADC_Channel<<3;     // Select The Required Channel (ANC)
	MOVF       FARG_ADC_Read_ADC_channel+0, 0
	MOVWF      R0+0
	RLF        R0+0, 1
	BCF        R0+0, 0
	RLF        R0+0, 1
	BCF        R0+0, 0
	RLF        R0+0, 1
	BCF        R0+0, 0
	MOVF       R0+0, 0
	IORWF      ADCON0+0, 1
;motorv2.h,215 :: 		Delay_us(30);       // The Minimum Tacq = 20us, So That should be enough
	MOVLW      49
	MOVWF      R13+0
L_ADC_Read15:
	DECFSZ     R13+0, 1
	GOTO       L_ADC_Read15
	NOP
	NOP
;motorv2.h,216 :: 		ADCON0.GO_DONE = 1;          // Start A/D Conversion
	BSF        ADCON0+0, 2
;motorv2.h,217 :: 		while(ADCON0.GO_DONE); // Polling GO_DONE Bit
L_ADC_Read16:
	BTFSS      ADCON0+0, 2
	GOTO       L_ADC_Read17
	GOTO       L_ADC_Read16
L_ADC_Read17:
;motorv2.h,219 :: 		return ((ADRESH << 8) + ADRESL); // Return The Right-Justified 10-Bit Result
	MOVF       ADRESH+0, 0
	MOVWF      R0+1
	CLRF       R0+0
	MOVF       ADRESL+0, 0
	ADDWF      R0+0, 1
	BTFSC      STATUS+0, 0
	INCF       R0+1, 1
;motorv2.h,220 :: 		}
L_end_ADC_Read:
	RETURN
; end of _ADC_Read

_main:

;motorv2.c,20 :: 		void main() {
;motorv2.c,22 :: 		ADCON1.PCFG3 = 1;
	BSF        ADCON1+0, 3
;motorv2.c,23 :: 		ADCON1.PCFG2 = 1;
	BSF        ADCON1+0, 2
;motorv2.c,24 :: 		ADCON1.PCFG1 = 1;
	BSF        ADCON1+0, 1
;motorv2.c,25 :: 		ADCON1.PCFG0 = 0;
	BCF        ADCON1+0, 0
;motorv2.c,27 :: 		TRISA.B3 =0;
	BCF        TRISA+0, 3
;motorv2.c,28 :: 		TRISA.B4 =0;
	BCF        TRISA+0, 4
;motorv2.c,29 :: 		TRISA.B5 =0;
	BCF        TRISA+0, 5
;motorv2.c,30 :: 		PORTA.B3 = 1;
	BSF        PORTA+0, 3
;motorv2.c,31 :: 		PORTA.B4 = 1;
	BSF        PORTA+0, 4
;motorv2.c,32 :: 		PORTA.B5 = 1;
	BSF        PORTA+0, 5
;motorv2.c,34 :: 		Calibrate_Motor();
	CALL       _Calibrate_Motor+0
;motorv2.c,35 :: 		qUART_TX_Config();
	CALL       _qUART_TX_Config+0
;motorv2.c,37 :: 		Timer0_Config();
	CALL       _Timer0_Config+0
;motorv2.c,38 :: 		PWM2_Config();
	CALL       _PWM2_Config+0
;motorv2.c,39 :: 		PORTA.B1=1;  //PWM
	BSF        PORTA+0, 1
;motorv2.c,40 :: 		PORTA.B2=0;
	BCF        PORTA+0, 2
;motorv2.c,42 :: 		PWM_direction=312;
	MOVLW      56
	MOVWF      _PWM_direction+0
	MOVLW      1
	MOVWF      _PWM_direction+1
;motorv2.c,43 :: 		PWM2_Set_Duty(PWM_direction); //Direction
	MOVLW      56
	MOVWF      FARG_PWM2_Set_Duty_dc+0
	MOVLW      1
	MOVWF      FARG_PWM2_Set_Duty_dc+1
	CALL       _PWM2_Set_Duty+0
;motorv2.c,44 :: 		Read_Encoder_InputCapture_ChannelAB();
	CALL       _Read_Encoder_InputCapture_ChannelAB+0
;motorv2.c,45 :: 		qADC_Init();
	CALL       _qADC_Init+0
;motorv2.c,47 :: 		while(1)
L_main18:
;motorv2.c,50 :: 		calculate_torque();
	CALL       _calculate_torque+0
;motorv2.c,51 :: 		Motor_Torque_Control(torque_sp,torque_pv);
	MOVF       _torque_sp+0, 0
	MOVWF      FARG_Motor_Torque_Control_SP+0
	MOVF       _torque_sp+1, 0
	MOVWF      FARG_Motor_Torque_Control_SP+1
	MOVF       _torque_sp+2, 0
	MOVWF      FARG_Motor_Torque_Control_SP+2
	MOVF       _torque_sp+3, 0
	MOVWF      FARG_Motor_Torque_Control_SP+3
	MOVF       _torque_pv+0, 0
	MOVWF      FARG_Motor_Torque_Control_PV+0
	MOVF       _torque_pv+1, 0
	MOVWF      FARG_Motor_Torque_Control_PV+1
	MOVF       _torque_pv+2, 0
	MOVWF      FARG_Motor_Torque_Control_PV+2
	MOVF       _torque_pv+3, 0
	MOVWF      FARG_Motor_Torque_Control_PV+3
	CALL       _Motor_Torque_Control+0
;motorv2.c,53 :: 		Delay_ms(1);
	MOVLW      7
	MOVWF      R12+0
	MOVLW      125
	MOVWF      R13+0
L_main20:
	DECFSZ     R13+0, 1
	GOTO       L_main20
	DECFSZ     R12+0, 1
	GOTO       L_main20
;motorv2.c,55 :: 		}
	GOTO       L_main18
;motorv2.c,56 :: 		}
L_end_main:
	GOTO       $+0
; end of _main

_Read_current_sensor:

;motorv2.c,58 :: 		void Read_current_sensor(void)
;motorv2.c,60 :: 		adc_value=ADC_Read(0);
	CLRF       FARG_ADC_Read_ADC_channel+0
	CALL       _ADC_Read+0
	MOVF       R0+0, 0
	MOVWF      _adc_value+0
	MOVF       R0+1, 0
	MOVWF      _adc_value+1
;motorv2.c,62 :: 		}
L_end_Read_current_sensor:
	RETURN
; end of _Read_current_sensor

_calculate_torque:

;motorv2.c,63 :: 		void calculate_torque(void)
;motorv2.c,65 :: 		voltage =  (double)((double)adc_value/65536.0)*5.0;
	MOVF       _adc_value+0, 0
	MOVWF      R0+0
	MOVF       _adc_value+1, 0
	MOVWF      R0+1
	CALL       _word2double+0
	MOVLW      0
	MOVWF      R4+0
	MOVLW      0
	MOVWF      R4+1
	MOVLW      0
	MOVWF      R4+2
	MOVLW      143
	MOVWF      R4+3
	CALL       _Div_32x32_FP+0
	MOVLW      0
	MOVWF      R4+0
	MOVLW      0
	MOVWF      R4+1
	MOVLW      32
	MOVWF      R4+2
	MOVLW      129
	MOVWF      R4+3
	CALL       _Mul_32x32_FP+0
	MOVF       R0+0, 0
	MOVWF      FLOC__calculate_torque+0
	MOVF       R0+1, 0
	MOVWF      FLOC__calculate_torque+1
	MOVF       R0+2, 0
	MOVWF      FLOC__calculate_torque+2
	MOVF       R0+3, 0
	MOVWF      FLOC__calculate_torque+3
	MOVF       FLOC__calculate_torque+0, 0
	MOVWF      _voltage+0
	MOVF       FLOC__calculate_torque+1, 0
	MOVWF      _voltage+1
	MOVF       FLOC__calculate_torque+2, 0
	MOVWF      _voltage+2
	MOVF       FLOC__calculate_torque+3, 0
	MOVWF      _voltage+3
;motorv2.c,66 :: 		omega = (double)(Pulse_per_second*2.0*3.14/330.0);
	MOVF       _Pulse_per_second+0, 0
	MOVWF      R0+0
	MOVF       _Pulse_per_second+1, 0
	MOVWF      R0+1
	CALL       _word2double+0
	MOVLW      0
	MOVWF      R4+0
	MOVLW      0
	MOVWF      R4+1
	MOVLW      0
	MOVWF      R4+2
	MOVLW      128
	MOVWF      R4+3
	CALL       _Mul_32x32_FP+0
	MOVLW      195
	MOVWF      R4+0
	MOVLW      245
	MOVWF      R4+1
	MOVLW      72
	MOVWF      R4+2
	MOVLW      128
	MOVWF      R4+3
	CALL       _Mul_32x32_FP+0
	MOVLW      0
	MOVWF      R4+0
	MOVLW      0
	MOVWF      R4+1
	MOVLW      37
	MOVWF      R4+2
	MOVLW      135
	MOVWF      R4+3
	CALL       _Div_32x32_FP+0
	MOVF       R0+0, 0
	MOVWF      FLOC__calculate_torque+4
	MOVF       R0+1, 0
	MOVWF      FLOC__calculate_torque+5
	MOVF       R0+2, 0
	MOVWF      FLOC__calculate_torque+6
	MOVF       R0+3, 0
	MOVWF      FLOC__calculate_torque+7
	MOVF       FLOC__calculate_torque+4, 0
	MOVWF      _omega+0
	MOVF       FLOC__calculate_torque+5, 0
	MOVWF      _omega+1
	MOVF       FLOC__calculate_torque+6, 0
	MOVWF      _omega+2
	MOVF       FLOC__calculate_torque+7, 0
	MOVWF      _omega+3
;motorv2.c,67 :: 		torque_pv =  (double)((voltage/4700.0)/0.000377)*(double)((12.0*((double)PWM_direction-312.0)/312.0)/omega);   //N.m
	MOVLW      0
	MOVWF      R4+0
	MOVLW      224
	MOVWF      R4+1
	MOVLW      18
	MOVWF      R4+2
	MOVLW      139
	MOVWF      R4+3
	MOVF       FLOC__calculate_torque+0, 0
	MOVWF      R0+0
	MOVF       FLOC__calculate_torque+1, 0
	MOVWF      R0+1
	MOVF       FLOC__calculate_torque+2, 0
	MOVWF      R0+2
	MOVF       FLOC__calculate_torque+3, 0
	MOVWF      R0+3
	CALL       _Div_32x32_FP+0
	MOVLW      21
	MOVWF      R4+0
	MOVLW      168
	MOVWF      R4+1
	MOVLW      69
	MOVWF      R4+2
	MOVLW      115
	MOVWF      R4+3
	CALL       _Div_32x32_FP+0
	MOVF       R0+0, 0
	MOVWF      FLOC__calculate_torque+0
	MOVF       R0+1, 0
	MOVWF      FLOC__calculate_torque+1
	MOVF       R0+2, 0
	MOVWF      FLOC__calculate_torque+2
	MOVF       R0+3, 0
	MOVWF      FLOC__calculate_torque+3
	MOVF       _PWM_direction+0, 0
	MOVWF      R0+0
	MOVF       _PWM_direction+1, 0
	MOVWF      R0+1
	CALL       _word2double+0
	MOVLW      0
	MOVWF      R4+0
	MOVLW      0
	MOVWF      R4+1
	MOVLW      28
	MOVWF      R4+2
	MOVLW      135
	MOVWF      R4+3
	CALL       _Sub_32x32_FP+0
	MOVLW      0
	MOVWF      R4+0
	MOVLW      0
	MOVWF      R4+1
	MOVLW      64
	MOVWF      R4+2
	MOVLW      130
	MOVWF      R4+3
	CALL       _Mul_32x32_FP+0
	MOVLW      0
	MOVWF      R4+0
	MOVLW      0
	MOVWF      R4+1
	MOVLW      28
	MOVWF      R4+2
	MOVLW      135
	MOVWF      R4+3
	CALL       _Div_32x32_FP+0
	MOVF       FLOC__calculate_torque+4, 0
	MOVWF      R4+0
	MOVF       FLOC__calculate_torque+5, 0
	MOVWF      R4+1
	MOVF       FLOC__calculate_torque+6, 0
	MOVWF      R4+2
	MOVF       FLOC__calculate_torque+7, 0
	MOVWF      R4+3
	CALL       _Div_32x32_FP+0
	MOVF       FLOC__calculate_torque+0, 0
	MOVWF      R4+0
	MOVF       FLOC__calculate_torque+1, 0
	MOVWF      R4+1
	MOVF       FLOC__calculate_torque+2, 0
	MOVWF      R4+2
	MOVF       FLOC__calculate_torque+3, 0
	MOVWF      R4+3
	CALL       _Mul_32x32_FP+0
	MOVF       R0+0, 0
	MOVWF      _torque_pv+0
	MOVF       R0+1, 0
	MOVWF      _torque_pv+1
	MOVF       R0+2, 0
	MOVWF      _torque_pv+2
	MOVF       R0+3, 0
	MOVWF      _torque_pv+3
;motorv2.c,69 :: 		FloatToStr((float)torque_pv,current_uart);
	MOVF       R0+0, 0
	MOVWF      FARG_FloatToStr_fnum+0
	MOVF       R0+1, 0
	MOVWF      FARG_FloatToStr_fnum+1
	MOVF       R0+2, 0
	MOVWF      FARG_FloatToStr_fnum+2
	MOVF       R0+3, 0
	MOVWF      FARG_FloatToStr_fnum+3
	MOVLW      _current_uart+0
	MOVWF      FARG_FloatToStr_str+0
	CALL       _FloatToStr+0
;motorv2.c,70 :: 		qUART_PutString(current_uart);
	MOVLW      _current_uart+0
	MOVWF      FARG_qUART_PutString_s+0
	CALL       _qUART_PutString+0
;motorv2.c,71 :: 		qUART_PutString(" ");
	MOVLW      ?lstr1_motorv2+0
	MOVWF      FARG_qUART_PutString_s+0
	CALL       _qUART_PutString+0
;motorv2.c,78 :: 		}
L_end_calculate_torque:
	RETURN
; end of _calculate_torque

_Calibrate_Motor:

;motorv2.c,79 :: 		void Calibrate_Motor(void)
;motorv2.c,81 :: 		PID=0.0;
	CLRF       _PID+0
	CLRF       _PID+1
	CLRF       _PID+2
	CLRF       _PID+3
;motorv2.c,82 :: 		Error=0.0;
	CLRF       _Error+0
	CLRF       _Error+1
	CLRF       _Error+2
	CLRF       _Error+3
;motorv2.c,83 :: 		P_part=0.0;
	CLRF       _P_part+0
	CLRF       _P_part+1
	CLRF       _P_part+2
	CLRF       _P_part+3
;motorv2.c,84 :: 		pre_Error=0.0;
	CLRF       _pre_Error+0
	CLRF       _pre_Error+1
	CLRF       _pre_Error+2
	CLRF       _pre_Error+3
;motorv2.c,85 :: 		pre_I_Part=0.0;
	CLRF       _pre_I_Part+0
	CLRF       _pre_I_Part+1
	CLRF       _pre_I_Part+2
	CLRF       _pre_I_Part+3
;motorv2.c,86 :: 		I_part=0.0;
	CLRF       _I_part+0
	CLRF       _I_part+1
	CLRF       _I_part+2
	CLRF       _I_part+3
;motorv2.c,87 :: 		D_part=0.0;
	CLRF       _D_part+0
	CLRF       _D_part+1
	CLRF       _D_part+2
	CLRF       _D_part+3
;motorv2.c,88 :: 		Output=0.0;
	CLRF       _Output+0
	CLRF       _Output+1
	CLRF       _Output+2
	CLRF       _Output+3
;motorv2.c,89 :: 		Pulse_per_second=0;
	CLRF       _Pulse_per_second+0
	CLRF       _Pulse_per_second+1
;motorv2.c,90 :: 		sum_count=0;
	CLRF       _sum_count+0
	CLRF       _sum_count+1
;motorv2.c,91 :: 		cnt=0;
	CLRF       _cnt+0
	CLRF       _cnt+1
;motorv2.c,92 :: 		PWM_direction=312;
	MOVLW      56
	MOVWF      _PWM_direction+0
	MOVLW      1
	MOVWF      _PWM_direction+1
;motorv2.c,93 :: 		omega = 1.0;
	MOVLW      0
	MOVWF      _omega+0
	MOVLW      0
	MOVWF      _omega+1
	MOVLW      0
	MOVWF      _omega+2
	MOVLW      127
	MOVWF      _omega+3
;motorv2.c,94 :: 		torque_pv=0.0;
	CLRF       _torque_pv+0
	CLRF       _torque_pv+1
	CLRF       _torque_pv+2
	CLRF       _torque_pv+3
;motorv2.c,95 :: 		}
L_end_Calibrate_Motor:
	RETURN
; end of _Calibrate_Motor

_Motor_Torque_Control:

;motorv2.c,98 :: 		void Motor_Torque_Control(double SP,double PV)
;motorv2.c,101 :: 		Error = SP-PV;
	MOVF       FARG_Motor_Torque_Control_PV+0, 0
	MOVWF      R4+0
	MOVF       FARG_Motor_Torque_Control_PV+1, 0
	MOVWF      R4+1
	MOVF       FARG_Motor_Torque_Control_PV+2, 0
	MOVWF      R4+2
	MOVF       FARG_Motor_Torque_Control_PV+3, 0
	MOVWF      R4+3
	MOVF       FARG_Motor_Torque_Control_SP+0, 0
	MOVWF      R0+0
	MOVF       FARG_Motor_Torque_Control_SP+1, 0
	MOVWF      R0+1
	MOVF       FARG_Motor_Torque_Control_SP+2, 0
	MOVWF      R0+2
	MOVF       FARG_Motor_Torque_Control_SP+3, 0
	MOVWF      R0+3
	CALL       _Sub_32x32_FP+0
	MOVF       R0+0, 0
	MOVWF      FLOC__Motor_Torque_Control+12
	MOVF       R0+1, 0
	MOVWF      FLOC__Motor_Torque_Control+13
	MOVF       R0+2, 0
	MOVWF      FLOC__Motor_Torque_Control+14
	MOVF       R0+3, 0
	MOVWF      FLOC__Motor_Torque_Control+15
	MOVF       FLOC__Motor_Torque_Control+12, 0
	MOVWF      _Error+0
	MOVF       FLOC__Motor_Torque_Control+13, 0
	MOVWF      _Error+1
	MOVF       FLOC__Motor_Torque_Control+14, 0
	MOVWF      _Error+2
	MOVF       FLOC__Motor_Torque_Control+15, 0
	MOVWF      _Error+3
;motorv2.c,102 :: 		I_part = pre_I_Part + Error * T;
	MOVF       FLOC__Motor_Torque_Control+12, 0
	MOVWF      R0+0
	MOVF       FLOC__Motor_Torque_Control+13, 0
	MOVWF      R0+1
	MOVF       FLOC__Motor_Torque_Control+14, 0
	MOVWF      R0+2
	MOVF       FLOC__Motor_Torque_Control+15, 0
	MOVWF      R0+3
	MOVF       _T+0, 0
	MOVWF      R4+0
	MOVF       _T+1, 0
	MOVWF      R4+1
	MOVF       _T+2, 0
	MOVWF      R4+2
	MOVF       _T+3, 0
	MOVWF      R4+3
	CALL       _Mul_32x32_FP+0
	MOVF       _pre_I_Part+0, 0
	MOVWF      R4+0
	MOVF       _pre_I_Part+1, 0
	MOVWF      R4+1
	MOVF       _pre_I_Part+2, 0
	MOVWF      R4+2
	MOVF       _pre_I_Part+3, 0
	MOVWF      R4+3
	CALL       _Add_32x32_FP+0
	MOVF       R0+0, 0
	MOVWF      FLOC__Motor_Torque_Control+8
	MOVF       R0+1, 0
	MOVWF      FLOC__Motor_Torque_Control+9
	MOVF       R0+2, 0
	MOVWF      FLOC__Motor_Torque_Control+10
	MOVF       R0+3, 0
	MOVWF      FLOC__Motor_Torque_Control+11
	MOVF       FLOC__Motor_Torque_Control+8, 0
	MOVWF      _I_part+0
	MOVF       FLOC__Motor_Torque_Control+9, 0
	MOVWF      _I_part+1
	MOVF       FLOC__Motor_Torque_Control+10, 0
	MOVWF      _I_part+2
	MOVF       FLOC__Motor_Torque_Control+11, 0
	MOVWF      _I_part+3
;motorv2.c,103 :: 		D_part = (Error - pre_Error)/T;
	MOVF       _pre_Error+0, 0
	MOVWF      R4+0
	MOVF       _pre_Error+1, 0
	MOVWF      R4+1
	MOVF       _pre_Error+2, 0
	MOVWF      R4+2
	MOVF       _pre_Error+3, 0
	MOVWF      R4+3
	MOVF       FLOC__Motor_Torque_Control+12, 0
	MOVWF      R0+0
	MOVF       FLOC__Motor_Torque_Control+13, 0
	MOVWF      R0+1
	MOVF       FLOC__Motor_Torque_Control+14, 0
	MOVWF      R0+2
	MOVF       FLOC__Motor_Torque_Control+15, 0
	MOVWF      R0+3
	CALL       _Sub_32x32_FP+0
	MOVF       _T+0, 0
	MOVWF      R4+0
	MOVF       _T+1, 0
	MOVWF      R4+1
	MOVF       _T+2, 0
	MOVWF      R4+2
	MOVF       _T+3, 0
	MOVWF      R4+3
	CALL       _Div_32x32_FP+0
	MOVF       R0+0, 0
	MOVWF      FLOC__Motor_Torque_Control+4
	MOVF       R0+1, 0
	MOVWF      FLOC__Motor_Torque_Control+5
	MOVF       R0+2, 0
	MOVWF      FLOC__Motor_Torque_Control+6
	MOVF       R0+3, 0
	MOVWF      FLOC__Motor_Torque_Control+7
	MOVF       FLOC__Motor_Torque_Control+4, 0
	MOVWF      _D_part+0
	MOVF       FLOC__Motor_Torque_Control+5, 0
	MOVWF      _D_part+1
	MOVF       FLOC__Motor_Torque_Control+6, 0
	MOVWF      _D_part+2
	MOVF       FLOC__Motor_Torque_Control+7, 0
	MOVWF      _D_part+3
;motorv2.c,104 :: 		PID = Kp*Error + Ki*I_part + Kd*D_part;
	MOVF       _Kp+0, 0
	MOVWF      R0+0
	MOVF       _Kp+1, 0
	MOVWF      R0+1
	MOVF       _Kp+2, 0
	MOVWF      R0+2
	MOVF       _Kp+3, 0
	MOVWF      R0+3
	MOVF       FLOC__Motor_Torque_Control+12, 0
	MOVWF      R4+0
	MOVF       FLOC__Motor_Torque_Control+13, 0
	MOVWF      R4+1
	MOVF       FLOC__Motor_Torque_Control+14, 0
	MOVWF      R4+2
	MOVF       FLOC__Motor_Torque_Control+15, 0
	MOVWF      R4+3
	CALL       _Mul_32x32_FP+0
	MOVF       R0+0, 0
	MOVWF      FLOC__Motor_Torque_Control+0
	MOVF       R0+1, 0
	MOVWF      FLOC__Motor_Torque_Control+1
	MOVF       R0+2, 0
	MOVWF      FLOC__Motor_Torque_Control+2
	MOVF       R0+3, 0
	MOVWF      FLOC__Motor_Torque_Control+3
	MOVF       _Ki+0, 0
	MOVWF      R0+0
	MOVF       _Ki+1, 0
	MOVWF      R0+1
	MOVF       _Ki+2, 0
	MOVWF      R0+2
	MOVF       _Ki+3, 0
	MOVWF      R0+3
	MOVF       FLOC__Motor_Torque_Control+8, 0
	MOVWF      R4+0
	MOVF       FLOC__Motor_Torque_Control+9, 0
	MOVWF      R4+1
	MOVF       FLOC__Motor_Torque_Control+10, 0
	MOVWF      R4+2
	MOVF       FLOC__Motor_Torque_Control+11, 0
	MOVWF      R4+3
	CALL       _Mul_32x32_FP+0
	MOVF       FLOC__Motor_Torque_Control+0, 0
	MOVWF      R4+0
	MOVF       FLOC__Motor_Torque_Control+1, 0
	MOVWF      R4+1
	MOVF       FLOC__Motor_Torque_Control+2, 0
	MOVWF      R4+2
	MOVF       FLOC__Motor_Torque_Control+3, 0
	MOVWF      R4+3
	CALL       _Add_32x32_FP+0
	MOVF       R0+0, 0
	MOVWF      FLOC__Motor_Torque_Control+0
	MOVF       R0+1, 0
	MOVWF      FLOC__Motor_Torque_Control+1
	MOVF       R0+2, 0
	MOVWF      FLOC__Motor_Torque_Control+2
	MOVF       R0+3, 0
	MOVWF      FLOC__Motor_Torque_Control+3
	MOVF       _Kd+0, 0
	MOVWF      R0+0
	MOVF       _Kd+1, 0
	MOVWF      R0+1
	MOVF       _Kd+2, 0
	MOVWF      R0+2
	MOVF       _Kd+3, 0
	MOVWF      R0+3
	MOVF       FLOC__Motor_Torque_Control+4, 0
	MOVWF      R4+0
	MOVF       FLOC__Motor_Torque_Control+5, 0
	MOVWF      R4+1
	MOVF       FLOC__Motor_Torque_Control+6, 0
	MOVWF      R4+2
	MOVF       FLOC__Motor_Torque_Control+7, 0
	MOVWF      R4+3
	CALL       _Mul_32x32_FP+0
	MOVF       FLOC__Motor_Torque_Control+0, 0
	MOVWF      R4+0
	MOVF       FLOC__Motor_Torque_Control+1, 0
	MOVWF      R4+1
	MOVF       FLOC__Motor_Torque_Control+2, 0
	MOVWF      R4+2
	MOVF       FLOC__Motor_Torque_Control+3, 0
	MOVWF      R4+3
	CALL       _Add_32x32_FP+0
	MOVF       R0+0, 0
	MOVWF      _PID+0
	MOVF       R0+1, 0
	MOVWF      _PID+1
	MOVF       R0+2, 0
	MOVWF      _PID+2
	MOVF       R0+3, 0
	MOVWF      _PID+3
;motorv2.c,105 :: 		pre_Error = Error;
	MOVF       FLOC__Motor_Torque_Control+12, 0
	MOVWF      _pre_Error+0
	MOVF       FLOC__Motor_Torque_Control+13, 0
	MOVWF      _pre_Error+1
	MOVF       FLOC__Motor_Torque_Control+14, 0
	MOVWF      _pre_Error+2
	MOVF       FLOC__Motor_Torque_Control+15, 0
	MOVWF      _pre_Error+3
;motorv2.c,106 :: 		pre_I_Part = I_part;
	MOVF       FLOC__Motor_Torque_Control+8, 0
	MOVWF      _pre_I_Part+0
	MOVF       FLOC__Motor_Torque_Control+9, 0
	MOVWF      _pre_I_Part+1
	MOVF       FLOC__Motor_Torque_Control+10, 0
	MOVWF      _pre_I_Part+2
	MOVF       FLOC__Motor_Torque_Control+11, 0
	MOVWF      _pre_I_Part+3
;motorv2.c,108 :: 		Output=(double)(Output+PID);
	MOVF       _Output+0, 0
	MOVWF      R4+0
	MOVF       _Output+1, 0
	MOVWF      R4+1
	MOVF       _Output+2, 0
	MOVWF      R4+2
	MOVF       _Output+3, 0
	MOVWF      R4+3
	CALL       _Add_32x32_FP+0
	MOVF       R0+0, 0
	MOVWF      _Output+0
	MOVF       R0+1, 0
	MOVWF      _Output+1
	MOVF       R0+2, 0
	MOVWF      _Output+2
	MOVF       R0+3, 0
	MOVWF      _Output+3
;motorv2.c,109 :: 		if (Output >=3.5) // kg/cm
	MOVLW      0
	MOVWF      R4+0
	MOVLW      0
	MOVWF      R4+1
	MOVLW      96
	MOVWF      R4+2
	MOVLW      128
	MOVWF      R4+3
	CALL       _Compare_Double+0
	MOVLW      1
	BTFSS      STATUS+0, 0
	MOVLW      0
	MOVWF      R0+0
	MOVF       R0+0, 0
	BTFSC      STATUS+0, 2
	GOTO       L_Motor_Torque_Control21
;motorv2.c,111 :: 		PWM_direction = 624;
	MOVLW      112
	MOVWF      _PWM_direction+0
	MOVLW      2
	MOVWF      _PWM_direction+1
;motorv2.c,112 :: 		PWM2_Set_Duty(PWM_direction);
	MOVLW      112
	MOVWF      FARG_PWM2_Set_Duty_dc+0
	MOVLW      2
	MOVWF      FARG_PWM2_Set_Duty_dc+1
	CALL       _PWM2_Set_Duty+0
;motorv2.c,113 :: 		}
	GOTO       L_Motor_Torque_Control22
L_Motor_Torque_Control21:
;motorv2.c,114 :: 		else if ( Output <= 0.0)
	MOVF       _Output+0, 0
	MOVWF      R4+0
	MOVF       _Output+1, 0
	MOVWF      R4+1
	MOVF       _Output+2, 0
	MOVWF      R4+2
	MOVF       _Output+3, 0
	MOVWF      R4+3
	CLRF       R0+0
	CLRF       R0+1
	CLRF       R0+2
	CLRF       R0+3
	CALL       _Compare_Double+0
	MOVLW      1
	BTFSS      STATUS+0, 0
	MOVLW      0
	MOVWF      R0+0
	MOVF       R0+0, 0
	BTFSC      STATUS+0, 2
	GOTO       L_Motor_Torque_Control23
;motorv2.c,116 :: 		PWM_direction = 312;
	MOVLW      56
	MOVWF      _PWM_direction+0
	MOVLW      1
	MOVWF      _PWM_direction+1
;motorv2.c,117 :: 		PWM2_Set_Duty(PWM_direction);
	MOVLW      56
	MOVWF      FARG_PWM2_Set_Duty_dc+0
	MOVLW      1
	MOVWF      FARG_PWM2_Set_Duty_dc+1
	CALL       _PWM2_Set_Duty+0
;motorv2.c,118 :: 		} // N/m
	GOTO       L_Motor_Torque_Control24
L_Motor_Torque_Control23:
;motorv2.c,121 :: 		PWM_direction = 312+(uint16_t)(Output*312.0/3.5);
	MOVF       _Output+0, 0
	MOVWF      R0+0
	MOVF       _Output+1, 0
	MOVWF      R0+1
	MOVF       _Output+2, 0
	MOVWF      R0+2
	MOVF       _Output+3, 0
	MOVWF      R0+3
	MOVLW      0
	MOVWF      R4+0
	MOVLW      0
	MOVWF      R4+1
	MOVLW      28
	MOVWF      R4+2
	MOVLW      135
	MOVWF      R4+3
	CALL       _Mul_32x32_FP+0
	MOVLW      0
	MOVWF      R4+0
	MOVLW      0
	MOVWF      R4+1
	MOVLW      96
	MOVWF      R4+2
	MOVLW      128
	MOVWF      R4+3
	CALL       _Div_32x32_FP+0
	CALL       _double2word+0
	MOVLW      56
	ADDWF      R0+0, 1
	MOVLW      1
	BTFSC      STATUS+0, 0
	ADDLW      1
	ADDWF      R0+1, 1
	MOVF       R0+0, 0
	MOVWF      _PWM_direction+0
	MOVF       R0+1, 0
	MOVWF      _PWM_direction+1
;motorv2.c,122 :: 		PWM2_Set_Duty(PWM_direction);
	MOVF       R0+0, 0
	MOVWF      FARG_PWM2_Set_Duty_dc+0
	MOVF       R0+1, 0
	MOVWF      FARG_PWM2_Set_Duty_dc+1
	CALL       _PWM2_Set_Duty+0
;motorv2.c,128 :: 		}
L_Motor_Torque_Control24:
L_Motor_Torque_Control22:
;motorv2.c,129 :: 		}
L_end_Motor_Torque_Control:
	RETURN
; end of _Motor_Torque_Control

_Interrupt:
	MOVWF      R15+0
	SWAPF      STATUS+0, 0
	CLRF       STATUS+0
	MOVWF      ___saveSTATUS+0
	MOVF       PCLATH+0, 0
	MOVWF      ___savePCLATH+0
	CLRF       PCLATH+0

;motorv2.c,132 :: 		void Interrupt ()
;motorv2.c,135 :: 		if (INTCON.TMR0IF == 1)
	BTFSS      INTCON+0, 2
	GOTO       L_Interrupt25
;motorv2.c,138 :: 		INTCON.TMR0IF = 0;
	BCF        INTCON+0, 2
;motorv2.c,139 :: 		}//TMR0
L_Interrupt25:
;motorv2.c,141 :: 		if (PIR1.ADIF == 1)
	BTFSS      PIR1+0, 6
	GOTO       L_Interrupt26
;motorv2.c,147 :: 		PIR1.ADIF = 0;
	BCF        PIR1+0, 6
;motorv2.c,148 :: 		}
L_Interrupt26:
;motorv2.c,150 :: 		if (PIR1.CCP1IF == 1)
	BTFSS      PIR1+0, 2
	GOTO       L_Interrupt27
;motorv2.c,152 :: 		PORTA.B3 = !PORTA.B3;
	MOVLW      8
	XORWF      PORTA+0, 1
;motorv2.c,153 :: 		if (CCPR1 >=65535)
	MOVLW      255
	SUBWF      CCPR1+1, 0
	BTFSS      STATUS+0, 2
	GOTO       L__Interrupt69
	MOVLW      255
	SUBWF      CCPR1+0, 0
L__Interrupt69:
	BTFSS      STATUS+0, 0
	GOTO       L_Interrupt28
;motorv2.c,155 :: 		CCPR1=0;
	CLRF       CCPR1+0
	CLRF       CCPR1+1
;motorv2.c,156 :: 		TMR1H= TMR1L=0;
	CLRF       TMR1L+0
	MOVF       TMR1L+0, 0
	MOVWF      TMR1H+0
;motorv2.c,157 :: 		}
L_Interrupt28:
;motorv2.c,159 :: 		if(PORTB.B4 == 0)
	BTFSC      PORTB+0, 4
	GOTO       L_Interrupt29
;motorv2.c,161 :: 		PORTA.B4 = !PORTA.B4;
	MOVLW      16
	XORWF      PORTA+0, 1
;motorv2.c,162 :: 		direct = 0;
	CLRF       _direct+0
;motorv2.c,163 :: 		count++;
	MOVF       _count+0, 0
	MOVWF      R0+0
	MOVF       _count+1, 0
	MOVWF      R0+1
	MOVF       _count+2, 0
	MOVWF      R0+2
	MOVF       _count+3, 0
	MOVWF      R0+3
	INCF       R0+0, 1
	BTFSC      STATUS+0, 2
	INCF       R0+1, 1
	BTFSC      STATUS+0, 2
	INCF       R0+2, 1
	BTFSC      STATUS+0, 2
	INCF       R0+3, 1
	MOVF       R0+0, 0
	MOVWF      _count+0
	MOVF       R0+1, 0
	MOVWF      _count+1
	MOVF       R0+2, 0
	MOVWF      _count+2
	MOVF       R0+3, 0
	MOVWF      _count+3
;motorv2.c,164 :: 		}
	GOTO       L_Interrupt30
L_Interrupt29:
;motorv2.c,166 :: 		else if(PORTB.B4 == 1)
	BTFSS      PORTB+0, 4
	GOTO       L_Interrupt31
;motorv2.c,168 :: 		PORTA.B5 = !PORTA.B5;
	MOVLW      32
	XORWF      PORTA+0, 1
;motorv2.c,169 :: 		direct = 1;
	MOVLW      1
	MOVWF      _direct+0
;motorv2.c,170 :: 		count--;
	MOVLW      1
	SUBWF      _count+0, 1
	BTFSS      STATUS+0, 0
	SUBWF      _count+1, 1
	BTFSS      STATUS+0, 0
	SUBWF      _count+2, 1
	BTFSS      STATUS+0, 0
	SUBWF      _count+3, 1
;motorv2.c,171 :: 		}
L_Interrupt31:
L_Interrupt30:
;motorv2.c,174 :: 		TMR0 =0;
	CLRF       TMR0+0
;motorv2.c,175 :: 		while(PORTC.B2 ==  1)
L_Interrupt32:
	BTFSS      PORTC+0, 2
	GOTO       L_Interrupt33
;motorv2.c,176 :: 		while(PORTC.B2 ==  0)
L_Interrupt34:
	BTFSC      PORTC+0, 2
	GOTO       L_Interrupt35
;motorv2.c,178 :: 		TMR0 =0;
	CLRF       TMR0+0
	GOTO       L_Interrupt34
L_Interrupt35:
	GOTO       L_Interrupt32
L_Interrupt33:
;motorv2.c,179 :: 		while(PORTC.B2 ==  1)
L_Interrupt36:
	BTFSS      PORTC+0, 2
	GOTO       L_Interrupt37
;motorv2.c,181 :: 		if (TMR0 >= 255)
	MOVLW      255
	SUBWF      TMR0+0, 0
	BTFSS      STATUS+0, 0
	GOTO       L_Interrupt38
;motorv2.c,183 :: 		sum_count = sum_count + 256;
	MOVLW      0
	ADDWF      _sum_count+0, 1
	MOVLW      1
	BTFSC      STATUS+0, 0
	ADDLW      1
	ADDWF      _sum_count+1, 1
;motorv2.c,184 :: 		TMR0 = 0;
	CLRF       TMR0+0
;motorv2.c,185 :: 		}
L_Interrupt38:
;motorv2.c,186 :: 		}
	GOTO       L_Interrupt36
L_Interrupt37:
;motorv2.c,187 :: 		while(PORTC.B2 ==  0)
L_Interrupt39:
	BTFSC      PORTC+0, 2
	GOTO       L_Interrupt40
;motorv2.c,189 :: 		if (TMR0 >= 255)
	MOVLW      255
	SUBWF      TMR0+0, 0
	BTFSS      STATUS+0, 0
	GOTO       L_Interrupt41
;motorv2.c,191 :: 		sum_count = sum_count + 256;
	MOVLW      0
	ADDWF      _sum_count+0, 1
	MOVLW      1
	BTFSC      STATUS+0, 0
	ADDLW      1
	ADDWF      _sum_count+1, 1
;motorv2.c,192 :: 		TMR0 = 0;
	CLRF       TMR0+0
;motorv2.c,193 :: 		}
L_Interrupt41:
;motorv2.c,194 :: 		}
	GOTO       L_Interrupt39
L_Interrupt40:
;motorv2.c,195 :: 		sum_count = sum_count + TMR0;
	MOVF       TMR0+0, 0
	ADDWF      _sum_count+0, 0
	MOVWF      R4+0
	MOVF       _sum_count+1, 0
	BTFSC      STATUS+0, 0
	ADDLW      1
	MOVWF      R4+1
	MOVF       R4+0, 0
	MOVWF      _sum_count+0
	MOVF       R4+1, 0
	MOVWF      _sum_count+1
;motorv2.c,196 :: 		Pulse_per_second_pre = Pulse_per_second;
	MOVF       _Pulse_per_second+0, 0
	MOVWF      _Pulse_per_second_pre+0
	MOVF       _Pulse_per_second+1, 0
	MOVWF      _Pulse_per_second_pre+1
;motorv2.c,197 :: 		Pulse_per_second = 156250/sum_count;
	MOVLW      0
	MOVWF      R4+2
	MOVWF      R4+3
	MOVLW      90
	MOVWF      R0+0
	MOVLW      98
	MOVWF      R0+1
	MOVLW      2
	MOVWF      R0+2
	MOVLW      0
	MOVWF      R0+3
	CALL       _Div_32x32_S+0
	MOVF       R0+0, 0
	MOVWF      _Pulse_per_second+0
	MOVF       R0+1, 0
	MOVWF      _Pulse_per_second+1
;motorv2.c,198 :: 		if (Pulse_per_second >2000 )
	MOVF       R0+1, 0
	SUBLW      7
	BTFSS      STATUS+0, 2
	GOTO       L__Interrupt70
	MOVF       R0+0, 0
	SUBLW      208
L__Interrupt70:
	BTFSC      STATUS+0, 0
	GOTO       L_Interrupt42
;motorv2.c,200 :: 		Pulse_per_second =  Pulse_per_second_pre;
	MOVF       _Pulse_per_second_pre+0, 0
	MOVWF      _Pulse_per_second+0
	MOVF       _Pulse_per_second_pre+1, 0
	MOVWF      _Pulse_per_second+1
;motorv2.c,201 :: 		}
L_Interrupt42:
;motorv2.c,206 :: 		sum_count=0;
	CLRF       _sum_count+0
	CLRF       _sum_count+1
;motorv2.c,207 :: 		TMR0 = 0;
	CLRF       TMR0+0
;motorv2.c,209 :: 		Read_current_sensor();
	CALL       _Read_current_sensor+0
;motorv2.c,210 :: 		PIR1.CCP1IF = 0; // Clear Flag Capture
	BCF        PIR1+0, 2
;motorv2.c,211 :: 		}//CCP1
L_Interrupt27:
;motorv2.c,213 :: 		}//Interrupt
L_end_Interrupt:
L__Interrupt68:
	MOVF       ___savePCLATH+0, 0
	MOVWF      PCLATH+0
	SWAPF      ___saveSTATUS+0, 0
	MOVWF      STATUS+0
	SWAPF      R15+0, 1
	SWAPF      R15+0, 0
	RETFIE
; end of _Interrupt
