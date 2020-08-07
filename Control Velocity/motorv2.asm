
_Timer0_Config:

;motorv2.h,22 :: 		void Timer0_Config(void)
;motorv2.h,24 :: 		OPTION_REG.T0CS = 0;
	BCF        OPTION_REG+0, 5
;motorv2.h,25 :: 		OPTION_REG.PSA = 0;
	BCF        OPTION_REG+0, 3
;motorv2.h,27 :: 		OPTION_REG.PS2 = 1; // prescaler = 32
	BSF        OPTION_REG+0, 2
;motorv2.h,28 :: 		OPTION_REG.PS1 = 0;
	BCF        OPTION_REG+0, 1
;motorv2.h,29 :: 		OPTION_REG.PS0 = 0;
	BCF        OPTION_REG+0, 0
;motorv2.h,30 :: 		TMR0= 0;
	CLRF       TMR0+0
;motorv2.h,31 :: 		INTCON.TMR0IF = 0;
	BCF        INTCON+0, 2
;motorv2.h,32 :: 		INTCON.TMR0IE = 1;
	BSF        INTCON+0, 5
;motorv2.h,33 :: 		INTCON.GIE = 1;
	BSF        INTCON+0, 7
;motorv2.h,34 :: 		}
L_end_Timer0_Config:
	RETURN
; end of _Timer0_Config

_Delay0_ms:

;motorv2.h,35 :: 		void Delay0_ms(unsigned int t)
;motorv2.h,38 :: 		while(t--)
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
;motorv2.h,40 :: 		INTCON.TMR0IF=0;
	BCF        INTCON+0, 2
;motorv2.h,41 :: 		while(!INTCON.TMR0IF);
L_Delay0_ms2:
	BTFSC      INTCON+0, 2
	GOTO       L_Delay0_ms3
	GOTO       L_Delay0_ms2
L_Delay0_ms3:
;motorv2.h,42 :: 		}
	GOTO       L_Delay0_ms0
L_Delay0_ms1:
;motorv2.h,43 :: 		}
L_end_Delay0_ms:
	RETURN
; end of _Delay0_ms

_PWM2_Config:

;motorv2.h,48 :: 		void PWM2_Config()
;motorv2.h,50 :: 		CCP2CON.CCP2M3= 1 ;
	BSF        CCP2CON+0, 3
;motorv2.h,51 :: 		CCP2CON.CCP2M2= 1 ;
	BSF        CCP2CON+0, 2
;motorv2.h,52 :: 		TRISC.B1= 0;
	BCF        TRISC+0, 1
;motorv2.h,53 :: 		PR2 = 155;
	MOVLW      155
	MOVWF      PR2+0
;motorv2.h,55 :: 		T2CON.T2CKPS0 = 1;
	BSF        T2CON+0, 0
;motorv2.h,56 :: 		T2CON.T2CKPS1 = 1;
	BSF        T2CON+0, 1
;motorv2.h,57 :: 		T2CON.TMR2ON = 1;
	BSF        T2CON+0, 2
;motorv2.h,58 :: 		ADCON1 |=0x07;
	MOVLW      7
	IORWF      ADCON1+0, 1
;motorv2.h,59 :: 		TRISA.B1 = 0;
	BCF        TRISA+0, 1
;motorv2.h,60 :: 		TRISA.B2 = 0;
	BCF        TRISA+0, 2
;motorv2.h,61 :: 		PORTA.B1=0;
	BCF        PORTA+0, 1
;motorv2.h,62 :: 		PORTA.B2=0;
	BCF        PORTA+0, 2
;motorv2.h,63 :: 		CCP2CON.CCP2Y = (0) & 1;
	BCF        CCP2CON+0, 4
;motorv2.h,64 :: 		CCP2CON.CCP2X = (0) & 2;
	BCF        CCP2CON+0, 5
;motorv2.h,65 :: 		CCPR2L = 0 >> 2;
	CLRF       CCPR2L+0
;motorv2.h,66 :: 		}
L_end_PWM2_Config:
	RETURN
; end of _PWM2_Config

_PWM2_Set_Duty:

;motorv2.h,68 :: 		void PWM2_Set_Duty(uint16_t dc)
;motorv2.h,71 :: 		if(dc<1024)
	MOVLW      4
	SUBWF      FARG_PWM2_Set_Duty_dc+1, 0
	BTFSS      STATUS+0, 2
	GOTO       L__PWM2_Set_Duty36
	MOVLW      0
	SUBWF      FARG_PWM2_Set_Duty_dc+0, 0
L__PWM2_Set_Duty36:
	BTFSC      STATUS+0, 0
	GOTO       L_PWM2_Set_Duty4
;motorv2.h,73 :: 		CCP2CON.CCP2Y = (dc) & 1;  // Get bit-0 (The LSB)
	MOVLW      1
	ANDWF      FARG_PWM2_Set_Duty_dc+0, 0
	MOVWF      R0+0
	BTFSC      R0+0, 0
	GOTO       L__PWM2_Set_Duty37
	BCF        CCP2CON+0, 4
	GOTO       L__PWM2_Set_Duty38
L__PWM2_Set_Duty37:
	BSF        CCP2CON+0, 4
L__PWM2_Set_Duty38:
;motorv2.h,74 :: 		CCP2CON.CCP2X = (dc) & 2;  // Get bit-1
	MOVLW      2
	ANDWF      FARG_PWM2_Set_Duty_dc+0, 0
	MOVWF      R0+0
	BTFSC      R0+0, 0
	GOTO       L__PWM2_Set_Duty39
	BCF        CCP2CON+0, 5
	GOTO       L__PWM2_Set_Duty40
L__PWM2_Set_Duty39:
	BSF        CCP2CON+0, 5
L__PWM2_Set_Duty40:
;motorv2.h,75 :: 		CCPR2L = dc >> 2;        // Move The 8 MSBs To CCPR1L register
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
;motorv2.h,76 :: 		}
L_PWM2_Set_Duty4:
;motorv2.h,77 :: 		}
L_end_PWM2_Set_Duty:
	RETURN
; end of _PWM2_Set_Duty

_Read_Encoder_InputCapture_ChannelAB:

;motorv2.h,78 :: 		void Read_Encoder_InputCapture_ChannelAB(void)
;motorv2.h,80 :: 		TMR1H= TMR1L=0;
	CLRF       TMR1L+0
	MOVF       TMR1L+0, 0
	MOVWF      TMR1H+0
;motorv2.h,81 :: 		TMR1= TMR1H;
	MOVF       TMR1H+0, 0
	MOVWF      _TMR1+0
	CLRF       _TMR1+1
;motorv2.h,82 :: 		TMR1<<=8;
	MOVF       _TMR1+0, 0
	MOVWF      _TMR1+1
	CLRF       _TMR1+0
;motorv2.h,83 :: 		TMR1|=TMR1L;
	MOVF       TMR1L+0, 0
	IORWF      _TMR1+0, 1
	MOVLW      0
	IORWF      _TMR1+1, 1
;motorv2.h,84 :: 		T1CON.T1CKPS0 = 0;
	BCF        T1CON+0, 4
;motorv2.h,85 :: 		T1CON.T1CKPS1 = 0;
	BCF        T1CON+0, 5
;motorv2.h,86 :: 		T1CON.TMR1CS = 1;
	BSF        T1CON+0, 1
;motorv2.h,87 :: 		T1CON.T1OSCEN = 0;
	BCF        T1CON+0, 3
;motorv2.h,88 :: 		T1CON.T1SYNC = 0;
	BCF        T1CON+0, 2
;motorv2.h,89 :: 		T1CON.TMR1ON = 1;
	BSF        T1CON+0, 0
;motorv2.h,90 :: 		CCPR1H=CCPR1L=0;
	CLRF       CCPR1L+0
	MOVF       CCPR1L+0, 0
	MOVWF      CCPR1H+0
;motorv2.h,91 :: 		CCP1CON.CCP1M0 = 1;
	BSF        CCP1CON+0, 0
;motorv2.h,92 :: 		CCP1CON.CCP1M1 = 0;
	BCF        CCP1CON+0, 1
;motorv2.h,93 :: 		CCP1CON.CCP1M2 = 1;
	BSF        CCP1CON+0, 2
;motorv2.h,94 :: 		CCP1CON.CCP1M3 = 0;
	BCF        CCP1CON+0, 3
;motorv2.h,95 :: 		TRISC.B2=1;
	BSF        TRISC+0, 2
;motorv2.h,96 :: 		TRISC.B0=1;
	BSF        TRISC+0, 0
;motorv2.h,97 :: 		TRISB.B4 = 1;
	BSF        TRISB+0, 4
;motorv2.h,98 :: 		PIE1.CCP1IE = 1;
	BSF        PIE1+0, 2
;motorv2.h,99 :: 		INTCON.PEIE = 1;
	BSF        INTCON+0, 6
;motorv2.h,100 :: 		INTCON.GIE = 1;
	BSF        INTCON+0, 7
;motorv2.h,101 :: 		}
L_end_Read_Encoder_InputCapture_ChannelAB:
	RETURN
; end of _Read_Encoder_InputCapture_ChannelAB

_Timer1_RunCounter:

;motorv2.h,103 :: 		void Timer1_RunCounter(void)
;motorv2.h,105 :: 		TMR1= TMR1H;
	MOVF       TMR1H+0, 0
	MOVWF      _TMR1+0
	CLRF       _TMR1+1
;motorv2.h,106 :: 		TMR1<<=8;
	MOVF       _TMR1+0, 0
	MOVWF      _TMR1+1
	CLRF       _TMR1+0
;motorv2.h,107 :: 		TMR1|=TMR1L;
	MOVF       TMR1L+0, 0
	IORWF      _TMR1+0, 1
	MOVLW      0
	IORWF      _TMR1+1, 1
;motorv2.h,109 :: 		}
L_end_Timer1_RunCounter:
	RETURN
; end of _Timer1_RunCounter

_qUART_TX_Config:

;motorv2.h,111 :: 		void qUART_TX_Config(void)
;motorv2.h,114 :: 		SPBRG = 129;   // set Baurate = 9600 mbps
	MOVLW      129
	MOVWF      SPBRG+0
;motorv2.h,115 :: 		TXSTA.BRGH = 1;   //High Speed
	BSF        TXSTA+0, 2
;motorv2.h,118 :: 		TXSTA.SYNC = 0;
	BCF        TXSTA+0, 4
;motorv2.h,119 :: 		RCSTA.SPEN=  1;
	BSF        RCSTA+0, 7
;motorv2.h,121 :: 		TRISC.B6=1;
	BSF        TRISC+0, 6
;motorv2.h,122 :: 		TRISC.B7=1;
	BSF        TRISC+0, 7
;motorv2.h,131 :: 		TXSTA.TXEN = 1;
	BSF        TXSTA+0, 5
;motorv2.h,137 :: 		}
L_end_qUART_TX_Config:
	RETURN
; end of _qUART_TX_Config

_qUART_RX_Config:

;motorv2.h,139 :: 		void qUART_RX_Config(void)
;motorv2.h,142 :: 		SPBRG = 129;    //Baudrate 9600 Mbps
	MOVLW      129
	MOVWF      SPBRG+0
;motorv2.h,143 :: 		TXSTA.BRGH = 1;
	BSF        TXSTA+0, 2
;motorv2.h,145 :: 		TXSTA.SYNC = 0;
	BCF        TXSTA+0, 4
;motorv2.h,146 :: 		RCSTA.SPEN=  1;
	BSF        RCSTA+0, 7
;motorv2.h,148 :: 		TRISC.B6=1;
	BSF        TRISC+0, 6
;motorv2.h,149 :: 		TRISC.B7=1;
	BSF        TRISC+0, 7
;motorv2.h,151 :: 		INTCON.GIE = 1;  //Enable Global Interrupt
	BSF        INTCON+0, 7
;motorv2.h,152 :: 		INTCON.PEIE = 1;   //Enable PEIE Interrupt
	BSF        INTCON+0, 6
;motorv2.h,153 :: 		PIE1.RCIE= 1;      // Enable RX UART interrupt
	BSF        PIE1+0, 5
;motorv2.h,156 :: 		RCSTA.CREN = 1;
	BSF        RCSTA+0, 4
;motorv2.h,157 :: 		}
L_end_qUART_RX_Config:
	RETURN
; end of _qUART_RX_Config

_qUART_Transmit:

;motorv2.h,160 :: 		void qUART_Transmit(unsigned char dataUART)
;motorv2.h,162 :: 		while(!TXSTA.TRMT);
L_qUART_Transmit5:
	BTFSC      TXSTA+0, 1
	GOTO       L_qUART_Transmit6
	GOTO       L_qUART_Transmit5
L_qUART_Transmit6:
;motorv2.h,163 :: 		TXREG = dataUART;
	MOVF       FARG_qUART_Transmit_dataUART+0, 0
	MOVWF      TXREG+0
;motorv2.h,164 :: 		}
L_end_qUART_Transmit:
	RETURN
; end of _qUART_Transmit

_qUART_PutString:

;motorv2.h,165 :: 		void qUART_PutString(unsigned char* s)
;motorv2.h,167 :: 		while(*s)
L_qUART_PutString7:
	MOVF       FARG_qUART_PutString_s+0, 0
	MOVWF      FSR
	MOVF       INDF+0, 0
	BTFSC      STATUS+0, 2
	GOTO       L_qUART_PutString8
;motorv2.h,169 :: 		qUART_Transmit(*s++);
	MOVF       FARG_qUART_PutString_s+0, 0
	MOVWF      FSR
	MOVF       INDF+0, 0
	MOVWF      FARG_qUART_Transmit_dataUART+0
	CALL       _qUART_Transmit+0
	INCF       FARG_qUART_PutString_s+0, 1
;motorv2.h,170 :: 		}
	GOTO       L_qUART_PutString7
L_qUART_PutString8:
;motorv2.h,171 :: 		}
L_end_qUART_PutString:
	RETURN
; end of _qUART_PutString

_qUART_ReadData:

;motorv2.h,173 :: 		unsigned char qUART_ReadData()
;motorv2.h,175 :: 		while(!PIR1.RCIF);
L_qUART_ReadData9:
	BTFSC      PIR1+0, 5
	GOTO       L_qUART_ReadData10
	GOTO       L_qUART_ReadData9
L_qUART_ReadData10:
;motorv2.h,176 :: 		return RCREG;
	MOVF       RCREG+0, 0
	MOVWF      R0+0
;motorv2.h,177 :: 		}
L_end_qUART_ReadData:
	RETURN
; end of _qUART_ReadData

_main:

;motorv2.c,16 :: 		void main() {
;motorv2.c,18 :: 		ADCON1 |=0x07;
	MOVLW      7
	IORWF      ADCON1+0, 1
;motorv2.c,19 :: 		TRISA.B3 =0;
	BCF        TRISA+0, 3
;motorv2.c,20 :: 		TRISA.B4 =0;
	BCF        TRISA+0, 4
;motorv2.c,21 :: 		TRISA.B5 =0;
	BCF        TRISA+0, 5
;motorv2.c,22 :: 		PORTA.B3 = 1;
	BSF        PORTA+0, 3
;motorv2.c,23 :: 		PORTA.B4 = 1;
	BSF        PORTA+0, 4
;motorv2.c,24 :: 		PORTA.B5 = 1;
	BSF        PORTA+0, 5
;motorv2.c,27 :: 		Calibrate_Motor();
	CALL       _Calibrate_Motor+0
;motorv2.c,28 :: 		qUART_TX_Config();
	CALL       _qUART_TX_Config+0
;motorv2.c,29 :: 		Timer0_Config();
	CALL       _Timer0_Config+0
;motorv2.c,30 :: 		PWM2_Config();
	CALL       _PWM2_Config+0
;motorv2.c,35 :: 		Read_Encoder_InputCapture_ChannelAB();
	CALL       _Read_Encoder_InputCapture_ChannelAB+0
;motorv2.c,39 :: 		while(1)
L_main11:
;motorv2.c,41 :: 		Motor_Control_Velocity(Velocity_SP,Velocity_PV);
	MOVF       _Velocity_SP+0, 0
	MOVWF      FARG_Motor_Control_Velocity_SP+0
	MOVF       _Velocity_SP+1, 0
	MOVWF      FARG_Motor_Control_Velocity_SP+1
	MOVF       _Velocity_PV+0, 0
	MOVWF      FARG_Motor_Control_Velocity_PV+0
	MOVF       _Velocity_PV+1, 0
	MOVWF      FARG_Motor_Control_Velocity_PV+1
	CALL       _Motor_Control_Velocity+0
;motorv2.c,42 :: 		Delay_ms(1);
	MOVLW      7
	MOVWF      R12+0
	MOVLW      125
	MOVWF      R13+0
L_main13:
	DECFSZ     R13+0, 1
	GOTO       L_main13
	DECFSZ     R12+0, 1
	GOTO       L_main13
;motorv2.c,45 :: 		}
	GOTO       L_main11
;motorv2.c,46 :: 		}
L_end_main:
	GOTO       $+0
; end of _main

_Calibrate_Motor:

;motorv2.c,48 :: 		void Calibrate_Motor(void)
;motorv2.c,50 :: 		PID=0.0;
	CLRF       _PID+0
	CLRF       _PID+1
	CLRF       _PID+2
	CLRF       _PID+3
;motorv2.c,51 :: 		Error=0.0;
	CLRF       _Error+0
	CLRF       _Error+1
	CLRF       _Error+2
	CLRF       _Error+3
;motorv2.c,52 :: 		P_part=0.0;
	CLRF       _P_part+0
	CLRF       _P_part+1
	CLRF       _P_part+2
	CLRF       _P_part+3
;motorv2.c,53 :: 		pre_Error=0.0;
	CLRF       _pre_Error+0
	CLRF       _pre_Error+1
	CLRF       _pre_Error+2
	CLRF       _pre_Error+3
;motorv2.c,54 :: 		pre_I_Part=0.0;
	CLRF       _pre_I_Part+0
	CLRF       _pre_I_Part+1
	CLRF       _pre_I_Part+2
	CLRF       _pre_I_Part+3
;motorv2.c,55 :: 		I_part=0.0;
	CLRF       _I_part+0
	CLRF       _I_part+1
	CLRF       _I_part+2
	CLRF       _I_part+3
;motorv2.c,56 :: 		D_part=0.0;
	CLRF       _D_part+0
	CLRF       _D_part+1
	CLRF       _D_part+2
	CLRF       _D_part+3
;motorv2.c,57 :: 		Output=0.0;
	CLRF       _Output+0
	CLRF       _Output+1
	CLRF       _Output+2
	CLRF       _Output+3
;motorv2.c,58 :: 		Velocity_PV=0;
	CLRF       _Velocity_PV+0
	CLRF       _Velocity_PV+1
;motorv2.c,59 :: 		Pulse_per_second=0;
	CLRF       _Pulse_per_second+0
	CLRF       _Pulse_per_second+1
;motorv2.c,60 :: 		sum_count=0;
	CLRF       _sum_count+0
	CLRF       _sum_count+1
;motorv2.c,61 :: 		cnt=0;
	CLRF       _cnt+0
	CLRF       _cnt+1
;motorv2.c,62 :: 		}
L_end_Calibrate_Motor:
	RETURN
; end of _Calibrate_Motor

_Motor_Control_Velocity:

;motorv2.c,63 :: 		void Motor_Control_Velocity(uint16_t SP /* rpm */ , uint16_t PV)
;motorv2.c,67 :: 		Velocity_PV =(uint16_t)((double)(Pulse_per_second/330.0)*60.0)  ;
	MOVF       _Pulse_per_second+0, 0
	MOVWF      R0+0
	MOVF       _Pulse_per_second+1, 0
	MOVWF      R0+1
	CALL       _word2double+0
	MOVLW      0
	MOVWF      R4+0
	MOVLW      0
	MOVWF      R4+1
	MOVLW      37
	MOVWF      R4+2
	MOVLW      135
	MOVWF      R4+3
	CALL       _Div_32x32_FP+0
	MOVLW      0
	MOVWF      R4+0
	MOVLW      0
	MOVWF      R4+1
	MOVLW      112
	MOVWF      R4+2
	MOVLW      132
	MOVWF      R4+3
	CALL       _Mul_32x32_FP+0
	CALL       _double2word+0
	MOVF       R0+0, 0
	MOVWF      _Velocity_PV+0
	MOVF       R0+1, 0
	MOVWF      _Velocity_PV+1
;motorv2.c,68 :: 		WordToStr(Velocity_PV,positionUart);
	MOVF       R0+0, 0
	MOVWF      FARG_WordToStr_input+0
	MOVF       R0+1, 0
	MOVWF      FARG_WordToStr_input+1
	MOVLW      _positionUart+0
	MOVWF      FARG_WordToStr_output+0
	CALL       _WordToStr+0
;motorv2.c,69 :: 		qUART_PutString(positionUart);
	MOVLW      _positionUart+0
	MOVWF      FARG_qUART_PutString_s+0
	CALL       _qUART_PutString+0
;motorv2.c,70 :: 		qUART_PutString(" ");
	MOVLW      ?lstr1_motorv2+0
	MOVWF      FARG_qUART_PutString_s+0
	CALL       _qUART_PutString+0
;motorv2.c,72 :: 		PV = Velocity_PV;
	MOVF       _Velocity_PV+0, 0
	MOVWF      FARG_Motor_Control_Velocity_PV+0
	MOVF       _Velocity_PV+1, 0
	MOVWF      FARG_Motor_Control_Velocity_PV+1
;motorv2.c,73 :: 		Error = (double)SP-(double)PV;
	MOVF       FARG_Motor_Control_Velocity_SP+0, 0
	MOVWF      R0+0
	MOVF       FARG_Motor_Control_Velocity_SP+1, 0
	MOVWF      R0+1
	CALL       _word2double+0
	MOVF       R0+0, 0
	MOVWF      FLOC__Motor_Control_Velocity+0
	MOVF       R0+1, 0
	MOVWF      FLOC__Motor_Control_Velocity+1
	MOVF       R0+2, 0
	MOVWF      FLOC__Motor_Control_Velocity+2
	MOVF       R0+3, 0
	MOVWF      FLOC__Motor_Control_Velocity+3
	MOVF       _Velocity_PV+0, 0
	MOVWF      R0+0
	MOVF       _Velocity_PV+1, 0
	MOVWF      R0+1
	CALL       _word2double+0
	MOVF       R0+0, 0
	MOVWF      R4+0
	MOVF       R0+1, 0
	MOVWF      R4+1
	MOVF       R0+2, 0
	MOVWF      R4+2
	MOVF       R0+3, 0
	MOVWF      R4+3
	MOVF       FLOC__Motor_Control_Velocity+0, 0
	MOVWF      R0+0
	MOVF       FLOC__Motor_Control_Velocity+1, 0
	MOVWF      R0+1
	MOVF       FLOC__Motor_Control_Velocity+2, 0
	MOVWF      R0+2
	MOVF       FLOC__Motor_Control_Velocity+3, 0
	MOVWF      R0+3
	CALL       _Sub_32x32_FP+0
	MOVF       R0+0, 0
	MOVWF      FLOC__Motor_Control_Velocity+12
	MOVF       R0+1, 0
	MOVWF      FLOC__Motor_Control_Velocity+13
	MOVF       R0+2, 0
	MOVWF      FLOC__Motor_Control_Velocity+14
	MOVF       R0+3, 0
	MOVWF      FLOC__Motor_Control_Velocity+15
	MOVF       FLOC__Motor_Control_Velocity+12, 0
	MOVWF      _Error+0
	MOVF       FLOC__Motor_Control_Velocity+13, 0
	MOVWF      _Error+1
	MOVF       FLOC__Motor_Control_Velocity+14, 0
	MOVWF      _Error+2
	MOVF       FLOC__Motor_Control_Velocity+15, 0
	MOVWF      _Error+3
;motorv2.c,75 :: 		I_part = pre_I_Part + Error * T;
	MOVF       FLOC__Motor_Control_Velocity+12, 0
	MOVWF      R0+0
	MOVF       FLOC__Motor_Control_Velocity+13, 0
	MOVWF      R0+1
	MOVF       FLOC__Motor_Control_Velocity+14, 0
	MOVWF      R0+2
	MOVF       FLOC__Motor_Control_Velocity+15, 0
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
	MOVWF      FLOC__Motor_Control_Velocity+8
	MOVF       R0+1, 0
	MOVWF      FLOC__Motor_Control_Velocity+9
	MOVF       R0+2, 0
	MOVWF      FLOC__Motor_Control_Velocity+10
	MOVF       R0+3, 0
	MOVWF      FLOC__Motor_Control_Velocity+11
	MOVF       FLOC__Motor_Control_Velocity+8, 0
	MOVWF      _I_part+0
	MOVF       FLOC__Motor_Control_Velocity+9, 0
	MOVWF      _I_part+1
	MOVF       FLOC__Motor_Control_Velocity+10, 0
	MOVWF      _I_part+2
	MOVF       FLOC__Motor_Control_Velocity+11, 0
	MOVWF      _I_part+3
;motorv2.c,76 :: 		D_part = (Error - pre_Error)/T;
	MOVF       _pre_Error+0, 0
	MOVWF      R4+0
	MOVF       _pre_Error+1, 0
	MOVWF      R4+1
	MOVF       _pre_Error+2, 0
	MOVWF      R4+2
	MOVF       _pre_Error+3, 0
	MOVWF      R4+3
	MOVF       FLOC__Motor_Control_Velocity+12, 0
	MOVWF      R0+0
	MOVF       FLOC__Motor_Control_Velocity+13, 0
	MOVWF      R0+1
	MOVF       FLOC__Motor_Control_Velocity+14, 0
	MOVWF      R0+2
	MOVF       FLOC__Motor_Control_Velocity+15, 0
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
	MOVWF      FLOC__Motor_Control_Velocity+4
	MOVF       R0+1, 0
	MOVWF      FLOC__Motor_Control_Velocity+5
	MOVF       R0+2, 0
	MOVWF      FLOC__Motor_Control_Velocity+6
	MOVF       R0+3, 0
	MOVWF      FLOC__Motor_Control_Velocity+7
	MOVF       FLOC__Motor_Control_Velocity+4, 0
	MOVWF      _D_part+0
	MOVF       FLOC__Motor_Control_Velocity+5, 0
	MOVWF      _D_part+1
	MOVF       FLOC__Motor_Control_Velocity+6, 0
	MOVWF      _D_part+2
	MOVF       FLOC__Motor_Control_Velocity+7, 0
	MOVWF      _D_part+3
;motorv2.c,77 :: 		PID = Kp*Error + Ki*I_part + Kd*D_part;
	MOVF       _Kp+0, 0
	MOVWF      R0+0
	MOVF       _Kp+1, 0
	MOVWF      R0+1
	MOVF       _Kp+2, 0
	MOVWF      R0+2
	MOVF       _Kp+3, 0
	MOVWF      R0+3
	MOVF       FLOC__Motor_Control_Velocity+12, 0
	MOVWF      R4+0
	MOVF       FLOC__Motor_Control_Velocity+13, 0
	MOVWF      R4+1
	MOVF       FLOC__Motor_Control_Velocity+14, 0
	MOVWF      R4+2
	MOVF       FLOC__Motor_Control_Velocity+15, 0
	MOVWF      R4+3
	CALL       _Mul_32x32_FP+0
	MOVF       R0+0, 0
	MOVWF      FLOC__Motor_Control_Velocity+0
	MOVF       R0+1, 0
	MOVWF      FLOC__Motor_Control_Velocity+1
	MOVF       R0+2, 0
	MOVWF      FLOC__Motor_Control_Velocity+2
	MOVF       R0+3, 0
	MOVWF      FLOC__Motor_Control_Velocity+3
	MOVF       _Ki+0, 0
	MOVWF      R0+0
	MOVF       _Ki+1, 0
	MOVWF      R0+1
	MOVF       _Ki+2, 0
	MOVWF      R0+2
	MOVF       _Ki+3, 0
	MOVWF      R0+3
	MOVF       FLOC__Motor_Control_Velocity+8, 0
	MOVWF      R4+0
	MOVF       FLOC__Motor_Control_Velocity+9, 0
	MOVWF      R4+1
	MOVF       FLOC__Motor_Control_Velocity+10, 0
	MOVWF      R4+2
	MOVF       FLOC__Motor_Control_Velocity+11, 0
	MOVWF      R4+3
	CALL       _Mul_32x32_FP+0
	MOVF       FLOC__Motor_Control_Velocity+0, 0
	MOVWF      R4+0
	MOVF       FLOC__Motor_Control_Velocity+1, 0
	MOVWF      R4+1
	MOVF       FLOC__Motor_Control_Velocity+2, 0
	MOVWF      R4+2
	MOVF       FLOC__Motor_Control_Velocity+3, 0
	MOVWF      R4+3
	CALL       _Add_32x32_FP+0
	MOVF       R0+0, 0
	MOVWF      FLOC__Motor_Control_Velocity+0
	MOVF       R0+1, 0
	MOVWF      FLOC__Motor_Control_Velocity+1
	MOVF       R0+2, 0
	MOVWF      FLOC__Motor_Control_Velocity+2
	MOVF       R0+3, 0
	MOVWF      FLOC__Motor_Control_Velocity+3
	MOVF       _Kd+0, 0
	MOVWF      R0+0
	MOVF       _Kd+1, 0
	MOVWF      R0+1
	MOVF       _Kd+2, 0
	MOVWF      R0+2
	MOVF       _Kd+3, 0
	MOVWF      R0+3
	MOVF       FLOC__Motor_Control_Velocity+4, 0
	MOVWF      R4+0
	MOVF       FLOC__Motor_Control_Velocity+5, 0
	MOVWF      R4+1
	MOVF       FLOC__Motor_Control_Velocity+6, 0
	MOVWF      R4+2
	MOVF       FLOC__Motor_Control_Velocity+7, 0
	MOVWF      R4+3
	CALL       _Mul_32x32_FP+0
	MOVF       FLOC__Motor_Control_Velocity+0, 0
	MOVWF      R4+0
	MOVF       FLOC__Motor_Control_Velocity+1, 0
	MOVWF      R4+1
	MOVF       FLOC__Motor_Control_Velocity+2, 0
	MOVWF      R4+2
	MOVF       FLOC__Motor_Control_Velocity+3, 0
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
;motorv2.c,78 :: 		pre_Error = Error;
	MOVF       FLOC__Motor_Control_Velocity+12, 0
	MOVWF      _pre_Error+0
	MOVF       FLOC__Motor_Control_Velocity+13, 0
	MOVWF      _pre_Error+1
	MOVF       FLOC__Motor_Control_Velocity+14, 0
	MOVWF      _pre_Error+2
	MOVF       FLOC__Motor_Control_Velocity+15, 0
	MOVWF      _pre_Error+3
;motorv2.c,79 :: 		pre_I_Part = I_part;
	MOVF       FLOC__Motor_Control_Velocity+8, 0
	MOVWF      _pre_I_Part+0
	MOVF       FLOC__Motor_Control_Velocity+9, 0
	MOVWF      _pre_I_Part+1
	MOVF       FLOC__Motor_Control_Velocity+10, 0
	MOVWF      _pre_I_Part+2
	MOVF       FLOC__Motor_Control_Velocity+11, 0
	MOVWF      _pre_I_Part+3
;motorv2.c,83 :: 		Output=(double)(Output+PID);
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
;motorv2.c,85 :: 		if (Output >=333.0)
	MOVLW      0
	MOVWF      R4+0
	MOVLW      128
	MOVWF      R4+1
	MOVLW      38
	MOVWF      R4+2
	MOVLW      135
	MOVWF      R4+3
	CALL       _Compare_Double+0
	MOVLW      1
	BTFSS      STATUS+0, 0
	MOVLW      0
	MOVWF      R0+0
	MOVF       R0+0, 0
	BTFSC      STATUS+0, 2
	GOTO       L_Motor_Control_Velocity14
;motorv2.c,87 :: 		Output_int = 624;
	MOVLW      112
	MOVWF      _Output_int+0
	MOVLW      2
	MOVWF      _Output_int+1
;motorv2.c,88 :: 		}
	GOTO       L_Motor_Control_Velocity15
L_Motor_Control_Velocity14:
;motorv2.c,91 :: 		Output_int = (uint16_t)((double)(Output/333.0)*624.0 );
	MOVLW      0
	MOVWF      R4+0
	MOVLW      128
	MOVWF      R4+1
	MOVLW      38
	MOVWF      R4+2
	MOVLW      135
	MOVWF      R4+3
	MOVF       _Output+0, 0
	MOVWF      R0+0
	MOVF       _Output+1, 0
	MOVWF      R0+1
	MOVF       _Output+2, 0
	MOVWF      R0+2
	MOVF       _Output+3, 0
	MOVWF      R0+3
	CALL       _Div_32x32_FP+0
	MOVLW      0
	MOVWF      R4+0
	MOVLW      0
	MOVWF      R4+1
	MOVLW      28
	MOVWF      R4+2
	MOVLW      136
	MOVWF      R4+3
	CALL       _Mul_32x32_FP+0
	CALL       _double2word+0
	MOVF       R0+0, 0
	MOVWF      _Output_int+0
	MOVF       R0+1, 0
	MOVWF      _Output_int+1
;motorv2.c,92 :: 		}
L_Motor_Control_Velocity15:
;motorv2.c,93 :: 		PORTA.B1=1;
	BSF        PORTA+0, 1
;motorv2.c,94 :: 		PWM2_Set_Duty(Output_int);
	MOVF       _Output_int+0, 0
	MOVWF      FARG_PWM2_Set_Duty_dc+0
	MOVF       _Output_int+1, 0
	MOVWF      FARG_PWM2_Set_Duty_dc+1
	CALL       _PWM2_Set_Duty+0
;motorv2.c,98 :: 		}
L_end_Motor_Control_Velocity:
	RETURN
; end of _Motor_Control_Velocity

_Interrupt:
	MOVWF      R15+0
	SWAPF      STATUS+0, 0
	CLRF       STATUS+0
	MOVWF      ___saveSTATUS+0
	MOVF       PCLATH+0, 0
	MOVWF      ___savePCLATH+0
	CLRF       PCLATH+0

;motorv2.c,101 :: 		void Interrupt ()
;motorv2.c,103 :: 		if (INTCON.TMR0IF == 1)
	BTFSS      INTCON+0, 2
	GOTO       L_Interrupt16
;motorv2.c,106 :: 		INTCON.TMR0IF = 0;
	BCF        INTCON+0, 2
;motorv2.c,107 :: 		}//TMR0
L_Interrupt16:
;motorv2.c,108 :: 		if (PIR1.CCP1IF == 1)
	BTFSS      PIR1+0, 2
	GOTO       L_Interrupt17
;motorv2.c,110 :: 		PORTA.B3 = !PORTA.B3;
	MOVLW      8
	XORWF      PORTA+0, 1
;motorv2.c,111 :: 		if (CCPR1 >=65535)
	MOVLW      255
	SUBWF      CCPR1+1, 0
	BTFSS      STATUS+0, 2
	GOTO       L__Interrupt53
	MOVLW      255
	SUBWF      CCPR1+0, 0
L__Interrupt53:
	BTFSS      STATUS+0, 0
	GOTO       L_Interrupt18
;motorv2.c,113 :: 		CCPR1=0;
	CLRF       CCPR1+0
	CLRF       CCPR1+1
;motorv2.c,114 :: 		TMR1H= TMR1L=0;
	CLRF       TMR1L+0
	MOVF       TMR1L+0, 0
	MOVWF      TMR1H+0
;motorv2.c,115 :: 		}
L_Interrupt18:
;motorv2.c,117 :: 		if(PORTB.B4 == 0)
	BTFSC      PORTB+0, 4
	GOTO       L_Interrupt19
;motorv2.c,119 :: 		PORTA.B4 = !PORTA.B4;
	MOVLW      16
	XORWF      PORTA+0, 1
;motorv2.c,120 :: 		direct = 0;
	CLRF       _direct+0
;motorv2.c,121 :: 		count++;
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
;motorv2.c,122 :: 		}
	GOTO       L_Interrupt20
L_Interrupt19:
;motorv2.c,124 :: 		else if(PORTB.B4 == 1)
	BTFSS      PORTB+0, 4
	GOTO       L_Interrupt21
;motorv2.c,126 :: 		PORTA.B5 = !PORTA.B5;
	MOVLW      32
	XORWF      PORTA+0, 1
;motorv2.c,127 :: 		direct = 1;
	MOVLW      1
	MOVWF      _direct+0
;motorv2.c,128 :: 		count--;
	MOVLW      1
	SUBWF      _count+0, 1
	BTFSS      STATUS+0, 0
	SUBWF      _count+1, 1
	BTFSS      STATUS+0, 0
	SUBWF      _count+2, 1
	BTFSS      STATUS+0, 0
	SUBWF      _count+3, 1
;motorv2.c,129 :: 		}
L_Interrupt21:
L_Interrupt20:
;motorv2.c,132 :: 		TMR0 =0;
	CLRF       TMR0+0
;motorv2.c,133 :: 		while(PORTC.B2 ==  1)
L_Interrupt22:
	BTFSS      PORTC+0, 2
	GOTO       L_Interrupt23
;motorv2.c,134 :: 		while(PORTC.B2 ==  0)
L_Interrupt24:
	BTFSC      PORTC+0, 2
	GOTO       L_Interrupt25
;motorv2.c,136 :: 		TMR0 =0;
	CLRF       TMR0+0
	GOTO       L_Interrupt24
L_Interrupt25:
	GOTO       L_Interrupt22
L_Interrupt23:
;motorv2.c,137 :: 		while(PORTC.B2 ==  1)
L_Interrupt26:
	BTFSS      PORTC+0, 2
	GOTO       L_Interrupt27
;motorv2.c,139 :: 		if (TMR0 >= 255)
	MOVLW      255
	SUBWF      TMR0+0, 0
	BTFSS      STATUS+0, 0
	GOTO       L_Interrupt28
;motorv2.c,141 :: 		sum_count = sum_count + 256;
	MOVLW      0
	ADDWF      _sum_count+0, 1
	MOVLW      1
	BTFSC      STATUS+0, 0
	ADDLW      1
	ADDWF      _sum_count+1, 1
;motorv2.c,142 :: 		TMR0 = 0;
	CLRF       TMR0+0
;motorv2.c,143 :: 		}
L_Interrupt28:
;motorv2.c,144 :: 		}
	GOTO       L_Interrupt26
L_Interrupt27:
;motorv2.c,145 :: 		while(PORTC.B2 ==  0)
L_Interrupt29:
	BTFSC      PORTC+0, 2
	GOTO       L_Interrupt30
;motorv2.c,147 :: 		if (TMR0 >= 255)
	MOVLW      255
	SUBWF      TMR0+0, 0
	BTFSS      STATUS+0, 0
	GOTO       L_Interrupt31
;motorv2.c,149 :: 		sum_count = sum_count + 256;
	MOVLW      0
	ADDWF      _sum_count+0, 1
	MOVLW      1
	BTFSC      STATUS+0, 0
	ADDLW      1
	ADDWF      _sum_count+1, 1
;motorv2.c,150 :: 		TMR0 = 0;
	CLRF       TMR0+0
;motorv2.c,151 :: 		}
L_Interrupt31:
;motorv2.c,152 :: 		}
	GOTO       L_Interrupt29
L_Interrupt30:
;motorv2.c,153 :: 		sum_count = sum_count + TMR0;
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
;motorv2.c,155 :: 		Pulse_per_second = 156250/sum_count;
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
;motorv2.c,158 :: 		sum_count=0;
	CLRF       _sum_count+0
	CLRF       _sum_count+1
;motorv2.c,159 :: 		TMR0 = 0;
	CLRF       TMR0+0
;motorv2.c,162 :: 		PIR1.CCP1IF = 0; // Clear Flag Capture
	BCF        PIR1+0, 2
;motorv2.c,163 :: 		}//CCP1
L_Interrupt17:
;motorv2.c,166 :: 		}//Interrupt
L_end_Interrupt:
L__Interrupt52:
	MOVF       ___savePCLATH+0, 0
	MOVWF      PCLATH+0
	SWAPF      ___saveSTATUS+0, 0
	MOVWF      STATUS+0
	SWAPF      R15+0, 1
	SWAPF      R15+0, 0
	RETFIE
; end of _Interrupt
