
_Timer0_Config:

;motorv1.h,22 :: 		void Timer0_Config(void)
;motorv1.h,24 :: 		OPTION_REG.T0CS = 0;
	BCF        OPTION_REG+0, 5
;motorv1.h,25 :: 		OPTION_REG.PSA = 0;
	BCF        OPTION_REG+0, 3
;motorv1.h,26 :: 		OPTION_REG.PS2 = 1;
	BSF        OPTION_REG+0, 2
;motorv1.h,27 :: 		OPTION_REG.PS1 = 0;
	BCF        OPTION_REG+0, 1
;motorv1.h,28 :: 		OPTION_REG.PS0 = 0;
	BCF        OPTION_REG+0, 0
;motorv1.h,29 :: 		TMR0= 100;
	MOVLW      100
	MOVWF      TMR0+0
;motorv1.h,30 :: 		INTCON.TMR0IF = 0;
	BCF        INTCON+0, 2
;motorv1.h,31 :: 		INTCON.TMR0IE = 1;
	BSF        INTCON+0, 5
;motorv1.h,32 :: 		INTCON.GIE = 1;
	BSF        INTCON+0, 7
;motorv1.h,33 :: 		}
L_end_Timer0_Config:
	RETURN
; end of _Timer0_Config

_Delay0_ms:

;motorv1.h,34 :: 		void Delay0_ms(unsigned int t)
;motorv1.h,37 :: 		while(t--)
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
;motorv1.h,39 :: 		INTCON.TMR0IF=0;
	BCF        INTCON+0, 2
;motorv1.h,40 :: 		while(!INTCON.TMR0IF);
L_Delay0_ms2:
	BTFSC      INTCON+0, 2
	GOTO       L_Delay0_ms3
	GOTO       L_Delay0_ms2
L_Delay0_ms3:
;motorv1.h,41 :: 		}
	GOTO       L_Delay0_ms0
L_Delay0_ms1:
;motorv1.h,42 :: 		}
L_end_Delay0_ms:
	RETURN
; end of _Delay0_ms

_PWM2_Config:

;motorv1.h,47 :: 		void PWM2_Config()
;motorv1.h,49 :: 		CCP2CON.CCP2M3= 1 ;
	BSF        CCP2CON+0, 3
;motorv1.h,50 :: 		CCP2CON.CCP2M2= 1 ;
	BSF        CCP2CON+0, 2
;motorv1.h,51 :: 		TRISC.B1= 0;
	BCF        TRISC+0, 1
;motorv1.h,52 :: 		PR2 = 155;
	MOVLW      155
	MOVWF      PR2+0
;motorv1.h,54 :: 		T2CON.T2CKPS0 = 1;
	BSF        T2CON+0, 0
;motorv1.h,55 :: 		T2CON.T2CKPS1 = 1;
	BSF        T2CON+0, 1
;motorv1.h,56 :: 		T2CON.TMR2ON = 1;
	BSF        T2CON+0, 2
;motorv1.h,57 :: 		ADCON1 |=0x07;
	MOVLW      7
	IORWF      ADCON1+0, 1
;motorv1.h,58 :: 		TRISA.B1 = 0;
	BCF        TRISA+0, 1
;motorv1.h,59 :: 		TRISA.B2 = 0;
	BCF        TRISA+0, 2
;motorv1.h,60 :: 		PORTA.B1=0;
	BCF        PORTA+0, 1
;motorv1.h,61 :: 		PORTA.B2=0;
	BCF        PORTA+0, 2
;motorv1.h,62 :: 		CCP2CON.CCP2Y = (0) & 1;
	BCF        CCP2CON+0, 4
;motorv1.h,63 :: 		CCP2CON.CCP2X = (0) & 2;
	BCF        CCP2CON+0, 5
;motorv1.h,64 :: 		CCPR2L = 0 >> 2;
	CLRF       CCPR2L+0
;motorv1.h,65 :: 		}
L_end_PWM2_Config:
	RETURN
; end of _PWM2_Config

_PWM2_Set_Duty:

;motorv1.h,67 :: 		void PWM2_Set_Duty(uint16_t dc)
;motorv1.h,70 :: 		if(dc<1024)
	MOVLW      4
	SUBWF      FARG_PWM2_Set_Duty_dc+1, 0
	BTFSS      STATUS+0, 2
	GOTO       L__PWM2_Set_Duty35
	MOVLW      0
	SUBWF      FARG_PWM2_Set_Duty_dc+0, 0
L__PWM2_Set_Duty35:
	BTFSC      STATUS+0, 0
	GOTO       L_PWM2_Set_Duty4
;motorv1.h,72 :: 		CCP2CON.CCP2Y = (dc) & 1;  // Get bit-0 (The LSB)
	MOVLW      1
	ANDWF      FARG_PWM2_Set_Duty_dc+0, 0
	MOVWF      R0+0
	BTFSC      R0+0, 0
	GOTO       L__PWM2_Set_Duty36
	BCF        CCP2CON+0, 4
	GOTO       L__PWM2_Set_Duty37
L__PWM2_Set_Duty36:
	BSF        CCP2CON+0, 4
L__PWM2_Set_Duty37:
;motorv1.h,73 :: 		CCP2CON.CCP2X = (dc) & 2;  // Get bit-1
	MOVLW      2
	ANDWF      FARG_PWM2_Set_Duty_dc+0, 0
	MOVWF      R0+0
	BTFSC      R0+0, 0
	GOTO       L__PWM2_Set_Duty38
	BCF        CCP2CON+0, 5
	GOTO       L__PWM2_Set_Duty39
L__PWM2_Set_Duty38:
	BSF        CCP2CON+0, 5
L__PWM2_Set_Duty39:
;motorv1.h,74 :: 		CCPR2L = dc >> 2;        // Move The 8 MSBs To CCPR1L register
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
;motorv1.h,75 :: 		}
L_PWM2_Set_Duty4:
;motorv1.h,76 :: 		}
L_end_PWM2_Set_Duty:
	RETURN
; end of _PWM2_Set_Duty

_Read_Encoder_InputCapture_ChannelAB:

;motorv1.h,77 :: 		void Read_Encoder_InputCapture_ChannelAB(void)
;motorv1.h,79 :: 		TMR1H= TMR1L=0;
	CLRF       TMR1L+0
	MOVF       TMR1L+0, 0
	MOVWF      TMR1H+0
;motorv1.h,80 :: 		TMR1= TMR1H;
	MOVF       TMR1H+0, 0
	MOVWF      _TMR1+0
	CLRF       _TMR1+1
;motorv1.h,81 :: 		TMR1<<=8;
	MOVF       _TMR1+0, 0
	MOVWF      _TMR1+1
	CLRF       _TMR1+0
;motorv1.h,82 :: 		TMR1|=TMR1L;
	MOVF       TMR1L+0, 0
	IORWF      _TMR1+0, 1
	MOVLW      0
	IORWF      _TMR1+1, 1
;motorv1.h,83 :: 		T1CON.T1CKPS0 = 0;
	BCF        T1CON+0, 4
;motorv1.h,84 :: 		T1CON.T1CKPS1 = 0;
	BCF        T1CON+0, 5
;motorv1.h,85 :: 		T1CON.TMR1CS = 1;
	BSF        T1CON+0, 1
;motorv1.h,86 :: 		T1CON.T1OSCEN = 0;
	BCF        T1CON+0, 3
;motorv1.h,87 :: 		T1CON.T1SYNC = 0;
	BCF        T1CON+0, 2
;motorv1.h,88 :: 		T1CON.TMR1ON = 1;
	BSF        T1CON+0, 0
;motorv1.h,89 :: 		CCPR1H=CCPR1L=0;
	CLRF       CCPR1L+0
	MOVF       CCPR1L+0, 0
	MOVWF      CCPR1H+0
;motorv1.h,90 :: 		CCP1CON.CCP1M0 = 1;
	BSF        CCP1CON+0, 0
;motorv1.h,91 :: 		CCP1CON.CCP1M1 = 0;
	BCF        CCP1CON+0, 1
;motorv1.h,92 :: 		CCP1CON.CCP1M2 = 1;
	BSF        CCP1CON+0, 2
;motorv1.h,93 :: 		CCP1CON.CCP1M3 = 0;
	BCF        CCP1CON+0, 3
;motorv1.h,94 :: 		TRISC.B2=1;
	BSF        TRISC+0, 2
;motorv1.h,95 :: 		TRISC.B0=1;
	BSF        TRISC+0, 0
;motorv1.h,96 :: 		TRISB.B4 = 1;
	BSF        TRISB+0, 4
;motorv1.h,97 :: 		PIE1.CCP1IE = 1;
	BSF        PIE1+0, 2
;motorv1.h,98 :: 		INTCON.PEIE = 1;
	BSF        INTCON+0, 6
;motorv1.h,99 :: 		INTCON.GIE = 1;
	BSF        INTCON+0, 7
;motorv1.h,100 :: 		}
L_end_Read_Encoder_InputCapture_ChannelAB:
	RETURN
; end of _Read_Encoder_InputCapture_ChannelAB

_Timer1_RunCounter:

;motorv1.h,102 :: 		void Timer1_RunCounter(void)
;motorv1.h,104 :: 		TMR1= TMR1H;
	MOVF       TMR1H+0, 0
	MOVWF      _TMR1+0
	CLRF       _TMR1+1
;motorv1.h,105 :: 		TMR1<<=8;
	MOVF       _TMR1+0, 0
	MOVWF      _TMR1+1
	CLRF       _TMR1+0
;motorv1.h,106 :: 		TMR1|=TMR1L;
	MOVF       TMR1L+0, 0
	IORWF      _TMR1+0, 1
	MOVLW      0
	IORWF      _TMR1+1, 1
;motorv1.h,108 :: 		}
L_end_Timer1_RunCounter:
	RETURN
; end of _Timer1_RunCounter

_qUART_TX_Config:

;motorv1.h,110 :: 		void qUART_TX_Config(void)
;motorv1.h,113 :: 		SPBRG = 129;   // set Baurate = 9600 mbps
	MOVLW      129
	MOVWF      SPBRG+0
;motorv1.h,114 :: 		TXSTA.BRGH = 1;   //High Speed
	BSF        TXSTA+0, 2
;motorv1.h,117 :: 		TXSTA.SYNC = 0;
	BCF        TXSTA+0, 4
;motorv1.h,118 :: 		RCSTA.SPEN=  1;
	BSF        RCSTA+0, 7
;motorv1.h,120 :: 		TRISC.B6=1;
	BSF        TRISC+0, 6
;motorv1.h,121 :: 		TRISC.B7=1;
	BSF        TRISC+0, 7
;motorv1.h,130 :: 		TXSTA.TXEN = 1;
	BSF        TXSTA+0, 5
;motorv1.h,136 :: 		}
L_end_qUART_TX_Config:
	RETURN
; end of _qUART_TX_Config

_qUART_RX_Config:

;motorv1.h,138 :: 		void qUART_RX_Config(void)
;motorv1.h,141 :: 		SPBRG = 129;    //Baudrate 9600 Mbps
	MOVLW      129
	MOVWF      SPBRG+0
;motorv1.h,142 :: 		TXSTA.BRGH = 1;
	BSF        TXSTA+0, 2
;motorv1.h,144 :: 		TXSTA.SYNC = 0;
	BCF        TXSTA+0, 4
;motorv1.h,145 :: 		RCSTA.SPEN=  1;
	BSF        RCSTA+0, 7
;motorv1.h,147 :: 		TRISC.B6=1;
	BSF        TRISC+0, 6
;motorv1.h,148 :: 		TRISC.B7=1;
	BSF        TRISC+0, 7
;motorv1.h,150 :: 		INTCON.GIE = 1;  //Enable Global Interrupt
	BSF        INTCON+0, 7
;motorv1.h,151 :: 		INTCON.PEIE = 1;   //Enable PEIE Interrupt
	BSF        INTCON+0, 6
;motorv1.h,152 :: 		PIE1.RCIE= 1;      // Enable RX UART interrupt
	BSF        PIE1+0, 5
;motorv1.h,155 :: 		RCSTA.CREN = 1;
	BSF        RCSTA+0, 4
;motorv1.h,156 :: 		}
L_end_qUART_RX_Config:
	RETURN
; end of _qUART_RX_Config

_qUART_Transmit:

;motorv1.h,159 :: 		void qUART_Transmit(unsigned char dataUART)
;motorv1.h,161 :: 		while(!TXSTA.TRMT);
L_qUART_Transmit5:
	BTFSC      TXSTA+0, 1
	GOTO       L_qUART_Transmit6
	GOTO       L_qUART_Transmit5
L_qUART_Transmit6:
;motorv1.h,162 :: 		TXREG = dataUART;
	MOVF       FARG_qUART_Transmit_dataUART+0, 0
	MOVWF      TXREG+0
;motorv1.h,163 :: 		}
L_end_qUART_Transmit:
	RETURN
; end of _qUART_Transmit

_qUART_PutString:

;motorv1.h,164 :: 		void qUART_PutString(unsigned char* s)
;motorv1.h,166 :: 		while(*s)
L_qUART_PutString7:
	MOVF       FARG_qUART_PutString_s+0, 0
	MOVWF      FSR
	MOVF       INDF+0, 0
	BTFSC      STATUS+0, 2
	GOTO       L_qUART_PutString8
;motorv1.h,168 :: 		qUART_Transmit(*s++);
	MOVF       FARG_qUART_PutString_s+0, 0
	MOVWF      FSR
	MOVF       INDF+0, 0
	MOVWF      FARG_qUART_Transmit_dataUART+0
	CALL       _qUART_Transmit+0
	INCF       FARG_qUART_PutString_s+0, 1
;motorv1.h,169 :: 		}
	GOTO       L_qUART_PutString7
L_qUART_PutString8:
;motorv1.h,170 :: 		}
L_end_qUART_PutString:
	RETURN
; end of _qUART_PutString

_qUART_ReadData:

;motorv1.h,172 :: 		unsigned char qUART_ReadData()
;motorv1.h,174 :: 		while(!PIR1.RCIF);
L_qUART_ReadData9:
	BTFSC      PIR1+0, 5
	GOTO       L_qUART_ReadData10
	GOTO       L_qUART_ReadData9
L_qUART_ReadData10:
;motorv1.h,175 :: 		return RCREG;
	MOVF       RCREG+0, 0
	MOVWF      R0+0
;motorv1.h,176 :: 		}
L_end_qUART_ReadData:
	RETURN
; end of _qUART_ReadData

_main:

;Motorv1.c,17 :: 		void main() {
;Motorv1.c,19 :: 		ADCON1 |=0x07;
	MOVLW      7
	IORWF      ADCON1+0, 1
;Motorv1.c,20 :: 		TRISA.B3 =0;
	BCF        TRISA+0, 3
;Motorv1.c,21 :: 		TRISA.B4 =0;
	BCF        TRISA+0, 4
;Motorv1.c,22 :: 		TRISA.B5 =0;
	BCF        TRISA+0, 5
;Motorv1.c,23 :: 		PORTA.B3 = 1;
	BSF        PORTA+0, 3
;Motorv1.c,24 :: 		PORTA.B4 = 1;
	BSF        PORTA+0, 4
;Motorv1.c,25 :: 		PORTA.B5 = 1;
	BSF        PORTA+0, 5
;Motorv1.c,29 :: 		qUART_TX_Config();
	CALL       _qUART_TX_Config+0
;Motorv1.c,30 :: 		Timer0_Config();
	CALL       _Timer0_Config+0
;Motorv1.c,31 :: 		PWM2_Config();
	CALL       _PWM2_Config+0
;Motorv1.c,36 :: 		Read_Encoder_InputCapture_ChannelAB();
	CALL       _Read_Encoder_InputCapture_ChannelAB+0
;Motorv1.c,40 :: 		while(1)
L_main11:
;Motorv1.c,43 :: 		}
	GOTO       L_main11
;Motorv1.c,44 :: 		}
L_end_main:
	GOTO       $+0
; end of _main

_Motor_Control_Position:

;Motorv1.c,48 :: 		void Motor_Control_Position(int32_t SP , int32_t PV)
;Motorv1.c,52 :: 		Error = (double)(SP-PV);
	MOVF       FARG_Motor_Control_Position_SP+0, 0
	MOVWF      R0+0
	MOVF       FARG_Motor_Control_Position_SP+1, 0
	MOVWF      R0+1
	MOVF       FARG_Motor_Control_Position_SP+2, 0
	MOVWF      R0+2
	MOVF       FARG_Motor_Control_Position_SP+3, 0
	MOVWF      R0+3
	MOVF       FARG_Motor_Control_Position_PV+0, 0
	SUBWF      R0+0, 1
	MOVF       FARG_Motor_Control_Position_PV+1, 0
	BTFSS      STATUS+0, 0
	INCFSZ     FARG_Motor_Control_Position_PV+1, 0
	SUBWF      R0+1, 1
	MOVF       FARG_Motor_Control_Position_PV+2, 0
	BTFSS      STATUS+0, 0
	INCFSZ     FARG_Motor_Control_Position_PV+2, 0
	SUBWF      R0+2, 1
	MOVF       FARG_Motor_Control_Position_PV+3, 0
	BTFSS      STATUS+0, 0
	INCFSZ     FARG_Motor_Control_Position_PV+3, 0
	SUBWF      R0+3, 1
	CALL       _longint2double+0
	MOVF       R0+0, 0
	MOVWF      FLOC__Motor_Control_Position+12
	MOVF       R0+1, 0
	MOVWF      FLOC__Motor_Control_Position+13
	MOVF       R0+2, 0
	MOVWF      FLOC__Motor_Control_Position+14
	MOVF       R0+3, 0
	MOVWF      FLOC__Motor_Control_Position+15
	MOVF       FLOC__Motor_Control_Position+12, 0
	MOVWF      _Error+0
	MOVF       FLOC__Motor_Control_Position+13, 0
	MOVWF      _Error+1
	MOVF       FLOC__Motor_Control_Position+14, 0
	MOVWF      _Error+2
	MOVF       FLOC__Motor_Control_Position+15, 0
	MOVWF      _Error+3
;Motorv1.c,53 :: 		I_part = pre_I_Part + Error * T;
	MOVF       FLOC__Motor_Control_Position+12, 0
	MOVWF      R0+0
	MOVF       FLOC__Motor_Control_Position+13, 0
	MOVWF      R0+1
	MOVF       FLOC__Motor_Control_Position+14, 0
	MOVWF      R0+2
	MOVF       FLOC__Motor_Control_Position+15, 0
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
	MOVWF      FLOC__Motor_Control_Position+8
	MOVF       R0+1, 0
	MOVWF      FLOC__Motor_Control_Position+9
	MOVF       R0+2, 0
	MOVWF      FLOC__Motor_Control_Position+10
	MOVF       R0+3, 0
	MOVWF      FLOC__Motor_Control_Position+11
	MOVF       FLOC__Motor_Control_Position+8, 0
	MOVWF      _I_part+0
	MOVF       FLOC__Motor_Control_Position+9, 0
	MOVWF      _I_part+1
	MOVF       FLOC__Motor_Control_Position+10, 0
	MOVWF      _I_part+2
	MOVF       FLOC__Motor_Control_Position+11, 0
	MOVWF      _I_part+3
;Motorv1.c,54 :: 		D_part = (Error - pre_Error)/T;
	MOVF       _pre_Error+0, 0
	MOVWF      R4+0
	MOVF       _pre_Error+1, 0
	MOVWF      R4+1
	MOVF       _pre_Error+2, 0
	MOVWF      R4+2
	MOVF       _pre_Error+3, 0
	MOVWF      R4+3
	MOVF       FLOC__Motor_Control_Position+12, 0
	MOVWF      R0+0
	MOVF       FLOC__Motor_Control_Position+13, 0
	MOVWF      R0+1
	MOVF       FLOC__Motor_Control_Position+14, 0
	MOVWF      R0+2
	MOVF       FLOC__Motor_Control_Position+15, 0
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
	MOVWF      FLOC__Motor_Control_Position+4
	MOVF       R0+1, 0
	MOVWF      FLOC__Motor_Control_Position+5
	MOVF       R0+2, 0
	MOVWF      FLOC__Motor_Control_Position+6
	MOVF       R0+3, 0
	MOVWF      FLOC__Motor_Control_Position+7
	MOVF       FLOC__Motor_Control_Position+4, 0
	MOVWF      _D_part+0
	MOVF       FLOC__Motor_Control_Position+5, 0
	MOVWF      _D_part+1
	MOVF       FLOC__Motor_Control_Position+6, 0
	MOVWF      _D_part+2
	MOVF       FLOC__Motor_Control_Position+7, 0
	MOVWF      _D_part+3
;Motorv1.c,55 :: 		Output = Kp*Error + Ki*I_part + Kd*D_part;
	MOVF       _Kp+0, 0
	MOVWF      R0+0
	MOVF       _Kp+1, 0
	MOVWF      R0+1
	MOVF       _Kp+2, 0
	MOVWF      R0+2
	MOVF       _Kp+3, 0
	MOVWF      R0+3
	MOVF       FLOC__Motor_Control_Position+12, 0
	MOVWF      R4+0
	MOVF       FLOC__Motor_Control_Position+13, 0
	MOVWF      R4+1
	MOVF       FLOC__Motor_Control_Position+14, 0
	MOVWF      R4+2
	MOVF       FLOC__Motor_Control_Position+15, 0
	MOVWF      R4+3
	CALL       _Mul_32x32_FP+0
	MOVF       R0+0, 0
	MOVWF      FLOC__Motor_Control_Position+0
	MOVF       R0+1, 0
	MOVWF      FLOC__Motor_Control_Position+1
	MOVF       R0+2, 0
	MOVWF      FLOC__Motor_Control_Position+2
	MOVF       R0+3, 0
	MOVWF      FLOC__Motor_Control_Position+3
	MOVF       _Ki+0, 0
	MOVWF      R0+0
	MOVF       _Ki+1, 0
	MOVWF      R0+1
	MOVF       _Ki+2, 0
	MOVWF      R0+2
	MOVF       _Ki+3, 0
	MOVWF      R0+3
	MOVF       FLOC__Motor_Control_Position+8, 0
	MOVWF      R4+0
	MOVF       FLOC__Motor_Control_Position+9, 0
	MOVWF      R4+1
	MOVF       FLOC__Motor_Control_Position+10, 0
	MOVWF      R4+2
	MOVF       FLOC__Motor_Control_Position+11, 0
	MOVWF      R4+3
	CALL       _Mul_32x32_FP+0
	MOVF       FLOC__Motor_Control_Position+0, 0
	MOVWF      R4+0
	MOVF       FLOC__Motor_Control_Position+1, 0
	MOVWF      R4+1
	MOVF       FLOC__Motor_Control_Position+2, 0
	MOVWF      R4+2
	MOVF       FLOC__Motor_Control_Position+3, 0
	MOVWF      R4+3
	CALL       _Add_32x32_FP+0
	MOVF       R0+0, 0
	MOVWF      FLOC__Motor_Control_Position+0
	MOVF       R0+1, 0
	MOVWF      FLOC__Motor_Control_Position+1
	MOVF       R0+2, 0
	MOVWF      FLOC__Motor_Control_Position+2
	MOVF       R0+3, 0
	MOVWF      FLOC__Motor_Control_Position+3
	MOVF       _Kd+0, 0
	MOVWF      R0+0
	MOVF       _Kd+1, 0
	MOVWF      R0+1
	MOVF       _Kd+2, 0
	MOVWF      R0+2
	MOVF       _Kd+3, 0
	MOVWF      R0+3
	MOVF       FLOC__Motor_Control_Position+4, 0
	MOVWF      R4+0
	MOVF       FLOC__Motor_Control_Position+5, 0
	MOVWF      R4+1
	MOVF       FLOC__Motor_Control_Position+6, 0
	MOVWF      R4+2
	MOVF       FLOC__Motor_Control_Position+7, 0
	MOVWF      R4+3
	CALL       _Mul_32x32_FP+0
	MOVF       FLOC__Motor_Control_Position+0, 0
	MOVWF      R4+0
	MOVF       FLOC__Motor_Control_Position+1, 0
	MOVWF      R4+1
	MOVF       FLOC__Motor_Control_Position+2, 0
	MOVWF      R4+2
	MOVF       FLOC__Motor_Control_Position+3, 0
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
;Motorv1.c,56 :: 		pre_Error = Error;
	MOVF       FLOC__Motor_Control_Position+12, 0
	MOVWF      _pre_Error+0
	MOVF       FLOC__Motor_Control_Position+13, 0
	MOVWF      _pre_Error+1
	MOVF       FLOC__Motor_Control_Position+14, 0
	MOVWF      _pre_Error+2
	MOVF       FLOC__Motor_Control_Position+15, 0
	MOVWF      _pre_Error+3
;Motorv1.c,57 :: 		pre_I_Part = I_part;
	MOVF       FLOC__Motor_Control_Position+8, 0
	MOVWF      _pre_I_Part+0
	MOVF       FLOC__Motor_Control_Position+9, 0
	MOVWF      _pre_I_Part+1
	MOVF       FLOC__Motor_Control_Position+10, 0
	MOVWF      _pre_I_Part+2
	MOVF       FLOC__Motor_Control_Position+11, 0
	MOVWF      _pre_I_Part+3
;Motorv1.c,60 :: 		if ((Output >=624) || (Output <= -624))
	MOVLW      0
	MOVWF      R4+0
	MOVLW      0
	MOVWF      R4+1
	MOVLW      28
	MOVWF      R4+2
	MOVLW      136
	MOVWF      R4+3
	CALL       _Compare_Double+0
	MOVLW      1
	BTFSS      STATUS+0, 0
	MOVLW      0
	MOVWF      R0+0
	MOVF       R0+0, 0
	BTFSS      STATUS+0, 2
	GOTO       L__Motor_Control_Position30
	MOVF       _Output+0, 0
	MOVWF      R4+0
	MOVF       _Output+1, 0
	MOVWF      R4+1
	MOVF       _Output+2, 0
	MOVWF      R4+2
	MOVF       _Output+3, 0
	MOVWF      R4+3
	MOVLW      0
	MOVWF      R0+0
	MOVLW      0
	MOVWF      R0+1
	MOVLW      156
	MOVWF      R0+2
	MOVLW      136
	MOVWF      R0+3
	CALL       _Compare_Double+0
	MOVLW      1
	BTFSS      STATUS+0, 0
	MOVLW      0
	MOVWF      R0+0
	MOVF       R0+0, 0
	BTFSS      STATUS+0, 2
	GOTO       L__Motor_Control_Position30
	GOTO       L_Motor_Control_Position15
L__Motor_Control_Position30:
;Motorv1.c,62 :: 		Output_int = 624;
	MOVLW      112
	MOVWF      _Output_int+0
	MOVLW      2
	MOVWF      _Output_int+1
;Motorv1.c,63 :: 		}
	GOTO       L_Motor_Control_Position16
L_Motor_Control_Position15:
;Motorv1.c,66 :: 		Output_int = (uint16_t)(fabs(Output))+210;
	MOVF       _Output+0, 0
	MOVWF      FARG_fabs_d+0
	MOVF       _Output+1, 0
	MOVWF      FARG_fabs_d+1
	MOVF       _Output+2, 0
	MOVWF      FARG_fabs_d+2
	MOVF       _Output+3, 0
	MOVWF      FARG_fabs_d+3
	CALL       _fabs+0
	CALL       _double2word+0
	MOVLW      210
	ADDWF      R0+0, 0
	MOVWF      _Output_int+0
	MOVF       R0+1, 0
	BTFSC      STATUS+0, 0
	ADDLW      1
	MOVWF      _Output_int+1
;Motorv1.c,67 :: 		}
L_Motor_Control_Position16:
;Motorv1.c,68 :: 		if ( Error >1)
	MOVF       _Error+0, 0
	MOVWF      R4+0
	MOVF       _Error+1, 0
	MOVWF      R4+1
	MOVF       _Error+2, 0
	MOVWF      R4+2
	MOVF       _Error+3, 0
	MOVWF      R4+3
	MOVLW      0
	MOVWF      R0+0
	MOVLW      0
	MOVWF      R0+1
	MOVLW      0
	MOVWF      R0+2
	MOVLW      127
	MOVWF      R0+3
	CALL       _Compare_Double+0
	MOVLW      1
	BTFSC      STATUS+0, 0
	MOVLW      0
	MOVWF      R0+0
	MOVF       R0+0, 0
	BTFSC      STATUS+0, 2
	GOTO       L_Motor_Control_Position17
;Motorv1.c,70 :: 		PORTA.B1=1;
	BSF        PORTA+0, 1
;Motorv1.c,72 :: 		PWM2_Set_Duty(Output_int);
	MOVF       _Output_int+0, 0
	MOVWF      FARG_PWM2_Set_Duty_dc+0
	MOVF       _Output_int+1, 0
	MOVWF      FARG_PWM2_Set_Duty_dc+1
	CALL       _PWM2_Set_Duty+0
;Motorv1.c,73 :: 		}
	GOTO       L_Motor_Control_Position18
L_Motor_Control_Position17:
;Motorv1.c,74 :: 		else if  (Error<-1)  //2 degree
	MOVLW      0
	MOVWF      R4+0
	MOVLW      0
	MOVWF      R4+1
	MOVLW      128
	MOVWF      R4+2
	MOVLW      127
	MOVWF      R4+3
	MOVF       _Error+0, 0
	MOVWF      R0+0
	MOVF       _Error+1, 0
	MOVWF      R0+1
	MOVF       _Error+2, 0
	MOVWF      R0+2
	MOVF       _Error+3, 0
	MOVWF      R0+3
	CALL       _Compare_Double+0
	MOVLW      1
	BTFSC      STATUS+0, 0
	MOVLW      0
	MOVWF      R0+0
	MOVF       R0+0, 0
	BTFSC      STATUS+0, 2
	GOTO       L_Motor_Control_Position19
;Motorv1.c,76 :: 		PORTA.B1=0;
	BCF        PORTA+0, 1
;Motorv1.c,78 :: 		PWM2_Set_Duty(Output_int);
	MOVF       _Output_int+0, 0
	MOVWF      FARG_PWM2_Set_Duty_dc+0
	MOVF       _Output_int+1, 0
	MOVWF      FARG_PWM2_Set_Duty_dc+1
	CALL       _PWM2_Set_Duty+0
;Motorv1.c,79 :: 		}
	GOTO       L_Motor_Control_Position20
L_Motor_Control_Position19:
;Motorv1.c,82 :: 		PORTA.B1=0;
	BCF        PORTA+0, 1
;Motorv1.c,83 :: 		PORTA.B2=0;
	BCF        PORTA+0, 2
;Motorv1.c,84 :: 		PWM2_Set_Duty(1);
	MOVLW      1
	MOVWF      FARG_PWM2_Set_Duty_dc+0
	MOVLW      0
	MOVWF      FARG_PWM2_Set_Duty_dc+1
	CALL       _PWM2_Set_Duty+0
;Motorv1.c,85 :: 		}
L_Motor_Control_Position20:
L_Motor_Control_Position18:
;Motorv1.c,87 :: 		}
L_end_Motor_Control_Position:
	RETURN
; end of _Motor_Control_Position

_Interrupt:
	MOVWF      R15+0
	SWAPF      STATUS+0, 0
	CLRF       STATUS+0
	MOVWF      ___saveSTATUS+0
	MOVF       PCLATH+0, 0
	MOVWF      ___savePCLATH+0
	CLRF       PCLATH+0

;Motorv1.c,90 :: 		void Interrupt ()
;Motorv1.c,93 :: 		if (PIR1.CCP1IF == 1)
	BTFSS      PIR1+0, 2
	GOTO       L_Interrupt21
;Motorv1.c,95 :: 		PORTA.B3 = !PORTA.B3;
	MOVLW      8
	XORWF      PORTA+0, 1
;Motorv1.c,96 :: 		if (CCPR1 >=65535)
	MOVLW      255
	SUBWF      CCPR1+1, 0
	BTFSS      STATUS+0, 2
	GOTO       L__Interrupt51
	MOVLW      255
	SUBWF      CCPR1+0, 0
L__Interrupt51:
	BTFSS      STATUS+0, 0
	GOTO       L_Interrupt22
;Motorv1.c,98 :: 		CCPR1=0;
	CLRF       CCPR1+0
	CLRF       CCPR1+1
;Motorv1.c,99 :: 		TMR1H= TMR1L=0;
	CLRF       TMR1L+0
	MOVF       TMR1L+0, 0
	MOVWF      TMR1H+0
;Motorv1.c,100 :: 		}
L_Interrupt22:
;Motorv1.c,102 :: 		if(PORTB.B4 == 0)
	BTFSC      PORTB+0, 4
	GOTO       L_Interrupt23
;Motorv1.c,104 :: 		PORTA.B4 = !PORTA.B4;
	MOVLW      16
	XORWF      PORTA+0, 1
;Motorv1.c,105 :: 		direct = 0;
	CLRF       _direct+0
;Motorv1.c,106 :: 		count++;
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
;Motorv1.c,107 :: 		}
	GOTO       L_Interrupt24
L_Interrupt23:
;Motorv1.c,109 :: 		else if(PORTB.B4 == 1)
	BTFSS      PORTB+0, 4
	GOTO       L_Interrupt25
;Motorv1.c,111 :: 		PORTA.B5 = !PORTA.B5;
	MOVLW      32
	XORWF      PORTA+0, 1
;Motorv1.c,112 :: 		direct = 1;
	MOVLW      1
	MOVWF      _direct+0
;Motorv1.c,113 :: 		count--;
	MOVLW      1
	SUBWF      _count+0, 1
	BTFSS      STATUS+0, 0
	SUBWF      _count+1, 1
	BTFSS      STATUS+0, 0
	SUBWF      _count+2, 1
	BTFSS      STATUS+0, 0
	SUBWF      _count+3, 1
;Motorv1.c,114 :: 		}
L_Interrupt25:
L_Interrupt24:
;Motorv1.c,116 :: 		if (count >= 330)
	MOVLW      128
	XORWF      _count+3, 0
	MOVWF      R0+0
	MOVLW      128
	SUBWF      R0+0, 0
	BTFSS      STATUS+0, 2
	GOTO       L__Interrupt52
	MOVLW      0
	SUBWF      _count+2, 0
	BTFSS      STATUS+0, 2
	GOTO       L__Interrupt52
	MOVLW      1
	SUBWF      _count+1, 0
	BTFSS      STATUS+0, 2
	GOTO       L__Interrupt52
	MOVLW      74
	SUBWF      _count+0, 0
L__Interrupt52:
	BTFSS      STATUS+0, 0
	GOTO       L_Interrupt26
;Motorv1.c,118 :: 		rotary++;
	MOVF       _rotary+0, 0
	MOVWF      R0+0
	MOVF       _rotary+1, 0
	MOVWF      R0+1
	MOVF       _rotary+2, 0
	MOVWF      R0+2
	MOVF       _rotary+3, 0
	MOVWF      R0+3
	INCF       R0+0, 1
	BTFSC      STATUS+0, 2
	INCF       R0+1, 1
	BTFSC      STATUS+0, 2
	INCF       R0+2, 1
	BTFSC      STATUS+0, 2
	INCF       R0+3, 1
	MOVF       R0+0, 0
	MOVWF      _rotary+0
	MOVF       R0+1, 0
	MOVWF      _rotary+1
	MOVF       R0+2, 0
	MOVWF      _rotary+2
	MOVF       R0+3, 0
	MOVWF      _rotary+3
;Motorv1.c,119 :: 		count=0;
	CLRF       _count+0
	CLRF       _count+1
	CLRF       _count+2
	CLRF       _count+3
;Motorv1.c,125 :: 		}
	GOTO       L_Interrupt27
L_Interrupt26:
;Motorv1.c,126 :: 		else if (count <= -330)
	MOVLW      127
	MOVWF      R0+0
	MOVLW      128
	XORWF      _count+3, 0
	SUBWF      R0+0, 0
	BTFSS      STATUS+0, 2
	GOTO       L__Interrupt53
	MOVF       _count+2, 0
	SUBLW      255
	BTFSS      STATUS+0, 2
	GOTO       L__Interrupt53
	MOVF       _count+1, 0
	SUBLW      254
	BTFSS      STATUS+0, 2
	GOTO       L__Interrupt53
	MOVF       _count+0, 0
	SUBLW      182
L__Interrupt53:
	BTFSS      STATUS+0, 0
	GOTO       L_Interrupt28
;Motorv1.c,128 :: 		rotary--;
	MOVLW      1
	SUBWF      _rotary+0, 1
	BTFSS      STATUS+0, 0
	SUBWF      _rotary+1, 1
	BTFSS      STATUS+0, 0
	SUBWF      _rotary+2, 1
	BTFSS      STATUS+0, 0
	SUBWF      _rotary+3, 1
;Motorv1.c,129 :: 		count=0;
	CLRF       _count+0
	CLRF       _count+1
	CLRF       _count+2
	CLRF       _count+3
;Motorv1.c,135 :: 		}
L_Interrupt28:
L_Interrupt27:
;Motorv1.c,136 :: 		Position_Angles_PV =(count*360)/330;
	MOVF       _count+0, 0
	MOVWF      R0+0
	MOVF       _count+1, 0
	MOVWF      R0+1
	MOVF       _count+2, 0
	MOVWF      R0+2
	MOVF       _count+3, 0
	MOVWF      R0+3
	MOVLW      104
	MOVWF      R4+0
	MOVLW      1
	MOVWF      R4+1
	CLRF       R4+2
	CLRF       R4+3
	CALL       _Mul_32x32_U+0
	MOVLW      74
	MOVWF      R4+0
	MOVLW      1
	MOVWF      R4+1
	CLRF       R4+2
	CLRF       R4+3
	CALL       _Div_32x32_S+0
	MOVF       R0+0, 0
	MOVWF      _Position_Angles_PV+0
	MOVF       R0+1, 0
	MOVWF      _Position_Angles_PV+1
	MOVF       R0+2, 0
	MOVWF      _Position_Angles_PV+2
	MOVF       R0+3, 0
	MOVWF      _Position_Angles_PV+3
;Motorv1.c,145 :: 		PIR1.CCP1IF = 0; // Clear Flag Capture
	BCF        PIR1+0, 2
;Motorv1.c,146 :: 		}//CCP1
L_Interrupt21:
;Motorv1.c,148 :: 		if (INTCON.TMR0IF == 1)
	BTFSS      INTCON+0, 2
	GOTO       L_Interrupt29
;Motorv1.c,150 :: 		TMR0= 100;
	MOVLW      100
	MOVWF      TMR0+0
;Motorv1.c,152 :: 		Motor_Control_Position(Position_SP,Position_Angles_PV);
	MOVF       _Position_SP+0, 0
	MOVWF      FARG_Motor_Control_Position_SP+0
	MOVF       _Position_SP+1, 0
	MOVWF      FARG_Motor_Control_Position_SP+1
	MOVF       _Position_SP+2, 0
	MOVWF      FARG_Motor_Control_Position_SP+2
	MOVF       _Position_SP+3, 0
	MOVWF      FARG_Motor_Control_Position_SP+3
	MOVF       _Position_Angles_PV+0, 0
	MOVWF      FARG_Motor_Control_Position_PV+0
	MOVF       _Position_Angles_PV+1, 0
	MOVWF      FARG_Motor_Control_Position_PV+1
	MOVF       _Position_Angles_PV+2, 0
	MOVWF      FARG_Motor_Control_Position_PV+2
	MOVF       _Position_Angles_PV+3, 0
	MOVWF      FARG_Motor_Control_Position_PV+3
	CALL       _Motor_Control_Position+0
;Motorv1.c,154 :: 		INTCON.TMR0IF = 0;
	BCF        INTCON+0, 2
;Motorv1.c,155 :: 		}//TMR0
L_Interrupt29:
;Motorv1.c,156 :: 		}//Interrupt
L_end_Interrupt:
L__Interrupt50:
	MOVF       ___savePCLATH+0, 0
	MOVWF      PCLATH+0
	SWAPF      ___saveSTATUS+0, 0
	MOVWF      STATUS+0
	SWAPF      R15+0, 1
	SWAPF      R15+0, 0
	RETFIE
; end of _Interrupt
