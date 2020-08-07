
_main:

;math.c,12 :: 		void main() {
;math.c,13 :: 		a=1/(double)b*9;
	MOVF       _b+0, 0
	MOVWF      R0+0
	MOVF       _b+1, 0
	MOVWF      R0+1
	CALL       _int2double+0
	MOVF       R0+0, 0
	MOVWF      R4+0
	MOVF       R0+1, 0
	MOVWF      R4+1
	MOVF       R0+2, 0
	MOVWF      R4+2
	MOVF       R0+3, 0
	MOVWF      R4+3
	MOVLW      0
	MOVWF      R0+0
	MOVLW      0
	MOVWF      R0+1
	MOVLW      0
	MOVWF      R0+2
	MOVLW      127
	MOVWF      R0+3
	CALL       _Div_32x32_FP+0
	MOVLW      0
	MOVWF      R4+0
	MOVLW      0
	MOVWF      R4+1
	MOVLW      16
	MOVWF      R4+2
	MOVLW      130
	MOVWF      R4+3
	CALL       _Mul_32x32_FP+0
	MOVF       R0+0, 0
	MOVWF      _a+0
	MOVF       R0+1, 0
	MOVWF      _a+1
	MOVF       R0+2, 0
	MOVWF      _a+2
	MOVF       R0+3, 0
	MOVWF      _a+3
;math.c,14 :: 		Maths();
	CALL       _Maths+0
;math.c,15 :: 		}
L_end_main:
	GOTO       $+0
; end of _main

_Maths:

;math.c,16 :: 		void Maths(void)
;math.c,22 :: 		while(1)
L_Maths0:
;math.c,25 :: 		Error = (double)(SP-PV);
	MOVF       _SP+0, 0
	MOVWF      R0+0
	MOVF       _SP+1, 0
	MOVWF      R0+1
	MOVF       _SP+2, 0
	MOVWF      R0+2
	MOVF       _SP+3, 0
	MOVWF      R0+3
	MOVF       _PV+0, 0
	SUBWF      R0+0, 1
	MOVF       _PV+1, 0
	BTFSS      STATUS+0, 0
	INCFSZ     _PV+1, 0
	SUBWF      R0+1, 1
	MOVF       _PV+2, 0
	BTFSS      STATUS+0, 0
	INCFSZ     _PV+2, 0
	SUBWF      R0+2, 1
	MOVF       _PV+3, 0
	BTFSS      STATUS+0, 0
	INCFSZ     _PV+3, 0
	SUBWF      R0+3, 1
	CALL       _longint2double+0
	MOVF       R0+0, 0
	MOVWF      FLOC__Maths+12
	MOVF       R0+1, 0
	MOVWF      FLOC__Maths+13
	MOVF       R0+2, 0
	MOVWF      FLOC__Maths+14
	MOVF       R0+3, 0
	MOVWF      FLOC__Maths+15
	MOVF       FLOC__Maths+12, 0
	MOVWF      _Error+0
	MOVF       FLOC__Maths+13, 0
	MOVWF      _Error+1
	MOVF       FLOC__Maths+14, 0
	MOVWF      _Error+2
	MOVF       FLOC__Maths+15, 0
	MOVWF      _Error+3
;math.c,26 :: 		I_part = pre_I_Part + Error * T;
	MOVF       FLOC__Maths+12, 0
	MOVWF      R0+0
	MOVF       FLOC__Maths+13, 0
	MOVWF      R0+1
	MOVF       FLOC__Maths+14, 0
	MOVWF      R0+2
	MOVF       FLOC__Maths+15, 0
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
	MOVWF      FLOC__Maths+8
	MOVF       R0+1, 0
	MOVWF      FLOC__Maths+9
	MOVF       R0+2, 0
	MOVWF      FLOC__Maths+10
	MOVF       R0+3, 0
	MOVWF      FLOC__Maths+11
	MOVF       FLOC__Maths+8, 0
	MOVWF      _I_part+0
	MOVF       FLOC__Maths+9, 0
	MOVWF      _I_part+1
	MOVF       FLOC__Maths+10, 0
	MOVWF      _I_part+2
	MOVF       FLOC__Maths+11, 0
	MOVWF      _I_part+3
;math.c,27 :: 		D_part = (Error - pre_Error)/T;
	MOVF       _pre_Error+0, 0
	MOVWF      R4+0
	MOVF       _pre_Error+1, 0
	MOVWF      R4+1
	MOVF       _pre_Error+2, 0
	MOVWF      R4+2
	MOVF       _pre_Error+3, 0
	MOVWF      R4+3
	MOVF       FLOC__Maths+12, 0
	MOVWF      R0+0
	MOVF       FLOC__Maths+13, 0
	MOVWF      R0+1
	MOVF       FLOC__Maths+14, 0
	MOVWF      R0+2
	MOVF       FLOC__Maths+15, 0
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
	MOVWF      FLOC__Maths+4
	MOVF       R0+1, 0
	MOVWF      FLOC__Maths+5
	MOVF       R0+2, 0
	MOVWF      FLOC__Maths+6
	MOVF       R0+3, 0
	MOVWF      FLOC__Maths+7
	MOVF       FLOC__Maths+4, 0
	MOVWF      _D_part+0
	MOVF       FLOC__Maths+5, 0
	MOVWF      _D_part+1
	MOVF       FLOC__Maths+6, 0
	MOVWF      _D_part+2
	MOVF       FLOC__Maths+7, 0
	MOVWF      _D_part+3
;math.c,28 :: 		Output = Kp*Error + Ki*I_part + Kd*D_part;
	MOVF       _Kp+0, 0
	MOVWF      R0+0
	MOVF       _Kp+1, 0
	MOVWF      R0+1
	MOVF       _Kp+2, 0
	MOVWF      R0+2
	MOVF       _Kp+3, 0
	MOVWF      R0+3
	MOVF       FLOC__Maths+12, 0
	MOVWF      R4+0
	MOVF       FLOC__Maths+13, 0
	MOVWF      R4+1
	MOVF       FLOC__Maths+14, 0
	MOVWF      R4+2
	MOVF       FLOC__Maths+15, 0
	MOVWF      R4+3
	CALL       _Mul_32x32_FP+0
	MOVF       R0+0, 0
	MOVWF      FLOC__Maths+0
	MOVF       R0+1, 0
	MOVWF      FLOC__Maths+1
	MOVF       R0+2, 0
	MOVWF      FLOC__Maths+2
	MOVF       R0+3, 0
	MOVWF      FLOC__Maths+3
	MOVF       _Ki+0, 0
	MOVWF      R0+0
	MOVF       _Ki+1, 0
	MOVWF      R0+1
	MOVF       _Ki+2, 0
	MOVWF      R0+2
	MOVF       _Ki+3, 0
	MOVWF      R0+3
	MOVF       FLOC__Maths+8, 0
	MOVWF      R4+0
	MOVF       FLOC__Maths+9, 0
	MOVWF      R4+1
	MOVF       FLOC__Maths+10, 0
	MOVWF      R4+2
	MOVF       FLOC__Maths+11, 0
	MOVWF      R4+3
	CALL       _Mul_32x32_FP+0
	MOVF       FLOC__Maths+0, 0
	MOVWF      R4+0
	MOVF       FLOC__Maths+1, 0
	MOVWF      R4+1
	MOVF       FLOC__Maths+2, 0
	MOVWF      R4+2
	MOVF       FLOC__Maths+3, 0
	MOVWF      R4+3
	CALL       _Add_32x32_FP+0
	MOVF       R0+0, 0
	MOVWF      FLOC__Maths+0
	MOVF       R0+1, 0
	MOVWF      FLOC__Maths+1
	MOVF       R0+2, 0
	MOVWF      FLOC__Maths+2
	MOVF       R0+3, 0
	MOVWF      FLOC__Maths+3
	MOVF       _Kd+0, 0
	MOVWF      R0+0
	MOVF       _Kd+1, 0
	MOVWF      R0+1
	MOVF       _Kd+2, 0
	MOVWF      R0+2
	MOVF       _Kd+3, 0
	MOVWF      R0+3
	MOVF       FLOC__Maths+4, 0
	MOVWF      R4+0
	MOVF       FLOC__Maths+5, 0
	MOVWF      R4+1
	MOVF       FLOC__Maths+6, 0
	MOVWF      R4+2
	MOVF       FLOC__Maths+7, 0
	MOVWF      R4+3
	CALL       _Mul_32x32_FP+0
	MOVF       FLOC__Maths+0, 0
	MOVWF      R4+0
	MOVF       FLOC__Maths+1, 0
	MOVWF      R4+1
	MOVF       FLOC__Maths+2, 0
	MOVWF      R4+2
	MOVF       FLOC__Maths+3, 0
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
;math.c,29 :: 		pre_Error = Error;
	MOVF       FLOC__Maths+12, 0
	MOVWF      _pre_Error+0
	MOVF       FLOC__Maths+13, 0
	MOVWF      _pre_Error+1
	MOVF       FLOC__Maths+14, 0
	MOVWF      _pre_Error+2
	MOVF       FLOC__Maths+15, 0
	MOVWF      _pre_Error+3
;math.c,30 :: 		pre_I_Part = I_part;
	MOVF       FLOC__Maths+8, 0
	MOVWF      _pre_I_Part+0
	MOVF       FLOC__Maths+9, 0
	MOVWF      _pre_I_Part+1
	MOVF       FLOC__Maths+10, 0
	MOVWF      _pre_I_Part+2
	MOVF       FLOC__Maths+11, 0
	MOVWF      _pre_I_Part+3
;math.c,31 :: 		}
	GOTO       L_Maths0
;math.c,32 :: 		}
L_end_Maths:
	RETURN
; end of _Maths
