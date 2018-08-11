; Fuses High=0xFA, Low = 0xEA

.device		ATtiny13A
.include	"tn13Adef.inc"

.def		Angle_L	= r1
.def		Angle_H	= r2

.def		Zero_L	= r3
.def		Zero_H	= r4

.def		RealA_L = r5
.def		RealA_H = r6

.def		FLAG	= r7				; ���	��������
										;  0	���� ��������� ���������
										;  1	������ EEPROM. �������� �������� Zero_L
										;  2	������ EEPROM. �������� �������� Zero_H
										;  3	������ EEPROM. �������� �������� ������������ �����
										;  4	������ EEPROM.
										;  5	
										;  6
										;  7

.def		COUNT	= r8
.def		COUNT_D	= r9

.def		CHECK	= r13

.def		COUNTA	= r24
.def		COUNTB	= r25

.equ CS =	PINB4						; PINB4 - ������ �Sn �� AS5048A
.equ MISO =	PINB2						; PINB2 - SPI MISO
.equ SCK =	PINB3						; PINB3 - SPI SCK
.equ PWM0 = PINB0						; PINB0 - ����� ��� OC0A
.equ PWM1 = PINB1						; PINB1 - ����� ��� OC0B


; EEPROM
.eseg
.org 47		ZERO0L:		.db	0xC4			; ������������� �������� (����)
.org 6		ZERO0H:		.db	0x27
.org 39		ZERO1L:		.db	0xC4
.org 54		ZERO1H:		.db	0x27
.org 12		ZERO2L:		.db	0xC4
.org 29		ZERO2H:		.db	0x27
.org 11		CS0:		.db	0x97
.org 36		CS1:		.db	0x97
.org 2		CS2:		.db	0x97


.dseg
.org	SRAM_START

.cseg

; ������� �������� ����������
.org 0
			rjmp	RESET				; Reset Handler
;			reti ;			rjmp	EXT_INT0			; IRQ0 Handler
;			reti ;			rjmp	PCINT0				; PCINT0 Handler
.org OVF0addr
			rjmp	TIM0_OVF			; Timer0 Overflow Handler
;			reti ;			rjmp	EE_RDY				; EEPROM Ready Handler
;			reti ;			rjmp	ANA_COMP			; Analog Comparator Handler
;			reti ;			rjmp	TIM0_COMPA			; Timer0 CompareA Handler
;			reti ;			rjmp	TIM0_COMPB			; Timer0 CompareB Handler
;			reti ;			rjmp	WATCHDOG			; Watchdog Interrupt Handler
;			reti ;			rjmp	ADC					; ADC Conversion Handler


.org	INT_VECTORS_SIZE

RESET:		
			cli							; ��������� ����������

			ldi		r16, RAMEND			; ���������� ��������� ����� � ����� ����������� ������
			out		SPL, r16	

			sei							; ��������� ����������

; ������������� ����������
			clr		FLAG
			clr		COUNT

			ldi		r16, 100
			mov		COUNT_D, r16

; ��������� �������� ������
			ldi		r16,(1<<PRADC)
			out		PRR, r16

			ldi		r16, (1<<ACD)
			out		ACSR, r16


; ������������ ������ �����-������
			ldi		r16, (1<<CS)|(1<<SCK)|(1<<PWM0)|(1<<PWM1)
			out		DDRB, r16

; ���������� SPI
			sbi		PORTB, CS			; ������� ������� �� �Sn AS5048A
			cbi		PORTB, SCK			; ������ ������� �� CLK
			sbi		PORTB, MISO			; �������� �� MISO

			rcall	R_SPI16				; ��������� ��������� ������� (������ ��� ������ ���������)

; ������������� ���				
			ldi		r16, (1<<COM0A1)|(1<<COM0B1)|(1<<WGM01)|(1<<WGM00)
			out		TCCR0A, r16
			ldi		r16, (1<<CS00)
			out		TCCR0B, r16
			ldi		r16, 10
			out		OCR0A, r16
			clr		r16
			out		OCR0B, r16

			ldi		r16, (1<<TOIE0)
			out		TIMSK0, r16

; ������������� WDT
			wdr
			ldi		r16, (1<<WDCE)|(1<<WDE)|(1<<WDP1)|(1<<WDP2)
			out		WDTCR, r16

NO_RESET:								; ��������� ������� ��������� �� EEPROM
			ldi		r17, low(ZERO0L)
			rcall	EEPROM_read
			mov		r10, r19

			ldi		r17, low(ZERO1L)
			rcall	EEPROM_read
			mov		r11, r19

			ldi		r17, low(ZERO2L)
			rcall	EEPROM_read
			mov		r12, r19

			cp		r10, r11
			breq	ZL_R10_OK
			
			cp		r10, r12
			breq	ZL_R10_OK

			cp		r11, r12
			breq	ZL_R11_OK

			cli
			mov		r16, FLAG
			sbr		r16, 0b00010010
			mov		FLAG, r16
			sei

			rjmp	MAIN_EEPROM_ERROR

ZL_R10_OK:
			mov		Zero_L, r10
			rjmp	ZL_END

ZL_R11_OK:
			mov		Zero_L, r11

ZL_END:
			ldi		r17, low(ZERO0H)
			rcall	EEPROM_read
			mov		r10, r19

			ldi		r17, low(ZERO1H)
			rcall	EEPROM_read
			mov		r11, r19

			ldi		r17, low(ZERO2H)
			rcall	EEPROM_read
			mov		r12, r19


			cp		r10, r11
			breq	ZH_R10_OK
			
			cp		r10, r12
			breq	ZH_R10_OK

			cp		r11, r12
			breq	ZH_R11_OK

			cli
			mov		r16, FLAG
			sbr		r16, 0b00010100
			mov		FLAG, r16
			sei

			rjmp	MAIN_EEPROM_ERROR

ZH_R10_OK:
			mov		Zero_H, r10
			rjmp	ZH_END

ZH_R11_OK:
			mov		Zero_H, r11

ZH_END:
			ldi		r17, low(CS0)
			rcall	EEPROM_read
			mov		r10, r19

			ldi		r17, low(CS1)
			rcall	EEPROM_read
			mov		r11, r19

			ldi		r17, low(CS2)
			rcall	EEPROM_read
			mov		r12, r19

			cp		r10, r11
			breq	CS_R10_OK
			
			cp		r10, r12
			breq	CS_R10_OK

			cp		r11, r12
			breq	CS_R11_OK

			cli
			mov		r16, FLAG
			sbr		r16, 0b00011000
			mov		FLAG, r16
			sei

			rjmp	MAIN_EEPROM_ERROR
			
CS_R10_OK:
			mov		CHECK, r10
			rjmp	CS_END

CS_R11_OK:
			mov		CHECK, r11

CS_END:								; ��������� ����������� �����
			rcall	CCS

			cp		CHECK, r19
			breq	LL_OK	
			
			cli
			mov		r16, FLAG
			sbr		r16, 0b00010000
			mov		FLAG, r16
			sei

			rjmp	MAIN_EEPROM_ERROR			

LL_OK:
			rjmp	MAIN

MAIN_EEPROM_ERROR:
			rcall	R_SPI16				; ��������� ��������� �������

			mov		Zero_H, Angle_H
			mov		Zero_L, Angle_L

MAIN:
			ldi		r16, (1<<SE)
			out		MCUCR, r16
			sleep						; �������� �� ���������� Timer0 Overflow

			sbrs	FLAG, 0
			rjmp	MAIN

			wdr

			cli
			mov		r16, FLAG
			cbr		r16, 0b00000001
			mov		FLAG, r16		
			sei
			
			rcall	R_SPI16				; ��������� ��������� �������

; ���������� �� ������� ������
			sbic	PINB, PINB5
			rjmp	S0

			dec		COUNT_D
			brne	S00

			rcall	ADJ0
K0:
			sbis	PINB, PINB5
			rjmp	K0

			rjmp	NO_RESET
; ����� ����� ���� ����������

S0:
			ldi		r16, 100
			mov		COUNT_D, r16

S00:
			mov		RealA_H, Zero_H
			mov		RealA_L, Zero_L

			sub		RealA_L, Angle_L
			sbc		RealA_H, Angle_H
			
			mov		r16, RealA_H				; ������� "������" ����
			andi	r16, 0b00111111
			mov		RealA_H, r16

			sbrs	RealA_H, 5
			rjmp	S1

; ��������������, ���� � EEPROM ������ �� �����
			sbrs	FLAG, 4
			rjmp	NO_ERR

			mov		Zero_H, Angle_H		
			mov		Zero_L, Angle_L
; ����� ��������������

NO_ERR:
			ldi		COUNTA, 10
			clr		COUNTB
			rjmp	S_END

S1:
			sbrs	RealA_H, 4
			rjmp	S2
			ser		COUNTA
			ser		COUNTB
			rjmp	S_END

S2:
			ldi		r17, 0x04
			ldi		r18, 0x15
			cp		r18, RealA_L
			cpc		r17, RealA_H
			brsh	S3

			ser		COUNTA

			rjmp	S4

S3:
			mov		r16, RealA_L
			mov		r17, RealA_H
			ldi		r18, 60

			rcall	mpy16u
			subi	r20, -10
			mov		COUNTA, r20

S4:
			ldi		r17, 0x03
			ldi		r18, 0x06
			cp		r18, RealA_L
			cpc		r17, RealA_H
			brsh	S7

			ldi		r17, 0x0F
			ldi		r18, 0xD3
			cp		RealA_L, r18
			cpc		RealA_H, r17
			brsh	S8

			mov		r16, RealA_L
			mov		r17, RealA_H

			subi	r16, 0x06
			sbci	r17, 0x03

			ldi		r18, 20

			rcall	mpy16u
			mov		COUNTB, r20

			rjmp	S_END

S7:
			clr		COUNTB
			rjmp	S_END

S8:
			ser		COUNTB

S_END:
			out		OCR0A, COUNTA
			out		OCR0B, COUNTB
			
			rjmp	MAIN

; ��������� ���������� ������������ ������� 0
TIM0_OVF:
			push	r16
			in		r16, SREG
			push	r16

			dec		COUNT
			brne	TIM0_OVF_END

			mov		r16, FLAG
			sbr		r16, 0b00000001
			mov		FLAG, r16

			ldi		r16, 47
			mov		COUNT, r16

TIM0_OVF_END:
			pop		r16						
			out		SREG, r16
			pop		r16

			reti

; ������ 16 ��� ����� SPI. ������� ���� � Angle_H, ������� � Angle_L
R_SPI16:
			clr		r19					; ������� ��� �������� ��������
			cbi		PORTB, CS			; ����� AS5048, ������ �������

			rcall	R_SPI				; ��������� ������� ����
			andi	r17, 0b00111111
			mov		Angle_H, r17		; ��������� ��� � Angle_H
			rcall	R_SPI				; ��������� ������� ����
			mov		Angle_L, r17		; ��������� ��� � Angle_L
			sbi		PORTB, CS			; ������� ������� ������� �� CS

			sbrs	r19, 0				; �������� ���� ��������
			rjmp	R_SPI16_OK

			ldi		r16, (1<<SM1)|(1<<SE)
			out		MCUCR, r16			; ���� ������ �� AS5048A � �������
			sleep						; �������� �� ������� (��. ��������� WDT)

R_SPI16_OK:
			ret

; ������ ������ ����� ����� SPI � r17
R_SPI:
			ldi		r16, 8				; ������� ���
			clr		r17

spi_loop:
			sbi		PORTB, SCK			; ������ �����
			lsl		r17					; ����� ����� � �������� �������
			cbi		PORTB, SCK			; ������ �����
			
			sbic	PINB, MISO			; ������ ��� � miso
			sbr		r17, 0b00000001

			sbrc	r17, 0
			inc		r19

			dec		r16
			brne	spi_loop
			
			ret

; ������ � EEPROM
EEPROM_write:
			push	r19
			push	r18
			mov		r18, r19
			rcall	EEPROM_read
			cp		r18, r19
			breq	EEPROM_write_END

EEPROM_write_loop:
			sbic	EECR,EEPE			; ���� ���������� ������ � ������. �������� � �����
			rjmp	EEPROM_write_loop	; �� ��� ��� ���� �� ��������� ���� EEWE
			cli							; ����� ��������� ����������.

			out		EEARL, R17			; ��������� ����� ������ ������
			out		EEDR, R18			; � ���� ������, ������� ��� ����� ���������
			sbi		EECR, EEMPE			; ������� ��������������
			sbi		EECR, EEPE			; ���������� ����
			sei							; ��������� ����������

EEPROM_write_END:
			pop		r18
			pop		r19

			ret

; ������ �� EEPROM
EEPROM_read:
			sbic	EECR, EEPE			; Wait for completion of previous write
			rjmp	EEPROM_read
			out		EEARL, r17			; Set up address (r17) in address register
			sbi		EECR, EERE			; Start eeprom read by writing EERE
			in		r19, EEDR			; Read data from data register

			ret

;***************************************************************************
;*
;* "mpy16u" - 16x16 Bit Unsigned Multiplication
;*
;* This subroutine multiplies the two 16-bit register variables 
;* mp16uH:mp16uL and mc16uH:mc16uL.
;* The result is placed in m16u3:m16u2:m16u1:m16u0.
;*  
;* Number of words	:14 + return
;* Number of cycles	:153 + return
;* Low registers used	:None
;* High registers used  :7 (mp16uL,mp16uH,mc16uL/m16u0,mc16uH/m16u1,m16u2,
;*                          m16u3,mcnt16u)	
;*
;***************************************************************************

;***** Subroutine Register Variables

/*
r16		;multiplicand low byte
r17		;multiplicand high byte
r18		;multiplier low byte

r19		;result byte 0 (LSB)
r20		;result byte 1
r21		;result byte 2

r22		;loop counter
*/
;***** Code

mpy16u:
			clr	r21		;clear 2 highest bytes of result
			clr	r20
			ldi	r22,8	;init loop counter
m16u_1:	
			ror	r18

			brcc	noad8		;if bit 0 of multiplier set
			add	r20,r16	;add multiplicand Low to byte 2 of res
			adc	r21,r17	;add multiplicand high to byte 3 of res
noad8:
			ror	r21		;rotate right result byte 2
			ror	r20		;rotate result byte 1 and multiplier High
			ror	r19		;rotate result byte 0 and multiplier Low
			dec	r22		;decrement loop counter
			brne	m16u_1		;if not done, loop more
			
			ret

; ����������. ������
; ���� ������ ������ - �������� � EEPROM ��������� �������
ADJ0:
			cli							; ����� ������ ������ EEPROM
			mov		r16, FLAG
			cbr		r16, 0b00011110
			mov		FLAG, r16
			sei

			mov		Zero_L, Angle_L
			mov		Zero_H, Angle_H

; ��������� ����������� �����
			rcall	CCS
			
			mov		CHECK, r19

			rcall	SAVE_ZERO
ADJ0_END:
			ret
; ����������. �����.


; ������� ������������ �����
CCS:
			clr		r19				; ���� ����� ������� ��������. ������� ����
			ldi		r17, 8			; ��������� ������ �����
			mov		r16, Zero_L		; � ���� ����� ����� ������� ��������
 
LL1:
 			sbrc	r16, 0			; ���� ��� ���� �� ����� ���� 
			inc		r19				; �������� ������� ��������
			lsr		r16				; ��������� � ���������� ����

			dec		r17				; �������� �������
			brne	LL1				; ���� ���� ���� �������� - �����.


			ldi		r17, 8			; ��������� ������ �����
			mov		r16, Zero_H		; � ���� ����� ����� ������� ��������
 
LL2:
 			sbrc	r16, 0
			inc		r19
			lsr		r16

			dec		r17				; �������� �������
			brne	LL2				; ���� ���� ���� �������� - �����.

			ldi		r18, 0x0F			; ���� ����� ������� ������.
			sub		r18, r19
			andi	r18, 0x0F
			andi	r19, 0x0F
			swap	r18
			or		r19, r18		; ����������� ����� � �������� r19

			ret

; ���������� �������� ��������� � EEPROM
SAVE_ZERO:
			mov		r19, Zero_L
			ldi		r17, low(ZERO0L)
			rcall	EEPROM_write
			ldi		r17, low(ZERO1L)
			rcall	EEPROM_write
			ldi		r17, low(ZERO2L)
			rcall	EEPROM_write

			
			mov		r19, Zero_H
			ldi		r17, low(ZERO0H)
			rcall	EEPROM_write
			ldi		r17, low(ZERO1H)
			rcall	EEPROM_write
			ldi		r17, low(ZERO2H)
			rcall	EEPROM_write

			mov		r19, CHECK
			ldi		r17, low(CS0)
			rcall	EEPROM_write
			ldi		r17, low(CS1)
			rcall	EEPROM_write
			ldi		r17, low(CS2)
			rcall	EEPROM_write

			ret