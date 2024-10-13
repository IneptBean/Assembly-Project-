	AREA DATA, CODE, READONLY 

RCGCGPIO EQU 0X400FE608 ;Address of GPIO Module clock/power, it ACTIVATES PORTS...
							; ...bit 0 = port A, bit 1 = port B, ... and bit 5 = port F	
; LEDs should be connected to PB0,PB1,PB2,PB3,PB4,PB5,PB6, and PB7 in a circle, clockwise from top being PB0.
; ( 0 up, 1 up/right, 2 right, 3 down/right, 4 down, 5 down/left, 6 left, 7 up/left)
;Port A had our UART connections.  Some elements are commented out and incomplete.  If you finished lab7, then you can use that data.
GPIOADATA_RW EQU 0X400043FC ; Address of Port A (DATA)of our microcontroller (buttons)
GPIOADIR 	 EQU 0x40004400 ; Address of Port A (DIRECTION SETTING)of our microcontroller
GPIOADEN	 EQU 0X4000451C	; Address of Port A (DIGITAL SETTING) of our microcontroller
GPIOAAFSEL	 EQU 0X40004420 ; Address of Port A (alternate function select) of our microcontroller port a
GPIOAPCTL	 EQU 0X4000452C; Which alternate functions are on Port A
; We are using all of port B for LED output
GPIOBDATA_RW EQU 0X400053FC ; Address of Port B (DATA)of our microcontroller (LEDs)
GPIOBDIR 	 EQU 0x40005400 ; Address of Port B (DIRECTION SETTING)of our microcontroller
GPIOBDEN 	 EQU 0X4000551C	; Address of Port B (DIGITAL SETTING) of our microcontroller
GPIOBAFSEL 	 EQU 0X40005420 ; Address of Port B (Alternate function SETTING) of our microcontroller
GPIOBPCTL 	 EQU 0X4000552C ; Address to configure which alternate function to use 
; Port E we will use for the ADC inputs
GPIOEDATA_RW EQU 0X400243FC ; Address of Port E (DATA)of our microcontroller (LEDs)
GPIOEDIR  	 EQU 0x40024400 ; Address of Port E (DIRECTION SETTING)of our microcontroller
GPIOEDEN 	 EQU 0X4002451C	; Address of Port E (DIGITAL SETTING) of our microcontroller

;PORT F - TIVABOARD USES PORT F, PINS PF1,PF2, AND PF3 FOR THE ONBOARD RGB LED
GPIOFDATA_RW EQU 0X400253FC ; ADDRESS OF PORT F PINS - DATA REGISTER
GPIOFDIR	 EQU 0X40025400 ; ADDRESS OF PORT F - DIRECTION
GPIOFDEN	 EQU 0X4002551C ; ADDRESS OF PORT F PINS - DIGITAL ENABLE

; Y-Axis Joystick should be connected to PE0 and X-Axis to PE1

;ADC General Settings
RCGCADC 	 EQU 0x400FE638 	 ;******Step 2A****** Power control for ADCs 
			; ...Enables ADC Module, PROVIDES A CLOCK  Bit 0 = MODULE 0 Bit 1 = MODULE 1		

;ADC analog mode select Settings for Port E 
GPIOEAMSEL 	 EQU 0x40024000	 ;*****Step 2C******* Address of Port E (ANALOG SETTING) of our microcontroller
GPIOEAFSEL	 EQU 0x40024420; Not needed for purely analog port E

;UART ACCESS
RCGCUART	EQU 0X400FE618; ***2A1- ENABLE UART MODULE USING THIS REGISTER (344)
	
;UART general data
UART0		EQU 0x4000C000 ; Base address of all UART0 functions
UART0DR		EQU UART0+0x000 ;***2B address of the UART0 data register
UART0FR		EQU UART0+0x018 ;***2B UART0 flag register 
UART0IBRD	EQU UART0+0x024;***2B UART0 integer baud-rate divider register
UART0FBRD	EQU UART0+0x028 ;***2B UART0 fractional baud-rate divider register
UART0LCRH	EQU UART0+0x02C ;***2B UART0 Line Control register
UART0CTL	EQU UART0+0x030 ;***2B UART0 control register
UART0CC		EQU UART0+0xFC8 ;***2B UART0 Clock Configuration register

							; ...Enables ADC Module, PROVIDES A CLOCK  Bit 0 = MODULE 0 Bit 1 = MODULE 1		
ADCPC 		 EQU 0X40038FC4 ; Select ADC Speed 1-125KSPS, 3-250KSPS, 5-500KSPS, 7 - 1MSPS
ADCACTSS 	 EQU 0X40038000 ; Address to enable and disable the ADC Sequencer (Seq 0 = Bit 0) ;;(BIT 0-3 SEQ 0 -3) BIT 16 - (0 IDLE) (1 BUSY) 
ADCEMUX 	 EQU 0X40038014 ; Select which event triggers the sample sequencer (Seq 0 bits(0- 3) 0XF = ALWAYS)
ADCSSMUX0 	 EQU 0X40038040 ; Select which ADC channels will the sequencer 0 read (AIN3 = PE0, AIN2 = PE1)
ADCSSCTL0 	 EQU 0X40038044	; Address to configure the sample control bits (interruption and end of sequencer) 
ADCRIS 		 EQU 0X40038004	; Registers with the flags for "done" bits 
ADCSSFIFO0 	 EQU 0X40038048 ; DATA (BITS 0-11)
ADCISC 		 EQU 0X4003800C ; Register that clears the ADCRIS flags
ADCPSSI		 EQU 0X40038028 ; ADC Sample Sequencer Processor-Initiate (bit 0 is Sequencer 0)
ADCSAC		 EQU 0X40038030 ; ADC Sample Averaging Control-- 2^ADCSAC samples are averaged for result (ADCSAC<7)
	
;PWM General Settings
RCGCPWM 		EQU 0X400FE640 ; ENABLE PWM MODULE PROVIDE A CLOCK  BIT O MODULE 0 BIT 1 MODULE 1
SYSCTL_RCC 		EQU 0X400FE060 ; Microcontroller Clock Control Configuration 
PWM0CTL 		EQU 0X40028040 ;Disable/Enable PWM Generator
PWMENABLE 		EQU 0X40028008 ;Disable/Enable PWM Outputs
PWM0CMPB 		EQU 0X4002805C ;Value at which Output will change (Duty Cycle) 
PWM0GENB  		EQU 0X40028064 ;Control bits for PWM0, Configuration 
PWM0LOAD 		EQU 0X40028050 ;Period, cycles needed to count

;constants 
THRESH_LOW 	EQU 0x0FE		 ; lower threshold for axial ADC value
THRESH_HIGH 	EQU 0xDF0		 ; higher threshold for axial ADC value
COUNT1 			EQU 0x00D00000
MIDDLE_C_DIV	EQU 0x1754 ; 261.626HZ
MIDDLE_D_DIV	EQU 0x14C9 ; 293.665HZ
MIDDLE_E_DIV	EQU 0x1284 ; 329.628HZ
MIDDLE_F_DIV	EQU 0x117A ; 349.228HZ
MIDDLE_G_DIV	EQU 0xF92 ; 391.995HZ
MIDDLE_A_DIV	EQU 0xDDF ; 440.000HZ
MIDDLE_B_DIV	EQU 0xC5C ; 493.883HZ
	
MAX_CMP			EQU 0X700
; values representing bits to set for activating positional LED's
LED_UP	 	EQU 1<<0
LED_RIGHT 	EQU 1<<2
LED_DOWN 	EQU 1<<4
LED_LEFT 	EQU 1<<6
	
; macros
	MACRO
	TRANSMIT8BITS $BITS_TO_TRANSMIT
	MOV R0,#$BITS_TO_TRANSMIT
	BL _TRANSMIT
	MEND
	
	MACRO 
	WRITEBITS $addr, $data
	LDR R0, =$addr
	MOV R1, #$data
	STR R1, [R0]
	MEND

	MACRO
	SETBITS $ADDRESS, $BITS
	LDR R1,=$BITS
	LDR R0,=$ADDRESS
	BL _SETBITS
	MEND

	MACRO
	CLEARBITS $ADDRESS, $BITS
	LDR R1,=$BITS
	LDR R0,=$ADDRESS
	BL _CLEARBITS
	MEND

	MACRO 
	DLAY $DELAYLOOPCOUNT
	LDR R0,=$DELAYLOOPCOUNT
	BL _DELAY
	MEND

	END