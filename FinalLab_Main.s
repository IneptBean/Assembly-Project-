; CS 100 Lab FINAL
; Due Date:
; Student Name:	Sabine Aliev
; Section: 

; Title: "Lab9_Main"
; © 2021 DigiPen, All Rights Reserved.
	
	GET FinalLab_Data.s             		; Get/include the data file
	GLOBAL __main               		; Global main function
	AREA Lab9_Main, CODE, READONLY		; Area of code that is read only
	ALIGN 2                     		; Align the data boundary to a multiple of 2
	ENTRY                       		; Entry into the code segment

;======Bit Methods-Nothing-new-here===================;
;---------------------------------  
; _SETBITS: Sets bits in memory (passing "1001" in R1 will set bits at position 0 and 3 in R0)
_SETBITS ; Turn on bits at address R0 specified by 1's in R1 
    PUSH {R4-R11, LR}
	LDR R4, [R0]
	ORR R4, R1
	STR R4, [R0]
    POP {R4-R11, LR}
  BX LR

;--------------------------------- 
; _CLEARBITS: Clears bits in memory (passing "1001" in R1 will clear bits at position 0 and 3 in R0)
_CLEARBITS ; Turn off bits at address R0 specified by 1's in R1
    PUSH {R4-R11, LR}
	LDR R4, [R0]
	MVN R3, R1
	AND R4, R3
	STR R4, [R0]
    POP {R4-R11, LR}
  BX LR	
;----------------------------------
_DELAY ; Loop R0 times
    PUSH {R4-R11, LR}
	MOV R10, R0
LOOPDELAY
	SUBS R10, R10, #1
	CMP R10, #0 
	BNE LOOPDELAY
    POP {R4-R11, LR}
  BX LR

;====================ADC Subroutines=======================;
;---------------------------------  
; Loops until ADC value is ready to be read
; Inputs: R0 = address of value to read
; Outputs: None
_wait_adc
	PUSH {LR, R4-R11}
	
    	
_wait_for_adc_loop; don't keep pushing registers.
; STUDENT CODE HERE ****SUBROUTINE STEP A****
    LDR R4, [R0]
	TST R4, #1
	BEQ _wait_for_adc_loop

; END STUDENT CODE
	POP {LR, R4-R11}
	BX LR
	
	
;---------------------------------  
; Reads the last 3 nibbles of the register.
; Inputs: R0 = address of value to read
; Outputs: Masked result of ADC in R0
_read_adc
	PUSH {LR, R4-R11}
; STUDENT CODE HERE ****SUBROUTINE STEP B****
    LDR R4, [R0]
	LDR R5, =0xFFF
	AND R4, R5
	MOV R0, R4

	
; END STUDENT CODE
	POP {LR, R4-R11}	
	BX LR


;====================SETUP=======================;
;---------------------------------  
; Subroutine: ports_activation 
; Description: Initializes output Ports so they are set up for use. If we don't
;   do this, the pin won't work. 

_led_pins_activation 
    PUSH { R4-R11, LR } ; stack preserved registers and link register

      SETBITS RCGCGPIO, 2_10011 ; A(1), B(2), C(4), D(8), E(10), F(20), we're only turning on ports A+B+E
      SETBITS GPIOBDEN, 2_11111111	;Configure used pins of Port B as digital
        
    POP { R4-R11, LR } ; Restore the link register and R4-R11 in case we changed them here
  BX LR;Return back to the calling subroutine.

output_input_pins_config 
	
  PUSH { R4-R11, LR } 					; push registers and link register to save them for what follows-
	SETBITS RCGCGPIO, 0X33    			; Read in the current GPIO Module configuration and Enable Ports A, B, E AND F
    SETBITS GPIOADEN, 0XF3				; Configure used pins of Port A as digital (PA0,1,4-7)
	SETBITS GPIOBDEN, 0xC0 				; PB7, pb6 (2_11000000)	;Configure used pins of Port B as digital
	SETBITS GPIOFDEN, 0x0E 				; Configure pins 1PF1, PF2 and PF3 for use (TIVA LAUNCHPAD RGB LED)
	SETBITS GPIOADIR, 0X00 				; PA7, PA6, PA5, PA4 (2_11110000) as output
	SETBITS GPIOBDIR, 0xC0 				; Enable PB7, pb6 (2_11000000) for output
	SETBITS GPIOFDIR, 0x0E 				; Configure PF1, PF2 and PF3 for OUTPUT (TIVA LAUNCHPAD RGB LED)
	CLEARBITS GPIOFDATA_RW, 0x0E 		; Initialize pins PF1, PF2 and PF3 as off 	
    
  POP { R4-R11, LR } 					; Pop back contents from the stack onto the registers they came from
  BX LR									; Return back to the calling subroutine.


;---------------------------------  
; Subroutine: led_initialization
; Description: Initializes our LEDs so they are set up for output. If we don't
;   do this, we will not be able to correctly turn them on or off later.

_led_initialization
    PUSH { R4-R11, LR } ; stack preserved registers and link register
      SETBITS GPIOBDIR, 2_11111111 ; MAKE SURE ALL PORTB PINS (PB0-PB7) ARE SET UP FOR OUTPUT
      CLEARBITS GPIOBDATA_RW, 2_00000000 ; MAKE SURE ALL PORTB PINS (PB0-PB7) START WITH 0 ON THE OUTPUT	
    POP { R4-R11, LR }; restore the preserved registers and link register
  BX LR; Return back to the calling subroutine.
  

SERIAL_INITIALIZATION
  PUSH { R4-R11, LR } 					; push preseverd registers and link register onto the stack to save them
	SETBITS GPIOADEN, 0X3				; Enable PA0 and PA1 as digital ports (not analog) - already done, but demonstrating how
	SETBITS RCGCUART, 0X1									; 3A- enable the UART module 0 (UART0) using RCGCUART (pp 344)		
	SETBITS	RCGCGPIO, 0X1								; 3B- enable clock to GPIO module through RCGCGPIO (pp340/1351)
	SETBITS	GPIOAAFSEL, 0X3							; 3C- Set GPIO Alternate function select GPIOAFSEL (671/1344)
										; No need to configure GPIO drive control or slew rate (Defaults to 2-Ma drive, which is fine)
										; No need to configure PMCn fields in GPIOPCTL (Defualts to PA0/PA1, which is fine)
SERAL_CONFIGURATION
; EXAMPLE SPECIFIC TO 9600 BAUD/8BIT/1 STOP/NO PARITY/FIFO OFF/NO INTERRUPTS
	CLEARBITS UART0CTL, 0X1				; 5A- DISABLE UART WHILE OPERATING-- CLEAR UARTEN BIT (O) IN UARTCTL	
										; NOTE** PLL IS SET TO 3, SO WE'RE WORKING WITH 48MHz.  *
										; SET BAUD-RATE-DIVISOR FOR BRD=48,000,000/(CLKDiv-16 or 8)(9600)=III.FFFFF  
	WRITEBITS UART0IBRD, 312									; 5B- (Set UART0IBRD=III)
	WRITEBITS UART0FBRD, 32									; 5C- Set UART0FBRD = INT(0.FFFFF*64+0.5) - FROM 0 TO 64 for fraction
	WRITEBITS UART0LCRH, 0x00000060								; 5D- Select serial com. parameters in UARTLCRH (8 BITS, the rest should be default)
	WRITEBITS UART0CC,	0x0								; 5E- Configure UART Clock source in UARTCTL (DEFAULT=0=SYSTEM CLOCK+DIVISOR)
	SETBITS UART0CTL, 0x301								; 5F- Enable UART0 for receive, Enable UART0 for Transmit, Enable UART0 total
	POP { R4-R11, LR } 					; Pop back the preserved registers and link register to what they were when we started SERIAL_INITIALIZATION
  BX LR 								; Return back to the calling subroutine.  

;---------------------------------  
; Subroutine: adc_initialization
; Description: Initializes our ADC Module and Sequencer so they are set up for analog input. 
output_pins_config 
	
  PUSH { R4-R11, LR } ; stack preserved registers and link register

   ;STUDENT CODE STARTS HERE 
   SETBITS RCGCPWM, 2_1
   SETBITS RCGCGPIO, 2_10
   BL delay
   SETBITS GPIOBDEN, 1<<7
   SETBITS GPIOBDIR, 1<<7
   SETBITS GPIOBAFSEL, 1<<7 
   SETBITS GPIOBPCTL, 1<<30
   
   ;STUDENT CODE ENDS HERE 
   
    ; Return back to the calling subroutine.
    POP { R4-R11, LR }
    BX LR

_adc_initialization
    PUSH { R4-R11, LR } ; stack preserved registers and link register

;---ADC Module Initialization (PP 817): this is copy-pasted-edited-- see lecture------------
	SETBITS RCGCADC, 2_01 ;1.Enable the ADC clock - RCGCADC (PP 352).
;2.We did this in Step 3A -Enable RCGCGPIO register FOR PORT E(see page 340).
;3.GPIOAFSEL initialize as 0, and we're not using their digital Alternate function.
	CLEARBITS GPIOEDEN,2_11 ;4.Config AINx AS analog input-clear corresponding DEN bit in(GPIOEDEN) (PP682).
	SETBITS GPIOEAMSEL, 2_11 ;5. WRITE TO GPIOEAMSEL (687) ANALOG INPUTS TO BE ANALOG.
;6. SAMPLE SEQUENCER PRIORITY BEYOND SCOPE OF COURSE.

;CONFIGURE Sample Sequencer 0------------------------------------------------

	CLEARBITS ADCACTSS, 2_1 ;1. disable SAMPLE SEQUENCER-clear ASENn bit in ADCACTSS.
	CLEARBITS ADCEMUX, 0XF;2. SET SS0 TRIGGER IN ADCEMUX TO USE 'PROCESSOR' TRIGGERING.
;3. NOT using a PWM generator as the trigger source.
	WRITEBITS ADCSSMUX0, 0x00000132 ;SET BITS FOR EACH input source in the ADCSSMUXn register.
	WRITEBITS ADCSSCTL0, 0x00006000 ;5. SET ADCSSCTL0 SO THAT 2ND IN SEQUENCE ENDS SEQUENCE AND STARTS INTERRUPT.
;6. SKIP - NOT USING INTERRUPT6. If interrupts are to be used, set the corresponding MASK bit in the ADCIM register.
	WRITEBITS ADCPC, 1 ; set samples per second to 125,000. 

	SETBITS ADCACTSS, 2_1 ;7. Enable sample sequencer0 - setting the  ASEN0 bit in the ADCACTSS register.

			 
    POP { R4-R11, LR }
    BX LR; Return back to the calling subroutine.


_GET_Y_ASCII
	PUSH { R4-R11, LR }
	 AND R0, R6, 0xF00
	 LSR R0, #8
     ADD R0, 0x30
	 CMP R0, 0x3A
	 ADDGE R0, #7
     BL _TRANSMIT	
     AND R0, R6, 0xF0
     LSR R0, #4
	 ADD R0, 0x30
     CMP R0, 0x3A
	 ADDGE R0, #7
     BL _TRANSMIT	
     AND R0, R6, 0xF
	 ADD R0, 0x30
     CMP R0, 0x3A 
	ADDGE R0, #7
     BL _TRANSMIT	
	 POP { R4-R11, LR }
	 BX LR

_GET_Y2_ASCII
	PUSH { R4-R11, LR }
	 AND R0, R8, 0xF00
	 LSR R0, #8
     ADD R0, 0x30
	 CMP R0, 0x3A
	 ADDGE R0, #7
     BL _TRANSMIT	
     AND R0, R8, 0xF0
     LSR R0, #4
	 ADD R0, 0x30
     CMP R0, 0x3A
	 ADDGE R0, #7
     BL _TRANSMIT	
     AND R0, R8, 0xF
	 ADD R0, 0x30
     CMP R0, 0x3A 
	ADDGE R0, #7
     BL _TRANSMIT	
	 POP { R4-R11, LR }
	 BX LR
_GET_X_ASCII
	PUSH { R4-R11, LR }
	 AND R0, R5, 0xF00
	 LSR R0, #8
     ADD R0, 0x30
	 CMP R0, 0x3A
	 ADDGE R0, #7
     BL _TRANSMIT	 
	 AND R0, R5, 0xF0
     LSR R0, #4
	 ADD R0, 0x30
     CMP R0, 0x3A
	 ADDGE R0, #7
     BL _TRANSMIT	
     AND R0, R5, 0xF
	 ADD R0, 0x30
     CMP R0, 0x3A 
	 ADDGE R0, #7
     BL _TRANSMIT	
	 POP { R4-R11, LR }
	 BX LR
	 
_GET_X2_ASCII
	PUSH { R4-R11, LR }
	 AND R0, R7, 0xF00
	 LSR R0, #8
     ADD R0, 0x30
	 CMP R0, 0x3A
	 ADDGE R0, #7
     BL _TRANSMIT	 
	 AND R0, R7, 0xF0
     LSR R0, #4
	 ADD R0, 0x30
     CMP R0, 0x3A
	 ADDGE R0, #7
     BL _TRANSMIT	
     AND R0, R7, 0xF
	 ADD R0, 0x30
     CMP R0, 0x3A 
	 ADDGE R0, #7
     BL _TRANSMIT	
	 POP { R4-R11, LR }
	 BX LR	 
	 


_TURN_ON_UP_LED ; ****STEP 6B**** CREATE A SUBROUTINE HERE THAT TURNS ON LED AND RETURNS
     PUSH { R4-R11, LR }
	 WRITEBITS PWM0LOAD, MIDDLE_A_DIV
	 POP { R4-R11, LR }
	 BX LR
	 
_TURN_OFF_LED 
     PUSH { R4-R11, LR }
	 WRITEBITS PWM0LOAD, MIDDLE_C_DIV
	 POP { R4-R11, LR }
	 BX LR
	 
_TURN_ON_DOWN_LED
	PUSH { R4-R11, LR }
	
	WRITEBITS PWM0LOAD, MIDDLE_B_DIV
	POP { R4-R11, LR }
	BX LR
	
_TURN_ON_RIGHT_LED
	PUSH { R4-R11, LR }
	
	WRITEBITS PWM0LOAD, MIDDLE_D_DIV
	POP { R4-R11, LR }
	BX LR


_TURN_ON_LEFT_LED
	PUSH { R4-R11, LR }
	
	WRITEBITS PWM0LOAD, MIDDLE_E_DIV
	POP { R4-R11, LR }
	BX LR

	
_TRANSMIT
  PUSH { R4-R11, LR } 					; stack preserved registers and link register
_WAIT_FOR_CLEAR_OUTPUT_FIFO
	LDR R1, =UART0FR									; 6A1- Load the address of the UART0 Flag register
	LDR R2, [R1]									; 6A2- Get the contents of the UART0 Flag register into a register we're not using
	TST R2, #1<<5									; 6B- Check the Transmit FIFO0 Full bit (TXFF) on that register with a TST (single bit ANDS)
	BNE _WAIT_FOR_CLEAR_OUTPUT_FIFO									; 6B1- If the Transmit FIFO0 IS full, go back to _WAIT_FOR_CLEAR_OUTPUT_FIFO
	AND R0, #2_11111111									; 6C- Mask out all but the lowest 8 bits for sending from R0
	LDR R4, =UART0DR									; 6D1- Place the data in R0 into the UART0Data Register (UART0DR)
	STR R0, [R4]									; 6D2- (two lines)
  POP { R4-R11, LR } 					; Pop back the preserved registers and link register
  BX LR 								; Return back to the calling subroutine.
  
_RECEIVE
  PUSH { R4-R11, LR } 					; stack preserved registers and link register	
_WAIT_FOR_CLEAR_OUTPUT_FIFORX
	LDR R4, =UART0FR									; 6A1- Load the address of the UART0 Flag register
	LDR R4, [R4]									; 6A2- Get the contents of the UART0 Flag register into a register we're not using
	TST R4, #1<<5									; 6B- Check the Transmit FIFO0 Full bit (TXFF) on that register with a TST (single bit ANDS)
	BNE _WAIT_FOR_CLEAR_OUTPUT_FIFORX									; 6B1- If the Transmit FIFO0 IS full, go back to _WAIT_FOR_CLEAR_OUTPUT_FIFO
	LDR R4, =UART0DR									; 6D1- Place the data in R0 into the UART0Data Register (UART0DR)
	LDR R0,[R4]									; 6D2- (two lines)
  POP { R4-R11, LR } 					; Pop back the preserved registers and link register
  BX LR 								; Return back to the calling subroutine.
  
delay 
	MOV R10, #COUNT1
L
	SUBS R10, R10, #1
	CMP R10, #0 
	BNE L
	BX LR

red_button
	 PUSH {R4-R11, LR}
	LDR R4, [R0]
	TRANSMIT8BITS 67
	ORR R4, R1
	
	STR R4, [R0]
	
    POP {R4-R11, LR}
  BX LR
  
green_button
     PUSH {R4-R11, LR}
	LDR R4, [R0]
	TRANSMIT8BITS 84
	ORR R4, R1
	
	STR R4, [R0]
	
    POP {R4-R11, LR}
  BX LR

blue_button
     PUSH {R4-R11, LR}
	LDR R4, [R0]
	TRANSMIT8BITS 81
	ORR R4, R1
	
	STR R4, [R0]
	
    POP {R4-R11, LR}
  BX LR
  
yellow_button
     PUSH {R4-R11, LR}
	LDR R4, [R0]
	TRANSMIT8BITS 78
	ORR R4, R1
	
	STR R4, [R0]
	
    POP {R4-R11, LR}
  BX LR
  LTORG

pwm_setup 
	
  PUSH { R4-R11, LR } ; stack preserved registers and link register


    ;STUDENT CODE STARTS HERE 
   SETBITS SYSCTL_RCC, 2_110000000000000000000
   CLEARBITS SYSCTL_RCC, 2_1100000000000000000
   WRITEBITS PWM0CTL, 0X000
   WRITEBITS PWM0GENB, 0x80C
   WRITEBITS PWM0LOAD, MIDDLE_C_DIV
   WRITEBITS PWM0CMPB, 0x50
   SETBITS PWM0CTL, 2_1
   SETBITS PWMENABLE, 2_10
   
   ;STUDENT CODE ENDS HERE 

	; Return back to the calling subroutine.
    POP { R4-R11, LR }
    BX LR	
	LTORG
;=======================LOGIC==========================;

__main

_INITIALIZATION_ROUTINES
	BL output_input_pins_config 
	BL SERIAL_INITIALIZATION
	BL output_pins_config 

	; Activate PORTS (B, E) and corresponding pins 
	BL _led_pins_activation 

; Initialize our LEDs so we can turn them on/off at will.
   	BL _led_initialization

; Turns on two lights if setbits correctly implemented
	LDR R0, =GPIOBDATA_RW
	MOV R1, #LED_RIGHT
	ADD R1, #LED_DOWN
	BL _SETBITS
	
	NOP ; BREAKPOINT 1
	
	BL _CLEARBITS
	
	NOP ; BREAKPOINT 2

    ; Initialize our ADC Module so we can read from PE0 and PE1 
    BL _adc_initialization
	


	LDR R11, =THRESH_LOW
	LDR R10, =THRESH_HIGH

    BL pwm_setup
; 5. At this point we need to implement a loop which reads the data from each 
;    of the A/D data registers corresponding to the pins we have set up, and 
;    implement logic to decide which, if any, LEDs we should trigger in
;    response. This will occur within the update_loop label.

loop

; Clear interrupt flag so that we know we're reading a *new* adc conversion	
	SETBITS ADCISC, 0X01 ; STEP 5-1A
; Start processor trigger (tell sequencer 0 to start converting ADCs)
	SETBITS ADCPSSI, 1 ; STEP 5-1B
	

; Wait until reading is complete (use end flag) STEP 5-2
	LDR R0, =ADCRIS; _wait_adc EXPECTS TO HAVE THE ADC-Raw Interrupt Status address loaded into r0.
	BL _wait_adc	; and come back from there once you have ADC conversions for us.

; read axes 
	LDR R0, =ADCSSFIFO0
	BL _read_adc 
	MOV R5, R0  ;STEP 5-3 store the first read value-- x-value-- in R5.
	BL _GET_X_ASCII
	LDR R0, =ADCSSFIFO0
	BL _read_adc
	MOV R6, R0  ;STEP 5-4 store y-value
	BL _GET_Y_ASCII
	TRANSMIT8BITS 0x20
	LDR R0, =ADCSSFIFO0
	BL _read_adc 
	MOV R7, R0  ;STEP 5-3 store the first read value-- x-value-- in R5.
	BL _GET_X2_ASCII
	LDR R0, =ADCSSFIFO0
	BL _read_adc
	MOV R8, R0  ;STEP 5-4 store y-value
	BL _GET_Y2_ASCII
	TRANSMIT8BITS 0x20
	

	
	PUSH {R4-R11, LR}
	;================= Step A : Load value at the button address,
	; STUDENT BUTTON READ LOGIC GOES HERE
	LDR R4, =GPIOADATA_RW
	LDR R5, [R4]
	
	; END STUDENT CODE
	
	;=================Step B:Check for button press using ANDS or TST
	; Make the check update the xPSR's Z bit or not, and use this information in step D.
	; STUDENT BITWISE LOGIC GOES HERE
		TST R5, #1<<4

	; END STUDENT CODE

	;================= Step C: Load value at the LEDs address of Port B, and define the LEDs pins you will use
	; STUDENT LOAD PRIOR LED VALUES GOES HERE
	LDR R0, =GPIOBDATA_RW
	LDR R1, =2_11111111	

	; END STUDENT CODE
	
	;================= Step D: Branch to either set_bits or clear_bits subroutines depending on read results. 
	;		 Set the proper bit to turn on LED if button is pressed.
	;	   	 Clear the bit to turn off LED if button is not pressed.
	;   	 Store the modified value back into the LED address
	;	 	 to apply the change
	; STUDENT CONDITIONAL STORE LED_DATA LOGIC GOES HERE
	BLEQ _CLEARBITS
	BLNE yellow_button

	; END STUDENT CODE
	
	NOP 				;Breakpoint 
	POP {R4-R11, LR}
	
	
    PUSH {R4-R11, LR}
	;================= Step A : Load value at the button address,
	; STUDENT BUTTON READ LOGIC GOES HERE
	LDR R4, =GPIOADATA_RW
	LDR R5, [R4]
	

	
	; END STUDENT CODE
	
	;=================Step B:Check for button press using ANDS or TST
	; Make the check update the xPSR's Z bit or not, and use this information in step D.
	; STUDENT BITWISE LOGIC GOES HERE
		TST R5, #1<<5

	; END STUDENT CODE

	;================= Step C: Load value at the LEDs address of Port B, and define the LEDs pins you will use
	; STUDENT LOAD PRIOR LED VALUES GOES HERE
	LDR R0, =GPIOBDATA_RW
	LDR R1, =2_11111111	

	; END STUDENT CODE
	
	;================= Step D: Branch to either set_bits or clear_bits subroutines depending on read results. 
	;		 Set the proper bit to turn on LED if button is pressed.
	;	   	 Clear the bit to turn off LED if button is not pressed.
	;   	 Store the modified value back into the LED address
	;	 	 to apply the change
	; STUDENT CONDITIONAL STORE LED_DATA LOGIC GOES HERE
	BLEQ _CLEARBITS
	BLNE green_button

	; END STUDENT CODE
	
	NOP 				;Breakpoint 
	POP {R4-R11, LR}
	
    PUSH {R4-R11, LR}
	;================= Step A : Load value at the button address,
	; STUDENT BUTTON READ LOGIC GOES HERE
	LDR R4, =GPIOADATA_RW
	LDR R5, [R4]
	
	; END STUDENT CODE
	
	;=================Step B:Check for button press using ANDS or TST
	; Make the check update the xPSR's Z bit or not, and use this information in step D.
	; STUDENT BITWISE LOGIC GOES HERE
		TST R5, #1<<6

	; END STUDENT CODE

	;================= Step C: Load value at the LEDs address of Port B, and define the LEDs pins you will use
	; STUDENT LOAD PRIOR LED VALUES GOES HERE
	LDR R0, =GPIOBDATA_RW
	LDR R1, =2_11111111	

	; END STUDENT CODE
	
	;================= Step D: Branch to either set_bits or clear_bits subroutines depending on read results. 
	;		 Set the proper bit to turn on LED if button is pressed.
	;	   	 Clear the bit to turn off LED if button is not pressed.
	;   	 Store the modified value back into the LED address
	;	 	 to apply the change
	; STUDENT CONDITIONAL STORE LED_DATA LOGIC GOES HERE
	BLEQ _CLEARBITS
	BLNE red_button

	; END STUDENT CODE
	
	NOP 				;Breakpoint 
	POP {R4-R11, LR}
	
	PUSH {R4-R11, LR}
	;================= Step A : Load value at the button address,
	; STUDENT BUTTON READ LOGIC GOES HERE
	LDR R4, =GPIOADATA_RW
	LDR R5, [R4]
	
	; END STUDENT CODE
	
	;=================Step B:Check for button press using ANDS or TST
	; Make the check update the xPSR's Z bit or not, and use this information in step D.
	; STUDENT BITWISE LOGIC GOES HERE
		TST R5, #1<<7

	; END STUDENT CODE

	;================= Step C: Load value at the LEDs address of Port B, and define the LEDs pins you will use
	; STUDENT LOAD PRIOR LED VALUES GOES HERE
	LDR R0, =GPIOBDATA_RW
	LDR R1, =2_11111111	

	; END STUDENT CODE
	
	;================= Step D: Branch to either set_bits or clear_bits subroutines depending on read results. 
	;		 Set the proper bit to turn on LED if button is pressed.
	;	   	 Clear the bit to turn off LED if button is not pressed.
	;   	 Store the modified value back into the LED address
	;	 	 to apply the change
	; STUDENT CONDITIONAL STORE LED_DATA LOGIC GOES HERE
	BLEQ _CLEARBITS
	BLNE blue_button

	; END STUDENT CODE
	
	NOP 				;Breakpoint 
	POP {R4-R11, LR}
	
	
	
	
; turn off all LEDS (GPIOxDATA_RW - ports for LEDs)

	CLEARBITS GPIOBDATA_RW, 0XFF ;STEP 5-5 TURN OFF ALL PB OUTPUTS.
; STUDENT CODE HERE 
;	NOP	;****STEP 5A**** REMEMBER: WITHOUT CODE, THERE IS NO PLACE FOR A BREAKPOINT
	CMP R8, THRESH_HIGH
    BLGT _TURN_ON_UP_LED
;    ;BLGT _GET_Y_ASCII	;****STEP 6A**** COMPARE R6 WITH HIGH THRESHOLD, BRANCH TO _TURN_ON_UP_LED IF HIGH,
	BLLT _TURN_OFF_LED	;****STEP 6C**** IF COMPARE ENDS WITH OTHER RESULTS, BRANCH TO TURN OFF THAT LED
;		;****STEP 6D...**** NOW DO THE SAME FOR DOWN, LEFT, AND RIGHT.
	CMP R8, THRESH_LOW
	BLLT _TURN_ON_DOWN_LED
;	;BLLT _GET_Y_ASCII
	BLGT _TURN_OFF_LED

	CMP R7, THRESH_HIGH
	BLGT _TURN_ON_RIGHT_LED
;	;BLGT _GET_X_ASCII
	BLLT _TURN_OFF_LED
;	
	CMP R7, THRESH_LOW
	BLLT _TURN_ON_LEFT_LED
;	;BLLT _GET_X_ASCII
	BLGT _TURN_OFF_LED


; END STUDENT CODE
    ;TRANSMIT8BITS 13
;	WRITEBITS PWM0LOAD, MIDDLE_C_DIV
;	BL delay
;	WRITEBITS PWM0LOAD, MIDDLE_E_DIV
;	BL delay
;	WRITEBITS PWM0LOAD, MIDDLE_D_DIV
;	
   
	BL delay
	B loop
	
	END
		
