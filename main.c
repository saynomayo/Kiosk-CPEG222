/*===================================CPEG222====================================
 * Program:      Project 3 template
 * Authors:     Raphael Daluz & Mekhai Waples
 * Date:        10/12/2024
 * This is a template that you can use to write your project 3 code, for mid-stage and final demo.
==============================================================================*/
/*-------------- Board system settings. PLEASE DO NOT MODIFY THIS PART ----------*/
#ifndef _SUPPRESS_PLIB_WARNING          //suppress the plib warning during compiling
#define _SUPPRESS_PLIB_WARNING
#endif
#pragma config FPLLIDIV = DIV_2         // PLL Input Divider (2x Divider)
#pragma config FPLLMUL = MUL_20         // PLL Multiplier (20x Multiplier)
#pragma config FPLLODIV = DIV_1         // System PLL Output Clock Divider (PLL Divide by 1)
#pragma config FNOSC = PRIPLL           // Oscillator Selection Bits (Primary Osc w/PLL (XT+,HS+,EC+PLL))
#pragma config FSOSCEN = OFF            // Secondary Oscillator Enable (Disabled)
#pragma config POSCMOD = XT             // Primary Oscillator Configuration (XT osc mode)
#pragma config FPBDIV = DIV_8           // Peripheral Clock Divisor (Pb_Clk is Sys_Clk/8)
/*----------------------------------------------------------------------------*/

#include <xc.h>   //Microchip XC processor header which links to the PIC32MX370512L header
#include <stdio.h>  // need this for sprintf
#include <sys/attribs.h>
#include "config.h" // Basys MX3 configuration header
#include "lcd.h"    // Digilent Library for using the on-board LCD
#include "acl.h"    // Digilent Library for using the on-board accelerometer

#define TRUE 1
#define FALSE 0

// below are keypad row and column definitions based on the assumption that JB will be used and columns are CN pins
// If you want to use JA or use rows as CN pins, modify this part
#define R4 LATCbits.LATC14
#define R3 LATDbits.LATD0
#define R2 LATDbits.LATD1
#define R1 LATCbits.LATC13
#define C4 PORTDbits.RD9
#define C3 PORTDbits.RD11
#define C2 PORTDbits.RD10
#define C1 PORTDbits.RD8

typedef enum _KEY {K0, K1, K2, K3, K4, K5, K6, K7, K8, K9, K_A, K_B, K_C, K_D, K_E, K_F, K_NONE} eKey ;
typedef enum _MODE {MODE1,MODE2} eModes ;

eModes mode = MODE1;

char new_press = FALSE;

// subrountines
void CNConfig();
void handle_new_keypad_press(eKey key) ;
void mode1();
void mode2();
void mode1_input(eKey key);
void mode2_input(eKey key);

int main(void) {

    /* Initialization of LED, LCD, SSD, etc */
    DDPCONbits.JTAGEN = 0; // Required to use Pin RA0 (connected to LED 0) as IO
    LCD_Init() ;
    ACL_Init();

    float rgACLGVals[3];
    ACL_ReadGValues(rgACLGVals);
    int seed = rgACLGVals[0] * 10000;
    srand((unsigned) seed);
    // below are keypad row and column configurations based on the assumption that JB will be used and columns are CN pins
    // If you want to use JA or use rows as CN pins, modify this part

    // keypad rows as outputs
    TRISDbits.TRISD0 = 0;
    TRISDbits.TRISD1 = 0;
    ANSELDbits.ANSD1 = 0;
    TRISCbits.TRISC14 = 0;
    TRISCbits.TRISC13 = 0;

    // keypad columns as inputs
    TRISDbits.TRISD8 = 1;
    TRISDbits.TRISD9 = 1;
    TRISDbits.TRISD10 = 1;
    TRISDbits.TRISD11 = 1;
    
    // You need to enable all the rows
    R1 = R2 = R3 = R4 = 0;
    
    LCD_WriteStringAtPos("   Project 3    ",0,0);
    LCD_WriteStringAtPos("    Mode 1!     ",1,0);
    
    CNConfig();

    /* Other initialization and configuration code */

    while (TRUE) 
    {
        //You can put key-pad indepenent mode transition here, such as countdown-driven mode trasition, monitoring of microphone or update of RGB.
    }
} 


void CNConfig() {
    /* Make sure vector interrupts is disabled prior to configuration */
    macro_disable_interrupts;
    
    /* Complete the following configuration of CN interrupts, then uncomment them
    CNCONDbits.ON = ____;   //all port D pins to trigger CN interrupts
    CNEND = _______;      	//configure PORTD pins 8-11 as CN pins
    CNPUD = _______;      	//enable pullups on PORTD pins 8-11

    IPC8bits.CNIP = _______;  	// set CN priority to  5
    IPC8bits.CNIS = _______;  	// set CN sub-priority to 3

    IFS1bits.CNDIF = ______;   	//Clear interrupt flag status bit
    IEC1bits.CNDIE = _____  ;   	//Enable CN interrupt on port D
    */
    
    int j = PORTD;             //read port to clear mismatch on CN pins
    macro_enable_interrupts();	// re-enable interrupts
}

// This ISR is for the change notice interrupt. The timer will use another interrupt.
// Try looking at ssd.c for reference when setting it up! You will also source code from Project 2's alarm
// and put it on another timer for this project.

void __ISR(_CHANGE_NOTICE_VECTOR) CN_Handler(void) {
    eKey key = K_NONE;
    
    // 1. Disable CN interrupts
    IEC1bits.CNDIE = 0;     

    // 2. Debounce keys for 10ms
    for (int i=0; i<1426; i++) {}

    // 3. Handle "button locking" logic

    unsigned char key_is_pressed = (!C1 || !C2 || !C3 || !C4);
    // If a key is already pressed, don't execute the rest second time to eliminate double pressing
    if (!key_is_pressed)
    {
        new_press = FALSE;
    }
    else if (!new_press)
    {
        new_press = TRUE;

        // 4. Decode which key was pressed
        
        // check first row 
        R1 = 0; R2 = R3 = R4 = 1;
        if (C1 == 0) { /* ... */ }      // first column
        else if (C2 == 0) { /* ... */ } // second column
        else if (C3 == 0) { /* ... */ } // third column
        else if (C4 == 0) { /* ... */ } // fourth column

        // check second row 
        R2 = 0; R1 = R3 = R4 = 1;
        if (C1 == 0) { /* ... */ }
        else if (C2 == 0) { /* ... */ }
        else if (C3 == 0) { /* ... */ }
        else if (C4 == 0) { /* ... */ }

        // check third row 
        // ....

        // check fourth row 
        // ....

        // re-enable all the rows for the next round
        R1 = R2 = R3 = R4 = 0;
    
    }
    
    // if any key has been pressed, update next state and outputs
    if (key != K_NONE) {
        handle_new_keypad_press(key) ;
    }
    
    
    int j = PORTD;              //read port to clear mismatch on CN pints
    
    // 5. Clear the interrupt flag
    IFS1bits.CNDIF = 0;     

    // 6. Reenable CN interrupts
    IEC1bits.CNDIE = 1; 
}



void handle_new_keypad_press(eKey key)
{
    switch (mode)
    {
    case MODE1:
        mode1_input(key);
    break;
    case MODE2:
        mode2_input(key);
    break;
    }
}

void mode1(){
    mode = MODE1;

    LCD_WriteStringAtPos("    Mode 1!     ",1,0);
}

void mode2(){
    mode = MODE2;

    LCD_WriteStringAtPos("    Mode 2!     ",1,0);
}

void mode1_input(eKey key){
    //Go to mode 2 if A key is pressed
    switch(key){
        case K_A:
            mode2();
        break;
    }
}

void mode2_input(eKey key){
    //Go to mdoe 1 if any number key is pressed

    switch(key){
        case K0: case K1: case K2: case K3: case K4: case K5: case K6: case K7: case K8: case K9:
            mode1();
        break;
    }
}
