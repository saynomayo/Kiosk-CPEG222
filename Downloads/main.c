/*===================================CPEG222====================================
 * Program:     Project 4 (Line Follower)
 * Authors:     Mekhai Waples & Raphael Daluz
 * Date:        11/13/24
 * This is a guide that you can use to write your project 4 code
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
#define SYS_FREQ (80000000L) // 80MHz system clock
#define _80Mhz_ (80000000L)
#define LOOPS_NEEDED_TO_DELAY_ONE_MS_AT_80MHz 1426
#define LOOPS_NEEDED_TO_DELAY_ONE_MS (LOOPS_NEEDED_TO_DELAY_ONE_MS_AT_80MHz * (SYS_FREQ / _80Mhz_))

#define TRUE 1
#define FALSE 0
#define SEVEN 7


// Libraries
#include <string.h>
#include <xc.h>   //Microchip XC processor header which links to the PIC32MX370512L header
#include <stdio.h>  // need this for sprintf
#include <sys/attribs.h>
#include "config.h" // Basys MX3 configuration header
#include "led.h"
#include "ssd.h"
#include "lcd.h"
#include "swt.h"

// Function Declarations
void initializePorts();
void pwmConfig();
void activateServo();


int main(void) {
    Initialize_Ports();
    PWM_Config();
    
    while (TRUE) {
        //PS.. It might be a good idea to put this function in a timer ISR later on.
        activateServo();
    }
}




// Initialize ports on board
void intializePorts() {
    LED_Init();
    LCD_Init();
    SSD_Init();
    SWT_Init();
}

void pwmConfig() {
    
    // configure Timer X (select the timer, replace X with desired timer number)
    /*
    PRX = ; //Period Register.
    TXCONbits.TCKPS = ; //Timer Prescaler 
    TXCONbits.TGATE = 0; // not gated input (the default)
    TXCONbits.TCS = 0; // PBCLK input (the default)
    TXCONbits.ON = 1;  //Turn on Timer
    TMRX = 0; // Set Timer X to 0
    
    IPCXbits.TXIP = ;  //    priority
    IPCXbits.TXIS = ;  //    subpriority
    IFS0bits.TXIF = 0; //    clear interrupt flag
    IEC0bits.TXIE = 1; //    enable interrupt
    
    
    // Configure Output Compare Module 4
    
    OC4CONbits.OCM = 6;      // PWM mode on OC4; Fault pin is disabled
    OC4CONbits.OCTSEL = ;   // Select the timer to use as a clock source
    OC4RS = ;//OC4RS is some fraction of the Period
    OC4R = OC4RS;
    OC4CONbits.ON = 1;       // Start the OC4 module
    
    //Do The same for OC5**************************
   
   
   TRISBbits.TRISB8 = ; //set servo 0 as output
   TRISAbits.TRISA15 = ; //set servo 1 as output
   ANSELBbits.ANSB8 = ; //set servo 0 as digital

   RPB8R = 0x0B; // connect Servo 0 to OC5
   RPA15R = 0x0B;// connect Servo 1 to OC4
    */

    //Set up additional timers here if necessary
}


//ISR's are here is you need them. Don't forget to set them up!
void __ISR(_TIMER_2_VECTOR) Timer2ISR(void) {
    IEC0bits.T2IE = 0; // disable interrupt
    
    IFS0bits.T2IF = 0; // clear interrupt flag
    IEC0bits.T2IE = 1; // enable interrupt
}

void __ISR(_TIMER_3_VECTOR) Timer3ISR(void) {
    IEC0bits.T3IE = 0; // disable interrupt
    
    IFS0bits.T3IF = 0; // clear interrupt flag
    IEC0bits.T3IE = 1; // enable interrupt
}

void activateServo(){
    if(SWT_GetValue(6)){
        //If Switch 6 is on, move servo...
        //Replace X with your Timer Number
        //OC4RS = (int) PRX / 25;
    }else{
        //If Switch 6 is off, stop moving servo...
        //Replace X with your Timer Number
        //OC4RS = (int) PRX / 13.8; 
    }
}
