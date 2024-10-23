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
#include "ssd.h" // Digilent Library for using the on-board SSD
#include "led.h"

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

#define RGSIN_SIZE  (sizeof(rgSinSamples) / sizeof(rgSinSamples[0]))
#define TMR_FREQ_SINE   48000 // 48 kHz

typedef enum _KEY {K0, K1, K2, K3, K4, K5, K6, K7, K8, K9, K_A, K_B, K_C, K_D, K_E, K_F, K_NONE} eKey ;
typedef enum _MODE {MODE1,MODE2,MODE3,MODE4,MODE5,MODE6} eModes ;
int seconds = 0;
int digit1 = -1;
int digit2 = -1;
int digit3 = -1;
int digit4 = -1;
int ssd_index = 1;
int JINGLE = 0;

int cntAudioBuf, idxAudioBuf;

unsigned short *pAudioSamples;

unsigned short rgSinSamples [] = {200, 200, 200, 200, 200, 200, 200,
                                  200, 200, 200, 200, 200, 200, 200,
                                  200, 200, 200, 200, 200, 0, 0,
                                  0, 0, 0};

eModes mode = MODE4;

char new_press = FALSE;

// subrountines
void CNConfig();
void TIMERConfig();
void JINGLEConfig();
void handle_new_keypad_press(eKey key) ;
void mode1();
void mode2();
void mode4();
void mode1_input(eKey key);
void mode2_input(eKey key);
void mode4_input(eKey key);
void TurnOnJingle(void);
void TurnOffJingle(void);

void initialize_ports();

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
    
    //Initialize Visual Outputs
    SSD_Init();
    LED_Init();
    
    // You need to enable all the rows
    R1 = R2 = R3 = R4 = 0;
    
    LCD_WriteStringAtPos("Group 8",0,4);
    //LCD_WriteStringAtPos("    Mode 1!     ",1,0);
    LCD_WriteStringAtPos("Mode 4",1,4);
    
    CNConfig();
    TIMERConfig();
    JINGLEConfig();
    
    SSD_WriteDigits(digit4, digit3, digit2, digit1, 0, 0, 0, 0);

    /* Other initialization and configuration code */
    while (TRUE) {
        
    }
} 


void CNConfig() {
    /* Make sure vector interrupts is disabled prior to configuration */
    macro_disable_interrupts;
    
    CNCONDbits.ON = 1;   //all port D pins to trigger CN interrupts
    CNEND = 0x0F00;      	//configure PORTD pins 8-11 as CN pins
    CNPUD = 0x0F00;      	//enable pullups on PORTD pins 8-11

    IPC8bits.CNIP = 101;  	// set CN priority to  5
    IPC8bits.CNIS = 11;  	// set CN sub-priority to 3

    IFS1bits.CNDIF = 0;   	//Clear interrupt flag status bit
    IEC1bits.CNDIE = 1;   	//Enable CN interrupt on port D
    
    int j = PORTD;             //read port to clear mismatch on CN pins
    macro_enable_interrupts();	// re-enable interrupts
}

void TIMERConfig() {
    macro_disable_interrupts;
    T2CONbits.ON = 0;
    T2CONbits.TCKPS = 0b111;
    T2CONbits.TCS = 0;
    TMR2 = 0;
    PR2 = (10000000 / 256);
    IPC2bits.T2IP = 4;
    IPC2bits.T2IS = 0;
    IFS0bits.T2IF = 0;
    IEC0bits.T2IE = 1;
    T2CONbits.ON = 1;
    macro_enable_interrupts();
}

void __ISR(_TIMER_2_VECTOR) TM_HANDLER(void) {
    LATA = (0xFF00 | 0xFF >> (seconds%9));
    seconds++;
    TurnOffJingle();
    IFS0bits.T2IF = 0;
    macro_enable_interrupts();
}

void __ISR(_TIMER_3_VECTOR, IPL7AUTO) Timer3ISR(void) 
{  
    // play sine
    // load sine value into OC register
    OC1RS = 2*pAudioSamples[(++idxAudioBuf) % cntAudioBuf];
    
    IFS0bits.T3IF = 0;      // clear Timer3 interrupt flag
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
        if (C1 == 0) { key = K1; }      // first column
        else if (C2 == 0) { key = K2; } // second column
        else if (C3 == 0) { key = K3; } // third column
        else if (C4 == 0) { key = K_A; } // fourth column

        // check second row 
        R2 = 0; R1 = R3 = R4 = 1;
        if (C1 == 0) { key = K4; }
        else if (C2 == 0) { key = K5; }
        else if (C3 == 0) { key = K6; }
        else if (C4 == 0) { key = K_B; }

        // check third row 
        R3 = 0; R1 = R2 = R4 = 1;
        if (C1 == 0) { key = K7; }
        else if (C2 == 0) { key = K8; }
        else if (C3 == 0) { key = K9; }
        else if (C4 == 0) { key = K_C; }

        // check fourth row 
        R4 = 0; R1 = R3 = R2 = 1;
        if (C1 == 0) { key = K0; }
        else if (C2 == 0) { key = K_F; }
        else if (C3 == 0) { key = K_E; }
        else if (C4 == 0) { key = K_D; }

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
    case MODE4:
        mode4_input(key);
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

void mode4() {
    mode = MODE4;
    LCD_WriteStringAtPos("Mode 4",1,4);
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

void mode4_input(eKey key){
    if (key == K_C) {
        ssd_index = 1;
        digit1 = -1;
        digit2 = -1;
        digit3 = -1;
        digit4 = -1;
    }
    if (ssd_index == 1) { 
        switch(key){
            case K_D:
                digit1 = -1;
                break;
            case K0:
                digit1 = 0;
                ssd_index++;
                break;
            case K1:
                digit1 = 1;
                ssd_index++;
                break;
            case K2: 
                digit1 = 2;
                ssd_index++;
                break;
            case K3:
                digit1 = 3;
                ssd_index++;
                break;
            case K4:
                digit1 = 4;
                ssd_index++;
                break;
            case K5:
                digit1 = 5;
                ssd_index++;
                break;
            case K6:
                digit1 = 6;
                ssd_index++;
                break;
            case K7: 
                digit1 = 7;
                ssd_index++;
                break;
            case K8:
                digit1 = 8;
                ssd_index++;
                break;
            case K9:
                digit1 = 9;
                ssd_index++;
                break;
        }
        SSD_WriteDigits(digit4, digit3, digit2, digit1, 0, 0, 0, 0);
    }
    else if (ssd_index == 2){
        switch(key){
            case K_D:
                digit1 = -1;
                ssd_index--;
                break;
            case K0:
                digit2 = 0;
                ssd_index++;
                break;
            case K1:
                digit2 = 1;
                ssd_index++;
                break;
            case K2: 
                digit2 = 2;
                ssd_index++;
                break;
            case K3:
                digit2 = 3;
                ssd_index++;
                break;
            case K4:
                digit2 = 4;
                ssd_index++;
                break;
            case K5:
                digit2 = 5;
                ssd_index++;
                break;
            case K6:
                digit2 = 6;
                ssd_index++;
                break;
            case K7: 
                digit2 = 7;
                ssd_index++;
                break;
            case K8:
                digit2 = 8;
                ssd_index++;
                break;
            case K9:
                digit2 = 9;
                ssd_index++;
                break;
        }
        SSD_WriteDigits(digit4, digit3, digit2, digit1, 0, 0, 0, 0);
    }
    else if (ssd_index == 3){
        switch(key){
            case K_D:
                digit2 = -1;
                ssd_index--;
                break;
            case K0:
                digit3 = 0;
                ssd_index++;
                break;
            case K1:
                digit3 = 1;
                ssd_index++;
                break;
            case K2: 
                digit3 = 2;
                ssd_index++;
                break;
            case K3:
                digit3 = 3;
                ssd_index++;
                break;
            case K4:
                digit3 = 4;
                ssd_index++;
                break;
            case K5:
                digit3 = 5;
                ssd_index++;
                break;
            case K6:
                digit3 = 6;
                ssd_index++;
                break;
            case K7: 
                digit3 = 7;
                ssd_index++;
                break;
            case K8:
                digit3 = 8;
                ssd_index++;
                break;
            case K9:
                digit3 = 9;
                ssd_index++;
                break;
        }
        SSD_WriteDigits(digit4, digit3, digit2, digit1, 0, 0, 0, 0);
    }
    else if (ssd_index == 4){
        switch(key){
            case K_D:
                digit3 = -1;
                ssd_index--;
                break;
            case K0:
                digit4 = 0;
                ssd_index++;
                break;
            case K1:
                digit4 = 1;
                ssd_index++;
                break;
            case K2: 
                digit4 = 2;
                ssd_index++;
                break;
            case K3:
                digit4 = 3;
                ssd_index++;
                break;
            case K4:
                digit4 = 4;
                ssd_index++;
                break;
            case K5:
                digit4 = 5;
                ssd_index++;
                break;
            case K6:
                digit4 = 6;
                ssd_index++;
                break;
            case K7: 
                digit4 = 7;
                ssd_index++;
                break;
            case K8:
                digit4 = 8;
                ssd_index++;
                break;
            case K9:
                digit4 = 9;
                ssd_index++;
                break;
        }
        SSD_WriteDigits(digit4, digit3, digit2, digit1, 0, 0, 0, 0);
    }
    else if (ssd_index == 5) {
        if (key == K_D) {
            digit4 = -1;
            ssd_index--;
        }
        else if (key == K_E) {
            TurnOnJingle();
            ssd_index = 1;
            digit1 = -1;
            digit2 = -1;
            digit3 = -1;
            digit4 = -1;
        }
        SSD_WriteDigits(digit4, digit3, digit2, digit1, 0, 0, 0, 0);
    }  
    else {
        LCD_WriteStringAtPos("YOU'RE COOKED",1,0);
    }
}

void JINGLEConfig(void) {
    macro_disable_interrupts;
     // the following lines configure interrupts to control the speaker
    T3CONbits.ON = 0;       // turn off Timer3
    OC1CONbits.ON = 0;      // Turn off OC1
        /* The following code sets up the alarm timer and interrupts */
    tris_A_OUT = 0;    
    rp_A_OUT = 0x0C; // 1100 = OC1
        // disable analog (set pins as digital)
    ansel_A_OUT = 0;
    
    T3CONbits.TCKPS = 0;     //1:1 prescale value
    T3CONbits.TGATE = 0;     //not gated input (the default)
    T3CONbits.TCS = 0;       //PCBLK input (the default)
    
    OC1CONbits.ON = 0;       // Turn off OC1 while doing setup.
    OC1CONbits.OCM = 6;      // PWM mode on OC1; Fault pin is disabled
    OC1CONbits.OCTSEL = 1;   // Timer3 is the clock source for this Output Compare module
    
    IPC3bits.T3IP = 7;      // interrupt priority
    IPC3bits.T3IS = 3;      // interrupt subpriority
    macro_enable_interrupts();
}

void TurnOnJingle(void) {
    //set up alarm
    PR3 = (int)((float)((float)PB_FRQ/TMR_FREQ_SINE) + 0.5);               
    idxAudioBuf = 0;
    cntAudioBuf = RGSIN_SIZE;
    pAudioSamples = rgSinSamples;

    // load first value
    OC1RS = pAudioSamples[0];
    TMR3 = 0;

    T3CONbits.ON = 1;        //turn on Timer3
    OC1CONbits.ON = 1;       // Start the OC1 module  
    IEC0bits.T3IE = 1;      // enable Timer3 interrupt    
    IFS0bits.T3IF = 0;      // clear Timer3 interrupt flag
}

void TurnOffJingle(void) {
    T3CONbits.ON = 0;       // turn off Timer3
    OC1CONbits.ON = 0;      // Turn off OC1
}
