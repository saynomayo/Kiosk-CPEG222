/*===================================CPEG222====================================
 * Program:      Project 3 Mid-Stage
 * Authors:     Raphael Daluz & Mekhai Waples
 * Date:        10/23/2024
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
#include <stdio.h>  // n=d this for sprintf
#include <sys/attribs.h>
#include "config.h" // Basys MX3 configuration header
#include "lcd.h"    // Digilent Library for using the on-board LCD
#include "acl.h"    // Digilent Library for using the on-board accelerometer
#include "ssd.h" // Digilent Library for using the on-board SSD
#include "led.h"
#include <string.h>

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

typedef enum _KEY {K0, K1, K2, K3, K4, K5, K6, K7, K8, K9, K_A, K_B, K_C, K_D, K_E, K_F, K_NONE} eKey ; //keypad keys

typedef enum _MODE {MAINMENU,ORDERING,ORDERCONFIRM,LOOKUP,ORDERINFO,ERROR} eModes ; //system modes

//struct declaration for Order
typedef struct {
            int pos; 
            int time;
            char status[16]; 
            char item[16]; 
            int code[4];
            } Order;

int seconds = 0;

//code typed and generated
int code[4] = {-1, -1, -1, -1};
int rCode[4];

int compareCodes(Order queue[], int lookupcode[]);

//lookup globals
char *LOOKUPITEM;
char *LOOKUPSTATUS;

//error msg
char *QUEUEFULL = "Queue is full";
char *INVALIDCODE = "Invalid Code";

//Queue stuff
int QUEUESIZE = 0;
Order queue[];
void queueOrder(Order newOrder, Order queue[], int QUEUESIZE);
Order createOrder(const char* item, int QUEUESIZE);
int PREPPING = 0;
int ORDERFOUND = 0;
int WAITINGPOSITION = -1;
int REPLACEINDEX;
int REPLACE=0;

//food info
int menuindex = 0;
char menu[3][16] = {"1-Burrito", "2-Taco", "3-Fajita"};
char items[3][16] = {"Burrito", "Taco", "Fajita"};

//variables for convention's sake
int ssd_index = 1;
int JINGLE = 0;

int cntAudioBuf, idxAudioBuf;

unsigned short *pAudioSamples;

unsigned short rgSinSamples [] = {200, 200, 200, 200, 200, 200, 200,
                                  200, 200, 200, 200, 200, 200, 200,
                                  200, 200, 200, 200, 200, 0, 0,
                                  0, 0, 0};

eModes mode = MAINMENU;

char new_press = FALSE;

// subrountines
// interrupt configs
void interruptCONFIG();

//input handler
void handle_new_keypad_press(eKey key) ;

//mode functionality
void mainmenu();
void ordering();
void orderconfirm();
void lookup();
void orderinfo();
void error(char *message);

//mode input interfacing
void mainmenu_input(eKey key);
void ordering_input(eKey key);
void orderconfirm_input(eKey key);
void lookup_input(eKey key);
void orderinfo_input(eKey key);
void error_input(eKey key);

//functions to turn jingle on and off
void TurnOffJingle(void);
void TurnOnJingle(void);

void initializePORTS(void);

int main(void) {

    initializePORTS();
    interruptCONFIG();
    
    mainmenu();
    /* Other initialization and configuration code */
    while (TRUE) {
        
    }
} 

void initializePORTS() {
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
    
    SSD_WriteDigits(code[3], code[2], code[1], code[0], 0, 0, 0, 0);
    
    // You need to enable all the rows
    R1 = R2 = R3 = R4 = 0;
}

void interruptCONFIG() {
    macro_disable_interrupts;
    CNCONDbits.ON = 1;   //all port D pins to trigger CN interrupts
    CNEND = 0x0F00;      	//configure PORTD pins 8-11 as CN pins
    CNPUD = 0x0F00;      	//enable pullups on PORTD pins 8-11

    IPC8bits.CNIP = 5;  	// set CN priority to  5
    IPC8bits.CNIS = 3;  	// set CN sub-priority to 3

    IFS1bits.CNDIF = 0;   	//Clear interrupt flag status bit
    IEC1bits.CNDIE = 1;   	//Enable CN interrupt on port D
    
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
    
    int j = PORTD;             //read port to clear mismatch on CN pins
    macro_enable_interrupts();
}

void __ISR(_TIMER_2_VECTOR) TM_HANDLER(void) {
    ledStatuses();
    updateStatuses();
    seconds++; // increment seconds each second
    TurnOffJingle(); // turn off jingle every second
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


//based on system mode, call input functions with key pressed as argument
void handle_new_keypad_press(eKey key)
{
    switch (mode) //check what the system mode is
    {
    case MAINMENU:
        mainmenu_input(key);
    break;
    case ORDERING:
        ordering_input(key);
    break;
    case ORDERCONFIRM:
        orderconfirm_input(key);
    break;       
    case LOOKUP:
        lookup_input(key);
        break;
    case ORDERINFO:
        orderinfo_input(key);
        break;
    case ERROR:
        error_input(key);
        break;
    }
}

//main menu | mode 1
void mainmenu(){
    LCD_WriteStringAtPos("                ",0,0); //anytime this pattern is seen
    LCD_WriteStringAtPos("                ",1,0); // it's to clear the LCD
    
    mode = MAINMENU;
    
    SSD_WriteDigits(-1, -1, -1, -1, 0, 0, 0, 0); // anytime this pattern is seen, it's to clear SSD
    LCD_WriteStringAtPos("Select Mode",0,0);
    LCD_WriteStringAtPos("A-Place B-Status",1,0);
}

//placing an order
void ordering(){
    menuindex = 0;
    LCD_WriteStringAtPos("                ",0,0);
    LCD_WriteStringAtPos("                ",1,0);
    mode = ORDERING;

    LCD_WriteStringAtPos("Place Your Order",0,0);
    LCD_WriteStringAtPos(menu[0],1,3); //when first ordering, the first item in the menu is shown
}

//order is placed
void orderconfirm() {
    LCD_WriteStringAtPos("                ",0,0);
    LCD_WriteStringAtPos("                ",1,0);
    mode = ORDERCONFIRM;
    TurnOnJingle(); // plays jingle, automatically turns off when the clock edge occurs for the timer interrupt, so may last anywhere from 0.1s - 1.0s
    
    LCD_WriteStringAtPos("Order Placed For",0,0);
    LCD_WriteStringAtPos(items[menuindex],1,4); // displays item in menu that was selected, without number attached
}

//looking up order
void lookup() {
    code[0] = -1; // this patterns clears the user typed code each time the user goes to lookup orders
    code[1] = -1;
    code[2] = -1;
    code[3] = -1;
    LCD_WriteStringAtPos("                ",0,0);
    LCD_WriteStringAtPos("                ",1,0);
    mode = LOOKUP;
    
    LCD_WriteStringAtPos("Order Lookup",0,0);
    LCD_WriteStringAtPos("Enter Code",1,0);
}

//order info shown for order looked up
void orderinfo() {
    LCD_WriteStringAtPos("                ",0,0);
    LCD_WriteStringAtPos("                ",1,0);
    mode = ORDERINFO;
    if (strcmp(LOOKUPITEM, "Burrito") == 0) { //this pattern checks what the item is for the order looked up
        LCD_WriteStringAtPos("Burrito",0,3); // and then displays the item
    }
    else if (strcmp(LOOKUPITEM, "Taco") == 0) {
        LCD_WriteStringAtPos("Taco",0,5);
    }
    else if (strcmp(LOOKUPITEM, "Fajita") == 0) {
        LCD_WriteStringAtPos("Fajita",0,4);
    }
    
    else {
        LCD_WriteStringAtPos("Err",0,0);
    }
    
    if (strcmp(LOOKUPSTATUS, "In Queue") == 0) { // this pattern checks the status of the order looked u[]
        LCD_WriteStringAtPos("In Queue",1,4); // and then displays the status
    }
    else if (strcmp(LOOKUPSTATUS, "In Prep") == 0) {
        LCD_WriteStringAtPos("In Prep",1,0);
    }
    else if (strcmp(LOOKUPSTATUS, "Ready") == 0) {
        LCD_WriteStringAtPos("Ready",1,0);
    }
    else {
        LCD_WriteStringAtPos("Err",1,0);
    }
}
//error protocol, takes a message as an argument and displays the error
void error(char *message) {
    LCD_WriteStringAtPos("                ",0,0);
    LCD_WriteStringAtPos("                ",1,0);
    mode = ERROR;
    LCD_WriteStringAtPos("Error!",0,5);
    LCD_WriteStringAtPos(message,1,1); // displays the error message passed to error()
}
//handles input in main menu
void mainmenu_input(eKey key){
    //Go to mode 2 if A key is pressed
    switch(key){
        case K_A:
            checkReady();
        break;
        case K_B:
            lookup(); // goes to lookup mode
    }
}
//handles input while ordering / scrolling thru menu
void ordering_input(eKey key){
    //wrap through menu items, select menu item
    switch(key){
        case K_A: 
            if (menuindex == 2) {
                menuindex = 0; // wraps back to first menu item
            }
            else {
                menuindex++; // goes to next menu item
            }
            LCD_WriteStringAtPos("                ",1,0);
            LCD_WriteStringAtPos(menu[menuindex],1,3); // displays menu item
        break;
        case K_B:
            if (menuindex == 0) {
                menuindex = 2; // wraps to last menu item
            }
            else {
                menuindex--; // goes to last menu item
            }
            LCD_WriteStringAtPos("                ",1,0);
            LCD_WriteStringAtPos(menu[menuindex],1,3); // displays menu item
            break;
        case K_E:
            LCD_WriteStringAtPos("                ",1,0);
            if (REPLACE) {
                queueOrder(createOrder(items[menuindex], REPLACEINDEX), queue, REPLACEINDEX);
                SSD_WriteDigits(queue[REPLACEINDEX].code[3], // writes order code to digits
                            queue[REPLACEINDEX].code[2], 
                            queue[REPLACEINDEX].code[1], 
                            queue[REPLACEINDEX].code[0], 
                            0, 0, 0, 0);
                REPLACE = 0;
            }
            else {
                queueOrder(createOrder(items[menuindex], QUEUESIZE), queue, QUEUESIZE); // creates order and adds to queue
                SSD_WriteDigits(queue[QUEUESIZE].code[3], // writes order code to digits
                            queue[QUEUESIZE].code[2], 
                            queue[QUEUESIZE].code[1], 
                            queue[QUEUESIZE].code[0], 
                            0, 0, 0, 0);
            }
            if (QUEUESIZE < 8) {
                QUEUESIZE++;
            }
            orderconfirm(); // moves to orderconfirm mode
        break;
    }
}
// handled input for confirming order
void orderconfirm_input(eKey key){
    if (key == K_E) {
        mainmenu(); // moves to main menu
    }
}
//handles input for looking up order
void lookup_input(eKey key){
    if (key == K_C) { // clears SSD and returns 'cursor' to first digit of SSD
        ssd_index = 1;
        code[0] = -1;
        code[1] = -1;
        code[2] = -1;
        code[3] = -1;
    }
    if (ssd_index == 1) { 
        switch(key){
            case K_D:
                code[0] = -1; // deletes first digit of SSD, this pattern is repeated
                break;
            case K0:
                code[0] = 0;
                ssd_index++; // moves to next digit of SSD, this pattern is repeated
                break;
            case K1:
                code[0] = 1;
                ssd_index++;
                break;
            case K2: 
                code[0] = 2;
                ssd_index++;
                break;
            case K3:
                code[0] = 3;
                ssd_index++;
                break;
            case K4:
                code[0] = 4;
                ssd_index++;
                break;
            case K5:
                code[0] = 5;
                ssd_index++;
                break;
            case K6:
                code[0] = 6;
                ssd_index++;
                break;
            case K7: 
                code[0] = 7;
                ssd_index++;
                break;
            case K8:
                code[0] = 8;
                ssd_index++;
                break;
            case K9:
                code[0] = 9;
                ssd_index++;
                break;
        }
        SSD_WriteDigits(code[3], code[2], code[1], code[0], 0, 0, 0, 0);
    }
    else if (ssd_index == 2){
        switch(key){
            case K_D:
                code[0] = -1;
                ssd_index--;
                break;
            case K0:
                code[1] = 0;
                ssd_index++;
                break;
            case K1:
                code[1] = 1;
                ssd_index++;
                break;
            case K2: 
                code[1] = 2;
                ssd_index++;
                break;
            case K3:
                code[1] = 3;
                ssd_index++;
                break;
            case K4:
                code[1] = 4;
                ssd_index++;
                break;
            case K5:
                code[1] = 5;
                ssd_index++;
                break;
            case K6:
                code[1] = 6;
                ssd_index++;
                break;
            case K7: 
                code[1] = 7;
                ssd_index++;
                break;
            case K8:
                code[1] = 8;
                ssd_index++;
                break;
            case K9:
                code[1] = 9;
                ssd_index++;
                break;
        }
        SSD_WriteDigits(code[3], code[2], code[1], code[0], 0, 0, 0, 0);
    }
    else if (ssd_index == 3){
        switch(key){
            case K_D:
                code[1] = -1;
                ssd_index--;
                break;
            case K0:
                code[2] = 0;
                ssd_index++;
                break;
            case K1:
                code[2] = 1;
                ssd_index++;
                break;
            case K2: 
                code[2] = 2;
                ssd_index++;
                break;
            case K3:
                code[2] = 3;
                ssd_index++;
                break;
            case K4:
                code[2] = 4;
                ssd_index++;
                break;
            case K5:
                code[2] = 5;
                ssd_index++;
                break;
            case K6:
                code[2] = 6;
                ssd_index++;
                break;
            case K7: 
                code[2] = 7;
                ssd_index++;
                break;
            case K8:
                code[2] = 8;
                ssd_index++;
                break;
            case K9:
                code[2] = 9;
                ssd_index++;
                break;
        }
        SSD_WriteDigits(code[3], code[2], code[1], code[0], 0, 0, 0, 0);
    }
    else if (ssd_index == 4){
        switch(key){
            case K_D:
                code[2] = -1;
                ssd_index--;
                break;
            case K0:
                code[3] = 0;
                ssd_index++;
                break;
            case K1:
                code[3] = 1;
                ssd_index++;
                break;
            case K2: 
                code[3] = 2;
                ssd_index++;
                break;
            case K3:
                code[3] = 3;
                ssd_index++;
                break;
            case K4:
                code[3] = 4;
                ssd_index++;
                break;
            case K5:
                code[3] = 5;
                ssd_index++;
                break;
            case K6:
                code[3] = 6;
                ssd_index++;
                break;
            case K7: 
                code[3] = 7;
                ssd_index++;
                break;
            case K8:
                code[3] = 8;
                ssd_index++;
                break;
            case K9:
                code[3] = 9;
                ssd_index++;
                break;
        }
        SSD_WriteDigits(code[3], code[2], code[1], code[0], 0, 0, 0, 0);
    }
    else if (ssd_index == 5) {
        if (key == K_D) {
            code[3] = -1;
            ssd_index--;
        }
        else if (key == K_E) {
            ssd_index = 1; // sets ssd index back to first digit
            if (compareCodes(queue, code)) { // if the code entered is the same as any of the order codes in the queue
                orderinfo(); // move to display the order info
            }
            else if (compareCodes(queue, code) == 0) { // if the code does not match any of the orders
                error(INVALIDCODE); // move to error mode with invalid code message
            }
        }
        SSD_WriteDigits(code[3], code[2], code[1], code[0], 0, 0, 0, 0); // show the numbers currently typed
    }  
    else {
        LCD_WriteStringAtPos("YOU'RE COOKED",1,0);
    }
}
//handles input for when order info is displayed
void orderinfo_input (eKey key) {
    if (key == K_E) {
        mainmenu(); // move to main menu
    }
}
//mode for when error is diplayed
void error_input(eKey key) {
    if (key == K_E) {
        mainmenu(); // move to main menu
    }
}

//creates a variable of type order based on menu item and queue position as arguments
Order createOrder(const char* item, int QUEUESIZE) {
    Order newOrder;
    strncpy(newOrder.item, item, sizeof(newOrder.item) - 1); // manipulates item field of newOrder
    newOrder.item[sizeof(newOrder.item) - 1] = '\0'; // Ensures null-termination
    strcpy(newOrder.status, "In Queue"); // sets status of newOrder
    newOrder.pos = QUEUESIZE;
    newOrder.code[0] = rand() % 10; // randomly generate code
    newOrder.code[1] = rand() % 10;
    newOrder.code[2] = rand() % 10;
    newOrder.code[3] = rand() % 10;
    newOrder.time = 0;
    return newOrder;
}

//adds variable of type order to queue
void queueOrder(Order newOrder, Order queue[], int QUEUESIZE) {
    queue[QUEUESIZE] = newOrder; // placed Order passed into function into queue array
}
//compares codes of orders to code entered on keypad in lookup()
int compareCodes(Order queue[], int lookupcode[]) {
    int i;
    for(i=0;i<QUEUESIZE;i++) { // iterates through queue to see if code entered matched any order codes
        if (queue[i].code[0] == lookupcode[0] && queue[i].code[1] == lookupcode[1] && queue[i].code[2] == lookupcode[2] && queue[i].code[3] == lookupcode[3]) {
            LOOKUPITEM = queue[i].item; // stores order item for display
            LOOKUPSTATUS = queue[i].status; // stores order status for display
            return 1; //returns 1 if codes matched
        }
    }
    return 0; // returns 0 if no match found
}

void ledStatuses(void) {
    for(int i=0;i<QUEUESIZE;i++) {
        if (strcmp(queue[i].status, "In Queue") == 0) {
            LED_SetValue(i, 1);
        }
        else if (strcmp(queue[i].status, "In Prep") == 0) {
            LED_SetValue(i, 0); // turns off for now, will do blinking soon
        }
        else if (strcmp(queue[i].status, "Ready") == 0) {
            if (seconds%2 == 0) {
                LED_SetValue(i, 1);
            }
            else if (seconds%2 == 1) {
                LED_SetValue(i, 0);
            }
        }
        else if (strcmp(queue[i].status, "No Status") == 0) {
            LED_SetValue(i, 0);
        }
    }
}

void updateStatuses(void) {
    for(int i=0;i<QUEUESIZE;i++) {
        if (strcmp(queue[i].status, "In Queue") == 0 && !PREPPING) {
            strncpy(queue[i].status, "In Prep", sizeof(queue[i].status) - 1); // puts each order in queue to prep
            queue[i].status[sizeof(queue[i].status) - 1] = '\0'; // Ensures null-termination
            PREPPING = 1;
        }
        else if (strcmp(queue[i].status, "In Prep") == 0) {
            if (queue[i].time > 10) {
                strncpy(queue[i].status, "Ready", sizeof(queue[i].status) - 1); // puts each order in queue to prep
                queue[i].status[sizeof(queue[i].status) - 1] = '\0'; // Ensures null-termination
                PREPPING = 0;
            }
            else {
                queue[i].time++;
            }
        }
    }
}

void checkReady(void) {
    if (QUEUESIZE == 8) {
        for (int i=0; i < QUEUESIZE && !REPLACE; i++) {
            if (strcmp(queue[i].status, "Ready") == 0) {
                REPLACEINDEX = i;
                REPLACE = 1;
            }
        }
        if (REPLACE) {
            ordering();
        }
        else {
            error(QUEUEFULL);
        }
    }
    else {
        ordering();
    }
}

//plays a simple sound when order is placed
void TurnOnJingle(void) {
    int i;
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
//self explanatory, turns jingle off
void TurnOffJingle(void) {
    T3CONbits.ON = 0;       // turn off Timer3
    OC1CONbits.ON = 0;      // Turn off OC1
}