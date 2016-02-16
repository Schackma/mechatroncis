/********************************************************************
 * FileName:        Stepper_Motor_using_interrupts.c
 * Processor:       PIC18F4520
 * Compiler:        MPLAB C18 v.3.36 
 *
 * This file uses the timer 0 to set an interrupt event.  When the 
 *  interrupt occurs, it changes the RC0:RC4 state.  You can modify
 *  code within the high priority interrupt to change how often the
 *  interrupt occurs.  
 *
 *  
 *
 * Author                           Date        Comment
 * Peter Heath & Matthew Schack     2/9/10      all sorts of fun stuff
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/


/** Processor Header Files *****************************************/
#include <p18f4520.h> 
#include <timers.h>
#include <adc.h>
#include <delays.h>

/** Define Constants Here ******************************************/
#define STEP1 0b00001010
#define STEP2 0b00001001
#define STEP3 0b00000101
#define STEP4 0b00000110

#define PRESSED 0
#define UNPRESSED 1

#define AT_POINT 1
#define NOT_AT_POINT 0;

#define TOL 4

#define REV_TO_DIST 12 //TODO: change  0.0123 inches/tick

/** Local Function Prototypes **************************************/
void low_isr(void);
void high_isr(void);
char moveForward(char recentState);
char moveBackwards(char recentState);
void move(void);
void home(void);
void movePattern(void);

// ============================================================
// Configuration Bits 
// ============================================================
#pragma config OSC = INTIO67  // Use the internal oscillator
#pragma config WDT = OFF
#pragma config LVP = OFF
#pragma config BOREN = OFF
#pragma config XINST = OFF

/** Declare Interrupt Vector Sections ****************************/
#pragma code high_vector=0x08

void interrupt_at_high_vector(void) {
    _asm goto high_isr _endasm
}

#pragma code low_vector=0x18

void interrupt_at_low_vector(void) {
    _asm goto low_isr _endasm
}

/** Global Variables *********************************************/
char recentStateX = STEP1;
char recentStateY = STEP1;
int motor_spd = 65410;
volatile long goalX = 0;
volatile long goalY = 0;
volatile long x = 0;
volatile long y = 0;
volatile long xLength = 0;
volatile long yLength = 0;
int hereX = 0;
int hereY = 0;
int stepsTillRead = 100;
int steps = 0;

/*****************************************************************
 * Function:        void main(void)
 ******************************************************************/
#pragma code

void main(void) {
    // Run the clock at 500 kHz (I could've picked about anything)
    OSCCONbits.IRCF2 = 0;
    OSCCONbits.IRCF1 = 1;
    OSCCONbits.IRCF0 = 1;




    // Enable Global interrupts
    INTCONbits.GIE = 1; //  Enable High priority interrupt

    // Setup the digital IO pins
    ADCON1 = 0x0D; // Make RA0:RA1 analog inputs
    TRISC = 0x00; // Make the RC ports outputs
    TRISD = 0x00; // Make RD ports outputs
    PORTC = 0x00; // Clear the bits to start with
    TRISE = 0x07; // RE0:RE2 are inputs

    OpenADC(ADC_FOSC_8 & ADC_RIGHT_JUST & ADC_12_TAD,
            ADC_CH0 & ADC_INT_OFF & ADC_REF_VDD_VSS,
            0x0E);

    home();
    OpenTimer0(TIMER_INT_ON & T0_16BIT & T0_SOURCE_INT & T0_PS_1_4);
    WriteTimer0(65530); // Start the timer at a some high value to get to the ISR the first time asap
    hereX = 1;
    hereY = 1;
    while (1) {
        // A blank while loop, think of all the things you could do here!
        // When you use an interrupt the main loop is free for something else
        if (steps % stepsTillRead == 0) {
            SetChanADC(ADC_CH0);
            ConvertADC();
            while (BusyADC());
            goalX = (long) ReadADC() * xLength * REV_TO_DIST / 1023;

            SetChanADC(ADC_CH1);
            ConvertADC();
            while (BusyADC());
            goalY = (long) ReadADC() * yLength * REV_TO_DIST / 1023;
        }
        if (PORTEbits.RE1 == PRESSED) {
            movePattern();
        }
    }
}


/*****************************************************************
 * Function:        void high_isr(void)
 * Overview:  This interrupt changes the state of the RC4:RC0 pins when
 *            the timer zero overflows (0xFFFF -> 0x0000) and triggers
 *            this interrupt code to run
 ******************************************************************/
#pragma interrupt high_isr

void high_isr(void) {
    // Check whether it was the timer interrupt that got us here 
    // (better be, since that's the only interrupt right now)
    if (INTCONbits.TMR0IF) {
        INTCONbits.TMR0IF = 0; // Clear interrupt flag for timer 0
        move(); // call move function to increment position
    }

}

/******************************************************************
 * Function:        void low_isr(void)
 * Input:
 * Output:
 * Overview:
 ********************************************************************/
#pragma interruptlow low_isr

void low_isr(void) {
    // no low isr
}

char moveForward(char recentState) {
    switch (recentState) {
        case STEP1:
            recentState = STEP2;
            break;
        case STEP2:
            recentState = STEP3;
            break;
        case STEP3:
            recentState = STEP4;
            break;
        case STEP4:
            recentState = STEP1;
            break;
        default:
            recentState = STEP1;
            break;
    }
    return recentState;
}

char moveBackwards(char recentState) {
    switch (recentState) {
        case STEP1:
            recentState = STEP4;
            break;
        case STEP2:
            recentState = STEP1;
            break;
        case STEP3:
            recentState = STEP2;
            break;
        case STEP4:
            recentState = STEP3;
            break;
        default:
            recentState = STEP1;
            break;
    }
    return recentState;
}

void move(void) {
    char output;
    if (x - goalX < -TOL * REV_TO_DIST) {
        recentStateX = moveForward(recentStateX);
        x += REV_TO_DIST;
        steps++;
        hereX = NOT_AT_POINT;
    } else if (x - goalX >= TOL * REV_TO_DIST) {
        recentStateX = moveBackwards(recentStateX);
        x -= REV_TO_DIST;
        steps++;
        hereX = NOT_AT_POINT;

    } else {
        steps++;
        hereX = AT_POINT;
    }
    if (y - goalY < -TOL * REV_TO_DIST) {
        recentStateY = moveForward(recentStateY);
        //            recentStateY = recentStateY << 4;
        y += REV_TO_DIST;
        steps++;
        hereY = NOT_AT_POINT;
    } else if (y - goalY >= TOL * REV_TO_DIST) {
        recentStateY = moveBackwards(recentStateY);
        //            recentStateY = recentStateY << 4;
        y -= REV_TO_DIST;
        steps++;
        hereY = NOT_AT_POINT;
    } else {
        steps++;
        hereY = AT_POINT;
    }
    output = recentStateX;
    PORTC = output;
    output = recentStateY;
    PORTD = output;
    WriteTimer0(motor_spd);
}

void home(void) {
    while (PORTEbits.RE0 != PRESSED);
    while (PORTBbits.RB2 != PRESSED) {
        int output;
        recentStateX = moveBackwards(recentStateX);
        Delay100TCYx(5);
        output = recentStateX;
        PORTC = output;
        output = recentStateY;
        PORTD = output;
    }
    while (PORTBbits.RB3 != PRESSED) {
        int output;
        recentStateX = moveForward(recentStateX);
        Delay100TCYx(5);
        output = recentStateX;
        PORTC = output;
        output = recentStateY;
        PORTD = output;
        xLength++;
    }
    while (PORTBbits.RB1 != PRESSED) {
        int output;
        recentStateY = moveBackwards(recentStateY);
        Delay100TCYx(5);
        output = recentStateX;
        PORTC = output;
        output = recentStateY;
        PORTD = output;
    }
    while (PORTBbits.RB0 != PRESSED) {
        int output;
        recentStateY = moveForward(recentStateY);
        Delay100TCYx(5);
        output = recentStateX;
        PORTC = output;
        output = recentStateY;
        PORTD = output;
        yLength++;
    }
    x = xLength*REV_TO_DIST;
    y = yLength*REV_TO_DIST;
}

void movePattern(void) {
    
    while (PORTEbits.RE2 != PRESSED) {
        goalY = (long) 100 * yLength * REV_TO_DIST / 1023;
        while (hereY != AT_POINT && PORTEbits.RE2 != PRESSED) {

        }
        goalX = (long) 100 * yLength * REV_TO_DIST / 1023;
        while (hereX != AT_POINT && PORTEbits.RE2 != PRESSED) {

        }
        goalY = (long) 1000 * yLength * REV_TO_DIST / 1023;
        while (hereY != AT_POINT && PORTEbits.RE2 != PRESSED) {

        }
        goalX = (long) 1000 * yLength * REV_TO_DIST / 1023;
        while (hereX != AT_POINT && PORTEbits.RE2 != PRESSED) {

        }
    }
    WriteTimer0(65530);
}