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
 *  H-Bridge connections for driving a stepper motor.
 *	RC0		=	L293 Enable line
 *	RC1		=	Phase A control line
 *  	RC2		=	Phase A control line
 *  	RC3		=	Phase B control line
 *  	RC4		=	Phase B control line
 *
 * Author               Date        Comment
 * David Fisher       9/25/07      Created
 * David Fisher       12/18/09     Moved enable line to RB0
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/


/** Processor Header Files *****************************************/
#include <p18f4520.h> 
#include <timers.h>
#include <adc.h>

/** Define Constants Here ******************************************/
#define STEP1 0b00001010
#define STEP2 0b00001001
#define STEP3 0b00000101
#define STEP4 0b00000110

#define REV_TO_DIST 100 //TODO: change

/** Local Function Prototypes **************************************/
void low_isr(void);
void high_isr(void);
char moveForward(char recentState);
char moveBackwards(char recentState);

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
int motor_spd = 65380;
int goalX = 0;
int goalY = 0;
int x = 0;
int y = 0;


/*****************************************************************
 * Function:        void main(void)
 ******************************************************************/
#pragma code

void main(void) {
    // Run the clock at 500 kHz (I could've picked about anything)
    OSCCONbits.IRCF2 = 0;
    OSCCONbits.IRCF1 = 1;
    OSCCONbits.IRCF0 = 1;

    // Setup the timer with a 1:4 prescaler with 16 bits resolution
    // Therefore the timer0 freq is 500 kHz / 4 / 4 = 31.25 kHz
    OpenTimer0(TIMER_INT_ON & T0_16BIT & T0_SOURCE_INT & T0_PS_1_4);
    // Should take a little over 2 seconds to overflow the counter from TMR0 = 0
    // If you write in a different starting value for TMR0 it'll overflow sooner

    WriteTimer0(65530); // Start the timer at a some high value to get to the ISR the first time asap

    // Enable Global interrupts
    INTCONbits.GIE = 1; //  Enable High priority interrupt

    // Setup the digital IO pins
    ADCON1 = 0x0F; // Make sure they are digital not analog
    TRISC = 0xE0; // Make the RC4:RC0 outputs
    PORTC = 0x00; // Clear the bits to start with

    OpenADC(ADC_FOSC_8 & ADC_RIGHT_JUST & ADC_12_TAD,
            ADC_CH0 & ADC_INT_OFF & ADC_REF_VDD_VSS,
            0x0E);

    while (1) {
        // A blank while loop, think of all the things you could do here!
        // When you use an interrupt the main loop is free for something else
        SetChanADC(ADC_CH0);
        ConvertADC();
        while (BusyADC());
        goalX = ReadADC()*8500 / 1023;

        SetChanADC(ADC_CH1);
        ConvertADC();
        while (BusyADC());
        goalY = ReadADC()*1100 / 1023;
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
        if (x - goalX > 0) {
            recentStateX = moveForward(recentStateX);
            x += REV_TO_DIST;
        } else if (x - goalX < 0) {
            recentStateX = moveBackwards(recentStateX);
            x -= REV_TO_DIST;
        }
        if (y - goalY > 0) {
            recentStateY = moveForward(recentStateY);
            recentStateY = recentStateY << 4;
            y += REV_TO_DIST;
        } else if (y - goalY < 0) {
            recentStateY = moveBackwards(recentStateY);
            recentStateY = recentStateY << 4;
            y -= REV_TO_DIST;
        }
    }

    PORTC = recentStateX | recentStateY;
    //TODO: bit shifting magic for y movement

    // The Timer0 frequency is 31.25 kHz 
    // Pick where to start the time to determine how fast it overflows
    // Every overflow, the stepper motor will take a single step

    //WriteTimer0(3036);   // 1 step every 2 seconds
    //WriteTimer0(18661);  // 1 step every 1.5 seconds
    //WriteTimer0(34286);  // 1 step every 1 seconds
    //WriteTimer0(49911);  // 1 step every 0.5 seconds
    //WriteTimer0(57723);  // 1 step every 0.25 seconds
    //WriteTimer0(62411);  // 1 step every 0.1 seconds
    //WriteTimer0(63973);  // 20 step every second
    //WriteTimer0(64911);  // 50 step every second
    //WriteTimer0(65224);  // 100 step every second
    WriteTimer0(motor_spd); // 200 step every second
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
