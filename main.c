/*
 * File:   main.c
 * Author: sean
 *
 * Created on August 11, 2013, 1:16 PM
 */

#include "config.h"
#include <pic18.h>
#include <htc.h>

#include <xc.h>

#include "display.h"
#include "spi.h"

// Globals
int temperature = 0;
int setpoint;
long error;
char mode = 0;
int foo = 100;
char digit;

void setup(void) {
    ADCON1 = 0x06; // All pins digital
    ADCON0 = 0; // A/D off
    
    TRISA = 0b00111000;
    TRISB = 0b00000000; // B0 is CS for the temperature sensor
    TRISD = 0x00; // PORTD is the 7 segment LED
    TRISE = 0x00; // PORTE controls the status of the LED
    TRISC = 0b00010000; // C4 is SPI data in

    spiSlaveDisable(0);
 
    digit = 0;

    TMR0 = 0;
    T0CON = 0b11010100; // enable Timer0 as 8 bit, 1:32 prescale

    T1CON = 0b00110001; // 1:8 prescale, Fosc/4

    INTCONbits.GIE = 1;     // Enable global interrupts
    INTCONbits.PEIE = 1;    // Enable peripheral interrupts
    INTCONbits.TMR0IF = 0;    // Clear Timer 0 interrupt flag
    INTCONbits.TMR0IE = 1;    // Enable Timer 0 interrupt

    TMR1IE = 1;
    TMR1IF = 0;
    /* Initialize SPI */
    SSPEN = 0;
    SSPSTAT=0b01000000; /* 7 0 - sample input at middle of data output time
                           6 1 - Transmit occurs on transition from active to Idle clock state
                           rest is read only
                         */
    SSPCON1=0b00100010;  /* 7 0 - no collision
                           6 0 - ignore: for SPI slave mode
                           5 1 - Enables serial port and configures SCK, SDO, SDI, and SS as serial port pins
                           4 0 - Idle state for clock is a low level
                           3:0 0010 - SPI Master mode, clock = FOSC/64 */
    SSPEN=1;            // SPI enable!
}

void interrupt tc_int(void) {
    static volatile char tmr1count = 0;

    if (T0IE && T0IF) {  // Timer 0 resets quickly
        T0IF = 0;
        displayDigit(digit, pickDigit(digit, mode == 0 ? temperature : foo));
        digit++;
        digit %= 3;
        return;
    }

    if (TMR1IE && TMR1IF) {
        if (tmr1count++ == 10) { // every 2 seconds
            tmr1count = 0;
            PORTAbits.RA0 = ! PORTAbits.RA0;
        }
        TMR1IF=0;
    }
}

int main(void) {

    int d;
    setup();
    while(1) {
 
        if (PORTAbits.RA3 == 1) {
            mode = ! mode;
            while (PORTAbits.RA3 == 1) ;
        }
        if (PORTAbits.RA4 == 1) {
            foo++;
            while (PORTAbits.RA4 == 1) ;
        }
        if (PORTAbits.RA5 == 1) {
            foo--;
            while (PORTAbits.RA5 == 1) ;
        }
        temperature = readTemperature();
        __delay_ms( 10 );
    }
   
    return 0;
}
