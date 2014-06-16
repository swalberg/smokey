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

typedef struct {
    double error;
    double setpoint;
    double temperature;
    double accumulated_error;
    double last_error;
} pidInfoStruct;

char mode = 0;
int foo = 100;
char digit;
pidInfoStruct pidData;

void setup(void) {

    pidData.accumulated_error = 0;
    pidData.setpoint = 225;
    pidData.error = 0;
    
    ADCON1 = 0x06; // All pins digital
    ADCON0 = 0; // A/D off
    
    TRISA = 0b00000000;
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

double percentage_error(double measured, double setpoint) {
    return(measured/setpoint);
}
void /* interrupt */ tc_int(void) {
    static volatile char tmr1count = 0;
    
    
    if (T0IE && T0IF) {  // Timer 0 resets quickly
        T0IF = 0;
//        displayDigit(digit, pickDigit(digit, mode == 0 ? (int)pidData.temperature : foo));
        digit++;
        digit %= 3;
        return;
    }

    if (TMR1IE && TMR1IF) {
        if (tmr1count++ == 10) { // every 2 seconds
            tmr1count = 0;
           // PORTAbits.RA0 = ! PORTAbits.RA0;

            // update accumulated_error
            if (pidData.error != 0) {
                pidData.accumulated_error += pidData.error;
                if(pidData.accumulated_error > 500) {
                    pidData.accumulated_error = 500;
                }
            }
        }
        TMR1IF=0;
    }
}


double run_pid(double error, pidInfoStruct *pid) {

    double p, i, d;

    p = 2.0 * error;
    i = 3.0 * pid->accumulated_error;
    d = 4.0 * (error - pid->last_error);

    pid->last_error = error;

    return p + i + d;
}
void stepper_delay(void) {
    int i;
    for (i=0; i < 1; i++) {
        __delay_ms(5);
    }
}

#define ON 1
#define OFF 0

void blue(int status) {
    PORTB &= 0b11111101;
    PORTB |= (status << 1);
}

void pink(int status) {
    PORTB &= 0b11111011;
    PORTB |= (status << 2);
}

void yellow(int status) {
    PORTB &= 0b11110111;
    PORTB |= (status << 3);
}

void orange(int status) {
    PORTB &= 0b11101111;
    PORTB |= (status << 4);
}

int main(void) {

    int d;
    setup();

    while (1) {
        blue(ON);
        pink(OFF);
        yellow(OFF);
        orange(OFF);

        stepper_delay();

        blue(OFF);
        pink(ON);
        yellow(OFF);
        orange(OFF);

        stepper_delay();

        blue(OFF);
        pink(OFF);
        yellow(ON);
        orange(OFF);

        stepper_delay();

        blue(OFF);
        pink(OFF);
        yellow(OFF);
        orange(ON);

        stepper_delay();

    }

    while(1) {
       pidData.temperature = readTemperature();
       pidData.error = percentage_error(pidData.temperature, pidData.setpoint);
        
        __delay_ms( 10 );
    }
   
    return 0;
}
