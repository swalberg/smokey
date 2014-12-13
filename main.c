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
#include "motor.h"

#define MAX_INTEGRAL 250
#define MIN_INTEGRAL 0
#define MAX_OPEN 160

typedef struct {
    double error;
    double setpoint;
    double temperature;
    double accumulated_error;
    double last_error;
    double last_value;
    unsigned long int last_time;
} pidInfoStruct;

char mode = 0;
int foo = 100;
char digit;
pidInfoStruct pidData;
unsigned long int current_time;

void setup(void) {

    pidData.accumulated_error = 0;
    pidData.setpoint = 110; // Celsius!
    pidData.error = 0;
    current_time = 0;
    
    ADCON1 = 0x06; // All pins digital
    ADCON0 = 0; // A/D off
    
    TRISA = 0b00000000;
    TRISB = 0b00000000; // B0 is CS for the temperature sensor
                        // B1:4 are for the stepper motor
    TRISD = 0x00; // PORTD is the 7 segment LED
    TRISE = 0x00; // PORTE controls the status of the LED
    TRISC = 0b00010000; // C4 is SPI data in

    spiSlaveDisable(0);
 
    digit = 0;

    TMR0 = 0;
    T0CON = 0b11010100; // enable Timer0 as 8 bit, 1:32 prescale

    T1CON = 0b00100001; // 1:4 prescale, Fosc/4

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

double calculate_error(double measured, double setpoint) {
    return(setpoint - measured);
}

void interrupt tc_int(void) {
    static volatile char tmr1count = 0;
    
    
    if (T0IE && T0IF) {  // Timer 0 resets quickly
        T0IF = 0;
        switch(mode) {
            case 0: // temp
                displayDigit(digit, pickDigit(digit, (int)pidData.temperature));
                break;
            case 1: // last value of pid
                displayDigit(digit, pickDigit(digit, (int)pidData.last_value));
                break;
            case 2: // acc error
                displayDigit(digit, pickDigit(digit, (int)pidData.accumulated_error));
                break;
        }
        digit++;
        digit %= 3;
        return;
    }

    if (TMR1IE && TMR1IF) { // every .1 seconds
        current_time++;
       
        if (current_time % 128 == 0) {
            mode += 1;
            mode %= 3;
        }
        TMR1IF=0;
    }
}

double run_pid(pidInfoStruct *pid) {

    double p, i, d;
    double output;

#define PID_FREQUENCY 10 // every second
    if (current_time - pid->last_time < PID_FREQUENCY) {
        return(pid->last_value);
    }

    pid->temperature = readTemperature();

    pid->error = pid->setpoint - pid->temperature;
    pid->accumulated_error += pid->error;
    if (pid->accumulated_error > MAX_INTEGRAL) {
        pid->accumulated_error = MAX_INTEGRAL;
    }
    if (pid->accumulated_error < MIN_INTEGRAL) {
        pid->accumulated_error = MIN_INTEGRAL;
    }
    
    p = 20 * pid->error;
    i = 0.2 * pid->accumulated_error;
    d = pid->error - pid->last_error;
    
    pid->last_error = pid->error;
    pid->last_time = current_time;
    
    output = p + i + d;
    pid->last_value = output;

   return(output);
}
int move_to(int from, int to) {
    if (to < 0) {
        to = 0;
    }
    if (to > MAX_OPEN) {
        to = MAX_OPEN;
    }

    if (from > to) {
        forward(from - to);
    } else {
        backward(to - from);
    }
    return(to);
}
int main(void) {

    int d, current_position;
    double setting;

    current_position = 0;
    setup();

    backward(150); forward(150);
    while(1) {
        setting = run_pid(&pidData);
        current_position = move_to(current_position, (int)pidData.last_value);
        __delay_ms( 10 );
    }
   
    return 0;
}
