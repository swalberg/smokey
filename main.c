/*
 * File:   main.c
 * Author: sean
 *
 * Created on August 11, 2013, 1:16 PM
 */


#include <xc.h>

#define _XTAL_FREQ 20000000
#pragma config FOSC = HS        // Oscillator Selection bits (HS oscillator)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable bit (BOR enabled)
#pragma config CP = OFF         // Flash Program Memory Code Protection bit (Code protection off)
#pragma config LVP = OFF

// Globals
int temperature;
char mode = 0;
int foo = 100;
char digit;

void spiSlaveDisable(char);
void spiSlaveEnable(char);

void setup(void) {
    ADCON1 = 0x06;
    
    TRISA = 0b00111000;
    TRISB = 0x00; // PORTB is the 7 segment LED
    TRISC = 0b00010000; // C4 is SPI data in

    spiSlaveDisable(0);

    
    digit = 0;

    TMR0 = 0;
    OPTION_REG = 0b00000011; // prescalar = 1:16 on timer 0

    INTCONbits.GIE = 1;     // Enable global interrupts
    INTCONbits.PEIE = 1;    // Enable peripheral interrupts
    INTCONbits.TMR0IF = 0;    // Clear Timer 1 interrupt flag
    INTCONbits.TMR0IE = 1;    // Enable Timer 1 interrupt

    /* Initialize SPI */
    SSPEN = 0;
    SSPSTAT=0b01000000; /* 7 0 - sample input at middle of data output time
                           6 1 - Transmit occurs on transition from active to Idle clock state
                           rest is read only
                         */
    SSPCON=0b00100010;  /* 7 0 - no collision
                           6 0 - for SPI slave mode
                           5 1 - Enables serial port and configures SCK, SDO, SDI, and SS as serial port pins
                           4 0 - Idle state for clock is a low level
                           3:0 0010 - SPI Master mode, clock = FOSC/64 */
    SSPEN=1;            // SPI enable!
}

void enableDigit(char position) {
    char mask = 0b00000001; // A0:2
    mask = mask << position;
    PORTA &= 0b11111000;
    PORTA |= mask;
}

char readSPIByte() {
    PIR1bits.SSPIF=0; // clear interrupt
    SSPBUF=0x00;
    while (!SSPSTATbits.BF) ; // wait until done
    return (SSPBUF);

}
int readTemperature(void) {
    char b1, b2;

    spiSlaveEnable(0);
    __delay_ms(1);
    b1 = readSPIByte();
    b2 = readSPIByte();
    __delay_ms(1);
    
    spiSlaveDisable(0);

    // b1[6:0] and b2[7:2] divided by 4
    temperature = (b1 << 5 | b2 >>3) >> 2;
}

void spiSlaveEnable(char device) {
    // enable the MAX6675
    PORTC &= 0b10111111;
}

void spiSlaveDisable(char device) {
    // Turn off the MAX6675
    PORTC |= 0b01000000;
}

void displayDigit(char position, char value) {
    char tmp_port_c;
    char digits[10] = {0x22, 0xaf, 0x31, 0x25, 0xac, 0x64, 0x60, 0x2f, 0x20, 0x24};
    char digit;

    tmp_port_c = PORTC;

    tmp_port_c &= 0b11111100; // all display off, clear led
    
    digit = digits[value];
    PORTB = (digit >> 2);
    PORTC = (tmp_port_c | (digit & 3));

    enableDigit(position);
}

char pickDigit(char position, int from) {
    char display;
    
    if (position == 0) {
        display = (from - (from % 100)) / 100;
    } else if (position == 1) {
        display = (from % 100 - from % 10) / 10;
    } else {
        display = from % 10;
    }
    return(display);
}
void interrupt tc_int(void) {
    if (T0IE && T0IF) {  // Timer 0 resets quickly
        T0IF = 0;
        displayDigit(digit, pickDigit(digit, mode == 0 ? temperature : foo));
        digit++;
        digit %= 3;
        return;
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
        readTemperature();
        __delay_ms( 100 );
    }
   
    return 0;
}
