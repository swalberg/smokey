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
char digit;

void setup(void) {
    TRISA = 0b00000000;
    TRISB = 0x00; // PORTB is the 7 segment LED
    TRISC = 0b00000000;

    temperature = 451;
    digit = 0;

    TMR0 = 0;
    OPTION_REG = 0B00000001;

    INTCONbits.GIE = 1;     // Enable global interrupts
    INTCONbits.PEIE = 1;    // Enable peripheral interrupts
    INTCONbits.TMR0IF = 0;    // Clear Timer 1 interrupt flag
    INTCONbits.TMR0IE = 1;    // Enable Timer 1 interrupt
}

void enableDigit(char position) {
    char mask = 0b00000001; // A0:2
    mask = mask << position;
    PORTA = mask;
}
void displayDigit(char position, char value) {
    char tmp_port_c;
    char digits[10] = {0x22, 0xaf, 0x31, 0x25, 0xac, 0x64, 0x60, 0x2f, 0x20, 0x24};
    char digit;

    tmp_port_c = PORTC;

    tmp_port_c &= 0b00011100; // all display off, clear led
    
    digit = digits[value];
    PORTB = (digit >> 2);
    PORTC = (tmp_port_c | (digit & 3));

    enableDigit(position);
}

void interrupt tc_int(void) {
    if (T0IE && T0IF) {  // Timer 0 resets quickly
        T0IF = 0;
        char display;
        if (digit == 0) {
            display = (temperature - (temperature % 100)) / 100;
        } else if (digit == 1) {
            display = (temperature % 100 - temperature % 10) / 10;
        } else {
            display = temperature % 10;
        }
        displayDigit(digit++, display);
        digit %= 3;
        return;
  }
}

int main(void) {

    setup();

    while(1) {
        
  
    }
   
    return 0;
}
