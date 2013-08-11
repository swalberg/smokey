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
#pragma config LVP = OFF        // Low-Voltage (Single-Supply) In-Circuit Serial Programming Enable bit (RB3 is digital I/O, HV on MCLR must be used for programming)
#pragma config CPD = OFF        // Data EEPROM Memory Code Protection bit (Data EEPROM code protection off)
#pragma config WRT = OFF        // Flash Program Memory Write Enable bits (Write protection off; all program memory may be written to by EECON control)
#pragma config CP = OFF         // Flash Program Memory Code Protection bit (Code protection off)

void setup(void) {
    TRISB = 0x00; // PORTB is the 7 segment LED
    /*
     * PORT A   0 - A/D 0
     *          1 - A/D 1
     *          2
     *          3 
     *          4 
     *          5 
     *          6
     *          7
     */
    TRISC = 0b000000000;
}

void displayDigit(char position, char value) {
    char tmp_port_c;
    char digits[10] = {0x22, 0xaf, 0x31, 0x25, 0xac, 0x64, 0x60, 0x2f, 0x20, 0x24};
    char digit;

    char mask = 0b00100000; // C5-7
    mask = mask << position;

    tmp_port_c = PORTC;

    tmp_port_c &= 0b00011100; // all display off, clear led
    tmp_port_c |= mask;       // turn on our light

    digit = digits[value];
    PORTB = (digit >> 2);
    PORTC = (tmp_port_c | (digit & 3));
 }

int main(void) {

    setup();

    while(1) {
        displayDigit(0, 3);
        displayDigit(1, 5);
        displayDigit(2, 0);
    }
   
    return 0;
}
