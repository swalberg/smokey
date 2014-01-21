/*
 * File:   main.c
 * Author: sean
 *
 * Created on August 11, 2013, 1:16 PM
 */


#include <pic18.h>
#include <htc.h>

#define _XTAL_FREQ 20000000

#include <xc.h>

// CONFIG1H
#pragma config OSC = HS         // Oscillator Selection bits (HS oscillator)
#pragma config OSCS = OFF       // Oscillator System Clock Switch Enable bit (Oscillator system clock switch option is disabled (main oscillator is source))

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOR = ON         // Brown-out Reset Enable bit (Brown-out Reset enabled)
#pragma config BORV = 20        // Brown-out Reset Voltage bits (VBOR set to 2.0V)

// CONFIG2H
#pragma config WDT = OFF         // Watchdog Timer Enable bit (WDT enabled)
#pragma config WDTPS = 128      // Watchdog Timer Postscale Select bits (1:128)

// CONFIG3H
#pragma config CCP2MUX = ON     // CCP2 Mux bit (CCP2 input/output is multiplexed with RC1)

// CONFIG4L
#pragma config STVR = ON        // Stack Full/Underflow Reset Enable bit (Stack Full/Underflow will cause RESET)
#pragma config LVP = OFF        // Low Voltage ICSP Enable bit (Low Voltage ICSP enabled)

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000200-001FFFh) not code protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (002000-003FFFh) not code protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (004000-005FFFh) not code protected)
#pragma config CP3 = OFF        // Code Protection bit (Block 3 (006000-007FFFh) not code protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot Block (000000-0001FFh) not code protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000200-001FFFh) not write protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (002000-003FFFh) not write protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (004000-005FFFh) not write protected)
#pragma config WRT3 = OFF       // Write Protection bit (Block 3 (006000-007FFFh) not write protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block (000000-0001FFh) not write protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000200-001FFFh) not protected from Table Reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (002000-003FFFh) not protected from Table Reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (004000-005FFFh) not protected from Table Reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit (Block 3 (006000-007FFFh) not protected from Table Reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot Block (000000-0001FFh) not protected from Table Reads executed in other blocks)


// Globals
int temperature = 0;
int setpoint;
long error;
char mode = 0;
int foo = 100;
char digit;

void spiSlaveDisable(char);
void spiSlaveEnable(char);

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


char readSPIByte() {
    PIR1bits.SSPIF=0; // clear interrupt
    SSPBUF=0x00;
    while (!SSPSTATbits.BF) ; // wait until done
    return (SSPBUF);

}
int readTemperature(void) {
    char b1, b2;
    int tmp;

    spiSlaveEnable(0);
    __delay_ms(1);
    b1 = readSPIByte();
    b2 = readSPIByte();
    __delay_ms(1);
    
    spiSlaveDisable(0);

    // b1[6:0] and b2[7:2] divided by 4
    return (b1 << 5 | b2 >>3) >> 2;
}

void spiSlaveEnable(char device) {
    // enable the MAX6675... Active low
    PORTB &= 0b11111110;
}

void spiSlaveDisable(char device) {
    // Turn off the MAX6675... Active low
    PORTB |= 0b00000001;
}

void enableDigit(char position) {
    char mask = 0b00000001; // E0:2
    mask = mask << position;

    PORTE = mask;
}

void displayDigit(char position, char value) {
    char tmp_port_c;
    char digits[10] = {0x22, 0xaf, 0x31, 0x25, 0xac, 0x64, 0x60, 0x2f, 0x20, 0x24};
    char digit;

    digit = digits[value];
    PORTD = digit;
    
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
