#include <xc.h>
#include "config.h"
#include "spi.h"

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

