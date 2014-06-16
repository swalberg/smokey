#include <xc.h>

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
