#include "motor.h"

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
void all_off(void) {
    blue(OFF);
    pink(OFF);
    yellow(OFF);
    orange(OFF);
    
}
void forward(int steps) {
    int i;
    
    if (steps == 0) { return; }

    for (i=0; i < steps; i++) {
        blue(ON);
        pink(ON);
        yellow(OFF);
        orange(OFF);

        stepper_delay();

        blue(OFF);
        pink(ON);
        yellow(ON);
        orange(OFF);

        stepper_delay();

        blue(OFF);
        pink(OFF);
        yellow(ON);
        orange(ON);

        stepper_delay();

        blue(ON);
        pink(OFF);
        yellow(OFF);
        orange(ON);

        stepper_delay();
    }

}

void backward(int steps) {
    int i;

    if (steps == 0) { return; }

    for (i=0; i < steps; i++) {
        blue(ON);
        pink(OFF);
        yellow(OFF);
        orange(ON);

        stepper_delay();

        blue(OFF);
        pink(OFF);
        yellow(ON);
        orange(ON);

        stepper_delay();

        blue(OFF);
        pink(ON);
        yellow(ON);
        orange(OFF);

        stepper_delay();

        blue(ON);
        pink(ON);
        yellow(OFF);
        orange(OFF);

        stepper_delay();

    }

}

