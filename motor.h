/* 
 * File:   motor.h
 * Author: sean
 *
 * Created on June 21, 2014, 8:15 PM
 */

#ifndef MOTOR_H
#define	MOTOR_H
#include <pic18.h>
#include <htc.h>
#include "config.h"
#include <xc.h>

#ifdef	__cplusplus
extern "C" {
#endif

void stepper_delay(void);

#define ON 1
#define OFF 0

void blue(int status);
void pink(int status);
void yellow(int status);
void orange(int status);
void forward(int steps);
void backward(int steps);
void all_off(void);



#ifdef	__cplusplus
}
#endif

#endif	/* MOTOR_H */

