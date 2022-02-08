/*
 *
 * High Speed Pick and Place Delta Robot Graduation Project
 *
 */
#ifndef SERVO_H_
#define SERVO_H_

#include "globalVariables.h"

void servoActuation(uint32_t degrees_1, uint32_t degrees_2);
void PWM_DividerGet(void);
void pulseDurationComputation(void);

#endif /* SERVO_H_ */
