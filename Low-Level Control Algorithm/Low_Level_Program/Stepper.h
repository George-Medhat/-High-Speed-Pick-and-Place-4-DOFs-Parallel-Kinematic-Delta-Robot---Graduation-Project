/*
 *
 * High Speed Pick and Place Delta Robot Graduation Project
 *
 */
#ifndef STEPPER_H_
#define STEPPER_H_

#include "globalVariables.h"


void Move_double(float steps, float speed11, float accel11, int first,
                 float steps2, float speed22, float accel22, float steps3,
                 float speed33, float accel33);
void home();

#endif /* STEPPER_H_ */
