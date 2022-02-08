/*
 *
 * High Speed Pick and Place Delta Robot Graduation Project
 *
 */
#ifndef INITIALIZATION_H_
#define INITIALIZATION_H_

#include "globalVariables.h"



void InitI2C0(void);
void init_PWM(void);
void Ports_Init(void);
void Timer_Init(void);
void Interrupt_Handler1(void);
void Interrupt_Handler2(void);
void Interrupt_Handler3(void);
void Interrupt_Handler4(void);
void Interrupt_Handler5(void);
void Interrupt_Handler6(void);
void microStepInit(void);


#endif /* INITIALIZATION_H_ */
