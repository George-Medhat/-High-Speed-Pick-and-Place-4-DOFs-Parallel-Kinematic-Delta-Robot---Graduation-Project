/*
 *
 * High Speed Pick and Place Delta Robot Graduation Project
 *
 */
#ifndef DELAY_H_
#define DELAY_H_

#include "tm4c123gh6pm.h"
#include <stdbool.h>
#include <stdint.h>
#include "driverlib/sysctl.h"

void delayMs(uint32_t ui32Ms);
void delayUs(uint32_t ui32Us);
void delayNs(uint32_t ui32Ns);

#endif /* DELAY_H_ */
