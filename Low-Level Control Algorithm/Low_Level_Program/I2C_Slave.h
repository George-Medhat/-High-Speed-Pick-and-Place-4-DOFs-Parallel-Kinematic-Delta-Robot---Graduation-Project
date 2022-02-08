/*
 *
 * High Speed Pick and Place Delta Robot Graduation Project
 *
 */
#ifndef I2C_SLAVE_H_
#define I2C_SLAVE_H_


#include "globalVariables.h"



int theta_mergingData(volatile uint8_t * theta_Data);
int vel_acc_mergingData(volatile uint8_t * vel_acc_Data);
void I2CReceive(uint8_t slave_addr);

#endif /* I2C_SLAVE_H_ */
