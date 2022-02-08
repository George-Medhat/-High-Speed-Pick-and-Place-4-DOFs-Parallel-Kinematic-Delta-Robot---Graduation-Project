/*
 *
 * High Speed Pick and Place Delta Robot Graduation Project
 *
 */
#include "I2C_Slave.h"

int theta_mergingData(volatile uint8_t * theta_Data)
{
    int32_t temp = 0;
    temp = theta_Data[0];
    temp += (theta_Data[1] << 8);

    if (theta_Data[2] == 1)
    {
        return (temp * -1);
    }
    else
    {
        return temp;
    }
}



int vel_acc_mergingData(volatile uint8_t * vel_acc_Data)
{
    int32_t temp_intPart = 0;
    temp_intPart = vel_acc_Data[0];
    temp_intPart += (vel_acc_Data[1] << 8);
    return temp_intPart;
}
void I2CReceive(uint8_t slave_addr)
{
    uint32_t i = 0;
    uint32_t j = 0;
    uint32_t k = 0;
    uint32_t l = 0;
    GPIOPinWrite(GPIO_PORTE_BASE,transmission_Ready,transmission_Ready);
    while (!(I2CSlaveStatus(I2C0_BASE) & I2C_SLAVE_ACT_RREQ))
    {

    }

    I2CSlaveIntClear(I2C0_BASE);
    TimeSlice = I2CSlaveDataGet(I2C0_BASE);
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);

    while (!(I2CSlaveStatus(I2C0_BASE) & I2C_SLAVE_ACT_RREQ))
    {

    }

    I2CSlaveIntClear(I2C0_BASE);
    servo_angle = I2CSlaveDataGet(I2C0_BASE);

    for (i = 0; i < 3; i++)
    {
        for (j = 0; j <= (TimeSlice - 1); j++)
        {
            for (k = 0; k < 3; k++)
            {
                if (k == 0)
                {
                    for (l = 0; l < 3; l++)
                    {
                        while (!(I2CSlaveStatus(I2C0_BASE) & I2C_SLAVE_ACT_RREQ))
                        {

                        }

                        I2CSlaveIntClear(I2C0_BASE);
                        theta_mergedData[l] = I2CSlaveDataGet(I2C0_BASE);
                    }
                }

                if (k == 1)
                {
                    for (l = 0; l < 2; l++)
                    {
                        while (!(I2CSlaveStatus(I2C0_BASE) & I2C_SLAVE_ACT_RREQ))
                        {

                        }

                        I2CSlaveIntClear(I2C0_BASE);
                        vel_acc_mergedData[l] = I2CSlaveDataGet(I2C0_BASE);
                    }
                }
                if (k == 2)
                {
                    for (l = 0; l < 2; l++)
                    {
                        while (!(I2CSlaveStatus(I2C0_BASE) & I2C_SLAVE_ACT_RREQ))
                        {

                        }

                        I2CSlaveIntClear(I2C0_BASE);
                        vel_acc_mergedData[l] = I2CSlaveDataGet(I2C0_BASE);
                    }
                }

                if (i == 0)
                {
                    if (k == 0)
                    {
                        path1[j][k] = theta_mergingData(theta_mergedData);
                    }
                    else
                    {
                        path1[j][k] = vel_acc_mergingData(vel_acc_mergedData);
                    }
                }
                else if (i == 1)
                {
                    if (k == 0)
                    {
                        path2[j][k] = theta_mergingData(theta_mergedData);
                    }
                    else
                    {
                        path2[j][k] = vel_acc_mergingData(vel_acc_mergedData);
                    }
                }
                else if (i == 2)
                {
                    if (k == 0)
                    {
                        path3[j][k] = theta_mergingData(theta_mergedData);
                    }
                    else
                    {
                        path3[j][k] = vel_acc_mergingData(vel_acc_mergedData);
                    }
                }
            }

            if (i==0){
                if (GPIOPinRead(GPIO_PORTE_BASE,servo_Motion_Ready)==0 && suctionStartIndex == 0){
                    suctionStartIndex = j;
                    nextIndex = 1 ;

                }
                if (!GPIOPinRead(GPIO_PORTE_BASE,servo_Motion_Ready)==0 && nextIndex == 1){
                    suctionStopIndex = j;
                    nextIndex = 0 ;
                }
                if (GPIOPinRead(GPIO_PORTE_BASE, servo_first_time) == 0 && I2C_servo_first == 0){
                    servoFirstIndex = j;
                }
            }

    }}
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);
    GPIOPinWrite(GPIO_PORTE_BASE,transmission_Ready,0);

}
