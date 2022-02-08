/*
 *
 * High Speed Pick and Place Delta Robot Graduation Project
 *
 */
#include "globalVariables.h"
#include "I2C_Slave.h"
#include "initialization.h"
#include "servo.h"
#include "Stepper.h"



void main()
{

    SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ); // 80MHz
    SysCtlPWMClockSet(SYSCTL_PWMDIV_64);
    PWM_DividerGet();
    Ports_Init();
//    while(1){
//    GPIOPinWrite(GPIO_PORTD_BASE,MS1,MS1);
//    GPIOPinWrite(GPIO_PORTD_BASE,MS2,MS2);
//    GPIOPinWrite(GPIO_PORTD_BASE,MS3,MS3);
//    delayMs(1000);
//    GPIOPinWrite(GPIO_PORTD_BASE,MS1,0);
//    GPIOPinWrite(GPIO_PORTD_BASE,MS2,0);
//    GPIOPinWrite(GPIO_PORTD_BASE,MS3,0);
//    delayMs(1000);
//    }
    init_PWM();
    pulseDurationComputation();
    InitI2C0();
    Timer_Init();
    GPIOPinWrite(GPIO_PORTD_BASE,solenoid_1,0);
    servoActuation(48, 0);
    //microStepInit();
    microStep =8;
    home();

    while (1)
    {

        I2CReceive(slave_address);
        servoActuation(servo_angle,0);

        float step11 = 0;
        float speed11 = 0;
        float accel11 = 0;
        float step22 = 0;
        float speed22 = 0;
        float accel22 = 0;
        float step33 = 0;
        float speed33 = 0;
        float accel33 = 0;


        for (path_iterator = 0; path_iterator < TimeSlice; path_iterator++)
        {
            step11 = path1[path_iterator][0];
            speed11 = path1[path_iterator][1] * ((gearRatio * microStep) / AnglerPerStep);
            accel11 = path1[path_iterator][2] * ((gearRatio * microStep) / AnglerPerStep);

            step22 = path2[path_iterator][0];
            speed22 = path2[path_iterator][1] * ((gearRatio * microStep) / AnglerPerStep);
            accel22 = path2[path_iterator][2] * ((gearRatio * microStep) / AnglerPerStep);

            step33 = path3[path_iterator][0];
            speed33 = path3[path_iterator][1] * ((gearRatio * microStep) / AnglerPerStep);
            accel33 = path3[path_iterator][2] * ((gearRatio * microStep) / AnglerPerStep);

            if (path_iterator == 0)
            {
                first = 1; // used to know if this is the first time slice or not

            }
            else
            {
                first = 0;

            }

            if (path_iterator == suctionStartIndex){
               GPIOPinWrite(GPIO_PORTD_BASE,solenoid_1,solenoid_1);
            }
            if (path_iterator == (suctionStartIndex+5)){
                suctionStartIndex = 255;
            }

            if (path_iterator == servoFirstIndex && I2C_servo_first == 0){
                servoActuation(45,0);
                I2C_servo_first = 1;
            }
            if (path_iterator == suctionStopIndex - 1 ){
                GPIOPinWrite(GPIO_PORTD_BASE,solenoid_1,0);
                suctionStopIndex = 255;
            }


            Move_double(step11, speed11, accel11, first, step22, speed22,
                        accel22, step33, speed33, accel33);
        }


        suctionStartIndex=0;
    }
}

