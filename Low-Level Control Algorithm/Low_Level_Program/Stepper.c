/*
 *
 * High Speed Pick and Place Delta Robot Graduation Project
 *
 */
#include "Stepper.h"

/*
 *
 * The Move_double Function is responsible for the robot's motion.
 * It takes number of steps, speed, and acceleration for each motor,
 * this data was sent from the path planning algorithm.
 * The function deduce the direction of the motor based on the old absolute steps
 * sent and the current steps.
 * It controls all the motor simultaneously
 *
*/
void Move_double(float steps, float speed11, float accel11, int first,
                 float steps2, float speed22, float accel22, float steps3,
                 float speed33, float accel33)
{


    int dir;
    int dir2;
    int dir3;
    if (first == 1)
    {
        oldSpeed1 = 0;
        requiredSpeed1 = 0;
        oldAccel1=accel11;
        oldSpeed2 = 0;
        requiredSpeed2 = 0;
        oldAccel2=accel22;
        oldSpeed3 = 0;
        requiredSpeed3 = 0;
        oldAccel3=accel33;
        steps1ToGo = 0;
        steps2ToGo = 0;
        steps3ToGo = 0;
        oldstep = steps;
        oldstep2 = steps2;
        oldstep3 = steps3;
        motor1globalsteps = steps;
        motor2globalsteps = steps2;
        motor3globalsteps = steps3;
        return;
    }

    if (first == 0)
    {
        if (steps > oldstep)
        {
            steps1ToGo = steps - motor1globalsteps;
            dir = Dir_Pin1;
        }else if(steps == oldstep){
            steps1ToGo = 0;
        }
        else
        {
             steps1ToGo = fabs(steps - motor1globalsteps);
            dir = 0;
        }
        if (steps2 > oldstep2)
        {
            steps2ToGo = steps2 - motor2globalsteps;
            dir2 = Dir_Pin2;
        }else if(steps2 == oldstep2){
            steps2ToGo = 0;
        }
        else
        {
            steps2ToGo = fabs( steps2 - motor2globalsteps);
            dir2 = 0;
        }
        if (steps3 > oldstep3)
        {
            steps3ToGo = steps3 - motor3globalsteps;
            dir3 = Dir_Pin3;
        }else if(steps3 == oldstep3){
            steps3ToGo = 0;
        }
        else
        {
            steps3ToGo = fabs( steps3 - motor3globalsteps);
            dir3 = 0;
        }
    }

    GPIOPinWrite(GPIO_PORTA_BASE, Dir_Pin3, dir3);
    GPIOPinWrite(GPIO_PORTE_BASE, Dir_Pin2, dir2);
    GPIOPinWrite(GPIO_PORTB_BASE, Dir_Pin1, dir);

    delayUs(5); // 5 microseconds delay for setting the direction

    if(speed11 <= 10e-5 && speed11 >= -10e-5){
        speed11 = 200;
    }
    if(speed22 <= 10e-5 && speed22 >= -10e-5){
        speed22 = 200;
    }
    if(speed33 <= 10e-5 && speed33 >= -10e-5){

        speed33 = 200;
    }

    while (steps1ToGo >= 1  || steps2ToGo >= 1  || (steps3ToGo >= 1))
    {
        if (steps1ToGo >= 1)
        {
            if (falling_edge1 == 1)
            {
                falling_edge1 = 0;
                GPIOPinWrite(GPIO_PORTB_BASE, Step_Pin1, Step_Pin1);
                TimerEnable(TIMER5_BASE, TIMER_A);
                steps1ToGo--;
                if (dir == Dir_Pin1)
                {
                    motor1globalsteps++;
                }
                else
                {
                    motor1globalsteps--;
                }
            }
            if (rising_edge1 == 1)
            {
                rising_edge1 = 0;

                if (steps1ToGo > 0)
                    if (accel11 < oldAccel1)
                    {
                        requiredSpeed1 = oldSpeed1 - fabs(accel11 / oldSpeed1);
                        accelerating1 = 0;
                        oldAccel1= accel11;
                    }
                    else
                    {
                        oldAccel1= accel11;
                        accelerating1 = 1;
                        if (oldSpeed1 == 0)
                        {
                            requiredSpeed1 = sqrt(2.0 * accel11);
                        }
                        else
                        {
                            requiredSpeed1 = oldSpeed1+ fabs(accel11 / oldSpeed1);

                        }
                    }
                if (requiredSpeed1 > speed11 & accelerating1 == 1)
                    requiredSpeed1 = speed11;
                if (requiredSpeed1 < speed11 & accelerating1 == 0)
                    requiredSpeed1 = speed11;

                stepInterval1 = 1000000 / requiredSpeed1;
                stepInterval1 = stepInterval1
                        * (SysCtlClockGet() / 3 / 1000000);
                oldSpeed1 = requiredSpeed1;

                TimerLoadSet(TIMER4_BASE, TIMER_A, stepInterval1);
                TimerEnable(TIMER4_BASE, TIMER_A);
            }
        }

        if (steps2ToGo >= 1)
        {

            if (falling_edge2 == 1)
            {
                falling_edge2 = 0;
                GPIOPinWrite(GPIO_PORTE_BASE, Step_Pin2, Step_Pin2);
                TimerEnable(TIMER3_BASE, TIMER_A);
                steps2ToGo--;
                if (dir2 == Dir_Pin2)
                {
                    motor2globalsteps++;
                }
                else
                {
                    motor2globalsteps--;
                }
            }
            if (rising_edge2 == 1)
            {
                rising_edge2 = 0;

                if (steps2ToGo > 0)
                    if (accel22 < oldAccel2)
                    {
                        requiredSpeed2 = oldSpeed2 - fabs(accel22 / oldSpeed2);
                        accelerating2 = 0;
                        oldAccel2= accel22;
                    }
                    else
                    {
                        oldAccel2= accel22;
                        accelerating2 = 1;
                        if (oldSpeed2 == 0)
                        {
                            requiredSpeed2 = sqrt(2.0 * accel22);
                        }
                        else
                        {
                            requiredSpeed2 = oldSpeed2+ fabs(accel22 / oldSpeed2);

                        }
                    }
                if (requiredSpeed2 > speed22 & accelerating2 == 1)
                    requiredSpeed2 = speed22;
                if (requiredSpeed2 < speed22 & accelerating2 == 0)
                    requiredSpeed2 = speed22;

                stepInterval2 = 1000000 / requiredSpeed2;
                stepInterval2 = stepInterval2
                        * (SysCtlClockGet() / 3 / 1000000);
                oldSpeed2 = requiredSpeed2;
                TimerLoadSet(TIMER2_BASE, TIMER_A, stepInterval2);
                TimerEnable(TIMER2_BASE, TIMER_A);
            }
        }

        if (steps3ToGo >= 1)
        {

            if (falling_edge3 == 1)
            {
                falling_edge3 = 0;
                GPIOPinWrite(GPIO_PORTA_BASE, Step_Pin3, Step_Pin3);
                TimerEnable(TIMER1_BASE, TIMER_A);
                steps3ToGo--;
                if (dir3 == Dir_Pin3)
                {
                    motor3globalsteps++;
                }
                else
                {
                    motor3globalsteps--;
                }
            }
            if (rising_edge3 == 1)
            {
                rising_edge3 = 0;

                if (steps3ToGo > 0)
                    if (accel33 < oldAccel3)
                    {
                        requiredSpeed3 = oldSpeed3 - fabs(accel33 / oldSpeed3);
                        accelerating3 = 0;
                        oldAccel3= accel33;
                    }
                    else
                    {
                        oldAccel3=accel33;
                        accelerating3 = 1;
                        if (oldSpeed3 == 0)
                        {
                            requiredSpeed3 = sqrt(2.0 * accel33);
                        }
                        else
                        {
                            requiredSpeed3 = oldSpeed3+ fabs(accel33 / oldSpeed3);

                        }
                    }
                if (requiredSpeed3 > speed33 & accelerating3 == 1)
                    requiredSpeed3 = speed33;
                if (requiredSpeed3 < speed33 & accelerating3 == 0)
                    requiredSpeed3 = speed33;

                stepInterval3 = 1000000 / requiredSpeed3;
                stepInterval3 = stepInterval3
                        * (SysCtlClockGet() / 3 / 1000000);
                oldSpeed3 = requiredSpeed3;
                TimerLoadSet(TIMER0_BASE, TIMER_A, stepInterval3);
                TimerEnable(TIMER0_BASE, TIMER_A);
            }
        }

    }
oldstep=steps;
oldstep2 = steps2;
oldstep3 = steps3;
}

/*
 *
 * The home function moves all the motors upward step by step while
 * checking the state of each limit switch until they are all pressed on
 * then the function move the end effector to a virtual origin 10cm below
 * the home position
 *
 */

void home(){
    GPIOPinWrite(GPIO_PORTA_BASE, Dir_Pin3, 0);    // setting direction to upward to home the Robot
    GPIOPinWrite(GPIO_PORTE_BASE, Dir_Pin2, 0);
    GPIOPinWrite(GPIO_PORTB_BASE, Dir_Pin1, 0);

    delayUs(5);
    int volatile x = 0;
    int volatile y = 0;
    int volatile z = 0;
    x = GPIOPinRead(GPIO_PORTC_BASE, limit1);
    y = GPIOPinRead(GPIO_PORTC_BASE, limit2);
    z = GPIOPinRead(GPIO_PORTC_BASE, limit3);

    while (GPIOPinRead(GPIO_PORTC_BASE, limit1) || GPIOPinRead(GPIO_PORTC_BASE, limit2) || GPIOPinRead(GPIO_PORTC_BASE, limit3))
    {
        x = GPIOPinRead(GPIO_PORTC_BASE, limit1);
        y = GPIOPinRead(GPIO_PORTC_BASE, limit2);
        z = GPIOPinRead(GPIO_PORTC_BASE, limit3);
        if (GPIOPinRead(GPIO_PORTC_BASE, limit1))
        {
            if (falling_edge1 == 1)
            {
                falling_edge1 = 0;
                GPIOPinWrite(GPIO_PORTB_BASE, Step_Pin1, Step_Pin1);
                TimerEnable(TIMER5_BASE, TIMER_A);
            }
            if (rising_edge1 == 1)
            {
                rising_edge1 = 0;
                TimerLoadSet(TIMER4_BASE, TIMER_A, 312 * microStep * (80 / 3));
                TimerEnable(TIMER4_BASE, TIMER_A);

            }

        }
        if (GPIOPinRead(GPIO_PORTC_BASE, limit2))
        {
            if (falling_edge2 == 1)
            {
                falling_edge2 = 0;
                GPIOPinWrite(GPIO_PORTE_BASE, Step_Pin2, Step_Pin2);
                TimerEnable(TIMER3_BASE, TIMER_A);
            }
            if (rising_edge2 == 1)
            {
                rising_edge2 = 0;
                TimerLoadSet(TIMER2_BASE, TIMER_A, 312 * microStep * (80 / 3));
                TimerEnable(TIMER2_BASE, TIMER_A);

            }
        }

        if (GPIOPinRead(GPIO_PORTC_BASE, limit3))
        {
            if (falling_edge3 == 1)
            {
                falling_edge3 = 0;
                GPIOPinWrite(GPIO_PORTA_BASE, Step_Pin3, Step_Pin3);
                TimerEnable(TIMER1_BASE, TIMER_A);
            }
            if (rising_edge3 == 1)
            {
                rising_edge3 = 0;
                TimerLoadSet(TIMER0_BASE, TIMER_A,  312 * microStep * (80 / 3));
                TimerEnable(TIMER0_BASE, TIMER_A);

            }
        }
    }


    float step11 = 0;
    float speed11 = 0;
    float accel11 = 0;
    float step22 = 0;
    float speed22 = 0;
    float accel22 = 0;
    float step33 = 0;
    float speed33 = 0;
    float accel33 = 0;
    int i = 0;

    for (i = 0; i < HomingTimeSlice - 1; i++)
    {
        step11 = 5 * homingPath[i][0] * (microStep / AnglerPerStep);
        speed11 = 5 * fabs(homingPath[i][1] * (microStep / AnglerPerStep));
        accel11 = 5 * fabs(homingPath[i][2] * (microStep / AnglerPerStep));

        step22 = 5 * homingPath[i][0] * (microStep / AnglerPerStep);
        speed22 = 5 * fabs(homingPath[i][1] * (microStep / AnglerPerStep));
        accel22 = 5 * fabs(homingPath[i][2] * (microStep / AnglerPerStep));

        step33 = 5 * homingPath[i][0] * (microStep / AnglerPerStep);
        speed33 = 5 * fabs(homingPath[i][1] * (microStep / AnglerPerStep));
        accel33 = 5 * fabs(homingPath[i][2] * (microStep / AnglerPerStep));

        if (i == 0)
        {
            first = 1; // used to know if this is the first time slice or not

        }
        else
        {
            first = 0;

        }
        Move_double(step11, speed11, accel11, first, step22, speed22, accel22,
                    step33, speed33, accel33);
    }
}

