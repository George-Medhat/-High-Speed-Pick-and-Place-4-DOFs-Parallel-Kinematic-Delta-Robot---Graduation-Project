/*
 *
 * High Speed Pick and Place Delta Robot Graduation Project
 *
 */
#include "initialization.h"

        int time1 = 2.5 * 80 / 3;  // 2.5 micro second
        int time2;
        int time3 = 2.5 * 80 / 3;
        int time4;
        int time5 = 2.5 * 80 / 3;
        int time6;

        float steps1ToGo=0;
        float steps2ToGo=0;
        float steps3ToGo=0;

        volatile float oldstep=0;
        volatile float oldstep2=0;
        volatile float oldstep3=0;

        volatile float stepInterval1;
        volatile float stepInterval2;
        volatile float stepInterval3;
        volatile float requiredSpeed1;
        volatile float requiredSpeed2;
        volatile float requiredSpeed3;

        float Speed1, Speed2, Speed3, Accel1, Accel2, Accel3;

        int accelerating1 = 1;
        int accelerating2 = 1;
        int accelerating3 = 1;
        float oldSpeed = 0;
        float oldSpeed1 = 0;
        float oldSpeed2 = 0;
        float oldSpeed3 = 0;
        float oldAccel1 =0;
        float oldAccel2 =0;
        float oldAccel3 =0;
        float motor1globalsteps = 0;
        float motor2globalsteps = 0;
        float motor3globalsteps = 0;

        int rising_edge1 = 0;
        int falling_edge1 = 1;

        int rising_edge2 = 0;
        int falling_edge2 = 1;

        int rising_edge3 = 0;
        int falling_edge3 = 1;

        int path_iterator = 0;

        int first = 1;
        uint32_t TimeSlice = 0;
        volatile int suctionStartIndex = 0;
        volatile int suctionStopIndex = 0;
        volatile int nextIndex = 0;
        volatile int servoFirstIndex = 0;
        volatile int I2C_servo_first = 0;

        volatile int microStep = 8;

        float homingPath[HomingTimeSlice][3]={
                                                  {0.0,0.0,0.0},
                                                  {0.025710375000000008,1.5026625000000002,56.943000000000005},
                                                  {0.19003200000000006,5.394600000000001,95.90400000000001},
                                                  {0.5907836250000003,10.826662500000003,118.88100000000001},
                                                  {1.2858240000000003,17.0496,127.87200000000001},
                                                  {2.298046875,23.4140625,124.875},
                                                  {3.620376000000002,29.370600000000003,111.88800000000002},
                                                  {5.220760125,34.46966250000001,90.90899999999996},
                                                  {7.047168000000003,38.361599999999996,63.93599999999998},
                                                  {9.032583375000002,40.7966625,32.966999999999956},
                                                  {11.1,41.625,0.0},
                                                  {13.167416625000005,40.79666249999998,-32.96700000000004},
                                                  {15.152832000000007,38.36159999999997,-63.93599999999992},
                                                  {16.979239874999994,34.46966250000007,-90.9090000000001},
                                                  {18.579623999999995,29.370600000000053,-111.88800000000015},
                                                  {19.901953125,23.4140625,-124.875},
                                                  {20.914176000000026,17.04959999999994,-127.87200000000007},
                                                  {21.60921637499996,10.826662500000054,-118.88100000000009},
                                                  {22.009968000000015,5.394599999999912,-95.904},
                                                  {22.174289625000014,1.5026624999999285,-56.942999999999756},
                                                  {22.19999999999999,0.0,0.0}
            };


void InitI2C0(void)
{
    //enable I2C module 0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_I2C0))
    {

    }
    //reset module
    SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0);

    // Configure the pin muxing for I2C0 functions on port B2 and B3.
    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);

    // Select the I2C function for these pins.
    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);
    //GPIOPadConfigSet(GPIO_PORTB_BASE, GPIO_PIN_2 | GPIO_PIN_3, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD_WPU);

    // Enable and initialize the I2C0 master module.  Use the system clock for
    // the I2C0 module.  The last parameter sets the I2C data transfer rate.
    // If false the data rate is set to 100kbps and if true the data rate will
    // be set to 400kbps.
    I2CSlaveIntEnableEx(I2C0_BASE, I2C_SLAVE_INT_DATA);
    I2CSlaveInit(I2C0_BASE, slave_address);

    //clear I2C FIFOs
    HWREG(I2C0_BASE + I2C_O_FIFOCTL) = 80008000;
}

void init_PWM(void){

    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM1)){}
    PWMGenDisable(PWM1_BASE, PWM_GEN_0);
    PWMClock = SysCtlClockGet() / PWM_Divider;                              // Divide the PWM clock by the desired frequency (50Hz) to determine the count to be loaded into the Load register.
    load = (PWMClock / PWM_Frequency) - 1;                        // Then subtract 1 since the counter down-counts to zero.
    PWMGenConfigure(PWM1_BASE, PWM_GEN_0, (PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC));
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_0, load);
    PWMGenEnable(PWM1_BASE, PWM_GEN_0);
    PWMOutputState(PWM1_BASE, PWM_OUT_0_BIT, true);
}

void Ports_Init(void)
{
    // LED to indicate transmission in process
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF))
    {

    }
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2|GPIO_PIN_1|GPIO_PIN_3);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    while (!(SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA)))
    {
    }
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    while (!(SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD)))
    {
    }

    GPIOPinConfigure(servoMotor_D0_PWM);
    GPIOPinTypePWM(GPIO_PORTD_BASE, servoMotor_D0);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    while (!(SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB)))
    {
    }
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    while (!(SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE)))
    {
    }
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, Dir_Pin3 | Step_Pin3);

    GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, solenoid_1);
    GPIOPinTypeGPIOInput(GPIO_PORTE_BASE,servo_Motion_Ready |servo_first_time);
    GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE,transmission_Ready);
    GPIOPinWrite(GPIO_PORTE_BASE,transmission_Ready,0);
    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= MS2;
    GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, MS1|MS2|MS3);
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, Dir_Pin1 | Step_Pin1);
    GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, Dir_Pin2 | Step_Pin2);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    while (!(SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOC))){}
    GPIOPinTypeGPIOInput (GPIO_PORTC_BASE, limit1| limit2| limit3);

}

void Interrupt_Handler1(void)
{
    GPIOPinWrite(GPIO_PORTB_BASE, Step_Pin1, 0);
    rising_edge1 = 1;
    TimerIntClear(TIMER5_BASE, TIMER_A);

}
void Interrupt_Handler2(void)
{
    falling_edge1 = 1;
    TimerIntClear(TIMER4_BASE, TIMER_A);
}
void Interrupt_Handler3(void)
{

    GPIOPinWrite(GPIO_PORTE_BASE, Step_Pin2, 0);
    rising_edge2 = 1;
    TimerIntClear(TIMER3_BASE, TIMER_A);

}

void Interrupt_Handler4(void)
{
    falling_edge2 = 1;
    TimerIntClear(TIMER2_BASE, TIMER_A);

}

void Interrupt_Handler5(void)
{

    GPIOPinWrite(GPIO_PORTA_BASE, Step_Pin3, 0);
    rising_edge3 = 1;
    TimerIntClear(TIMER1_BASE, TIMER_A);

}

void Interrupt_Handler6(void)
{

    falling_edge3 = 1;
    TimerIntClear(TIMER0_BASE, TIMER_A);

}
void Timer_Init(void){

       SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER5);
       SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER4);
       SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3);
       SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
       SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
       SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);

       while (!(SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER5)))
       {
       }
       while (!(SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER4)))
       {
       }
       while (!(SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER3)))
       {
       }
       while (!(SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER2)))
       {
       }
       while (!(SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER1)))
       {
       }
       while (!(SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER0)))
       {
       }

       TimerDisable(TIMER5_BASE, TIMER_A);
       TimerDisable(TIMER4_BASE, TIMER_A);
       TimerDisable(TIMER3_BASE, TIMER_A);
       TimerDisable(TIMER2_BASE, TIMER_A);
       TimerDisable(TIMER1_BASE, TIMER_A);
       TimerDisable(TIMER0_BASE, TIMER_A);

       TimerConfigure(TIMER5_BASE, TIMER_CFG_ONE_SHOT);
       TimerConfigure(TIMER4_BASE, TIMER_CFG_ONE_SHOT);
       TimerConfigure(TIMER3_BASE, TIMER_CFG_ONE_SHOT);
       TimerConfigure(TIMER2_BASE, TIMER_CFG_ONE_SHOT);
       TimerConfigure(TIMER1_BASE, TIMER_CFG_ONE_SHOT);
       TimerConfigure(TIMER0_BASE, TIMER_CFG_ONE_SHOT);

       TimerIntClear(TIMER5_BASE, TIMER_A);
       TimerIntClear(TIMER4_BASE, TIMER_A);
       TimerIntClear(TIMER3_BASE, TIMER_A);
       TimerIntClear(TIMER2_BASE, TIMER_A);
       TimerIntClear(TIMER1_BASE, TIMER_A);
       TimerIntClear(TIMER0_BASE, TIMER_A);

       TimerLoadSet(TIMER5_BASE, TIMER_A, time1 - 1);
       TimerLoadSet(TIMER4_BASE, TIMER_A, time2 - 1);
       TimerLoadSet(TIMER3_BASE, TIMER_A, time3 - 1);
       TimerLoadSet(TIMER2_BASE, TIMER_A, time4 - 1);
       TimerLoadSet(TIMER1_BASE, TIMER_A, time5 - 1);
       TimerLoadSet(TIMER0_BASE, TIMER_A, time6 - 1);

       TimerIntRegister(TIMER5_BASE, TIMER_A, Interrupt_Handler1);
       TimerIntRegister(TIMER4_BASE, TIMER_A, Interrupt_Handler2);
       TimerIntRegister(TIMER3_BASE, TIMER_A, Interrupt_Handler3);
       TimerIntRegister(TIMER2_BASE, TIMER_A, Interrupt_Handler4);
       TimerIntRegister(TIMER1_BASE, TIMER_A, Interrupt_Handler5);
       TimerIntRegister(TIMER0_BASE, TIMER_A, Interrupt_Handler6);

       TimerIntEnable(TIMER5_BASE, TIMER_TIMA_TIMEOUT);
       TimerIntEnable(TIMER4_BASE, TIMER_TIMA_TIMEOUT);
       TimerIntEnable(TIMER3_BASE, TIMER_TIMA_TIMEOUT);
       TimerIntEnable(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
       TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
       TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);


}
void microStepInit(void)
{
    volatile int MS_1 = (GPIOPinRead(GPIO_PORTD_BASE, MS1) == MS1);
    volatile int MS_2 = (GPIOPinRead(GPIO_PORTD_BASE, MS2) == MS2);
    volatile int MS_3 = (GPIOPinRead(GPIO_PORTD_BASE, MS3) == MS3);

    if (!MS_1 & !MS_2 & !MS_3)              // Full Step    - 300 Steps in Motor Driver
        microStep = 1;
    else if (!MS_1 & !MS_2 & MS_3)          // Half Step    - 600 Steps in Motor Driver
        microStep = 2;
    else if (!MS_1 & MS_2 & !MS_3)          // 1/4 Step     - 1200 Steps in Motor Driver
        microStep = 4;
    else if (!MS_1 & MS_2 & MS_3)           // 1/8 Step     - 2400 Steps in Motor Driver
        microStep = 8;
    else if (MS_1 & !MS_2 & !MS_3)          // 1/16 Step    - 4800 Steps in Motor Driver
        microStep = 16;
    else if (MS_1 & !MS_2 & MS_3)           // 1/32 Step    - 9600 Steps in Motor Driver
        microStep = 32;
    else if (MS_1 & MS_2 & !MS_3)           // 1/64 Step    - 19200 Steps in Motor Driver
        microStep = 64;
    else                                    // 1/128 Step   - 38400 Steps in Motor Driver
        microStep = 128;
    /*else if (MS_1 & MS_2 & !MS_3)             // 1/32 Step    - 9600 Steps in Motor Driver
    {
        microStep = 32;
        // Do Something Extra or else here
    }
    else                                        // 1/32 Step   - 9600 Steps in Motor Driver
    {
        microStep = 32;
        // Do Something Extra or else here
    }
    */
}
