/*
 *
 * High Speed Pick and Place Delta Robot Graduation Project
 *
 */

#ifndef GLOBALVARIABLES_H_
#define GLOBALVARIABLES_H_


#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "inc/hw_types.h"
#include "inc/hw_i2c.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/gpio.h"
#include "driverlib/i2c.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#include "driverlib/pwm.h"
#include "delay.h"

// Constants
#define gearRatio          5
#define AnglerPerStep      1.2         // 1.8 for nema 17 / 1.2 for nema 34

#define Dir_Pin1     0x00000020    // Motor_1_Pin_5_Port_B
#define Step_Pin1    0x00000001    // Motor_1_Pin_0_Port_B

#define Dir_Pin2     0x00000010    // Motor_2_Pin_4_Port_E
#define Step_Pin2    0x00000020    // Motor_2_Pin_5_Port_E

#define Dir_Pin3     0x00000040    // Motor_3_Pin_6_Port_A
#define Step_Pin3    0x00000080    // Motor_3_Pin_7_Port_A


#define limit1       GPIO_PIN_4     // Port_C Pin 4 M1 limit switch
#define limit2       GPIO_PIN_5     // Port_C Pin 5 M2 limit switch
#define limit3       GPIO_PIN_6     // Port_C Pin 6 M3 limit switch

#define MS1                 GPIO_PIN_6
#define MS2                 GPIO_PIN_7
#define MS3                 GPIO_PIN_3  //port d

#define solenoid_1                      GPIO_PIN_2 // port  D second pin the first pin is 1 -

#define transmission_Ready              GPIO_PIN_1
#define servo_first_time                GPIO_PIN_2
#define servo_Motion_Ready              GPIO_PIN_3 // port E


// Servo Motor Data
#define servoMotor_D0                   GPIO_PIN_0
#define servoMotor_D0_PWM               GPIO_PD0_M1PWM0
#define servoMotor_D0_Output            PWM_OUT_0_BIT
#define servoMotor_B4                   GPIO_PIN_4
#define servoMotor_B4_PWM               GPIO_PB4_M0PWM2
#define servoMotor_B4_Output            PWM_OUT_4_BIT


#define PWM_Frequency 50.0                        // Frequency designed for Servo motors for 20ms pulse duration
volatile uint32_t load;                           // Load value to be calculated to specify the 20ms
volatile uint32_t PWMClock;                       // Used to calculate the load value
volatile uint32_t PWM_Divider;                    // Division value used for PWM

#define servoMotor_ZeroPostionPulseWidthLimit 0.5                                   // 0.5ms High Pulse required to actuate the Servo to 0 degrees
#define servoMotor_MaxPostionPulseWidthLimit  2.5                                   // 2.5ms High Pulse required to actuate the Servo to 180 degrees
#define resolution 1800.0                                                           // Resolution used for mapping pulse width range to 0-180 degrees range
volatile uint32_t pulseDuration;
volatile uint32_t pulseDuration_Min;
volatile uint32_t pulseDuration_Max;

// I2C Variables
#define N_DECIMAL_POINTS_PRECISION (100.0)
#define slave_address 0x30
uint8_t mergedData[9];
uint8_t theta_mergedData[3];
uint8_t vel_acc_mergedData[2];
uint32_t servo_angle;


#define HomingTimeSlice 21
#define arrayConstantSize 300
extern uint32_t TimeSlice;

extern int time1;  // Interrupt timer
extern int time2;
extern int time3;
extern int time4;
extern int time5;
extern int time6;

#define Pin_0           0x00000001     // Pin 0
#define Pin_1           0x00000002     // Pin 1
#define Pin_2           0x00000004     // Pin 2
#define Pin_3           0x00000008     // Pin 3
#define Pin_4           0x00000010     // Pin 4
#define Pin_5           0x00000020     // Pin 5
#define Pin_6           0x00000030     // Pin 6
#define Pin_7           0x00000040     // Pin 7

extern volatile int suctionStartIndex;  // Index of solenoid start
extern volatile int suctionStopIndex;   // Index of solenoid stop
extern volatile int nextIndex;
extern volatile int servoFirstIndex;
extern volatile int I2C_servo_first;

extern int first;


extern float steps1ToGo;                // Number of steps to be done
extern float steps2ToGo;
extern float steps3ToGo;

extern volatile float oldstep;          // last Time slice steps
extern volatile float oldstep2;
extern volatile float oldstep3;

extern int rising_edge1;
extern int falling_edge1;

extern int rising_edge2;
extern int falling_edge2;

extern int rising_edge3;
extern int falling_edge3;

extern int path_iterator;


extern volatile float stepInterval1;    // Time delay between each step
extern volatile float stepInterval2;
extern volatile float stepInterval3;
extern volatile float requiredSpeed1;
extern volatile float requiredSpeed2;
extern volatile float requiredSpeed3;

extern float Speed1, Speed2, Speed3, Accel1, Accel2, Accel3;

extern int accelerating1;
extern int accelerating2;
extern int accelerating3;
extern float oldSpeed;
extern float oldSpeed1;
extern float oldSpeed2;
extern float oldSpeed3;
extern float oldAccel1;
extern float oldAccel2;
extern float oldAccel3;
extern float motor1globalsteps;         // Absolute position of the motor
extern float motor2globalsteps;
extern float motor3globalsteps;

extern float homingPath[HomingTimeSlice][3];
float path1[arrayConstantSize][3];
float path2[arrayConstantSize][3];
float path3[arrayConstantSize][3];

extern volatile int microStep;


#endif /* GLOBALVARIABLES_H_ */
