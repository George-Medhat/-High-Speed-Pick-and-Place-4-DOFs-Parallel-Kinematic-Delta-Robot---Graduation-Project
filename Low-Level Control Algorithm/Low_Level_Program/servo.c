/*
 *
 * High Speed Pick and Place Delta Robot Graduation Project
 *
 */
#include "servo.h"


void servoActuation(uint32_t degrees_1, uint32_t degrees_2)
{
    if (degrees_1 >= 180)
    {
        pulseDuration = pulseDuration_Max;
    }
    else
    {
        pulseDuration = pulseDuration_Min + ((float) degrees_1 * (pulseDuration_Max - pulseDuration_Min)) / 180.0;
    }

    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, pulseDuration * load / resolution);
}

void PWM_DividerGet(void)
 {
     uint32_t PWMDiv = SysCtlPWMClockGet();
     if (PWMDiv == SYSCTL_PWMDIV_1)
     PWM_Divider = 1;
     else if (PWMDiv == SYSCTL_PWMDIV_2)
     PWM_Divider = 2;
     else if (PWMDiv == SYSCTL_PWMDIV_4)
     PWM_Divider = 4;
     else if (PWMDiv == SYSCTL_PWMDIV_8)
     PWM_Divider = 8;
     else if (PWMDiv == SYSCTL_PWMDIV_16)
     PWM_Divider = 16;
     else if (PWMDiv == SYSCTL_PWMDIV_32)
     PWM_Divider = 32;
     else
     PWM_Divider = 64;
 }

void pulseDurationComputation(void){

    uint32_t pulsePeriod = (1.0/PWM_Frequency) * 1000;                              // Pulse Period of 20ms
    double pulseStepDuration = pulsePeriod/resolution;                              // Divide the Pulse Period by the resolution to have more accurate values from mapping
    pulseDuration_Min = servoMotor_ZeroPostionPulseWidthLimit/pulseStepDuration;    // Calculate the minimum pulse duration to actuate the servo to 0 degrees with resoultion factor
    pulseDuration_Max = servoMotor_MaxPostionPulseWidthLimit/pulseStepDuration;     // Calculate the maximum pulse duration to actuate the servo to 0 degrees with resoultion factor
}
