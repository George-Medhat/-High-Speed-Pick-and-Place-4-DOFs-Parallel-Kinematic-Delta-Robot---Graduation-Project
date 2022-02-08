/*
 *
 * High Speed Pick and Place Delta Robot Graduation Project
 *
 */
#include "delay.h"

void delayMs(uint32_t ui32Ms) {

    SysCtlDelay(ui32Ms * (SysCtlClockGet() / 3 / 1000));
}

void delayUs(uint32_t ui32Us) {
    SysCtlDelay(ui32Us * (SysCtlClockGet() / 3 / 1000000));
}

void delayNs(uint32_t ui32Ns) {
    SysCtlDelay(ui32Ns * (SysCtlClockGet() / 3 / 1000000000));
}


