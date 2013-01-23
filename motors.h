//motors.h
//Leonidas Tolias

//Methods for controlling Brushless Motor Electronic Speed Controllers on the Stellaris platform



#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/gpio.h"


#include "stdint.h"


#define MOTORUPDATE 100 //Hertz


void motorSet(int motor, uint16_t percent);


void initPWM();