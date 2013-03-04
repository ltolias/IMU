//motors.h
//Leonidas Tolias

//Methods for controlling Brushless Motor Electronic Speed Controllers on the Stellaris platform



#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/gpio.h"


#include "stdint.h"


#define MOTORUPDATE 200.0f //Hertz
#define MOTORS 50.0f


void motorSet(int motor, float percent);


void initPWM();