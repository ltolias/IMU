// main.h
// Leonidas Tolias

//device specific includes
#include "inc/hw_types.h"
#include "utils/uartstdio.h"
#include "driverlib/timer.h"
#include "driverlib/fpu.h"
#include "inc/hw_ints.h"
#include "driverlib/interrupt.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"

//genral includes
#include "MPU6050.h"
#include "spi.h"
#include "kalman.h"
#include "motors.h"


//proportionality constants
#define DIVAX 185
#define DIVGX 131
#define DIVAY DIVAX
#define DIVGY DIVGX

//update rates
#define FILTERUPDATE 500 //Hertz, this ones a little fussy, but it should work...

#define USEPID
//#define USEPROPORTIONAL

#define USEKALMAN
//#define USECOMPLEMENTARY


#ifdef USEPID
typedef struct errorStructTag{
	float current;
	float lastangle;
	float currangle;
	float sum;
	float derr;
	float delta;
} errorStruct;
#endif


//holders for the raw accel/gyro data
int16_t ax, ay, az, gx, gy, gz;
//self explanatory
float trueangle[2];
float desiredangle[2];
float thrust;
//stabilizing changes to each motor thrust
float motorDelta[5];
//thrust correction to hold altitude at when angled
float motorTCorr[5];

float ka; //filter constant


#ifdef USEPID
errorStruct error[2]; //error struct
float kp; //Proportional gain
float kd; //Derivative gain
float ki; //Integral gain
#endif
#ifdef USEPROPORTIONAL
float kp; //orientation adjustment gain (motor percents per degree)
#endif


//why the hell not?
int i;
