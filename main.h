// main.h
// Leonidas Tolias

//Main Program Loop and Interrupt Handlers

/*This program is designed to use a Stellaris Launchpad from Texas Instrumentsand an MPU-6050 from Invensense
to maintain and control the attitude of a quadcopter.

Attitude Estimation is performed by two offline Kalman Filters, each of which tracks angle and angular velocity about one axis.  A description of the derivation of the filter can be found at 
www.leonidastolias.com.

Motor Control assumes that brushless motors are being used, and uses 2 timers (TIMER1 and TIMER2) split into 4 subtimers
to create pulses that vary between 1 and 20 ms which are standard control inputs for brushless Electronic Speed Controllers.
The Pulse repetition rate can be controlled with MOTORUPDATE.

Communications use simple serial control over a bluetooth serial port.  In the future, a NRF24L01 controll interface will be added.

The program exists in the following files:
main.c - System Initialization and main control loop
motors.c - methods for controlling Brushless Electronic Speed Controllers
offline_kalman.c - methods for precomputing Kalman Gain
MPU6050.c - methods for controlling the MPU-6050 from Invensense
I2Clib.c - methods for hardware I2C communications

The code attempts to be as easy to port as possible.  Platform specific code exists only in:
motors.c - timer routines
I2Clib.c - Hardware I2C routines
main.c - Interrupt Handlers and Configuration, Hardware UART routines

*/

//device specific includes
#include "inc/hw_types.h"
#include "inc/hw_uart.h"
#include "driverlib/timer.h"
#include "driverlib/fpu.h"
#include "inc/hw_ints.h"
#include "driverlib/interrupt.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"
#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
//#include "utils/uartstdio.h"

//genral includes
#include "math.h"
#include "MPU6050.h"
#include "spi.h"
#include "motors.h"
#include "offline_kalman.h"
#include "buffereduart.h"


//some useful constants
#define DIVA 20.48f
#define DIVG 65.5f

#define XNAUGHT 10	
#define	YNAUGHT -10
#define YAWNAUGHT 15

#define TELEMETRYUPDATE 10


#define UPDATERATIO MOTORUPDATE/FILTERUPDATE

#define USEPID
//#define USEPROPORTIONAL

//#define USEKALMAN
#define USECOMPLEMENTARY



#define IRANGE 10.0f


#ifdef USEPID
typedef struct{
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
float yaw;
//stabilizing changes to each motor thrust
float motorDelta[5];
//thrust correction to hold altitude at when angled
float motorTCorr[5];

float ka; //filter constant

//for averaging states between motor updates
float sumx, sumy, avx, avy;

K_state zero;

volatile int inc;
volatile unsigned char c;



float Accel_Variance, Gyro_Variance;

#ifdef USEPID
errorStruct error[2]; //error struct
float kp; //Proportional gain
float kd; //Derivative gain
float ki; //Integral gain
float kdd;
float kii;
#endif
#ifdef USEPROPORTIONAL
float kp; //orientation adjustment gain (motor percents per degree)
#endif





//why the hell not?
int i;

void initConsole(void);
void initFlightVars(void);
float square(float a);
void calibrateIMU();
void configureInterupts();
void stopInterrupts();
void Initialize_System();
void transferToUART();
void doPID();



