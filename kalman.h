//kalman.h
//Leonidas Tolias

//Kalman Filter Implementation for Quadcopters

//Implements 2 parallel, identical Kalman Filters that determine angle and angular velocities
//about the x and y axes for the purposes of attitute estimation of Quadcopter Devices
//All matrix operations are hardcoded to save time, so the code doesn't really look like a Kalman Filter
//A nice description of how this filter is set up can be found at www.leonidastolias.com/drop/kalman.pdf




#define DELTA 1
#define ACCELDEVIATION 5
#define GRYODEVIATION 5
#define PROCESSDEVIATION 5

#define DSQUARED DELTA * DELTA
#define ACCELCOVARIANCE ACCELDEVIATION*ACCELDEVIATION
#define GYROCOVARIANCE GRYODEVIATION*GRYODEVIATION
#define PROCESSCOVARIANCE PROCESSDEVIATION*PROCESSDEVIATION

//heres a pretty tricky part, we want this constant I to be equal to the thrust (grams) per motor percentage multiplied by
//the lever arm of the motors (torque) divided by the moment of innertia of the quadcopter about the axis
//this could be found experimentally I guess... or we can try a lovely value of... .2?
//this value is, however, very important to the accuracy of the model...

#define I 0.2


typedef struct{
	float x;
	float dx;
	float y;
	float dy;
}K_state;

K_state state;

typedef struct{
	float x;
	float dx;
	float y;
	float dy;
}K_observation;

K_observation observation;

float control_x;
float control_y;

float R[4];

float Q[4];

float Px[2][4];
float Py[2][4];

float Kx[4];
float Ky[4];

void K_initialize();

void K_Get_Control(float x, float y);

void K_Predict_State();

void K_Predict_P();

void K_Get_Observation(float x, float dx, float y, float dy);

void K_Update_K();

void K_Update_State();

void K_Report_Attitude(float *trueangle);

void K_Update_P();