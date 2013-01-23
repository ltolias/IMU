//kalman.h
//Leonidas Tolias

//Kalman Filter Implementation for Quadcopters

//Implements 2 parallel, identical Kalman Filters that determine angle and angular velocities
//about the x and y axes for the purposes of attitute estimation of Quadcopter Devices
//All matrix operations are hardcoded to save time, so the code doesn't really look like a Kalman Filter
//A nice description of how this filter is set up can be found at www.leonidastolias.com/drop/kalman.pdf




#define DELTA 0.002
#define DSQUARED DELTA * DELTA
#define ACCELCOVARIANCE 10
#define GYROCOVARIANCE 10
#define PROCESSCOVARIANCE 10



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

void K_Predict_State();

void K_Predict_P();

void K_Get_Observation(float x, float dx, float y, float dy);

void K_Update_K();

void K_Update_State();

void K_Report_Attitude(float *trueangle);

void K_Update_P();