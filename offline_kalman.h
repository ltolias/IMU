//offline_kalman.h
//Leonidas Tolias

//Offline Kalman Filter Implementation for Quadcopters

//Implements a Kalman Filter that determine angle and angular velocities
//about an axis for the purposes of attitute estimation of Quadcopters
//Precalculates Kalman gain (which stabilizes after a few iterations anyways) to shorten the filter loop when flying
//All matrix operations are hardcoded and loops unrolled to save time, so the code doesn't really look like a Kalman Filter
//A nice description of how this filter is set up can be found at www.leonidastolias.com/drop/kalman.pdf



#define FILTERUPDATE 400.0f //Hertz

#define DELTA 1/FILTERUPDATE

#define DSQUARED DELTA * DELTA




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


float R[4];

float Q[4];

float P[2][4];

float K[2][4];

float K_theta_theta;
float K_theta_dtheta;
float K_theta_obs;
float K_theta_dobs;
float K_dtheta_dtheta;
float K_dtheta_obstheta;
float K_dtheta_dobs;


void K_Initialize(float gyro, float accell, float process);

void K_Predict_P();

void K_Update_K();

void K_Update_P();

int K_Precalculate_Gains();
