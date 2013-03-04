//offline_kalman.c
//Leonidas Tolias

//Offline Kalman Filter Implementation for Quadcopters

//Implements a Kalman Filter that determine angle and angular velocities
//about an axis for the purposes of attitute estimation of Quadcopters
//Precalculates Kalman gain (which stabilizes after a few itterations anyways) to shorten the filter loop when flying
//All matrix operations are hardcoded and loops unrolled to save time, so the code doesn't really look like a Kalman Filter
//A nice description of how this filter is set up can be found at www.leonidastolias.com/drop/kalman.pdf


#include "offline_kalman.h"



void K_Initialize(float gyro, float accell, float process)
{

	state.x = 0;
	state.y = 0;
	state.dx = 0;
	state.dy = 0;

	observation.x = 0;
	observation.dx = 0;
	observation.y = 0;
	observation.dy = 0;


	R[0] = accell;
	R[1] = 0;
	R[2] = 0;
	R[3] = gyro;

	//Q[0] = PROCESSCOVARIANCE * DELTA*DELTA*DELTA*DELTA / 4;
	//Q[1] = PROCESSCOVARIANCE * DELTA*DELTA*DELTA / 2;
	//Q[2] = PROCESSCOVARIANCE * DELTA*DELTA*DELTA / 2;
	//Q[3] = PROCESSCOVARIANCE * DELTA*DELTA ;

	Q[0] = process;
	Q[1] = 0;
	Q[2] = 0;
	Q[3] = process;


	P[1][0] = 0;
	P[1][1] = 0.0;
	P[1][2] = 0.0;
	P[1][3] = 0;

}



void K_Predict_P()
{

	P[0][0] = P[1][3] * DSQUARED + P[1][0] + (P[1][1] + P[1][2]) * DELTA + Q[0];
	P[0][1] = P[1][3] * DELTA + P[1][1] + Q[1];
	P[0][2] = P[1][3] * DELTA + P[1][2] + Q[2];
	P[0][3] = P[1][3] + Q[3];
}



void K_Update_K()
{
	float temp[4];
	temp[0] = R[0] + P[0][0];
	temp[1] = P[0][1];
	temp[2] = P[0][2];
	temp[3] = R[3] + P[0][3];

	float mult = temp[3] * temp[0] - temp[2] * temp[1];
	if (mult == 0) mult = 1;
	temp[0] /= mult;
	temp[1] /= mult;
	temp[2] /= mult;
	temp[3] /= mult;

	K[1][0] = P[0][0] * temp[3] + P[0][1] * -temp[2];
	K[1][1] = P[0][0] * -temp[1] + P[0][1] * temp[0];
	K[1][2] = P[0][2] * temp[3] + P[0][3] * -temp[2];
	K[1][3] = P[0][2] * -temp[1] + P[0][3] * temp[0];

}


void K_Update_P()
{
	float temp[4];
	temp[0] = R[0] + P[0][0];
	temp[1] = P[0][1];
	temp[2] = P[0][2];
	temp[3] = R[3] + P[0][3];
	P[1][0] = P[0][0] - (temp[0]*K[1][0]*K[1][0] + (temp[1] + temp[2])*K[1][1]*K[1][0] + temp[3]*K[1][1]*K[1][1]); 
	P[1][1] = P[0][1] - ((temp[0]*K[1][2] + temp[1]*K[1][3])*K[1][0] + (temp[2]*K[1][2] + temp[3]*K[1][3])*K[1][1]);
	P[1][2] = P[0][2] - ((temp[0]*K[1][2] + temp[2]*K[1][3])*K[1][0] + (temp[1]*K[1][2] + temp[3]*K[1][3])*K[1][1]);
	P[1][3] = P[0][3] - (temp[0]*K[1][2]*K[1][2] + (temp[1] + temp[2])*K[1][3]*K[1][2] + temp[3]*K[1][3]*K[1][3]);

}

int K_Precalculate_Gains()
{
	int i = 0;

    do
    {
        i ++;
        //UARTprintf("%i", i);
        K_Predict_P();
        K[0][0] = K[1][0];
        K[0][1] = K[1][1];
        K[0][2] = K[1][2];
        K[0][3] = K[1][3];
        K_Update_K();

        K_Update_P();
        //UARTprintf("%i", (int)((K[1][0] - K[0][0])*1000));

    } while ((K[1][0] - K[0][0]) != 0);
    
    K_theta_theta = (1-K[1][0]);
    K_theta_dtheta = ((1*DELTA)-K[1][1]);
    K_theta_obs = (K[1][0]);
    K_theta_dobs = (K[1][1]);
    K_dtheta_dtheta = (1-K[1][3]);
    K_dtheta_obstheta =  K[1][2];
    K_dtheta_dobs = K[1][3];

    return i;



}
