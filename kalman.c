//kalman.c
//Leonidas Tolias

//Kalman Filter Implementation for Quadcopters

//Implements 2 parallel, identical Kalman Filters that determine angle and angular velocities
//about the x and y axes for the purposes of attitute estimation of Quadcopter Devices
//All matrix operations are hardcoded to save time, so the code doesn't really look like a Kalman Filter
//A nice description of how this filter is set up can be found at www.leonidastolias.com/drop/kalman.pdf


#include "kalman.h"




void K_initialize()
{
	state.x = 0;
	state.y = 0;
	state.dx = 0;
	state.dy = 0;
	control_x = 0;
	control_y = 0;

	R[0] = ACCELCOVARIANCE;
	R[1] = 0;
	R[2] = 0;
	R[3] = GYROCOVARIANCE;

	Q[0] = PROCESSCOVARIANCE * DELTA*DELTA*DELTA*DELTA / 4;
	Q[1] = PROCESSCOVARIANCE * DELTA*DELTA*DELTA / 2;
	Q[2] = PROCESSCOVARIANCE * DELTA*DELTA*DELTA / 2;
	Q[3] = PROCESSCOVARIANCE * DELTA*DELTA ;

	Px[1][0] = 0.1;
	Px[1][1] = 0.0;
	Px[1][2] = 0.0;
	Px[1][3] = 0.1;

	Py[1][0] = 0.1;
	Py[1][1] = 0.0;
	Py[1][2] = 0.0;
	Py[1][3] = 0.1;

}


void K_Predict_State()
{
	state.x = state.x + DELTA * state.dx + DSQUARED * control_x / 2;
	state.dx = state.dx + DELTA * control_x;
	state.y = state.y + DELTA * state.dy + DSQUARED * control_y / 2;
	state.dy = state.dy + DELTA * control_y;
}


void K_Predict_P()
{

	Px[0][0] = Px[1][3] * DSQUARED + Px[1][0] + (Px[1][1] + Px[1][2]) * DELTA + Q[0];
	Px[0][1] = Px[1][3] * DELTA + Px[1][1] + Q[1];
	Px[0][2] = Px[1][3] * DELTA + Px[1][2] + Q[2];
	Px[0][3] = Px[1][3] + Q[3];

	Py[0][0] = Py[1][3] * DSQUARED + Py[1][0] + (Py[1][1] + Py[1][2]) * DELTA + Q[0];
	Py[0][1] = Py[1][3] * DELTA + Py[1][1] + Q[1];
	Py[0][2] = Py[1][3] * DELTA + Py[1][2] + Q[2];
	Py[0][3] = Py[1][3] + Q[3];
}

void K_Get_Observation(float x, float dx, float y, float dy)
{
	observation.x = x;
	observation.y = y;
	observation.dx = dx;
	observation.dy = dy;

}

void K_Update_K()
{
	float temp[4];
	temp[0] = R[0] + Px[0][0];
	temp[1] = Px[0][1];
	temp[2] = Px[0][2];
	temp[3] = R[3] + Px[0][3];

	float mult = temp[3] * temp[0] - temp[2] * temp[1];
	if (mult == 0) mult = 1;
	temp[0] /= mult;
	temp[1] /= mult;
	temp[2] /= mult;
	temp[3] /= mult;

	Kx[0] = Px[0][0] * temp[3] + Px[0][1] * -temp[2];
	Kx[1] = Px[0][0] * -temp[1] + Px[0][1] * temp[0];
	Kx[2] = Px[0][2] * temp[3] + Px[0][3] * -temp[2];
	Kx[3] = Px[0][2] * -temp[1] + Px[0][3] * temp[0];


	temp[0] = R[0] + Py[0][0];
	temp[1] = Py[0][1];
	temp[2] = Py[0][2];
	temp[3] = R[3] + Py[0][3];

	mult = temp[3] * temp[0] - temp[2] * temp[1];
	if (mult == 0) mult = 1;

	temp[0] /= mult;
	temp[1] /= mult;
	temp[2] /= mult;
	temp[3] /= mult;

	Ky[0] = Py[0][0] * temp[3] + Py[0][1] * -temp[2];
	Ky[1] = Py[0][0] * -temp[1] + Py[0][1] * temp[0];
	Ky[2] = Py[0][2] * temp[3] + Py[0][3] * -temp[2];
	Ky[3] = Py[0][2] * -temp[1] + Py[0][3] * temp[0];
}

void K_Update_State()
{
	float b0 = (observation.x - state.x);
	float b1 = (observation.dx - state.dx);
	state.x = state.x + Kx[0] * b0 + Kx[1] * b1;
	state.dx = state.dx + Kx[2] * b0 + Kx[3] * b1;

	b0 = (observation.y - state.y);
	b1 = (observation.dy - state.dy);
	state.y = state.y + Ky[0] * b0 + Ky[1] * b1;
	state.dy = state.dy + Ky[2] * b0 + Ky[3] * b1;
}

void K_Report_Attitude(float *trueangle)
{
	trueangle[0] = state.x;
	trueangle[1] = state.y;
}


void K_Update_P()
{
	float temp[4];
	temp[0] = R[0] + Px[0][0];
	temp[1] = Px[0][1];
	temp[2] = Px[0][2];
	temp[3] = R[3] + Px[0][3];
	Px[1][0] = Px[0][0] - (temp[0]*Kx[0]*Kx[0] + (temp[1] + temp[2])*Kx[1]*Kx[0] + temp[3]*Kx[1]*Kx[1]); 
	Px[1][1] = Px[0][1] - ((temp[0]*Kx[2] + temp[1]*Kx[3])*Kx[0] + (temp[2]*Kx[2] + temp[3]*Kx[3])*Kx[1]);
	Px[1][2] = Px[0][2] - ((temp[0]*Kx[2] + temp[2]*Kx[3])*Kx[0] + (temp[1]*Kx[2] + temp[3]*Kx[3])*Kx[1]);
	Px[1][3] = Px[0][3] - (temp[0]*Kx[2]*Kx[2] + (temp[1] + temp[2])*Kx[3]*Kx[2] + temp[3]*Kx[3]*Kx[3]);



	temp[0] = R[0] + Py[0][0];
	temp[1] = Py[0][1];
	temp[2] = Py[0][2];
	temp[3] = R[3] + Py[0][3];
	Py[1][0] = Py[0][0] - (temp[0]*Ky[0]*Ky[0] + (temp[1] + temp[2])*Ky[1]*Ky[0] + temp[3]*Ky[1]*Ky[1]); 
	Py[1][1] = Py[0][1] - ((temp[0]*Ky[2] + temp[1]*Ky[3])*Ky[0] + (temp[2]*Ky[2] + temp[3]*Ky[3])*Ky[1]);
	Py[1][2] = Py[0][2] - ((temp[0]*Ky[2] + temp[2]*Ky[3])*Ky[0] + (temp[1]*Ky[2] + temp[3]*Ky[3])*Ky[1]);
	Py[1][3] = Py[0][3] - (temp[0]*Ky[2]*Ky[2] + (temp[1] + temp[2])*Ky[3]*Ky[2] + temp[3]*Ky[3]*Ky[3]);
}




