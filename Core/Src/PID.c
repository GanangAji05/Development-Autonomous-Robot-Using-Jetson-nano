/*
 * PID.c
 *
 *  Created on: Jul 13, 2024
 *      Author: Ganang PC
 */
#include "PID.h"
#include <stdlib.h>

float error;
float P, I, D;
float Total_error_I, previous_error_I;
float Total_error_D, previous_error_D;

//int error2;
//int P2, I2, D2;
//float Total_error_I2 , previous_error_I2;
//float Total_error_D2 ,previous_error_D2;

void PID(float Kp, float Ki, float Kd, float *Setpoint, float *Input, float time, float *output, float *output1)
{
	error = *Setpoint - *Input;
	P = Kp * error;
	//

	//Integral
	Total_error_I = error + previous_error_I;
	I = Ki * Total_error_I*time;
	previous_error_I = Total_error_I;
	//

	//Derivativ
	Total_error_D = error - previous_error_D;
	D = (Kd * Total_error_D)/time;
	previous_error_D = error;
	//

	//Last PID
	*output = P+I+D;
	*output1 = fabs(*output);
}

//void PID2(float Kp , float Ki, float Kd, int *setpoint, int *Input, float time, int *output, int output1)
//{
//	error2 = *setpoint - *Input;
//    P2 = Kp * error2;
//	//
//
//	//Integral
//	Total_error_I2 = error2 + previous_error_I2;
//	I2 = Ki * Total_error_I2*time;
//	previous_error_I2 = Total_error_I2;
//	//
//
//	//Derivativ
//	Total_error_D2 = error2 - previous_error_D2;
//	D2 = (Kd * Total_error_D2)/time;
//	previous_error_D2 = error2;
//	//
//
//	//Last PID
//	*output = P2+I2+D2;
//	output1 = abs(*output);
//}



