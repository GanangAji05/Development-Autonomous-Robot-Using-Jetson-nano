/*
 * PID.h
 *
 *  Created on: Jul 10, 2024
 *      Author: Ganang PC
 */

#ifndef INC_PID_H_
#define INC_PID_H_

extern float error;
extern float P, I, D;
extern float Total_error_I, previous_error_I;
extern float Total_error_D, previous_error_D;

//extern int error2;
//extern int P2, I2, D2;
//extern float Total_error_I2 , previous_error_I2;
//extern float Total_error_D2 ,previous_error_D2;

void PID(float Kp, float Ki, float Kd, float *Setpoint, float *Input, float time, float *output, float *output1);

//void PID2(float Kp, float Ki, float Kd, int *setpoint, int *Input, float time, int *output, int output1);

#endif /* INC_PID_H_ */

