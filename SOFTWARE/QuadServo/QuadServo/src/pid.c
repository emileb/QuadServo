
#include "pid.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

int16_t pidRun( tPidData *pidData, uint16_t pos)
{
	int32_t error = (int16_t)pidData->setPoint - (int16_t)pos;
	
	if( abs(error) < 6)
	{
		
		error = 0;
		//pidData->integral = 0;
		//pidData->prevError = 0;
	}
	
	if( pidData->integral > pidData->integralLimit ) pidData->integral =  pidData->integralLimit;
	if( pidData->integral < -pidData->integralLimit ) pidData->integral =  -pidData->integralLimit;
	
		
	//if( abs( error) < 3)
	//	pidData->integral = 0;
	
	int32_t p = (pidData->p * error);
	int32_t i = (pidData->i * (pidData->integral));
	int32_t d =  (pidData->d * (error - pidData->prevError));
	
	int32_t output = (p) + (i) + (d);
	
	output = output / PID_INT_SCALE;
	

	if( output > pidData->max ) 
		output = pidData->max;	
	else if( output < -pidData->max ) 
		output = -pidData->max;
	else
		pidData->integral += error;
		
	//printf("vel = %d\n", current_velocity);
		
	//printf("E=%ld,p=%ld,i=%ld,d=%ld\n",error,p,i,d)	;
	//printf("E=%ld,p=%ld,i=%ld,d=%ld\n",error,p,i,d)	;
	
	pidData->prevError = error;
	
	return output * pidData->sign;
}

pidInit( tPidData *pid, float p, float i, float d, float intLimit, int8_t sign, uint8_t max  )
{
	pid->p = PID_COEF(p);
	pid->i = PID_COEF(i);
	pid->d = PID_COEF(d);
	pid->integral = 0;
	pid->integralLimit = intLimit;	
	pid->sign = sign;
	pid->max = max;
	
	pid->setPoint = 1024;
}