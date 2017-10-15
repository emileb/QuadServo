/*
 * pid.h
 *
 * Created: 25/05/2016 17:07:00
 *  Author: Emile
 */ 


#ifndef PID_H_
#define PID_H_

#include <stdint.h>

#define PID_INT_SCALE 2048

#define PID_COEF(x) ((float)x * (float)PID_INT_SCALE)

typedef struct
{
	uint16_t p;
	uint16_t i;
	uint16_t d;
	
	uint16_t setPoint;

	int32_t integral;
	int32_t integralLimit;
	
	int16_t prevError;
	
	uint8_t max;
	
	int8_t sign;
	
} tPidData;

pidInit( tPidData *pid, float p, float i, float d, float intLimit, int8_t sign, uint8_t max );

int16_t pidRun( tPidData *pid, uint16_t pos);


#endif /* PID_H_ */