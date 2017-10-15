/*
 * servo.h
 *
 * Created: 08/12/2016 18:49:54
 *  Author: Emile
 */ 


#ifndef SERVO_H_
#define SERVO_H_

#include <stdint.h>

typedef struct
{
	uint16_t max;
	uint16_t min;
	uint16_t currentPoint;
	uint32_t average;
	uint16_t setPoint;
	
	uint32_t setPointRamp;
	int16_t  moveStep;
}servo_data_t;


void servo_setPoint( servo_data_t *servoData, uint16_t setPoint );


void servo_loadSettings( uint8_t nbr, servo_data_t *servoData );
void servo_saveSettings( uint8_t nbr, servo_data_t *servoData );

#endif /* SERVO_H_ */