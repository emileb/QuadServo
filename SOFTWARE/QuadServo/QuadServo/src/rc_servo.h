/*
 * rc_servo.h
 *
 * Created: 05/02/2017 17:32:03
 *  Author: Emile
 */ 


#ifndef RC_SERVO_H_
#define RC_SERVO_H_


#include <stdint.h>

void rc_servo_init( uint8_t output );

void rc_servo_setPos( uint8_t servo, uint8_t pos );


#endif /* RC_SERVO_H_ */