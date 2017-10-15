/*
 * hal.h
 *
 * Created: 18/05/2016 21:42:11
 *  Author: Emile
 */ 


#ifndef HAL_H_
#define HAL_H_

#include <asf.h>

#define MY_LED    IOPORT_CREATE_PIN(PORTB, 1)

#define MY_ADC    ADCA
#define MOTOR_0_ADC_CH ADC_CH0
#define MOTOR_1_ADC_CH ADC_CH1
#define MOTOR_2_ADC_CH ADC_CH2
#define MOTOR_3_ADC_CH ADC_CH3

#define TEST_ADC_CH ADC_CH3


typedef enum
{
	MOTOR_0,
	MOTOR_1,
	MOTOR_2,
	MOTOR_3,
	MOTOR_NBR
} eMotor;

void setupLED( void );

void setupMotor( void );

void setMotorPWM( eMotor motor, uint8_t p);

void setMotorVelocity( eMotor motor, int8_t p);

void setupADC(uint8_t ADC_CHAN, enum adcch_positive_input POS );

void setupIdentityPins( void );

void setupUart( void );

uint8_t getIdentity( void );

#endif /* HAL_H_ */