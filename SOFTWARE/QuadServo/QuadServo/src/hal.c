
#include "hal.h"

#include <asf.h>


#define DEVICE_0_STANDBY    IOPORT_CREATE_PIN(PORTR, 1)
#define DEVICE_1_STANDBY    IOPORT_CREATE_PIN(PORTR, 0)

#define MOTOR_1_IN1    IOPORT_CREATE_PIN(PORTD, 0)
#define MOTOR_1_IN2    IOPORT_CREATE_PIN(PORTD, 1)

#define MOTOR_0_IN1    IOPORT_CREATE_PIN(PORTB, 3)
#define MOTOR_0_IN2    IOPORT_CREATE_PIN(PORTB, 2)

#define MOTOR_3_IN1    IOPORT_CREATE_PIN(PORTD, 3)
#define MOTOR_3_IN2    IOPORT_CREATE_PIN(PORTD, 2)

#define MOTOR_2_IN1    IOPORT_CREATE_PIN(PORTD, 4)
#define MOTOR_2_IN2    IOPORT_CREATE_PIN(PORTD, 5)

static struct pwm_config motor_0_pwm;
static struct pwm_config motor_1_pwm;
static struct pwm_config motor_2_pwm;
static struct pwm_config motor_3_pwm;

#define PWM_FREQ 25000

#define IDENT_IN_0  IOPORT_CREATE_PIN(PORTC, 4)
#define IDENT_IN_1  IOPORT_CREATE_PIN(PORTC, 5)


void setupMotor( void )
{
	ioport_set_pin_dir(DEVICE_0_STANDBY, IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(DEVICE_1_STANDBY, IOPORT_DIR_OUTPUT);
	
	ioport_set_pin_dir(MOTOR_0_IN1, IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(MOTOR_0_IN2, IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(MOTOR_1_IN1, IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(MOTOR_1_IN2, IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(MOTOR_2_IN1, IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(MOTOR_2_IN2, IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(MOTOR_3_IN1, IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(MOTOR_3_IN2, IOPORT_DIR_OUTPUT);
	
	ioport_set_pin_level(DEVICE_0_STANDBY, 1);
	ioport_set_pin_level(DEVICE_1_STANDBY, 1);
	
	ioport_set_pin_level(MOTOR_0_IN1, 1);
	ioport_set_pin_level(MOTOR_0_IN2, 0);
	ioport_set_pin_level(MOTOR_1_IN1, 1);
	ioport_set_pin_level(MOTOR_1_IN2, 0);
	ioport_set_pin_level(MOTOR_2_IN1, 1);
	ioport_set_pin_level(MOTOR_2_IN2, 0);
	ioport_set_pin_level(MOTOR_3_IN1, 1);
	ioport_set_pin_level(MOTOR_3_IN2, 0);
	
	//PWM
	ioport_set_pin_dir( IOPORT_CREATE_PIN(PORTE, 0), IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir( IOPORT_CREATE_PIN(PORTE, 1), IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir( IOPORT_CREATE_PIN(PORTE, 2), IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir( IOPORT_CREATE_PIN(PORTE, 3), IOPORT_DIR_OUTPUT);
	
	pwm_init(&motor_1_pwm, PWM_TCE0, PWM_CH_A, PWM_FREQ); 
	pwm_init(&motor_0_pwm, PWM_TCE0, PWM_CH_B, PWM_FREQ); 
	pwm_init(&motor_3_pwm, PWM_TCE0, PWM_CH_D, PWM_FREQ); 
	pwm_init(&motor_2_pwm, PWM_TCE0, PWM_CH_C, PWM_FREQ); 
	
	pwm_start(&motor_0_pwm, 0); 
	pwm_start(&motor_1_pwm, 0); 
	pwm_start(&motor_2_pwm, 0); 
	pwm_start(&motor_3_pwm, 0); 
	
}

void setMotorPWM( eMotor motor, uint8_t p)
{
	switch( motor )
	{
		case MOTOR_0:
			pwm_set_duty_cycle_percent(&motor_0_pwm, p);
		break;
		case MOTOR_1:
			pwm_set_duty_cycle_percent(&motor_1_pwm, p);
		break;
		case MOTOR_2:
			pwm_set_duty_cycle_percent(&motor_2_pwm, p);
		break;
		case MOTOR_3:
			pwm_set_duty_cycle_percent(&motor_3_pwm, p);
		break;		
	}
}

void setMotorVelocity( eMotor motor, int8_t p)
{
	switch( motor )
	{
		case MOTOR_0:
		if(p > 0)
		{
			ioport_set_pin_level(MOTOR_0_IN1, 1);
			ioport_set_pin_level(MOTOR_0_IN2, 0);
			pwm_set_duty_cycle_percent(&motor_0_pwm, p);
		}
		else if (p < 0)
		{
			ioport_set_pin_level(MOTOR_0_IN1, 0);
			ioport_set_pin_level(MOTOR_0_IN2, 1);
			pwm_set_duty_cycle_percent(&motor_0_pwm, -p);
		}
		else //0 = break
		{
			ioport_set_pin_level(MOTOR_0_IN1, 1);
			ioport_set_pin_level(MOTOR_0_IN2, 1);
			pwm_set_duty_cycle_percent(&motor_0_pwm, 0);
		}
		break;
		
		case MOTOR_1:
		if(p > 0)
		{
			ioport_set_pin_level(MOTOR_1_IN1, 1);
			ioport_set_pin_level(MOTOR_1_IN2, 0);
			pwm_set_duty_cycle_percent(&motor_1_pwm, p);
		}
		else if (p < 0)
		{
			ioport_set_pin_level(MOTOR_1_IN1, 0);
			ioport_set_pin_level(MOTOR_1_IN2, 1);
			pwm_set_duty_cycle_percent(&motor_1_pwm, -p);
		}
		else //0 = break
		{
			ioport_set_pin_level(MOTOR_1_IN1, 1);
			ioport_set_pin_level(MOTOR_1_IN2, 1);
			pwm_set_duty_cycle_percent(&motor_1_pwm, 0);
		}
		break;
		
		case MOTOR_2:
		if(p > 0)
		{
			ioport_set_pin_level(MOTOR_2_IN1, 1);
			ioport_set_pin_level(MOTOR_2_IN2, 0);
			pwm_set_duty_cycle_percent(&motor_2_pwm, p);
		}
		else if (p < 0)
		{
			ioport_set_pin_level(MOTOR_2_IN1, 0);
			ioport_set_pin_level(MOTOR_2_IN2, 1);
			pwm_set_duty_cycle_percent(&motor_2_pwm, -p);
		}
		else //0 = break
		{
			ioport_set_pin_level(MOTOR_2_IN1, 1);
			ioport_set_pin_level(MOTOR_2_IN2, 1);
			pwm_set_duty_cycle_percent(&motor_2_pwm, 0);
		}
		break;	
		
		case MOTOR_3:
		if(p > 0)
		{
			ioport_set_pin_level(MOTOR_3_IN1, 1);
			ioport_set_pin_level(MOTOR_3_IN2, 0);
			pwm_set_duty_cycle_percent(&motor_3_pwm, p);
		}
		else if (p < 0)
		{
			ioport_set_pin_level(MOTOR_3_IN1, 0);
			ioport_set_pin_level(MOTOR_3_IN2, 1);
			pwm_set_duty_cycle_percent(&motor_3_pwm, -p);
		}
		else //0 = break
		{
			ioport_set_pin_level(MOTOR_3_IN1, 1);
			ioport_set_pin_level(MOTOR_3_IN2, 1);
			pwm_set_duty_cycle_percent(&motor_3_pwm, 0);
		}
		break;
		
	}
}

void setupADC(uint8_t ADC_CHAN, enum adcch_positive_input POS )
{
	 struct adc_config adc_conf;
	 struct adc_channel_config adcch_conf;

	 adc_read_configuration(&MY_ADC, &adc_conf);
	 adcch_read_configuration(&MY_ADC, ADC_CHAN, &adcch_conf);

	 adc_set_conversion_parameters(&adc_conf, ADC_SIGN_ON, ADC_RES_12, ADC_REF_AREFB);
	 adc_set_conversion_trigger(&adc_conf, ADC_TRIG_MANUAL, 1, 0);
	 adc_set_clock_rate(&adc_conf, 200000UL);

	 //adcch_set_input(&adcch_conf, ADCCH_POS_PIN0, ADCCH_NEG_NONE, 1);
	 adcch_set_input(&adcch_conf, POS, ADCCH_NEG_PAD_GND, 1);

	 adc_write_configuration(&MY_ADC, &adc_conf);
	 adcch_write_configuration(&MY_ADC, ADC_CHAN, &adcch_conf);
	
}

void setupIdentityPins()
{
	ioport_set_pin_dir(IDENT_IN_0, IOPORT_DIR_INPUT);
	ioport_set_pin_dir(IDENT_IN_0, IOPORT_DIR_INPUT);
	
	// Inverting to make easier to read
	ioport_set_pin_mode(IDENT_IN_0, IOPORT_MODE_PULLUP | IOPORT_MODE_INVERT_PIN);
	ioport_set_pin_mode(IDENT_IN_1, IOPORT_MODE_PULLUP | IOPORT_MODE_INVERT_PIN);
}

uint8_t getIdentity()
{
	uint8_t ret = 0;
	ret += ioport_get_value(IDENT_IN_0)?1:0;
	ret += ioport_get_value(IDENT_IN_1)?2:0;
	return ret;	
}

void setupUart()
{
	ioport_set_pin_dir(IOPORT_CREATE_PIN(PORTC, 3), IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(IOPORT_CREATE_PIN(PORTC, 2), IOPORT_DIR_INPUT);
	
	sysclk_enable_peripheral_clock(&USARTC0);
	usart_set_mode(&USARTC0, USART_CMODE_ASYNCHRONOUS_gc);

	USARTC0.CTRLC = (uint8_t) USART_CHSIZE_8BIT_gc | USART_PMODE_DISABLED_gc | false;
		
	USARTC0.CTRLB |= USART_RXEN_bm;
	USARTC0.CTRLB |= USART_TXEN_bm;
		
		
	//usart_set_baudrate(&USARTC0,57600,sysclk_get_per_hz());
	usart_set_baudrate(&USARTC0,115200,sysclk_get_per_hz());
	
}