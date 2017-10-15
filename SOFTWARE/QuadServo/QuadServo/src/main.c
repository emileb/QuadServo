/**
 * \file
 *
 * \brief Empty user application template
 *
 */

/**
 * \mainpage User Application template doxygen documentation
 *
 * \par Empty user application template
 *
 * Bare minimum empty user application template
 *
 * \par Content
 *
 * -# Include the ASF header files (through asf.h)
 * -# "Insert system clock initialization code here" comment
 * -# Minimal main function that starts with a call to board_init()
 * -# "Insert application code here" comment
 *
 */

/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */
#include <asf.h>
#include "hal.h"
#include "pid.h"
#include <stdio.h>
#include <avr/io.h> 
#include "fifo.h"
#include "quadservo_protocol.h"
#include "string.h"
#include "servo.h"
#include "rc_servo.h"


static int uart_putchar(char c, FILE *stream);
static void uart_init (void);
static FILE mystdout = FDEV_SETUP_STREAM (uart_putchar, NULL, _FDEV_SETUP_WRITE);

static uint32_t time = 0;

#define RAMP_SCALE 10000ul
#define AVERAGE_SCALE 4096ull

#define SERVO_EN(S) (1 << S)
#if 0 // RC servo board
	//#define USE_IDENTITY 
	//#define ENABLE_SERVO
	#define ENABLE_RC_SERVO 0xFF

	//#define ENABLE_PORTC_UART
	#define ENABLE_USB_UART
	#define ENABLE_PORTA_OUTPUT

#elif 1 // Bad Catto
	//#define USE_IDENTITY
	#define ENABLE_SERVO    SERVO_EN(MOTOR_0) + SERVO_EN(MOTOR_1)
	#define ENABLE_RC_SERVO 0x3
	
	#define ENABLE_PORTC_UART
	#define ENABLE_USB_UART
	//#define ENABLE_PORTA_OUTPUT
#endif


static uint8_t debugChannel = 0;

static void position(void);

static void processCmd(uint8_t data[]);

static void sendCmd( uint8_t id, uint8_t param,uint16_t data1, uint16_t data2 );

tPidData pid[MOTOR_NBR];
servo_data_t servoData[MOTOR_NBR];
fifo_t rxFifo;


uint8_t indentity;

int main (void)
{
	/* Insert system clock initialization code here (sysclk_init()). */

	
	for( int n = 0 ;n < 1000; n++ )
	{
		
		
	}
	
	for( int n = 0; n < MOTOR_NBR; n++ )
	{
		// Set defaults
		servoData[ n ].setPoint = 1000;
		servoData[ n ].moveStep = 0;
		
		servoData[ n ].min = 500;
		servoData[ n ].max = 1500;
		
		// Loads if valid
		servo_loadSettings( n, &servoData[ n ]);
	}
	

	stdout = &mystdout;

	pmic_init();
	
	sysclk_init();
	
	irq_initialize_vectors();
	
	sleepmgr_init();

#ifdef ENABLE_USB_UART
	udc_start();
#endif

#ifdef USE_IDENTITY
	setupIdentityPins();
	indentity = getIdentity();
#else
	indentity = 0;
#endif

	// Always setup motor driver control		
	setupMotor();
	setMotorVelocity(MOTOR_0_ADC_CH, 0);
	setMotorVelocity(MOTOR_1_ADC_CH, 0);
	setMotorVelocity(MOTOR_2_ADC_CH, 0);
	setMotorVelocity(MOTOR_3_ADC_CH, 0);


#ifdef ENABLE_SERVO
	
	setupADC(MOTOR_0_ADC_CH, ADCCH_POS_PIN0);
	setupADC(MOTOR_1_ADC_CH, ADCCH_POS_PIN1);
	setupADC(MOTOR_2_ADC_CH, ADCCH_POS_PIN2);
	setupADC(MOTOR_3_ADC_CH, ADCCH_POS_PIN3);
	//setupADC(TEST_ADC_CH, ADCCH_POS_PIN7);

	//pidInit( &pid[MOTOR_0], 1.0, 0.000, 0, 10000, 1, 100);
	//pidInit( &pid[MOTOR_1], 0.7, 0.000, 0, 10000, 1, 70);
	pidInit( &pid[MOTOR_0], 0.6, 0.000, 0.0, 10000, 1, 100);
	pidInit( &pid[MOTOR_1], 0.6, 0.000, 0.0, 10000, 1, 70);
	pidInit( &pid[MOTOR_2], 0.4, 0.00, 0.0, 10000, 1, 70);
	pidInit( &pid[MOTOR_3], 0.0, 0.00, 0, 10000, 1, 0);
	
	tc_enable(&TCC0);
	tc_set_overflow_interrupt_callback(&TCC0, position);
	tc_set_wgm(&TCC0, TC_WG_NORMAL);
	tc_write_period(&TCC0, 1000 * 10); // 1000 is 500Hz, 10000 is 50Hz
	tc_set_overflow_interrupt_level(&TCC0, TC_INT_LVL_LO);

	tc_write_clock_source(&TCC0, TC_CLKSEL_DIV64_gc);
#endif

#ifdef ENABLE_PORTA_OUTPUT
	ioport_set_port_dir(IOPORT_PORTA, 0xFF, IOPORT_DIR_OUTPUT);
#endif

#ifdef ENABLE_RC_SERVO
	rc_servo_init( ENABLE_RC_SERVO );
#endif

#ifdef ENABLE_PORTC_UART
	setupUart();
#endif
	cpu_irq_enable();
	/*
	uint32_t t = 0;
	uint8_t pos = 0;
	while(1)
	{
		
		if( t % 500 == 0)
		{
			
			pos++;
			rc_servo_setPos(0, pos);
		}
		t++;
		
	}
	*/
	
	
	int16_t result;

	adc_enable(&MY_ADC);
	
	fifo_reset(&rxFifo);
	
	//usart_set_rx_interrupt_level( &USARTC0, USART_INT_LVL_HI );
	
	while (true)
	{
		//cpu_irq_disable();
		usart_set_rx_interrupt_level( &USARTC0, USART_INT_LVL_OFF );
	
		//Clear any bytes if not in sync
		while( fifo_bytes(&rxFifo) && ( fifo_peek_byte(&rxFifo) != SERVO_PROT_SYNC_BYTE ))
		{
			fifo_get_byte( &rxFifo ); //Pop off bad bytes
		}
		
		unsigned char packet[SERVO_PROT_PACKET_LEN];
		Bool packetFound = false;
		if( fifo_bytes(&rxFifo) >= SERVO_PROT_PACKET_LEN )
		{
			for(uint8_t n = 0; n < SERVO_PROT_PACKET_LEN; n++ )
			{
				packet[n] = fifo_get_byte( &rxFifo );
			}	
			packetFound = true;	
		}
		
		//Get interrupts back on ASAP
		//cpu_irq_enable();
		usart_set_rx_interrupt_level( &USARTC0, USART_INT_LVL_LO );
	
		if( packetFound )
		{
			processCmd( packet );
			//printf( "PACKAET FOUND" );
		}
		
		for(int n = 0; n < 1000; n++)
		{
			volatile m = n;
		}
		//printf( "p = %d\n", pid.p);
		{
			//cpu_irq_disable();
			uint16_t cur;
			cur = servoData[debugChannel].currentPoint;
			//cur = servoData[debugChannel].average / AVERAGE_SCALE;
			//cpu_irq_enable();
		
			//sendCmd( 1, SERVO_SEND_DATA, servoData[debugChannel].setPoint,cur)	;
		}
		static uint32_t d = 0;
		d++;
		if(d % 30 == 0)
		{
			
			static uint8_t cnt = 0;
			cnt ++;
			
			ioport_set_pin_level( IOPORT_CREATE_PIN(PORTA, 0), cnt&1 );
			ioport_set_pin_level( IOPORT_CREATE_PIN(PORTA, 1), cnt&2 );
			ioport_set_pin_level( IOPORT_CREATE_PIN(PORTA, 2), cnt&4 );
			ioport_set_pin_level( IOPORT_CREATE_PIN(PORTA, 3), cnt&8 );
			ioport_set_pin_level( IOPORT_CREATE_PIN(PORTA, 4), cnt&16 );
			ioport_set_pin_level( IOPORT_CREATE_PIN(PORTA, 5), cnt&32 );
			ioport_set_pin_level( IOPORT_CREATE_PIN(PORTA, 6), cnt&64 );

		}
	}
}

static void sendCmd( uint8_t id, uint8_t param, uint16_t data1, uint16_t data2 )
{
	command_data_t cmdData;
	
	cmdData.start = SERVO_PROT_SYNC_BYTE;
	cmdData.id = id;
	cmdData.param = param;
	cmdData.data1 = data1;
	cmdData.data2 = data2;
	cmdData.crc = 0x99;
	
	for( int n = 0; n < SERVO_PROT_PACKET_LEN; n++)
	{
		usart_putchar(&USARTC0, ((uint8_t*)&cmdData)[n]);
	}
}


static void processCmd(uint8_t data[])
{
	command_data_t command;
	memcpy( &command, data, SERVO_PROT_PACKET_LEN );
	
#ifdef USE_IDENTITY
	if(command.id >> 2 == indentity)
	{
		
		// Get real servo id
		command.id = command.id & 0x3;
		
#endif
	
		if ( command.crc == 0x99 )
		{		
			if (command.param == SERVO_SET_SETPOINT)
			{
				servo_setPoint( &servoData[ command.id ], command.data1 );
			
			}
			else if(command.param == SERVO_SET_TIME)
			{
				servo_setPoint( &servoData[ command.id ], command.data1 );
			
				int32_t diff = servoData[ command.id ].setPoint - servoData[ command.id ].currentPoint;
				diff *= RAMP_SCALE;
			
				servoData[ command.id ].setPointRamp = servoData[ command.id ].currentPoint  * RAMP_SCALE;
				servoData[ command.id ].moveStep = diff / command.data1;
			}
			else if(command.param == SERVO_SET_MIN)
			{
				servoData[ command.id ].min = command.data1;
				// Reset setpoint
				servo_setPoint( &servoData[ command.id ], servoData[ command.id ].setPoint );
				servo_saveSettings( command.id, &servoData[ command.id ]);
			}
			else if(command.param == SERVO_SET_MAX)
			{
				servoData[ command.id ].max = command.data1;
				// Reset setpoint
				servo_setPoint( &servoData[ command.id ], servoData[ command.id ].setPoint );
				servo_saveSettings( command.id, &servoData[ command.id ]);
			}
			else if(command.param == SERVO_GET_MAXMIN )
			{
				uint8_t id =  command.id;
				debugChannel = id;
				sendCmd( id, SERVO_SEND_MAXMIN, servoData[id].min , servoData[id].max );
			}
			else if(command.param == SERVO_SET_PWM )
			{
				setMotorVelocity( command.id, (int8_t)command.data1);
			}
			else if(command.param == SERVO_SAVE_EEPROM)
			{
				for( int n = 0; n < MOTOR_NBR; n++ )
				{
					servo_saveSettings( n, &servoData[ n ]);
				}
			}
			
			else if(command.param == RC_SERVO_SET_SETPOINT)
			{
				rc_servo_setPos(  command.id , command.data1 );
			}
		}
		else
		{
			int size =  fifo_bytes(&rxFifo);
			printf("ERROR %d", size);
		}
#ifdef USE_IDENTITY
	}
#endif
}

// Interrupt from normal UART
ISR(USARTC0_RXC_vect)
{
	uint8_t data;
	data = USARTC0.DATA;

	fifo_put_byte(&rxFifo, data);
}


// Interrupt from USB UART
void uart_rx_notify(uint8_t port)
{
	while (udi_cdc_is_rx_ready()) {
		uint8_t data;
		data = udi_cdc_getc();
		fifo_put_byte(&rxFifo, data);
	} 	
}

uint32_t lowPassExponential(uint32_t input, uint32_t average, uint32_t factor)
{
	uint32_t t = (input * AVERAGE_SCALE );
	t = t * factor;
	t = t / 256 ;
	
	uint32_t s = (average * (256 - factor));
	s = s / 256;
	return t + s;
    //return (((input * AVERAGE_SCALE ) * 256 ) / factor) + ((average * 256) /  (256-factor)); 
}

static void position(void)
{
	time++;
	
	static uint32_t t = 0;
	if(t == 50)
	{
		static uint8_t led = 0;
		led = ! led;
		ioport_set_pin_level(MY_LED, led);
		t = 0;
	}

	t++;
	
	//adc_wait_for_interrupt_flag(&MY_ADC, TEST_ADC_CH);
	//adc_wait_for_interrupt_flag(&MY_ADC, MOTOR_0_ADC_CH);
/*

	int16_t setPoint = adc_get_result(&MY_ADC, TEST_ADC_CH);
	if (setPoint < 0) setPoint = 0;
		
	static int16_t setPointLast;
	if (abs(setPointLast - setPoint) > 0)
	{
		setPointLast = setPoint;
	}
	
	*/
	
	int16_t currentPoint = adc_get_result(&MY_ADC, MOTOR_0_ADC_CH);
	if (currentPoint < 0) currentPoint = 0;
	servoData[MOTOR_0].currentPoint = currentPoint;
	
	currentPoint = adc_get_result(&MY_ADC, MOTOR_1_ADC_CH);
	if (currentPoint < 0) currentPoint = 0;
	servoData[MOTOR_1].currentPoint = currentPoint;
	
	currentPoint = adc_get_result(&MY_ADC, MOTOR_2_ADC_CH);
	if (currentPoint < 0) currentPoint = 0;
	servoData[MOTOR_2].currentPoint = currentPoint;

	currentPoint = adc_get_result(&MY_ADC, MOTOR_3_ADC_CH);
	if (currentPoint < 0) currentPoint = 0;
	servoData[MOTOR_3].currentPoint = currentPoint;


	for( int p = 0; p < 4; p++ )
	{
		// Check servo is enabled
		if( ENABLE_SERVO & (1 << p) )
		{
			servoData[p].average = lowPassExponential( servoData[p].currentPoint, servoData[p].average, 250 );
			
			uint16_t average = servoData[p].average / AVERAGE_SCALE;
			average = servoData[p].currentPoint;
			
			pid[p].setPoint = servoData[p].setPoint;
			
			int16_t pwm = pidRun( &pid[p], average);
			
			setMotorVelocity(p, pwm);
		}
		
	/*
		if( p == debugChannel )
		{
			static int16_t n = 0;
			n++;
			if( n % 10 == 0 )
				printf("set = %d, cur = %d, av = %d, e = %d, pwm = %d\n",pid[p].setPoint,
																servoData[p].currentPoint,
																(average),
																pid[p].setPoint - average,
																pwm);
		}
		//printf("%d: %d,%d %d,   %d\n",p, setPointLast,pos,pwm,setPointLast-pos);
*/	
		
					
	}
		
	
	adc_start_conversion(&MY_ADC, MOTOR_0_ADC_CH | MOTOR_1_ADC_CH | MOTOR_2_ADC_CH | MOTOR_3_ADC_CH );
}




static int uart_putchar (char c, FILE *stream)
{
	if (c == '\n')
		uart_putchar('\r', stream);

	usart_putchar(&USARTC0,c);

	return 0;
}

