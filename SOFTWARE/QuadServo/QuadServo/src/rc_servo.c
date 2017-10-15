/*
 * rc_servo.c
 *
 * Created: 05/02/2017 17:32:18
 *  Author: Emile
 */ 

#include "rc_servo.h"

#include <asf.h>

#define RC_SERVO_TIMER TCD0

#define RC_SERVO_PORT PORTC

#define PERIOD (500 * 30)

#define MINIMUM (500 * 0.9)

#define RANGE (125 * 2.5)




typedef enum
{
	STATE_LOW,
	STATE_HIGH,
	STATE_POS
	
} tState;

 
static uint8_t m_outputs;

static uint8_t m_pos[8];

static uint8_t m_timingArray[256];

static uint8_t m_arrayPos = 0;

static tState m_state = LOW;


static void timer_callback()
{
	if( m_state == STATE_POS) // This must be very very fast
	{
		//Clear any outputs set in our array
		RC_SERVO_PORT.OUTCLR = m_timingArray[m_arrayPos];
		
		m_arrayPos++;
		if( m_arrayPos == 0)
		{
			m_state = STATE_LOW; // Set to low state
			RC_SERVO_PORT.OUTCLR = m_outputs;		
			// Setup timer to trigger in 20ms (low time)
			tc_write_clock_source(&RC_SERVO_TIMER, TC_CLKSEL_DIV64_gc ); 
			tc_write_period(&RC_SERVO_TIMER, PERIOD ); // ((32000000/1000)*20) / 64 = 10000
			tc_write_count(&RC_SERVO_TIMER, 0);
			tc_clear_overflow(&RC_SERVO_TIMER);
			
		}
	}
	else if( m_state == STATE_LOW) // Low, go to High for 1ms state
	{
		RC_SERVO_PORT.OUTSET = m_outputs;
		m_state = STATE_HIGH;
		tc_write_period(&RC_SERVO_TIMER, MINIMUM ); // Minimum position
		tc_write_count(&RC_SERVO_TIMER, 0);
		tc_clear_overflow(&RC_SERVO_TIMER);
	}
	else if( m_state == STATE_HIGH) // Now go to the position state
	{
		m_state = STATE_POS;
		tc_write_period(&RC_SERVO_TIMER, RANGE ); 
		tc_write_clock_source(&RC_SERVO_TIMER, TC_CLKSEL_DIV1_gc);		
		tc_write_count(&RC_SERVO_TIMER, 0);
		tc_clear_overflow(&RC_SERVO_TIMER);
	}
}

#define DEFAULT 100

void rc_servo_init( uint8_t output )
{
	m_outputs = output;
	
	for( int n = 0; n < 256; n++ )
	{
		m_timingArray[n] = 0;
	}
	
	//Put all in the middle
	m_timingArray[DEFAULT] = m_outputs;
	
	for( int n = 0; n < 8; n++ )
	{
		m_pos[n] = DEFAULT;
	}
	
	tc_enable(&RC_SERVO_TIMER);
	tc_set_overflow_interrupt_callback(&RC_SERVO_TIMER, timer_callback);
	tc_set_wgm(&RC_SERVO_TIMER, TC_WG_NORMAL);

	tc_set_overflow_interrupt_level(&RC_SERVO_TIMER, TC_INT_LVL_HI);
	

	tc_write_clock_source(&RC_SERVO_TIMER, TC_CLKSEL_DIV64_gc );
	tc_write_period(&RC_SERVO_TIMER, 10000 ); // ((32000000/1000)*20) / 64 = 10000
	
	// Set outputs
	RC_SERVO_PORT.DIRSET = m_outputs;
	
}

void rc_servo_setPos( uint8_t servo, uint8_t pos )
{
	// Clear old position
	m_timingArray[ m_pos[servo] ] &= ~(1<<servo);
	
	//Set new position
	m_timingArray[ pos ] |= (1<<servo);
	
	//Save new position
	m_pos[servo] = pos;
	
}