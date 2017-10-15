/*
 * servo.c
 *
 * Created: 08/12/2016 19:00:54
 *  Author: Emile
 */ 
#include "servo.h"
#include "nvm.h"


#define VALID_ADDR 0x00
#define MAX_ADDR   0x02
#define MIN_ADDR   0x04

#define VALID_CHECK 0xA501

void servo_setPoint( servo_data_t *servoData, uint16_t setPoint  )
{
	if( setPoint >= servoData->max )
		servoData->setPoint = servoData->max;
    else if( setPoint <= servoData->min )
		servoData->setPoint = servoData->min;
	else
		servoData->setPoint = setPoint;
}

void servo_loadSettings( uint8_t nbr, servo_data_t *servoData )
{
	uint8_t read_page[EEPROM_PAGE_SIZE];
	nvm_eeprom_read_buffer( nbr * EEPROM_PAGE_SIZE, read_page, EEPROM_PAGE_SIZE);
	
	uint16_t validCheck = *((uint16_t *)&read_page[VALID_ADDR]);
	
	if( validCheck != VALID_CHECK )
	{
		return;
	}
	
	servoData->max = *((uint16_t *)&read_page[MAX_ADDR]);
	servoData->min = *((uint16_t *)&read_page[MIN_ADDR]);
	
}

void servo_saveSettings( uint8_t nbr, servo_data_t *servoData )
{
	uint8_t write_page[EEPROM_PAGE_SIZE];
	
	*((uint16_t *)&write_page[VALID_ADDR]) = VALID_CHECK;
	
	*((uint16_t *)&write_page[MAX_ADDR]) = servoData->max;
	*((uint16_t *)&write_page[MIN_ADDR]) = servoData->min;
	
	nvm_eeprom_load_page_to_buffer(write_page);
	nvm_eeprom_atomic_write_page(nbr);
	
}