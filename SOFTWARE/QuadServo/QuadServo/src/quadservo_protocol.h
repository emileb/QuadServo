/*
 * protocol.h
 *
 * Created: 27/06/2016 20:41:50
 *  Author: Emile
 */ 


#ifndef QUADSERVO_PROTOCOL_H_
#define QUADSERVO_PROTOCOL_H_

#define SERVO_PROT_PACKET_LEN 8
#define SERVO_PROT_SYNC_BYTE  0x39


#define SERVO_SET_SETPOINT  0
#define SERVO_SET_TIME      1
#define SERVO_SET_MIN       2
#define SERVO_SET_MAX       3

#define SERVO_GET_MAXMIN    4
#define SERVO_SAVE_EEPROM   5

#define SERVO_SET_PWM       10




#define RC_SERVO_SET_SETPOINT  50

#define SERVO_SEND_DATA     100
#define SERVO_SEND_MAXMIN   101


typedef struct
{
	uint8_t start;
	uint8_t id;
	uint8_t param;
	uint16_t data1;
	uint16_t data2;
	uint8_t crc;		
}command_data_t;



#endif /* PROTOCOL_H_ */