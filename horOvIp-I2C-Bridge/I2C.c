/*
 * I2C.c
 *
 * Created: 06.10.2019 18:33:21
 *  Author: kai_horstmann
 */ 

#include <stdint.h>
#include <stdbool.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include "uip.h"


/** I2C clock speed
 * Use the standard I2C speed.
 */
#define I2C_CLOCK_FREQ 100000

#define TWI_BIT_RATE_REG_VAL ((F_CPU/I2C_CLOCK_FREQ-15)/2)



/** 
 * Create the address/R_W byte on the I2C bus.
 * This is the first byte being sent by the master after creating the START condition
 *
 * @param i2cAddr
 */
inline uint8_t i2CAddrByte (uint8_t i2cAddr,bool read) {
	return ((i2cAddr << 1) | read?1:0);
}

enum I2CAction {
	ACT_SEND_START,
	ACT_SEND_STOP,
	ACT_SEND_RESTART,
	ACT_SEND_ADDR_RW,
	ACT_SEND_BYTE,
	ACT_RECV_BYTE_ACK,
	ACT_RECV_BYTE_NACK
	};

enum I2CStatus {
	STAT_IDLE,
	STAT_START_SENT,
	START_RESTART_SENT,
	STAT_ADDR_SENT,
	STAT_BYTE_SENT,
	STAT_BYTE_RECEIVED_ACK,
	STAT_BYTE_RECEIVED_NACK,
	STAT_NONE
};
	
	
typedef struct {
	enum I2CStatus nextStatusOK;
	enum I2CAction nextActionOK;
	enum I2CStatus nextStatusNOK;
	enum I2CAction nextActionNOK;
	uint8_t expectStatusCode;
	} TActionSequence;	
	
static const TActionSequence sequenceSendBytes [] = {
	//STAT_IDLE,
	{STAT_START_SENT,ACT_SEND_START,STAT_START_SENT,ACT_SEND_START,0xff},
	//STAT_START_SENT,
	{STAT_ADDR_SENT,ACT_SEND_ADDR_RW,STAT_ADDR_SENT,ACT_SEND_ADDR_RW,0x08}
	//START_RESTART_SENT,
	//STAT_ADDR_SENT,
	//STAT_BYTE_SENT,
	//STAT_BYTE_RECEIVED_ACK,
	//STAT_BYTE_RECEIVED_NACK
	//STAT_NONE
};
