/*
 * I2C.c
 *
 * Created: 06.10.2019 18:33:21
 *  Author: kai_horstmann
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>

#include "uip.h"
#include "serDebugOut.h"
#include "i2c.h"

#include <util/delay.h>

/** I2C clock speed
 * Use the standard I2C speed.
 */
#define I2C_CLOCK_FREQ 100000

// Use 15 instead 16 to round the register value
#define TWI_BIT_RATE_REG_VAL ((F_CPU/I2C_CLOCK_FREQ-15)/2)



/** 
 * Create the address/R_W byte on the I2C bus.
 * This is the first byte being sent by the master after creating the START condition
 *
 * @param i2cAddr
 */
#define i2CAddrByte(i2cAddr,read) ((i2cAddr << 1) | (read?1:0))
/*
 * uint8_t i2CAddrByte(uint8_t i2cAddr,bool read) {
 * 	return ((i2cAddr << 1) | (read?1:0));
 * }
 */

enum I2CAction {
	ACT_NONE,
	ACT_SEND_START,
	ACT_SEND_ADDR_W,
	ACT_SEND_ADDR_R,
	ACT_SEND_BYTE,
	ACT_RECV_BYTE_ACK,
	ACT_RECV_BYTE_NACK,
	ACT_SEND_STOP
	};

	
typedef struct {
	enum I2CStatus nextStatusOK;
	enum I2CAction nextActionOK;
	enum I2CStatus nextStatusNOK;
	enum I2CAction nextActionNOK;
	uint8_t expectStatusCode;
	} TActionSequence;	
	
static const TActionSequence sequenceSendBytes [] = {
	
	// Idle loops to itself.
	//STAT_IDLE
	{STAT_IDLE,ACT_NONE,STAT_IDLE,ACT_NONE,0xff},

	//STAT_START_SENT,
	{STAT_ADDR_W_SENT,ACT_SEND_ADDR_W,STAT_ADDR_W_SENT,ACT_SEND_ADDR_W,0x08},

	//STAT_RESTART_SENT,
	{STAT_ADDR_W_SENT,ACT_SEND_ADDR_W,STAT_ADDR_W_SENT,ACT_SEND_ADDR_W,0x10},
	
	//STAT_ADDR_W_SENT,
	{STAT_BYTE_SENT,ACT_SEND_BYTE,STAT_STOP_SENT,ACT_SEND_STOP,0x18},
	
	// This should never happen
	//STAT_ADDR_R_SENT,
	{STAT_STOP_SENT,ACT_SEND_STOP,STAT_STOP_SENT,ACT_SEND_STOP,0x00},

	// Default sequence assuming more than one byte is sent.
	// For the last byte the action code must twist the next sequence to STAT_LAST_BYTE_SENT
	//STAT_BYTE_SENT,
	{STAT_BYTE_SENT,ACT_SEND_BYTE,STAT_STOP_SENT,ACT_SEND_STOP,0x28},
	
	// STAT_LAST_BYTE_SENT,
	{STAT_STOP_SENT,ACT_SEND_STOP,STAT_STOP_SENT,ACT_SEND_STOP,0x28},
	
	// Should never occur, I am SENDING!
	//STAT_BYTE_RECEIVED,
	{STAT_STOP_SENT,ACT_SEND_STOP,STAT_STOP_SENT,ACT_SEND_STOP,0x00},
	
	// Should never occur, I am SENDING!
	//STAT_LAST_BYTE_RECEIVED
	{STAT_STOP_SENT,ACT_SEND_STOP,STAT_STOP_SENT,ACT_SEND_STOP,0x00},
	
	//STAT_STOP_SENT,
	{STAT_IDLE,ACT_NONE,STAT_IDLE,ACT_NONE,0xff}
};

static const TActionSequence sequenceReceiveBytesWithRegisterAdress [] = {

	// Idle loops to itself.
	//STAT_IDLE
	{STAT_IDLE,ACT_NONE,STAT_IDLE,ACT_NONE,0xff},

	//STAT_START_SENT,
	{STAT_ADDR_W_SENT,ACT_SEND_ADDR_W,STAT_ADDR_W_SENT,ACT_SEND_ADDR_W,0x08},

	//STAT_RESTART_SENT,
	{STAT_ADDR_R_SENT,ACT_SEND_ADDR_R,STAT_ADDR_R_SENT,ACT_SEND_ADDR_R,0x10},
	
	//STAT_ADDR_W_SENT,
	{STAT_BYTE_SENT,ACT_SEND_BYTE,STAT_STOP_SENT,ACT_SEND_STOP,0x18},
	
	// If only one byte is to be received the action code can twist the next status to STAT_LAST_BYTE_RECEIVED_NACK
	//STAT_ADDR_R_SENT,
	{STAT_BYTE_RECEIVED,ACT_RECV_BYTE_ACK,STAT_STOP_SENT,ACT_SEND_STOP,0x40},

	// Default sequence assuming more than one byte is sent as .
	// For the last byte the action code must twist the next sequence to STAT_LAST_BYTE_SENT
	//STAT_BYTE_SENT,
	{STAT_BYTE_SENT,ACT_SEND_BYTE,STAT_STOP_SENT,ACT_SEND_STOP,0x28},
	
	// This is actually legit for sending the register address to send.
	// STAT_LAST_BYTE_SENT,
	{STAT_RESTART_SENT,ACT_SEND_START,STAT_STOP_SENT,ACT_SEND_STOP,0x28},
	
	// If the last byte is to be received the action code can be twisted to ACT_RECV_BYTE_NACK,
	// and the next status to STAT_LAST_BYTE_RECEIVED
	// STAT_BYTE_RECEIVED
	{STAT_BYTE_RECEIVED,ACT_RECV_BYTE_ACK,STAT_STOP_SENT,ACT_SEND_STOP,0x50},
	
	//STAT_LAST_BYTE_RECEIVED
	{STAT_STOP_SENT,ACT_SEND_STOP,STAT_STOP_SENT,ACT_SEND_STOP,0x58},
	
	//STAT_STOP_SENT,
	{STAT_IDLE,ACT_NONE,STAT_IDLE,ACT_NONE,0xff}
};

static const TActionSequence sequenceReceiveBytes [] = {

	// Idle loops to itself.
	//STAT_IDLE
	{STAT_IDLE,ACT_NONE,STAT_IDLE,ACT_NONE,0xff},

	//STAT_START_SENT,
	{STAT_ADDR_R_SENT,ACT_SEND_ADDR_R,STAT_ADDR_R_SENT,ACT_SEND_ADDR_R,0x08},

	//STAT_RESTART_SENT,
	{STAT_ADDR_R_SENT,ACT_SEND_ADDR_R,STAT_ADDR_R_SENT,ACT_SEND_ADDR_R,0x10},
	
	// Should never happen!
	//STAT_ADDR_W_SENT,
	{STAT_STOP_SENT,ACT_SEND_STOP,STAT_STOP_SENT,ACT_SEND_STOP,0x00},
	
	// If only one byte is to be received the action code can twist the next status to STAT_LAST_BYTE_RECEIVED_NACK
	//STAT_ADDR_R_SENT,
	{STAT_BYTE_RECEIVED,ACT_RECV_BYTE_ACK,STAT_STOP_SENT,ACT_SEND_STOP,0x40},

	// Should never happen!
	//STAT_BYTE_SENT,
	{STAT_STOP_SENT,ACT_SEND_STOP,STAT_STOP_SENT,ACT_SEND_STOP,0x00},
	
	// This should never happen
	// STAT_LAST_BYTE_SENT,
	{STAT_STOP_SENT,ACT_SEND_STOP,STAT_STOP_SENT,ACT_SEND_STOP,0x00},
	
	// If the last byte is to be received the action code can be twisted to ACT_RECV_BYTE_NACK,
	// and the next status to STAT_LAST_BYTE_RECEIVED
	// STAT_BYTE_RECEIVED
	{STAT_BYTE_RECEIVED,ACT_RECV_BYTE_ACK,STAT_STOP_SENT,ACT_SEND_STOP,0x50},
	
	//STAT_LAST_BYTE_RECEIVED
	{STAT_STOP_SENT,ACT_SEND_STOP,STAT_STOP_SENT,ACT_SEND_STOP,0x58},
	
	//STAT_STOP_SENT,
	{STAT_IDLE,ACT_NONE,STAT_IDLE,ACT_NONE,0xff}
};


/// The current applicable I2C sequence, i.e. points to the start of the sequence
static const TActionSequence *currSequence = NULL;
/// The current sequence status.
static const TActionSequence *currSequenceStatus = NULL;
/// The corresponding index of currSequenceStatus
static enum I2CStatus currStatusIndex = STAT_IDLE;
/// The address of the addressed slave on the bus
static uint8_t slaveAddr = 0;

/// Last index in sendBuffer, i.e. dataLength-1
static uint8_t lastSendIndex = 0;
/// Current index in sendBuffer while sending bytes
static uint8_t currSendIndex = 0;
/// Pointer to the data to be sent.
static uint8_t *sendBuffer    = NULL;

/// Last index in recvBuffer, i.e. dataLength-1
static uint8_t lastRecvIndex = 0;
/// Current index in recvBuffer while sending bytes
static uint8_t currRecvIndex = 0;
/// Pointer to the buffer which will accept the received data.
static uint8_t *recvBuffer    = NULL;

/// Is a transfer active?
static bool transferActive = false;
/// The the final result of a transfer. Can be set in between, even if cleanup is being send (i.e. STOP)
static enum I2CTransferResult transferResult = I2C_RC_UNDEF;
/// The status code of the I2C hardware when transferResult is not I2C_RC_OK.
static uint8_t i2cHWStatusCode = 0;
// The sequence status when an error occurred.
static enum I2CStatus errSeqStatus = STAT_IDLE;
	

void I2CStartTransferSend (uint8_t slAddr, uint8_t *data, uint8_t dataLen) {
	
	while (transferActive) {
		__asm__ __volatile__ ("nop" ::: "memory");
	}

	currSequence = sequenceSendBytes;
	currStatusIndex = STAT_START_SENT;
	currSequenceStatus = &(currSequence[currStatusIndex]);
	slaveAddr = slAddr;
	lastSendIndex = dataLen - 1;
	currSendIndex = 0;
	sendBuffer = data;
	
	// Don't worry about receive stuff here.
	
	transferActive = true;
	transferResult = I2C_RC_OK;
	/// The status code of the I2C hardware when transferResult is not I2C_RC_OK.
	i2cHWStatusCode = 0;
	errSeqStatus = STAT_IDLE;

	__asm__ __volatile__ ("nop" ::: "memory");
	// Activate I2C, activate the interrupt, occupy the bus, and send START
	TWCR = 0
	| _BV(TWINT)
	// | _BV(TWEA)
	| _BV(TWSTA)
	// | _BV(TWSTO)
	// | _BV(TWWC)
	| _BV(TWEN)
	| _BV(TWIE)
	;
	
}

void I2CStartTransferSendReceive (
	uint8_t slAddr, 
	uint8_t *sendData,uint8_t sendDataLen,
	uint8_t *recvData,uint8_t recvDataLen) {
	
	while (transferActive) {
		__asm__ __volatile__ ("nop" ::: "memory");
	}

	currSequence = sequenceReceiveBytesWithRegisterAdress;
	currStatusIndex = STAT_START_SENT;
	currSequenceStatus = &(currSequence[currStatusIndex]);
	slaveAddr = slAddr;
	lastSendIndex = sendDataLen - 1;
	currSendIndex = 0;
	sendBuffer = sendData;

	lastRecvIndex = recvDataLen - 1;
	currRecvIndex = 0;
	recvBuffer = recvData;
	
	transferActive = true;
	transferResult = I2C_RC_OK;
	/// The status code of the I2C hardware when transferResult is not I2C_RC_OK.
	i2cHWStatusCode = 0;
	errSeqStatus = STAT_IDLE;

	__asm__ __volatile__ ("nop" ::: "memory");
	// Activate I2C, activate the interrupt, occupy the bus, and send START
	TWCR = 0
	| _BV(TWINT)
	// | _BV(TWEA)
	| _BV(TWSTA)
	// | _BV(TWSTO)
	// | _BV(TWWC)
	| _BV(TWEN)
	| _BV(TWIE)
	;
	
}

void I2CStartTransferReceive (uint8_t slAddr, uint8_t *data, uint8_t dataLen) {
	
	while (transferActive) {
		__asm__ __volatile__ ("nop" ::: "memory");
	}

	currSequence = sequenceReceiveBytes;
	currStatusIndex = STAT_START_SENT;
	currSequenceStatus = &(currSequence[currStatusIndex]);
	slaveAddr = slAddr;
	lastRecvIndex = dataLen - 1;
	currRecvIndex = 0;
	recvBuffer = data;
	
	transferActive = true;
	transferResult = I2C_RC_OK;
	/// The status code of the I2C hardware when transferResult is not I2C_RC_OK.
	i2cHWStatusCode = 0;
	errSeqStatus = STAT_IDLE;

	__asm__ __volatile__ ("nop" ::: "memory");
	// Activate I2C, activate the interrupt, occupy the bus, and send START
	TWCR = 0
	| _BV(TWINT)
	// | _BV(TWEA)
	| _BV(TWSTA)
	// | _BV(TWSTO)
	// | _BV(TWWC)
	| _BV(TWEN)
	| _BV(TWIE)
	;
	
}

enum I2CTransferResult waitGetTransferResult(
	uint8_t *pHWStatus,
	enum I2CStatus *pErrSeqStatus) {

	while (transferActive) {
		__asm__ __volatile__ ("nop" ::: "memory");
	}
	
	*pHWStatus = i2cHWStatusCode;
	*pErrSeqStatus = errSeqStatus;
	return transferResult;
}

bool I2CIsTransferActive() {
	return transferActive;
}


/// Interrupt handler 
ISR(TWI_vect){

	uint8_t actualHWStatus;
	uint8_t expectedHWStatus;
	enum I2CAction action;
	
	if (!currSequence || !currSequenceStatus) {
		// Switch I2C off.
		TWCR = 0;
		transferActive = false;
		currSequence = currSequenceStatus = NULL;
		return;
	}
	
	// When reading the byte is ready to be picked up.
	if (currStatusIndex == STAT_LAST_BYTE_RECEIVED || currStatusIndex == STAT_BYTE_RECEIVED) {
		recvBuffer[currRecvIndex] = TWDR;
		
		currRecvIndex++;
	}
	
	// Read the HW status register.
	// Mask out the lower two bits as these are the prescaler.
	actualHWStatus = TWSR & ~0b11;

	// Evaluate the expected code and the hardware status code
	// First unconditional error conditions
	expectedHWStatus = currSequenceStatus->expectStatusCode;
	if (expectedHWStatus == 0) {
		// Expected status 0 means I reached an illegal status. 
		// This is due to an error of the status diagram definition, i.e. a program error.
		transferResult = I2C_RC_ILLEGAL_SEQ;
		errSeqStatus = currStatusIndex;
		action = currSequenceStatus->nextActionNOK;
		currStatusIndex = currSequenceStatus->nextStatusNOK;
	} else { // if (expectedHWStatus == 0)
		// Expected status 0xff means that the HW status is irrelevant.
		if ((expectedHWStatus == 0xff) || (expectedHWStatus == actualHWStatus)) {
			// Either the status does not matter or the expected status was assumed.
			action = currSequenceStatus->nextActionOK;
			currStatusIndex = currSequenceStatus->nextStatusOK;
		} else { // if ((expectedHWStatus == 0xff) || (expectedHWStatus == actualHWStatus))
			// A different HW status is assumed than the expected one.
			// Set error flags and codes.
			// Perform cleanup, i.e. 
			transferResult = I2C_RC_HW_ERROR;
			errSeqStatus = currStatusIndex;
			i2cHWStatusCode = actualHWStatus;
			action = currSequenceStatus->nextActionNOK;
			currStatusIndex = currSequenceStatus->nextStatusNOK;
		} // if ((expectedHWStatus == 0xff) || (expectedHWStatus == actualHWStatus))
	} // if (expectedHWStatus == 0)

	// Here's one irregularity: When I am about to receive the last
	// Character I must respond the slave with a NAK.
	if (action == ACT_RECV_BYTE_ACK && currRecvIndex == lastRecvIndex) {
		action = ACT_RECV_BYTE_NACK;
		currStatusIndex = STAT_LAST_BYTE_RECEIVED;
	}

	// And now: Action!
	switch (action) {
		case ACT_NONE:
			// Switch I2C off.
			TWCR = 0;
			transferActive = false;
			// The status diagram ended.
			currSequence = currSequenceStatus = NULL;
			break;
			
		case ACT_SEND_START:
			TWCR = 0
				| _BV(TWINT)
				// | _BV(TWEA)
				| _BV(TWSTA)
				// | _BV(TWSTO)
				// | _BV(TWWC)
				| _BV(TWEN)
				| _BV(TWIE)
				;

			break;
			
		case ACT_SEND_ADDR_W:
			TWDR = i2CAddrByte(slaveAddr,false);
			TWCR = 0
			| _BV(TWINT)
			// | _BV(TWEA)
			// | _BV(TWSTA)
			// | _BV(TWSTO)
			// | _BV(TWWC)
			| _BV(TWEN)
			| _BV(TWIE)
			;

			break;
			
		case ACT_SEND_ADDR_R:
			TWDR = i2CAddrByte(slaveAddr,true);
			TWCR = 0
			| _BV(TWINT)
			// | _BV(TWEA)
			// | _BV(TWSTA)
			// | _BV(TWSTO)
			// | _BV(TWWC)
			| _BV(TWEN)
			| _BV(TWIE)
			;

			break;

		case ACT_SEND_BYTE:
			// Send the register number to read
			TWDR = sendBuffer[currSendIndex];
			TWCR = 0
			| _BV(TWINT)
			// | _BV(TWEA)
			// | _BV(TWSTA)
			// | _BV(TWSTO)
			// | _BV(TWWC)
			| _BV(TWEN)
			| _BV(TWIE)
			;
			
			if (lastSendIndex == currSendIndex) {
				currStatusIndex = STAT_LAST_BYTE_SENT;
			} else {
				currSendIndex++;
			}

			break;

		case ACT_RECV_BYTE_ACK:
			// Receive byte, send ACK to slave
			TWCR = 0
			| _BV(TWINT)
			| _BV(TWEA)
			// | _BV(TWSTA)
			// | _BV(TWSTO)
			// | _BV(TWWC)
			| _BV(TWEN)
			| _BV(TWIE)
			;

			break;

		case ACT_RECV_BYTE_NACK:
			// Receive byte, send NACK to slave
			TWCR = 0
			| _BV(TWINT)
			// | _BV(TWEA)
			// | _BV(TWSTA)
			// | _BV(TWSTO)
			// | _BV(TWWC)
			| _BV(TWEN)
			| _BV(TWIE)
			;

			break;

		case ACT_SEND_STOP:
			// Send stop
			TWCR = 0
			| _BV(TWINT)
			// | _BV(TWEA)
			// | _BV(TWSTA)
			| _BV(TWSTO)
			// | _BV(TWWC)
			| _BV(TWEN)
			| _BV(TWIE)
			;

			transferActive = false;

			break;

	}
	
	// proceed to the status for the next round.
	currSequenceStatus = &(currSequence[currStatusIndex]);
	
}
