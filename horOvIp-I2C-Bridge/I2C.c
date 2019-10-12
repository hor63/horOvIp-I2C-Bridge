/*
 * I2C.c
 *
 * Created: 06.10.2019 18:33:21
 *  Author: kai_horstmann
 */ 

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include "uip.h"
#include "serDebugOut.h"

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

enum I2CStatus {
	STAT_IDLE,
	STAT_START_SENT,
	STAT_RESTART_SENT,
	STAT_ADDR_W_SENT,
	STAT_ADDR_R_SENT,
	STAT_BYTE_SENT,
	STAT_LAST_BYTE_SENT,
	STAT_BYTE_RECEIVED,
	STAT_LAST_BYTE_RECEIVED,
	STAT_STOP_SENT
};
	
	
typedef struct {
	enum I2CStatus nextStatusOK;
	enum I2CAction nextActionOK;
	enum I2CStatus nextStatusNOK;
	enum I2CAction nextActionNOK;
	uint8_t expectStatusCode;
	} TActionSequence;	
	
static const TActionSequence sequenceSendBytes [] = {
	//STAT_IDLE
	{STAT_START_SENT,ACT_SEND_START,STAT_START_SENT,ACT_SEND_START,0xff},

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
	//STAT_IDLE
	{STAT_START_SENT,ACT_SEND_START,STAT_START_SENT,ACT_SEND_START,0xff},

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
	//STAT_IDLE
	{STAT_START_SENT,ACT_SEND_START,STAT_START_SENT,ACT_SEND_START,0xff},

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

enum I2CTransferResult {
	I2C_RC_OK,
	I2C_RC_ILLEGAL_SEQ,
	I2C_RC_HW_ERROR,
	I2C_RC_UNDEF
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

// Assume I send and receive 3 bytes, the index for the hybrid read is 2 bytes long
uint8_t rcSend [] = {0xff,0x08,0x18,0x28,0x28,0x28,0xff};
uint8_t rcRecvWithIndex [] = {0xff,0x08,0x18,0x28,0x28,0x10,0x40,0x50,0x50,0x58,0xff};
uint8_t rcRecvOnly [] = {0xff,0x08,0x40,0x50,0x50,0x58,0xff};

static uint8_t tstSendBuffer[3];
static uint8_t tstReceiveBuffer[3];


static void testOneSequence();
static void tstISR();
static uint8_t *tstRcList;
static uint8_t tstRcIndex;
	

void I2CTestSequences() {
	
	// Test the write sequence
	currSequence = sequenceSendBytes;
	currStatusIndex = STAT_IDLE;
	currSequenceStatus = &(currSequence[currStatusIndex]);
	slaveAddr = 0x68;
	lastSendIndex = 2;
	currSendIndex = 0;
	sendBuffer = tstSendBuffer;
	
	// Don't worry about receive stuff here.
	
	transferActive = true;
	transferResult = I2C_RC_OK;
	/// The status code of the I2C hardware when transferResult is not I2C_RC_OK.
	i2cHWStatusCode = 0;
	errSeqStatus = STAT_IDLE;

	tstRcList = rcSend;
	tstRcIndex = 0;
	
	
	testOneSequence();
	

	// Test the write/read sequence
	currSequence = sequenceReceiveBytesWithRegisterAdress;
	currStatusIndex = STAT_IDLE;
	currSequenceStatus = &(currSequence[currStatusIndex]);
	slaveAddr = 0x68;
	lastSendIndex = 1;
	currSendIndex = 0;
	sendBuffer = tstSendBuffer;
	lastRecvIndex = 2;
	currRecvIndex = 0;
	recvBuffer = tstReceiveBuffer;
	
	transferActive = true;
	transferResult = I2C_RC_OK;
	/// The status code of the I2C hardware when transferResult is not I2C_RC_OK.
	i2cHWStatusCode = 0;
	errSeqStatus = STAT_IDLE;

	tstRcList = rcRecvWithIndex;
	tstRcIndex = 0;
	
	
	testOneSequence();
	

	// Test the read-only sequence
	currSequence = sequenceReceiveBytes;
	currStatusIndex = STAT_IDLE;
	currSequenceStatus = &(currSequence[currStatusIndex]);
	slaveAddr = 0x68;
	lastSendIndex = 1;
	currSendIndex = 0;
	sendBuffer = tstSendBuffer;
	lastRecvIndex = 2;
	currRecvIndex = 0;
	recvBuffer = tstReceiveBuffer;

	transferActive = true;
	transferResult = I2C_RC_OK;
	/// The status code of the I2C hardware when transferResult is not I2C_RC_OK.
	i2cHWStatusCode = 0;
	errSeqStatus = STAT_IDLE;

	tstRcList = rcRecvOnly;
	tstRcIndex = 0;


	testOneSequence();

}

static void testOneSequence() {
	
	do {
		
		tstISR();
		
		tstRcIndex ++;
		
	} while (currStatusIndex != STAT_IDLE);
	
}

/// Mostly the prototype for the 
static void tstISR() {

	uint8_t addrRW;
	
	uint8_t actualHWStatus;
	enum I2CAction action;
	uint8_t expectedHWStatus;
	// Read the status register.
	// Mask out the lower two bits as these are the prescaler.
	// rc = TWSR & ~0b11;
	actualHWStatus = tstRcList[tstRcIndex]; // Instead
	
	// When reading the by is ready to be picked up.
	if (currStatusIndex == STAT_LAST_BYTE_RECEIVED || currStatusIndex == STAT_BYTE_RECEIVED) {
		recvBuffer[currRecvIndex] = TWDR;
		
		currRecvIndex++;
	}
	
	// Evaluate the expected code and the hardware status code
	// First unconditional error conditions
	expectedHWStatus = currSequenceStatus->expectStatusCode;
	if (expectedHWStatus == 0) {
		// I reached an unexpected status.
		transferResult = I2C_RC_ILLEGAL_SEQ;
		errSeqStatus = currStatusIndex;
		action = currSequenceStatus->nextActionNOK;
		currStatusIndex = currSequenceStatus->nextStatusNOK;
	} else { // if (expectedHWStatus == 0)
		if ((expectedHWStatus == 0xff) || (expectedHWStatus == actualHWStatus)) {
			// Either the status does not matter or the expected status was assumed.
			action = currSequenceStatus->nextActionOK;
			currStatusIndex = currSequenceStatus->nextStatusOK;
			
		} else { // if ((expectedHWStatus == 0xff) || (expectedHWStatus == actualHWStatus))
			transferResult = I2C_RC_HW_ERROR;
			action = currSequenceStatus->nextActionNOK;
			errSeqStatus = currStatusIndex;
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

			break;
			
		case ACT_SEND_START:
			TWCR = 0
				| _BV(TWINT)
				// | TWEA
				| _BV(TWSTA)
				// | TWSTO
				// | TWWC
				| _BV(TWEN)
				// | TWIE
				;

			break;
			
		case ACT_SEND_ADDR_W:
			addrRW = i2CAddrByte(slaveAddr,false);
			TWDR = addrRW;
			TWCR = 0
			| _BV(TWINT)
			// | TWEA
			// | TWSTA
			// | TWSTO
			// | TWWC
			| _BV(TWEN)
			// | TWIE
			;

			break;
			
		case ACT_SEND_ADDR_R:
			addrRW = i2CAddrByte(slaveAddr,true);
			TWDR = addrRW;
			TWCR = 0
			| _BV(TWINT)
			// | TWEA
			// | TWSTA
			// | TWSTO
			// | TWWC
			| _BV(TWEN)
			// | TWIE
			;

			break;

		case ACT_SEND_BYTE:
			// Send the register number to read
			TWDR = sendBuffer[currSendIndex];
			TWCR = 0
			| _BV(TWINT)
			// | TWEA
			// | TWSTA
			// | TWSTO
			// | TWWC
			| _BV(TWEN)
			// | TWIE
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
			// | TWSTA
			// | TWSTO
			// | TWWC
			| _BV(TWEN)
			// | TWIE
			;

			break;

		case ACT_RECV_BYTE_NACK:
			// Receive byte, send NACK to slave
			TWCR = 0
			| _BV(TWINT)
			// | TWEA
			// | TWSTA
			// | TWSTO
			// | TWWC
			| _BV(TWEN)
			// | TWIE
			;

			break;

		case ACT_SEND_STOP:
			// Send stop
			TWCR = 0
			| _BV(TWINT)
			// | TWEA
			// | TWSTA
			| _BV(TWSTO)
			// | TWWC
			| _BV(TWEN)
			// | TWIE
			;

			break;

	}

	
	// proceed to the status for the next round.
	currSequenceStatus = &(currSequence[currStatusIndex]);
	
}

void I2CTest() {
	
	TWBR = TWI_BIT_RATE_REG_VAL;
	
	// Send start condition
	TWCR = 0
		| _BV(TWINT)
		// | TWEA
		| _BV(TWSTA)
		// | TWSTO
		// | TWWC
		| _BV(TWEN)
		// | TWIE
		;
	while (!(TWCR & _BV(TWINT)))	{}
	
	// Send the address and write
	TWDR = i2CAddrByte(0x68,false);
	TWCR = 0
		| _BV(TWINT)
		// | TWEA
		// | TWSTA
		// | TWSTO
		// | TWWC
		| _BV(TWEN)
		// | TWIE
		;
	while (!(TWCR & _BV(TWINT)))	{}
	
	
	DEBUG_OUT ("I2C Sent Address and Write, status = ");
	DEBUG_UINT_HEX_OUT (TWSR);
	DEBUG_CHR_OUT('\n');
	
	// Send the register number to read
	TWDR = 0;
	TWCR = 0
		| _BV(TWINT)
		// | TWEA
		// | TWSTA
		// | TWSTO
		// | TWWC
		| _BV(TWEN)
		// | TWIE
		;
	while (!(TWCR & _BV(TWINT)))	{}
	
	DEBUG_OUT ("I2C Sent 0, status = ");
	DEBUG_UINT_HEX_OUT (TWSR);
	DEBUG_CHR_OUT('\n');
		
	// Send restart condition
	TWCR = 0
		| _BV(TWINT)
		// | TWEA
		| _BV(TWSTA)
		// | TWSTO
		// | TWWC
		| _BV(TWEN)
		// | TWIE
		;
	
	while (!(TWCR & _BV(TWINT)))	{}
	
	
	// Send the address and read
	TWDR = i2CAddrByte(0x68,true);
	TWCR = 0
		| _BV(TWINT)
		// | TWEA
		// | TWSTA
		// | TWSTO
		// | TWWC
		| _BV(TWEN)
		// | TWIE
		;
	while (!(TWCR & _BV(TWINT)))	{}
	
	DEBUG_OUT ("I2C Sent Address and Read, status = ");
	DEBUG_UINT_HEX_OUT (TWSR);
	DEBUG_CHR_OUT('\n');
	
	// Read register 0 with NAK
	TWCR = 0
		| _BV(TWINT)
		// | TWEA
		// | TWSTA
		// | TWSTO
		// | TWWC
		| _BV(TWEN)
		// | TWIE
		;
	while (!(TWCR & _BV(TWINT)))	{}
	
	DEBUG_OUT ("I2C Read register 0, status = ");
	DEBUG_UINT_HEX_OUT (TWSR);
	DEBUG_CHR_OUT('\n');
	DEBUG_OUT ("I2C register 0 value = ");
	DEBUG_UINT_HEX_OUT (TWDR);
	DEBUG_CHR_OUT('\n');

	// Send stop
	TWCR = 0
		| _BV(TWINT)
		// | TWEA
		// | TWSTA
		| _BV(TWSTO)
		// | TWWC
		| _BV(TWEN)
		// | TWIE
		;
	while (!(TWCR & _BV(TWINT)))	{}
	
		
}