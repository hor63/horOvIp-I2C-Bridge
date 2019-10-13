/*
 * I2C.h
 *
 * Created: 06.10.2019 18:30:59
 *  Author: kai_horstmann
 */ 


#ifndef I2C_H_
#define I2C_H_

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>


enum I2CTransferResult {
	I2C_RC_OK,
	I2C_RC_ILLEGAL_SEQ,
	I2C_RC_HW_ERROR,
	I2C_RC_UNDEF
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


void I2CStartTransferSend (uint8_t slAddr, uint8_t *data, uint8_t dataLen);
void I2CStartTransferSendReceive (
	uint8_t slAddr,
	uint8_t *sendData,uint8_t sendDataLen,
	uint8_t *recvData,uint8_t recvDataLen);
void I2CStartTransferReceive (uint8_t slAddr, uint8_t *data, uint8_t dataLen);	
bool I2CIsTransferActive();
enum I2CTransferResult waitGetTransferResult(
	uint8_t *pHWStatus,
	enum I2CStatus *pErrSeqStatus);

#endif /* I2C_H_ */