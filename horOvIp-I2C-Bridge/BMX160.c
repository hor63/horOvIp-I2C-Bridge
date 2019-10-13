/*
 * BMX160.c
 *
 * Created: 13.10.2019 17:09:25
 *  Author: kai_horstmann
 */ 

#include "config.h"

#include "BMX160.h"
#include "serDebugOut.h"
#include <util/delay.h>

#define BMX160ADDR 0x68

static uint8_t dataBuf [128];

void BMX160Test() {

	enum I2CTransferResult rc;
	uint8_t HWStatus;
	enum I2CStatus errSeqStatus;

	while (1) {
		// Address register
		dataBuf[0] = 0;
	
		I2CStartTransferSendReceive (
			BMX160ADDR,
			dataBuf,1,
			dataBuf,1);

		rc = waitGetTransferResult(
			&HWStatus,
			&errSeqStatus);

		if (rc == I2C_RC_OK) {
			DEBUG_OUT ("BMX160 Read ID OK. ID = ");
			DEBUG_UINT_HEX_OUT(dataBuf[0]);
			DEBUG_CHR_OUT('\n');
		} else {
			DEBUG_OUT ("BMX160 Read ID Not OK. rc = ");
			DEBUG_UINT_OUT(rc);
			DEBUG_OUT (" ,HW status = ");
			DEBUG_UINT_OUT(HWStatus);
			DEBUG_OUT (" ,Status diagram position = ");
			DEBUG_UINT_OUT(errSeqStatus);
			DEBUG_CHR_OUT('\n');
		}
		
		
		// Address register
		dataBuf[0] = 0x18;
				
		I2CStartTransferSendReceive (
		BMX160ADDR,
		dataBuf,1,
		dataBuf,3);

		rc = waitGetTransferResult(
		&HWStatus,
		&errSeqStatus);

		// Start the next transfer again right away
		// Test transfers without delays
		dataBuf[3] = 0x18;
		
		I2CStartTransferSendReceive (
		BMX160ADDR,
		&(dataBuf[3]),1,
		&(dataBuf[3]),3);

		if (rc == I2C_RC_OK) {
			DEBUG_OUT ("BMX160 Read Counter OK. Value = ");
			DEBUG_UINT_HEX_OUT(dataBuf[2]);
			DEBUG_UINT_HEX_OUT(dataBuf[1]);
			DEBUG_UINT_HEX_OUT(dataBuf[0]);
			DEBUG_CHR_OUT('\n');
		} else {
			DEBUG_OUT ("BMX160 Read Counter Not OK. rc = ");
			DEBUG_UINT_OUT(rc);
			DEBUG_OUT (" ,HW status = ");
			DEBUG_UINT_OUT(HWStatus);
			DEBUG_OUT (" ,Status diagram position = ");
			DEBUG_UINT_OUT(errSeqStatus);
			DEBUG_CHR_OUT('\n');
		}

		rc = waitGetTransferResult(
		&HWStatus,
		&errSeqStatus);

		if (rc == I2C_RC_OK) {
			DEBUG_OUT ("BMX160 Read Counter OK. Value = ");
			DEBUG_UINT_HEX_OUT(dataBuf[5]);
			DEBUG_UINT_HEX_OUT(dataBuf[4]);
			DEBUG_UINT_HEX_OUT(dataBuf[3]);
			DEBUG_CHR_OUT('\n');
			} else {
			DEBUG_OUT ("BMX160 Read Counter Not OK. rc = ");
			DEBUG_UINT_OUT(rc);
			DEBUG_OUT (" ,HW status = ");
			DEBUG_UINT_OUT(HWStatus);
			DEBUG_OUT (" ,Status diagram position = ");
			DEBUG_UINT_OUT(errSeqStatus);
			DEBUG_CHR_OUT('\n');
		}
	
		_delay_ms(500);
	}
	
}
