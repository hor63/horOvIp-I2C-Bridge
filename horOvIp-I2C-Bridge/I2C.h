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

/** \brief Initialize the I2C module
 *
 * Essentially set the prescaler to 0 and set the data rate to 100kHz
 */
void I2CInit();

/** \brief Pointer to a callback function of the user
 *
 * @param transferResult The the final result of a transfer
 * @param i2cHWStatusCode The status code of the I2C hardware when transferResult is not I2C_RC_OK.
 * Refer to the [ATMega1284P datasheet](http://ww1.microchip.com/downloads/en/DeviceDoc/ATmega164A_PA-324A_PA-644A_PA-1284_P_Data-Sheet-40002070A.pdf),
 * tables 21-3, pg. 224 and 21-4, pg. 227
 * @param errSeqStatus The sequence status when an error occurred
 */
typedef void (*I2CTransferFinishedCallbPtr) (
		enum I2CTransferResult transferResult,
		uint8_t i2cHWStatusCode,
		enum I2CStatus errSeqStatus
);

/** \brief Start a write-only transfer to an I2C slave.
 *
 * Many I2C devices expect the register address where the data are being written at the start of the transfer.
 * The register address format and length is device dependent. Please consult the device data sheet for more information.
 * Simply put the register address at the start of the send buffer before the actual data.
 *
 * The transfer is started, and the function returns immediately.
 * The end of the transfer and the result can either be polled with \ref I2CIsTransferActive()
 * or you can synchonously wait with \ref I2CWaitGetTransferResult(),
 * or you provide a function pointer in \ref callback, and the application will call you back when the transfer is finished.
 *
 * @param slAddr 7-bit I2C slave address
 * @param data Pointer to the data being sent
 * @param dataLen \ref data length in bytes
 * @param callback Pointer to the callback function. Optional. If left NULL you must poll the transfer end yourself
 */
void I2CStartTransferSend (
		uint8_t slAddr,
		uint8_t *data,
		uint8_t dataLen,
		I2CTransferFinishedCallbPtr callback);

/**  \brief Start a write-read transfer to an I2C slave.
 *
 * This kind of transfer is used with I2C slaves where the register address within the device is sent,
 * then the data is read from the device starting at the register address.
 * The register address format and length is device dependent. Please consult the device data sheet for more information.
 *
 * The transfer is started, and the function returns immediately.
 * The end of the transfer and the result can either be polled with \ref I2CIsTransferActive()
 * or you can synchronously wait with \ref I2CWaitGetTransferResult(),
 * or you provide a function pointer in \ref callback, and the application will call you back when the transfer is finished.
 *
 * @param slAddr 7-bit I2C slave address
 * @param sendData Pointer to the data being sent
 * @param sendDataLen \ref sendData length in bytes
 * @param recvData Pointer to the data being sent
 * @param recvDataLen \ref recvData length in bytes
 * @param callback Pointer to the callback function. Optional. If left NULL you must poll the transfer end yourself
 */
void I2CStartTransferSendReceive (
	uint8_t slAddr,
	uint8_t *sendData,uint8_t sendDataLen,
	uint8_t *recvData,uint8_t recvDataLen,
	I2CTransferFinishedCallbPtr callback);

/** \brief Start a read-only transfer to an I2C slave.
 *
 * Please consult your device data sheet how it behaves on a read-only transfer.
 * Some may continue returning data from the register location where the last transfer ended,
 * some may return registers beginning at the start each time
 *
 * The transfer is started, and the function returns immediately.
 * The end of the transfer and the result can either be polled with \ref I2CIsTransferActive()
 * or you can synchonously wait with \ref I2CWaitGetTransferResult(),
 * or you provide a function pointer in \ref callback, and the application will call you back when the transfer is finished.
 *
 * @param slAddr 7-bit I2C slave address
 * @param data Pointer to the buffer for receiving data
 * @param dataLen \ref data length in bytes
 * @param callback Pointer to the callback function. Optional. If left NULL you must poll the transfer end yourself
 */
void I2CStartTransferReceive (uint8_t slAddr,
		uint8_t *data,
		uint8_t dataLen,
		I2CTransferFinishedCallbPtr callback);

/** \brief Check if an I2C transfer is active
 *
 * @return true when a transfer is active.
 */
bool I2CIsTransferActive();

/** \brief Wait until an I2C transfer has finished, and return the transfer results
 *
 * If an I2C transfer is active the function waits in a loop until the running transfer is finished.
 * Within the active wait loop the processor enters sleep mode because the transfer finish flag can only be set by the I2C interrupt handler.
 * If a transfer is already finished when this function is called the function returns immediately and returns the results of the last transfer.
 * If you call the function repeatedly without starting a new transfer the function will return the same results each time.
 *
 * You can poll for a transfer to finish without blocking with \ref I2CIsTransferActive(), and then call I2CWaitGetTransferResult to retrieve the results
 * without delay.
 *
 * @param pHWStatus
 * @param pErrSeqStatus
 * @return
 */
enum I2CTransferResult I2CWaitGetTransferResult(
	uint8_t *pHWStatus,
	enum I2CStatus *pErrSeqStatus);

/** \brief Periodic poll call to call application callbacks when a transfer has finished
 *
 * This function must be called each time in the main loop.
 * It checks if a callback is pending, and if a transfer finished. If both applies the callback is called and the set NULL.
 *
 */
void I2CPoll();

#endif /* I2C_H_ */
