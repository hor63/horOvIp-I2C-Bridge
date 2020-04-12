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

/** \brief Return code of all calls which send or receive data to or from the I2C slave
 *
 */
enum I2CTransferResult {
	I2C_RC_OK,				//< Transfer successful
	I2C_RC_ILLEGAL_SEQ,		//< An unexpected sequence code was encountered. Reason is a wrong status machine for the transfer.
	I2C_RC_HW_ERROR,		//< Self explanory. Review the hardware error value returned from the call to this module.
	I2C_RC_UNDEF			//< Unspecified error. Someting went wrong :)
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

/** \brief Initialize the interface and start the driver thread
 *
 * This function **MUST** be called before any other call of this module.
 * For efficiency there are not further checks if the initialization was done.
 *
 * @return true when successful, false if an error occurred. If false was returned the interface is unusable!
 */
bool I2CInit();

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
 * @param[out] i2cHWStatusCode The status code of the I2C hardware when return value is not I2C_RC_OK.
 * Not written/undefined when return value is I2C_RC_OK
 * You can pass NULL if you are not interested.
 * @see See the [ATMega1284P datasheet](http://ww1.microchip.com/downloads/en/DeviceDoc/ATmega164A_PA-324A_PA-644A_PA-1284_P_Data-Sheet-40002070A.pdf),
 * tables 21-3, pg. 224 and 21-4, pg. 227
 * @param[out] errSeqStatus The sequence status when an error occurred
 * You can pass NULL if you are not interested.
 * @return the transfer result. \p pHWStatus and \p pErrSeqStatus provide more information when the result is not I2C_RC_OK
 */
enum I2CTransferResult I2CTransferSend (
		uint8_t slAddr,
		uint8_t *data,
		uint8_t dataLen,
		uint8_t *pHWStatus,
		enum I2CStatus *pErrSeqStatus);

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
 * @param[out] i2cHWStatusCode The status code of the I2C hardware when transferResult is not I2C_RC_OK.
 * You can pass NULL if you are not interested.
 * \see Refer to the [ATMega1284P datasheet](http://ww1.microchip.com/downloads/en/DeviceDoc/ATmega164A_PA-324A_PA-644A_PA-1284_P_Data-Sheet-40002070A.pdf),
 * tables 21-3, pg. 224 and 21-4, pg. 227
 * @param[out] errSeqStatus The sequence status when an error occurred
 * You can pass NULL if you are not interested.
 * @return the transfer result. \p pHWStatus and \p pErrSeqStatus provide more information when the result is not I2C_RC_OK
 */
enum I2CTransferResult I2CTransferSendReceive (
	uint8_t slAddr,
	uint8_t *sendData,uint8_t sendDataLen,
	uint8_t *recvData,uint8_t recvDataLen,
	uint8_t *pHWStatus,
	enum I2CStatus *pErrSeqStatus);

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
 * @param[out] i2cHWStatusCode The status code of the I2C hardware when transferResult is not I2C_RC_OK.
 * You can pass NULL if you are not interested.
 * \see Refer to the [ATMega1284P datasheet](http://ww1.microchip.com/downloads/en/DeviceDoc/ATmega164A_PA-324A_PA-644A_PA-1284_P_Data-Sheet-40002070A.pdf),
 * tables 21-3, pg. 224 and 21-4, pg. 227
 * @param[out] errSeqStatus The sequence status when an error occurred
 * You can pass NULL if you are not interested.
 * @return the transfer result. \p pHWStatus and \p pErrSeqStatus provide more information when the result is not I2C_RC_OK
 */
enum I2CTransferResult I2CTransferReceive (uint8_t slAddr,
		uint8_t *data,
		uint8_t dataLen,
		uint8_t *pHWStatus,
		enum I2CStatus *pErrSeqStatus);

#endif /* I2C_H_ */
