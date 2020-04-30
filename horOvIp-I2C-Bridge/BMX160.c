/*
 * BMX160.c
 *
 * Created: 13.10.2019 17:09:25
 *  Author: kai_horstmann
 */ 

#include "config.h"

#include <setjmp.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include <string.h>

#include "BMX160.h"
#include "serDebugOut.h"
#include "timers.h"
#include "I2C.h"

#include "BMX160defs.h"

extern volatile bool mainLoopMustRun;

// Constants used in the compensation calculations
/**\name OVERFLOW DEFINITIONS  */
#define BMM150_XYAXES_FLIP_OVERFLOW_ADCVAL	INT16_C(-4096)
#define BMM150_ZAXIS_HALL_OVERFLOW_ADCVAL	INT16_C(-16384)
#define BMM150_OVERFLOW_OUTPUT			INT16_C(-32768)
#define BMM150_NEGATIVE_SATURATION_Z            INT16_C(-32767)
#define BMM150_POSITIVE_SATURATION_Z            UINT16_C(32767)


// Define the actual ODR values for the system ODR defined in config.h
#if BMX160ODR == 25
	#define BMX160_ACC_ODR BMX160_ACC_ODR_25
	#define BMX160_GYR_BMX160_GYR_ODR ODR_25
	#define BMX160_MAG_IF_ODR BMX160_MAG_IF_ODR_25
#else // #if BMX160ODR == 25
	#if BMX160ODR == 50
		#define BMX160_ACC_ODR BMX160_ACC_ODR_50
		#define BMX160_GYR_ODR BMX160_GYR_ODR_50
		#define BMX160_MAG_IF_ODR BMX160_MAG_IF_ODR_50
	#else // #if BMX160ODR == 50
		#if BMX160ODR == 100
			#define BMX160_ACC_ODR BMX160_ACC_ODR_100
			#define BMX160_GYR_BMX160_GYR_ODR ODR_100
			#define BMX160_MAG_IF_ODR BMX160_MAG_IF_ODR_100
		#else // #if BMX160ODR == 100
			#error BMX160ODR must be 25, 50, 100
		#endif // #if BMX160ODR == 100
	#endif // #if BMX160ODR == 50
#endif // #if BMX160ODR == 25

static struct BMX160Data bmx160Data;

static bool dataValid = false;
static bool transferRunning = false;

static uint8_t dataBuf [32];

static struct TimerTickCallbChain bmx160timerChainItem;

// Raw trim registers are being read at the startup
static uint8_t trim_x1y1[2] = {0};
static uint8_t trim_xyz_data[4] = {0};
static uint8_t trim_xy1xy2[10] = {0};


static bool waitI2CTransfer(const char* transferAction) {
enum I2CTransferResult rc;
uint8_t HWStatus;
enum I2CStatus errSeqStatus;
	
	rc = I2CWaitGetTransferResult(
	&HWStatus,
	&errSeqStatus);

	if (rc == I2C_RC_OK) {
		return true;
	} else {
		DEBUG_OUT(transferAction);
		DEBUG_OUT (" not OK. rc = ");
		DEBUG_UINT_OUT(rc);
		DEBUG_OUT (", HW status = ");
		DEBUG_UINT_OUT(HWStatus);
		DEBUG_OUT (", Status diagram position = ");
		DEBUG_UINT_OUT(errSeqStatus);
		DEBUG_OUT("\r\n");
		return false;
	}

}

static void wait_mag_man_op_finished() {

	// Wait for the mag_man_op flag to clear
	do {
		dataBuf[0] = BMX160_STATUS_REG;
		dataBuf[1] = 0;
		I2CStartTransferSendReceive(BMX160ADDR,dataBuf,1,dataBuf+1,1,NULL);
		waitI2CTransfer("Read status reg");
	} while (dataBuf[1] & (1 << BMX160_STATUS_MAG_MAN_OP));
	
}

static void readMagRegisters(uint8_t *buf,uint8_t reg,uint8_t len) {
	uint8_t lenCode;
	
	switch (len) {
		case 1:
			lenCode = BMX160_MAG_IF_MAG_READ_BURST_LEN_1;
			break;
		case 2:
			lenCode = BMX160_MAG_IF_MAG_READ_BURST_LEN_2;
			break;
		case 6:
			lenCode = BMX160_MAG_IF_MAG_READ_BURST_LEN_6;
			break;
		case 8:
			lenCode = BMX160_MAG_IF_MAG_READ_BURST_LEN_8;
			break;
		default:
			memset (buf,0,len);
			return;
	}
	
	// Set the burst read len
	dataBuf[0] = BMX160_MAG_IF_CONF_REG;
	dataBuf[1] = (1<<BMX160_MAG_IF_MAG_MAN_EN_BIT) | ((lenCode & BMX160_MAG_IF_MAG_READ_BURST_LEN_MASK) << BMX160_MAG_IF_MAG_READ_BURST_LEN_LWB);
	I2CStartTransferSend(BMX160ADDR,dataBuf,2,NULL);
	waitI2CTransfer("Set MAG read burst len");
	_delay_us(10);

	dataBuf[0] = BMX160_MAG_IF_READ_ADDR_REG;
	dataBuf[1] = reg;
	I2CStartTransferSend(BMX160ADDR,dataBuf,2,NULL);
	waitI2CTransfer("Set MAG read addr reg");
	_delay_us(10);

	// Wait for the transfer to finish
	wait_mag_man_op_finished();

	// Now the data reside in the MAG registers of the BMX160
	dataBuf[0] = BMX160_DATA_REG + BMX160_DATA_MAG_X_LSB_OFFS;
	I2CStartTransferSendReceive(BMX160ADDR,dataBuf,1,buf,len,NULL);
	waitI2CTransfer("Read Mag registers");
	
}

static void resetSensor() {

	// Send the soft reset command
	do {
		dataBuf[0] = BMX160_CMD_REG;
		dataBuf[1] = BMX160_CMD_SOFT_RESET;
		I2CStartTransferSend(BMX160ADDR,dataBuf,2,NULL);
		waitI2CTransfer("Soft reset BMX160");
		_delay_ms(BMX160_CMD_RESET_WAIT_TIME);

		// read out the error register
		dataBuf[0] = BMX160_ERR_REG;
		I2CStartTransferSendReceive (
		BMX160ADDR,
		dataBuf,2,
		dataBuf,1,
		NULL);
		waitI2CTransfer("Read Error register");
	} while (dataBuf[0] & BMX160_ERR_DROP_CMD_ERR_BIT);

	DEBUG_OUT ("Soft reset");
	DEBUG_OUT (". Last error code = ");
	DEBUG_UINT_HEX_OUT(dataBuf[0]);
	DEBUG_OUT("\r\n");

}

static void powerAllSensorsOn() {
	
	// Switch on the accelerometer
	do {
		dataBuf[0] = BMX160_CMD_REG;
		dataBuf[1] = BMX160_CMD_SET_ACC_PMU_MODE(BMX160_PMU_STATUS_ACC_NORMAL);
		I2CStartTransferSend(BMX160ADDR,dataBuf,2,NULL);
		waitI2CTransfer("Set Accel PMU Mode normal");
		_delay_ms(BMX160_CMD_SET_ACC_PMU_MODE_WAIT_TIME);

		// read out the error register
		dataBuf[0] = BMX160_ERR_REG;
		I2CStartTransferSendReceive (
		BMX160ADDR,
		dataBuf,2,
		dataBuf,1,
		NULL);
		waitI2CTransfer("Read Error register");
	} while (dataBuf[0] & BMX160_ERR_DROP_CMD_ERR_BIT);

	DEBUG_OUT ("Accel");
	DEBUG_OUT (" up. Last error code = ");
	DEBUG_UINT_HEX_OUT(dataBuf[0]);
	DEBUG_OUT("\r\n");
	
	// Switch on the gyro
	do {
		dataBuf[0] = BMX160_CMD_REG;
		dataBuf[1] = BMX160_CMD_SET_GYR_PMU_MODE(BMX160_PMU_STATUS_GYR_NORMAL);
		I2CStartTransferSend(BMX160ADDR,dataBuf,2,NULL);
		waitI2CTransfer("Set Gyro PMU Mode normal");
		_delay_ms(BMX160_CMD_SET_GYR_PMU_MODE_WAIT_TIME);

		// read out the error register
		dataBuf[0] = BMX160_ERR_REG;
		I2CStartTransferSendReceive (
		BMX160ADDR,
		dataBuf,1,
		dataBuf,1,
		NULL);
		waitI2CTransfer("Read Error register");
	} while (dataBuf[0] & BMX160_ERR_DROP_CMD_ERR_BIT);
	
	DEBUG_OUT ("Gyro");
	DEBUG_OUT (" up. Last error code = ");
	DEBUG_UINT_HEX_OUT(dataBuf[0]);
	DEBUG_OUT("\r\n");
	
	// Sequence see BMX160 datasheet V1.2, section 2.4.3.1.3, pg. 25
	// Switch on the magnetometer interface
	do {
		dataBuf[0] = BMX160_CMD_REG;
		dataBuf[1] = BMX160_CMD_SET_MAG_PMU_MODE(BMX160_PMU_STATUS_MAG_IF_NORMAL);
		I2CStartTransferSend(BMX160ADDR,dataBuf,2,NULL);
		waitI2CTransfer("Set Mag PMU Mode normal");
		_delay_ms(BMX160_CMD_SET_MAG_IF_PMU_MODE_WAIT_TIME);

		// read out the error register
		dataBuf[0] = BMX160_ERR_REG;
		I2CStartTransferSendReceive (
		BMX160ADDR,
		dataBuf,1,
		dataBuf,1,
		NULL);
		waitI2CTransfer("Read Error register");
	} while (dataBuf[0] & BMX160_ERR_DROP_CMD_ERR_BIT);

	DEBUG_OUT ("Mag interface");
	DEBUG_OUT (" up. Last error code = ");
	DEBUG_UINT_HEX_OUT(dataBuf[0]);
	DEBUG_OUT("\r\n");
		

}

static void configAccel() {

	dataBuf[0] = BMX160_ACC_CONF_REG;
	dataBuf[1] = 0
	| (BMX160_ACC_ODR << BMX160_ACC_ODR_LWB)
	| (BMX160_ACC_BWP_NORMAL << BMX160_ACC_BWP_LWB)
	// No undersampling | (BMX160_ACC_US_ENABLE << BMX160_ACC_US_BIT)
	;
	I2CStartTransferSend(BMX160ADDR,dataBuf,2,NULL);
	waitI2CTransfer("Set Accel configuration");
	_delay_ms(10);

	dataBuf[0] = BMX160_ACC_RANGE_REG;
	dataBuf[1] = BMX160_ACC_RANGE_4G;
	I2CStartTransferSend(BMX160ADDR,dataBuf,2,NULL);
	waitI2CTransfer("Set Accel rate");
	_delay_ms(10);

}

static void configGyro() {

	dataBuf[0] = BMX160_GYR_CONF_REG;
	dataBuf[1] = 0
	| (BMX160_GYR_ODR << BMX160_GYR_ODR_LWB)
	| (BMX160_GYR_BWP_NORMAL << BMX160_GYR_BWP_LWB)
	;
	I2CStartTransferSend(BMX160ADDR,dataBuf,2,NULL);
	waitI2CTransfer("Set gyro configuration");
	_delay_ms(10);

	dataBuf[0] = BMX160_GYR_RANGE_REG;
	dataBuf[1] = BMX160_GYR_RANGE_250_D_S;
	I2CStartTransferSend(BMX160ADDR,dataBuf,2,NULL);
	waitI2CTransfer("Set gyro rate");
	_delay_ms(10);

}

static void configMag () {

	// Switch interface to manual mode, offset=0
	dataBuf[0] = BMX160_MAG_IF_CONF_REG;
	dataBuf[1] = 0
		| (1<<BMX160_MAG_IF_MAG_MAN_EN_BIT)
		// | ((readOffset&BMX160_MAG_IF_MAG_TRIG_READ_OFFS_MASK) << BMX160_MAG_IF_MAG_TRIG_READ_OFFS_LWB)
		;
	I2CStartTransferSend(BMX160ADDR,dataBuf,2,NULL);
	waitI2CTransfer("Set Mag_IF to manual mode");
	_delay_us(10);

	// Mag mode 
	dataBuf[0] = BMX160_MAG_IF_WRITE_DATA_REG;
	dataBuf[1] = 0x01;
	I2CStartTransferSend(BMX160ADDR,dataBuf,2,NULL);
	waitI2CTransfer("Set Mag_IF Magic");
	_delay_us(10);

	// ... to sleep mode
	dataBuf[0] = BMX160_MAG_IF_WRITE_ADDR_REG;
	dataBuf[1] = 0x4b;
	I2CStartTransferSend(BMX160ADDR,dataBuf,2,NULL);
	waitI2CTransfer("Set Mag_IF Magic");
	_delay_us(10);
	wait_mag_man_op_finished();
	_delay_ms(100);

	// Mag REPXY preset 
	dataBuf[0] = BMX160_MAG_IF_WRITE_DATA_REG;
	dataBuf[1] = 0x04;
	I2CStartTransferSend(BMX160ADDR,dataBuf,2,NULL);
	waitI2CTransfer("Set Mag_IF Magic");
	_delay_us(10);
	
	// ... to normal
	dataBuf[0] = BMX160_MAG_IF_WRITE_ADDR_REG;
	dataBuf[1] = 0x51;
	I2CStartTransferSend(BMX160ADDR,dataBuf,2,NULL);
	waitI2CTransfer("Set Mag_IF Magic");
	_delay_us(10);
	wait_mag_man_op_finished();
	
	// Mag REPZ preset
	dataBuf[0] = BMX160_MAG_IF_WRITE_DATA_REG;
	dataBuf[1] = 0x0e;
	I2CStartTransferSend(BMX160ADDR,dataBuf,2,NULL);
	waitI2CTransfer("Set Mag_IF Magic");
	_delay_us(10);
	
	// ... to normal
	dataBuf[0] = BMX160_MAG_IF_WRITE_ADDR_REG;
	dataBuf[1] = 0x52;
	I2CStartTransferSend(BMX160ADDR,dataBuf,2,NULL);
	waitI2CTransfer("Set Mag_IF Magic");
	_delay_us(10);
	wait_mag_man_op_finished();
	
	// prepare mag_if for data mode 1
	dataBuf[0] = BMX160_MAG_IF_WRITE_DATA_REG;
	dataBuf[1] = 0x02;
	I2CStartTransferSend(BMX160ADDR,dataBuf,2,NULL);
	waitI2CTransfer("Set Mag_IF Magic");
	_delay_us(10);
	
	// prepare mag_if for data mode 2
	dataBuf[0] = BMX160_MAG_IF_WRITE_ADDR_REG;
	dataBuf[1] = 0x4c;
	I2CStartTransferSend(BMX160ADDR,dataBuf,2,NULL);
	waitI2CTransfer("Set Mag_IF Magic");
	_delay_us(10);
	wait_mag_man_op_finished();
	
	// prepare mag_if for data mode 3
	dataBuf[0] = BMX160_MAG_IF_READ_ADDR_REG;
	dataBuf[1] = 0x42;
	I2CStartTransferSend(BMX160ADDR,dataBuf,2,NULL);
	waitI2CTransfer("Set Mag_IF Magic");
	_delay_us(10);
	wait_mag_man_op_finished();

    // Data rate to 50 Hz	
	dataBuf[0] = BMX160_MAG_IF_ODR_REG;
	dataBuf[1] = BMX160_MAG_IF_ODR;
	I2CStartTransferSend(BMX160ADDR,dataBuf,2,NULL);
	waitI2CTransfer("Set Mag_IF data rate");
	_delay_us(10);
	
	// Set the mag_if to data mode
	dataBuf[0] = BMX160_MAG_IF_CONF_REG;
	dataBuf[1] = 0;
	I2CStartTransferSend(BMX160ADDR,dataBuf,2,NULL);
	waitI2CTransfer("Set Mag_IF to data mode");
	_delay_us(10);
	

	// Set mag_if to low-power mode
	dataBuf[0] = BMX160_CMD_REG;
	dataBuf[1] = BMX160_CMD_SET_MAG_PMU_MODE(BMX160_PMU_STATUS_MAG_IF_LOW_PWR);
	I2CStartTransferSend(BMX160ADDR,dataBuf,2,NULL);
	waitI2CTransfer("Set MAG_IF to low power");
	
	_delay_ms(BMX160_CMD_SET_MAG_IF_PMU_MODE_WAIT_TIME);
	
}

static inline int16_t readMagValueXY(uint8_t *lowByte) {

	int16_t rc;
	
	rc = *((int8_t*)(lowByte+1));
	rc *= 32;
	rc |= ((*lowByte) >> 0x03);
	
	return rc;
}

static inline int16_t readMagValueZ(uint8_t *lowByte) {

	int16_t rc;
	
	rc = *((int8_t*)(lowByte+1));
	rc *= 128;
	rc |= ((*lowByte) >> 0x01);
	
	return rc;
}

static inline int16_t readMagValueRHall(uint8_t *lowByte) {

	int16_t rc;
	
	rc = *((int8_t*)(lowByte+1));
	rc *= 64;
	rc |= ((*lowByte) >> 0x02);
	
	return rc;
}

static void readSensorDataWOMagCallb (
		enum I2CTransferResult transferResult,
		uint8_t i2cHWStatusCode,
		enum I2CStatus errSeqStatus) {

	bmx160Data.header.unionCode = BMX160DATA_ACC_GYR;
	bmx160Data.header.sensorTime0 = dataBuf[BMX160_SENSORTIME_BYTE_0 + BMX160_SENSORTIME_REG - BMX160_DATA_REG - BMX160_DATA_GYR_X_LSB_OFFS];
	bmx160Data.header.sensorTime1 = dataBuf[BMX160_SENSORTIME_BYTE_1 + BMX160_SENSORTIME_REG - BMX160_DATA_REG - BMX160_DATA_GYR_X_LSB_OFFS];
	bmx160Data.header.sensorTime2 = dataBuf[BMX160_SENSORTIME_BYTE_2 + BMX160_SENSORTIME_REG - BMX160_DATA_REG - BMX160_DATA_GYR_X_LSB_OFFS];
	bmx160Data.header.length = sizeof(bmx160Data.header) + sizeof(bmx160Data.accGyrData);

	bmx160Data.accGyrData.accX = *((int16_t*)(dataBuf + (BMX160_DATA_ACC_X_LSB_OFFS - BMX160_DATA_GYR_X_LSB_OFFS)));
	bmx160Data.accGyrData.accY = *((int16_t*)(dataBuf + (BMX160_DATA_ACC_Y_LSB_OFFS - BMX160_DATA_GYR_X_LSB_OFFS)));
	bmx160Data.accGyrData.accZ = *((int16_t*)(dataBuf + (BMX160_DATA_ACC_Z_LSB_OFFS - BMX160_DATA_GYR_X_LSB_OFFS)));

	bmx160Data.accGyrData.gyrX = *((int16_t*)(dataBuf + (BMX160_DATA_GYR_X_LSB_OFFS - BMX160_DATA_GYR_X_LSB_OFFS)));
	bmx160Data.accGyrData.gyrY = *((int16_t*)(dataBuf + (BMX160_DATA_GYR_Y_LSB_OFFS - BMX160_DATA_GYR_X_LSB_OFFS)));
	bmx160Data.accGyrData.gyrZ = *((int16_t*)(dataBuf + (BMX160_DATA_GYR_Z_LSB_OFFS - BMX160_DATA_GYR_X_LSB_OFFS)));

	dataValid = true;
	transferRunning = false;
	mainLoopMustRun = true;

/*
	DEBUG_OUT("Sensor data no Mag, len = ");
	DEBUG_UINT_OUT(bmx160Data.header.length);
	DEBUG_OUT(", AccX = ");
	DEBUG_INT_OUT(bmx160Data.accGyrData.accX);
	DEBUG_OUT(", GyrX = ");
	DEBUG_INT_OUT(bmx160Data.accGyrData.gyrX);
	DEBUG_OUT("\r\n");
*/

}

static void readSensorDataWithMagCallb (
		enum I2CTransferResult transferResult,
		uint8_t i2cHWStatusCode,
		enum I2CStatus errSeqStatus) {

	bmx160Data.header.unionCode = BMX160DATA_ACC_GYR_MAG;
	bmx160Data.header.sensorTime0 = dataBuf[BMX160_SENSORTIME_BYTE_0 + BMX160_SENSORTIME_REG - BMX160_DATA_REG];
	bmx160Data.header.sensorTime1 = dataBuf[BMX160_SENSORTIME_BYTE_1 + BMX160_SENSORTIME_REG - BMX160_DATA_REG];
	bmx160Data.header.sensorTime2 = dataBuf[BMX160_SENSORTIME_BYTE_2 + BMX160_SENSORTIME_REG - BMX160_DATA_REG];
	bmx160Data.header.length = sizeof(bmx160Data.header) + sizeof(bmx160Data.accGyrMagData);

	bmx160Data.accGyrMagData.accX = *((int16_t*)(dataBuf + (BMX160_DATA_ACC_X_LSB_OFFS)));
	bmx160Data.accGyrMagData.accY = *((int16_t*)(dataBuf + (BMX160_DATA_ACC_Y_LSB_OFFS)));
	bmx160Data.accGyrMagData.accZ = *((int16_t*)(dataBuf + (BMX160_DATA_ACC_Z_LSB_OFFS)));

	bmx160Data.accGyrMagData.gyrX = *((int16_t*)(dataBuf + (BMX160_DATA_GYR_X_LSB_OFFS)));
	bmx160Data.accGyrMagData.gyrY = *((int16_t*)(dataBuf + (BMX160_DATA_GYR_Y_LSB_OFFS)));
	bmx160Data.accGyrMagData.gyrZ = *((int16_t*)(dataBuf + (BMX160_DATA_GYR_Z_LSB_OFFS)));

	bmx160Data.accGyrMagData.magX = readMagValueXY(dataBuf + (BMX160_DATA_MAG_X_LSB_OFFS));
	bmx160Data.accGyrMagData.magY = readMagValueXY(dataBuf + (BMX160_DATA_MAG_Y_LSB_OFFS));
	bmx160Data.accGyrMagData.magZ = readMagValueZ(dataBuf + (BMX160_DATA_MAG_Z_LSB_OFFS));
	bmx160Data.accGyrMagData.magRHall = readMagValueRHall(dataBuf + (BMX160_DATA_RHALL_LSB_OFFS));

	dataValid = true;
	transferRunning = false;
	mainLoopMustRun = true;

/*
	DEBUG_OUT("Sensor data with Mag, len = ");
	DEBUG_UINT_OUT(bmx160Data.header.length);
	DEBUG_OUT(", AccX = ");
	DEBUG_INT_OUT(bmx160Data.accGyrMagData.accX);
	DEBUG_OUT(", GyrX = ");
	DEBUG_INT_OUT(bmx160Data.accGyrMagData.gyrX);
	DEBUG_OUT(", MagX = ");
	DEBUG_INT_OUT(bmx160Data.accGyrMagData.magX);
	DEBUG_OUT("\r\n");
*/
}

static void readStatusRegForSensorData();

static void readStatusRegForSensorDataCallb (
		enum I2CTransferResult transferResult,
		uint8_t i2cHWStatusCode,
		enum I2CStatus errSeqStatus) {

	// How often did you read the status before new data were present?
	static uint16_t numReadStatus = 1;


	if (transferResult == I2C_RC_OK) {

		// Check which data are available
		if ((dataBuf[0] & (1<<BMX160_STATUS_DRDY_ACC | 1<<BMX160_STATUS_DRDY_GYR)) == (1<<BMX160_STATUS_DRDY_ACC | 1<<BMX160_STATUS_DRDY_GYR)) {
			// At least Gyro and and Accel data are there.

/*
			DEBUG_OUT("New data present. Number status reads = ");
			DEBUG_UINT_OUT(numReadStatus);
			DEBUG_OUT("\r\n");
*/
			numReadStatus = 1;

			// Rewind the timer because it runs intentionally a bit faster than the sensor cycle.
			timerReset();

			if (dataBuf[0] & (1<<BMX160_STATUS_DRDY_MAG)) {
				// Mag data are present too.
				// Now read the data and the Sensortime registers
				dataBuf[0] = BMX160_DATA_REG;
				I2CStartTransferSendReceive(
						BMX160ADDR,dataBuf,
						1,
						dataBuf,
						BMX160_SENSORTIME_REG + 3 - BMX160_DATA_REG,
						readSensorDataWithMagCallb);
			} else {
				// Read only gyro and accel.
				// Now read the data and the Sensortime registers
				dataBuf[0] = BMX160_DATA_REG + BMX160_DATA_GYR_X_LSB_OFFS;
				I2CStartTransferSendReceive(
						BMX160ADDR,
						dataBuf,
						1,
						dataBuf,
						BMX160_SENSORTIME_REG + 3 - BMX160_DATA_REG - BMX160_DATA_GYR_X_LSB_OFFS,
						readSensorDataWOMagCallb);
			}
		} else {
			// It is too early. Data are not yet available.
			// Read the status until data are present
			readStatusRegForSensorData();
			numReadStatus ++;
		}

	} else { // if (transferResult == I2C_RC_OK)
		// Do it again. Something went wrong
		readStatusRegForSensorData();
		numReadStatus ++;
	} // if (transferResult == I2C_RC_OK)

}

static void readStatusRegForSensorData() {

	dataBuf[0] = BMX160_STATUS_REG;
	I2CStartTransferSendReceive(BMX160ADDR,dataBuf,1,dataBuf,1,readStatusRegForSensorDataCallb);

}

/** Start reading sensor data.
 *
 * This call is asynchronous. Processing depends on cyclic calls to \ref I2CPoll()
 * You can poll the success with \ref BMX160IsDataPresent() or directly \ref BMX160GetData()
 */
static void readoutSensors() {

	// Avoid nesting readouts. Sensor reading should be waaaay faster then the timer but you never know
	if (!transferRunning) {
		dataValid = false;
		transferRunning = true;
		// First read the status until all sensor data are present
		readStatusRegForSensorData();
	}

}

/** \brief Read the raw trim values from the sensor NVRAM
 *
 * Read the trim registers once before the Mag interface is being put into continuous data mode.
 *
 * Copied with some modification from the Bosch BMM150 driver
 *
 *
 */
void readTrimRegistersRawValues()
{

	/* Trim register value is read */
	readMagRegisters(trim_x1y1,BMM150_DIG_X1_REG,2);

	// 4-byte burst is not supported. Therefore 2 2-byte bursts.
	readMagRegisters(trim_xyz_data,BMM150_DIG_Z4_LSB_REG,2);
	readMagRegisters(trim_xyz_data + 2,BMM150_DIG_Z4_LSB_REG + 2,2);
	// The original read 10 bytes.
	// Since the indirect burst read only supports 8 bytes I read 8 and 2 bytes.
	readMagRegisters(trim_xy1xy2,BMM150_DIG_Z2_LSB_REG,8);
	readMagRegisters(trim_xy1xy2+8,BMM150_DIG_Z2_LSB_REG+8,2);
}


static void timerTickOccurred (uint8_t numTicks){
	readoutSensors();
}



void BMX160Init() {

	resetSensor();

	powerAllSensorsOn();

	readTrimRegistersRawValues();

	configAccel();
	configGyro();
	configMag();

}

// Copied with some modification from the Bosch BMM150 driver
void BMX160ReadTrimRegisters()
{
	
	uint16_t temp_msb = 0;

	
	/* Trim data is calculated from the raw values */
	bmx160Data.trimData.dig_x1 = (int8_t)trim_x1y1[0];
	bmx160Data.trimData.dig_y1 = (int8_t)trim_x1y1[1];
	bmx160Data.trimData.dig_x2 = (int8_t)trim_xyz_data[2];
	bmx160Data.trimData.dig_y2 = (int8_t)trim_xyz_data[3];
	temp_msb = ((uint16_t)trim_xy1xy2[3]) << 8;
	bmx160Data.trimData.dig_z1 = (uint16_t)(temp_msb | trim_xy1xy2[2]);
	temp_msb = ((uint16_t)trim_xy1xy2[1]) << 8;
	bmx160Data.trimData.dig_z2 = (int16_t)(temp_msb | trim_xy1xy2[0]);
	temp_msb = ((uint16_t)trim_xy1xy2[7]) << 8;
	bmx160Data.trimData.dig_z3 = (int16_t)(temp_msb | trim_xy1xy2[6]);
	temp_msb = ((uint16_t)trim_xyz_data[1]) << 8;
	bmx160Data.trimData.dig_z4 = (int16_t)(temp_msb | trim_xyz_data[0]);
	bmx160Data.trimData.dig_xy1 = trim_xy1xy2[9];
	bmx160Data.trimData.dig_xy2 = (int8_t)trim_xy1xy2[8];
	temp_msb = ((uint16_t)(trim_xy1xy2[5] & 0x7F)) << 8;
	bmx160Data.trimData.dig_xyz1 = (uint16_t)(temp_msb | trim_xy1xy2[4]);

	bmx160Data.header.unionCode = BMX160DATA_TRIM;
	bmx160Data.header.sensorTime0 = 0;
	bmx160Data.header.sensorTime1 = 0;
	bmx160Data.header.sensorTime2 = 0;
	bmx160Data.header.length = sizeof(bmx160Data.header) + sizeof(bmx160Data.trimData);

	// Use the content of bmx160Data containing the trim data immediately.
	// Until the next measurement cycle passes consider the data invalid as measurement data, and
	// prevent sending the trim data as cyclic data.
	dataValid = false;

}


void BMX160StartDataCapturing() {

	bmx160timerChainItem.callbFunc = timerTickOccurred;
	bmx160timerChainItem.next = NULL;
	timerAddCallback(&bmx160timerChainItem);

}

void BMX160StopDataCapturing() {

	timerRemoveCallback(timerTickOccurred);

}


bool BMX160IsDataValid(){
	return dataValid;
}

struct BMX160Data* BMX160GetData(){

	return &bmx160Data;
	
}
