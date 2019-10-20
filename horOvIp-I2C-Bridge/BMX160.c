/*
 * BMX160.c
 *
 * Created: 13.10.2019 17:09:25
 *  Author: kai_horstmann
 */ 

#include "config.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>

#include "BMX160.h"
#include "serDebugOut.h"
#include "timers.h"

#define BMX160ADDR 0x68

// Various useful BMX160 definitions
#define BMX160_ERR_REG 0x02
#define BMX160_ERR_DROP_CMD_ERR_BIT 6
#define BMX160_ERR_FATAL_CHIP_ERR_BIT 0
#define BMX160_ERR_CODE_LWB 1
#define BMX160_ERR_CODE_MASK 0b1111

// Power status of Accel, Gyro and Mag-IF
#define BMX160_PMU_STATUS_REG 0x03
#define BMX160_PMU_STATUS_ACC_LWB 4
#define BMX160_PMU_STATUS_ACC_MASK 0b11
#define BMX160_PMU_STATUS_ACC_SUSPEND 0b00
#define BMX160_PMU_STATUS_ACC_NORMAL 0b01
#define BMX160_PMU_STATUS_ACC_LOW_PWR 0b10

#define BMX160_PMU_STATUS_GYR_LWB 2
#define BMX160_PMU_STATUS_GYR_MASK 0b11
#define BMX160_PMU_STATUS_GYR_SUSPEND 0b00
#define BMX160_PMU_STATUS_GYR_NORMAL 0b01
#define BMX160_PMU_STATUS_GYR_RESERVED 0b10
#define BMX160_PMU_STATUS_GYR_FAST_STARTUP 0b11

#define BMX160_PMU_STATUS_MAG_IF_LWB 0
#define BMX160_PMU_STATUS_MAG_IF_MASK 0b11
#define BMX160_PMU_STATUS_MAG_IF_SUSPEND 0b00
#define BMX160_PMU_STATUS_MAG_IF_NORMAL 0b01
#define BMX160_PMU_STATUS_MAG_IF_LOW_PWR 0b10

// Accelerometer
#define BMX160_ACC_CONF_REG 0x40
// Accel Output data rate
#define BMX160_ACC_ODR_LWB 0
#define BMX160_ACC_ODR_MASK 0b1111
#define BMX160_ACC_ODR_25  0b0110
#define BMX160_ACC_ODR_50  0b0111
#define BMX160_ACC_ODR_100 0b1000
#define BMX160_ACC_ODR_200 0b1001
#define BMX160_ACC_ODR_400 0b1010

// Accel Bandwith
#define BMX160_ACC_BWP_LWB 4
#define BMX160_ACC_BWP_MASK 0b111
#define BMX160_ACC_BWP_NORMAL 0b010
#define BMX160_ACC_BWP_OSR2 0b001
#define BMX160_ACC_BWP_OSR4 0b000

/// Accel undersampling mode
#define BMX160_ACC_US_BIT 7
#define BMX160_ACC_US_ENABLE 0b1

// Accel range
#define BMX160_ACC_RANGE_REG 0x41
#define BMX160_ACC_RANGE_MASK 0b1111
#define BMX160_ACC_RANGE_2G  0b0011
#define BMX160_ACC_RANGE_4G  0b0101
#define BMX160_ACC_RANGE_8G  0b1000
#define BMX160_ACC_RANGE_16G 0b1100

// Gyroscope
#define BMX160_GYR_CONF_REG 0x42
// Gyroscope Output data rate
#define BMX160_GYR_ODR_LWB 0
#define BMX160_GYR_ODR_MASK 0b1111
#define BMX160_GYR_ODR_25  0b0110
#define BMX160_GYR_ODR_50  0b0111
#define BMX160_GYR_ODR_100 0b1000
#define BMX160_GYR_ODR_200 0b1001
#define BMX160_GYR_ODR_400 0b1010

#define BMX160_GYR_BWP_LWB 4
#define BMX160_GYR_BWP_MASK 0b11
#define BMX160_GYR_BWP_NORMAL 0b10
#define BMX160_GYR_BWP_OSR2 0b01
#define BMX160_GYR_BWP_OSR4 0b00

// Gyro range
#define BMX160_GYR_RANGE_REG 0x43
#define BMX160_GYR_RANGE_MASK 0b111
// D_S means Degrees per second
#define BMX160_GYR_RANGE_2000_D_S 0b000
#define BMX160_GYR_RANGE_1000_D_S 0b001
#define BMX160_GYR_RANGE_500_D_S  0b010
#define BMX160_GYR_RANGE_250_D_S  0b011
#define BMX160_GYR_RANGE_125_D_S  0b100

// Magnetometer interface
#define BMX160_MAG_IF_ODR_REG 0x44
// MAF_IFoscope Output data rate
#define BMX160_MAG_IF_ODR_LWB 0
#define BMX160_MAG_IF_ODR_MASK 0b1111
#define BMX160_MAG_IF_ODR_25  0b0110
#define BMX160_MAG_IF_ODR_50  0b0111
#define BMX160_MAG_IF_ODR_100 0b1000
#define BMX160_MAG_IF_ODR_200 0b1001
#define BMX160_MAG_IF_ODR_400 0b1010

// Indirect Access to the magnetometer registers
#define BMX160_MAG_IF_CONF_REG 0x4C
// Bit in the register enables the manual access to the magnetometer register bank.
#define BMX160_MAG_IF_MAG_MAN_EN_BIT 7
// Read offset between read-out and next trigger in 2.5 ms units
// Maximum trigger is 0b0000
#define BMX160_MAG_IF_MAG_TRIG_READ_OFFS_LWB 2
#define BMX160_MAG_IF_MAG_TRIG_READ_OFFS_MASK 0b1111
// Read burst length. Values are encoded. See below.
// Read data are being placed into registers BMX160_DATA_REG+BMX160_DATA_MAG_X_LSB_OFFS upward
#define BMX160_MAG_IF_MAG_READ_BURST_LEN_LWB 0
#define BMX160_MAG_IF_MAG_READ_BURST_LEN_MASK 0b11
#define BMX160_MAG_IF_MAG_READ_BURST_LEN_1 0b00
#define BMX160_MAG_IF_MAG_READ_BURST_LEN_2 0b01
#define BMX160_MAG_IF_MAG_READ_BURST_LEN_6 0b10
#define BMX160_MAG_IF_MAG_READ_BURST_LEN_8 0b11

// Fast offset compensation configuration register
#define BMX160_FOC_CONF_REG 0x69
#define BMX160_FOC_CONF_GYR_EN_BIT 6
#define BMX160_FOC_CONF_ACC_X_LWB 4
#define BMX160_FOC_CONF_ACC_X_MASK 0b11
#define BMX160_FOC_CONF_ACC_Y_LWB 2
#define BMX160_FOC_CONF_ACC_Y_MASK 0b11
#define BMX160_FOC_CONF_ACC_Z_LWB 0
#define BMX160_FOC_CONF_ACC_Z_MASK 0b11

// The set values for the offset compensation of the Accel.
// Typically one axis will observe +/-1G, 
// unless you run this thing in the orbit under 0-gravity :)
#define BMX160_FOC_CONF_ACC_DISABLE  0b00
#define BMX160_FOC_CONF_ACC_TARGET_PLUS1G  0b01
#define BMX160_FOC_CONF_ACC_TARGET_PLUS1G 0b01
#define BMX160_FOC_CONF_ACC_TARGET_0G 0b11

// Enable or disable NVM (non-volatile memory) programming
#define BMX160_CONF_REG 0x69
#define BMX160_NVM_PROG_EN_BIT 1

// Accel and Gyro offset values and configuration
#define BMX160_OFFS_ACC_X_REG 0x71
#define BMX160_OFFS_ACC_Y_REG 0x72
#define BMX160_OFFS_ACC_Z_REG 0x73
#define BMX160_OFFS_GYR_X_REG 0x74
#define BMX160_OFFS_GYR_Y_REG 0x75
#define BMX160_OFFS_GYR_Z_REG 0x76

// This register does not only enable the offsets for gyro and accel
// but also contains bit 8 and 9 of the gyro offset values!
#define BMX160_OFFS_CONF_REG 0x77
#define BMX160_OFFS_ACC_EN_BIT 6
#define BMX160_OFFS_GYR_EN_BIT 7
// Here the LSBs of bit 8 and 9 of the gyro offsets in this register
#define BMX160_OFFS_GYR_X_BIT89_LSB 0
#define BMX160_OFFS_GYR_Y_BIT89_LSB 2
#define BMX160_OFFS_GYR_Z_BIT89_LSB 4

// Command register to set power modes, reset, NVM programming etc.
#define BMX160_CMD_REG 0x7e
// retrieve the modes from the contents of BMX160_PMU_STATUS_REG
#define BMX160_CMD_SET_ACC_PMU_MODE(MODE) (0b00010000|MODE)
#define BMX160_CMD_SET_GYR_PMU_MODE(MODE) (0b00010100|MODE)
#define BMX160_CMD_SET_MAG_PMU_MODE(MODE) (0b00011000|MODE)

// wait times after setting PMU modes in ms
// These times are very conservative, and longer than the official max time from the data sheet
#define BMX160_CMD_SET_ACC_PMU_MODE_WAIT_TIME 5
#define BMX160_CMD_SET_GYR_PMU_MODE_WAIT_TIME 100
#define BMX160_CMD_SET_MAG_IF_PMU_MODE_WAIT_TIME 2

// Run fast offset calibration
#define BMX160_CMD_FOC 0x03
// Other commands
#define BMX160_CMD_PROG_NVM 0xa0
#define BMX160_CMD_FLUSH_FIFO 0xb0
#define BMX160_CMD_INT_RESET 0xb1
#define BMX160_CMD_SOFT_RESET 0xb6
#define BMX160_CMD_STEP_COUNTER_CLR 0xb2


// The data registers
#define BMX160_DATA_REG 0x04
#define BMX160_DATA_MAG_X_LSB_OFFS  0
#define BMX160_DATA_MAG_X_MSB_OFFS  1
#define BMX160_DATA_MAG_Y_LSB_OFFS  2
#define BMX160_DATA_MAG_Y_MSB_OFFS  3
#define BMX160_DATA_MAG_Z_LSB_OFFS  4
#define BMX160_DATA_MAG_Z_MSB_OFFS  5
#define BMX160_DATA_RHALL_LSB_OFFS  6
#define BMX160_DATA_RHALL_MSB_OFFS  7
#define BMX160_DATA_GYR_X_LSB_OFFS  8
#define BMX160_DATA_GYR_X_MSB_OFFS  9
#define BMX160_DATA_GYR_Y_LSB_OFFS 10
#define BMX160_DATA_GYR_Y_MSB_OFFS 11
#define BMX160_DATA_GYR_Z_LSB_OFFS 12
#define BMX160_DATA_GYR_Z_MSB_OFFS 13
#define BMX160_DATA_ACC_X_LSB_OFFS 14
#define BMX160_DATA_ACC_X_MSB_OFFS 15
#define BMX160_DATA_ACC_Y_LSB_OFFS 16
#define BMX160_DATA_ACC_Y_MSB_OFFS 17
#define BMX160_DATA_ACC_Z_LSB_OFFS 18
#define BMX160_DATA_ACC_Z_MSB_OFFS 19

#define BMX160_SENSORTIME_REG 0x18
// The 3 bytes are in little-endian order.
#define BMX160_SENSORTIME_BYTE_0 0x00
#define BMX160_SENSORTIME_BYTE_1 0x00
#define BMX160_SENSORTIME_BYTE_2 0x00

#define BMX160_STATUS_REG 0x1b
// Bits in the register
// DRDY means data ready
#define BMX160_STATUS_DRDY_ACC 7
#define BMX160_STATUS_DRDY_GYR 6
#define BMX160_STATUS_DRDY_MAG 5
// NVM = Non-volatile Memory controller status
#define BMX160_STATUS_NVM_RDY 4
// FOC = Fast Offset Compensation completed
#define BMX160_STATUS_FOC_RDY 3
// Manual Magnetometer interface operation on
#define BMX160_STATUS_MAG_MAN_OP 2
// 1= Gyro self test suffessful, 0= Running or failed
#define BMX160_STATUS_GYR_SELF_TEST_OK 1

// Temperature as signed integer. 0x0000 = 23DegC. Resolution is 1/2^9 K/LSB, range is from ~ -41 - 87 DegC
#define BMX160_TEMP_REG 0x20
#define BMX160_TEMP_LSB_OFFS 0x00
#define BMX160_TEMP_MSB_OFFS 0x01


static uint8_t dataBuf [128];

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
		DEBUG_CHR_OUT('\n');
		return false;
	}

}

static void powerAllSensorsOn() {
	
	// Switch on the accelerometer
	do {
		dataBuf[0] = BMX160_CMD_REG;
		dataBuf[1] = BMX160_CMD_SET_ACC_PMU_MODE(BMX160_PMU_STATUS_ACC_NORMAL);
		I2CStartTransferSend(BMX160ADDR,dataBuf,2);
		waitI2CTransfer("Set Accel PMU Mode normal");
		_delay_ms(BMX160_CMD_SET_ACC_PMU_MODE_WAIT_TIME);

		// read out the error register
		dataBuf[0] = BMX160_ERR_REG;
		I2CStartTransferSendReceive (
		BMX160ADDR,
		dataBuf,1,
		dataBuf,1);
		I2CStartTransferSend(BMX160ADDR,dataBuf,2);
		waitI2CTransfer("Read Error register");
	} while (dataBuf[0] & BMX160_ERR_DROP_CMD_ERR_BIT);

	DEBUG_OUT ("Accel");
	DEBUG_OUT (" up. Last error code = ");
	DEBUG_UINT_HEX_OUT(dataBuf[0]);
	DEBUG_CHR_OUT('\n');
	
	// Switch on the gyro
	do {
		dataBuf[0] = BMX160_CMD_REG;
		dataBuf[1] = BMX160_CMD_SET_GYR_PMU_MODE(BMX160_PMU_STATUS_GYR_NORMAL);
		I2CStartTransferSend(BMX160ADDR,dataBuf,2);
		waitI2CTransfer("Set Gyro PMU Mode normal");
		_delay_ms(BMX160_CMD_SET_GYR_PMU_MODE_WAIT_TIME);

		// read out the error register
		dataBuf[0] = BMX160_ERR_REG;
		I2CStartTransferSendReceive (
		BMX160ADDR,
		dataBuf,1,
		dataBuf,1);
		I2CStartTransferSend(BMX160ADDR,dataBuf,2);
		waitI2CTransfer("Read Error register");
	} while (dataBuf[0] & BMX160_ERR_DROP_CMD_ERR_BIT);
	
	DEBUG_OUT ("Gyro");
	DEBUG_OUT (" up. Last error code = ");
	DEBUG_UINT_HEX_OUT(dataBuf[0]);
	DEBUG_CHR_OUT('\n');
	
	// Switch on the magnetometer interface
	do {
		dataBuf[0] = BMX160_CMD_REG;
		dataBuf[1] = BMX160_CMD_SET_MAG_PMU_MODE(BMX160_PMU_STATUS_MAG_IF_NORMAL);
		I2CStartTransferSend(BMX160ADDR,dataBuf,2);
		waitI2CTransfer("Set Mag PMU Mode normal");
		_delay_ms(BMX160_CMD_SET_MAG_IF_PMU_MODE_WAIT_TIME);

		// read out the error register
		dataBuf[0] = BMX160_ERR_REG;
		I2CStartTransferSendReceive (
		BMX160ADDR,
		dataBuf,1,
		dataBuf,1);
		waitI2CTransfer("Read Error register");
	} while (dataBuf[0] & BMX160_ERR_DROP_CMD_ERR_BIT);

	DEBUG_OUT ("Mag interface");
	DEBUG_OUT (" up. Last error code = ");
	DEBUG_UINT_HEX_OUT(dataBuf[0]);
	DEBUG_CHR_OUT('\n');
	
}

static void configAccel() {

	dataBuf[0] = BMX160_ACC_CONF_REG;
	dataBuf[1] = 0
	| (BMX160_ACC_ODR_50 << BMX160_ACC_ODR_LWB)
	| (BMX160_ACC_BWP_NORMAL << BMX160_ACC_BWP_LWB)
	// No undersampling | (BMX160_ACC_US_ENABLE << BMX160_ACC_US_BIT)
	;
	I2CStartTransferSend(BMX160ADDR,dataBuf,2);
	waitI2CTransfer("Set Accel configuration");
	_delay_ms(10);

	dataBuf[0] = BMX160_ACC_RANGE_REG;
	dataBuf[1] = BMX160_ACC_RANGE_4G;
	I2CStartTransferSend(BMX160ADDR,dataBuf,2);
	waitI2CTransfer("Set Accel rate");
	_delay_ms(10);

}

static void configGyro() {

	dataBuf[0] = BMX160_GYR_CONF_REG;
	dataBuf[1] = 0
	| (BMX160_GYR_ODR_50 << BMX160_GYR_ODR_LWB)
	| (BMX160_GYR_BWP_NORMAL << BMX160_GYR_BWP_LWB)
	;
	I2CStartTransferSend(BMX160ADDR,dataBuf,2);
	waitI2CTransfer("Set gyro configuration");
	_delay_ms(10);

	dataBuf[0] = BMX160_ACC_RANGE_REG;
	dataBuf[1] = BMX160_ACC_RANGE_4G;
	I2CStartTransferSend(BMX160ADDR,dataBuf,2);
	waitI2CTransfer("Set gyro rate");
	_delay_ms(10);

}

static void wait_mag_man_op_finished() {

	// Wait for the mag_man_op flag to clear	
	do {
		dataBuf[0] = BMX160_STATUS_REG;
		I2CStartTransferSendReceive(BMX160ADDR,dataBuf,1,dataBuf+1,1);
		dataBuf[1] |= BMX160_STATUS_MAG_MAN_OP;
		waitI2CTransfer("Read status reg");
	} while (dataBuf[1] & BMX160_STATUS_MAG_MAN_OP);
	
	
}

static void configMag () {
	
	// Sequence see BMX160 datasheet V1.2, section 2.4.3.1.3, pg. 25
	// Mag_IF is already in normal mode.
	
	// Switch interface to manual manual, offset=0
	dataBuf[0] = BMX160_MAG_IF_CONF_REG;
	dataBuf[1] = 0
		| (1<<BMX160_MAG_IF_MAG_MAN_EN_BIT)
		// | ((readOffset&BMX160_MAG_IF_MAG_TRIG_READ_OFFS_MASK) << BMX160_MAG_IF_MAG_TRIG_READ_OFFS_LWB)
		;
	I2CStartTransferSend(BMX160ADDR,dataBuf,2);
	waitI2CTransfer("Set Mag_IF to manual mode");
	
	// Mag mode 
	dataBuf[0] = 0x4f;
	dataBuf[1] = 0x01;
	I2CStartTransferSend(BMX160ADDR,dataBuf,2);
	waitI2CTransfer("Set Mag_IF Magic");

	// ... to sleep mode
	dataBuf[0] = 0x4e;
	dataBuf[1] = 0x4b;
	I2CStartTransferSend(BMX160ADDR,dataBuf,2);
	waitI2CTransfer("Set Mag_IF Magic");
	wait_mag_man_op_finished();
	
	// Mag REPXY preset 
	dataBuf[0] = 0x4f;
	dataBuf[1] = 0x04;
	I2CStartTransferSend(BMX160ADDR,dataBuf,2);
	waitI2CTransfer("Set Mag_IF Magic");
	
	// ... to normal
	dataBuf[0] = 0x4e;
	dataBuf[1] = 0x51;
	I2CStartTransferSend(BMX160ADDR,dataBuf,2);
	waitI2CTransfer("Set Mag_IF Magic");
	wait_mag_man_op_finished();
	
	// Mag REPZ preset
	dataBuf[0] = 0x4f;
	dataBuf[1] = 0x0e;
	I2CStartTransferSend(BMX160ADDR,dataBuf,2);
	waitI2CTransfer("Set Mag_IF Magic");
	
	// ... to normal
	dataBuf[0] = 0x4e;
	dataBuf[1] = 0x52;
	I2CStartTransferSend(BMX160ADDR,dataBuf,2);
	waitI2CTransfer("Set Mag_IF Magic");
	wait_mag_man_op_finished();
	
	// prepare mag_if for data mode 1
	dataBuf[0] = 0x4f;
	dataBuf[1] = 0x02;
	I2CStartTransferSend(BMX160ADDR,dataBuf,2);
	waitI2CTransfer("Set Mag_IF Magic");
	
	// prepare mag_if for data mode 2
	dataBuf[0] = 0x4e;
	dataBuf[1] = 0x4c;
	I2CStartTransferSend(BMX160ADDR,dataBuf,2);
	waitI2CTransfer("Set Mag_IF Magic");
	wait_mag_man_op_finished();
	
	// prepare mag_if for data mode 3
	dataBuf[0] = 0x4d;
	dataBuf[1] = 0x42;
	I2CStartTransferSend(BMX160ADDR,dataBuf,2);
	waitI2CTransfer("Set Mag_IF Magic");
	wait_mag_man_op_finished();

    // Data rate to 25 Hz	
	dataBuf[0] = BMX160_MAG_IF_ODR_REG;
	dataBuf[1] = BMX160_MAG_IF_ODR_25;
	I2CStartTransferSend(BMX160ADDR,dataBuf,2);
	waitI2CTransfer("Set Mag_IF data rate");
	
	// Set the mag_if to data mode
	dataBuf[0] = BMX160_MAG_IF_CONF_REG;
	dataBuf[1] = 0;
	I2CStartTransferSend(BMX160ADDR,dataBuf,2);
	waitI2CTransfer("Set Mag_IF to data mode");
	
	// Set mag_if to low-power mode
	dataBuf[0] = BMX160_CMD_REG;
	dataBuf[1] = BMX160_CMD_SET_MAG_PMU_MODE(BMX160_PMU_STATUS_MAG_IF_LOW_PWR);
	I2CStartTransferSend(BMX160ADDR,dataBuf,2);
	waitI2CTransfer("Set MAG_IF to low power");
	
	_delay_ms(BMX160_CMD_SET_MAG_IF_PMU_MODE_WAIT_TIME);
	
}

static void printDataValueInt (uint8_t *data) {
	/*
	uint16_t uiData = *((uint16_t *)data);
	int16_t val;
	
	if (uiData > 0x7fff) {
		val = (int16_t)(uiData - 0x7fff);
	} else {
		val = ((int16_t)(uiData)) - ((int16_t)(0x7fff));
	}
	*/
	DEBUG_INT_OUT(*((int16_t*)data));
	
}

static void readoutSensors() {
	
	// First read the status
	dataBuf[0] = BMX160_STATUS_REG;
	I2CStartTransferSendReceive(BMX160ADDR,dataBuf,1,dataBuf,1);
	waitI2CTransfer("Read status reg");
	
	DEBUG_OUT ("Status reg = ");
	DEBUG_UINT_HEX_OUT(dataBuf[0]);
	DEBUG_CHR_OUT('\n');
	
	// Now read the data and the Sensortime registers
	dataBuf[0] = BMX160_DATA_REG;
	I2CStartTransferSendReceive(BMX160ADDR,dataBuf,1,dataBuf,BMX160_SENSORTIME_REG + 3 - BMX160_DATA_REG);
	waitI2CTransfer("Read data and sensortime");

	DEBUG_OUT("Mag: ");
	printDataValueInt (&dataBuf[0]);
	DEBUG_CHR_OUT(' ');
	printDataValueInt (&dataBuf[2]);
	DEBUG_CHR_OUT(' ');
	printDataValueInt (&dataBuf[4]);
	DEBUG_CHR_OUT(' ');
	DEBUG_OUT("\nRHALL: ");
	printDataValueInt (&dataBuf[6]);
	
	DEBUG_OUT("\nMagHex: ");
	DEBUG_UINT_HEX_OUT (*((uint16_t*)dataBuf));
	DEBUG_CHR_OUT(' ');
	DEBUG_UINT_HEX_OUT (*((uint16_t*)(dataBuf+2)));
	DEBUG_CHR_OUT(' ');
	DEBUG_UINT_HEX_OUT (*((uint16_t*)(dataBuf+4)));
	DEBUG_CHR_OUT(' ');
	DEBUG_OUT("\nRHALL: ");
	DEBUG_UINT_HEX_OUT (*((uint16_t*)(dataBuf+6)));
	
	DEBUG_OUT("\nGyro: ");
	printDataValueInt (&dataBuf[8]);
	DEBUG_CHR_OUT(' ');
	printDataValueInt (&dataBuf[10]);
	DEBUG_CHR_OUT(' ');
	printDataValueInt (&dataBuf[12]);
	
	DEBUG_OUT("\nAccel: ");
	printDataValueInt (&dataBuf[14]);
	DEBUG_CHR_OUT(' ');
	printDataValueInt (&dataBuf[16]);
	DEBUG_CHR_OUT(' ');
	printDataValueInt (&dataBuf[18]);

	DEBUG_OUT("\nSENSORTIME ");
	DEBUG_UINT_HEX_OUT(dataBuf[22]);
	DEBUG_UINT_HEX_OUT(dataBuf[21]);
	DEBUG_UINT_HEX_OUT(dataBuf[20]);
	DEBUG_OUT("\n\n");
	 
	 
}

void initBMX160() {
	
	powerAllSensorsOn();
	
	configAccel();
	configGyro();
	configMag();
	
}

void BMX160Test() {

	// Read the sensor ID
	// Address register
	dataBuf[0] = 0;
			
	I2CStartTransferSendReceive (
		BMX160ADDR,
		dataBuf,1,
		dataBuf,1);

	if (waitI2CTransfer("Read Chip ID")) {
		DEBUG_OUT ("Chip ID = ");
		DEBUG_UINT_HEX_OUT(dataBuf[0]);
		DEBUG_CHR_OUT('\n');
	}

	initBMX160();

	while (1) {
		
		readoutSensors();	

		cli();
		while (timerTicksElapsed < 4) {
			sleep_enable();
			sei();
			sleep_cpu();
			sleep_disable();
			cli();
		}
		timerTicksElapsed = 0;
		sei();


	}
	
}
