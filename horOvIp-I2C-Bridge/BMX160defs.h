/*
 * BMX160defs.h
 *
 * Created: 16.11.2019 16:35:32
 *  Author: kai_horstmann
 */ 


#ifndef BMX160DEFS_H_
#define BMX160DEFS_H_

#include "config.h"

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

#define BMX160_MAG_IF_READ_ADDR_REG 0x4D
#define BMX160_MAG_IF_WRITE_ADDR_REG 0x4E
#define BMX160_MAG_IF_WRITE_DATA_REG 0x4F

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
#define BMX160_DATA_MAG_X_LSB_OFFS 0x00
#define BMX160_DATA_MAG_X_MSB_OFFS 0x01
#define BMX160_DATA_MAG_Y_LSB_OFFS 0x02
#define BMX160_DATA_MAG_Y_MSB_OFFS 0x03
#define BMX160_DATA_MAG_Z_LSB_OFFS 0x04
#define BMX160_DATA_MAG_Z_MSB_OFFS 0x05
#define BMX160_DATA_RHALL_LSB_OFFS 0x06
#define BMX160_DATA_RHALL_MSB_OFFS 0x07
#define BMX160_DATA_GYR_X_LSB_OFFS 0x08
#define BMX160_DATA_GYR_X_MSB_OFFS 0x09
#define BMX160_DATA_GYR_Y_LSB_OFFS 0x0A
#define BMX160_DATA_GYR_Y_MSB_OFFS 0x0B
#define BMX160_DATA_GYR_Z_LSB_OFFS 0x0C
#define BMX160_DATA_GYR_Z_MSB_OFFS 0x0D
#define BMX160_DATA_ACC_X_LSB_OFFS 0x0E
#define BMX160_DATA_ACC_X_MSB_OFFS 0x0F
#define BMX160_DATA_ACC_Y_LSB_OFFS 0x10
#define BMX160_DATA_ACC_Y_MSB_OFFS 0x11
#define BMX160_DATA_ACC_Z_LSB_OFFS 0x12
#define BMX160_DATA_ACC_Z_MSB_OFFS 0x13

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

// The factory set trim registers used for the compensation of the raw mag values
#define BMM150_DIG_X1_REG 0x5D
#define BMM150_DIG_Z4_LSB_REG 0x62
#define BMM150_DIG_Z2_LSB_REG 0x68
#define BMM150_DIG_XY2_REG 0x70


#endif /* BMX160DEFS_H_ */
