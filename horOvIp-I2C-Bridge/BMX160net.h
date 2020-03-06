/*
 * BMX160net.h
 *
 * Created: 27.10.2019 20:31:01
 *  Author: kai_horstmann
 *
 * Common interface between the micro controller and the remote application:
 * Structures and constants.
 */ 


#ifndef BMX160NET_H_
#define BMX160NET_H_

#include <inttypes.h>


/** \brief IP Address of the sensor box
 *
 * \note Please note that this must be written as a macro.
 * Before use you must define a Macro MAKE_IP_ADDR which takes the 4 octets as parameters
 */
#define MAKE_IP_ADDR_I2C_BRIDGE MAKE_IP_ADDR(192,168,203,2)

/** \brief IP netmask of the sensor box
 *
 * The netmask leaves room only for the two peer addresses 1 and 2
 * ... well 3 would also fit, but anything else would be routed.
 * uIP has no concept of peer to peer networks but assumes an ethernet below.
 *
 * \note Please note that this must be written as a macro.
 * Before use you must define a Macro MAKE_IP_ADDR which takes the 4 octets as parameters
 */
#define MAKE_IP_NETMASK_I2C_BRIDGE MAKE_IP_ADDR(255,255,255,252)

/** \brief Default router, and peer address of the client of the sensor box
 *
 * For the peer to peer connection via SLIP the peer is at the same time also the default router
 * You must use this address as own address when you bring up the connection with slipup and ifconfig or ip
 *
 * \note Please note that this must be written as a macro.
 * Before use you must define a Macro MAKE_IP_ADDR which takes the 4 octets as parameters
 */
#define MAKE_IP_PEER_ADDR_I2C_BRIDGE MAKE_IP_ADDR(192,168,203,1)

/*!
 * @brief bmm150 trim data structure
 *
 * Copied with some modifications from the Bosch BMM150 driver
 *
 * All items longer than 1 byte are in *little-endian* prepresentation!
 *
 * The components are re-ordered compared to the original BMM150 driver.
 * First come the 16 bit components, then the 8-bit components to avoid potential
 * alignment issues between AVR and X86 or ARM.
 */
struct bmm150_trim_registers {
	/*! trim z1 data */
	uint16_t dig_z1;
	/*! trim z2 data */
	int16_t dig_z2;
	/*! trim z3 data */
	int16_t dig_z3;
	/*! trim z4 data */
	int16_t dig_z4;
	/*! trim xyz1 data */
	uint16_t dig_xyz1;
	/*! trim x1 data */
	int8_t dig_x1;
	/*! trim y1 data */
	int8_t dig_y1;
	/*! trim x2 data */
	int8_t dig_x2;
	/*! trim y2 data */
	int8_t dig_y2;
	/*! trim xy1 data */
	uint8_t dig_xy1;
	/*! trim xy2 data */
	int8_t dig_xy2;
} ;

// Constants used in the compensation calculations
/**\name OVERFLOW DEFINITIONS  */
#define BMM150_XYAXES_FLIP_OVERFLOW_ADCVAL	INT16_C(-4096)
#define BMM150_ZAXIS_HALL_OVERFLOW_ADCVAL	INT16_C(-16384)
#define BMM150_OVERFLOW_OUTPUT			INT16_C(-32768)
#define BMM150_NEGATIVE_SATURATION_Z            INT16_C(-32767)
#define BMM150_POSITIVE_SATURATION_Z            UINT16_C(32767)

/*** \brief Measurement data structure with inertial measurements only
 *
 */
struct BMX160MeasDataAccGyr {
	int16_t gyrX;
	int16_t gyrY;
	int16_t gyrZ;
	int16_t accX;
	int16_t accY;
	int16_t accZ;
};

/*** \brief Measurement data structure with inertial measurements only
 *
 */
struct BMX160MeasDataAccGyrMag {
	int16_t magX;
	int16_t magY;
	int16_t magZ;
	int16_t magRHall;
	int16_t gyrX;
	int16_t gyrY;
	int16_t gyrZ;
	int16_t accX;
	int16_t accY;
	int16_t accZ;
};

/// \brief Encodes which union element is being used in \ref struct BMX160Data
enum BMX160DataUnion {
	BMX160DATA_TRIM			= 1,
	BMX160DATA_ACC_GYR		= 2,
	BMX160DATA_ACC_GYR_MAG	= 3
};

/** \brief Common communications structure for sending data from the micro controller to the application
 *
 */
struct BMX160Data {
	struct  {
	uint8_t unionCode;
	uint8_t sensorTime0;
	uint8_t sensorTime1;
	uint8_t sensorTime2;
	uint16_t length;
	} header;
	union {
		struct bmm150_trim_registers trimData;
		struct BMX160MeasDataAccGyr accGyrData;
		struct BMX160MeasDataAccGyrMag accGyrMagData;
	};
};

/** \brief Encodes commands to the sensor board from the connected application
 *
 * Being used in \ref struct 
 */
enum BMX160RecvDataUnion {
	/// Dummy message to speed up TCP reply to the board
	BMX160RECV_DATA_NONE					= 1,
	
	/// Reset the IMU, re-read the magnetometer data,
	/// send the magnetic trim data.
	BMX160RECV_DATA_RESET_IMU				= 2,
	
	/// \brief Request magnetic trim data again.
	BMX160RECV_DATA_RESEND_MAG_TRIM_DATA	= 3
};

struct BMX160RecvData {
	struct {
		uint8_t unionCode;
		uint8_t filler;
		uint16_t length;
		} header;
	union {
		uint8_t dummy;
		};
	};

#endif /* BMX160NET_H_ */
