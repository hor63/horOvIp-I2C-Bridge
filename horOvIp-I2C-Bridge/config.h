/*
 * config.h
 *
 * Created: 24.08.2019 22:23:44
 *  Author: kai_horstmann
 */ 


#ifndef CONFIG_H_
#define CONFIG_H_

/** \brief The mandatory definition of the clock speed in Hz for avr-libc and a lot others
 *
 * The frequency is defined in Hz
 */
#define F_CPU 9216000UL

/** \brief The address of the BMX160 I2C on the I2C bus
 *
 */
#define BMX160ADDR 0x68

/** \brief The data rate of BMX160 in Hz
 *
 * This is the data rate at which data are captured within the BMX160.
 * It is also the timer rate, and the rate at which the application reads
 * data from the BMX160, and sends them to the vario.
 *
 * Allowed values are 100, 50, 25
 *
 */
#define BMX160ODR 50

/** 
 * Buffer size for the PPP receive and send buffers
 *
 */
#define PPP_BUFFER_SIZE 256

/** 
 * Baud rate for the PPP serial connection.
 *
 * Please note that the other serial parameters are fixed by the SLIP standard to 8,n,1
 */
#define PPP_BAUD_RATE 115200UL

/** \brief Timeout of the serial reader after which an incomplete frame is handed to PPP in ms
 *
 */
#define PPP_READ_TIMEOUT_INCOMPLETE_FRAME_MS 20

/**
 * Baud rate of the debugging port
 */
#define DEBUG_PORT_BAUD_RATE 115200UL

/** 
 * Is debug printing on?
 * Set this 1 or 0
 */
#define DEBUG_PRINT 1

/** \brief Task priority for the TCP stack
 *
 * TCP code runs in its own task.
 * It is CPU intensive, and runs low in order not to starve application and driver code of resources
 */
#define TASK_PRIO_TCP		2

/** \brief Task priority for the application code
 *
 */
#define TASK_PRIO_APP		3

/** \brief Task priority for timers for signaling and blinking
 *
 * Timers have high priority. They do not much more than switch the LED off or on.
 */
#define TASK_PRIO_TIMERS	4

/** \brief Task priority for device drivers
 *
 * Driver code which is too large and runs too long to fit inside the ISR like the I2C driver
 * but shall run ASAP. Therefore highest application priority
 */
#define TASK_PRIO_DRIVERS	5

#define configMINIMAL_STACK_SIZE	( ( unsigned short ) 200 )

#endif /* CONFIG_H_ */
