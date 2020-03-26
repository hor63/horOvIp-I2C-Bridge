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

/** \brief uIP periodic poll frequency in Hz
 *
 */
#define UIP_PERIODIC_POLL_FREQUENCY 4

/** 
 * Buffer size for the SLIP receive and send buffers as well as the uIP buffer
 *
 * This is 100 bytes application data. The rest is IP and TCP headers.
 * \see UIP_CONF_BUFFER_SIZE
 */
#define IP_BUFFER_SIZE 220

/** Number of SLIP receive buffers
 * Since the number of concurrent IP connections is 4 I provide 4 buffers
 *
 * \see UIP_CONF_MAX_CONNECTIONS
 */
#define SLIP_NUM_IP_RECV_BUFFERS 4


/** Number of SLIP send buffers
 * Since the number of concurrent IP connections is 4 I provide 4 buffers
 *
 * \see UIP_CONF_MAX_CONNECTIONS
 */
#define SLIP_NUM_IP_SEND_BUFFERS 4

/** 
 * Baud rate for the SLIP serial connection.
 *
 * Please note that the other serial parameters are fixed by the SLIP standard to 8,n,1
 */
#define SLIP_BAUD_RATE 115200UL
// #define SLIP_BAUD_RATE 19200UL

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

/** \brief Task priority for device drivers
 *
 * Driver code which is too large and runs too long to fit inside the ISR like the I2C driver
 */
#define TASK_PRIO_DRIVERS	4

/** \brief Task priority for timers for signaling and blinking
 *
 * Timers have high priority. They do not much more than switch the LED off or on.
 */
#define TASK_PRIO_TIMERS	5

#endif /* CONFIG_H_ */
