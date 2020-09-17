/*
 * horOvIp-I2C-Bridge.c
 *
 * Created: 24.08.2019 15:18:38
 * Author : kai_horstmann
 */ 

#include "config.h"
#include <stdint.h>
#include <string.h>
#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <avr/cpufunc.h>
#include <util/delay.h>

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

#include "lwip/tcpip.h"
#include "netif/ppp/pppapi.h"
#include "lwip/api.h"
#include "lwip/tcp.h"

#include "PPP/PPPApp.h"
#include "I2C.h"
#include "BMX160.h"
#include "serDebugOut.h"
#include "PPP/ppp_usart_atmega.h"


static void mainTask( void *pvParameters );
static void vBlinkTimerFunction( TimerHandle_t xTimer );
static void vStatisticsTimerFunction( TimerHandle_t xTimer );
static TimerHandle_t blinkTimerOn  = NULL;
static TimerHandle_t blinkTimerOff = NULL;
static TimerHandle_t statisticsTimer;

#define DDR_LED DDRB
#define PORT_LED PORTB
#define PIN_LED PINB

enum BMX160Status {
	BMX160_STAT_UNDEF = 0,
	BMX160_STAT_OK = 1,
	BMX160_STAT_WARN = 2,
	BMX160_STAT_ERR =3
};

static enum BMX160Status bmx160Status = BMX160_STAT_ERR;

// Green
#define LED_I2C_OK _BV(PORT2)
// Red
#define LED_I2C_ERR _BV(PORT3)
// Yellow (green and red)
#define LED_I2C_WARN (LED_I2C_OK|LED_I2C_ERR)
// Mask to reset the pins before setting a new color
#define LED_I2C_MASK (LED_I2C_OK|LED_I2C_ERR)

enum CommStatus {
	COMM_STAT_UNDEF = 0,
	COMM_STAT_OK = 1,
	COMM_STAT_WARN = 2,
	COMM_STAT_ERR = 3
};

static enum CommStatus commStatus = COMM_STAT_ERR;

// Green
#define LED_COMM_OK _BV(PORT0)
// Red
#define LED_COMM_ERR _BV(PORT1)
// Yellow (green and red)
#define LED_COMM_WARN (LED_COMM_OK|LED_COMM_ERR)
// Mask to reset the pins before setting a new color
#define LED_COMM_MASK (LED_COMM_OK|LED_COMM_ERR)

/** \brief Semaphore for synchronizing the UDP main task with the BMX160-Data-Ready signal line.
 *
 */
static SemaphoreHandle_t bmx160DataReadySem = NULL;

int main(void) {
	
	// Set all ports as input
	// Activate the pull-ups to avoid free floating inputs
	DDRA = 0;
	PORTA = 0xff;
	DDRD = 0;
	// Switch off the pullup of PD6. This is the interrupt line from the BMX160.
	PORTD = ~_BV(PD6);
	// Do not touch the SPI pins which are being used by ISP programmers
	DDRB &= _BV(PB5)|_BV(PB6)|_BV(PB7); // Do not touch MOSI, MISO and SCK
	PORTB |= ~(_BV(PB5)|_BV(PB6)|_BV(PB7)); // Do not touch MOSI, MISO and SCK
	// Do not touch the JTAG pins.
	DDRC &= _BV(PC2)|_BV(PC3)|_BV(PC4)|_BV(PC5);
	PORTC |= ~(_BV(PC2)|_BV(PC3)|_BV(PC4)|_BV(PC5));
	
	// Switch off the pull-ups on the LED pins. Otherwise they are glowing constantly.
	// Leave the LEDs off when I switch them to Output mode.
	PORT_LED &= ~(LED_I2C_MASK|LED_COMM_MASK);
	// Switch the LED pins as output pins
	DDR_LED |= LED_I2C_MASK|LED_COMM_MASK;

	// Set sleep mode to idle mode. All peripherals remain running.
	// This is a must because I am sleeping while I2C and USART are doing their thing,
	// And the pin PD6 needs clocking to fire the interrupt when the BMX16 has data ready.
	set_sleep_mode(0);

	// Start interrupts here
	sei();

	// Enable sleep here. The idle hook will sleep the processor
	sleep_enable();

	xTaskCreate( mainTask, "TCPMain", configMINIMAL_STACK_SIZE * 3, NULL, TASK_PRIO_APP, NULL );
	blinkTimerOn = xTimerCreate("BlinkOn",50/portTICK_PERIOD_MS,pdFALSE,(void*)1,vBlinkTimerFunction);
	blinkTimerOff = xTimerCreate("BlinkOff",2950/portTICK_PERIOD_MS,pdFALSE,(void*)0,vBlinkTimerFunction);
	statisticsTimer = xTimerCreate("Statistics",5000/portTICK_PERIOD_MS,pdTRUE,vStatisticsTimerFunction,vStatisticsTimerFunction);
	xTimerStart(blinkTimerOn,10);
	xTimerStart(statisticsTimer,10);

	vTaskStartScheduler();

}

void tcpipInitDoneCb (void* d) {
	TaskHandle_t mainTask = (TaskHandle_t) d;

	DEBUG_OUT("TCP PPP Init");

	// Initialize PPP and startup the PPP listener.
	pppAppInit();

	xTaskNotifyGive(mainTask);

	DEBUG_OUT("TCP startup done");

}

static void udpMainLoop();

static void mainTask( void *pvParameters ) {

	DEBUG_INIT();

	DEBUG_OUT_START_MSG();
	DEBUG_OUT("horOV IMU board V0.2");
	DEBUG_OUT_END_MSG();

	bmx160DataReadySem = xSemaphoreCreateBinary();

	// Let other components start up safely, particularly the IMU but also other sensors
	vTaskDelay(1000/portTICK_PERIOD_MS);

	if (I2CInit()) {
		BMX160Init();
		vTaskDelay(500/portTICK_PERIOD_MS);
		// Do the initialization again.
		// Reason is that the mag trim data are usually not readable on the first try.
		// After all is being set up an running, I can soft-reset the IMU, and then
		// read the trim data reliably.
		// I do not know what is missing when I read the trim data for the first time compared to bringing up
		// the mag sensor and the interface to full functionality.
		// Frankly I do not care. It seems to work reliably on the second attempt, and that suffices for me.
		BMX160Init();
	}

	// Let the data capturing get into the swing
	vTaskDelay(200/portTICK_PERIOD_MS);

/* Sadly I am not getting the data ready interrupt of the BMX160 to work.
	// Enable pin change interrupt 3
	PCICR |= _BV(PCIE3);
	// Enable PCINT30 mask to interrupt 3.
	// PCINT30 is pin PD6. This is the pin connected to the INT1 pin of the BMX160.
	PCMSK3 |= _BV(PCINT30);
*/

	//BMX160StartDataCapturing();

	DEBUG_OUT_START_MSG();
	DEBUG_OUT("Startup TCPIP");
	DEBUG_OUT_END_MSG();

	vTaskDelay(200/portTICK_PERIOD_MS);

	tcpip_init(tcpipInitDoneCb, xTaskGetCurrentTaskHandle());

	ulTaskNotifyTake(pdTRUE,10000);

	DEBUG_OUT_START_MSG();
	DEBUG_OUT("STARTUP done");
	DEBUG_OUT_END_MSG();

	udpMainLoop();

	// Terminate yourself
	vTaskDelete(NULL);

}

static void vBlinkTimerFunction( TimerHandle_t xTimer ) {

	uint8_t ledSet;

	if (pvTimerGetTimerID(xTimer) == (void*)1) {

		// LED on timer fired.
		// Now the dark period starts.
		xTimerStart(blinkTimerOff,10);

		// Make it dark when a defined status was assumed before.
		if (bmx160Status == BMX160_STAT_UNDEF) {
			// let the LED light up in yellow when no status update occurred
			ledSet = LED_I2C_WARN;

		} else {
			ledSet = 0;
			// Reset the status.
			bmx160Status = BMX160_STAT_UNDEF;
		}

		if (commStatus == COMM_STAT_UNDEF) {
			// Make the LEDs yellow
			ledSet |= LED_COMM_WARN;
		} else {
			// Reset the status.
			commStatus = COMM_STAT_UNDEF;
		}

		PORT_LED = (PORT_LED & ~(LED_I2C_MASK|LED_COMM_MASK)) | ledSet;
		// Set the the pin to output
		DDR_LED = (DDR_LED & ~(LED_I2C_MASK|LED_COMM_MASK)) | ledSet;


	} else { // if (xTimer->pvTimerID == (void*)1)
		// The dark timer expired.
		// Now the bright period starts.
		xTimerStart(blinkTimerOn,10);

		ledSet = 0;

		switch (bmx160Status) {
		case BMX160_STAT_WARN:
			ledSet = LED_I2C_WARN;
			break;
		case BMX160_STAT_OK:
			ledSet = LED_I2C_OK;
			break;
		case BMX160_STAT_UNDEF:
		case BMX160_STAT_ERR:
		default:
			ledSet = LED_I2C_ERR;
			break;
		}

		switch (commStatus) {
		case COMM_STAT_WARN:
			ledSet |= LED_COMM_WARN;
			break;
		case COMM_STAT_OK:
			ledSet |= LED_COMM_OK;
			break;
		case COMM_STAT_UNDEF:
		case COMM_STAT_ERR:
		default:
			ledSet |= LED_COMM_ERR;
			break;
		}

		// First switch the level high. If the direction is Input, switch on the pullup.
		PORT_LED = (PORT_LED & ~(LED_I2C_MASK | LED_COMM_MASK)) | ledSet;
		// Set the the pin to output
		DDR_LED = (DDR_LED & ~(LED_I2C_MASK | LED_COMM_MASK)) | ledSet;

	} // if (xTimer->pvTimerID == (void*)1)

}

static HeapStats_t heapStats;

static void vStatisticsTimerFunction( TimerHandle_t xTimer ) {

	//return;

	vPortGetHeapStats(&heapStats);

	DEBUG_OUT("LargestFreeBlock = ");
	DEBUG_UINT_OUT(heapStats.xSizeOfLargestFreeBlockInBytes);
	DEBUG_OUT("\r\n SmallestFreeBlock= ");
	DEBUG_UINT_OUT(heapStats.xSizeOfSmallestFreeBlockInBytes);
	DEBUG_OUT("\r\nNumFreeBlocks = ");
	DEBUG_UINT_OUT(heapStats.xNumberOfFreeBlocks);

	DEBUG_OUT("\r\nFreeSpace = ");
	DEBUG_UINT_OUT(heapStats.xAvailableHeapSpaceInBytes);
	DEBUG_OUT("\r\nMinEverSpace = ");
	DEBUG_UINT_OUT(heapStats.xMinimumEverFreeBytesRemaining);
	DEBUG_OUT("\r\n\r\n");

}

void vApplicationIdleHook( void ) {

	sleep_cpu();

}



static ip4_addr_t ipAddrSendTo;
static ip4_addr_t ipAddrOwn;

static volatile bool resendCalibrationData = false;
static volatile bool resetIMU = false;


static void udpRecvTask( void *pvParameters ) {
	err_t err;
	static struct netconn *recvUdpConn = NULL;

	// Create the per-thread semphore
	netconn_thread_init();

	for (;;) {
		struct netbuf *buf;
		recvUdpConn = netconn_new(NETCONN_UDP);

		err = ERR_OK;

		if (recvUdpConn != NULL) {
			err = netconn_bind(recvUdpConn, &ipAddrOwn, BMX_160_SENSOR_BOX_IP_PORT);

			if (err != ERR_OK) {
				DEBUG_OUT("UDP recv bind, err =");
				DEBUG_INT_OUT((int)err);
				DEBUG_OUT("\r\n");
			}
		}

		while (err == ERR_OK && recvUdpConn != NULL){
			// Now the connection is guaranteed to exist
			buf = NULL;
			err = netconn_recv(recvUdpConn,&buf);
			if (err == ERR_OK) {
				if (buf != NULL) {
					netbuf_first(buf);
					do {
						void* ptr = NULL;
						u16_t len = 0;
						if (netbuf_data(buf,&ptr,&len) == ERR_OK) {
							struct BMX160RecvData * recvData = (struct BMX160RecvData *)ptr;
							DEBUG_OUT("UDP recv");
							DEBUG_OUT ("\r\n  len = ");
							DEBUG_UINT_OUT(len);
							DEBUG_OUT ("\r\n");

							if (len == sizeof(struct BMX160RecvData) &&
									recvData->header.versionMajor == BMX160_SENSORBOX_MSG_VERSION_MAJOR &&
									recvData->header.versionMinor == BMX160_SENSORBOX_MSG_VERSION_MINOR &&
									recvData->header.crc == 0xffff) {
								// looks legitimate
								switch (recvData->header.unionCode) {
								case BMX160RECV_DATA_RESET_IMU:
									resetIMU = true;
									break;
								case BMX160RECV_DATA_RESEND_MAG_TRIM_DATA:
									resendCalibrationData = true;
									break;
								default:
									break;
								}
							}
						}
					} while (netbuf_next(buf) != -1);
				}

			} else {
				DEBUG_OUT("UDP recv, err =");
				DEBUG_INT_OUT((int)err);
				DEBUG_OUT("\r\n");
			}

			if (buf != NULL) {
				netbuf_delete(buf);
			}
		}

		if (recvUdpConn != NULL) {
			netconn_close(recvUdpConn);
			netconn_delete(recvUdpConn);
			recvUdpConn = NULL;
		}

	}
}


#define MAKE_IP_ADDR(a1,a2,a3,a4) PP_HTONL(LWIP_MAKEU32(a1,a2,a3,a4))

static void udpMainLoop(){
	static err_t err;
	static TickType_t lastTick;
	static struct BMX160Data* bmx160Data;
	static struct netconn *udpSendConn;
	static struct netbuf *netb;
	static void *netBufData;
	static bool bmx160DataValid;

	bmx160Data = BMX160GetData();

	// writeTaskSync = xSemaphoreCreateBinary();
	// readTaskSync = xSemaphoreCreateBinary();

	ipAddrSendTo.addr = MAKE_IP_PEER_ADDR_I2C_BRIDGE;
	ipAddrOwn.addr = MAKE_IP_ADDR_I2C_BRIDGE;

	// Create the per-thread semphor if configured
	netconn_thread_init();

	xTaskCreate( udpRecvTask, "UDPRecv", configMINIMAL_STACK_SIZE * 2, NULL, TASK_PRIO_APP, NULL );

	for (;;) {
		udpSendConn = netconn_new(NETCONN_UDP);

		// This is solely a sending connection.
		// Therefore I can assign a dynamic port (pass 0).
		err = netconn_bind(udpSendConn, &ipAddrOwn, 0);

	    if (err == ERR_OK) {

			netconn_set_nonblocking(udpSendConn,1);

//			while (!isPPPRunning()) {
//				vTaskDelay(500/portTICK_PERIOD_MS);
//			}


			lastTick = xTaskGetTickCount();

			while (err == ERR_OK || err == ERR_WOULDBLOCK) {
				/* +/
				DEBUG_OUT("TCP Sent ");
				DEBUG_UINT_OUT(bmx160Data->header.length);
				DEBUG_OUT(" bytes\r\n");
				/+ */
				vTaskDelayUntil(&lastTick,16/portTICK_PERIOD_MS);
//				DEBUG_OUT("Wait for data ready\r\n");
				/*
				if (xSemaphoreTake(bmx160DataReadySem,30/portTICK_PERIOD_MS) == pdFALSE) {
					DEBUG_OUT("Data ready Sem timeout\r\n");
					if (bmx160Status < I2C_STAT_WARN) {
						bmx160Status = I2C_STAT_WARN;
					}
				}
				*/

				bmx160DataValid = BMX160ReadoutSensors();
				if (bmx160DataValid) {
					if (bmx160Status < BMX160_STAT_OK) {
						bmx160Status = BMX160_STAT_OK;
					}
				} else {
					bmx160Status = BMX160_STAT_ERR;
				}

				// I wait 10 ms from now, after data are actually valid.
				lastTick = xTaskGetTickCount();
				if (bmx160DataValid && isPPPRunning()) {
					netb = netbuf_new();
					netBufData = netbuf_alloc(netb,sizeof(struct BMX160Data));
					memcpy(netBufData,bmx160Data,sizeof(struct BMX160Data));
					netb->addr.addr = ipAddrSendTo.addr;
					netb->port = BMX_160_SENSOR_BOX_IP_PORT;
					err = netconn_send(udpSendConn,netb);
					netbuf_delete(netb);

					if (err == ERR_OK) {
						if (resetIMU) {
							BMX160Init();

							resetIMU = false;
							resendCalibrationData = true;
						}
						if (resendCalibrationData) {

							BMX160ReadTrimRegisters();

							netb = netbuf_new();
							if (netb != NULL) {
								netBufData = netbuf_alloc(netb,sizeof(struct BMX160Data));
							}
							if (netBufData) {
								memcpy(netBufData,bmx160Data,sizeof(struct BMX160Data));
								netb->addr.addr = ipAddrSendTo.addr;
								netb->port = BMX_160_SENSOR_BOX_IP_PORT;
								err = netconn_send(udpSendConn,netb);
							}
							if (netb != NULL) {
								netbuf_delete(netb);
							}
							resendCalibrationData = false;
						}

						if (err == ERR_OK) {
							if (commStatus < COMM_STAT_OK) {
								commStatus = COMM_STAT_OK;
							}
						} else {
							if (err == ERR_WOULDBLOCK) {
								if (commStatus < COMM_STAT_WARN) {
									commStatus = COMM_STAT_WARN;
								}
							} else {
								commStatus = COMM_STAT_ERR;
							}
						}
					} // if (err == ERR_OK)
				} else { // if (i2CStatus < I2C_STAT_ERR && isPPPRunning())
					if (isPPPRunning()) {
						if (commStatus < COMM_STAT_OK) {
							commStatus = COMM_STAT_OK;
						}
					} else {
						commStatus = COMM_STAT_ERR;
					}
				} // if (i2CStatus < I2C_STAT_ERR && isPPPRunning())

			}

			DEBUG_OUT("UDP send, err =");
			DEBUG_INT_OUT((int)err);
			DEBUG_OUT("\r\n");

	    	netconn_close(udpSendConn);
	    	netconn_delete(udpSendConn);

	    	vTaskDelay(500/portTICK_PERIOD_MS);

	    } else {
	    	netconn_close(udpSendConn);
	    	netconn_delete(udpSendConn);

	    	vTaskDelay(500/portTICK_PERIOD_MS);
	    }

	}

}

/* Sadly I am not getting the data ready interrupt of the BMX160 to work.
// External pin interrupt.
ISR(PCINT3_vect){
	static uint8_t lastPD6Value = 0;
	uint8_t currPD6Value = (PIND & _BV(PIND6));
	BaseType_t higherPrioTaskWoken = pdFALSE;

	// If a transition from low to high occurred on PD6
	if ((currPD6Value && !lastPD6Value)) {
		xSemaphoreGiveFromISR(bmx160DataReadySem,&higherPrioTaskWoken);
	}

	lastPD6Value = currPD6Value;

	// Flush out all registers to memory.
	_MemoryBarrier();

#if configUSE_PREEMPTION
	if (higherPrioTaskWoken != pdFALSE) {
		vPortYield();
	}
#endif
}
*/
