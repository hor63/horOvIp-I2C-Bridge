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
static void vTestTimerFunction( TimerHandle_t xTimer );
static void vStatisticsTimerFunction( TimerHandle_t xTimer );
static TimerHandle_t testTimer;
static TimerHandle_t statisticsTimer;

int main(void) {
	
	// Set all ports as input
	// Activate the pull-ups to avoid free floating inputs
	DDRA = 0;
	PORTA = 0xff;
	DDRD = 0;
	PORTD = 0xff;
	// Do not touch the SPI pins which are being used by ISP programmers
	DDRB &= _BV(PB5)|_BV(PB6)|_BV(PB7); // Do not touch MOSI, MISO and SCK
	PORTB |= ~(_BV(PB5)|_BV(PB6)|_BV(PB7)); // Do not touch MOSI, MISO and SCK
	// Do not touch the JTAG pins.
	DDRC &= _BV(PC2)|_BV(PC3)|_BV(PC4)|_BV(PC5);
	PORTC |= ~(_BV(PC2)|_BV(PC3)|_BV(PC4)|_BV(PC5));
	
	set_sleep_mode(0);
	
	// Start interrupts here
	sei();

	// Enable sleep here. The idle hook will sleep the processor
	sleep_enable();

	xTaskCreate( mainTask, "TCPMain", configMINIMAL_STACK_SIZE * 3, NULL, TASK_PRIO_APP, NULL );
	testTimer = xTimerCreate("Blinky",33,pdTRUE,vTestTimerFunction,vTestTimerFunction);
	statisticsTimer = xTimerCreate("Statistics",5000/portTICK_PERIOD_MS,pdTRUE,vStatisticsTimerFunction,vStatisticsTimerFunction);
	xTimerStart(testTimer,10);
	xTimerStart(statisticsTimer,10);

	// Set the direction of the LED pin output
	DDRB |= _BV(DDB2);

	vTaskStartScheduler();

}

void tcpipInitDoneCb (void* d) {
	TaskHandle_t mainTask = (TaskHandle_t) d;

	// Initialize PPP and startup the PPP listener.
	pppAppInit();

	xTaskNotifyGive(mainTask);

}

static void udpMainLoop();

static void mainTask( void *pvParameters ) {

	DEBUG_INIT();

	DEBUG_OUT_START_MSG();
	DEBUG_OUT("horOV IMU board V0.2");
	DEBUG_OUT_END_MSG();

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

	//BMX160StartDataCapturing();

	DEBUG_OUT_START_MSG();
	DEBUG_OUT("Startup TCPIP");
	DEBUG_OUT_END_MSG();

	vTaskDelay(200/portTICK_PERIOD_MS);

	tcpip_init(tcpipInitDoneCb, xTaskGetCurrentTaskHandle());

	ulTaskNotifyTake(pdTRUE,10000);

	// Setup timer 3 as a simple overflow timer
	// Output Pins OCN3A and OCN3B are disconnected.
	// Operation mode "Normal", i.e. Counter running up until overflow.
	TCCR3A = 0U;
	// Input noise canceler off,
	// Operation mode is Normal
	// CLock select is system clock without prescaler.
	TCCR3B = _BV(CS30);

	// Enable timer 3 interrupt overflow.
	TIMSK3 = _BV(TOIE3);

	DEBUG_OUT_START_MSG();
	DEBUG_OUT("STARTUP done");
	DEBUG_OUT_END_MSG();

	udpMainLoop();

	// Terminate yourself
	vTaskDelete(NULL);

}

static void vTestTimerFunction( TimerHandle_t xTimer ) {

	// Toggle the pin
	PINB = _BV(PINB2);

}

static HeapStats_t heapStats;
// Used to count the system clocks with timer 3
// To form a 32-bit counter.
static uint16_t numTimer3Overruns = 0;

static uint32_t clocksBeforeSleep = 0;
static uint32_t clocksInSleep = 0;

static uint32_t getSysClocks() {
	uint32_t ret;

	portENTER_CRITICAL();

	ret = ((uint32_t)numTimer3Overruns << 16) + TCNT3;

	portEXIT_CRITICAL();

	return ret;
}

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
	DEBUG_OUT("\r\nSuccessAllocations = ");
	DEBUG_UINT_OUT(heapStats.xNumberOfSuccessfulAllocations);
	DEBUG_OUT("\r\nSuccessFrees = ");
	DEBUG_UINT_OUT(heapStats.xNumberOfSuccessfulFrees);
	DEBUG_OUT("\r\nMinEverSpace = ");
	DEBUG_UINT_OUT(heapStats.xMinimumEverFreeBytesRemaining);
	DEBUG_OUT("\r\nClocks = ");
	DEBUG_ULONG_OUT(getSysClocks());
	DEBUG_OUT("\r\nClocks in sleep = ");
	DEBUG_ULONG_OUT(clocksInSleep);
	DEBUG_OUT("\r\n\r\n");

}

void vApplicationIdleHook( void ) {

	uint32_t currSysClocks;


	clocksBeforeSleep = getSysClocks();

	sleep_cpu();

	currSysClocks = getSysClocks();

	clocksInSleep += (currSysClocks - clocksBeforeSleep);
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

			while (!isPPPRunning()) {
				vTaskDelay(500/portTICK_PERIOD_MS);
			}


			lastTick = xTaskGetTickCount();

			while (err == ERR_OK || err == ERR_WOULDBLOCK) {
				/* +/
				DEBUG_OUT("TCP Sent ");
				DEBUG_UINT_OUT(bmx160Data->header.length);
				DEBUG_OUT(" bytes\r\n");
				/+ */
				vTaskDelayUntil(&lastTick,20/portTICK_PERIOD_MS);

				BMX160ReadoutSensors();
				// I wait 15 ms from now, after data are actually valid.
				lastTick = xTaskGetTickCount();
				if (isPPPRunning()) {
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

					}
				}

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

ISR(TIMER3_OVF_vect) {
	numTimer3Overruns ++;
}

