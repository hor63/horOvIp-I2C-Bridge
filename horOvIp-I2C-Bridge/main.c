/*
 * horOvIp-I2C-Bridge.c
 *
 * Created: 24.08.2019 15:18:38
 * Author : kai_horstmann
 */ 

#include "config.h"
#include <stdint.h>
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

#include "PPP/PPPApp.h"

#include "I2C.h"
#include "BMX160.h"
#include "serDebugOut.h"
#include "PPP/ppp_usart_atmega.h"


static void initTask( void *pvParameters );
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

	xTaskCreate( initTask, "Init", configMINIMAL_STACK_SIZE * 2, NULL, TASK_PRIO_APP, NULL );
	testTimer = xTimerCreate("Blinky",33,pdTRUE,vTestTimerFunction,vTestTimerFunction);
	statisticsTimer = xTimerCreate("Statistics",pdMS_TO_TICKS(5000),pdTRUE,vStatisticsTimerFunction,vStatisticsTimerFunction);
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

static void initTask( void *pvParameters ) {

	DEBUG_INIT();

	DEBUG_OUT_START_MSG();
	DEBUG_OUT("horOV IMU board V0.2");
	DEBUG_OUT_END_MSG();

	// Let other components start up safely, particularly the IMU but also other sensors
	vTaskDelay(pdMS_TO_TICKS(1000));

	//BMX160Init();

	// Let the data capturing get into the swing
	vTaskDelay(pdMS_TO_TICKS(200));

	//BMX160StartDataCapturing();

	DEBUG_OUT_START_MSG();
	DEBUG_OUT("Startup TCPIP");
	DEBUG_OUT_END_MSG();

	vTaskDelay(pdMS_TO_TICKS(200));

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

	// And wait 500ms
	vTaskDelay(pdMS_TO_TICKS(500));

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


/** \brief process an incoming message on the TCP connection
 *
 * Only messages of type \ref struct BMX160RecvData are being expected
 *
 * @param data Pointer to the receive data
 * @param len length of received data
 * @return true when the function put a message into the send buffer, i.e. the uip application callback must not send any other data.
 */

static bool processTCPMessage(void const *data,size_t len) {
	bool rc = false;
	
	/*
	uip_appdata pointer. The size of the data is
	* available through the uip_len
	*/
	struct BMX160RecvData const *recvData = (struct BMX160RecvData const *)data;
	struct BMX160Data* bmxData;
	
	if (len == recvData->header.length) {
		// Seems legit
		switch (recvData->header.unionCode) {
			case BMX160RECV_DATA_RESET_IMU:
			
				// Stop the periodic data capturing temporarily
				BMX160StopDataCapturing();

				BMX160Init();
				BMX160ReadTrimRegisters();
				bmxData = BMX160GetData();				
				
				/** todo send answer
				if (uip_mss() >= bmxData->header.length) {
					uip_send (bmxData,bmxData->header.length);
					rc = true;
				}
				*/
				
				// restart the data capture
				BMX160StartDataCapturing();
				
				break;
			
			case BMX160RECV_DATA_RESEND_MAG_TRIM_DATA:

				// Stop the periodic data capturing temporarily
				BMX160StopDataCapturing();
				
				BMX160ReadTrimRegisters();
				bmxData = BMX160GetData();
				
//				if (uip_mss() >= bmxData->header.length) {
//					uip_send (bmxData,bmxData->header.length);
					rc = true;
//				}
				
				// restart the data capture
				BMX160StartDataCapturing();
			
				break;
			default:
				break;
		}
	}
	
	return rc;
}

// todo Replace this callback with a task based code
void ip_i2c_bridge_appcall() {
	struct BMX160Data* bmx160Data = BMX160GetData();
	bool sendBMXData = false;


	/** todo After accepting a new connection send the Mag trim data
	if(uip_connected()) {
		uip_conn->appstate.sensorTime0 = 0;
		uip_conn->appstate.sensorTime1 = 0;
		uip_conn->appstate.sensorTime2 = 0;

		BMX160ReadTrimRegisters();
		uip_send(bmx160Data,bmx160Data->header.length);
		uip_conn->appstate.numSendPackagesPending = 1;

		mainLoopMustRun = true;
		return;
	}
	 */


	/** todo Read and process incoming messages
	if(uip_newdata()) {
		if (processTCPMessage()) {
			mainLoopMustRun = true;
			return;
		}
	}
	*/


	/** todo Send BMX data when new ones become available.
	if (BMX160IsDataValid() &&
		bmx160Data->header.unionCode != BMX160DATA_TRIM && (
			bmx160Data->header.sensorTime0 != uip_conn->appstate.sensorTime0 ||
			bmx160Data->header.sensorTime1 != uip_conn->appstate.sensorTime1 ||
			bmx160Data->header.sensorTime2 != uip_conn->appstate.sensorTime2
			) &&
			uip_mss() >= bmx160Data->header.length) {
		sendBMXData = true;
	}

	if (sendBMXData) {
		uip_conn->appstate.sensorTime0 = bmx160Data->header.sensorTime0;
		uip_conn->appstate.sensorTime1 = bmx160Data->header.sensorTime1;
		uip_conn->appstate.sensorTime2 = bmx160Data->header.sensorTime2;
		uip_send(bmx160Data,bmx160Data->header.length);

		uip_conn->appstate.numSendPackagesPending ++;
		
		mainLoopMustRun = true;
	}
	*/
}

ISR(TIMER3_OVF_vect) {
	numTimer3Overruns ++;
}
