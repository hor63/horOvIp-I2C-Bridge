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

#include "PPP/PPPApp.h"

#include "I2C.h"
#include "BMX160.h"
#include "serDebugOut.h"
#include "PPP/ppp_usart_atmega.h"


static void initTask( void *pvParameters );
static void vTestTimerFunction( TimerHandle_t xTimer );
static TimerHandle_t testTimer;

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

	xTaskCreate( initTask, "Init", configMINIMAL_STACK_SIZE, NULL, TASK_PRIO_APP, NULL );
	testTimer = xTimerCreate("Blinky",33,pdTRUE,vTestTimerFunction,vTestTimerFunction);
	xTimerStart(testTimer,10);

	// Set the direction of the LED pin output
	DDRB |= _BV(DDB2);

	vTaskStartScheduler();

}

void tcpipInitDoneCb (void* d) {
	SemaphoreHandle_t sem = (SemaphoreHandle_t) d;

	xSemaphoreGive(sem);

}

static void initTask( void *pvParameters ) {

	DEBUG_INIT();
	DEBUG_OUT("\n\nhorOV IMU board V0.2\n");

	SemaphoreHandle_t tcpStartSem = xSemaphoreCreateBinary();

	tcpip_init(NULL, NULL);

	// Let other components start up safely, particularly the IMU but also other sensors
	vTaskDelay(1000 / portTICK_PERIOD_MS);

	BMX160Init();

	// Let the data capturing get into the swing
	vTaskDelay(200 / portTICK_PERIOD_MS);

	BMX160StartDataCapturing();

	xSemaphoreTake(tcpStartSem,10000/portTICK_PERIOD_MS);

	// Initialize PPP and startup the PPP listener.
	pppAppInit();

	DEBUG_OUT("STARTUP done\n");

	// And wait 500ms
	vTaskDelay(500 / portTICK_PERIOD_MS);

	// Terminate yourself
	vTaskDelete(NULL);

}

static void vTestTimerFunction( TimerHandle_t xTimer ) {

	// Toggle the pin
	PINB = _BV(PINB2);

}

void vApplicationIdleHook( void ) {

	sleep_cpu();

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
	struct BMX160RecvData *recvData = (struct BMX160RecvData const *)data;
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
				
				if (uip_mss() >= bmxData->header.length) {
					uip_send (bmxData,bmxData->header.length);
					rc = true;
				}
				
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
