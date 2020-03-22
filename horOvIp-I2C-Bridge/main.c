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

#include "uip.h"
#include "uip_slip/slip_usart.h"
#include "prvtimers.h"
#include "I2C.h"
#include "BMX160.h"
#include "serDebugOut.h"

volatile bool mainLoopMustRun = false;

static void uipTimerCallb (uint8_t numTicks) {
	// Round the number of ticks
	static const uint8_t numTicksUipPeriodic = BMX160ODR / UIP_PERIODIC_POLL_FREQUENCY;
	static uint8_t lastNumTicks = 0;

	static uint8_t numTicsElapsed = 0;
	uint8_t i;

	if (numTicks > lastNumTicks) {
		numTicsElapsed += numTicks - lastNumTicks;
	} else {
		numTicsElapsed += UINT8_MAX - lastNumTicks + numTicks;
	}
	lastNumTicks = numTicks;

	if (numTicsElapsed > numTicksUipPeriodic) {
		for(i = 0; i < UIP_CONNS; ++i) {
			uip_periodic(i);
			if(uip_len > 0) {
				slipQueueUIPSendMessage();
			}
		}
		numTicsElapsed = 0;
	}

}

static struct TimerTickCallbChain uipTimerCallChainItem;
static void vTestTask( void *pvParameters );
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
	
	DEBUG_INIT();
	DEBUG_OUT("\n\nhorOV IMU board V0.1\n");
	
	uip_init();

	slipInit();
	
	{
		// Define MAKE_IP_ADDRESS here. It is called in the IP address macros is BMX160Net.h
		// and is defined here just before it is used.
#define MAKE_IP_ADDR(a,b,c,d) uip_ipaddr(&addr,a,b,c,d)
		uip_ipaddr_t addr;
		MAKE_IP_ADDR_I2C_BRIDGE;
		uip_sethostaddr(&addr);
		MAKE_IP_NETMASK_I2C_BRIDGE;
		uip_setnetmask(&addr);
		MAKE_IP_PEER_ADDR_I2C_BRIDGE;
		uip_setdraddr(&addr);
	}
	uip_listen(HTONS(19463));

	// Start interrupts here
	sei();

	// Let other components start up safely, particularly the IMU but also other sensors
	_delay_ms(1000);

	BMX160Init();

	// Let the data capturing get into the swing
	_delay_ms(200);

	slipStart();

	BMX160StartDataCapturing();

	timerStart();

	uipTimerCallChainItem.callbFunc = uipTimerCallb;
	uipTimerCallChainItem.next = NULL;
	timerAddCallback(&uipTimerCallChainItem);

	DEBUG_OUT("STARTUP done\n");

	// Enable sleep here. The idle hook will sleep the processor
	sleep_enable();

	xTaskCreate( vTestTask, "Test", configMINIMAL_STACK_SIZE, NULL, TASK_PRIO_APP, NULL );
	testTimer = xTimerCreate("Blinky",33,pdTRUE,vTestTimerFunction,vTestTimerFunction);
	xTimerStart(testTimer,10);

	// Set the direction of the LED pin output
	DDRB |= _BV(DDB2);

	vTaskStartScheduler();


    while (1) {
    	uint8_t i;


		if (mainLoopMustRun) {
			// I am running the main loop body. So reset the flag.
			// It may be set by an interrupt handler or application code called here.
			mainLoopMustRun = false;

			timerPoll();
			I2CPoll();
			slipProcessReadBuffer();

			// Polling the connections without timer processing.
			for(i = 0; i < UIP_CONNS; ++i) {
				uip_poll_conn((&(uip_conns [i])));
				if(uip_len > 0) {
					slipQueueUIPSendMessage();
				}
			}
		}

		cli();
		if (!mainLoopMustRun) {
			sleep_enable();
			sei();
			sleep_cpu();
			sleep_disable();
		} else {
			sei();
		}
    }

}


static void vTestTask( void *pvParameters ) {

	// Set the direction of the LED pin output
	DDRB |= _BV(DDB3);

	for (;;) {
		// Toggle the pin
		PINB = _BV(PINB3);

		// And wait 500ms
		vTaskDelay(100);
	}

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
 * @return true when the function put a message into the send buffer, i.e. the uip application callback must not send any other data.
 */
static bool processTCPMessage() {
	bool rc = false;
	
	/*
	uip_appdata pointer. The size of the data is
	* available through the uip_len
	*/
	struct BMX160RecvData *recvData = (struct BMX160RecvData *)uip_appdata;
	struct BMX160Data* bmxData;
	
	if (uip_len == recvData->header.length) {
		// Seems legit
		switch (recvData->header.unionCode) {
			case BMX160RECV_DATA_RESET_IMU:
			
				// Stop the periodic data capturing temporarily
				BMX160StopDataCapturing();

				BMX160Init();
				BMX160ReadTrimRegisters();
				bmxData = BMX160GetData();				
				
				if (uip_mss() >= bmxData->header.length) {
					uip_send (bmxData,bmxData->header.length);
					rc = true;
				}
				
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


void ip_i2c_bridge_appcall() {
	struct BMX160Data* bmx160Data = BMX160GetData();
	bool sendBMXData = false;

/*
	uip_newdata()
	uip_poll()

	uip_mss()

*/

	if (uip_closed() || uip_aborted()) {
		uip_conn->appstate.numSendPackagesPending = 0;
		return;
	}

	if (uip_timedout()) {
		uip_close();
		uip_conn->appstate.numSendPackagesPending = 0;
		return;
	}

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

	if (uip_acked()) {
		(uip_conn->appstate.numSendPackagesPending)--;
	}

	if(uip_newdata()) {
		if (processTCPMessage()) {
			mainLoopMustRun = true;
			return;
		}
	}

	if (uip_rexmit()) {
		if (uip_mss() >= sizeof (struct BMX160Data)) {
			sendBMXData = true;
		}
	} else {
			if (BMX160IsDataValid() &&
				bmx160Data->header.unionCode != BMX160DATA_TRIM && (
					bmx160Data->header.sensorTime0 != uip_conn->appstate.sensorTime0 ||
					bmx160Data->header.sensorTime1 != uip_conn->appstate.sensorTime1 ||
					bmx160Data->header.sensorTime2 != uip_conn->appstate.sensorTime2
					) &&
					uip_mss() >= bmx160Data->header.length) {
				sendBMXData = true;
			}
	}

	if (sendBMXData) {
		uip_conn->appstate.sensorTime0 = bmx160Data->header.sensorTime0;
		uip_conn->appstate.sensorTime1 = bmx160Data->header.sensorTime1;
		uip_conn->appstate.sensorTime2 = bmx160Data->header.sensorTime2;
		uip_send(bmx160Data,bmx160Data->header.length);

		uip_conn->appstate.numSendPackagesPending ++;
		
		mainLoopMustRun = true;
	}
	
}
