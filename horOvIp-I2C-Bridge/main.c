/*
 * horOvIp-I2C-Bridge.c
 *
 * Created: 24.08.2019 15:18:38
 * Author : kai_horstmann
 */ 

#include "config.h"
#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "uip.h"
#include "uip_slip/slip_usart.h"
#include "timers.h"
#include "I2C.h"
#include "BMX160.h"
#include "serDebugOut.h"

volatile bool mainLoopMustRun = false;

/** CRC-16 CCIT taken from the PPP implementation of lwip
 *
 * \see http://git.savannah.nongnu.org/cgit/lwip.git/tree/src/netif/ppp/pppos.c line 97ff
 */
static const u16_t fcstab[256] = {
  0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
  0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
  0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
  0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
  0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
  0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
  0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
  0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
  0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
  0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
  0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
  0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
  0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
  0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
  0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
  0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
  0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
  0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
  0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
  0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
  0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
  0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
  0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
  0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
  0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
  0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
  0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
  0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
  0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
  0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
  0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
  0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};
#define PPP_FCS(fcs, c) (((fcs) >> 8) ^ fcstab[((fcs) ^ (c)) & 0xff])
static inline u16_t crcBlock(u16_t crc, const void* const block,u16_t len) {
	u16_t rc = crc;
	u16_t i;

	for (i=0; i < len; i++){
		rc = PPP_FCS(rc,((const char* const)block)[i]);
	}

	return rc;
}
/*
 * Values for FCS calculations.
 */
#define PPP_INITFCS     0xffffU  /* Initial FCS value */
#define PPP_GOODFCS     0xf0b8U  /* Good final FCS value */
// End of lwip section. Thank you.

inline

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
	uip_listen(HTONS(PORT_I2C_BRIDGE));

	// Start interrupts here
	sei();

	// Let other components start up safely, particularly the IMU but also other sensors
	_delay_ms(1000);

	BMX160Init();

	// Let the data capturing get into the swing
	_delay_ms(200);

	// Repeat the initialization to read the mag trim registers after a power-up reset.
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
	struct BMX160Data* bmx160Data;
	
	if (uip_len == recvData->header.length) {
		// Seems legit
		switch (recvData->header.unionCode) {
			case BMX160RECV_DATA_RESET_IMU:
			
				// Stop the periodic data capturing temporarily
				BMX160StopDataCapturing();

				BMX160Init();
				BMX160ReadTrimRegisters();
				bmx160Data = BMX160GetData();				
				
				if (uip_mss() >= bmx160Data->header.length) {
					bmx160Data->header.versionMajor = BMX160_SENSORBOX_MSG_VERSION_MAJOR;
					bmx160Data->header.versionMinor = BMX160_SENSORBOX_MSG_VERSION_MINOR;
					bmx160Data->header.crc = 0;
					bmx160Data->header.crc = crcBlock(PPP_INITFCS,bmx160Data,bmx160Data->header.length);
					uip_send (bmx160Data,bmx160Data->header.length);
					rc = true;
				}
				
				// restart the data capture
				BMX160StartDataCapturing();
				
				break;
			
			case BMX160RECV_DATA_RESEND_MAG_TRIM_DATA:

				// Stop the periodic data capturing temporarily
				BMX160StopDataCapturing();
				
				BMX160ReadTrimRegisters();
				bmx160Data = BMX160GetData();
				
				if (uip_mss() >= bmx160Data->header.length) {
					bmx160Data->header.versionMajor = BMX160_SENSORBOX_MSG_VERSION_MAJOR;
					bmx160Data->header.versionMinor = BMX160_SENSORBOX_MSG_VERSION_MINOR;
					bmx160Data->header.crc = 0;
					bmx160Data->header.crc = crcBlock(PPP_INITFCS,bmx160Data,bmx160Data->header.length);
					uip_send (bmx160Data,bmx160Data->header.length);
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
		bmx160Data->header.versionMajor = BMX160_SENSORBOX_MSG_VERSION_MAJOR;
		bmx160Data->header.versionMinor = BMX160_SENSORBOX_MSG_VERSION_MINOR;
		bmx160Data->header.crc = 0;
		bmx160Data->header.crc = crcBlock(PPP_INITFCS,bmx160Data,bmx160Data->header.length);
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
		bmx160Data->header.versionMajor = BMX160_SENSORBOX_MSG_VERSION_MAJOR;
		bmx160Data->header.versionMinor = BMX160_SENSORBOX_MSG_VERSION_MINOR;
		bmx160Data->header.crc = 0;
		bmx160Data->header.crc = crcBlock(PPP_INITFCS,bmx160Data,bmx160Data->header.length);
		uip_send(bmx160Data,bmx160Data->header.length);

		uip_conn->appstate.numSendPackagesPending ++;
		
		mainLoopMustRun = true;
	}
	
}
