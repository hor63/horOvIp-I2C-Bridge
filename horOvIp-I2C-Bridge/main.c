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

int main(void) {
	uint8_t i;
	
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
		uip_ipaddr_t addr;
		uip_ipaddr(&addr, 192,168,203,2);
		uip_sethostaddr(&addr);
		uip_ipaddr(&addr, 192,255,255,0);
		uip_setnetmask(&addr);
		uip_ipaddr(&addr, 192,168,203,1);
		uip_setdraddr(&addr);
	}
	uip_listen(HTONS(19463));

	// Start interrupts here
	sei();

	// Let other components start up safely, e.g. the IMU
	_delay_ms(500);

	slipStart();

	timersStart();

	DEBUG_OUT("STARTUP done\n");

	BMX160Test();
	
    while (1) {
		
		slipProcessReadBuffer();
		
		if (timerTicksElapsed > 3) {
			timerTicksElapsed = 0;
			for(i = 0; i < UIP_CONNS; ++i) {
				uip_periodic(i);
				if(uip_len > 0) {
					slipQueueUIPSendMessage();
				}
			}

		} else {
			// Polling the connections without timer processing.
			for(i = 0; i < UIP_CONNS; ++i) {
				uip_poll_conn((&(uip_conns [i])));
				if(uip_len > 0) {
					slipQueueUIPSendMessage();
				}
			}
		}
			
		
		cli();
		sleep_enable();
		sei();
		sleep_cpu();
		sleep_disable();
    }
	

	
}

void ip_i2c_bridge_appcall() {

	if(uip_connected()) {
		uip_send("Welcome to horOV IMU board\n",27);
	}

	if(uip_newdata() || uip_rexmit()) {
		uip_send("OK\n", 3);
	}
	
}
