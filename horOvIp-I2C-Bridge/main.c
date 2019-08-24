/*
 * horOvIp-I2C-Bridge.c
 *
 * Created: 24.08.2019 15:18:38
 * Author : kai_horstmann
 */ 

#include <avr/io.h>
#include "uip.h"

#include <util/delay.h>

int main(void) {
	
	uip_init();
	
	uip_listen(19463);
	
	
    /* Replace with your application code */
    while (1) {
		
		if(uip_newdata() || uip_rexmit()) {
			uip_send("ok\n", 3);
		}

    // Preliminary. Force resolution of dependencies
	uip_input();
	uip_periodic(1);
		
    }
}

void ip_i2c_bridge_appcall() {
	
}
