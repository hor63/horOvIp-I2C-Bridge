/*
 * timers.c
 *
 * Created: 29.09.2019 16:00:12
 *  Author: kai_horstmann
 */ 

#include "timers.h"

#include "uip.h"

#include <avr/io.h>
#include <avr/interrupt.h>

volatile uint8_t timerTicksElapsed = 0;

/// \brief Setup and Start the timers
void timersStart() {

uint16_t timeCompareValue;

	// 1024 is the pre-scaler value for the counter
	timeCompareValue = F_CPU / UIP_PERIODIC_POLL_FREQUENCY_4 / 1024; 
	
	// Set the compare value
	OCR1A = timeCompareValue;
	
	
	TCCR1A = 0; // Set the timer to Clear Timer on Compare Match (CTC) mode 4
	
	TCCR1B = 0
		| _BV(WGM12) // Set the timer to Clear Timer on Compare Match (CTC) mode 4
		| 0b101; // Clock from 1/1024 prescaler
	
	// Enable the interrupt Output Compare A Match
	TIMSK1 = _BV(OCIE1A);
	
}

// Timer 1 compare value
ISR(TIMER1_COMPA_vect) {
	
	// Blinky
	DDRB = ((DDRB & (_BV(DDB2)|_BV(DDB3))) + _BV(DDB2)) & (_BV(DDB2)|_BV(DDB3));
	
	timerTicksElapsed++;
	
}