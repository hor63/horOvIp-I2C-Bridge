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

uint8_t timerTicksElapsed = 0;
bool timerTickOccurred = false;

/// From main.c
extern volatile bool mainLoopMustRun;

static struct TimerTickCallbChain* callbackChain = NULL;

void timerAddCallback (struct TimerTickCallbChain* newItem){
	newItem->next = callbackChain;
	callbackChain = newItem;
}

void timerRemoveCallback (TTimerTickCallbFuncPtr callbFunc) {
	struct TimerTickCallbChain* i = callbackChain;
	struct TimerTickCallbChain* prev = NULL;
	while (i) {
		if (i->callbFunc == callbFunc) {
			// Match.
			if (prev) {
				// Progress to the next item
				i = i->next;
				// Unchain the item in the middle
				prev->next = i;
			} else { // if (prev)
				// The item to be un-chained is the start of the chain.
				// Therefore set the start of the chain to the next item
				i = callbackChain = i->next;
			} // if (prev)
		} else { // if (i->callbFunc == callbFunc)
			// No match. Progress to the next item.
			prev = i;
			i = i->next;
		} // if (i->callbFunc == callbFunc)
	} // while (i)
}

void timerPoll() {
	struct TimerTickCallbChain* i = callbackChain;

	if (timerTickOccurred) {
		timerTickOccurred = false;
		while (i) {
			i->callbFunc(timerTicksElapsed);
			i= i->next;
		}
	}

}


/// \brief Setup and Start the timers
void timerStart() {

uint16_t timeCompareValue;

	// 64 is the pre-scaler value for the counter
	// Let the timer run out 1 ms earlier. I am waiting the remainder actively for the data to become available.
	// Thus I am re-synchronizing with the sensor cycle each time.
	timeCompareValue = F_CPU / (1/((1.0/BMX160ODR) - 0.001)) / 64;
	
	// Set the compare value
	OCR1A = timeCompareValue;
	
	
	TCCR1A = 0; // Set the timer to Clear Timer on Compare Match (CTC) mode 4
	
	TCCR1B = 0
		| _BV(WGM12) // Set the timer to Clear Timer on Compare Match (CTC) mode 4
		| 0b011 // Clock from 1/64 prescaler
		;
	
	// Enable the interrupt Output Compare A Match
	TIMSK1 = _BV(OCIE1A);
	
}

void timerReset() {
	TCNT1 = 0;
}

ISR(TIMER1_COMPA_vect) {
	
	// Blinky
	// DDRB = ((DDRB & (_BV(DDB2)|_BV(DDB3))) + _BV(DDB2)) & (_BV(DDB2)|_BV(DDB3));
	
	timerTicksElapsed++;
	timerTickOccurred = true;
	mainLoopMustRun = true;
}
