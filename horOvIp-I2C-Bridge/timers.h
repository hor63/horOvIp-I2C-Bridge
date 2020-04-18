/*
 * timers.h
 *
 * Created: 29.09.2019 15:59:55
 *  Author: kai_horstmann
 */ 

#ifndef TIMERS_H_
#define TIMERS_H_

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#include "config.h"

/** \brief Function pointer type for timer tick callback function
 *
 * @param numTicks Counter of timer ticks. Incremented by the timer interrupt routine.
 * It resets to 0 when UINT8_MAX is reached. The callback functions must be aware of this feature.
 */
typedef void (*TTimerTickCallbFuncPtr) (uint8_t numTicks);

struct TimerTickCallbChain {
	TTimerTickCallbFuncPtr callbFunc;
	struct TimerTickCallbChain* next;
};


/** \brief Add a callback for timer ticks
 *
 * @param newItem Pointer to the new chain item. The memory to the pointer should be declared statically in the module of the caller.
 * The component .callbFunc *must* be set by the caller. It will never be changed in timer.c.
 * The component .next can be left uninitialized. It will be managed and changed in timer.c.
 */
void timerAddCallback (struct TimerTickCallbChain* newItem);

/** \brief Remove a callback from the chain of timer callbacks.
 *
 * The function runs through the timer callback chain and looks for an entry that matches \ref callbFunc.
 * A matching record will be removed from the chain but the memory of the record is not freed or otherwise cleared.
 * It is advised to declare chain items static when they are added by \ref timerAddCallback.
 *
 * @param callbFunc Function pointer to the callback function which shall be un-chained.
 */
void timerRemoveCallback (TTimerTickCallbFuncPtr callbFunc);

/** \brief Timer poll function
 * This function is run by the main loop after it awakes from sleep due to interrupts.
 * Then the interrupt handler of the timer indicates that the timer elapsed it calls
 * the timer callbacks which were added by \ref timerAddCallback.
 */
void timerPoll();

/** \brief Setup and start the timer.
 *
 */
void timerStart();
// void timerStop();

/** \brief Reset the timer count to 0.
 *
 * Lets the timer cycle start again from 0.
 */
void timerReset();


#endif /* TIMERS_H_ */
