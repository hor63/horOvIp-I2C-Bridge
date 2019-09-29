/*
 * timers.h
 *
 * Created: 29.09.2019 15:59:55
 *  Author: kai_horstmann
 */ 

#ifndef TIMERS_H_
#define TIMERS_H_

#include <stdint.h>

extern volatile uint8_t timerTicksElapsed;

void timersStart();
void timersStop();



#endif /* TIMERS_H_ */