/*
 * serDebugOut.h
 *
 * Created: 02.10.2019 21:53:30
 *  Author: kai_horstmann
 */ 


#ifndef SERDEBUGOUT_H_
#define SERDEBUGOUT_H_

// Including options, and uip_log if defined
#include "uipopt.h"

#if DEBUG_PRINT
	#define DEBUG_INIT() debugOutInit()
	#define DEBUG_OUT(x) debugOutStr(x)
	#define DEBUG_INT_OUT(x) debugOutInt(x)
	#define DEBUG_UINT_OUT(x) debugOutUInt(x)
	#define DEBUG_UINT_HEX_OUT(x) debugOutUIntHex(x)
	#define DEBUG_BYTE_HEX_OUT(x) debugOutByteHex(x)
	#define DEBUG_CHR_OUT(x) debugOutChr(x)
#else /* if DEBUG_PRINT */
	// As empty as possible but creates an error if you do not terminate it with ';' :)
	#define DEBUG_INIT() do{}while(0)
	#define DEBUG_OUT(x) do{}while(0)
	#define DEBUG_INT_OUT(x) do{}while(0)
	#define DEBUG_UINT_OUT(x) do{}while(0)
	#define DEBUG_UINT_HEX_OUT(x) do{}while(0)
	#define DEBUG_BYTE_HEX_OUT(x) do{}while(0)
	#define DEBUG_CHR_OUT(x) do{}while(0)
#endif /* if DEBUG_PRINT */

/// Call once at the start of program to configure and startup USART1
void debugOutInit();

/// Print a string out of USART1
void debugOutStr(const char* str);

/// Print a single character out of USART1
void debugOutChr(char chr);

/// Print an int value out of USART1
/// The function is not reentrant safe! Never call it from an interrupt handler!!!
void debugOutInt(int val);

/// Print an unsigned int value out of USART1
/// The function is not reentrant safe! Never call it from an interrupt handler!!!
void debugOutUInt(unsigned int val);

/// Print an unsigned int value as hex value out of USART1
/// The function is not reentrant safe! Never call it from an interrupt handler!!!
void debugOutUIntHex(unsigned int val);

/// Print an unsigned character value as hex value out of USART1
/// The function is not reentrant safe! Never call it from an interrupt handler!!!
void debugOutByteHex(unsigned char val);

#endif /* SERDEBUGOUT_H_ */