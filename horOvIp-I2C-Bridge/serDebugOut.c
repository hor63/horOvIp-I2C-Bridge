/*
 * serDebugOut.c
 *
 * Created: 02.10.2019 22:50:55
 *  Author: kai_horstmann
 */ 

#include <stdlib.h>
#include <stdbool.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/cpufunc.h>

#include "serDebugOut.h"

#include "FreeRTOS.h"
#include "semphr.h"

/// The term "(F_CPU + DEBUG_PORT_BAUD_RATE*8)" rounds the UBRR value to the next integer
#define USART_BAUD_CONFIG_VAL (((F_CPU + DEBUG_PORT_BAUD_RATE*8)/(16*DEBUG_PORT_BAUD_RATE))-1)
/// Calculate the actual baud rate in mHz
#define USART_ACTUAL_BAUD_RATE_1000 ((F_CPU*1000)/(16*(USART_BAUD_CONFIG_VAL+1)))
/// Calculate the ratio of the actual and the target baud rate in 1000s. 1000 means a ratio of 1.0, i.e. an exact fit
#define USART_BAUD_RATE_RATIO_PROMILLE (USART_ACTUAL_BAUD_RATE_1000/DEBUG_PORT_BAUD_RATE)

#if ((USART_BAUD_RATE_RATIO_PROMILLE > 1010) || (USART_BAUD_RATE_RATIO_PROMILLE < 990))
#	error Baud rate deviation is larger than 1%. Please use a more suitable clock which can be divided better to the baud rate.
#endif


/** \brief Character buffer for integer to string conversion
 * 10 digits + sign are enough for 32-bit ints
 */
static char convertBuf[14];

static uint8_t outBuf[256];
static uint8_t currOutIndex = 0;
static uint8_t nextFreeIndex = 0;

void debugOutInit() {
	
	// Setup the serial port USART1

	// Setup the baud rate.
	UBRR1H = (USART_BAUD_CONFIG_VAL >> 8) & 0b00001111; // upper 4 bits are reserved and must be set 0.
	UBRR1L = USART_BAUD_CONFIG_VAL & 0xff;

// set or reset the control and status register
	UCSR1A = 0
	// | _BV(RXC1)		// USART Receive Complete (R/O)
	| _BV(TXC1)		// USART Transmit Complete (Write 1 resets the flag)
	// | _BV(UDRE1)	// USART Data Register Empty (R/O)
	// | _BV(FE1)		// Frame Error (R/O)
	// | _BV(DOR1)		// Data OverRun (R/O)
	// | _BV(UPE1)		// USART Parity Error (R/O)
	// | _BV(U2X1)		// Double the USART Transmission Speed
	// | _BV(MPCM1)	// Multi-processor Communication Mode
	;

	// Activate the transmitter but not yet the interrupts
	UCSR1B = 0
	// | _BV(RXCIE1)	// RX Complete Interrupt Enable
	// | _BV(TXCIE1)	// TX Complete Interrupt Enable
	// | _BV(UDRIE1)	// USART Data Register Empty Interrupt Enable
	// | _BV(RXEN1)		// Receiver Enable
	| _BV(TXEN1)		// Transmitter Enable
	// | _BV(UCSZ12)	// UCSZn2 Character Size Bit 2: 011: 8 Bit
	// | _BV(RXB81)		// Receive Data Bit 8 (R/O)
	// | _BV(TXB81)		// Transmit Data Bit 8 (N/A here)
	;

	UCSR1C = 0
	// | _BV(UMSEL11)		// UMSELn1:0 USART Mode Select
	// | _BV(UMSEL10)		// 00: Asynchronous mode
	// | _BV(UPM11)			// UPMn1:0: Parity Mode
	// | _BV(UPM10)			// 0: No parity
	// | _BV(USBS1)			// Stop Bit Select: 0: 1 Stop bit
	| _BV(UCSZ11)		// UCSZn1:0: Character Size
	| _BV(UCSZ10)		// 011: 8 Bit
	// | _BV(UCPOL1)		// Clock Polarity (Unused, set 0)
	;


}

static inline void activateSender() {
	// Activate the Send buffer empty interrupt
	UCSR1B |= _BV(UDRIE1);
}

/** Before the end of the ring buffer over-takes the end
 * drop the earliest characters in the buffer, but preserve as much of the
 * debug output buffer.
 */
static inline void advanceBufferStart() {
	if (currOutIndex == nextFreeIndex) {
		++currOutIndex;
		_MemoryBarrier();
	}
}

void debugOutStartMsg(){
static TaskStatus_t ts;
	portENTER_CRITICAL();

	vTaskGetInfo(NULL,&ts,pdFALSE,eInvalid);
	debugOutStr("Task ");
	debugOutStr(ts.pcTaskName);
	debugOutChr(':');
	portEXIT_CRITICAL();
}

/** Print CR-LF and unlock the module for printing by other tasks \n
 * Complements \ref debugOutStartMsg
 */
void debugOutEndMsg(){

	debugOutStr("\r\n");

}

void debugOutStr(const char* str) {
	portENTER_CRITICAL();
	debugOutStrFromISR(str);
	portEXIT_CRITICAL();
}

void debugOutStrFromISR(const char* str) {
	bool strNotEmpty = false;

	while (*str) {
		outBuf[nextFreeIndex] = *str;
		++nextFreeIndex;
		advanceBufferStart();
		++str;
		strNotEmpty = true;
	}
	
	if (strNotEmpty) {
		activateSender();
	}
}

void debugOutChr(char chr) {
	portENTER_CRITICAL();
	
	if (chr) {
		outBuf[nextFreeIndex] = chr;
		++nextFreeIndex;
		advanceBufferStart();
		activateSender();
	}

	portEXIT_CRITICAL();
}

void debugOutInt(int val) {
	portENTER_CRITICAL();
	debugOutStr(itoa(val,convertBuf,10));
	portEXIT_CRITICAL();
}

void debugOutUInt(unsigned int val) {
	portENTER_CRITICAL();
	debugOutStr(utoa(val,convertBuf,10));
	portEXIT_CRITICAL();
}

static const char hexNibbleVal[] = {
	'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F',
};

void debugOutByteHex(unsigned char val) {
	portENTER_CRITICAL();
	debugOutChr(hexNibbleVal[val>>4]);
	debugOutChr(hexNibbleVal[val&0x0f]);
	portEXIT_CRITICAL();
}


void debugOutUIntHex(unsigned int val) {
	portENTER_CRITICAL();
	debugOutStr(utoa(val,convertBuf,16));
	portEXIT_CRITICAL();
}

void debugOutULong(unsigned long val) {
	portENTER_CRITICAL();
	debugOutStr(ultoa(val,convertBuf,10));
	portEXIT_CRITICAL();
}

void debugOutULongHex(unsigned long val) {
	portENTER_CRITICAL();
	debugOutStr(ultoa(val,convertBuf,16));
	portEXIT_CRITICAL();
}


// The send buffer is ready to take another byte to be sent
ISR(USART1_UDRE_vect) {
	
	UDR1 = outBuf[currOutIndex];
	++currOutIndex;

	if (currOutIndex == nextFreeIndex) {
 		// Disable the send buffer empty interrupt
 		UCSR1B &= ~_BV(UDRIE1);
	}
	
}

void debugAssertOut(const char* assertText,const char* filename,int line) {
	portENTER_CRITICAL();
	const char* basename = filename;
	for (;*filename == 0; filename++) {
		if (*filename == '\\' || *filename == '/') {
			basename = filename + 1;
		}
	}
	debugOutStr(basename);
	debugOutChr(':');
	debugOutInt(line);
	debugOutStr(" Assertion");
//	debugOutStr(assertText);
	debugOutStr("\r\n");
	portEXIT_CRITICAL();
}

void vApplicationStackOverflowHook( TaskHandle_t xTask,
                                    signed char *pcTaskName ) {
	(void) xTask;

	portENTER_CRITICAL();
	debugOutStr("!!!Stack overflow !!! Task ");
	debugOutStr((const char *)pcTaskName);
	debugOutStr("\r\n");
	portEXIT_CRITICAL();
}

void vApplicationMallocFailedHook( void ){

	debugOutStrFromISR("\r\n!!!Malloc FAILED!!!\r\n");
	
}
