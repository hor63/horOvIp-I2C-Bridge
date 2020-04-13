/*
 * ppp_usart_atmega.c
 *
 *  Created on: 25.03.2020
 *      Author: kai_horstmann
 */

#include "config.h"

#include <string.h>
#include <stdbool.h>

#include <avr/io.h>
#include <avr/interrupt.h>

#include "PPP/ppp_usart_atmega.h"

#include "serDebugOut.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "timers.h"

#include "serDebugOut.h"

/// The term "(F_CPU + SLIP_BAUD_RATE*8)" rounds the UBRR value to the next integer
#define USART_BAUD_CONFIG_VAL (((F_CPU + PPP_BAUD_RATE*8)/(16*PPP_BAUD_RATE))-1)
/// Calculate the actual baud rate in mHz
#define USART_ACTUAL_BAUD_RATE_1000 ((F_CPU*1000)/(16*(USART_BAUD_CONFIG_VAL+1)))
/// Calculate the ratio of the actual and the target baud rate in 1000s. 1000 means a ratio of 1.0, i.e. an exact fit
#define USART_BAUD_RATE_RATIO_PROMILLE (USART_ACTUAL_BAUD_RATE_1000/PPP_BAUD_RATE)

#if ((USART_BAUD_RATE_RATIO_PROMILLE > 1010) || (USART_BAUD_RATE_RATIO_PROMILLE < 990))
#	error Baud rate deviation is larger than 1%. Please use a more suitable clock which can be divided better to the baud rate.
#endif

/// Type of the index in the PPP buffers
typedef uint16_t TPPPBufferIndex;

#define SEND_INDEX_NONE UINT16_MAX


// PPP related stuff
/* \brief PPP frame separation character.
 *
 * Character which defines frame start and end.
 * back to back frames can be separated by one one separator
 *
 * \see [RFC 1662](https://tools.ietf.org/html/rfc1662#page-8)
 */
#define PPP_FLAG_SEQUENCE ((uint8_t)0x7e)

/* \brief PPP Control Escape character.
 *
 * The \ref PPP_FLAG_SEQUENCE and other special characters (like XON/XOFF) in the user data
 * are masked by this character.
 *
 * \see [RFC 1662](https://tools.ietf.org/html/rfc1662#page-8)
 */

#define PPP_CONTROL_ESCAPE ((uint8_t)0x7e)

/* PPP XOR value for masked special characters.
 *
 * Special character in the user data stream which are masked by \ref PPP_FLAG_SEQUENCE
 * are in addition XORed with this character to prevent their occurrence in the
 * serial data stream.
 *
 * \see [RFC 1662](https://tools.ietf.org/html/rfc1662#page-8)
 */
#define PPP_ESC_XOR_CHAR ((uint8_t)0x20)


/// \brief index to the next free position in the read buffer
///
/// This is the next position after the last valid data in the buffer
///
/// /// If \ref readBufferDataStart == \p readBufferFreeStart the buffer is empty.
static TPPPBufferIndex readBufferFreeStart = 0;
/// \brief Index to the first valid character in the read buffer.
///
/// If \p readBufferEnd == \ref readBufferStart the buffer is empty.
static TPPPBufferIndex readBufferDataStart = 0;
/// \brief Number of valid characters in the read buffer.
///
/// Book keeping of the number of bytes in the buffer makes it a lot easier to
/// deal with buffer wrap-arounds
static TPPPBufferIndex numInReadBuffer = 0;

static TaskHandle_t readDriverTask = 0;
static void readerTaskFunc(void* ctx);

static uint8_t readBuffer [PPP_BUFFER_SIZE];

/// \brief index to the next free position in the write buffer
///
/// This is the next position after the last valid data in the buffer
///
/// /// If \ref writeBufferDataStart == \p writeBufferFreeStart the buffer is empty.
static TPPPBufferIndex writeBufferFreeStart = 0;
/// \brief Index to the first valid character in the read buffer.
///
/// If \p readBufferEnd == \ref readBufferStart the buffer is empty.
static TPPPBufferIndex writeBufferDataStart = 0;
/// \brief Number of valid characters in the write buffer.
///
/// Book keeping of the number of bytes in the buffer makes it a lot easier to
/// deal with buffer wrap-arounds
static TPPPBufferIndex numInWriteBuffer = 0;

/// \brief Exclusive entry into \ref PPPUsartSend
volatile uint32_t ppp_usart_atmegaGuard1 = 0;
SemaphoreHandle_t ppp_usart_atmegaWriteMutex = (SemaphoreHandle_t)0;
volatile uint32_t ppp_usart_atmegaGuard2 = 0;

/// Task handle of a task waiting for free space in the send buffer
static TaskHandle_t waitingWriteTask = 0;

static uint8_t writeBuffer [PPP_BUFFER_SIZE];

/** \brief Reset the administration of the read buffer
 *
 * The caller must block interrupts in case that this is called in the middle of processing.
 */
static inline void initReadBuffer() {
	// (Re-)initialize the read buffer administration
	readBufferFreeStart = 0;
	readBufferDataStart = 0;
	numInReadBuffer = 0;
}

/** \brief Reset the administration of the write buffer
 *
 * The caller must block interrupts in case that this is called in the middle of processing.
 */
static inline void initWriteBuffer() {
	writeBufferFreeStart = 0;
	writeBufferDataStart = 0;
	numInWriteBuffer = 0;
}

void PPPUsartInit() {

	if (ppp_usart_atmegaWriteMutex == NULL) {
		ppp_usart_atmegaWriteMutex = xSemaphoreCreateMutex();
	}

	memset((void*)readBuffer,0,sizeof(readBuffer));
	memset((void*)writeBuffer,0,sizeof(writeBuffer));

	initReadBuffer();
	initWriteBuffer();

	// Setup the serial port USART0
	// set or reset the control and status register
	UCSR0A = 0
		// | _BV(RXC0)		// USART Receive Complete (R/O)
		| _BV(TXC0)		// USART Transmit Complete (Write 1 resets the flag)
		// | _BV(UDRE0)	// USART Data Register Empty (R/O)
		// | _BV(FE0)		// Frame Error (R/O)
		// | _BV(DOR0)		// Data OverRun (R/O)
		// | _BV(UPE0)		// USART Parity Error (R/O)
		// | _BV(U2X0)		// Double the USART Transmission Speed
		// | _BV(MPCM0)	// Multi-processor Communication Mode
		;

	// Do not activate the interfaces or the interrupts yet
	UCSR0B = 0
		// | _BV(RXCIE0)	// RX Complete Interrupt Enable
		// | _BV(TXCIE0)	// TX Complete Interrupt Enable
		// | _BV(UDRIE0)	// USART Data Register Empty Interrupt Enable
		// | _BV(RXEN0)		// Receiver Enable
		// | _BV(TXEN0)		// Transmitter Enable
		// | _BV(UCSZ02)	// UCSZn2 Character Size Bit 2: 011: 8 Bit
		// | _BV(RXB80)		// Receive Data Bit 8 (R/O)
		// | _BV(TXB80)		// Transmit Data Bit 8 (N/A here)
		;

	UCSR0C = 0
		// | _BV(UMSEL01)		// UMSELn1:0 USART Mode Select
		// | _BV(UMSEL00)		// 00: Asyncronous mode
		// | _BV(UPM01)			// UPMn1:0: Parity Mode
		// | _BV(UPM00)			// 0: No parity
		// | _BV(USBS0)			// Stop Bit Select: 0: 1 Stop bit
		| _BV(UCSZ01)		// UCSZn1:0: Character Size
		| _BV(UCSZ00)		// 011: 8 Bit
		// | _BV(UCPOL0)		// Clock Polarity (Unused, set 0)
		;

	// Setup the baud rate.
	UBRR0H = (USART_BAUD_CONFIG_VAL >> 8) & 0b00001111; // upper 4 bits are reserved and must be set 0.
	UBRR0L = USART_BAUD_CONFIG_VAL & 0xff;

}


void PPPUsartStart(ppp_pcb *ppp) {

	initReadBuffer();

	xTaskCreate(readerTaskFunc,"PPPReadTask",configMINIMAL_STACK_SIZE*2,ppp,TASK_PRIO_DRIVERS,&readDriverTask);

	taskENTER_CRITICAL();

	UCSR0B |= 0
	| _BV(RXCIE0)	// RX Complete Interrupt Enable
	// | _BV(TXCIE0)	// TX Complete Interrupt Enable
	// | _BV(UDRIE0)	// USART Data Register Empty Interrupt Enable
	| _BV(RXEN0)		// Receiver Enable
	| _BV(TXEN0)		// Transmitter Enable
	// | _BV(UCSZ02)	// UCSZn2 Character Size Bit 2: 011: 8 Bit
	// | _BV(RXB80)		// Receive Data Bit 8 (R/O)
	// | _BV(TXB80)		// Transmit Data Bit 8 (N/A here)
	;

	taskEXIT_CRITICAL();
}

void PPPUsartStop() {
	taskENTER_CRITICAL();

	UCSR0B &= ~(0
	| _BV(RXCIE0)	// RX Complete Interrupt Enable
	| _BV(TXCIE0)	// TX Complete Interrupt Enable
	| _BV(UDRIE0)	// USART Data Register Empty Interrupt Enable
	| _BV(RXEN0)		// Receiver Enable
	| _BV(TXEN0)		// Transmitter Enable
	// | _BV(UCSZ02)	// UCSZn2 Character Size Bit 2: 011: 8 Bit
	// | _BV(RXB80)		// Receive Data Bit 8 (R/O)
	// | _BV(TXB80)		// Transmit Data Bit 8 (N/A here)
	)
	;

	taskEXIT_CRITICAL();
}

void debugAssertOut(const char* assertText,const char* filename,int line);

/**
 * \brief PPPoS serial output callback
 *
 * @param pcb PPP control block
 * @param data buffer to write to serial port
 * @param len length of the data buffer
 * @param ctx optional user-provided callback context pointer
 * @return len if write succeed
 */
u32_t PPPUsartSend(ppp_pcb *pcb, u8_t *data, u32_t len, void *ctx) {
uint16_t remainingLen = (uint16_t)len;
TPPPBufferIndex lWriteBufferDataStart;
TPPPBufferIndex lWriteBufferFreeStart;
TPPPBufferIndex lNumInWriteBuffer;

	if (len == 0) {
		return 0;
	}

	if (ppp_usart_atmegaGuard1 != 0) {
		debugAssertOut("Guard 1 modified","Guard1 ppp_usart_atmega.c",__LINE__);
	}
	if (ppp_usart_atmegaGuard2 != 0) {
		debugAssertOut("Guard 2 modified","Guard2 ppp_usart_atmega.c",__LINE__);
	}

	if (xSemaphoreTake(ppp_usart_atmegaWriteMutex,portMAX_DELAY) != pdTRUE) {
		// Someone is blocking the sender overly long
		// should never happen. This should be the callback from the LWIP task only
		DEBUG_OUT("Send: Mutex timeout\r\n");
		return 0;
	}

	DEBUG_OUT("PPPUsartSend len = ");
	DEBUG_UINT_OUT((unsigned int)len);
	DEBUG_OUT("\r\n");
	for (;;) {
		// Get a local snapshot of the administrative data
		portENTER_CRITICAL();
		waitingWriteTask = NULL;
		portMEMORY_BARRIER();
		lWriteBufferDataStart = writeBufferDataStart;
		lWriteBufferFreeStart = writeBufferFreeStart;
		lNumInWriteBuffer = numInWriteBuffer;
		portEXIT_CRITICAL();
		// Is there space in the buffer at all?
		if (lNumInWriteBuffer < PPP_BUFFER_SIZE) {
			TPPPBufferIndex lenToWrite;
			if (lWriteBufferFreeStart >= lWriteBufferDataStart) {
				// The free space wraps around the buffer end.
				if ((PPP_BUFFER_SIZE - lWriteBufferFreeStart) < remainingLen) {
					lenToWrite = PPP_BUFFER_SIZE - lWriteBufferFreeStart;
				} else {
					lenToWrite = remainingLen;
				}
			} else { // if (lWriteBufferFreeStart > lWriteBufferDataStart)
				// The free space in the buffer is one chunk.
				if (remainingLen > (PPP_BUFFER_SIZE - lNumInWriteBuffer)) {
					lenToWrite = (PPP_BUFFER_SIZE - lNumInWriteBuffer);
				} else {
					lenToWrite = remainingLen;
				}
			} // if (lWriteBufferFreeStart > lWriteBufferDataStart)

			if ((lWriteBufferFreeStart + lenToWrite) > PPP_BUFFER_SIZE) {
				debugAssertOut("Buffer exceeded","Write buffer exceeded ppp_usart_atmega.c",__LINE__);
			}

			memcpy (writeBuffer + lWriteBufferFreeStart,data,lenToWrite);

			// Adjust the administration
			portENTER_CRITICAL();
			writeBufferFreeStart += lenToWrite;
			if (writeBufferFreeStart >= PPP_BUFFER_SIZE) {
				// start again at the buffer start
				writeBufferFreeStart = 0;
			}
			numInWriteBuffer += lenToWrite;
			remainingLen -= lenToWrite;

			if (numInWriteBuffer == PPP_BUFFER_SIZE && remainingLen > 0) {
				waitingWriteTask = xTaskGetCurrentTaskHandle();
				xTaskNotifyStateClear(waitingWriteTask);
			}

			// ensure that memory is written from the registers before leaving the critical section.
			portMEMORY_BARRIER();

			// Enable the send buffer empty interrupt thus starting the sender
			UCSR0B |= _BV(UDRIE0);
			portEXIT_CRITICAL();

			if (remainingLen == 0) {
				// Leave the loop. You are done here.
				break;
			}

			if (waitingWriteTask) {
				// Wait for the write ISR to empty the buffer
				// Or just try again after 100ms.
				ulTaskNotifyTake(pdTRUE,pdMS_TO_TICKS(1000));
			}

		}

	}

//	DEBUG_OUT("Send: ");
//	DEBUG_INT_OUT((int)len);
//	DEBUG_OUT("\r\n");
	DEBUG_OUT("PPPUsartSend done \r\n");

	xSemaphoreGive(ppp_usart_atmegaWriteMutex);
	return len;
}


static void readerTaskFunc(void* ctx) {
	ppp_pcb *ppp = (ppp_pcb *)ctx;
	TPPPBufferIndex lReadBufferFreeStart;
	TPPPBufferIndex lReadBufferDataStart;
	TPPPBufferIndex lNumInReadBuffer;


	for (;;) {
		// uint32_t taskNotifyRC = ulTaskNotifyTake(pdTRUE,pdMS_TO_TICKS(1000));
		ulTaskNotifyTake(pdTRUE,pdMS_TO_TICKS(100));

		TPPPBufferIndex readBlockLen;
		portENTER_CRITICAL();
		lReadBufferFreeStart = readBufferFreeStart;
		lReadBufferDataStart = readBufferDataStart;
		lNumInReadBuffer = numInReadBuffer;
		portEXIT_CRITICAL();

		if (lNumInReadBuffer > 0) {
			if (lReadBufferDataStart > lReadBufferFreeStart) {
				// There is a wrap-around in the read data
				readBlockLen = PPP_BUFFER_SIZE - lReadBufferDataStart;
			} else {
				readBlockLen = numInReadBuffer;
			}

//			if (taskNotifyRC == 0) {
//				DEBUG_OUT("ReciveTO: ");
//			} else {
//				DEBUG_OUT("Recive: ");
//			}
//			DEBUG_INT_OUT((int)readBlockLen);
//			DEBUG_OUT("\r\n");

			pppos_input_tcpip(ppp,readBuffer + lReadBufferDataStart,readBlockLen);

			portENTER_CRITICAL();
			numInReadBuffer -= readBlockLen;
			readBufferDataStart += readBlockLen;
			if (readBufferDataStart >= PPP_BUFFER_SIZE) {
				readBufferDataStart = 0;
			}
			portMEMORY_BARRIER();
			portEXIT_CRITICAL();
		}

	} // for (;;) - Outer loop endless
}

ISR(USART0_RX_vect) {

	uint8_t recvChar;
	/* Read the status register before the received character
	 * because reading the read buffer clears the frame and parity error bits
	 */
	uint8_t statusRegA;
	BaseType_t higherPrioTaskWoken;
	bool wakeReaderTask = false;

	/* Read the status register before the received character
	 * because reading the read buffer clears the frame and parity error bits
	 */
	statusRegA = UCSR0A;
	// Loop while the receiver FIFO contains data
	while (statusRegA & _BV(RXC0)) {
		recvChar = UDR0;

		// If the read buffer is full drop the input.
		if (numInReadBuffer < PPP_BUFFER_SIZE) {
			readBuffer[readBufferFreeStart] = recvChar;
			readBufferFreeStart ++;
			numInReadBuffer ++;

			if (readBufferFreeStart == PPP_BUFFER_SIZE) {
				readBufferFreeStart = 0;
				// Buffer wrap-around is one trigger for the reader task.
				wakeReaderTask = true;
			}

			if (numInReadBuffer >= (PPP_BUFFER_SIZE/2) ||
				// Do not notify when the flag sequence is the only character in the buffer.
				(recvChar == PPP_FLAG_SEQUENCE && numInReadBuffer >= 2)) {
				// A frame start/end/separator: Trigger the reader task
				// The read buffer is half full: Trigger the reader task too.
				wakeReaderTask = true;
			}

		}
		// Read the status register for the next iteration.
		statusRegA = UCSR0A;
	} // while (statusRegA & _BV(RXC0))

	if (wakeReaderTask) {
		higherPrioTaskWoken = pdFALSE;
		vTaskNotifyGiveFromISR(readDriverTask,&higherPrioTaskWoken);
#if configUSE_PREEMPTION
		if (higherPrioTaskWoken != pdFALSE) {
			portMEMORY_BARRIER();
			vPortYield();
		}
#endif
	}
}

// The send buffer is ready to take another byte to be sent
ISR(USART0_UDRE_vect) {
	BaseType_t higherPrioTaskWoken;

	if (numInWriteBuffer > 0) {
		uint8_t sendChar = writeBuffer[writeBufferDataStart];
		UDR0 = sendChar;

		numInWriteBuffer --;
		writeBufferDataStart ++;
		if (writeBufferDataStart == PPP_BUFFER_SIZE) {
			// wrap the buffer around.
			writeBufferDataStart = 0;
		}

	}

	if (numInWriteBuffer == 0) {
		// Disable the send buffer empty interrupt
		UCSR0B &= ~_BV(UDRIE0);
	}

	if (waitingWriteTask) {
		higherPrioTaskWoken = pdFALSE;
		vTaskNotifyGiveFromISR(waitingWriteTask,&higherPrioTaskWoken);
		waitingWriteTask = 0;

#if configUSE_PREEMPTION
		if (higherPrioTaskWoken != pdFALSE) {
			portMEMORY_BARRIER();
			vPortYield();
		}
#endif
	}
}
