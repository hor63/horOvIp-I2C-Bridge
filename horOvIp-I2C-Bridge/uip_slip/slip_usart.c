/*
 * slip_usart.c
 *
 * Created: 24.08.2019 16:52:04
 *  Author: kai_horstmann
 */ 

#include "config.h"

#include <string.h>
#include <stdbool.h>
#include "uip_slip/slip_usart.h"
#include <avr/io.h>
#include <avr/interrupt.h>

#include "serDebugOut.h"

/// From main.c
extern volatile bool mainLoopMustRun;

/// \brief Indicator for the main loop that a package is waiting in the receive buffer
bool slipBufferReceived = false;

/// The term "(F_CPU + SLIP_BAUD_RATE*8)" rounds the UBRR value to the next integer 
#define USART_BAUD_CONFIG_VAL (((F_CPU + SLIP_BAUD_RATE*8)/(16*SLIP_BAUD_RATE))-1)
/// Calculate the actual baud rate in mHz
#define USART_ACTUAL_BAUD_RATE_1000 ((F_CPU*1000)/(16*(USART_BAUD_CONFIG_VAL+1)))
/// Calculate the ratio of the actual and the target baud rate in 1000s. 1000 means a ratio of 1.0, i.e. an exact fit
#define USART_BAUD_RATE_RATIO_PROMILLE (USART_ACTUAL_BAUD_RATE_1000/SLIP_BAUD_RATE)

#if ((USART_BAUD_RATE_RATIO_PROMILLE > 1010) || (USART_BAUD_RATE_RATIO_PROMILLE < 990))
#	error Baud rate deviation is larger than 1%. Please use a more suitable clock which can be divided better to the baud rate.
#endif

/** \brief Continuous send buffer length
 *
 * All send buffers are managed as one continuous buffer, and messages are managed internally.
 * Thus in reality a lot more messages than \ref SLIP_NUM_IP_SEND_BUFFERS will fit when messages are smaller than the maximum size.
 * Please note that the header consists of the length, and index to the next segment
 */
#if ((IP_BUFFER_SIZE + 2) * SLIP_NUM_IP_SEND_BUFFERS) < UINT8_MAX
	typedef uint8_t TSendBufferIndex;
	#define CONT_SEND_BUFFER_LEN ((IP_BUFFER_SIZE + 2) * SLIP_NUM_IP_SEND_BUFFERS)
	#define SEND_INDEX_NONE UINT8_MAX
	// 8 bit index type does not require alignment
	#define ALIGNED_SEND_INDEX(idx) (idx)
#else /* if ((IP_BUFFER_SIZE + 2) * SLIP_NUM_IP_SEND_BUFFERS) < UINT8_MAX */
	#if ((IP_BUFFER_SIZE + 4) * SLIP_NUM_IP_SEND_BUFFERS) < UINT16_MAX
		typedef uint16_t TSendBufferIndex;
		#define CONT_SEND_BUFFER_LEN ((IP_BUFFER_SIZE + 4) * SLIP_NUM_IP_SEND_BUFFERS)
		#define SEND_INDEX_NONE UINT16_MAX
		#define ALIGNED_SEND_INDEX(idx) ((idx + 1) & ~1)
	#else /* if ((IP_BUFFER_SIZE + 4) * SLIP_NUM_IP_SEND_BUFFERS) < UINT16_MAX */
		typedef uint32_t TSendBufferIndex;
		#define CONT_SEND_BUFFER_LEN ((IP_BUFFER_SIZE + 8) * SLIP_NUM_IP_SEND_BUFFERS)
		#define SEND_INDEX_NONE UINT32_MAX
		#define ALIGNED_SEND_INDEX(idx) ((idx + 3) & ~3)
	#endif /* if ((IP_BUFFER_SIZE + 4) * SLIP_NUM_IP_SEND_BUFFERS) < UINT16_MAX */
#endif /* if ((IP_BUFFER_SIZE + 2) * SLIP_NUM_IP_SEND_BUFFERS) < UINT8_MAX */

/** \brief The structure of a segment in the send buffer
 *
 * These segments overlay the continuous send buffer.
 * The segments are never split over the end of the send buffer.
 * Their start is always aligned to the size of \ref TSendBufferIndex
 */
typedef	struct {
		TSendBufferIndex dataLen;
		TSendBufferIndex nextMsgIndex;
	} TSendBufferSegmentHeader;

typedef struct  {
	TSendBufferSegmentHeader header;
	uint8_t data [IP_BUFFER_SIZE];
	} TSendBufferSegment ;


/** \brief The continuous wrap-around send buffer
 * 
 * The occupied part of the buffer is organized in individual message blocks. Message blocks, a.k.a. segments are overlayed structs of type \ref TBufferMsg.
 * All index variables point into sendBuffer. If one index is \ref INDEX_NONE it is invalid, i.e. the segment is not valid.
 * \ref firstSegmentIndex and \ref lastSegmentIndex point to the first and last valid and committed segment in the buffer. If there is none both are \ref INDEX_NONE.
 * \ref firstSegmentIndex is always the segment which is in the process of being transmitted.
 * \ref sendCurrSegment is the physical pointer to the segment which is indexed by \ref firstSegmentIndex. When \ref firstSegmentIndex is INDEX_NONE \ref sendCurrSegment will therefore be NULL.
 * When \ref sendCurrSegment is not NULL \ref sendBufferCursor is the index into sendCurrSegment->data of the last byte which was sent.
 * When \ref sendCurrSegment is NULL the sender is also not active at this time.
 *
 * Please note that segments in the send buffer are never fragmented over the boundary of the buffer. If a new segment does not fit into \ref sendBuffer it will be placed to the start of the sendBuffer.
 * If it does not fit because it would overwrite the first segment in the sendBuffer the new segment is dropped.
 */
static uint8_t sendBuffer [CONT_SEND_BUFFER_LEN];
/// Index to the first (and active) segment in the send buffer. The byte the index points to in \ref sendBuffer is \ref sendCurrSegment.
static TSendBufferIndex sendFirstSegmentIndex = SEND_INDEX_NONE;
/// Index to the last segment in the send buffer. When adding a new segment to be sent it will be placed behind this segment, and will become sendLastSegmentIndex.
static TSendBufferIndex sendLastSegmentIndex = SEND_INDEX_NONE;
/// The current segment which is being sent.
static TSendBufferSegment *sendCurrSegment = NULL;
/// The index to the last byte in sendCurrSegment->data which was sent.
static TSendBufferIndex sendCurrSegmentCursor = SEND_INDEX_NONE;
/// Escaped special character to be sent. If 0 to be ignored, else the character is being send next
static uint8_t sendEscCharToSend = 0;

/** \brief Continuous receive buffer length
 *
 * All receive buffers are managed as one continuous buffer, and messages are managed internally.
 * Thus in reality a lot more messages than \ref SLIP_NUM_IP_RECV_BUFFERS will fit when messages are smaller than the maximum size.
 * Please note that the header consists of the length, and index to the next segment
 */

#if ((IP_BUFFER_SIZE + 2) * SLIP_NUM_IP_RECV_BUFFERS) < UINT8_MAX
	typedef uint8_t TRecvBufferIndex;
	#define CONT_RECV_BUFFER_LEN ((IP_BUFFER_SIZE + 2) * SLIP_NUM_IP_RECV_BUFFERS)
	#define RECV_INDEX_NONE UINT8_MAX
	// 8 bit index type does not require alignment
	#define ALIGNED_RECV_INDEX(idx) (idx)
#else /* if ((IP_BUFFER_SIZE + 2) * SLIP_NUM_IP_RECV_BUFFERS) < UINT8_MAX */
	#if ((IP_BUFFER_SIZE + 4) * SLIP_NUM_IP_RECV_BUFFERS) < UINT16_MAX
		typedef uint16_t TRecvBufferIndex;
		#define CONT_RECV_BUFFER_LEN ((IP_BUFFER_SIZE + 4) * SLIP_NUM_IP_RECV_BUFFERS)
		#define RECV_INDEX_NONE UINT16_MAX
		#define ALIGNED_RECV_INDEX(idx) ((idx + 1) & ~1)
	#else /* if ((IP_BUFFER_SIZE + 4) * SLIP_NUM_IP_RECV_BUFFERS) < UINT16_MAX */
		typedef uint32_t TRecvBufferIndex;
		#define CONT_RECV_BUFFER_LEN ((IP_BUFFER_SIZE + 8) * SLIP_NUM_IP_RECV_BUFFERS)
		#define RECV_INDEX_NONE UINT32_MAX
		#define ALIGNED_RECV_INDEX(idx) ((idx + 3) & ~3)
	#endif /* if ((IP_BUFFER_SIZE + 4) * SLIP_NUM_IP_RECV_BUFFERS) < UINT16_MAX */
#endif /* if ((IP_BUFFER_SIZE + 2) * SLIP_NUM_IP_RECV_BUFFERS) < UINT8_MAX */

/** \brief The structure of a segment in the send buffer
 *
 * These segments overlay the continuous send buffer.
 * The segments are never split over the end of the send buffer.
 * Their start is always aligned to the size of \ref TSendBufferIndex
 */
typedef struct {
		TRecvBufferIndex dataLen;
		TRecvBufferIndex nextMsgIndex;
	} TRecvBufferSegmentHeader;

typedef struct  {
	TRecvBufferSegmentHeader header;
	uint8_t data [IP_BUFFER_SIZE];
	} TRecvBufferSegment ;


/** \brief The continuous wrap-around receive buffer
 * 
 * The occupied part of the buffer is organized in individual message blocks. Message blocks can wrap around the end of the buffer, and and are continued at the start of the buffer.
 * \ref recvBufferStart points to the start of the first segment. At the start of the segment is the segment length of type \ref TRecvBufferIndex. Then come data bytes according to the segment length.
 * The next segment starts immediately with the segment length when one exists. If \ref recvBufferEnd points to the segment end that means that there is no segment following. Instead the memory
 * following up to \ref recvBufferStart is free memory in the buffer.
 * \ref recvBufferCurrMsgStart and \ref recvBufferCursor are used by the receiver interrupt loop to receive the current segment byte-wise. Until the currently received the received data are strictly speaking
 * still in the free buffer part. Only when a full segment was completely received the segment length is entered, and the new segment is committed.
 * When the main loop has handed a segment to uIP it will declare this segment free by setting \ref recvBufferStart to the next segment.
 * When there is no more segment to process the buffer start is *not* reset to the buffer start to avoid interference with the receiver interrupt loop which will continue to receive data asynchonously.
 *
 */
static uint8_t recvBuffer [CONT_RECV_BUFFER_LEN];
/// Index to the first segment in the receive buffer. When adding a new segment to be sent it will be placed behind this segment, and will become sendLastSegmentIndex.
static TRecvBufferIndex recvFirstSegmentIndex = RECV_INDEX_NONE;
/// Index to the last segment in the receive buffer.
static TRecvBufferIndex recvLastSegmentIndex = RECV_INDEX_NONE;
/// Index of first byte behind the last segment. Note that that index can point behind the end of \ref readBuffer!
static TRecvBufferIndex recvBehindLastSegmentIndex = RECV_INDEX_NONE;
/// The current segment which is being received, but is not yet complete.
static TRecvBufferSegment *recvCurrSegment = (TRecvBufferSegment*)recvBuffer;
/// The index of recvCurrSegment in \ref recvBuffer
static TRecvBufferIndex recvCurrSegmentIndex = 0;
/// The index to the last byte in sendCurrSegment->data which was sent.
static TRecvBufferIndex recvSegmentCursor = 0;

/// Flag if the received data shall be ignored until the next END is received.
static bool recvFlushMsg = false;
/// Flag if the last received character was the ESC character.
/// If true the next expected character is the masked ESC or masked END character.
static bool recvEscReceived = false;

// SLIP related stuff
/// SLIP ESC character.
/// \see [RFC 1055](https://tools.ietf.org/html/rfc1055)
#define SLIP_ESC ((uint8_t)0333)
/// SLIP END character.
/// \see [RFC 1055](https://tools.ietf.org/html/rfc1055)
#define SLIP_END ((uint8_t)0300)
/// SLIP Escaped ESC character. Is sent in a sequence with SLIP_ESC when an ESC character is in the user data
/// \see [RFC 1055](https://tools.ietf.org/html/rfc1055)
#define SLIP_ESC_ESC ((uint8_t)0335)
/// SLIP Escaped END character. Is sent in a sequence with SLIP_ESC when an END character is in the user data
/// \see [RFC 1055](https://tools.ietf.org/html/rfc1055)
#define SLIP_ESC_END ((uint8_t)0334)

/** \brief Reset the administration of the read buffer to an empty buffer
 *
 * The caller must block interrupts in case that this is called in the middle of processing.
 */
static void initReadBuffer() {
	// (Re-)initialize the receive buffer administration
	recvFirstSegmentIndex = RECV_INDEX_NONE;
	recvLastSegmentIndex = RECV_INDEX_NONE;
	recvBehindLastSegmentIndex = RECV_INDEX_NONE;
	recvCurrSegment = (TRecvBufferSegment*)recvBuffer;
	recvCurrSegmentIndex = 0;
	recvSegmentCursor = 0;

}

/** \brief Reset the administration of the send to an empty buffer
 *
 * The caller must block interrupts in case that this is called in the middle of processing.
 */
static void initSendBuffer() {
	// (Re-)initialize the send buffer administration
	sendFirstSegmentIndex = SEND_INDEX_NONE;
	sendLastSegmentIndex = SEND_INDEX_NONE;
	sendCurrSegment = NULL;
	sendCurrSegmentCursor = 0;
	sendEscCharToSend = 0;
}

/** \brief Reset the administration of read and send buffer to an empty buffer
 *
 * The caller must block interrupts in case that this is called in the middle of processing.
 */
static void initBuffers() {

	initSendBuffer();

	initReadBuffer();
	recvFlushMsg = false;
	recvEscReceived = false;
}

/** \brief Set up a new working segment for the USART reader interrupt handler
 *
 * Check if there is enough space in the buffer to accommodate another full size reading segment in the buffer.
 * If there is no space \ref recvCurrSegment is set NULL and \ref recvCurrSegmentIndex is RECV_INDEX_NONE.
 *
 */
static inline void setNewRecvSegment() {
	if (!recvCurrSegment) {
		// First check if there is no committed segment at all
		if (recvFirstSegmentIndex == RECV_INDEX_NONE) {
			recvCurrSegmentIndex = 0;
			recvCurrSegment = (TRecvBufferSegment *)(recvBuffer);
		} else { // if (recvFirstSegmentIndex == RECV_INDEX_NONE)
			// Two basic cases are to be distinguished:
			if (recvFirstSegmentIndex < recvBehindLastSegmentIndex) {
				// First: all segments are laying in the read buffer as one continuous block
				// There may be space at the end of the buffer behind the last segment
				// or there is space at the beginning of the buffer.
				// Both possibilities are checked here.
			
				// Check for space at the end of the read buffer
				if ((sizeof(recvBuffer) - recvBehindLastSegmentIndex) >= sizeof(TRecvBufferSegment)) {
					// Place the new segment behind the last committed segment
					recvCurrSegmentIndex = recvBehindLastSegmentIndex;
					recvCurrSegment = (TRecvBufferSegment *)(recvBuffer + recvCurrSegmentIndex);
				} else {
					// Check for space at the beginning of the read buffer.
					if (recvFirstSegmentIndex >= sizeof(TRecvBufferSegment)) {
						// There is space. Place the segment at the start of the read buffer.
						recvCurrSegmentIndex = 0;
						recvCurrSegment = (TRecvBufferSegment *)(recvBuffer);
					}
				}
			} else { // if (recvFirstSegmentIndex < recvBehindLastSegmentIndex)
				// Second: The occupied space starts somewhere in the middle of the read buffer, reached to the end
				// and wraps around the end, and the last segment(s) start at the beginning of the read buffer.
				// There may be a continuous free space in the middle of the buffer.
				if ((recvFirstSegmentIndex - recvBehindLastSegmentIndex) >= sizeof(TRecvBufferSegment)) {
					recvCurrSegmentIndex = recvBehindLastSegmentIndex;
					recvCurrSegment = (TRecvBufferSegment *)(recvBuffer + recvCurrSegmentIndex);
				}
			} // if (recvFirstSegmentIndex < recvBehindLastSegmentIndex)
		} // if (recvFirstSegmentIndex == RECV_INDEX_NONE)
		
		recvSegmentCursor = 0;
	} // if (!recvCurrSegment)
	
}

void slipInit() {

	memset((void*)recvBuffer,0,sizeof(recvBuffer));
	memset((void*)sendBuffer,0,sizeof(sendBuffer));

	initBuffers();

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


void slipStart() {
	cli();
	
	initBuffers();
	
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
	
	sei();
	
}

void slipStop() {
	cli();
	
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
	
	sei();
	
}

void slipProcessReadBuffer() {

	TRecvBufferSegment* currSegment;

	cli();
	
	// Read and process the buffers which have accumulated in the buffer
	while (recvFirstSegmentIndex != RECV_INDEX_NONE) {
	
		// This segment will not be touched by the receiving interrupt procedure.
		// On the other side copying the buffer to uIP, and processing it is a lengthy process.
		// Therefore allow interrupts here
		sei();

		currSegment = (TRecvBufferSegment*)(recvBuffer + recvFirstSegmentIndex);

/*
		DEBUG_OUT("Received segment, len= ");
		DEBUG_UINT_OUT(currSegment->header.dataLen);
		DEBUG_OUT("\r\n");
*/
		memcpy(uip_buf,currSegment->data,currSegment->header.dataLen);
			
		// Process the message by uIP.
		uip_len = currSegment->header.dataLen;
		uip_input();
		// If processing produces a new message queue it for sending it out.
		if (uip_len > 0) {
			slipQueueUIPSendMessage();
		}

		// Force the main loop to run again.
		mainLoopMustRun = true;

		cli();

		recvFirstSegmentIndex = currSegment->header.nextMsgIndex;
		
		// While more space becomes available setup a new receiving segment when
		// there was none in case that the read buffer was full.
		setNewRecvSegment();
		
	} // while (recvFirstSegmentIndex != RECV_INDEX_NONE)

	// Check if there was no receiving segment because of lack of space in the receive buffer.
	// Or if the current segment is still empty
	if (!recvCurrSegment || (recvSegmentCursor == 0)) {
		// At the end of the previous loop no finished segment is present.
		// Reset the whole buffer administration.
		initReadBuffer();
	} // if (!recvCurrSegment)

	sei();
}


void slipQueueUIPSendMessage() {
	
	TSendBufferSegment *lastSegment = NULL;
	TSendBufferSegment *newSegment = NULL;
	TSendBufferIndex newSegmentIndex = 0;
	
	if (uip_len == 0) return;
	
	cli();

	// Check if a new segment would fit.
	if (sendFirstSegmentIndex == SEND_INDEX_NONE ) {
		// Easy. The send buffer is empty.
		newSegment = (TSendBufferSegment *)sendBuffer;
		newSegmentIndex = 0;
	} else { // if (sendFirstSegmentIndex == SEND_INDEX_NONE )
		// There is data present.
		TSendBufferIndex numBytesRequired = sizeof(TSendBufferSegmentHeader) + uip_len;
		lastSegment = (TSendBufferSegment *)(sendBuffer + sendLastSegmentIndex);
		
		// Two cases to distinguish
		if (sendFirstSegmentIndex <= sendLastSegmentIndex) {
			newSegmentIndex = sendLastSegmentIndex + sizeof(TSendBufferSegmentHeader) + lastSegment->header.dataLen;

			// Send data are in one block in the send buffer.
			// Check behind the last segment to the end of the buffer first
			if ((newSegmentIndex + numBytesRequired <= sizeof(sendBuffer))) {
				// OK, fits at the end of the send buffer.
				newSegment = (TSendBufferSegment *)(sendBuffer + newSegmentIndex);
			}
			
			// Not enough space at the end of the buffer. Check the start of the buffer.
			if (!newSegment && (sendFirstSegmentIndex >= numBytesRequired)) {
				newSegmentIndex = 0;
				newSegment = (TSendBufferSegment *)(sendBuffer);
			}
			
		} else { // if (sendFirstSegmentIndex <= sendLastSegmentIndex)
			// The data in the buffer have already wrapped around.
			// So check if the gap between the last segment and the first segment is large enough.
			newSegmentIndex = sendLastSegmentIndex + sizeof(TSendBufferSegmentHeader) + lastSegment->header.dataLen;
			if ((newSegmentIndex + numBytesRequired) <= sendFirstSegmentIndex) {
				// Yo, fits.
				newSegment = (TSendBufferSegment *)(sendBuffer + newSegmentIndex);
			}
		} // if (sendFirstSegmentIndex <= sendLastSegmentIndex)
		
	} // if (sendFirstSegmentIndex == SEND_INDEX_NONE )
	
	sei();
	
	if (newSegment) {
		// Copy the data
		memcpy(newSegment->data,(void*)(uip_buf),uip_len);
		
		// Data are in the send buffer. Now start the sender when it was not yet active.
		cli();
	
		// Commit the new segment.
		newSegment->header.dataLen = uip_len;
		newSegment->header.nextMsgIndex = SEND_INDEX_NONE;
		
		// In the meantime existing segments may have been sent.
		if (sendLastSegmentIndex != SEND_INDEX_NONE) {
			lastSegment = (TSendBufferSegment *)(sendBuffer + sendLastSegmentIndex);
			lastSegment->header.nextMsgIndex = newSegmentIndex;
			sendLastSegmentIndex = newSegmentIndex;
		} else {
			sendFirstSegmentIndex = sendLastSegmentIndex = newSegmentIndex;
			sendCurrSegment = newSegment;
			
			// If the send register is empty send an END character first to flush
			// The receiver on the other side from any junk which may accumulated in the meantime
			if (UCSR0A & _BV(UDRE0)) {
				UDR0 = SLIP_END;
			}
	 		// Enable the send buffer empty interrupt thus starting the sender
	 		UCSR0B |= _BV(UDRIE0);
		}

		sendCurrSegmentCursor = 0;
	
		sei();	
	
	} // if (newSegment)
} 

// Discard any received data in the current segment.
// Reset the read cursor to the start of the current segment.
// Set the flush flag to discard any further characters until the next END character is received.
// Until then the main loop hopefully has processed the data in the buffer, and space is available.
static inline void enterFlushMode () {
	// Reset the current segment, and enter flush mode
	recvFlushMsg = true;
	recvSegmentCursor = 0;

}


ISR(USART0_RX_vect) {
	
	uint8_t recvChar;
	uint8_t statusRegA = UCSR0A; // Read the status register before the received character because reading the read buffer clears the frame and parity error bits
	
	// Loop while the receiver FIFO contains data
	while (statusRegA & _BV(RXC0)) {
		recvChar = UDR0;


		// Throw away any data when the buffer is full, or in flush mode until the next END character is received.
		if (recvFlushMsg ) {
			if (recvChar == SLIP_END) {
				// Stop flushing when an END character is received.
				recvFlushMsg = false;
				mainLoopMustRun = true;
				DEBUG_OUT("Leave flush mode.\r\n");
			}
		} else { // if (recvFlushMsg )
			// Normal receive mode
			
			// Deal with various error conditions which lead to entering the flush mode
			if (!recvCurrSegment  // No receiver segment available because the buffer is full. 
				|| (statusRegA & (_BV(FE0)|_BV(DOR0)|_BV(UPE0))) // Framing Error, buffer overrun, parity error
				) {
				if (!recvCurrSegment) {
					DEBUG_OUT("Flush Mode because no recv segment present.\r\n");
				} else {
					DEBUG_OUT("Flush Mode Ser hardware error.\r\n");
				}
				enterFlushMode();	
			} else { // Various error conditions
				// No error conditions
				if (recvChar == SLIP_ESC) {
					// ESC is the first byte of a 2-byte sequence of either ESC-ESC_END or ESC_ESC
					recvEscReceived = true;
				} else {
					if (recvChar == SLIP_END) {
						// The end of the current buffer segment.
						// Ignore empty segments, i.e. two or more consecutive END characters.
						// Some implementations send an END at the state of a message first to have a clear segregation of a new message from
						// potential junk on the line between the two messages.
						if (recvCurrSegment && recvSegmentCursor > 0) {
							TRecvBufferSegment *lastSegment;
								
							// Write the segment length into the segment header.
							recvCurrSegment->header.dataLen = recvSegmentCursor;
							// And set the next segment to none.
							recvCurrSegment->header.nextMsgIndex = SEND_INDEX_NONE;
							// When there is no other segment set this one as the one and only.
							if (recvLastSegmentIndex == SEND_INDEX_NONE) {
								recvFirstSegmentIndex = recvLastSegmentIndex = recvCurrSegmentIndex;
							} else { // if (recvLastSegmentIndex == SEND_INDEX_NONE)
								lastSegment = (TRecvBufferSegment *)(recvBuffer + recvLastSegmentIndex);
								lastSegment->header.nextMsgIndex = recvCurrSegmentIndex;
								recvLastSegmentIndex = recvCurrSegmentIndex;
							} // if (recvLastSegmentIndex == SEND_INDEX_NONE)

							// Keep in mind this index may point beyond the limit of recvBuffer!
							recvBehindLastSegmentIndex = recvCurrSegmentIndex + sizeof(TRecvBufferSegmentHeader) + recvCurrSegment->header.dataLen;
								
							// Switch to the next segment if there is enough space available in the buffer
							recvCurrSegment = NULL;
							setNewRecvSegment();
							
							slipBufferReceived = true;
							mainLoopMustRun = true;
							
						} // if (recvCurrMsgLen > 0)
					} else { // if (recvChar == SLIP_END)

						if (recvEscReceived) {
							recvEscReceived = false;
							switch (recvChar) {
								case SLIP_ESC_END:
									recvChar = SLIP_END;
									break;
								case SLIP_ESC_ESC:
									recvChar = SLIP_ESC;
									break;
								default:
									// Something went wrong here.
									enterFlushMode();
									DEBUG_OUT ("Flush mode: Illegal masked character.\r\n");
									goto nextCharFromFiFO;
							} // switch (recvChar)
						} // if (recvEscReceived)
						
						// Check if the cursor is over the limit (it was incremented which the previous received byte.
						if (recvSegmentCursor >= IP_BUFFER_SIZE) {
							enterFlushMode();
							DEBUG_OUT ("Flush mode: Received segment too long.\r\n");
							goto nextCharFromFiFO;
						}
						
						recvCurrSegment->data[recvSegmentCursor] = recvChar;
						++recvSegmentCursor;
					
					} // if (recvChar == SLIP_END)
					
				} // if (recvChar == SLIP_ESC)
			} // // Various error conditions
		} // if (recvFlushMsg )

		nextCharFromFiFO:
		// Read the status register for the next iteration.
		statusRegA = UCSR0A;
	} // while (statusRegA & _BV(RXC0))
	
	
}


// The send buffer is ready to take another byte to be sent
ISR(USART0_UDRE_vect) {

	if (sendCurrSegment) {
		
		if (sendEscCharToSend != 0) {
			// Send the escaped character from the previous interrupt handler call
			UDR0 = sendEscCharToSend;
			sendEscCharToSend = 0;
			
		} else { // if (sendEscCharToSend != 0)

			if (sendCurrSegmentCursor >= sendCurrSegment->header.dataLen) {
				// At the end of the segment. Send the END character.
				UDR0 = SLIP_END;
			
				// Switch to the next segment if there is one.
				sendFirstSegmentIndex = sendCurrSegment->header.nextMsgIndex;
				if (sendFirstSegmentIndex == SEND_INDEX_NONE) {
					// The last data. Send buffer is empty now.
					sendLastSegmentIndex = SEND_INDEX_NONE;
					sendCurrSegment = NULL;	
			 		// Disable the send buffer empty interrupt
			 		UCSR0B &= ~_BV(UDRIE0);
				} else {
					sendCurrSegment = (TSendBufferSegment *)(sendBuffer + sendFirstSegmentIndex);
					sendCurrSegmentCursor = 0;
				}
			
			} else { // if (sendCurrSegmentCursor >= sendCurrSegment->header.dataLen)
				uint8_t sendChar = sendCurrSegment->data[sendCurrSegmentCursor];
				switch (sendChar) {
					case SLIP_END:
						sendChar = SLIP_ESC;
						sendEscCharToSend = SLIP_ESC_END;
						break;
					case SLIP_ESC:
						sendChar = SLIP_ESC;
						sendEscCharToSend = SLIP_ESC_ESC;
						break;
				} // if (sendCurrSegmentCursor >= sendCurrSegment->header.dataLen)

			UDR0 = sendChar;
			++sendCurrSegmentCursor;
				
			}
		} // if (sendEscCharToSend != 0)
		
	} else {
 		// Disable the send buffer empty interrupt
 		UCSR0B &= ~_BV(UDRIE0);
		
	}
}
