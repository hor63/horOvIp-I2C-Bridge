/*
 * slip_usart.h
 *
 * Created: 24.08.2019 16:52:34
 *  Author: kai_horstmann
 */ 


#ifndef SLIP_USART_H_
#define SLIP_USART_H_

#include <avr/io.h>
#include <stdbool.h>

#include "uip.h"

extern bool slipBufferReceived;

/// \brief Initialize the buffers and management variables.
void slipInit();

/// \brief Activate the sender and receiver of the USART. Enable the interrupts.
void slipStart();

/// \brief Deactivate the sender and receiver of the USART. Disable the interrupts.
void slipStop();

/** \brief Check if a new complete message was received.
 *
 * If a message was received copy it into the uIP buffer and call \ref uip_input(), and send an answer when applicable.
 * Call this function as part of the main loop of the program.
 */
void slipProcessReadBuffer();

/** \brief Copy an outgoing message from the uIP buffer at the end of the send buffer and send it
 *
 * The message is provided in \ref uip_buf. The length of the message is given by \ref uip_len.
 * This function must be called when a uIP processing call indicates that send data are present in the uIP buffer.
 * The content in \ref uip_buf is copied as an additional segment of the SLIP send buffer.
 * If needed the sending is activated.
 * If the message does not fit in the send buffer it is simply dropped. uIP will deal with re-sending.
 * This function does not block but copies the buffer and queues it for sending asynchronously.
 */
void slipQueueUIPSendMessage();

#endif /* SLIP_USART_H_ */
