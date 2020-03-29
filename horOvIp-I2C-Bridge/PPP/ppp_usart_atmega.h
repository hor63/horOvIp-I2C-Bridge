/*
 * ppp_usart_atmega.h
 *
 *  Created on: 25.03.2020
 *      Author: kai_horstmann
 */

#ifndef PPP_USART_ATMEGA_H_

#include "lwip/tcpip.h"
#include "netif/ppp/pppos.h"
#include "netif/ppp/pppapi.h"


/** \brief Setup the serial port.
 *
 * Do not activate interrupts, i.e. the interface is not yet active.
 */
void PPPUsartInit();

/** \brief Start the PPP receiver task, and activate the interface
 *
 * Starts the receiver driver tasks, and the timer for incomplete packages.
 * Enables the receiver interrupt.
 */
void PPPUsartStart(ppp_pcb *ppp);

/** \brief Stop the PP receiver task
 *
 * Stops the timer, and the receiver interrupt.
 * The reciver task remains on.
 */
void PPPUsartStop();


/**
 * \brief PPPoS serial output callback
 *
 * @param pcb PPP control block
 * @param data buffer to write to serial port
 * @param len length of the data buffer
 * @param ctx optional user-provided callback context pointer
 * @return len if write succeed
 */
u32_t PPPUsartSend(ppp_pcb *pcb, u8_t *data, u32_t len, void *ctx);

#define PPP_USART_ATMEGA_H_

#endif /* PPP_USART_ATMEGA_H_ */
