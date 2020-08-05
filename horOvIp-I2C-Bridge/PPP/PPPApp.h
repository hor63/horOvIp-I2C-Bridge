/*
 * PPPApp.h
 *
 *  Created on: 26.03.2020
 *      Author: kai_horstmann
 */

#ifndef PPP_PPPAPP_H_
#define PPP_PPPAPP_H_

#include <stdbool.h>

/** \brief Initialize PPP
 *
 * *IMPORTANT*: This function *must* be called from within the lwip task!
 * Ideally call it from the tcpip_init_done_fn callback when calling tcpip_init.
 *
 * Sets up PPP.
 * Startup PPP in listen mode. First try to establish the connection.
 * Afterwards enter the listen mode if the connection was not established.
 *
 */
void pppAppInit();

/** \brief Check if the PPP status is running, i.e. if the network connection via PPP is up.
 *
 * @return \p true when the PPP connection is up and operational.
 */
bool isPPPRunning();

#endif /* PPP_PPPAPP_H_ */
