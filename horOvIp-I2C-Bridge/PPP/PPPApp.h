/*
 * PPPApp.h
 *
 *  Created on: 26.03.2020
 *      Author: kai_horstmann
 */

#ifndef PPP_PPPAPP_H_
#define PPP_PPPAPP_H_

/** \brief Initialize PPP
 *
 * Sets up PPP.
 * Startup PPP in listen mode. First try to establish the connection.
 * Afterwards enter the listen mode if the connection was not established.
 */
void pppAppInit();

#endif /* PPP_PPPAPP_H_ */
