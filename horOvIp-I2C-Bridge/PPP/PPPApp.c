/*
 * PPPApp.c
 *
 *  Created on: 26.03.2020
 *      Author: kai_horstmann
 */

#include <string.h>

#include "serDebugOut.h"

#include "FreeRTOS.h"
#include "task.h"

#include "lwip/tcpip.h"
#include "netif/ppp/pppos.h"
#include "netif/ppp/pppapi.h"

#include <PPP/PPPApp.h>
#include "PPP/ppp_usart_atmega.h"
#include "BMX160net.h"


/* The PPP control block */
static ppp_pcb *ppp;

/* The PPP IP interface */
static struct netif ppp_netif;

static bool pppIsRunning = false;

static void ppp_notify_phase_cb(ppp_pcb *pcb, u8_t phase, void *ctx) {
  bool locPppIsRunning = false;

  switch (phase) {

  /* Session is down (either permanently or briefly) */
  DEBUG_OUT_START_MSG();

  case PPP_PHASE_DEAD:
     DEBUG_OUT("PPP phase DEAD");
//    led_set(PPP_LED, LED_OFF);
    break;

  /* We are between two sessions */
  case PPP_PHASE_HOLDOFF:
     DEBUG_OUT("PPP phase HOLDOFF");
//    led_set(PPP_LED, LED_SLOW_BLINK);
    break;

  /* Session just started */
  case PPP_PHASE_INITIALIZE:
     DEBUG_OUT("PPP phase INITIALIZE");
//    led_set(PPP_LED, LED_FAST_BLINK);
    break;

  /* Session is running */
  case PPP_PHASE_RUNNING:
     DEBUG_OUT("PPP phase RUNNING");
     locPppIsRunning = true;

//    led_set(PPP_LED, LED_ON);
    break;
	
  case PPP_PHASE_MASTER:
	  DEBUG_OUT("PPP phase MASTER");
	  break;

  case PPP_PHASE_SERIALCONN:
	  DEBUG_OUT("PPP phase SERIALCONN");
	  break;

  case PPP_PHASE_DORMANT:
	  DEBUG_OUT("PPP phase DORMANT");
	  break;

  case PPP_PHASE_ESTABLISH:
	  DEBUG_OUT("PPP phase ESTABLISH");
	  break;

  case PPP_PHASE_AUTHENTICATE:
	  DEBUG_OUT("PPP phase AUTHENTICATE");
	  break;

  case PPP_PHASE_CALLBACK:
	  DEBUG_OUT("PPP phase CALLBACK");
	  break;

  case PPP_PHASE_NETWORK:
	  DEBUG_OUT("PPP phase NETWORK");
	  break;

  case PPP_PHASE_TERMINATE:
	  DEBUG_OUT("PPP phase ");
	  break;

  case PPP_PHASE_DISCONNECT:
	  DEBUG_OUT("PPP phase ");
	  break;

  default:
     DEBUG_OUT("PPP unkown phase ");
	 DEBUG_INT_OUT((int)phase);
    break;
  }
  DEBUG_OUT_END_MSG();

  pppIsRunning = locPppIsRunning;
}

/*
 * PPP status callback
 * ===================
 *
 * PPP status callback is called on PPP status change (up, down, …) from lwIP
 * core thread
 */

/* PPP status callback example */
static void status_cb(ppp_pcb *pcb, int err_code, void *ctx) {
  struct netif *pppif = ppp_netif(pcb);
  LWIP_UNUSED_ARG(ctx);

  switch(err_code) {
    case PPPERR_NONE: {
#if LWIP_DNS
      const ip_addr_t *ns;
#endif /* LWIP_DNS */
      DEBUG_OUT_START_MSG();
      DEBUG_OUT("PPP status PPPERR_NONE: Connected");
      DEBUG_OUT_END_MSG();
//      printf("status_cb: Connected\n");
#if PPP_IPV4_SUPPORT
//      printf("   our_ipaddr  = %s\n", ipaddr_ntoa(&pppif->ip_addr));
//      printf("   his_ipaddr  = %s\n", ipaddr_ntoa(&pppif->gw));
//      printf("   netmask     = %s\n", ipaddr_ntoa(&pppif->netmask));
#if LWIP_DNS
      ns = dns_getserver(0);
      printf("   dns1        = %s\n", ipaddr_ntoa(ns));
      ns = dns_getserver(1);
      printf("   dns2        = %s\n", ipaddr_ntoa(ns));
#endif /* LWIP_DNS */
#endif /* PPP_IPV4_SUPPORT */
#if PPP_IPV6_SUPPORT
      printf("   our6_ipaddr = %s\n", ip6addr_ntoa(netif_ip6_addr(pppif, 0)));
#endif /* PPP_IPV6_SUPPORT */
      break;
    }
    case PPPERR_PARAM: {
        DEBUG_OUT_START_MSG();
        DEBUG_OUT("PPP status PPPERR_PARAM: Invalid parameter");
        DEBUG_OUT_END_MSG();
//      printf("status_cb: Invalid parameter\n");
      break;
    }
    case PPPERR_OPEN: {
        DEBUG_OUT_START_MSG();
        DEBUG_OUT("PPP status PPPERR_OPEN: Unable to open PPP session");
        DEBUG_OUT_END_MSG();
//      printf("status_cb: Unable to open PPP session\n");
      break;
    }
    case PPPERR_DEVICE: {
        DEBUG_OUT_START_MSG();
        DEBUG_OUT("PPP status PPPERR_DEVICE: Invalid I/O device for PPP");
        DEBUG_OUT_END_MSG();
//      printf("status_cb: Invalid I/O device for PPP\n");
      break;
    }
    case PPPERR_ALLOC: {
        DEBUG_OUT_START_MSG();
        DEBUG_OUT("PPP status PPPERR_ALLOC: Unable to allocate resources");
        DEBUG_OUT_END_MSG();
//      printf("status_cb: Unable to allocate resources\n");
      break;
    }
    case PPPERR_USER: {
        DEBUG_OUT_START_MSG();
        DEBUG_OUT("PPP status PPPERR_USER: User interrupt");
        DEBUG_OUT_END_MSG();
//      printf("status_cb: User interrupt\n");
      break;
    }
    case PPPERR_CONNECT: {
        DEBUG_OUT_START_MSG();
        DEBUG_OUT("PPP status PPPERR_CONNECT: Connection lost");
        DEBUG_OUT_END_MSG();
//      printf("status_cb: Connection lost\n");
      break;
    }
    case PPPERR_AUTHFAIL: {
        DEBUG_OUT_START_MSG();
        DEBUG_OUT("PPP status PPPERR_AUTHFAIL: Failed authentication challenge");
        DEBUG_OUT_END_MSG();
//      printf("status_cb: Failed authentication challenge\n");
      break;
    }
    case PPPERR_PROTOCOL: {
        DEBUG_OUT_START_MSG();
        DEBUG_OUT("PPP status PPPERR_PROTOCOL: Failed to meet protocol");
        DEBUG_OUT_END_MSG();
//      printf("status_cb: Failed to meet protocol\n");
      break;
    }
    case PPPERR_PEERDEAD: {
        DEBUG_OUT_START_MSG();
        DEBUG_OUT("PPP status PPPERR_PEERDEAD: Connection timeout");
        DEBUG_OUT_END_MSG();
//      printf("status_cb: Connection timeout\n");
      break;
    }
    case PPPERR_IDLETIMEOUT: {
        DEBUG_OUT_START_MSG();
        DEBUG_OUT("PPP status PPPERR_IDLETIMEOUT: Idle Timeout");
        DEBUG_OUT_END_MSG();
//      printf("status_cb: Idle Timeout\n");
      break;
    }
    case PPPERR_CONNECTTIME: {
        DEBUG_OUT_START_MSG();
        DEBUG_OUT("PPP status PPPERR_CONNECTTIME: Max connect time reached");
        DEBUG_OUT_END_MSG();
//      printf("status_cb: Max connect time reached\n");
      break;
    }
    case PPPERR_LOOPBACK: {
        DEBUG_OUT_START_MSG();
        DEBUG_OUT("PPP status PPPERR_LOOPBACK: Loopback detected");
        DEBUG_OUT_END_MSG();
//      printf("status_cb: Loopback detected\n");
      break;
    }
    default: {
        DEBUG_OUT_START_MSG();
        DEBUG_OUT("PPP status UNKNOWN: Unknown error ");
        DEBUG_INT_OUT(err_code);
        DEBUG_OUT_END_MSG();
//      printf("status_cb: Unknown error code %d\n", err_code);
      break;
    }
  }

/*
 * This should be in the switch case, this is put outside of the switch
 * case for example readability.
 */

  if (err_code == PPPERR_NONE) {
    return;
  }

  /* ppp_close() was previously called, don't reconnect */
  if (err_code == PPPERR_USER) {
    /* ppp_free(); -- can be called here */
    return;
  }

  /*
   * Try to reconnect in 5 seconds, if you need a modem chatscript you have
   * to do a much better signaling here ;-)
   */
  ppp_connect(pcb, 5);
  /* OR ppp_listen(pcb); */
}


#define MAKE_IP_ADDR(a1,a2,a3,a4) PP_HTONL(LWIP_MAKEU32(a1,a2,a3,a4))

void pppAppInit() {

	/*
	 * Create a new PPPoS interface
	 *
	 * ppp_netif, netif to use for this PPP link, i.e. PPP IP interface
	 * output_cb, PPPoS serial output callback
	 * status_cb, PPP status callback, called on PPP status change (up, down, …)
	 * ctx_cb, optional user-provided callback context pointer
	 */
    DEBUG_OUT_START_MSG();
    DEBUG_OUT("call pppos_create");
    DEBUG_OUT_END_MSG();
	vTaskDelay(200/portTICK_PERIOD_MS);

	memset (&ppp_netif,0,sizeof(ppp_netif));

	ppp = pppos_create(&ppp_netif,
			PPPUsartSend, status_cb, NULL);

	/*
	 * Initiate PPP server listener
	 * ============================
	 */

	/*
	 * Basic PPP server configuration. Can only be set if PPP session is in the
	 * dead state (i.e. disconnected). We don't need to provide thread-safe
	 * equivalents through PPPAPI because those helpers are only changing
	 * structure members while session is inactive for lwIP core. Configuration
	 * only need to be done once.
	 */
	ip4_addr_t addr;

	/* Set our address */
	addr.addr = MAKE_IP_ADDR_I2C_BRIDGE;
	//IP4_ADDR(&addr, 192,168,203,2);
    DEBUG_OUT_START_MSG();
    DEBUG_OUT("ppp_set_ipcp_ouraddr");
    DEBUG_OUT_END_MSG();
	vTaskDelay(200/portTICK_PERIOD_MS);
	ppp_set_ipcp_ouraddr(ppp, &addr);

	/* Set peer(his) address */
	addr.addr = MAKE_IP_PEER_ADDR_I2C_BRIDGE;
	//IP4_ADDR(&addr, 192,168,203,1);
    DEBUG_OUT_START_MSG();
    DEBUG_OUT("ppp_set_ipcp_hisaddr");
    DEBUG_OUT_END_MSG();
	vTaskDelay(200/portTICK_PERIOD_MS);
	ppp_set_ipcp_hisaddr(ppp, &addr);
	/**/

	/* Set primary DNS server */
	//IP4_ADDR(&addr, 192,168,10,20);
	//ppp_set_ipcp_dnsaddr(ppp, 0, &addr);

	/* Set secondary DNS server */
	//IP4_ADDR(&addr, 192,168,10,21);
	//ppp_set_ipcp_dnsaddr(ppp, 1, &addr);

	/* Auth configuration, this is pretty self-explanatory */
    DEBUG_OUT_START_MSG();
    DEBUG_OUT("ppp_set_auth");
    DEBUG_OUT_END_MSG();
	ppp_set_auth(ppp, PPPAUTHTYPE_ANY, "open", "openSecret");

	/* Require peer to authenticate */
    DEBUG_OUT_START_MSG();
    DEBUG_OUT("ppp_set_auth_required");
    DEBUG_OUT_END_MSG();
	ppp_set_auth_required(ppp, 1);
	/* */

    DEBUG_OUT_START_MSG();
    DEBUG_OUT("ppp_set_notify_phase_callback");
    DEBUG_OUT_END_MSG();
	vTaskDelay(200/portTICK_PERIOD_MS);
	ppp_set_notify_phase_callback(ppp, ppp_notify_phase_cb);

	// Setup the serial port.
    DEBUG_OUT_START_MSG();
    DEBUG_OUT("PPPUsartInit");
    DEBUG_OUT_END_MSG();
	vTaskDelay(200/portTICK_PERIOD_MS);
	PPPUsartInit();

	// ... and startup the receiver
    DEBUG_OUT_START_MSG();
    DEBUG_OUT("PPPUsartStart");
    DEBUG_OUT_END_MSG();
	vTaskDelay(200/portTICK_PERIOD_MS);
	PPPUsartStart(ppp);

	/*
	 * Only for PPPoS, the PPP session should be up and waiting for input.
	 *
	 * Note: for PPPoS, ppp_connect() and ppp_listen() are actually the same thing.
	 * The listen call is meant for future support of PPPoE and PPPoL2TP server
	 * mode, where we will need to negotiate the incoming PPPoE session or L2TP
	 * session before initiating PPP itself. We need this call because there is
	 * two passive modes for PPPoS, ppp_set_passive and ppp_set_silent.
	 */
    DEBUG_OUT_START_MSG();
    DEBUG_OUT("ppp_set_passive");
    DEBUG_OUT_END_MSG();
	vTaskDelay(200/portTICK_PERIOD_MS);
	ppp_set_passive(ppp, 1);

	ppp_set_default(ppp);

	/*
	 * Initiate PPP listener (i.e. wait for an incoming connection), can only
	 * be called if PPP session is in the dead state (i.e. disconnected).
	 */
    DEBUG_OUT_START_MSG();
    DEBUG_OUT("ppp_listen");
    DEBUG_OUT_END_MSG();
	vTaskDelay(200/portTICK_PERIOD_MS);
	ppp_listen(ppp);
}


bool isPPPRunning() {
	return pppIsRunning;
}
