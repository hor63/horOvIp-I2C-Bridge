/*
 * Copyright (c) 2001-2003 Swedish Institute of Computer Science.
 * All rights reserved. 
 * 
 * Redistribution and use in source and binary forms, with or without modification, 
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission. 
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED 
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT 
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING 
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
 * OF SUCH DAMAGE.
 *
 * This file is part of the lwIP TCP/IP stack.
 * 
 * Author: Simon Goldschmidt
 *
 */
#ifndef LWIP_HDR_LWIPOPTS_H
#define LWIP_HDR_LWIPOPTS_H

#include "config.h"

/* We link to special sys_arch.c (for basic non-waiting API layers unit tests) */
#define NO_SYS                          0
#define SYS_LIGHTWEIGHT_PROT            0
#define LWIP_NETCONN                    1
#define LWIP_SOCKET                     1
#define LWIP_TCP_KEEPALIVE              1
#define LWIP_NETCONN_FULLDUPLEX         0
#define LWIP_NETBUF_RECVINFO            1

#define TCPIP_THREAD_STACKSIZE          configMINIMAL_STACK_SIZE*3
#define TCPIP_THREAD_PRIO               TASK_PRIO_TCP
#define DEFAULT_THREAD_STACKSIZE        configMINIMAL_STACK_SIZE
#define TCPIP_MBOX_SIZE                 3
#define DEFAULT_RAW_RECVMBOX_SIZE       1
#define DEFAULT_UDP_RECVMBOX_SIZE       1
#define DEFAULT_TCP_RECVMBOX_SIZE       3
#define DEFAULT_ACCEPTMBOX_SIZE         2



#define LWIP_SINGLE_NETIF				1

#define LWIP_ERRNO_STDINCLUDE 			1
#define LWIP_IPV6                       0
#define LWIP_ARP                        0

// Do not allow fragmentation. Not needed for the packet sizes used here.
#define IP_REASSEMBLY                   0
#define IP_FRAG                         0


// #define LWIP_ASSERT_CORE_LOCKED
#define LWIP_TCPIP_CORE_LOCKING         1
#define LWIP_TCPIP_CORE_LOCKING_INPUT   0

#define LWIP_NETCONN_SEM_PER_THREAD		0

#define MEM_LIBC_MALLOC					1
#define MEMP_MEM_MALLOC					1
#define MEM_SIZE						100
#define TCP_MSS                         (256+24)

// Redefine malloc and free to the FreeRTOS functions
#define mem_clib_free lwipPortFree
#define mem_clib_malloc lwipPortMalloc
#define mem_clib_calloc lwipPortCalloc

#define LWIP_SINGLE_NETIF				1

#define PPP_SUPPORT						1
#define LWIP_PPP_API					1
#define PPP_USE_PBUF_RAM                1
#define PAP_SUPPORT                     1
#define CHAP_SUPPORT                    0
#define MSCHAP_SUPPORT                  0
#define EAP_SUPPORT                     0
#define PPP_SERVER                      1
#define PPP_OUR_NAME                    "horOVSensBox"
#define PPP_NOTIFY_PHASE                1
#define PPP_MRU                         296
#define PPP_DEFMRU                      296
#define PPP_MAXMRU                      296

// Limit user and password length for PAP authentication
// This is not a top-secure application :)
#define MAXNAMELEN                      32
#define MAXSECRETLEN                    32


#define VJ_SUPPORT                      0
#define CCP_SUPPORT						0
#define DEFLATE_SUPPORT					0

// Work-around of socket ioctl macros which cannot be determined by avr-gcc
// I let the macros in sockets.h be calculated by a X64_86 gcc, and fill in the constants directly here.
#define FIONBIO 0x8008667e
#define FIONREAD 0x8008667f

// Eliminate one of the worst eaters of memory
// #define ppp_slprintf(buf,buflen,fmt, ...) 1

#endif /* LWIP_HDR_LWIPOPTS_H */
