/*This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief lwIP abstraction layer for AVR 8.
 *
 * - Compiler:           GNU GCC for AVR 8
 * - Supported devices:  All AVR 8 devices can be used.
 * - AppNote:
 *
 * \author               Kai Horstmann
 *
 *****************************************************************************/

#ifndef __CC_H__
#define __CC_H__

#include "cpu.h"

/*
 * typedef unsigned   char    u8_t;
 * typedef signed     char    s8_t;
 * typedef unsigned   short   u16_t;
 * typedef signed     short   s16_t;
 * typedef unsigned   long    u32_t;
 * typedef signed     long    s32_t;
 * typedef u16_t mem_ptr_t;
 * typedef int sys_prot_t;
 */

/*! Defines for the LWIP_STATS feature. */
#define S16_F   "d"
#define U16_F   "d"
#define X16_F   "d"
#define X32_F   "d"
#define U32_F   "d"
#define S32_F   "d"

#define LWIP_PLATFORM_DIAG(x)   
#define LWIP_PLATFORM_ASSERT(x)   

/* */
#if __GNUC__
#define PACK_STRUCT_BEGIN
#elif __ICCAVR32__
#define PACK_STRUCT_BEGIN _Pragma("pack(1)")
#endif

#if __GNUC__
#define PACK_STRUCT_STRUCT __attribute__ ((__packed__))
#elif __ICCAVR32__
#define PACK_STRUCT_STRUCT
#endif

#if __GNUC__
#define PACK_STRUCT_END
#elif __ICCAVR32__
#define PACK_STRUCT_END _Pragma("pack()")
#endif

#define PACK_STRUCT_FIELD(x) x

#endif /* __CC_H__ */
