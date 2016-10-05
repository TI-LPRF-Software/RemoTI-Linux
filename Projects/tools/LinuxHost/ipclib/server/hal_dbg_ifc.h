/**************************************************************************************************
  Filename:       hal_dbg_ifc.h
  Revised:        $Date: 2012-03-15 13:45:31 -0700 (Thu, 15 Mar 2012) $
  Revision:       $Revision: 237 $

  Description:     This file contains the interface to the CC2533 Flash Programmer.


  Copyright (C) {2016} Texas Instruments Incorporated - http://www.ti.com/


   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions
   are met:

     Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.

     Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the
     distribution.

     Neither the name of Texas Instruments Incorporated nor the names of
     its contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
   OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**************************************************************************************************/
#ifndef HAL_DBG_IFC_H
#define HAL_DBG_IFC_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */

//#include "hal_board.h"
#include "hal_types.h"
#include "npi_lnx.h"
#include "hal_gpio.h"

/*********************************************************************
 * MACROS
 */

#ifndef ATTR_PACKED
# ifdef __GNUC__
#  define ATTR_PACKED __attribute__ ((__packed__))
# else
#  define ATTR_PACKED
# endif
#endif

#define HAL_RNP_RUN()
#define HAL_RNP_RST()

#define HAL_GPIO_ACTIVE_HIGH		1
#define HAL_GPIO_ACTIVE_LOW			0

#define ERROR -1

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */
#if !defined PACK_1
#define PACK_1
#endif

// To be compatible with MS and unix native target
// declare pragma for structure packing
#if defined(_MSC_VER) || defined(unix) || (defined(__ICC430__) && (__ICC430__==1))
#pragma pack(1)
#endif


/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * FUNCTIONS - API
 */
void HalGpioUsage(void);
int Hal_debug_init(void);
int HalGpioDDInit(halGpioCfg_t *gpioCfg);
int HalGpioDCInit(halGpioCfg_t *gpioCfg);
void HalGpioDDClose( void );
void HalGpioDCClose( void );
int Hal_DebugInterface_SynchMsgCback( npiMsgData_t *pMsg );
int Hal_DebugInterface_AsynchMsgCback( npiMsgData_t *pMsg );
int Hal_EnterDebugModeReq(void);
int halGpioDDSetDirection(uint8 direction);

#ifdef __cplusplus
}
#endif

#endif
/******************************************************************************
******************************************************************************/
