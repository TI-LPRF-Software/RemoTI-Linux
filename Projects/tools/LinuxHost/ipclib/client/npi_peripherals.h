/**************************************************************************************************
  Filename:       npi_peripherals.h
  Revised:        $Date: 2011-11-23 12:02:49 -0800 (Wed, 23 Nov 2011) $
  Revision:       $Revision: 108 $

  Description:    This file contains special interface for NP linux surrogate

  Copyright (C) {2012} Texas Instruments Incorporated - http://www.ti.com/


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
#ifndef NPI_PER_H
#define NPI_PER_H


// -- Exported function --
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

#define ERROR -1
/*******************************************/

// Sub-Sub System IDs
#define RPC_CMD_ID_CAPSENSE_SUB_SUBSYSTEM_ID		0x01
#define RPC_CMD_ID_ATTENUATOR_SUB_SUBSYSTEM_ID		0x02
#define RPC_CMD_ID_SPI_MASTER_SUB_SUBSYSTEM_ID		0x03
#define RPC_CMD_ID_LED_SUBSYSTEM_ID          		0x04
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

int PER_AsynchMsgCback( npiMsgData_t *pMsg );


#ifdef __cplusplus
}
#endif

#endif // NPI_PER_H
