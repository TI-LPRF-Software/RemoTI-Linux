 /**************************************************************************************************
  Filename:       npi_peripherals.c
  Revised:        $Date: 2012-03-21 17:37:33 -0700 (Wed, 21 Mar 2012) $
  Revision:       $Revision: 246 $

  Description:    This file contains Linux platform specific SUB SYSTEM
                  Peripherals' implementation

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

/**************************************************************************************************
 *                                           Includes
 **************************************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <unistd.h>

#include "npi_ipc_client.h"
#include "npi_peripherals.h"
#include "lprfLogging.h"

#ifdef NPI_CAPSENSE
#include "npi_capSense.h"
#endif

#ifdef NPI_ATTENUATOR
#include "npi_attenuator.h"
#endif //NPI_ATTENUATOR

//TO DO, To Rename or suppress
#define RTI_ERROR_SYNCHRONOUS_NPI_TIMEOUT                0xFF
typedef uint8 rStatus_t;

#define msg_memcpy(src, dst, len)	memcpy(src, dst, len)


#define NAME_ELEMENT(element) [element&0x1F] = #element

/**************************************************************************************************
 * @fn          PER_AsynchMsgCback
 *
 * @brief       This function is a NPI callback to the client that indicates an
 *              asynchronous message from Peripherals modules has been received. The
 *              client software is expected to complete this call.
 *
 *              Note: The client must copy this message if it requires it
 *                    beyond the context of this call.
 *
 * input parameters
 *
 * @param       *pMsg - A pointer to an asynchronously received message.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
int PER_AsynchMsgCback( npiMsgData_t *pMsg )
{
	if (pMsg->subSys == RPC_SYS_PERIPHERALS)
	{
		switch (pMsg->cmdId)
		{
		case RPC_CMD_ID_CAPSENSE_SUB_SUBSYSTEM_ID:
		{
#ifdef NPI_CAPSENSE
			CAP_AsynchMsgCback(pMsg);
#endif //NPI_CAPSENSE
			break;
		}
		case RPC_CMD_ID_ATTENUATOR_SUB_SUBSYSTEM_ID:
		{
#ifdef NPI_ATTENUATOR
			ATT_AsynchMsgCback(pMsg);
#endif //NPI_ATTENUATOR
			break;
		}
		case RPC_CMD_ID_SPI_MASTER_SUB_SUBSYSTEM_ID:
		case RPC_CMD_ID_LED_SUBSYSTEM_ID:
		{
			LOG_INFO("Peripheral SubSubSystem 0x%.2X, not supported", pMsg->cmdId);
		}
		default:
			break;
		}
	}
	return 0;
}

/**************************************************************************************************
 **************************************************************************************************/
