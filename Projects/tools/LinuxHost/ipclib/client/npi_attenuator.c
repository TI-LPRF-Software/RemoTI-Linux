 /**************************************************************************************************
  Filename:       npi_attenuator.c
  Revised:        $Date: 2012-03-21 17:37:33 -0700 (Wed, 21 Mar 2012) $
  Revision:       $Revision: 246 $

  Description:    This file contains Linux platform specific SYS CAPSENSE
                  Surrogate implementation

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
#include "npi_attenuator.h"
#include "lprfLogging.h"

//TO DO, To Rename or suppress
#define RTI_ERROR_SYNCHRONOUS_NPI_TIMEOUT                0xFF
typedef uint8 rStatus_t;

#define msg_memcpy(src, dst, len)	memcpy(src, dst, len)


#define NAME_ELEMENT(element) [element&0x1F] = #element

/**************************************************************************************************
 * @fn          ATT_AsynchMsgCback
 *
 * @brief       This function is a NPI callback to the client that indicates an
 *              asynchronous message from Attenuator modules has been received. The
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
int ATT_AsynchMsgCback( npiMsgData_t *pMsg )
{
	if (pMsg->cmdId == RPC_CMD_ID_ATTENUATOR_SUB_SUBSYSTEM_ID)
	{
		switch (pMsg->pData[0])
		{
//		case RPC_CMD_ID_CAPSENSE_BUTTON_IND:
		{
			break;
		}
		default:
			break;
		}
	}
	return 0;
}


/**************************************************************************************************
 * @fn          HalAttenuatorSetAttenuation
 *
 * @brief       This function sets the attenuation value for the two HMC624ALP4E attenuators
 *              connected series on the BeagleBone Black Cape Board. Refer to the HMC624ALP4E
 *              datasheet for more information on attenuation values.
 *
 * input parameters
 *
 * @param       att_0 - 6-bit attenuation value for the HMC624ALP4E attenuator that is directly connected
 *                      to the CC2533 on the BeagleBone Black Cape Board.
 * @param       att_1 - 6-bit attenuation value for the HMC624ALP4E attenuator that is connected to SEROUT
 *                      of att_0.
 * output parameters
 *
 * None.
 *
 * @return      None.
 ***************************************************************************************************/
uint8 HalAttenuatorSetAttenuation(uint8 att_0, uint8 att_1)
{
	npiMsgData_t pMsg;
	pMsg.len = 3;
	pMsg.subSys = RPC_SYS_PERIPHERALS | RPC_CMD_SREQ;
	pMsg.cmdId = RPC_CMD_ID_ATTENUATOR_SUB_SUBSYSTEM_ID;
	pMsg.pData[0] = HAL_ATTENUATOR_SUBSYS_CMD_ID_SET_ATTENUATION;
	pMsg.pData[1] = att_0;
	pMsg.pData[2] = att_1;

	// send Program Buffer request to NP RTIS asynchronously as a confirm is due back
	NPI_SendSynchData( &pMsg );

	// DEBUG
	if ( pMsg.pData[0] == RTI_ERROR_SYNCHRONOUS_NPI_TIMEOUT )
	{
		  return( (rStatus_t)pMsg.pData[0] );
	}

	// return the status, which is stored is the second byte of the payload
	return(pMsg.pData[1]);
}



/**************************************************************************************************
 **************************************************************************************************/
