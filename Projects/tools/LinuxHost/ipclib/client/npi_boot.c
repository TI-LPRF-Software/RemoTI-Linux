 /**************************************************************************************************
  Filename:       npi_boot.c
  Revised:        $Date: 2012-03-21 17:37:33 -0700 (Wed, 21 Mar 2012) $
  Revision:       $Revision: 246 $

  Description:    This file contains Linux platform specific SYS API
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
#include "npi_boot.h"
#include "sb_load.h"

//TO DO, To Rename or suppress
#define RTI_ERROR_SYNCHRONOUS_NPI_TIMEOUT                0xFF
typedef uint8 rStatus_t;


#ifdef __BIG_DEBUG__
#define debug_printf(fmt, ...) printf( fmt, ##__VA_ARGS__)
#else
#define debug_printf(fmt, ...)
#endif

#define msg_memcpy(src, dst, len)	memcpy(src, dst, len)


#define NAME_ELEMENT(element) [element&0x1F] = #element

const char * const SysAreqCmdType_list[2 + 1] = { //15 Cmd Type
		[0 ... 2] = NULL,
		NAME_ELEMENT(MT_SYS_RESET_IND),
		NAME_ELEMENT(MT_SYS_OSAL_TIMER_EXPIRED),
};

/**************************************************************************************************
 * @fn          BOOT_AsynchMsgCback
 *
 * @brief       This function is a NPI callback to the client that indicates an
 *              asynchronous message from BOOT modules has been received. The client software is
 *              expected to complete this call.
 *
 *              Note: The client must copy this message if it requires it
 *                    beyond the context of this call.
 *
 * input parameters
 *
 * @param       *pMsg - A pointer to an asychronously received message.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
int BOOT_AsynchMsgCback( npiMsgData_t *pMsg )
{

	if (pMsg->subSys == RPC_SYS_BOOT)
	{
		debug_printf("[SYS] %s (%d) callback cmd received\n", (SysAreqCmdType_list[pMsg->cmdId & 0x1F] == NULL)?"unknow":SysAreqCmdType_list[pMsg->cmdId & 0x1F], pMsg->cmdId);

		switch( pMsg->cmdId )
		{

			default:
				debug_printf("[SYS] unknown callback command from device: %d\n", pMsg->cmdId);
				break;
		}
	}
	return 0;
}

/**************************************************************************************************
 *
 * @fn          BOOT_WriteReq
 *
 * @brief       This API is used to Write Data.
 *
 * input parameters
 *
 * @param       itemId - The Configuration Interface item identifier.
 * @param       len - The length in bytes of the item identifier's data.
 *
 * output parameters
 *
 * @param       *pValue - Pointer to buffer where read data is placed.
 *
 * @return      to do
 *
 **************************************************************************************************/
uint8 BOOT_WriteReq(uint8  *pValue , uint8  len, uint16 address)
{
  npiMsgData_t pMsg;

  // prep Read Item request
  // Note: no need to send pValue over the NPI
  pMsg.subSys   = RPC_SYS_BOOT;
  pMsg.cmdId    = SB_WRITE_CMD;
  pMsg.len      = 2+len;
  pMsg.pData[0] = (uint8) (address & 0xFF);
  pMsg.pData[1] = (uint8) (address >> 8);

  msg_memcpy( &pMsg.pData[2], pValue, len );

  // send Read Item request to NP RTIS synchronously
  NPI_SendSynchData( &pMsg );

  // DEBUG
  if ( pMsg.pData[0] == RTI_ERROR_SYNCHRONOUS_NPI_TIMEOUT )
  {
//    rtisFatalError( RTI_ERROR_SYNCHRONOUS_NPI_TIMEOUT );
	  return( (rStatus_t)pMsg.pData[0] );
  }

  // return the status, which is stored is the first byte of the payload
  return(pMsg.pData[0]);
}

/**************************************************************************************************
 *
 * @fn          BOOT_ReadReq
 *
 * @brief       This API is used to Read Data.
 *
 * input parameters
 *
 * @param       itemId - The Configuration Interface item identifier.
 * @param       len - The length in bytes of the item identifier's data.
 *
 * output parameters
 *
 * @param       *pValue - Pointer to buffer where read data is placed.
 *
 * @return      to do
 *
 **************************************************************************************************/
uint8 BOOT_ReadReq(uint8  *pValue , uint8  len, uint16 *address)
{
  npiMsgData_t pMsg;

  // prep Read Item request
  // Note: no need to send pValue over the NPI
  pMsg.subSys   = RPC_SYS_BOOT;
  pMsg.cmdId    = SB_READ_CMD;
  pMsg.len      = 2;
  pMsg.pData[0] = (uint8) (*address & 0xFF);
  pMsg.pData[1] = (uint8) (*address >> 8);


  // send Read Item request to NP RTIS synchronously
  NPI_SendSynchData( &pMsg );

  // DEBUG
  if ( pMsg.pData[0] == RTI_ERROR_SYNCHRONOUS_NPI_TIMEOUT )
  {
//    rtisFatalError( RTI_ERROR_SYNCHRONOUS_NPI_TIMEOUT );
	  return( (rStatus_t)pMsg.pData[0] );
  }

  *address = (uint16) pMsg.pData[1];
  *address |=  (uint16) pMsg.pData[2] <<8;

  msg_memcpy( pValue, &pMsg.pData[3], len );
  // return the status, which is stored is the first byte of the payload
  return(pMsg.pData[0]);
}

/**************************************************************************************************
 *
 * @fn          BOOT_HandshakeReq
 *
 * @brief       This API is used to send a Handshake command.
 *
 * input parameters
 *
 * @param       itemId - The Configuration Interface item identifier.
 * @param       len - The length in bytes of the item identifier's data.
 *
 * output parameters
 *
 * @param       *pValue - Pointer to buffer where read data is placed.
 *
 * @return      to do
 *
 **************************************************************************************************/
uint8 BOOT_HandshakeReq( void )
{
  npiMsgData_t pMsg;

  // prep Read Item request
  // Note: no need to send pValue over the NPI
  pMsg.subSys   = RPC_SYS_BOOT;
  pMsg.cmdId    = SB_HANDSHAKE_CMD;
  pMsg.len      = 0;


  // send Read Item request to NP RTIS synchronously
  NPI_SendSynchData( &pMsg );

  // DEBUG
  if ( pMsg.pData[0] == RTI_ERROR_SYNCHRONOUS_NPI_TIMEOUT )
  {
//    rtisFatalError( RTI_ERROR_SYNCHRONOUS_NPI_TIMEOUT );
	  return( (rStatus_t)pMsg.pData[0] );
  }

  // return the status, which is stored is the first byte of the payload
  return(pMsg.pData[0]);
}
/**************************************************************************************************
 *
 * @fn          BOOT_EnableReq
 *
 * @brief       This API is used to send an enable command.
 *
 * input parameters
 *
 * @param       itemId - The Configuration Interface item identifier.
 * @param       len - The length in bytes of the item identifier's data.
 *
 * output parameters
 *
 * @param       *pValue - Pointer to buffer where read data is placed.
 *
 * @return      to do
 *
 **************************************************************************************************/
uint8 BOOT_EnableReq( void )
{
  npiMsgData_t pMsg;

  // prep Read Item request
  // Note: no need to send pValue over the NPI
  pMsg.subSys   = RPC_SYS_BOOT;
  pMsg.cmdId    = SB_ENABLE_CMD;
  pMsg.len      = 0;


  // send Read Item request to NP RTIS synchronously
  NPI_SendSynchData( &pMsg );

  // DEBUG
  if ( pMsg.pData[0] == RTI_ERROR_SYNCHRONOUS_NPI_TIMEOUT )
  {
//    rtisFatalError( RTI_ERROR_SYNCHRONOUS_NPI_TIMEOUT );
	  return( (rStatus_t)pMsg.pData[0] );
  }

  // return the status, which is stored is the first byte of the payload
  return(pMsg.pData[0]);
}
/**************************************************************************************************
 *
 * @fn          BOOT_BootloadReq
 *
 * @brief       This API is used to switch the target in bootload mode.
 *
 * input parameters
 *
 * @param       itemId - The Configuration Interface item identifier.
 * @param       len - The length in bytes of the item identifier's data.
 *
 * output parameters
 *
 * @param       *pValue - Pointer to buffer where read data is placed.
 *
 * @return      to do
 *
 **************************************************************************************************/
void BOOT_BootloadReq( void )
{
  npiMsgData_t pMsg;

  // prep Read Item request
  // Note: no need to send pValue over the NPI
  pMsg.subSys   = RPC_SYS_BOOT;
  pMsg.cmdId    = SB_TGT_BOOTLOAD;
  pMsg.len      = 0;

  NPI_SendAsynchData( &pMsg);
}
/**************************************************************************************************
 **************************************************************************************************/
