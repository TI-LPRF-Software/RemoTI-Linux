/**************************************************************************************************
  Filename:       npi_rti.c
  Revised:        $Date: 2012-03-21 17:37:33 -0700 (Wed, 21 Mar 2012) $
  Revision:       $Revision: 246 $

  Description:    This file contains Linux platform specific RTI API
                  Surrogate implementation

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
#include "hal_defs.h"
#include "rti_lnx.h"
#include "tiLogging.h"


#if (defined FEATURE_MSO) && (FEATURE_MSO == TRUE)
#include "mso_common.h"
#endif

#define msg_memcpy(src, dst, len)	memcpy(src, dst, len)


#define NAME_ELEMENT(element) [element&0x1F] = #element

const char * const RtiAreqCmdType_list[2 + 1] = { //15 Cmd Type
		[0 ... 2] = NULL,
		NAME_ELEMENT(MT_SYS_RESET_IND),
		NAME_ELEMENT(MT_SYS_OSAL_TIMER_EXPIRED),
};

/**************************************************************************************************
 *                                     Local Variable
 **************************************************************************************************/
static uint8 rtiBE=FALSE; // big endian machine flag

/**************************************************************************************************
 *                                     Local Function Prototypes
 **************************************************************************************************/
// Endianness conversion
static void rtiAttribEConv( uint8 attrib, uint8 len, uint8 *pValue );

// Endianness conversion macros
#define RTI_ECONV16(_value) ((((_value) & 0xff)<<8)|(((_value) & 0xff00)>>8))
#define RTI_ECONV32(_value) \
  ((((_value) & 0xff)<<24)|(((_value) & 0xff00)<<8)| \
   (((_value) & 0xff0000)>>8)|(((_value) & 0xff000000)>>24))
#define RTI_PAIRING_ENTRY_REQUIRED_LEN(_field) \
  ((uint16) (&((rcnNwkPairingEntry_t *) 0)->_field) + sizeof(((rcnNwkPairingEntry_t *) 0)->_field))

// Macro to generate little endian 32 bit word to be used as NPI interface
#define RTI_SET_ITEM_WORD( pVal, attrib )                  \
  (pVal)[0] = (uint8)( ((uint32)(attrib) >>  0) & 0xFF );  \
  (pVal)[1] = (uint8)( ((uint32)(attrib) >>  8) & 0xFF );  \
  (pVal)[2] = (uint8)( ((uint32)(attrib) >> 16) & 0xFF );  \
  (pVal)[3] = (uint8)( ((uint32)(attrib) >> 24) & 0xFF );

// Macro to generate little endian 16 bit value to be used as NPI interface
#define RTI_SET_ITEM_HALFWORD( pVal, attrib )              \
  (pVal)[0] = (uint8)( ((uint16)(attrib) >>  0) & 0xFF );  \
  (pVal)[1] = (uint8)( ((uint16)(attrib) >>  8) & 0xFF );


/**************************************************************************************************
 * @fn          RTI_AsynchMsgCback
 *
 * @brief       This function is a NPI callback to the client that inidcates an
 *              asynchronous message has been received. The client software is
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
int RTI_AsynchMsgCback( npiMsgData_t *pMsg )
{
	if (pMsg->subSys == RPC_SYS_RCAF)
	{
		switch( pMsg->cmdId )
		{
		// confirmation to init request
		case RTIS_CMD_ID_RTI_INIT_CNF:
			RTI_InitCnf( (rStatus_t)pMsg->pData[0] );
			break;

			// confirmation to pair request
		case RTIS_CMD_ID_RTI_PAIR_CNF:
			// status, pairing ref table index, pairing table device type
			RTI_PairCnf( (rStatus_t)pMsg->pData[0], pMsg->pData[1], pMsg->pData[2] );
			break;

			// confirmation to pair abort request
		case RTIS_CMD_ID_RTI_PAIR_ABORT_CNF:
			RTI_PairAbortCnf( (rStatus_t) pMsg->pData[0] );
			break;

			// confirmation to allow pair request
		case RTIS_CMD_ID_RTI_ALLOW_PAIR_CNF:
			RTI_AllowPairCnf( (rStatus_t) pMsg->pData[0], pMsg->pData[1], pMsg->pData[2]);
			break;

			// confirmation to send data request
		case RTIS_CMD_ID_RTI_SEND_DATA_CNF:
			RTI_SendDataCnf( (rStatus_t)pMsg->pData[0] );
			break;

			// indication of received data
		case RTIS_CMD_ID_RTI_REC_DATA_IND:
			RTI_ReceiveDataInd( pMsg->pData[0], pMsg->pData[1],
					pMsg->pData[2] | (pMsg->pData[3] << 8), // vendor Id
					pMsg->pData[4],
					pMsg->pData[5],
					pMsg->pData[6],
					&pMsg->pData[7]);
			break;

		case RTIS_CMD_ID_RTI_STANDBY_CNF:
			RTI_StandbyCnf( (rStatus_t) pMsg->pData[0] );
			break;

			// confirmation to send data request
		case RTIS_CMD_ID_RTI_ENABLE_SLEEP_CNF:
			RTI_EnableSleepCnf( (rStatus_t)pMsg->pData[0] );
			break;

			// confirmation to send data request
		case RTIS_CMD_ID_RTI_DISABLE_SLEEP_CNF:
			RTI_DisableSleepCnf( (rStatus_t)pMsg->pData[0] );
			break;

		case RTIS_CMD_ID_RTI_RX_ENABLE_CNF:
			RTI_RxEnableCnf( (rStatus_t ) pMsg->pData[0] );
			break;

		case RTIS_CMD_ID_RTI_UNPAIR_CNF:
			RTI_UnpairCnf( (rStatus_t) pMsg->pData[0],
					pMsg->pData[1] ); // dstIndex
			break;

		case RTIS_CMD_ID_RTI_UNPAIR_IND:
			RTI_UnpairInd( pMsg->pData[0] ); // dstIndex
			break;

		case RTIS_CMD_ID_RTI_RESET_IND:
			RTI_ResetInd();
			break;

		case RTIS_CMD_ID_RTI_IR_IND:
			RTI_IrInd( pMsg->pData[0] ); // irData
			break;

#if (defined FEATURE_MSO) && (FEATURE_MSO == TRUE)
		case RTIS_CMD_ID_RTI_PAIR_IND:
			RTI_PairInd(pMsg->pData[0], pMsg->pData[1], pMsg->pData[2]);
			break;

		case RTIS_CMD_ID_RTI_START_VALIDATION_IND:
			RTI_StartValidationInd(pMsg->pData[0]);
			break;

		case RTIS_CMD_ID_RTI_GET_VALIDATION_STATUS_IND:
			RTI_GetValidationStatusInd(pMsg->pData[0], pMsg->pData[1]);
			break;

		case RTIS_CMD_ID_RTI_BIND_PARAMS_IND:
			if (pMsg->len == 0)
			{
				// Earlier versions of this API does not have any arguments
				RTI_BindingParamsInd(NULL, NULL);
			}
			else if (pMsg->len >= (SADDR_EXT_LEN + MSO_USER_STRING_SIZE))
			{
				RTI_BindingParamsInd((uint8 *)&pMsg->pData[0], (uint8 *)&pMsg->pData[SADDR_EXT_LEN]);
			}
			break;

		case RTIS_CMD_ID_RTI_UPDATE_BACKUP_P_ENTRY:
			RTI_UpdateBackupPairingEntry();
			break;
#endif //FEATURE_MSO

#if (defined FEATURE_ZRC20) && (FEATURE_ZRC20 == TRUE)
		case RTIS_CMD_ID_RTI_BIND_CNF://                     0x30
			RTI_BindCnf( pMsg->pData[0], pMsg->pData[1] ); //
			break;
		case RTIS_CMD_ID_RTI_SEND_PROFILE_CMD_CNF://         0x31
			RTI_SendProfileCommandCnf( pMsg->pData[0] ); //
			break;
		case RTIS_CMD_ID_RTI_BIND_IND://                     0x32
			RTI_BindInd( pMsg->pData[0], pMsg->pData[1] ); //
			break;
		case RTIS_CMD_ID_RTI_START_VALIDATION_IND://         0x33
			RTI_StartValidationInd( pMsg->pData[0] ); //
			break;
		case RTIS_CMD_ID_RTI_GET_VALIDATION_STATUS_IND://    0x34
			RTI_GetValidationStatusInd( ); //
			break;
		case RTIS_CMD_ID_RTI_POLL_CONFIG_CNF://              0x35
			RTI_PollConfigCnf( pMsg->pData[0], &pMsg->pData[1] ); //
			break;
		case RTIS_CMD_ID_RTI_IDENTIFICATION_CONFIG_CNF://    0x36
			RTI_IdentificationConfigCnf( pMsg->pData[0] ); //
			break;
		case RTIS_CMD_ID_RTI_POLL_CNF://                     0x37
			RTI_PollCnf( pMsg->pData[0] ); //
			break;
		case RTIS_CMD_ID_RTI_KEY_EXCHANGE_CNF://             0x38
			RTI_KeyExchangeCnf( pMsg->pData[0] ); //
			break;
		case RTIS_CMD_ID_RTI_GET_ATTRIBUTE_REQ://            0x39
			RTI_GetAttributeReq( pMsg->pData[0], pMsg->pData[1], BUILD_UINT16(pMsg->pData[2], pMsg->pData[3]) ); //
			break;
		case RTIS_CMD_ID_RTI_SET_ATTRIBUTE_REQ://            0x3a
			RTI_SetAttributeReq( pMsg->pData[0], (gdpAttrHeader_t *)&pMsg->pData[1], &pMsg->pData[1 + sizeof( gdpAttrHeader_t )] ); //
			break;
		case RTIS_CMD_ID_RTI_SEND_IRDB_VENDOR_SUPPORT_CNF:// 0x3b
			RTI_SendIrdbVendorSupportCnf( pMsg->pData[0] ); //
			break;
		case RTIS_CMD_ID_RTI_SEND_MAPPABLE_ACTIONS_CNF://    0x3c
			RTI_SendMappableActionsCnf( pMsg->pData[0] ); //
			break;
		case RTIS_CMD_ID_RTI_GET_ACTION_MAPPINGS_CNF://      0x3d
			RTI_GetActionMappingsCnf( pMsg->pData[0], pMsg->pData[1], &pMsg->pData[2] ); //
			break;
		case RTIS_CMD_ID_RTI_HA_SUPPORTED_ANNOUNCE_CNF://    0x3e
			RTI_HaSupportedAnnounceCnf( pMsg->pData[0] ); //
			break;
		case RTIS_CMD_ID_RTI_PULL_HA_ATTRIBUTES_CNF://       0x3f
			RTI_PullHaAttributesCnf( pMsg->pData[0] ); //
			break;
		case RTIS_CMD_ID_RTI_POLL_IND://                     0x40
			RTI_PollInd( pMsg->pData[0], pMsg->pData[1] ); //
			break;
		case RTIS_CMD_ID_RTI_UNBIND_CNF://                   0x41
			RTI_UnbindCnf( pMsg->pData[0], pMsg->pData[1] ); //
			break;
		case RTIS_CMD_ID_RTI_UNBIND_IND://                   0x42
			RTI_UnbindInd( pMsg->pData[0] ); //
			break;
		case RTIS_CMD_ID_RTI_BIND_ABORT_CNF://               0x43
			RTI_BindAbortCnf( pMsg->pData[0] ); //
			break;
#endif //ZRC20_PROFILE
		default:
			LOG_ERROR("%s(): Unhandled API (0x%.2X)!\n", __FUNCTION__, pMsg->cmdId);
			break;
		}
	}
	else if (pMsg->subSys == RPC_SYS_RCN_CLIENT)
	{
#ifdef _WIN32 // TODO: remove this compile flag once RCNS is ported to an app processor
		RCNS_AsynchMsgCback(pMsg);
#endif
	}
	return 0;
}

/**************************************************************************************************
 * @fn          rtiAttribEConv
 *
 * @brief       This function converts endianness of an attribute value if the current machine
 *              is a big endian machine.
 *              This function will not do anything if the machine is a little endian machine.
 *
 * input parameters
 *
 * @param       attrib - attribute identifier
 * @param       len    - length of the value buffer
 * @param       pValue - buffer where attribute value is stored
 *
 * output parameters
 *
 * @param       pValue - the buffer is rewritten with swapped endianness value.
 *
 * @return      None.
 **************************************************************************************************
 */
static void rtiAttribEConv( uint8 attrib, uint8 len, uint8 *pValue )
{
  if (rtiBE)
  {
    if (attrib == RTI_SA_ITEM_PT_CURRENT_ENTRY)
    {
      if (len >= sizeof(rcnNwkPairingEntry_t))
      {
        // Note that endianness conversion will not occur if the retrieved attribute length is
        // smaller.
        rcnNwkPairingEntry_t *pEntry = (rcnNwkPairingEntry_t *) pValue;

        pEntry->srcNwkAddress = RTI_ECONV16(pEntry->srcNwkAddress);
        // Note that IEEE address is not converted. It is always supposed to be little endian
        pEntry->panId = RTI_ECONV16(pEntry->panId);
        pEntry->nwkAddress = RTI_ECONV16(pEntry->nwkAddress);
        pEntry->recFrameCounter = RTI_ECONV32(pEntry->recFrameCounter);
        pEntry->vendorIdentifier = RTI_ECONV16(pEntry->vendorIdentifier);
      }
    }
    else
    {
      // all other attributes are natural number
      uint8 i, j, buf;

      for (i = 0, j = len - 1; i < j; i++, j--)
      {
        buf = pValue[i];
        pValue[i] = pValue[j];
        pValue[j] = buf;
      }
    }
  }
}

/**************************************************************************************************
 *
 * @fn          RTI_ReadItemEx
 *
 * @brief       This API is used to read an item from a Profile's Configuration Interface.
 *
 * input parameters
 *
 * @param       profileId - The Profile identifier.
 * @param       itemId - The Configuration Interface item identifier.
 * @param       len - The length in bytes of the item identifier's data.
 *
 * output parameters
 *
 * @param       *pValue - Pointer to buffer where read data is placed.
 *
 * @return      RTI_SUCCESS, RTI_ERROR_NOT_PERMITTED, RTI_ERROR_INVALID_INDEX,
 *              RTI_ERROR_INVALID_PARAMETER, RTI_ERROR_UNKNOWN_PARAMETER,
 *              RTI_ERROR_UNSUPPORTED_ATTRIBUTE, RTI_ERROR_OSAL_NV_OPER_FAILED,
 *              RTI_ERROR_OSAL_NV_ITEM_UNINIT, RTI_ERROR_OSAL_NV_BAD_ITEM_LEN
 *
 **************************************************************************************************/
rStatus_t RTI_ReadItemEx( uint8 profileId, uint8 itemId, uint8 len, uint8 *pValue )
{
  npiMsgData_t pMsg;

  // prep Read Item request
  // Note: no need to send pValue over the NPI
  pMsg.subSys   = RPC_SYS_RCAF;
  pMsg.cmdId    = RTIS_CMD_ID_RTI_READ_ITEM_EX;
  pMsg.len      = 3;
  pMsg.pData[0] = profileId;
  pMsg.pData[1] = itemId;
  pMsg.pData[2] = len;


  // send Read Item request to NPI socket synchronously
  NPI_SendSynchData( &pMsg );


  // DEBUG
  if ( pMsg.pData[0] == RTI_ERROR_SYNCHRONOUS_NPI_TIMEOUT )
  {
//    rtisFatalError( RTI_ERROR_SYNCHRONOUS_NPI_TIMEOUT );
  }

  // copy the reply data to the client's buffer
  // Note: the first byte of the payload is reserved for the status
  msg_memcpy( pValue, &pMsg.pData[1], len );

  // perform endianness change
  rtiAttribEConv( itemId, len, pValue );

  // return the status, which is stored is the first byte of the payload
  return( (rStatus_t)pMsg.pData[0] );
}

/**************************************************************************************************
 *
 * @fn          RTI_ReadIndexedItem
 *
 * @brief       This API is used to read an indexed item from a Profile's Configuration Interface.
 *
 * input parameters
 *
 * @param       profileId - The Profile identifier.
 * @param       itemId - The Configuration Interface item identifier.
 * @param       index - The Configuration Interface item index identifier.
 * @param       len - The length in bytes of the item identifier's data.
 *
 * output parameters
 *
 * @param       *pValue - Pointer to buffer where read data is placed.
 *
 * @return      RTI_SUCCESS, RTI_ERROR_NOT_PERMITTED, RTI_ERROR_INVALID_INDEX,
 *              RTI_ERROR_INVALID_PARAMETER, RTI_ERROR_UNKNOWN_PARAMETER,
 *              RTI_ERROR_UNSUPPORTED_ATTRIBUTE, RTI_ERROR_OSAL_NV_OPER_FAILED,
 *              RTI_ERROR_OSAL_NV_ITEM_UNINIT, RTI_ERROR_OSAL_NV_BAD_ITEM_LEN
 *
 **************************************************************************************************/
rStatus_t RTI_ReadIndexedItem( uint8 profileId, uint8 itemId, uint8 index, uint8 len, uint8 *pValue )
{
  npiMsgData_t pMsg;

  // prep Read Item request
  // Note: no need to send pValue over the NPI
  pMsg.subSys   = RPC_SYS_RCAF;
  pMsg.cmdId    = RTIS_CMD_ID_RTI_READ_INDEXED_ITEM_EX;
  pMsg.len      = 4;
  pMsg.pData[0] = profileId;
  pMsg.pData[1] = itemId;
  pMsg.pData[2] = index;
  pMsg.pData[3] = len;

  // send Read Item request to NPI socket synchronously
  NPI_SendSynchData( &pMsg );


  // DEBUG
  if ( pMsg.pData[0] == RTI_ERROR_SYNCHRONOUS_NPI_TIMEOUT )
  {
//    rtisFatalError( RTI_ERROR_SYNCHRONOUS_NPI_TIMEOUT );
  }

  // copy the reply data to the client's buffer
  // Note: the first byte of the payload is reserved for the status
  msg_memcpy( pValue, &pMsg.pData[1], len );

  // perform endianness change
  rtiAttribEConv( itemId, len, pValue );

  // return the status, which is stored is the first byte of the payload
  return( (rStatus_t)pMsg.pData[0] );
}

/**************************************************************************************************
 *
 * @fn          RTI_ReadItem
 *
 * @brief       This API is used to read the RTI Configuration Interface item
 *              from the Configuration Parameters table, the State Attributes
 *              table, or the Constants table.
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
 * @return      RTI_SUCCESS, RTI_ERROR_NOT_PERMITTED, RTI_ERROR_INVALID_INDEX,
 *              RTI_ERROR_INVALID_PARAMETER, RTI_ERROR_UNKNOWN_PARAMETER,
 *              RTI_ERROR_UNSUPPORTED_ATTRIBUTE, RTI_ERROR_OSAL_NV_OPER_FAILED,
 *              RTI_ERROR_OSAL_NV_ITEM_UNINIT, RTI_ERROR_OSAL_NV_BAD_ITEM_LEN
 *
 **************************************************************************************************/
rStatus_t RTI_ReadItem( uint8 itemId, uint8 len, uint8 *pValue )
{
  npiMsgData_t pMsg;

  // prep Read Item request
  // Note: no need to send pValue over the NPI
  pMsg.subSys   = RPC_SYS_RCAF;
  pMsg.cmdId    = RTIS_CMD_ID_RTI_READ_ITEM;
  pMsg.len      = 2;
  pMsg.pData[0] = itemId;
  pMsg.pData[1] = len;

  // send Read Item request to NP RTIS synchronously
  NPI_SendSynchData( &pMsg );

  // DEBUG
  if ( pMsg.pData[0] == RTI_ERROR_SYNCHRONOUS_NPI_TIMEOUT )
  {
//    rtisFatalError( RTI_ERROR_SYNCHRONOUS_NPI_TIMEOUT );
  }

  // copy the reply data to the client's buffer
  // Note: the first byte of the payload is reserved for the status
  msg_memcpy( pValue, &pMsg.pData[1], len );

  // perform endianness change
  rtiAttribEConv( itemId, len, pValue );

  // return the status, which is stored is the first byte of the payload
  return( (rStatus_t)pMsg.pData[0] );
}

/**************************************************************************************************
 *
 * @fn          RTI_WriteItemEx
 *
 * @brief       This API is used to write an item to a Profile's Configuration Interface.
 *
 * input parameters
 *
 * @param       profileId - The Profile identifier.
 * @param       itemId - The Configuration Interface item identifier.
 * @param       len - The length in bytes of the item identifier's data.
 * @param       *pValue - Pointer to buffer where write data is stored.
 *
 * input parameters
 *
 * None.
 *
 * @return      RTI_SUCCESS, RTI_ERROR_NOT_PERMITTED, RTI_ERROR_INVALID_INDEX,
 *              RTI_ERROR_INVALID_PARAMETER, RTI_ERROR_UNKNOWN_PARAMETER,
 *              RTI_ERROR_UNSUPPORTED_ATTRIBUTE, RTI_ERROR_OSAL_NV_OPER_FAILED,
 *              RTI_ERROR_OSAL_NV_ITEM_UNINIT, RTI_ERROR_OSAL_NV_BAD_ITEM_LEN
 *
 **************************************************************************************************/
rStatus_t RTI_WriteItemEx( uint8 profileId, uint8 itemId, uint8 len, uint8 *pValue )
{
  npiMsgData_t pMsg;

  // prep Write Item request
  pMsg.subSys   = RPC_SYS_RCAF;
  pMsg.cmdId    = RTIS_CMD_ID_RTI_WRITE_ITEM_EX;
  pMsg.len      = 3+len;
  pMsg.pData[0] = profileId;
  pMsg.pData[1] = itemId;
  pMsg.pData[2] = len;

  // copy the client's data to be sent
  msg_memcpy( &pMsg.pData[3], pValue, len );

  // perform endianness change
  rtiAttribEConv( itemId, len, &pMsg.pData[3] );

  // send Write Item request to NP RTIS synchronously
  NPI_SendSynchData( &pMsg );

  // DEBUG
  if ( pMsg.pData[0] == RTI_ERROR_SYNCHRONOUS_NPI_TIMEOUT )
  {
//    rtisFatalError( RTI_ERROR_SYNCHRONOUS_NPI_TIMEOUT );
  }

  // return the status, which is stored is the first byte of the payload
  return( (rStatus_t)pMsg.pData[0] );
}

/**************************************************************************************************
 *
 * @fn          RTI_WriteIndexedItem
 *
 * @brief       This API is used to write an indexed item to a Profile's Configuration Interface.
 *
 * input parameters
 *
 * @param       profileId - The Profile identifier.
 * @param       itemId - The Configuration Interface item identifier.
 * @param       index - The Configuration Interface item index identifier.
 * @param       len - The length in bytes of the item identifier's data.
 * @param       *pValue - Pointer to buffer where write data is stored.
 *
 * input parameters
 *
 * None.
 *
 * @return      RTI_SUCCESS, RTI_ERROR_NOT_PERMITTED, RTI_ERROR_INVALID_INDEX,
 *              RTI_ERROR_INVALID_PARAMETER, RTI_ERROR_UNKNOWN_PARAMETER,
 *              RTI_ERROR_UNSUPPORTED_ATTRIBUTE, RTI_ERROR_OSAL_NV_OPER_FAILED,
 *              RTI_ERROR_OSAL_NV_ITEM_UNINIT, RTI_ERROR_OSAL_NV_BAD_ITEM_LEN
 *
 **************************************************************************************************/
rStatus_t RTI_WriteIndexedItem( uint8 profileId, uint8 itemId, uint8 index, uint8 len, uint8 *pValue )
{
  npiMsgData_t pMsg;

  // prep Write Item request
  pMsg.subSys   = RPC_SYS_RCAF;
  pMsg.cmdId    = RTIS_CMD_ID_RTI_WRITE_INDEXED_ITEM_EX;
  pMsg.len      = 4+len;
  pMsg.pData[0] = profileId;
  pMsg.pData[1] = itemId;
  pMsg.pData[2] = index;
  pMsg.pData[3] = len;

  // copy the client's data to be sent
  msg_memcpy( &pMsg.pData[4], pValue, len );

  // perform endianness change
  rtiAttribEConv( itemId, len, &pMsg.pData[4] );

  // send Write Item request to NP RTIS synchronously
  NPI_SendSynchData( &pMsg );

  // DEBUG
  if ( pMsg.pData[0] == RTI_ERROR_SYNCHRONOUS_NPI_TIMEOUT )
  {
//    rtisFatalError( RTI_ERROR_SYNCHRONOUS_NPI_TIMEOUT );
  }

  // return the status, which is stored is the first byte of the payload
  return( (rStatus_t)pMsg.pData[0] );
}

/**************************************************************************************************
 *
 * @fn          RTI_WriteItem
 *
 * @brief       This API is used to write RTI Configuration Interface parameters
 *              to the Configuration Parameters table, and permitted attributes
 *              to the State Attributes table.
 *
 * input parameters
 *
 * @param       itemId  - The Configuration Interface item identifier.
 * @param       len - The length in bytes of the item identifier's data.
 * @param       *pValue - Pointer to buffer where write data is stored.
 *
 * input parameters
 *
 * None.
 *
 * @return      RTI_SUCCESS, RTI_ERROR_NOT_PERMITTED, RTI_ERROR_INVALID_INDEX,
 *              RTI_ERROR_INVALID_PARAMETER, RTI_ERROR_UNKNOWN_PARAMETER,
 *              RTI_ERROR_UNSUPPORTED_ATTRIBUTE, RTI_ERROR_OSAL_NV_OPER_FAILED,
 *              RTI_ERROR_OSAL_NV_ITEM_UNINIT, RTI_ERROR_OSAL_NV_BAD_ITEM_LEN
 *
 **************************************************************************************************/
rStatus_t RTI_WriteItem( uint8 itemId, uint8 len, uint8 *pValue )
{
  npiMsgData_t pMsg;

  // prep Write Item request
  pMsg.subSys   = RPC_SYS_RCAF;
  pMsg.cmdId    = RTIS_CMD_ID_RTI_WRITE_ITEM;
  pMsg.len      = 2+len;
  pMsg.pData[0] = itemId;
  pMsg.pData[1] = len;

  // copy the client's data to be sent
  msg_memcpy( &pMsg.pData[2], pValue, len );

  // perform endianness change
  rtiAttribEConv( itemId, len, &pMsg.pData[2] );

  // send Write Item request to NP RTIS synchronously
  NPI_SendSynchData( &pMsg );

  // DEBUG
  if ( pMsg.pData[0] == RTI_ERROR_SYNCHRONOUS_NPI_TIMEOUT )
  {
//    rtisFatalError( RTI_ERROR_SYNCHRONOUS_NPI_TIMEOUT );
  }
  // DEBUG, test if RNP not lock in boot mode
  if ( pMsg.subSys == RPC_SYS_BOOT )
  {
    return( 1 );
  }

  // return the status, which is stored is the first byte of the payload
  return( (rStatus_t)pMsg.pData[0] );
}

/**************************************************************************************************
 *
 * @fn          RTI_InitReq
 *
 * @brief       This API is used to initialize the RemoTI stack and begin
 *              network operation. A RemoTI confirmation callback is generated
 *              and handled by the client.
 *
 *              The first thing this function does is take a snapshot of the
 *              Configuration Parameters (CP) table stored in NV memory, and
 *              only the snapshot will be used by RTI until another call is made
 *              to this function (presumably due to a reset). Therefore, any
 *              changes to the CP table must be made prior to calling this
 *              function. Once the RTI is started, subsequent changes by the
 *              client to the CP table can be made, but they will have no affect
 *              on RTI operation. The CP table is stored in NV memory and will
 *              persist across a device reset. The client can restore the
 *              the CP table to its default settings by setting the Startup
 *              Option parameter accordingly.
 *
 *              The client's confirm callback will provide a status, which can
 *              be one of the following:
 *
 *              RTI_SUCCESS
 *              RTI_ERROR_INVALID_PARAMTER
 *              RTI_ERROR_UNSUPPORTED_ATTRIBUTE
 *              RTI_ERROR_INVALID_INDEX
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 *
 **************************************************************************************************/
RTILIB_API void RTI_InitReq( void )
{
  npiMsgData_t pMsg;

  // prep Init request
  pMsg.subSys = RPC_SYS_RCAF;
  pMsg.cmdId  = RTIS_CMD_ID_RTI_INIT_REQ;
  pMsg.len    = 0;

  // send Init request to NP RTIS asynchronously as a confirm is due back
  NPI_SendAsynchData( &pMsg );
}

/**************************************************************************************************
 *
 * @fn          RTI_PairReq
 *
 * @brief       This API is used to initiate a pairing process. Note that this
 *              call actually consists of a discovery followed by pairing. That
 *              is a NLME-DISCOVERY.request followed by NLME-PAIR.request.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 *
 **************************************************************************************************/
RTILIB_API void RTI_PairReq( void )
{
  npiMsgData_t pMsg;

  // prep Pair request
  pMsg.subSys = RPC_SYS_RCAF;
  pMsg.cmdId  = RTIS_CMD_ID_RTI_PAIR_REQ;
  pMsg.len    = 0;

  // send Pair request to NP RTIS asynchronously as a confirm is due back
  NPI_SendAsynchData( &pMsg );
}

/**************************************************************************************************
 *
 * @fn          RTI_PairAbortReq
 *
 * @brief       This API is used to abort an on-going pairing process.
 *
 *              The client's confirm callback will provide a status, which can
 *              be one of the following:
 *
 *              RTI_SUCCESS
 *              RTI_ERROR_PAIR_COMPLETE
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 *
 **************************************************************************************************/
RTILIB_API void RTI_PairAbortReq( void )
{
  npiMsgData_t pMsg;

  // prep Pair request
  pMsg.subSys = RPC_SYS_RCAF;
  pMsg.cmdId  = RTIS_CMD_ID_RTI_PAIR_ABORT_REQ;
  pMsg.len    = 0;

  // send Pair request to NP RTIS asynchronously as a confirm is due back
  NPI_SendAsynchData( &pMsg );
}

/**************************************************************************************************
 *
 * @fn          RTI_AllowPairReq
 *
 * @brief       This function is used by the Target application to ready the
 *              node for a pairing request, and thereby allow this node to
 *              respond.
 *
 *              The client's confirm callback will provide a status, which can
 *              be one of the following:
 *
 *              RTI_SUCCESS
 *              RTI_ERROR_OSAL_NO_TIMER_AVAIL
 *              RTI_ERROR_ALLOW_PAIRING_TIMEOUT
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 *
 **************************************************************************************************/
RTILIB_API void RTI_AllowPairReq( void )
{
  npiMsgData_t pMsg;

  // prep Pair request
  pMsg.subSys = RPC_SYS_RCAF;
  pMsg.cmdId  = RTIS_CMD_ID_RTI_ALLOW_PAIR_REQ;
  pMsg.len    = 0;

  // send Pair request to NP RTIS asynchronously as a confirm is due back
  NPI_SendAsynchData( &pMsg );
}

/**************************************************************************************************
 *
 * @fn          RTI_AllowPairAbortReq
 *
 * @brief       This API is used to attempt to abort an on-going allow-pairing process.
 *
 *              It is possible that allow pair is at a state of no return (no aborting).
 *              There is no callback associated to this function call.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 *
 **************************************************************************************************/
RTILIB_API void RTI_AllowPairAbortReq( void )
{
  npiMsgData_t pMsg;

  // prep Pair request
  pMsg.subSys = RPC_SYS_RCAF;
  pMsg.cmdId  = RTIS_CMD_ID_RTI_ALLOW_PAIR_ABORT_REQ;
  pMsg.len    = 0;

  // send Pair request to NP RTIS asynchronously as a confirm is due back
  NPI_SendAsynchData( &pMsg );
}

/**************************************************************************************************
 *
 * @fn          RTI_UnpairReq
 *
 * @brief       This API is used to trigger un-pairing of a pair entry
 *
 * input parameters
 *
 * @param      dstIndex - destination index
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 *
 **************************************************************************************************/
RTILIB_API void RTI_UnpairReq( uint8 dstIndex )
{
  npiMsgData_t pMsg;

  // prep Pair request
  pMsg.subSys = RPC_SYS_RCAF;
  pMsg.cmdId  = RTIS_CMD_ID_RTI_UNPAIR_REQ;
  pMsg.len    = 1;
  pMsg.pData[0] = dstIndex;

  // send Pair request to NP RTIS asynchronously as a confirm is due back
  NPI_SendAsynchData( &pMsg );
}

/**************************************************************************************************
 *
 * @fn          RTI_SendDataReq
 *
 * @brief       This function sends data to the destination specified by the
 *              pairing table index.
 *
 * input parameters
 *
 * @param       dstIndex  - Pairing table index.
 * @param       profileId - Profile identifier.
 * @param       vendorId  - Vendor identifier.
 * @param       txOptions - Transmission options, as specified in Table 2 of the
 *                          RF4CE specification.
 * @param       len       - Number of bytes to send.
 * @param       *pData    - Pointer to buffer of data to be sent.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 *
 **************************************************************************************************/
RTILIB_API void RTI_SendDataReq( uint8 dstIndex, uint8 profileId, uint16 vendorId, uint8 txOptions, uint8 len, uint8 *pData )
{
  npiMsgData_t pMsg;

  // prep Send Data request
  pMsg.subSys   = RPC_SYS_RCAF;
  pMsg.cmdId    = RTIS_CMD_ID_RTI_SEND_DATA_REQ;
  pMsg.len      = 6+len;
  pMsg.pData[0] = dstIndex;
  pMsg.pData[1] = profileId;
  RTI_SET_ITEM_HALFWORD( &pMsg.pData[2], vendorId );
  pMsg.pData[4] = txOptions;
  pMsg.pData[5] = len;

  // copy the client's data to be sent
  msg_memcpy( &pMsg.pData[6], pData, len );

  // send Send Data request to NP RTIS synchronously
  NPI_SendAsynchData( &pMsg );
}

/**************************************************************************************************
 *
 * @fn          RTI_StandbyReq
 *
 * @brief       This API is used by the Target client to place this node into
 *              standby mode. Th properties of the standby consist of the active
 *              period and the duty cycle. These values are set in the
 *              Configuration Parameters table using the RTI_WriteItemReq API,
 *              and go into effect when standby is enabled for this node.
 *
 * input parameters
 *
 * @param       mode - RTI_STANDBY_ON, RTI_STANDBY_OFF
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 *
 **************************************************************************************************/
RTILIB_API void RTI_StandbyReq( uint8 mode )
{
  npiMsgData_t pMsg;

  // prep Standby request
  pMsg.subSys   = RPC_SYS_RCAF;
  pMsg.cmdId    = RTIS_CMD_ID_RTI_STANDBY_REQ;
  pMsg.len      = 1;
  pMsg.pData[0] = mode;

  // send Standby request to NP RTIS asynchronously as a confirm is due back
  NPI_SendAsynchData( &pMsg );
}

/**************************************************************************************************
 *
 * @fn          RTI_RxEnableReq
 *
 * @brief       This API is used to enable the radio receiver, enable the radio
 *              receiver for a specified amount of time, or disable the radio
 *              receiver.
 *
 * input parameters
 *
 * @param       duration - RTI_RX_ENABLE_ON, RTI_RX_ENABLE_OFF, 1..0xFFFE
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 *
 **************************************************************************************************/
RTILIB_API void RTI_RxEnableReq( uint16 duration )
{
  npiMsgData_t pMsg;

  // prep Rx Enable request
  pMsg.subSys = RPC_SYS_RCAF;
  pMsg.cmdId  = RTIS_CMD_ID_RTI_RX_ENABLE_REQ;
  pMsg.len    = 2;
  RTI_SET_ITEM_HALFWORD( &pMsg.pData[0], duration);

  // send Rx Enable request to NP RTIS asynchronously as a confirm is due back
  NPI_SendAsynchData( &pMsg );
}

/**************************************************************************************************
 *
 * @fn          RTI_EnableSleepReq
 *
 * @brief       This API is used to enable sleep on the target.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 *
 **************************************************************************************************/
RTILIB_API void RTI_EnableSleepReq( void )
{
  npiMsgData_t pMsg;

  // prep Enable Sleep request
  pMsg.subSys = RPC_SYS_RCAF;
  pMsg.cmdId  = RTIS_CMD_ID_RTI_ENABLE_SLEEP_REQ;
  pMsg.len    = 0;

  // send Enable Sleep request to NP RTIS asynchronously
  NPI_SendAsynchData( &pMsg );
}

/**************************************************************************************************
 *
 * @fn          RTI_DisableSleepReq
 *
 * @brief       This API is used to disable sleep on the target.
 *
 *              Note: When used from the RTIS, no actual message is sent to the
 *                    RTI, but wakeup bytes are sent instead. The RTI will
 *                    disable sleep as a result.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 *
 **************************************************************************************************/
RTILIB_API void RTI_DisableSleepReq( void )
{
  npiMsgData_t pMsg;

  RTI_PingReq();

  // Then send real message that will be received and confirmed
  pMsg.subSys   = RPC_SYS_RCAF;
  pMsg.cmdId    = RTIS_CMD_ID_RTI_DISABLE_SLEEP_REQ; //RTIS_CMD_ID_RTI_DISABLE_SLEEP_REQ;
  pMsg.len      = 0;

  // send command to slave
  NPI_SendAsynchData( &pMsg );
}

/**************************************************************************************************
 *
 * @fn          RTI_SwResetReq
 *
 * @brief       This function resets the radio processor CPU by way of software triggering.
 *              Implementation of this function is target (CPU) dependent.
 *              Note that in production platform, the reset could be done by chip reset signal
 *              (halResetSlave) and hence use of this function should be restricted to development
 *              phase.
 *
 * input parameters
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 *
 **************************************************************************************************/
RTILIB_API void RTI_SwResetReq( void )
{
  npiMsgData_t pMsg;

  pMsg.subSys   = RPC_SYS_RCAF;
  pMsg.cmdId    = RTIS_CMD_ID_RTI_SW_RESET_REQ;
  pMsg.len      = 0;

  // send command to slave
  NPI_SendAsynchData( &pMsg );

  // wait for 200ms.
//  halDelay(200, 1);
}

/**************************************************************************************************
 *
 * @fn          RTI_TestModeReq
 *
 * @brief       This function is used to place the radio in test modes.
 *              Note that implementation is chip dependent. HAL is not used to reduce code
 *              size overhead.
 *
 * input parameters
 *
 * @param       mode - test mode: RTI_TEST_MODE_TX_RAW_CARRIER, RTI_TEST_MODE_TX_RANDOM_DATA
 *                     or RTI_TEST_MODE_RX_AT_FREQ
 * @param       txPower - transmit power as negative dBm value. That is, 20 implies -20dBm.
 * @param       channel - MAC channel number
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 *
 **************************************************************************************************/
RTILIB_API void RTI_TestModeReq( uint8 mode, int8 txPower, uint8 channel )
{
  npiMsgData_t pMsg;

  pMsg.subSys   = RPC_SYS_RCAF;
  pMsg.cmdId    = RTIS_CMD_ID_RTI_TEST_MODE_REQ;
  pMsg.len      = 3;
  pMsg.pData[0] = mode;
  pMsg.pData[1] = (uint8) txPower;
  pMsg.pData[2] = channel;

  // send command to slave
  NPI_SendAsynchData( &pMsg );
}

/**************************************************************************************************
 *
 * @fn          RTI_TestRxCounterGetReq
 *
 * @brief       This function is used to obtain received packet counter value.
 *
 * input parameters
 *
 * @param       resetFlag - whether or not to reset the counter after reading the value
 *
 * output parameters
 *
 * None.
 *
 * @return      counter value
 *
 **************************************************************************************************/
RTILIB_API uint16 RTI_TestRxCounterGetReq(uint8 resetFlag)
{
  npiMsgData_t pMsg;

  // serialize the request
  pMsg.subSys   = RPC_SYS_RCAF;
  pMsg.cmdId    = RTIS_CMD_ID_RTI_RX_COUNTER_GET_REQ;
  pMsg.len      = 1;
  pMsg.pData[0] = resetFlag;

  // send serialized request to NP RTIS synchronously
  NPI_SendSynchData( &pMsg );

  // return the status, which is stored is the first byte of the payload
  return (pMsg.pData[0] + ((uint16)pMsg.pData[1] << 8));
}

#if (defined FEATURE_ZRC20) && (FEATURE_ZRC20 == TRUE)
/**************************************************************************************************
 *
 * @fn          RTI_PairReq
 *
 * @brief       This API is used to initiate a pairing process. Note that this
 *              call actually consists of a discovery followed by pairing. That
 *              is a NLME-DISCOVERY.request followed by NLME-PAIR.request.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 *
 **************************************************************************************************/
RTILIB_API void RTI_BindReq( uint8 bindingType )
{
  npiMsgData_t pMsg;

  // prep Pair request
  pMsg.subSys = RPC_SYS_RCAF;
  pMsg.cmdId  = RTIS_CMD_ID_RTI_BIND_REQ;
  pMsg.len    = 1;

  pMsg.pData[0] = bindingType;

  // send Pair request to NP RTIS asynchronously as a confirm is due back
  NPI_SendAsynchData( &pMsg );
}

/**************************************************************************************************
 *
 * @fn          RTI_AllowBindReq
 *
 * @brief       This function is used by the Target application to ready the
 *              node for a pairing request, and thereby allow this node to
 *              respond.
 *
 *              The client's confirm callback will provide a status, which can
 *              be one of the following:
 *
 *              RTI_SUCCESS
 *              RTI_ERROR_OSAL_NO_TIMER_AVAIL
 *              RTI_ERROR_ALLOW_PAIRING_TIMEOUT
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 *
 **************************************************************************************************/
RTILIB_API void RTI_AllowBindReq( void )
{
  npiMsgData_t pMsg;

  // prep Pair request
  pMsg.subSys = RPC_SYS_RCAF;
  pMsg.cmdId  = RTIS_CMD_ID_RTI_ALLOW_BIND_REQ;
  pMsg.len    = 0;

  // send Pair request to NP RTIS asynchronously as a confirm is due back
  NPI_SendAsynchData( &pMsg );
}
/**************************************************************************************************
 *
 * @fn          RTI_GetValidationStatusRsp
 *
 * @brief       This function is used to provide the result of a validation request.
 *
 * input parameters
 *
 * @param       dstIndex - pairing index of device that requested validation
 * @param       status   - result of validation
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 *
 **************************************************************************************************/
RTILIB_API void RTI_GetValidationStatusRsp( uint8 status )
{
  npiMsgData_t pMsg;

  // serialize the request
  pMsg.subSys   = RPC_SYS_RCAF;
  pMsg.cmdId    = RTIS_CMD_ID_RTI_GET_VALIDATION_STATUS_RSP;
  pMsg.len      = 1;
  pMsg.pData[0] = status;

  // send serialized request to NP RTIS synchronously
  NPI_SendAsynchData( &pMsg );
}

/**************************************************************************************************
 *
 * @fn          RTI_PollConfigReq
 *
 * @brief       This function is used to initiate the poll negotiation procedure.
 *
 * input parameters
 *
 * @param       dstIndex - pairing index of device to configure polling with
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 *
 **************************************************************************************************/
RTILIB_API void RTI_PollConfigReq( uint8 dstIndex, uint8 len, uint8 *pPollConstraints )
{
  npiMsgData_t pMsg;

  // serialize the request
  pMsg.subSys   = RPC_SYS_RCAF;
  pMsg.cmdId    = RTIS_CMD_ID_RTI_POLL_CONFIG_REQ;
  pMsg.len      = 2 + len;
  pMsg.pData[0] = dstIndex;
  pMsg.pData[1] = len;
  msg_memcpy( &pMsg.pData[2], pPollConstraints, len );

  // send serialized request to NP RTIS synchronously
  NPI_SendAsynchData( &pMsg );
}

/**************************************************************************************************
 *
 * @fn          RTI_IdentificationConfigReq
 *
 * @brief       This function is used to initiate the identification capabilities announcement procedure.
 *
 * input parameters
 *
 * @param       dstIndex - pairing index of device to configure
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 *
 **************************************************************************************************/
RTILIB_API void RTI_IdentificationConfigReq( uint8 dstIndex )
{
  npiMsgData_t pMsg;

  // serialize the request
  pMsg.subSys   = RPC_SYS_RCAF;
  pMsg.cmdId    = RTIS_CMD_ID_RTI_IDENTIFICATION_CONFIG_REQ;
  pMsg.len      = 1;
  pMsg.pData[0] = dstIndex;

  // send serialized request to NP RTIS synchronously
  NPI_SendAsynchData( &pMsg );
}

/**************************************************************************************************
 *
 * @fn          RTI_PollReq
 *
 * @brief       This function is used to initiate the poll procedure.
 *
 * input parameters
 *
 * @param       dstIndex - pairing index of device to poll
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 *
 **************************************************************************************************/
RTILIB_API void RTI_PollReq( uint8 dstIndex, uint8 trigger, uint8 timeout )
{
  npiMsgData_t pMsg;

  // serialize the request
  pMsg.subSys   = RPC_SYS_RCAF;
  pMsg.cmdId    = RTIS_CMD_ID_RTI_POLL_REQ;
  pMsg.len      = 3;
  pMsg.pData[0] = dstIndex;
  pMsg.pData[1] = trigger;
  pMsg.pData[2] = timeout;

  // send serialized request to NP RTIS synchronously
  NPI_SendAsynchData( &pMsg );
}

/**************************************************************************************************
 *
 * @fn          RTI_PollRsp
 *
 * @brief       This function is used to answer a poll indication.
 *
 * input parameters
 *
 * @param       len - length of message to transmit
 * @param       pBuf - pointer to message to transmit
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 *
 **************************************************************************************************/
RTILIB_API void RTI_PollRsp( uint8 profileId, uint8 len, uint8 *pBuf )
{
  npiMsgData_t pMsg;

  // serialize the request
  pMsg.subSys   = RPC_SYS_RCAF;
  pMsg.cmdId    = RTIS_CMD_ID_RTI_POLL_RSP;
  pMsg.len      = 2 + len;
  pMsg.pData[0] = profileId;
  pMsg.pData[1] = len;

  // copy the client's data to be sent
  if (len > 0)
  {
    msg_memcpy( &pMsg.pData[2], pBuf, len );
  }

  // send serialized request to NP RTIS synchronously
  NPI_SendAsynchData( &pMsg );
}

/**************************************************************************************************
 *
 * @fn          RTI_KeyExchangeReq
 *
 * @brief       This function is used to initiate the poll procedure.
 *
 * input parameters
 *
 * @param       dstIndex - pairing index of device to poll
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 *
 **************************************************************************************************/
RTILIB_API void RTI_KeyExchangeReq( uint8 dstIndex, uint16 keyExchangeFlags )
{
  npiMsgData_t pMsg;

  // serialize the request
  pMsg.subSys   = RPC_SYS_RCAF;
  pMsg.cmdId    = RTIS_CMD_ID_RTI_KEY_EXCHANGE_REQ;
  pMsg.len      = 3;
  pMsg.pData[0] = dstIndex;
  pMsg.pData[1] = LO_UINT16( keyExchangeFlags );
  pMsg.pData[2] = HI_UINT16( keyExchangeFlags );

  // send serialized request to NP RTIS synchronously
  NPI_SendAsynchData( &pMsg );
}

/**************************************************************************************************
 *
 * @fn          RTI_GetAttributeCnf
 *
 * @brief       This function is used to provide the result of a validation request.
 *
 * input parameters
 *
 * @param       dstIndex - pairing index of device that requested validation
 * @param       status   - result of validation
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 *
 **************************************************************************************************/
RTILIB_API void RTI_GetAttributeCnf( uint8 dstIndex, rStatus_t status, uint8 len, uint8 *pData )
{
  npiMsgData_t pMsg;

  // serialize the request
  pMsg.subSys   = RPC_SYS_RCAF;
  pMsg.cmdId    = RTIS_CMD_ID_RTI_GET_ATTRIBUTE_CNF;
  pMsg.len      = 3 + len;
  pMsg.pData[0] = dstIndex;
  pMsg.pData[1] = status;
  pMsg.pData[2] = len;

  // copy the client's data to be sent
  msg_memcpy( &pMsg.pData[3], pData, len );

  // send serialized request to NP RTIS synchronously
  NPI_SendAsynchData( &pMsg );
}

/**************************************************************************************************
 *
 * @fn          RTI_SetAttributeCnf
 *
 * @brief       This function is used to provide the result of a validation request.
 *
 * input parameters
 *
 * @param       status   - result of validation
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 *
 **************************************************************************************************/
RTILIB_API void RTI_SetAttributeCnf( uint8 dstIndex, rStatus_t status )
{
  npiMsgData_t pMsg;

  // serialize the request
  pMsg.subSys   = RPC_SYS_RCAF;
  pMsg.cmdId    = RTIS_CMD_ID_RTI_SET_ATTRIBUTE_CNF;
  pMsg.len      = 2;
  pMsg.pData[0] = dstIndex;
  pMsg.pData[1] = status;

  // send serialized request to NP RTIS synchronously
  NPI_SendAsynchData( &pMsg );
}

/**************************************************************************************************
 *
 * @fn          RTI_SendIrdbVendorSupportReq
 *
 * @brief       This function is used to initiate the IRDB Vendor Support Announcement procedure.
 *
 * input parameters
 *
 * @param       dstIndex - pairing index of device to send IRDB Vendor Support attribute to
 * @param       len - length of attribute
 * @param       pIrdbVendorSupport - pointer to buffer containing attribute
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 *
 **************************************************************************************************/
RTILIB_API void RTI_SendIrdbVendorSupportReq( uint8 dstIndex, uint8 len, uint8 *pIrdbVendorSupport )
{
  npiMsgData_t pMsg;

  // serialize the request
  pMsg.subSys   = RPC_SYS_RCAF;
  pMsg.cmdId    = RTIS_CMD_ID_RTI_SEND_IRDB_VENDOR_SUPPORT_REQ;
  pMsg.len      = 2 + len;
  pMsg.pData[0] = dstIndex;
  pMsg.pData[1] = len;

  // copy the client's data to be sent
  msg_memcpy( &pMsg.pData[2], pIrdbVendorSupport, len );

  // send serialized request to NP RTIS synchronously
  NPI_SendAsynchData( &pMsg );
}

/**************************************************************************************************
 *
 * @fn          RTI_SendMappableActionsReq
 *
 * @brief       This function is used to initiate the poll procedure.
 *
 * input parameters
 *
 * @param       dstIndex - pairing index of device to poll
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 *
 **************************************************************************************************/
RTILIB_API void RTI_SendMappableActionsReq( uint8 dstIndex,
	                                        uint8 numMappableActions,
											uint8 *pMappableActionsIndices,
											zrcActionMap_t *pMappableActions )
{
  npiMsgData_t pMsg;
  uint8 mappableActionsLen = numMappableActions * sizeof( zrcActionMap_t );

  // serialize the request
  pMsg.subSys   = RPC_SYS_RCAF;
  pMsg.cmdId    = RTIS_CMD_ID_RTI_SEND_MAPPABLE_ACTIONS_REQ;
  pMsg.len      = 2 + numMappableActions + mappableActionsLen;
  pMsg.pData[0] = dstIndex;
  pMsg.pData[1] = numMappableActions;

  // copy the client's data to be sent
  msg_memcpy( &pMsg.pData[2], pMappableActionsIndices, numMappableActions );
  msg_memcpy( &pMsg.pData[2 + numMappableActions], pMappableActions, mappableActionsLen );

  // send serialized request to NP RTIS synchronously
  NPI_SendAsynchData( &pMsg );
}

/**************************************************************************************************
 *
 * @fn          RTI_GetActionMappingsReq
 *
 * @brief       This function is used to initiate the poll procedure.
 *
 * input parameters
 *
 * @param       dstIndex - pairing index of device to poll
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 *
 **************************************************************************************************/
RTILIB_API void RTI_GetActionMappingsReq( uint8 dstIndex, uint8 actionMappingIndex )
{
  npiMsgData_t pMsg;

  // serialize the request
  pMsg.subSys   = RPC_SYS_RCAF;
  pMsg.cmdId    = RTIS_CMD_ID_RTI_GET_ACTION_MAPPINGS_REQ;
  pMsg.len      = 2;
  pMsg.pData[0] = dstIndex;
  pMsg.pData[1] = actionMappingIndex;

  // send serialized request to NP RTIS synchronously
  NPI_SendAsynchData( &pMsg );
}

/**************************************************************************************************
 *
 * @fn          RTI_HaSupportedAnnounceReq
 *
 * @brief       This function is used to initiate the poll procedure.
 *
 * input parameters
 *
 * @param       dstIndex - pairing index of device to poll
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 *
 **************************************************************************************************/
RTILIB_API void RTI_HaSupportedAnnounceReq( uint8 dstIndex, uint8 numInstances )
{
  npiMsgData_t pMsg;

  // serialize the request
  pMsg.subSys   = RPC_SYS_RCAF;
  pMsg.cmdId    = RTIS_CMD_ID_RTI_HA_SUPPORTED_ANNOUNCE_REQ;
  pMsg.len      = 2;
  pMsg.pData[0] = dstIndex;
  pMsg.pData[1] = numInstances;

  // send serialized request to NP RTIS synchronously
  NPI_SendAsynchData( &pMsg );
}

/**************************************************************************************************
 *
 * @fn          RTI_PullHaAttributesReq
 *
 * @brief       This function is used to initiate the poll negotiation procedure.
 *
 * input parameters
 *
 * @param       dstIndex - pairing index of device to configure polling with
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 *
 **************************************************************************************************/
RTILIB_API void RTI_PullHaAttributesReq( uint8 dstIndex, uint8 haInstanceId, uint8 len, uint8 *pDirtyFlags )
{
  npiMsgData_t pMsg;

  // serialize the request
  pMsg.subSys   = RPC_SYS_RCAF;
  pMsg.cmdId    = RTIS_CMD_ID_RTI_PULL_HA_ATTRIBUTES_REQ;
  pMsg.len      = 3 + len;
  pMsg.pData[0] = dstIndex;
  pMsg.pData[1] = haInstanceId;
  pMsg.pData[2] = len;
  msg_memcpy( &pMsg.pData[3], pDirtyFlags, len );

  // send serialized request to NP RTIS synchronously
  NPI_SendAsynchData( &pMsg );
}

/**************************************************************************************************
 *
 * @fn          RTI_BindAbortReq
 *
 * @brief       This API is used to abort an on-going pairing process.
 *
 *              The client's confirm callback will provide a status, which can
 *              be one of the following:
 *
 *              RTI_SUCCESS
 *              RTI_ERROR_PAIR_COMPLETE
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 *
 **************************************************************************************************/
RTILIB_API void RTI_BindAbortReq( void )
{
  npiMsgData_t pMsg;

  // prep Pair request
  pMsg.subSys = RPC_SYS_RCAF;
  pMsg.cmdId  = RTIS_CMD_ID_RTI_BIND_ABORT_REQ;
  pMsg.len    = 0;

  // send Pair request to NP RTIS asynchronously as a confirm is due back
  NPI_SendAsynchData( &pMsg );
}

/**************************************************************************************************
 *
 * @fn          RTI_AllowBindAbortReq
 *
 * @brief       This API is used to attempt to abort an on-going allow-pairing process.
 *
 *              It is possible that allow pair is at a state of no return (no aborting).
 *              There is no callback associated to this function call.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 *
 **************************************************************************************************/
RTILIB_API void RTI_AllowBindAbortReq( void )
{
  npiMsgData_t pMsg;

  // prep Pair request
  pMsg.subSys = RPC_SYS_RCAF;
  pMsg.cmdId  = RTIS_CMD_ID_RTI_ALLOW_BIND_ABORT_REQ;
  pMsg.len    = 0;

  // send Pair request to NP RTIS asynchronously as a confirm is due back
  NPI_SendAsynchData( &pMsg );
}
#endif //ZRC20_PROFILE

#if (defined FEATURE_MSO) && (FEATURE_MSO == TRUE)
/**************************************************************************************************
 *
 * @fn          RTI_GetValidationStatusRsp
 *
 * @brief       This function is used to return the current status of the MSO validation process.
 *
 * input parameters
 *
 * @param       resetFlag - whether or not to reset the counter after reading the value
 *
 * output parameters
 *
 * None.
 *
 * @return      counter value
 *
 **************************************************************************************************/
RTILIB_API void RTI_GetValidationStatusRsp(uint8 index, uint8 flag)
{
  npiMsgData_t pMsg;

  // serialize the request
  pMsg.subSys   = RPC_SYS_RCAF;
  pMsg.cmdId    = RTIS_CMD_ID_RTI_GET_VALIDATION_STATUS_RSP;
  pMsg.len      = 2;
  pMsg.pData[0] = index;
  pMsg.pData[1] = flag;

  // send serialized request to NP RTIS synchronously
  NPI_SendAsynchData( &pMsg );
}


/**************************************************************************************************
 *
 * @fn          RTI_SetBindingParamsReq
 *
 * @brief       This function is used to respond to RTI_BindingParamsInd.
 *
 * input parameters
 *
 * @param       bindingParams - 4 bytes binding parameters
 *
 * output parameters
 *
 * None.
 *
 * @return      counter value
 *
 **************************************************************************************************/
RTILIB_API void RTI_SetBindingParamsReq(uint32 bindingParams)
{
  npiMsgData_t pMsg;

  // serialize the request
  pMsg.subSys   = RPC_SYS_RCAF;
  pMsg.cmdId    = RTIS_CMD_ID_RTI_BIND_PARAMS_RSP;
  pMsg.len      = 4;
  pMsg.pData[0] = ((uint8 *)&bindingParams)[0];
  pMsg.pData[1] = ((uint8 *)&bindingParams)[1];
  pMsg.pData[2] = ((uint8 *)&bindingParams)[2];
  pMsg.pData[3] = ((uint8 *)&bindingParams)[3];

  // send serialized request to NP RTIS synchronously
  NPI_SendAsynchData( &pMsg );
}
#endif //(defined FEATURE_MSO) && (FEATURE_MSO == TRUE)

/**************************************************************************************************
 *
 * @fn          RTI_PingReq
 *
 * @brief       This function is used to take a UART based RNP out of sleep
 *
 * input parameters
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 *
 **************************************************************************************************/
RTILIB_API void RTI_PingReq( void )
{
  npiMsgData_t pMsg;

  // ping NP; request will be discarded
  pMsg.subSys   = RPC_SYS_RCAF;
  pMsg.cmdId    = RTIS_CMD_ID_TEST_PING_REQ; //RTIS_CMD_ID_TEST_PING_REQ;
  pMsg.len      = 0;

  // send command to slave to wake it up
  NPI_SendAsynchData( &pMsg );
}

/**************************************************************************************************
 **************************************************************************************************/
