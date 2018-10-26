 /**************************************************************************************************
  Filename:       npi_rcn.c
  Revised:        $Date: 2012-03-21 17:37:33 -0700 (Wed, 21 Mar 2012) $
  Revision:       $Revision: 246 $

  Description:    This file contains Linux platform specific RCN API
                  Surrogate implementation

  Copyright (C) {2018} Texas Instruments Incorporated - http://www.ti.com/


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
#include "rcn_lnx.h"
#include "rcn_marshal.h"

#include "ctrlm_hal.h"

#include "lprfLogging.h"

#ifdef __BIG_DEBUG__
#define debug_printf(fmt, ...) printf( fmt, ##__VA_ARGS__)
#else
#define debug_printf(fmt, ...)
#endif

#define msg_memcpy(src, dst, len)	memcpy(src, dst, len)


#define NAME_ELEMENT(element) [element&0x1F] = #element

/**************************************************************************************************
 *                                        Externals
 **************************************************************************************************/

/**************************************************************************************************
 *                                           Constant
 **************************************************************************************************/

/**************************************************************************************************
 *                                        Type definitions
 **************************************************************************************************/

/**************************************************************************************************
 *                                        Global Variables
 **************************************************************************************************/

/**************************************************************************************************
 *                                        Local Variables
 **************************************************************************************************/

/**************************************************************************************************
 *                                     Local Function Prototypes
 **************************************************************************************************/

/**************************************************************************************************
 * @fn          RCN_NldeDataAlloc
 *
 * @brief       This function is a special function to allocate a memory block
 *              where NSDU should be copied into before passing NLDE-DATA.Request primitive
 *              to network layer.
 *
 * input parameters
 *
 * @param       pPrimitive - a pointer to a C structure describing NLDE-Data.Request.
 *                          nsdu field of the C structure will be filled in before this function
 *                          returns if the function successfully allocated memory block.
 *
 * output parameters
 *
 * None.
 *
 * @return      either RCN_SUCCESS or RCN_ERROR_OUT_OF_MEMORY
 **************************************************************************************************
 */
RCNLIB_API uint8 RCN_NldeDataAlloc( rcnNldeDataReq_t *pPrimitive )
{
  rcnNldeDataReqStream_t *pSerialized = (rcnNldeDataReqStream_t *) malloc(sizeof(rcnNldeDataReqStream_t) + pPrimitive->nsduLength);

  if (!pSerialized)
  {
    return RCN_ERROR_OUT_OF_MEMORY;
  }

  memcpy(pSerialized, pPrimitive, sizeof(rcnNldeDataReqStream_t));
  pPrimitive->internal = (uint8 *) pSerialized;
  pPrimitive->nsdu = (uint8 *) (pSerialized + 1);

  return RCN_SUCCESS;
}

/**************************************************************************************************
 * @fn          RCN_NldeDataReq
 *
 * @brief       This function requests transfer of a data NSDU.
 *
 * input parameters
 *
 * @param       pPrimitive - a pointer to a C structure containing NLDE-Data.Request.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
RCNLIB_API void RCN_NldeDataReq( rcnNldeDataReq_t *pPrimitive )
{
  npiMsgData_t msgData;

  msgData.len = sizeof(rcnNldeDataReqStream_t) + pPrimitive->nsduLength;
  msgData.subSys = RPC_SYS_RCN;
  msgData.cmdId = RCN_NLDE_DATA_REQ;
  memcpy(msgData.pData, pPrimitive->internal, sizeof(rcnNldeDataReqStream_t) + pPrimitive->nsduLength);

  free(pPrimitive->internal);

  NPI_SendAsynchData(&msgData);
}

/**************************************************************************************************
 * @fn          RCN_NlmeDiscoveryReq
 *
 * @brief       This function requests discovery of other devices of interest operating in the
 *              personal operating space of the device.
 *
 * input parameters
 *
 * @param       pPrimitive - a pointer to a C structure containing NLME-Discovery.Request.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
RCNLIB_API void RCN_NlmeDiscoveryReq( rcnNlmeDiscoveryReq_t *pPrimitive )
{
  npiMsgData_t msgData;

  msgData.len = sizeof(*pPrimitive);
  msgData.subSys = RPC_SYS_RCN;
  msgData.cmdId = RCN_NLME_DISCOVERY_REQ;
  memcpy(msgData.pData, pPrimitive, sizeof(*pPrimitive));

  NPI_SendAsynchData(&msgData);
}

/**************************************************************************************************
 * @fn          RCN_NlmeDiscoveryAbortReq
 *
 * @brief       This function requests aborting on-going discovery.
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
 */
RCNLIB_API void RCN_NlmeDiscoveryAbortReq( void )
{
  npiMsgData_t msgData;

  msgData.len = 0;
  msgData.subSys = RPC_SYS_RCN;
  msgData.cmdId = RCN_NLME_DISCOVERY_ABORT_REQ;

  NPI_SendAsynchData(&msgData);
}

/**************************************************************************************************
 * @fn          RCN_NlmeDiscoveryRsp
 *
 * @brief       This function requests the local NLME to respond to a discovery request command.
 *
 * input parameters
 *
 * @param       pPrimitive - a pointer to a C structure containing NLME-Discovery.Response.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
RCNLIB_API void RCN_NlmeDiscoveryRsp( rcnNlmeDiscoveryRsp_t *pPrimitive )
{
  npiMsgData_t msgData;

  msgData.len = sizeof(*pPrimitive);
  msgData.subSys = RPC_SYS_RCN;
  msgData.cmdId = RCN_NLME_DISCOVERY_RSP;
  memcpy(msgData.pData, pPrimitive, sizeof(*pPrimitive));

  NPI_SendAsynchData(&msgData);
}

/**************************************************************************************************
 * @fn          RCN_NlmeGetReq
 *
 * @brief       This function retrieves an Network Information Base (NIB) attribute value.
 *
 * input parameters
 *
 * @param       attribute - NIB attribute enumeration
 * @param       attributeIndex - NIB attribute index in case NIB attribute is a table or an array
 *                               type
 *
 * output parameters
 *
 * @param       pValue     - pointer to a data block where to copy the attribute value into.
 *
 * @return      RCN_SUCCESS or RCN_ERROR_UNSUPPORTED_ATTRIBUTE
 **************************************************************************************************
 */
RCNLIB_API uint8 RCN_NlmeGetReq( uint8 attribute, uint8 attributeIndex, uint8 *pValue )
{
  npiMsgData_t msgData;

  // build message
  msgData.len = sizeof(rcnNlmeGetReq_t);
  msgData.subSys = RPC_SYS_RCN;
  msgData.cmdId = RCN_NLME_GET_REQ;
  {
    rcnNlmeGetReq_t *pGetReq = (rcnNlmeGetReq_t *) msgData.pData;

    pGetReq->attribute = attribute;
    pGetReq->attributeIndex = attributeIndex;
  }

  // Send the synchronous message and receive a response
  NPI_SendSynchData(&msgData);

  // check received message
  if ((msgData.subSys & RPC_SUBSYSTEM_MASK) != RPC_SYS_RCN || //_CLIENT ||
    msgData.cmdId != RCN_NLME_GET_CNF)
  {
    LOG_WARN("Integrity failure msgData.subSys=0x%.2X msgData.cmdId=0x%.2X\n", msgData.subSys, msgData.cmdId);
    // error case
    return RCN_ERROR_COMMUNICATION;
  }

  {
    // parse received message
    rcnNlmeGetCnf_t *pGetCnf = (rcnNlmeGetCnf_t *) msgData.pData;

    if (pGetCnf->attribute != attribute)
    {
      LOG_WARN("Integrity failure pGetCnf->attribute=0x%.2X\n", pGetCnf->attribute);
      return RCN_ERROR_COMMUNICATION;
    }
    if (pGetCnf->status == RCN_SUCCESS)
    {
      memcpy(pValue, pGetCnf + 1, pGetCnf->length);
    }
    return pGetCnf->status;
  }
}

/**************************************************************************************************
 * @fn          RCN_NlmeGetSizeReq
 *
 * @brief       This function retrieves an Network Information Base (NIB) attribute value size.
 *              In case the attribute is a table or an array, the returned size is that of an
 *              element.
 *
 * input parameters
 *
 * @param       attribute - NIB attribute enumeration
 *
 * output parameters
 *
 * NOne.
 *
 * @return      Attribute value size or 0 if attribute is not found
 **************************************************************************************************
 */
RCNLIB_API uint8 RCN_NlmeGetSizeReq( uint8 attribute )
{
  switch (attribute)
  {
  case RCN_NIB_NWK_ACTIVE_PERIOD:
  case RCN_NIB_NWK_DISCOVERY_REPETITION_INTERVAL:
  case RCN_NIB_NWK_DUTY_CYCLE:
  case RCN_NIB_NWK_RESPONSE_WAIT_TIME:
  case RCN_NIB_NWK_VENDOR_IDENTIFIER:
  case RCN_NIB_PAN_ID:
  case RCN_NIB_SHORT_ADDRESS:
    return 2;
  case RCN_NIB_NWK_FRAME_COUNTER: 
  case RCN_NIB_TRANSMIT_LATENCY:
    return 4;
  case RCN_NIB_IEEE_ADDRESS:
    return 8;
  case RCN_NIB_NWK_USER_STRING:
    return RCN_USER_STRING_LENGTH;
  case RCN_NIB_NWK_VENDOR_STRING:
    return RCN_VENDOR_STRING_LENGTH;
  case RCN_NIB_NWK_PAIRING_TABLE:
    return sizeof(rcnNwkPairingEntry_t);
  case RCN_NIB_ENERGY_SAMPLE:
    return 20;
  }
  return 1;
}

/**************************************************************************************************
 * @fn          RCN_NlmePairReq
 *
 * @brief       This function requests the local NLME to pair with another device.
 *
 * input parameters
 *
 * @param       pPrimitive - a pointer to a C structure containing NLME-PAIR.Request.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
RCNLIB_API void RCN_NlmePairReq( rcnNlmePairReq_t *pPrimitive )
{
  npiMsgData_t msgData;

  msgData.len = sizeof(*pPrimitive);
  msgData.subSys = RPC_SYS_RCN;
  msgData.cmdId = RCN_NLME_PAIR_REQ;
  memcpy(msgData.pData, pPrimitive, sizeof(*pPrimitive));

  NPI_SendAsynchData(&msgData);
}

/**************************************************************************************************
 * @fn          RCN_NlmePairRsp
 *
 * @brief       This function requests the local NLME to respond to a pairing request command.
 *
 * input parameters
 *
 * @param       pPrimitive - a pointer to a C structure containing NLME-PAIR.Response.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
RCNLIB_API void RCN_NlmePairRsp( rcnNlmePairRsp_t *pPrimitive )
{
  npiMsgData_t msgData;

  msgData.len = sizeof(*pPrimitive);
  msgData.subSys = RPC_SYS_RCN;
  msgData.cmdId = RCN_NLME_PAIR_RSP;
  memcpy(msgData.pData, pPrimitive, sizeof(*pPrimitive));

  NPI_SendAsynchData(&msgData);
}

/**************************************************************************************************
 * @fn          RCN_NlmeResetReq
 *
 * @brief       This function resets network layer.
 *
 * input parameters
 *
 * @param       setDefaultNib - If TRUE, the NWK layer is reset and all NIB attributes are set
 *                              to their default values.
 *                              If FALSE, the NWK layer is reset but all NIB attributes retain
 *                              their values prior to the generation of the NLME-RESET.request
 *                              primitive.
 *
 * output parameters
 *
 * @return      None.
 **************************************************************************************************
 */
RCNLIB_API void RCN_NlmeResetReq( uint8 setDefaultNib )
{
  npiMsgData_t msgData;

  msgData.len = sizeof(rcnNlmeResetReq_t);
  msgData.subSys = RPC_SYS_RCN;
  msgData.cmdId = RCN_NLME_RESET_REQ;
  {
    rcnNlmeResetReq_t *pResetReq = (rcnNlmeResetReq_t *)msgData.pData;

    pResetReq->setDefaultNib = setDefaultNib;
  }

  NPI_SendSynchData(&msgData);

  // received data must be reset confirm.
  // There is no way to return failure and hence no verification is performed
  // here.
}

/**************************************************************************************************
 * @fn          RCN_NlmeRxEnableReq
 *
 * @brief       This function requests that the receiver is either enabled (for a finite period
 *              or until further notice) or disabled..
 *
 * input parameters
 *
 * @param       rxOnDurationInMs - Duration in ms for which the receiver is to be enabled.
 *                                 If this parameter is equal to 0x0000,
 *                                 the receiver is to be disabled.
 *                                 If this parameter is equal to 0xffff,
 *                                 the receiver is to be enabled until further notice.
 *
 * output parameters
 *
 * @return      RCN_SUCCESS, MAC_PAST_TIME, MAC_ON_TIME_TOO_LONG or MAC_INVALID_PARAMETER
 **************************************************************************************************
 */
RCNLIB_API uint8 RCN_NlmeRxEnableReq( uint16 rxOnDurationInMs )
{
  npiMsgData_t msgData;

  // build primitive
  msgData.len = sizeof(rcnNlmeRxEnableReq_t);
  msgData.subSys = RPC_SYS_RCN;
  msgData.cmdId = RCN_NLME_RX_ENABLE_REQ;
  {
    rcnNlmeRxEnableReq_t *pRxEnableReq = (rcnNlmeRxEnableReq_t *)msgData.pData;

    pRxEnableReq->rxOnDurationInMs = rxOnDurationInMs;
  }

  // send the request and get confirmation
  NPI_SendSynchData(&msgData);

  // check received message
  if ((msgData.subSys & RPC_SUBSYSTEM_MASK) != RPC_SYS_RCN || //_CLIENT ||
    msgData.cmdId != RCN_NLME_RX_ENABLE_CNF)
  {
    // error case
    return RCN_ERROR_COMMUNICATION;
  }

  {
    // parse received message
    rcnNlmeRxEnableCnf_t *pRxEnableCnf = (rcnNlmeRxEnableCnf_t *) msgData.pData;

    return pRxEnableCnf->status;
  }
}

/**************************************************************************************************
 * @fn          RCN_NlmeSetReq
 *
 * @brief       This function sets an NIB attribute value.
 *
 * input parameters
 *
 * @param       attribute - The identifier of the NIB attribute to write.
 * @param       attributeIndex - index within a table or an array if the NIB attibute is a table
 *                               or an array
 *
 * @param       pValue - pointer to attribute value
 *
 * output parameters
 *
 * @return      RCN_SUCCESS, RCN_ERROR_UNSUPPORTED_ATTRIBUTE or RCN_ERROR_INVALID_INDEX
 **************************************************************************************************
 */
RCNLIB_API uint8 RCN_NlmeSetReq( uint8 attribute, uint8 attributeIndex, uint8 *pValue )
{
  npiMsgData_t msgData;
  uint8 valueLength;

  // build primitive
  valueLength = RCN_NlmeGetSizeReq(attribute);
  msgData.len = sizeof(rcnNlmeSetReq_t) + valueLength;
  msgData.subSys = RPC_SYS_RCN;
  msgData.cmdId = RCN_NLME_SET_REQ;
  {
    rcnNlmeSetReq_t *pSetReq = (rcnNlmeSetReq_t *)msgData.pData;

    pSetReq->nibAttribute = attribute;
    pSetReq->nibAttributeIndex = attributeIndex;
    pSetReq->length = valueLength;

    memcpy(pSetReq + 1, pValue, valueLength);
  }

  // send the request and get confirmation
  NPI_SendSynchData(&msgData);

  // check received message
  if ((msgData.subSys & RPC_SUBSYSTEM_MASK) != RPC_SYS_RCN || //_CLIENT ||
    msgData.cmdId != RCN_NLME_SET_CNF)
  {
    // error case
    return RCN_ERROR_COMMUNICATION;
  }

  {
    // parse received message
    rcnNlmeSetCnf_t *pSetCnf = (rcnNlmeSetCnf_t *) msgData.pData;

    return pSetCnf->status;
  }
}

/**************************************************************************************************
 * @fn          RCN_NlmeStartReq
 *
 * @brief       This function requests NLME to start network.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * @return      None.
 **************************************************************************************************
 */
RCNLIB_API void RCN_NlmeStartReq( void )
{
  npiMsgData_t msgData;

  msgData.len = 0;
  msgData.subSys = RPC_SYS_RCN;
  msgData.cmdId = RCN_NLME_START_REQ;

  NPI_SendAsynchData(&msgData);
}

/**************************************************************************************************
 * @fn          RCN_NlmeUnpairReq
 *
 * @brief       This function removes an entry from the pairing table.
 *              This function will trigger a callback of RCN_NLME_UNPAIR_CNF with
 *              the following status codes:
 *              RCN_SUCCESS, RCN_ERROR_NO_PAIRING, MAC_TRANSACTION_OVERFLOW,
 *              MAC_TRANSACTION_EXPIRED, MAC_CHANNEL_ACCESS_FAILURE, MAC_INVALID_ADDRESS,
 *              MAC_NO_ACK, MAC_COUNTER_ERROR, MAC_FRAME_TOO_LONG, MAC_UNAVAILABLE_KEY,
 *              MAC_UNSUPPORTED_SECURITY or MAC_INVALID_PARAMETER
 *
 * input parameters
 *
 * @param       pairingRef - The reference into the local pairing table of the entry
 *                           that is to be removed.
 *
 * output parameters
 *
 * None
 *
 * @return      None.
 **************************************************************************************************
 */
RCNLIB_API void RCN_NlmeUnpairReq( uint8 pairingRef )
{
  npiMsgData_t msgData;

  msgData.len = sizeof(rcnNlmeUnpairReq_t);
  msgData.subSys = RPC_SYS_RCN;
  msgData.cmdId = RCN_NLME_UNPAIR_REQ;
  {
    rcnNlmeUnpairReq_t *pUnpairReq = (rcnNlmeUnpairReq_t *) msgData.pData;

    pUnpairReq->pairingRef = pairingRef;
  }

  NPI_SendAsynchData(&msgData);
}


/**************************************************************************************************
 * @fn          RCN_NlmeUnpairRsp
 *
 * @brief       This function removes an entry from the pairing table in response to
 *              NLME-UNPAIR.indication.
 *
 * input parameters
 *
 * @param       pairingRef - The reference into the local pairing table of the entry
 *                           that is to be removed.
 *
 * output parameters
 *
 * @return      None
 **************************************************************************************************
 */
RCNLIB_API void RCN_NlmeUnpairRsp( uint8 pairingRef )
{
  npiMsgData_t msgData;

  msgData.len = sizeof(rcnNlmeUnpairRsp_t);
  msgData.subSys = RPC_SYS_RCN;
  msgData.cmdId = RCN_NLME_UNPAIR_RSP;
  {
    rcnNlmeUnpairRsp_t *pUnpairRsp = (rcnNlmeUnpairRsp_t *) msgData.pData;

    pUnpairRsp->pairingRef = pairingRef;
  }

  NPI_SendAsynchData(&msgData);
}

/**************************************************************************************************
 * @fn          RCN_NlmeDiscoveryAcceptReq
 *
 * @brief       This function configures the local NLME to respond to a discovery request command.
 *
 * input parameters
 *
 * @param       pPrimitive - a pointer to a C structure containing NLME-DISCOVERY-ACCEPT.request.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
RCNLIB_API void RCN_NlmeAutoDiscoveryReq( rcnNlmeAutoDiscoveryReq_t *pPrimitive )
{
  npiMsgData_t msgData;

  msgData.len = sizeof(*pPrimitive);
  msgData.subSys = RPC_SYS_RCN;
  msgData.cmdId = RCN_NLME_AUTO_DISCOVERY_REQ;
  memcpy(msgData.pData, pPrimitive, sizeof(*pPrimitive));

  NPI_SendAsynchData(&msgData);
}

/**************************************************************************************************
 * @fn          RCN_NlmeAutoDiscoveryAbortReq
 *
 * @brief       This function requests aborting on-going auto-discovery.
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
 */
RCNLIB_API void RCN_NlmeAutoDiscoveryAbortReq( void )
{
  npiMsgData_t msgData;

  msgData.len = 0;
  msgData.subSys = RPC_SYS_RCN;
  msgData.cmdId = RCN_NLME_AUTO_DISCOVERY_ABORT_REQ;

  NPI_SendAsynchData(&msgData);
}

/**************************************************************************************************
 * @fn          RCNS_AsynchMsgCback
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
 *              It is assumed that pMsg->subSys is already checked out to
 *              be RPC_SYS_RCN_CLIENT
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void RCN_AsynchMsgCback( npiMsgData_t *pMsg )
{
  if ( pMsg->cmdId == RCN_NLDE_DATA_IND )
  {
//    struct
//    {
//      uint8 eventId;
//      rcnNldeDataInd_t dataInd;
//    } cbackData;
//
//    memcpy(&cbackData, &pMsg->cmdId, sizeof(cbackData.eventId) + sizeof(rcnNldeDataIndStream_t));
//    cbackData.dataInd.nsdu = &pMsg->pData[sizeof(rcnNldeDataIndStream_t)];
//    RCN_CbackEvent( (rcnCbackEvent_t *) &cbackData );

    rcnNldeDataInd_t dataInd;
    memcpy(&dataInd, pMsg->pData, sizeof(rcnNldeDataIndStream_t));
    dataInd.nsdu = &pMsg->pData[sizeof(rcnNldeDataIndStream_t)];
    RCN_NldeDataInd(&dataInd);
  }
  else
  {
    switch (pMsg->cmdId)
    {
    case RCN_NLME_START_CNF:
      RCN_NlmeStartCnf((rcnNlmeStartCnf_t *)pMsg->pData);
      break;
    case RCN_NLDE_DATA_CNF:
      RCN_NldeDataCnf((rcnNldeDataCnf_t *)pMsg->pData);
      break;
    case RCN_NLME_COMM_STATUS_IND:
      RCN_NlmeCommStatusInd((rcnNlmeCommStatusInd_t *)pMsg->pData);
      break;
    case RCN_NLME_DISCOVERY_IND:
      RCN_NlmeDiscoveryInd((rcnNlmeDiscoveryInd_t *)pMsg->pData);
      break;
    case RCN_NLME_DISCOVERED_EVENT:
      RCN_NlmeDiscoveredEvent((rcnNlmeDiscoveredEvent_t *)pMsg->pData);
      break;
    case RCN_NLME_DISCOVERY_CNF:
      RCN_NlmeDiscoveryCnf((rcnNlmeDiscoveryCnf_t *)pMsg->pData);
      break;
    case RCN_NLME_AUTO_DISCOVERY_CNF:
      RCN_NlmeAutoDiscoveryCnf((rcnNlmeAutoDiscoveryCnf_t *)pMsg->pData);
      break;
    case RCN_NLME_PAIR_IND:
      RCN_NlmePairInd((rcnNlmePairInd_t *)pMsg->pData);
      break;
    case RCN_NLME_PAIR_CNF:
      RCN_NlmePairCnf((rcnNlmePairCnf_t *)pMsg->pData);
      break;
    case RCN_NLME_UNPAIR_IND:
      RCN_NlmeUnpairInd((rcnNlmeUnpairInd_t *)pMsg->pData);
      break;
    case RCN_NLME_UNPAIR_CNF:
      RCN_NlmeUnpairCnf((rcnNlmeUnpairCnf_t *)pMsg->pData);
      break;
    default:

      break;
    }
  }
}


/**************************************************************************************************
 **************************************************************************************************/

