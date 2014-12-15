/**************************************************************************************************
  Filename:       gdp_common.h
  Revised:        $Date: 2014-09-24 12:24:14 -0700 (Wed, 24 Sep 2014) $
  Revision:       $Revision: 40267 $

  Description:    Module containing APIs for functions that are used by both
                  GDP 2.0 Originator and Recipient devices.

  Copyright 2013-2014 Texas Instruments Incorporated. All rights reserved.

  Copyright 2014 Texas Instruments Incorporated. All rights reserved.

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

#ifndef GDP_COMMON_H
#define GDP_COMMON_H

#ifdef __cplusplus
extern "C"
{
#endif

/**************************************************************************************************
 * INCLUDES
 **************************************************************************************************/

#include "hal_types.h"
#include "rcn_nwk.h"
#include "rti.h"
#include "saddr.h"

/**************************************************************************************************
 * MACROS
 **************************************************************************************************/

/**************************************************************************************************
 * CONSTANTS
 **************************************************************************************************/

// GDP profile user string defines
#define GDP_DISC_REQ_USER_STRING_NULL_CHAR_INDEX 8
#define GDP_DISC_REQ_USER_STRING_VENDOR_ID_FILTER_LSB_INDEX 9
#define GDP_DISC_REQ_USER_STRING_VENDOR_ID_FILTER_MSB_INDEX 10
#define GDP_DISC_REQ_USER_STRING_MIN_MAX_CLASS_FILTER_INDEX 11
#define GDP_DISC_REQ_USER_STRING_MIN_LQI_FILTER_INDEX 12

#define GDP_DISC_RSP_USER_STRING_NULL_CHAR_INDEX 8
#define GDP_DISC_RSP_USER_STRING_TERTIARY_CLASS_DESCRIPTOR_INDEX 11
#define GDP_DISC_RSP_USER_STRING_SECONDARY_CLASS_DESCRIPTOR_INDEX 12
#define GDP_DISC_RSP_USER_STRING_PRIMARY_CLASS_DESCRIPTOR_INDEX 13
#define GDP_DISC_RSP_USER_STRING_DISCOVERY_LQI_THRESHOLD_INDEX 14

#define GDP_PAIR_REQ_USER_STRING_NULL_CHAR_INDEX 8
#define GDP_PAIR_REQ_USER_STRING_ADVANCED_BINDING_SUPPORT_INDEX 9

#define GDP_COMMON_INVALID_INDEX 0xFF

/**************************************************************************************************
 * TYPEDEFS
 **************************************************************************************************/
typedef enum
{
  GDP_TYPE_10,
  GDP_TYPE_20
} gdpGdpType_t;

/**************************************************************************************************
 * GLOBALS
 **************************************************************************************************/

/**************************************************************************************************
 * MACROS
 **************************************************************************************************/

// Tx options for controller to target communication
#define TX_OPTIONS_GDP_C2T ( RTI_TX_OPTION_ACKNOWLEDGED | RTI_TX_OPTION_SECURITY )

// Tx options for target to controller communication
#define TX_OPTIONS_GDP_T2C ( RTI_TX_OPTION_ACKNOWLEDGED | RTI_TX_OPTION_SECURITY | RTI_TX_OPTION_SINGLE_CHANNEL )

// Same as above with security disabled
#define TX_OPTIONS_GDP_NO_SECURITY_C2T ( RTI_TX_OPTION_ACKNOWLEDGED )
#define TX_OPTIONS_GDP_NO_SECURITY_T2C ( RTI_TX_OPTION_ACKNOWLEDGED | RTI_TX_OPTION_SINGLE_CHANNEL )

/*********************************************************************
 * FUNCTIONS
 */
void gdpCommon_UpdateProfileList( gdpGdpType_t gdpTypeToKeep,
                                  rcnAppInfo_t *pAppCap );

uint8 gdpCommon_BuildGetOrPullAttributesCmd( uint8 cmd,
                                             gdpAttrId_t *pAttrList,
                                             uint8 numAttr,
                                             uint8 *pBuf );

uint8 gdpCommon_BuildConfigurationCompleteCmd( uint8 status,
                                               uint8 *pBuf );

uint8 gdpCommon_BuildPushAttributesCmd( gdpAttrId_t *pAttrList,
                                        uint8 numAttr,
                                        gdpAttrTable_t *pAttrTable,
                                        uint8 *pBuf );

uint8 gdpCommon_BuildGenericResponseCmd( uint8 status,
                                         uint8 *pBuf );

uint8 gdpCommon_BuildGetOrPullAttributesRspCmd( uint8 cmd,
                                                gdpAttrId_t *pAttrList,
                                                uint8 *pAttrStatusList,
                                                uint8 numAttr,
                                                gdpAttrTable_t *pAttrTable,
                                                uint8 *pBuf );

uint8 gdpCommon_CalcNumAttrInAttrListCmd( rcnNldeDataInd_t *pDataInd );

uint8 gdpCommon_CalcNumAttrInAttrCmd( gdpAttrFrameType_t attrFrameType,
                                      rcnNldeDataInd_t *pDataInd );

uint8 *gdpCommon_GetNextAttrIdInAttrListCmd( uint8 *pCmd,
                                             gdpAttrId_t *pAttrId );

uint8 *gdpCommon_GetNextAttrInAttrCmd( gdpAttrFrameType_t attrFrameType,
                                       uint8 *pCmd,
                                       gdpParsedAttr_t *pAttr );

bool gdpCommon_IsAttributeArrayed( uint8 attrId );

uint8 gdpCommon_CountBitsSet( uint8 *pBuf, uint8 len );

bool gdpCommon_GetNextBitSet( uint8 *pBuf,
                              uint8 len,
                              uint8 firstBit,
                              uint8 *pResult );

void gdpCommon_AddNodeToAttrList( gdpArrayedAttrList_t *pNode,
                                  gdpArrayedAttrList_t **ppList );

void gdpCommon_DeleteAttrList( gdpArrayedAttrList_t **ppList );

void gdpCommon_CreateBackupPairingEntry( sAddrExt_t ieeeAddress );

bool gdpCommon_RestoreBackupPairingEntry( uint8 dstIndex );

void gdpCommon_CleanupBackupPairingInfo( void );

void gdpCommon_CreateAttrStatusBufs( uint8 numAttr,
                                     uint8 **ppAttrStatusBuf,
                                     gdpAttrId_t **ppAttrList );

void gdpCommon_DeleteAttrStatusBufs( uint8 *pAttrStatusBuf );

void gdpCommon_BuildAndSendGenericRsp( uint8 pairIndex,
                                       uint8 profileId,
                                       uint8 status,
                                       rtiCmdCnfCbackFn_t cback );

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* MSO_COMMON_H */
