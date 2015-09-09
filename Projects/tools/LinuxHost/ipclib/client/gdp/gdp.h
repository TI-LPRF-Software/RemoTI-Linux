/**************************************************************************************************
  Filename:       gdp.h

  Description:

  This file contains the common declarations for the Generic Device Profile (GDP).

  Copyright (C) 2015 Texas Instruments Incorporated. All rights reserved.

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
#ifndef GDP_H
#define GDP_H

#ifdef __cplusplus
extern "C" {
#endif

// End packing rule
#if defined(_MSC_VER) || defined(unix) || (defined(__ICC430__) && (__ICC430__==1))
#pragma pack(1)
#endif

/*********************************************************************
 * INCLUDES
 */
#include "hal_types.h"
#include "gdp_api.h"
#include "gdp_profile.h"
#include "rti_lnx.h"

/*********************************************************************
 * CONSTANTS
 */
// GDP Profile Item Ids
//
// It may be required to support local Item Id's which differ from GDP-specified Attribute Id's.

// GDP item IDs to be used for RTI_ReadItemEx() and RTI_WriteItemEx()
// The first set of IDs correspond to GDP profile attributes
#define GDP_ITEM_GDP_VERSION                                     aplGDPVersion                            // 0x80
#define GDP_ITEM_GDP_CAPABILITIES                                aplGDPCapabilities                       // 0x81
#define GDP_ITEM_KEY_EX_TRANSFER_COUNT                           aplKeyExchangeTransferCount              // 0x82
#define GDP_ITEM_POWER_STATUS                                    aplPowerStatus                           // 0x83
#define GDP_ITEM_MAX_BINDING_CANDIDATES                          aplMaxBindingCandidates                  // 0x86
#define GDP_ITEM_AUTO_CHECK_VALIDATION_PERIOD                    aplAutoCheckValidationPeriod             // 0x87
#define GDP_ITEM_LINK_LOST_WAIT_TIME                             aplLinkLostWaitTime                      // 0x8a
#define GDP_ITEM_IDENTIFICATION_CAPABILITIES                     aplIdentificationCapabilities            // 0x8b

// The following item IDs are outside the GDP attribute ID range and should be safe
#define GDP_ITEM_VENDOR_ID_FILTER                                0xE0
#define GDP_ITEM_MIN_CLASS_FILTER                                0xE1
#define GDP_ITEM_MAX_CLASS_FILTER                                0xE2
#define GDP_ITEM_MIN_LQI_FILTER                                  0xE3
#define GDP_ITEM_PROXY_PAIR_INFO                                 0xE4
#define GDP_ITEM_TERTIARY_CLASS_DESCRIPTOR                       0xE5
#define GDP_ITEM_SECONDARY_CLASS_DESCRIPTOR                      0xE6
#define GDP_ITEM_PRIMARY_CLASS_DESCRIPTOR                        0xE7
#define GDP_ITEM_BINDING_CAP                                     0xE8
#define GDP_ITEM_BINDING_RECIPIENT_NORMAL_VALIDATION_WAIT_TIME   0xE9
#define GDP_ITEM_BINDING_RECIPIENT_EXTENDED_VALIDATION_WAIT_TIME 0xEA
#define GDP_ITEM_MINIMUM_PEER_GDP_CAPABILITIES                   0xEB
#define GDP_ITEM_DEFAULT_SECRET                                  0xEC

// GDP NV items
#if !defined GDP_NVID_BEG
#define GDP_NVID_BEG 0xA0  // Arbitrary, safe NV Id that does not conflict with RTI usages.
#endif

#define GDP_NVID_KEY_EXCHANGE_TRANSFER_COUNT      (GDP_NVID_BEG + 0)
#define GDP_NVID_BINDING_STATUS                   (GDP_NVID_BEG + 1)
#define GDP_NVID_GDP_CAPABILITIES                 (GDP_NVID_BEG + 2) // gdp capabilities attribute for peer nodes
#define GDP_NVID_NEXT_AVAILABLE                   (GDP_NVID_GDP_CAPABILITIES + RCN_CAP_PAIR_TABLE_SIZE) // stress that there is 1 GDP capabilities NVID per pairing table entry

// Other GDP constants
#define GDP_NUM_POLLING_METHODS_SUPPORTED 1

// Values for the Binding Capabilities configuration parameter
#define GDP_BINDING_CAP_TYPE_PUSH_BUTTON                 0x00
#define GDP_BINDING_CAP_TYPE_PUSH_BUTTON_AND_VALIDATION  0x01


// Number of bytes needed to store "binding completed" bit for each entry of
// the pairing table.
#define GDP_BIND_DISC_CNT                ((RCN_CAP_PAIR_TABLE_SIZE + 7) / 8)

/*********************************************************************
 * MACROS
 */
// Build the TX Options Bit Mask for RTI_SendDataReq()
#define GDP_TX_OPTIONS_CONTROL_PIPE \
  (RTI_TX_OPTION_ACKNOWLEDGED | RTI_TX_OPTION_SECURITY)

// Macros for interpreting the aplPowerStatus attribute
#define POWER_STATUS_SET_POWER_METER(x,v)       st(x = ((x) & 0xF0) | ((v) & 0x0F);)
#define POWER_STATUS_GET_POWER_METER(x,v)       st(x = ((v) & 0x0F);)
#define POWER_STATUS_SET_CHARGING_BIT(x)        st(x |= 0x10;)
#define POWER_STATUS_GET_CHARGING_BIT(x)        ((x) >> 5)
#define POWER_STATUS_CLR_CHARGING_BIT(x)        st(x &= ~0x10;)
#define POWER_STATUS_SET_IMPENDING_DOOM_BIT(x)  st(x |= 0x80;)
#define POWER_STATUS_GET_IMPENDING_DOOM_BIT(x)  ((x) >> 8)
#define POWER_STATUS_CLR_IMPENDING_DOOM_BIT(x)  st(x &= ~0x80;)

/*********************************************************************
 * TYPEDEFS
 */
// Proxy Pairing Info
PACK_1 typedef struct ATTR_PACKED
{
  sAddrExt_t   addr;
  uint16       panId;
} gdpProxyParams_t;

// Table of Configuration Parameters for this node
PACK_1 typedef struct ATTR_PACKED
{
  rtiAppCap_t  configAppCap;
  gdpProxyParams_t   proxyPairInfo;
  uint32       gdpCapabilities;
  uint32       peerGdpCapabilities[RCN_CAP_PAIR_TABLE_SIZE];
  uint32       minimumPeerGdpCapabilities;
  uint16       gdpVersion;
  uint16       autoCheckValidationPeriod;
  uint16       linkLostWaitTime;
  bool         proxyPairInfoValid;
  uint8        pairRefIndex; // current pairing index for multiple profile configuration
  uint8        nextProfileToConfig;
  uint8        maxProfileId;
  uint8        profileDiscsToConfigure;
  uint8        powerStatus;
#ifdef FEATURE_ORIGINATOR
  uint16       vendorIdFilter;
  uint8 *      pPollingConstraints;
  uint8        keyExchangeTransferCount;
  uint8        maxBindingCandidates;
  uint8        identificationCapabilities;
  uint8        minClassFilter;
  uint8        maxClassFilter;
  uint8        minLqiFilter;
#endif
#ifdef FEATURE_RECIPIENT
  uint8        tertiaryClassDescriptor;
  uint8        secondaryClassDescriptor;
  uint8        primaryClassDescriptor;
  uint8        bindingCap;
  uint16       bindingRecipientNormalValidationWaitTime;
  uint16       bindingRecipientExtendedValidationWaitTime;
  gdpPollConfigAttr_t pollConfig;
#endif
} gdpConfigParams_t;

typedef struct
{
  // This is a collection of binding information that needs to be stored in NV.
  // This is needed because the binding procedure will first have the network
  // layer store the pairing information when pairing is complete. However, the
  // configuration and validation phases have not completed yet. Thus, if the
  // device is powered down in the middle of the binding procedure, the pairing
  // entry will be valid, but the binding will not have completed.
  uint8 cfgCompleteDisc[GDP_BIND_DISC_CNT];
} gdpBindingStatus_t;

// Used to populate tables containing attribute information used to validate
// incoming attributes read from a peer node.
typedef struct
{
  uint8 attrId;
  uint8 attrLen;
  uint8 *pBuf;
} gdpAttrStats_t;

typedef struct
{
  uint8 len;
  const gdpAttrStats_t *pAttrInfo;
} gdpAttrTable_t;

typedef struct
{
  gdpAttrHeader_t attrHdr;
  uint8 *pAttrData;
} gdpParsedAttr_t;

typedef struct gdpArrayedAttrList
{
  struct gdpArrayedAttrList *pNext;
  uint8 attrLen;
  uint8 value[];
} gdpArrayedAttrList_t;

typedef struct
{
  uint8 attrId;
  uint8 entryIdLsb;
  uint8 entryIdMsb;
} gdpAttrId_t;

typedef enum
{
  GDP_ATTR_FRAME_TYPE_CMD,
  GDP_ATTR_FRAME_TYPE_RSP
} gdpAttrFrameType_t;

#ifdef __cplusplus
}
#endif

#endif
