/**************************************************************************************************
  Filename:       zrc.h
  Revised:        $Date: 2014-09-24 12:24:14 -0700 (Wed, 24 Sep 2014) $
  Revision:       $Revision: 40267 $

  Description:

  This file contains the declarations for the ZRC 2.0 Profile.

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
#ifndef ZRC_H
#define ZRC_H

#ifdef __cplusplus
extern "C" {
#endif

#if !defined PACK_1
#define PACK_1
#endif

#if defined(_MSC_VER) || defined(unix) || (defined(__ICC430__) && (__ICC430__==1))
#pragma pack(1)
#endif

/*********************************************************************
 * INCLUDES
 */
#include "hal_types.h"
#include "rti_lnx.h"
#include "zrc_profile.h"

/*********************************************************************
 * CONSTANTS
 */
// ZRC Profile Item Ids
#define ZRC_ITEM_ZRC_PROFILE_VERSION                    aplZrcProfileVersion                     // 0xa0
#define ZRC_ITEM_ZRC_PROFILE_CAPABILITIES               aplZrcProfileCapabilities                // 0xa1
#define ZRC_ITEM_ACTION_BANKS_SUPPORTED_RX              aplActionBanksSupportedRx                // 0xa4
#define ZRC_ITEM_ACTION_BANKS_SUPPORTED_TX              aplActionBanksSupportedTx                // 0xa5
#define ZRC_ITEM_ZRC_ACTION_BANKS_VERSION               aplZrcActionBanksVersion                 // 0xa6

// The following item IDs are outside the ZRC attribute ID range and should be safe
#define ZRC_ITEM_MINIMUM_PEER_ZRC_PROFILE_CAPABILITIES  0xE0

// ZRC NV items
#if !defined ZRC_NVID_BEG
#define ZRC_NVID_BEG 0xB0  // Arbitrary, safe NV Id that does not conflict with RTI usages.
#endif

#define ZRC_NVID_ZRC_CAPABILITIES                 (ZRC_NVID_BEG + 0) // ZRC capabilities attribute for peer nodes
#define ZRC_NVID_NEXT_AVAILABLE                   (ZRC_NVID_ZRC_CAPABILITIES + RCN_CAP_PAIR_TABLE_SIZE) // stress that there is 1 ZRC capabilities NVID per pairing table entry

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * TYPEDEFS
 */
// Configuration Parameters Table
PACK_1 typedef struct ATTR_PACKED
{
  uint32       zrcProfileCapabilities;
  uint32       minimumPeerZrcProfileCapabilities;
  uint8        actionRepeatTriggerInterval;
#ifdef FEATURE_RECIPIENT
  uint8        actionBanksSupportedRx[ZRC_ATTR_LEN_ACTION_BANKS_SUPPORTED];
  uint8        numActionBanksSupportedRx;
#endif
#ifdef FEATURE_ORIGINATOR
  uint8        actionBanksSupportedTx[ZRC_ATTR_LEN_ACTION_BANKS_SUPPORTED];
  uint8        numActionBanksSupportedTx;
#endif
  uint16       zrcProfileVersion;
  uint16       actionRepeatWaitTime;
  uint16       zrcActionBanksVersion;
  uint8 *      pIrdbVendorSupport;
#ifdef FEATURE_RECIPIENT
  uint8 *      pActionCodesSupportedRx;
#endif
#ifdef FEATURE_ORIGINATOR
  uint8 *      pActionCodesSupportedTx;
#endif
  uint8 *      pMappableActions;
  uint8 *      pActionMappings;
  uint8 *      pHomeAutomation;
  uint8 *      pHomeAutomationSupported;
  uint32       peerZrcCapabilities[RCN_CAP_PAIR_TABLE_SIZE];
} zrcConfigParams_t;

#if defined _MSC_VER || defined(unix)
#pragma pack()
#endif

#ifdef __cplusplus
}
#endif

#endif
