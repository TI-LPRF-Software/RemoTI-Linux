/**************************************************************************************************
  Filename:       zrc_profile.h

  Description:    This file contains the declarations for the RF4CE ZRC2.0 Profile.

  Copyright (C) 2014-2015 Texas Instruments Incorporated - http://www.ti.com/


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

#ifndef ZRC_PROFILE_H
#define ZRC_PROFILE_H

#ifdef __cplusplus
extern "C"
{
#endif

// To be compatible with MS native target
// declare pragma for structure packing
#if defined(_MSC_VER) || defined(unix) || (defined(__ICC430__) && (__ICC430__==1))
#pragma pack(1)
#endif

/*********************************************************************
 * INCLUDES
 */
#include "hal_types.h"

/*********************************************************************
 * CONSTANTS
 */
// ZRC Command ID field values
#define ZRC_CMD_ID_ACTIONS                0x06

/*
 * ZRC Profile Constants
 */
// Maximum time between consecutive action command frame transmissions
// indicating a repeated action.
#define aplcMaxActionRepeatTriggerInterval    200   // Time in msec

// The time that an action control record should be repeated
#define aplcShortRetryDuration                100   // Time in msec

/*
 * ZRC Profile Attribute Ids
 */
#define aplZrcProfileVersion                  0xa0
#define aplZrcProfileCapabilities             0xa1
#define aplActionRepeatTriggerInterval        0xa2
#define aplActionRepeatWaitTime               0xa3
#define aplActionBanksSupportedRx             0xa4
#define aplActionBanksSupportedTx             0xa5
#define aplIrdbVendorSupport                  0xa6
#define aplZrcActionBanksVersion              0xa7
#define aplActionCodesSupportedRx             0xc0
#define aplActionCodesSupportedTx             0xc1
#define aplMappableActions                    0xc2
#define aplActionMappings                     0xc3
#define aplHomeAutomation                     0xc4
#define aplHomeAutomationSupported            0xc5

/*
 * ZRC Attribute lengths, in bytes
 */
#define ZRC_ATTR_LEN_ZRC_PROFILE_VERSION                    2
#define ZRC_ATTR_LEN_ZRC_PROFILE_CAPABILITIES               4
#define ZRC_ATTR_LEN_ACTION_REPEAT_TRIGGER_INTERVAL         1
#define ZRC_ATTR_LEN_ACTION_REPEAT_WAIT_TIME                2
#define ZRC_ATTR_LEN_ACTION_BANKS_SUPPORTED                 32
#define ZRC_ATTR_LEN_ZRC_ACTION_BANKS_VERSION               2
#define ZRC_ATTR_LEN_ACTION_CODES_SUPPORTED                 32
#define ZRC_ATTR_LEN_MAPPABLE_ACTIONS                       3
#define ZRC_ATTR_LEN_IRDB_VENDOR_SUPPORT(_numVendors)       (16 * (_numVendors))
#define ZRC_ATTR_LEN_HOME_AUTOMATION_SUPPORTED              32

/*
 * ZRC command field constants
 */
// To be used for control field of zrcActionRecord_t
#define ZRC_ACTION_CTRL_TYPE_START            0x01
#define ZRC_ACTION_CTRL_TYPE_REPEAT           0x02
#define ZRC_ACTION_CTRL_TYPE_ATOMIC           0x03
#define ZRC_ACTION_TYPE_MASK                  0x03
#define ZRC_MODIFIER_MASK_GUI                 0x10
#define ZRC_MODIFIER_MASK_ALT                 0x20
#define ZRC_MODIFIER_MASK_SHIFT               0x40
#define ZRC_MODIFIER_MASK_CTRL                0x80

/*
 * ZRC attribute defines
 */
// aplZrcProfileCapabilities
#define ZRC_CAP_SUPPORT_ACTIONS_ORIGINATOR_MASK            0x00000001
#define ZRC_CAP_SUPPORT_ACTIONS_RECIPIENT_MASK             0x00000002
#define ZRC_CAP_SUPPORT_HA_ACTIONS_ORIGINATOR_MASK         0x00000004
#define ZRC_CAP_SUPPORT_HA_ACTIONS_RECIPIENT_MASK          0x00000008
#define ZRC_CAP_SUPPORT_ACTION_MAPPING_CLIENT_MASK         0x00000010
#define ZRC_CAP_SUPPORT_ACTION_MAPPING_SERVER_MASK         0x00000020
#define ZRC_CAP_SUPPORT_VENDOR_SPECIFIC_IRDB_FORMATS_MASK  0x00000040
#define ZRC_CAP_INFORM_ABOUT_SUPPORTED_ACTIONS_MASK        0x00000080

/*
 * Action Bank "class" definitions - these define bits 5-7 of the action bank
 */
#define ZRC_ACTION_BANK_CLASS_HDMI_CEC                  (0 << 5)
#define ZRC_ACTION_BANK_CLASS_HID                       (1 << 5)
#define ZRC_ACTION_BANK_CLASS_HA                        (4 << 5)
#define ZRC_ACTION_BANK_CLASS_VENDOR_SPECIFIC_SOURCE    (5 << 5)
#define ZRC_ACTION_BANK_CLASS_VENDOR_SPECIFIC_RECIPIENT (6 << 5)
#define ZRC_ACTION_BANK_CLASS_VENDOR_SPECIFIC_EXPLICIT  (7 << 5)

/*
 * HA Action Bank defines
 */
#define ZRC_HA_ACTION_BANK_MIN_INDEX    16
#define ZRC_HA_ACTION_BANK_MAX_INDEX    19

/*
 * Action bank "bank" definitions - these define bits 0-4 of the action bank
 */
#define ZRC_ACTION_BANK_HDMI_CEC          0
#define ZRC_ACTION_BANK_HID_KEYBOARD_A    0
#define ZRC_ACTION_BANK_HID_TELEPHONY_A   1
#define ZRC_ACTION_BANK_HID_CONSUMER_A    2
#define ZRC_ACTION_BANK_HID_CONSUMER_B    3
#define ZRC_ACTION_BANK_HID_CONSUMER_C    4
#define ZRC_ACTION_BANK_HID_GAME_CONTROLS 5

/*
 * ZRC Identification types
 */
#define ZRC_CLIENT_NOTIFICATION_SUB_TYPE_REQUEST_ACTION_MAPPING_NEGOTIATION      0x40
#define ZRC_CLIENT_NOTIFICATION_SUB_TYPE_REQUEST_HOME_AUTOMATION_PULL            0x41
#define ZRC_CLIENT_NOTIFICATION_SUB_TYPE_REQUEST_SELECTIVE_ACTION_MAPPING_UPDATE 0x42
#define ZRC_HA_PULL_DIRTY_FLAGS_LEN 32

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * TYPEDEFS
 */

// Actions command and associated records
typedef struct
{
  uint8 hdr;
  uint8 actionRecordsList[];
} zrcActionsCmd_t;

typedef struct
{
  uint8 control;
  uint8 actionPayloadLen;
  uint8 actionBank;
  uint8 actionCode;
  uint8 actionVendorPayload[];
} zrcActionRecord_t;

typedef struct
{
  uint8 control;
  uint8 actionPayloadLen;
  uint8 actionBank;
  uint8 actionCode;
  uint16 vendorId;
  uint8 actionVendorPayload[];
} zrcVendorActionRecord_t;

// aplMappableActions element
typedef struct
{
  uint8 actionDeviceType;
  uint8 actionBank;
  uint8 actionCode;
} zrcActionMap_t;

// aplActionMappings element
typedef struct
{
  uint8 flags;
  uint8 descriptors[];
} zrcActionMappingsAttr_t;

#define ZRC_FLAGS_RF_SPECIFIED_MASK        0x01
#define ZRC_FLAGS_IR_SPECIFIED_MASK        0x02
#define ZRC_FLAGS_RF_DESCRIPTOR_FIRST_MASK 0x04
#define ZRC_FLAGS_USE_DEFAULT_MASK         0x40
#define ZRC_FLAGS_PERMANENT_MASK           0x80

typedef struct
{
  uint8 rfConfig;
  uint8 txOptions;
  uint8 payloadLength;
  uint8 payload[];
} zrcRfDescriptor_t;

#define ZRC_RF_CONFIG_MIN_TRANSMISSIONS_MASK      0x0F
#define ZRC_RF_CONFIG_XMIT_UNTIL_KEY_RELEASE_MASK 0x10
#define ZRC_RF_CONFIG_SHORT_RETRY_MASK            0x20
#define ZRC_RF_CONFIG_ATOMIC_ACTION_MASK          0x40

typedef struct
{
  uint8 irConfig;
  uint8 irCodeInfo[];
} zrcIrDescriptor_t;

#define ZRC_IR_CONFIG_VENDOR_SPECIFIC_MASK 0x01

// aplHomeAutomation element
typedef struct
{
  uint8 haAttrStatus;
  uint8 haAttrValue[];
} zrcHomeAutomationAttr_t;

#define ZRC_HA_VALUE_AVAILABLE_MASK 0x01

#define ZRC_ATTR_REC_LEN_ZRC_PROFILE_VERSION \
  (ZRC_ATTR_LEN_ZRC_PROFILE_VERSION + sizeof( gdpAttrRecScalar_t ))
#define ZRC_ATTR_STATUS_REC_LEN_ZRC_PROFILE_VERSION \
  (ZRC_ATTR_LEN_ZRC_PROFILE_VERSION + sizeof( gdpAttrStatusRecScalar_t ))

#define ZRC_ATTR_REC_LEN_ZRC_PROFILE_CAPABILITIES \
  (ZRC_ATTR_LEN_ZRC_PROFILE_CAPABILITIES + sizeof( gdpAttrRecScalar_t ))
#define ZRC_ATTR_STATUS_REC_LEN_ZRC_PROFILE_CAPABILITIES \
  (ZRC_ATTR_LEN_ZRC_PROFILE_CAPABILITIES + sizeof( gdpAttrStatusRecScalar_t ))

#define ZRC_ATTR_REC_LEN_ACTION_BANKS_SUPPORTED \
  (ZRC_ATTR_LEN_ACTION_BANKS_SUPPORTED + sizeof( gdpAttrRecScalar_t ))
#define ZRC_ATTR_STATUS_REC_LEN_ACTION_BANKS_SUPPORTED \
  (ZRC_ATTR_LEN_ACTION_BANKS_SUPPORTED + sizeof( gdpAttrStatusRecScalar_t ))

#define ZRC_ATTR_REC_LEN_ACTION_CODES_SUPPORTED \
  (ZRC_ATTR_LEN_ACTION_CODES_SUPPORTED + sizeof( gdpAttrRecIndexed_t ))
#define ZRC_ATTR_STATUS_REC_LEN_ACTION_CODES_SUPPORTED \
  (ZRC_ATTR_LEN_ACTION_CODES_SUPPORTED + sizeof( gdpAttrStatusRecIndexed_t ))

// client notification fields
typedef struct
{
  uint8 haInstanceId;
  uint8 haAttributeDirtyFlags[ZRC_HA_PULL_DIRTY_FLAGS_LEN];
} zrcClientNotificationHaPull_t;

/*********************************************************************
 * GLOBALS
 */

/*********************************************************************
 * FUNCTIONS
 */

/*********************************************************************
*********************************************************************/

// End packing rule
#if defined(_MSC_VER) || defined(unix) || (defined(__ICC430__) && (__ICC430__==1))
#pragma pack()
#endif

#ifdef __cplusplus
}
#endif

#endif
