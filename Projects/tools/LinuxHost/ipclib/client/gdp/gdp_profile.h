/**************************************************************************************************
  Filename:       gdp_profile.h
  Revised:        $Date: 2014-06-25 14:42:58 -0700 (Wed, 25 Jun 2014) $
  Revision:       $Revision: 39205 $

  Description:    This file contains the declarations for the RF4CE GDP2.0 Profile.

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

#ifndef GDP_PROFILE_H
#define GDP_PROFILE_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "hal_types.h"

/*********************************************************************
 * CONSTANTS
 */
// GDP Header masks
#define GDP_FRM_CNTRL_CMD_CODE_MASK                            0x0f
#define GDP_FRM_CNTRL_GDP_CMD_MASK                             0x40
#define GDP_FRM_CNTRL_DATA_PENDING_MASK                        0x80

// GDP Command ID field values, with GDP command bit preset
#define GDP_CMD_ID_GENERIC_RESPONSE                            0x00
#define GDP_CMD_ID_CONFIGURATION_COMPLETE                      0x01
#define GDP_CMD_ID_HEARTBEAT                                   0x02
#define GDP_CMD_ID_GET_ATTRIBUTES                              0x03
#define GDP_CMD_ID_GET_ATTRIBUTES_RESPONSE                     0x04
#define GDP_CMD_ID_PUSH_ATTRIBUTES                             0x05
#define GDP_CMD_ID_SET_ATTRIBUTES                              0x06
#define GDP_CMD_ID_PULL_ATTRIBUTES                             0x07
#define GDP_CMD_ID_PULL_ATTRIBUTES_RESPONSE                    0x08
#define GDP_CMD_ID_CHECK_VALIDATION                            0x09
#define GDP_CMD_ID_CLIENT_NOTIFICATION                         0x0A
#define GDP_CMD_ID_KEY_EXCHANGE                                0x0B

// GDP command field constants
#define GDP_GENERIC_RSP_SUCCESS                                0x00
#define GDP_GENERIC_RSP_UNSUPPORTED_REQ                        0x01
#define GDP_GENERIC_RSP_INVALID_PARAM                          0x02
#define GDP_GENERIC_RSP_CONFIG_FAILURE                         0x03

#define GDP_HEARTBEAT_TRIGGER_GENERIC_ACTIVITY                 0x00
#define GDP_HEARTBEAT_TRIGGER_TIME_BASED                       0x01
#define GDP_HEARTBEAT_TRIGGER_KEY_PRESS                        0x02
#define GDP_HEARTBEAT_TRIGGER_USER_PICKUP                      0x03
#define GDP_HEARTBEAT_TRIGGER_RESET                            0x04
#define GDP_HEARTBEAT_TRIGGER_MICROPHONE_ACTIVITY              0x05
#define GDP_HEARTBEAT_TRIGGER_OTHER_USER_ACTIVITY              0x06

#define GDP_ATTR_RSP_SUCCESS                                   0x00
#define GDP_ATTR_RSP_UNSUPPORTED                               0x01
#define GDP_ATTR_RSP_ILLEGAL_REQ                               0x02
#define GDP_ATTR_RSP_INVALID_ENTRY                             0x03

#define GDP_CHECK_VALIDATION_SUB_TYPE_REQ                      0x00
#define GDP_CHECK_VALIDATION_SUB_TYPE_RSP                      0x01

#define GDP_CHECK_VALIDATION_STATUS_SUCCESS                    0x00
#define GDP_CHECK_VALIDATION_STATUS_PENDING                    0x01
#define GDP_CHECK_VALIDATION_STATUS_TIMEOUT                    0x02
#define GDP_CHECK_VALIDATION_STATUS_FAILURE                    0x03

#define GDP_CLIENT_NOTIFICATION_SUB_TYPE_IDENTIFY                 0x00
#define GDP_CLIENT_NOTIFICATION_SUB_TYPE_REQUEST_POLL_NEGOTIATION 0x01
#define GDP_CLIENT_NOTIFICATION_SUB_TYPE_VENDOR_SPECIFIC_START    0xc0

#define GDP_CLIENT_NOTIFICATION_FLAGS_STOP_ON_ACTION_MASK      0x01
#define GDP_CLIENT_NOTIFICATION_FLAGS_FLASH_LIGHT_MASK         0x02
#define GDP_CLIENT_NOTIFICATION_FLAGS_SHORT_SOUND_MASK         0x04
#define GDP_CLIENT_NOTIFICATION_FLAGS_VIBRATE_MASK             0x08

#define GDP_KEY_EXCHANGE_SUB_TYPE_CHALLENGE                    0x00
#define GDP_KEY_EXCHANGE_SUB_TYPE_CHALLENGE_RSP                0x01
#define GDP_KEY_EXCHANGE_SUB_TYPE_RSP                          0x02
#define GDP_KEY_EXCHANGE_SUB_TYPE_CNF                          0x03

#define GDP_KEY_EXCHANGE_FLAGS_STANDARD_PASSWORD_MASK          0x01
#define GDP_KEY_EXCHANGE_FLAGS_VENDOR_SPECIFIC_PASSWORD_MASK   0x02

#define GDP_CLASS_NUM_PRE_COMMISSIONED                         0x00
#define GDP_CLASS_NUM_BUTTON_PRESS_IND                         0x01
#define GDP_CLASS_NUM_DISCOVERABLE_ONLY                        0x0f

#define GDP_DUP_CLASS_HDL_USE_AS_IS                            0
#define GDP_DUP_CLASS_HDL_RECLASS_NODE                         1
#define GDP_DUP_CLASS_HDL_ABORT                                2
#define GDP_DUP_CLASS_HDL_RESERVED                             3
#define GDP_DUP_CLASS_HDL_DO_NOT_USE                           0xFF // special value
/*
 * GDP Profile Constants
 */
// Length of time between completing the configuration phase of one profile and starting the next
// to allow the node to perform internal housekeeping tasks.
#define aplcConfigBlackoutTime             100   // Time in msec

// Duration of the binding window.
#define aplcBindWindowDuration             30000 // Time in msec.

// Maximum allowed time for the aplAutoCheckValidationPeriod attribute.
#define aplcMaxAutoCheckValidationPeriod   10000   // Time in msec.

// Maximum time the binding recipient shall wait to receive a command frame
// from a binding originator during its configuration phase.
#define aplcMaxConfigWaitTime              100   // Time in msec.

// Maximum time the validation can take in normal validation mode.
#define aplcMaxNormalValidationDuration    25000 // Time in msec.

// Maximum time the validation can take in extended validation mode.
#define aplcMaxExtendedValidationDuration  65000 // Time in msec.

// Maximum allowed value to configure the polling timeout in the
// aplPollConfiguration attribute.
#define aplcMaxPollingTimeout              100   // Time in msec.

// Maximum time a node shall leave its receiver on in order to receive data
// indicated via the data pending subfield of the frame control field of an
// incoming frame.
#define aplcMaxRxOnWaitTime                100   // Time in msec.

// Maximum time a node shall wait for a response command frame following a
// request command frame.
#ifdef aplcMaxResponseWaitTime
#undef aplcMaxResponseWaitTime
#endif
#define aplcMaxResponseWaitTime            100   // Time in msec.

// Min value of the KeyExTransferCount parameter passed to a pair request primitive during pairing.
#define aplcMinKeyExchangeTransferCount    3

// Max time a device waits after receiving successful NLME-AUTO-DISCOVERY.confirm for a pair ind.
#define aplcGdpMaxPairIndicationWaitTime   1200  // Time in msec.

/*
 * GDP Profile Attribute Ids
 */
#define aplGDPVersion                          0x80
#define aplGDPCapabilities                     0x81
#define aplKeyExchangeTransferCount            0x82
#define aplPowerStatus                         0x83
#define aplPollConstraints                     0x84
#define aplPollConfiguration                   0x85
#define aplMaxBindingCandidates                0x86
#define aplAutoCheckValidationPeriod           0x87
#define aplBindingRecipientValidationWaitTime  0x88
#define aplBindingOriginatorValidationWaitTime 0x89
#define aplLinkLostWaitTime                    0x8a
#define aplIdentificationCapabilities          0x8b

// Arrayed attribute ranges
#define GDP_ATTR_SCALAR_OR_ARRAY_MASK 0xf0
#define GDP_ATTR_ID_ARRAYED_MASK_1 0x90
#define GDP_ATTR_ID_ARRAYED_MASK_2 0xc0
#define GDP_ATTR_ID_ARRAYED_MASK_3 0xd0

/*
 * GDP Attribute lengths, in bytes
 */
#define GDP_ATTR_LEN_VARIABLE 0xFE   // special value not in spec
#define GDP_ATTR_LEN_GDP_VERSION                             2
#define GDP_ATTR_LEN_GDP_CAPABILITIES                        4
#define GDP_ATTR_LEN_KEY_EXCHANGE_TRANSFER_COUNT             1
#define GDP_ATTR_LEN_POWER_STATUS                            1
#define GDP_ATTR_REC_LEN_POLL_CONSTRAINT_REC                 13
#define GDP_ATTR_LEN_POLL_CONSTRAINTS( _numMethods ) \
  (1 + (_numMethods) * GDP_ATTR_REC_LEN_POLL_CONSTRAINT_REC)
#define GDP_ATTR_LEN_POLL_CONFIGURATION                      9
#define GDP_ATTR_LEN_MAX_PAIRING_CANDIDATES                  1
#define GDP_ATTR_LEN_AUTO_CHECK_VALIDATION_PERIOD            2
#define GDP_ATTR_LEN_BINDING_RECIPIENT_VALIDATION_WAIT_TIME  2
#define GDP_ATTR_LEN_BINDING_ORIGINATOR_VALIDATION_WAIT_TIME 2
#define GDP_ATTR_LEN_LINK_LOST_WAIT_TIME                     2
#define GDP_ATTR_LEN_IDENTIFICATION_CAPABILITIES             1

/*
 * GDP attribute defines
 */
#define GDP_POWER_STATUS_POWER_METER_MASK    0x0f
#define GDP_POWER_STATUS_CRITICAL_MASK       0x20
#define GDP_POWER_STATUS_CHARGING_MASK       0x40
#define GDP_POWER_STATUS_IMDENDING_DOOM_MASK 0x80

#define GDP_CAP_SUPPORT_EXTENDED_VALIDATION_MASK    0x00000001
#define GDP_CAP_SUPPORT_POLL_SERVER_MASK            0x00000002
#define GDP_CAP_SUPPORT_POLL_CLIENT_MASK            0x00000004
#define GDP_CAP_SUPPORT_IDENTIFICATION_SERVER_MASK  0x00000008
#define GDP_CAP_SUPPORT_IDENTIFICATION_CLIENT_MASK  0x00000010
#define GDP_CAP_SUPPORT_ENHANCED_SECURITY_MASK      0x00000020
#define GDP_CAP_SUPPORT_SHARED_SECRET_LOCAL_VENDOR  0x00000040
#define GDP_CAP_SUPPORT_SHARED_SECRET_REMOTE_VENDOR 0x00000080

#define GDP_IDENTIFY_CAP_SUPPORT_FLASH_LIGHT_MASK 0x02
#define GDP_IDENTIFY_CAP_SUPPORT_SHORT_SOUND_MASK 0x04
#define GDP_IDENTIFY_CAP_SUPPORT_VIBRATE_MASK     0x08

/*
 * GDP command field indices
 */
#define GDP_COMMAND_INDEX_FRAME_CONTROL 0

/*********************************************************************
 * MACROS
 */
#define GDP_CMD_BUILD_CMD_ID(_cmd) ((_cmd) | GDP_FRM_CNTRL_GDP_CMD_MASK)

#define GDP_ATTR_IS_ATTR_ARRAYED(_attrId)                        \
               ( (((_attrId) >= 0x90) && ((_attrId) <= 0x9f)) || \
                 (((_attrId) >= 0xc0) && ((_attrId) <= 0xdf)) )

#define GDP_CLASS_DESC_GET_CLASS_NUMBER(_desc) \
  ((_desc) & 0x0F)
#define GDP_CLASS_DESC_GET_DUPL_HDL(_desc) \
  (((_desc) & 0x30) >> 4)

#define GDP_CLASS_DESC_BUILD_DESCRIPTOR(_classNum, _duplHdl) \
  ( ((_classNum) & 0x0F) | (((_duplHdl) & 0x03) << 4) )
#define GDP_CLASS_DESC_SET_CLASS_NUMBER(_desc, _val) \
  ((_desc) |= ((_val) & 0x0F))
#define GDP_CLASS_DESC_SET_DUPL_HDL(_desc, _val) \
  ((_desc) |= (((_val) & 0x03) << 4))

/*********************************************************************
 * TYPEDEFS
 */

// To be compatible with MS native target
// declare pragma for structure packing
#if defined(_MSC_VER) || defined(unix) || (defined(__ICC430__) && (__ICC430__==1))
#pragma pack(1)
#endif

// Generic Response command
typedef struct
{
  uint8 hdr;
  uint8 rspCode;
} gdpGenericRspCmd_t;

// Configuration Complete command
typedef struct
{
  uint8 hdr;
  uint8 status;
} gdpCfgCompleteCmd_t;

// Heartbeat command
typedef struct
{
  uint8 hdr;
  uint8 trigger;
} gdpHeartbeatCmd_t;

// Get Attributes command
typedef struct
{
  uint8 hdr;
  uint8 attrIdList[]; // variable length list of variable length records
} gdpGetAttrCmd_t;

// Get Attributes Response command
typedef struct
{
  uint8 hdr;
  uint8 attrStatusRecordList[]; // variable length list of variable length records
} gdpGetAttrRspCmd_t;

// Push Attributes command
typedef struct
{
  uint8 hdr;
  uint8 attrRecordList[];
} gdpPushAttrCmd_t;

// Set Attributes command
typedef struct
{
  uint8 hdr;
  uint8 attrRecordList[];
} gdpSetAttrCmd_t;

// Pull Attributes command
typedef struct
{
  uint8 hdr;
  uint8 attrIdList[]; // variable length list of variable length records
} gdpPullAttrCmd_t;

// Pull Attributes Response command
typedef struct
{
  uint8 hdr;
  uint8 attrStatusRecordList[]; // variable length list of variable length records
} gdpPullAttrRspCmd_t;

// Check Validation command
typedef struct
{
  uint8 hdr;
  uint8 subType;
  uint8 payload;
} gdpChkValidationCmd_t;

// Client Notification command
typedef struct
{
  uint8 hdr;
  uint8 subType;
  uint8 payload[];
} gdpClientNotificationCmd_t;

// Key Exchange command
typedef struct
{
  uint8 hdr;
  uint8 subType;
  uint8 payload[];
} gdpKeyExchangeCmd_t;

// Attribute Id
typedef struct
{
  uint8 attrId;
} gdpAttrIdScalar_t;

typedef struct
{
  uint8 attrId;
  uint8 entryIdLsb;
  uint8 entryIdMsb;
} gdpAttrIdIndexed_t;

// Push Attributes command attribute record
typedef struct
{
  uint8 attrId;
  uint8 attrLen;
  uint8 value[];
} gdpAttrRecScalar_t;

typedef struct
{
  uint8 attrId;
  uint8 entryIdLsb;
  uint8 entryIdMsb;
  uint8 attrLen;
  uint8 value[];
} gdpAttrRecIndexed_t;

// Get Attributes Response command attribute record
typedef struct
{
  uint8 attrId;
  uint8 attrStatus;
  uint8 attrLen;
  uint8 value[];
} gdpAttrStatusRecScalar_t;

typedef struct
{
  uint8 attrId;
  uint8 entryIdLsb;
  uint8 entryIdMsb;
  uint8 attrStatus;
  uint8 attrLen;
  uint8 value[];
} gdpAttrStatusRecIndexed_t;

#define GDP_ATTR_REC_LEN_GDP_VERSION \
  (GDP_ATTR_LEN_GDP_VERSION + sizeof( gdpAttrRecScalar_t ))
#define GDP_ATTR_STATUS_REC_LEN_GDP_VERSION \
  (GDP_ATTR_LEN_GDP_VERSION + sizeof( gdpAttrStatusRecScalar_t ))

#define GDP_ATTR_REC_LEN_KEY_EXCHANGE_TRANSFER_COUNT \
  (GDP_ATTR_LEN_KEY_EXCHANGE_TRANSFER_COUNT + sizeof( gdpAttrRecScalar_t ))

#define GDP_ATTR_REC_LEN_POWER_STATUS \
  (GDP_ATTR_LEN_POWER_STATUS + sizeof( gdpAttrRecScalar_t ))

#define GDP_ATTR_REC_LEN_GDP_CAPABILITIES \
  (GDP_ATTR_LEN_GDP_CAPABILITIES + sizeof( gdpAttrRecScalar_t ))
#define GDP_ATTR_STATUS_REC_LEN_GDP_CAPABILITIES \
  (GDP_ATTR_LEN_GDP_CAPABILITIES + sizeof( gdpAttrStatusRecScalar_t ))

#define GDP_ATTR_REC_LEN_POLL_CONSTRAINTS( _numRecords ) \
  (GDP_ATTR_LEN_POLL_CONSTRAINTS( (_numRecords) ) + sizeof( gdpAttrRecScalar_t ))
#define GDP_REC_LEN_POLL_CONSTRAINTS( _numRecords ) \
  ((_numRecords) * GDP_ATTR_LEN_POLL_CONSTRAINTS_REC)

#define GDP_ATTR_REC_LEN_POLL_CONFIGURATION \
  (GDP_ATTR_LEN_POLL_CONFIGURATION + sizeof( gdpAttrRecScalar_t ))
#define GDP_ATTR_STATUS_REC_LEN_POLL_CONFIGURATION \
  (GDP_ATTR_LEN_POLL_CONFIGURATION + sizeof( gdpAttrStatusRecScalar_t ))

#define GDP_ATTR_REC_LEN_MAX_PAIRING_CANDIDATES \
  (GDP_ATTR_LEN_MAX_PAIRING_CANDIDATES + sizeof( gdpAttrRecScalar_t ))

#define GDP_ATTR_REC_LEN_AUTO_CHECK_VALIDATION_PERIOD \
  (GDP_ATTR_LEN_AUTO_CHECK_VALIDATION_PERIOD + sizeof( gdpAttrRecScalar_t ))
#define GDP_ATTR_STATUS_REC_LEN_AUTO_CHECK_VALIDATION_PERIOD \
  (GDP_ATTR_LEN_AUTO_CHECK_VALIDATION_PERIOD + sizeof( gdpAttrStatusRecScalar_t ))

#define GDP_ATTR_REC_LEN_BINDING_RECIPIENT_VALIDATION_WAIT_TIME \
  (GDP_ATTR_LEN_BINDING_RECIPIENT_VALIDATION_WAIT_TIME + sizeof( gdpAttrRecScalar_t ))

#define GDP_ATTR_REC_LEN_BINDING_ORIGINATOR_VALIDATION_WAIT_TIME \
  (GDP_ATTR_LEN_BINDING_ORIGINATOR_VALIDATION_WAIT_TIME + sizeof( gdpAttrRecScalar_t ))

#define GDP_ATTR_REC_LEN_LINK_LOST_WAIT_TIME \
  (GDP_ATTR_LEN_LINK_LOST_WAIT_TIME + sizeof( gdpAttrRecScalar_t ))
#define GDP_ATTR_STATUS_REC_LEN_LINK_LOST_WAIT_TIME \
  (GDP_ATTR_LEN_LINK_LOST_WAIT_TIME + sizeof( gdpAttrStatusRecScalar_t ))

#define GDP_ATTR_REC_LEN_IDENTIFICATION_CAPABILITIES \
  (GDP_ATTR_LEN_IDENTIFICATION_CAPABILITIES + sizeof( gdpAttrRecScalar_t ))

// aplPollConstraints attribute
typedef struct
{
  uint8 pollingMethodId;
  uint16 pollingTriggerCapabilities;
  uint8 minPollingKeyPressCounter;
  uint8 maxPollingKeyPressCounter;
  uint32 minPollingTimeInterval;
  uint32 maxPollingTimeInterval;
} gdpPollConstraintRec_t;

typedef struct
{
  uint8 numPollMethodsSupported;
  gdpPollConstraintRec_t records[];
} gdpPollConstraintsAttr_t;

#define GDP_POLL_METHOD_DISABLED  0x00
#define GDP_POLL_METHOD_HEARTBEAT 0x01

#define GDP_POLL_TIME_BASED_MASK           0x01
#define GDP_POLL_KEY_PRESS_MASK            0x02
#define GDP_POLL_PICK_UP_MASK              0x04
#define GDP_POLL_RESET_MASK                0x08
#define GDP_POLL_MICROPHONE_ACTIVITY_MASK  0x10
#define GDP_POLL_OTHER_USER_ACTIVITY_MASK  0x20

#define GDP_POLL_MIN_TIME_INTERVAL_MIN_VALUE 50
#define GDP_POLL_MIN_TIME_INTERVAL_MAX_VALUE 3600000
#define GDP_POLL_MAX_TIME_INTERVAL_MIN_VALUE 60000
#define GDP_POLL_MAX_TIME_INTERVAL_MAX_VALUE 86400000

// aplPollConfiguration attribute
typedef struct
{
  uint8 pollingMethodId;
  uint16 pollingTriggerConfig;
  uint8 pollingKeyPressCounter;
  uint32 pollingTimeInterval;
  uint8 pollingTimeout;
} gdpPollConfigAttr_t;

// client notification fields
typedef struct
{
  uint8 identifyFlags;
  uint16 identifyTime;
} gdpClientNotificationIdentify_t;

// Key Exchange feature definitions
#define GDP_RANDOM_STRING_LENGTH 8
#define GDP_TAG_STRING_LENGTH 4

#define GDP_KEY_EXCHANGE_FLAGS_DEFAULT_SECRET_MASK             0x0001
#define GDP_KEY_EXCHANGE_FLAGS_INITIATOR_VENDOR_SPECIFIC_MASK  0x0002
#define GDP_KEY_EXCHANGE_FLAGS_RESPONDER_VENDOR_SPECIFIC_MASK  0x0004
#define GDP_KEY_EXCHANGE_FLAGS_VENDOR_SPECIFIC_PARAMETERS_MASK 0xFF00

// key exchange challenge sub-type payload
typedef struct
{
  uint16 keyExchangeFlags;
  uint8 randA[GDP_RANDOM_STRING_LENGTH];
} gdpKeyExchangeChallenge_t;

// key exchange challenge response sub-type payload
typedef struct
{
  uint16 keyExchangeFlags;
  uint8 randB[GDP_RANDOM_STRING_LENGTH];
  uint8 tagB[GDP_TAG_STRING_LENGTH];
} gdpKeyExchangeChallengeRsp_t;

// key exchange response sub-type payload
typedef struct
{
  uint8 tagA[GDP_TAG_STRING_LENGTH];
} gdpKeyExchangeRsp_t;

/*********************************************************************
 * GLOBALS
 */

/*********************************************************************
 * FUNCTIONS
 */

/*********************************************************************
*********************************************************************/

#if defined _MSC_VER || defined(unix)
#pragma pack()
#endif

#ifdef __cplusplus
}
#endif

#endif
