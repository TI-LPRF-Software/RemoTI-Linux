/**************************************************************************************************
  Filename:       rti_lnx.h
  Revised:        $Date: 2012-05-10 08:17:33 -0700 (Thu, 10 May 2012) $
  Revision:       $Revision: 308 $

  Description:    This file contains the the RemoTI (RTI) API.

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

#ifndef RTI_LNX_H
#define RTI_LNX_H

#ifdef __cplusplus
extern "C"
{
#endif

/**************************************************************************************************
 * INCLUDES
 **************************************************************************************************/

// Hal Driver Includes
#include "hal_types.h"

// RTI_LNX Includes
#include "rti_lnx_constants.h"

// Services
#include "saddr.h"

/**************************************************************************************************
 * MACROS
 **************************************************************************************************/

// Define a prefix macro for exported functions and global variables
// which can be used for generating proper DLL interface for windows platform
#ifdef _WIN32
#  ifdef RTILIB_EXPORTS
#    define RTILIB_API __declspec(dllexport)
#  else
#    define RTILIB_API __declspec(dllimport)
#  endif
#else
#  define RTILIB_API
#endif

// Network layer implicit constants as in standard used by attribute structure
#define RCN_MAX_NUM_DEV_TYPES                     3 // number of device types supported per device
#define RCN_SEC_KEY_LENGTH                        16 // length in bytes of network layer security key

#define RCN_MAX_NUM_PROFILE_IDS                   7 // number of profile IDs supported per device
#define RCN_VENDOR_STRING_LENGTH                  7
#define RCN_USER_STRING_LENGTH                    15
#define RCN_SEC_KEY_SEED_LENGTH                   80

#define RCN_NODE_CAP_TARGET						  1

#ifndef RCN_FEATURE_SECURITY
# define RCN_FEATURE_SECURITY                     TRUE
#endif

#define	RCN_PROFILE_DISCS_SIZE					  1

#ifndef RCN_FEATURE_EXTRA_PAIR_INFO
# define RCN_FEATURE_EXTRA_PAIR_INFO              TRUE
#endif

#if RCN_FEATURE_EXTRA_PAIR_INFO
// Discrete bit definitions for rcnNwkPairingEntry_t.profileDiscs[].
#define RCN_PROFILE_DISC_GT8                      0  // GT8 set would imply one or more >= 8.
#define RCN_PROFILE_DISC_ZRC                      1
#define RCN_PROFILE_DISC_ZID                      2
#define RCN_PROFILE_DISC_Z3D                      3
#define RCN_PROFILE_DISC_SP4                      4
#define RCN_PROFILE_DISC_SP5                      5
#define RCN_PROFILE_DISC_SP6                      6
#define RCN_PROFILE_DISC_SP7                      7

#define RCN_PROFILE_DISCS_SIZE                    1  // Profile Discretes bytes allowed.
#endif

// Node capabilities bits
#define RCN_NODE_CAP_TARGET                       0x01
#define RCN_NODE_CAP_POWER_SOURCE                 0x02
#define RCN_NODE_CAP_SECURITY_CAPABLE             0x04
#define RCN_NODE_CAP_CHANNEL_NORMALIZATION_CAPABLE 0x08
#define RCN_NODE_CAP_NUM_OF_PROFILES              0x70


#define RCN_NIB_NWK_USER_STRING                   0x6f // nwkUserString

/* Primitive Data Structures */

// sub structure for application capabilities field in macro
#define RCN_APP_CAPA_GET_USER_STRING(_capa) \
  ((_capa) & 0x01)
#define RCN_APP_CAPA_SET_USER_STRING(_capa,_val) \
{                                                \
  ((_capa) &= ~0x01);                            \
  ((_capa) |= (_val));                           \
}
#define RCN_APP_CAPA_GET_NUM_DEV_TYPES(_capa) \
  (((_capa) & 0x06) >> 1)
#define RCN_APP_CAPA_SET_NUM_DEV_TYPES(_capa,_val) \
{                                                  \
  ((_capa) &= ~0x06);                              \
  ((_capa) |= ((_val) << 1));                      \
}
#define RCN_APP_CAPA_GET_NUM_PROFILES(_capa) \
  (((_capa) & 0x70) >> 4)
#define RCN_APP_CAPA_SET_NUM_PROFILES(_capa,_val) \
{                                                 \
  ((_capa) &= ~0x70);                             \
  ((_capa) |= ((_val) << 4));                     \
}

// Macro to build a byte for node capabilities - each parameter has to be set to either 0 or 1.
#define RTI_BUILD_NODE_CAPABILITIES(_target,_ac_powered,_security_capable,_ch_norm_capable ) \
  ((_target) | ((_ac_powered) << 1) | ((_security_capable) << 2) | ((_ch_norm_capable) << 3))

// Macro to build a byte for app capabilities - each parameter has to be set to either 0 or 1.
#define RTI_BUILD_APP_CAPABILITIES(_usr_str, _num_devs, _num_prof) \
  ((_usr_str) | ((_num_devs) << 1) | ((_num_prof) << 4))

#ifndef ATTR_PACKED
# ifdef __GNUC__
#  define ATTR_PACKED __attribute__ ((__packed__))
# else
#  define ATTR_PACKED
# endif
#endif

/**************************************************************************************************
 * CONSTANTS
 **************************************************************************************************/

#if !defined FEATURE_USER_STRING_PAIRING
#define FEATURE_USER_STRING_PAIRING                      FALSE
#endif

// RTI API Status Values

// Application framework layer primitive status field values
#define RTI_SUCCESS                                      0
#define RTI_ERROR_INVALID_INDEX                          0xF9
#define RTI_ERROR_INVALID_PARAMETER                      0xE8
#define RTI_ERROR_UNSUPPORTED_ATTRIBUTE                  0xF4
#define RTI_ERROR_NO_ORG_CAPACITY                        0xB0
#define RTI_ERROR_NO_REC_CAPACITY                        0xB1
#define RTI_ERROR_NO_PAIRING_INDEX                       0xB2
#define RTI_ERROR_NO_RESPONSE                            0xB3
#define RTI_ERROR_NOT_PERMITTED                          0xB4
#define RTI_ERROR_FRAME_COUNTER_EXPIRED                  0xB6
#define RTI_ERROR_DISCOVERY_ERROR                        0xB7
#define RTI_ERROR_DISCOVERY_TIMEOUT                      0xB8
#define RTI_ERROR_SECURITY_TIMEOUT                       0xB9
#define RTI_ERROR_SECURITY_FAILURE                       0xBA
#define RTI_ERROR_NO_SECURITY_KEY                        0xBD
#define RTI_ERROR_OUT_OF_MEMORY                          0xBE
#define RTI_ERROR_OSAL_NO_TIMER_AVAIL                    0x08
#define RTI_ERROR_OSAL_NV_OPER_FAILED                    0x0A
#define RTI_ERROR_OSAL_NV_ITEM_UNINIT                    0x09
#define RTI_ERROR_OSAL_NV_BAD_ITEM_LEN                   0x0C
#define RTI_ERROR_MAC_TRANSACTION_EXPIRED                0xF0
#define RTI_ERROR_MAC_TRANSACTION_OVERFLOW               0xF1
#define RTI_ERROR_MAC_NO_RESOURCES                       0x1A
#define RTI_ERROR_MAC_UNSUPPORTED                        0x18
#define RTI_ERROR_MAC_BAD_STATE                          0x19
#define RTI_ERROR_MAC_CHANNEL_ACCESS_FAILURE             0xE1
#define RTI_ERROR_MAC_NO_ACK                             0xE9
#define RTI_ERROR_MAC_BEACON_LOST                        0xE0
#define RTI_ERROR_MAC_PAN_ID_CONFLICT                    0xEE
#define RTI_ERROR_MAC_SCAN_IN_PROGRESS                   0xFC
#define RTI_ERROR_UNKNOWN_STATUS_RETURNED                0x20
#define RTI_ERROR_FAILED_TO_DISCOVER                     0x21
#define RTI_ERROR_FAILED_TO_PAIR                         0x22
#define RTI_ERROR_ALLOW_PAIRING_TIMEOUT                  0x23
#define RTI_ERROR_FAILED_TO_CONFIGURE_ZRC                0x41
#define RTI_ERROR_FAILED_TO_CONFIGURE_ZID                0x42
#define RTI_ERROR_FAILED_TO_CONFIGURE_Z3D                0x43
#define RTI_ERROR_FAILED_TO_CONFIGURE_INV_MASK           (0x40)
  // reserve failure IDs 0x44-0x4A for future profiles
#define RTI_ERROR_SYNCHRONOUS_NPI_TIMEOUT                0xFF



// RTI API Parameter Constants

// RTI_SendDataReq TX Options Bit Mask
#define RTI_TX_OPTION_BROADCAST                          0x01
#define RTI_TX_OPTION_IEEE_ADDRESS                       0x02
#define RTI_TX_OPTION_ACKNOWLEDGED                       0x04
#define RTI_TX_OPTION_SECURITY                           0x08
#define RTI_TX_OPTION_SINGLE_CHANNEL                     0x10
#define RTI_TX_OPTION_CHANNEL_DESIGNATOR                 0x20
#define RTI_TX_OPTION_VENDOR_SPECIFIC                    0x40

// RTI_ReceiveDataInd RX Flags Bit Mask
#define RTI_RX_FLAGS_BROADCAST                           0x01
#define RTI_RX_FLAGS_SECURITY                            0x02
#define RTI_RX_FLAGS_VENDOR_SPECIFIC                     0x04

// RTI_RxEnableReq
#define RTI_RX_ENABLE_OFF                                0
#define RTI_RX_ENABLE_ON                                 0xFFFF

// RTI_StandbyReq
#define RTI_STANDBY_OFF                                  0
#define RTI_STANDBY_ON                                   1

// RTI Configuration Parameter default values
// The following configuration parameter default values are set arbitrarily.
// Modifying these constant values will require rebuilding RTI module.

#if RCN_FEATURE_SECURITY
#define RTI_DEFAULT_NODE_CAPABILITIES      (RTI_NODE_CAP_SEC_CAP_BM | RTI_NODE_CAP_CHAN_NORM_BM)
#else
#define RTI_DEFAULT_NODE_CAPABILITIES       RTI_NODE_CAP_CHAN_NORM_BM
#endif

#if FEATURE_USER_STRING_PAIRING
#define RTI_DEFAULT_APP_CAPABILITIES        RTI_APP_CAP_USER_STR_BM
#else
#define RTI_DEFAULT_APP_CAPABILITIES        0
#endif

#define RTI_STANDBY_ACTIVE_PERIOD                        16    // 16ms   (0x60 - nwkActivePeriod)
#define RTI_STANDBY_DUTY_CYCLE                           330   // 330ms  (0x64 - nwkDutyCycle)

// The follwoing constant values are dictated by
// RF4CE standard and hence cannot be modified at all.
#define RTI_MAX_NUM_DEV_TYPES                            RCN_MAX_NUM_DEV_TYPES
#define RTI_MAX_NUM_PROFILE_IDS                          RCN_MAX_NUM_PROFILE_IDS
#define RTI_VENDOR_STRING_LENGTH                         RCN_VENDOR_STRING_LENGTH
#define RTI_USER_STRING_LENGTH                           RCN_USER_STRING_LENGTH
#define RTI_DEST_PAN_ID_WILDCARD                         0xFFFF
#define RTI_DEST_NWK_ADDR_WILDCARD                       0xFFFF
#define RTI_REC_DEV_TYPE_WILDCARD                        0xFF
#define RTI_INVALID_PAIRING_REF                          0xFF
#define RTI_INVALID_DEVICE_TYPE                          0xFF

/**************************************************************************************************
 * ZRC specifies the following initial NIB attribute settings to comply with the proscribed
 * "Discovery/Pairing Procedure".
 */

// Controller discovery duration shall be 0x00186a symbols --> 100 msec.
#define DPP_DISCOVERY_DURATION             100

// Target automatic discovery response mode duration shall be 0x1c9c38 symbols --> 30 seconds.
#define DPP_AUTO_RESP_DURATION             30000

// 0x00f424 symbols --> 1 second.
#define DPP_DISCOVERY_REP_INTERVAL         1000   // 0x63 - nwkDiscoveryRepetitionInterval

#define DPP_MAX_DISCOVERY_REPS             30     // 0x69 - nwkDiscoveryRepetitions

#define DPP_MAX_REPORTED_NODE_DESC         1      // 0x6c - nwkMaxReportedNodeDescriptors

/**************************************************************************************************
 * ZRC leaves the following initial NIB attribute settings as implementation specific for the
 * "Discovery/Pairing Procedure".
 */

#define DPP_LQI_THRESHOLD                  0      // 0x62 - nwkDiscoveryLQIThreshold

/**************************************************************************************************
 * ZRC specifies the following constants.
 */

// Max time between consecutive user control repeated command frame transmissions.
#define aplcMaxKeyRepeatInterval           100   // Time in msec.

// Max duration that the receiver is enabled on a Controller after pairing to Rx a cmd discovery.
#define aplcMaxCmdDiscRxOnDuration         200   // Time in msec.

// Max time a Target waits after NLME-AUTO-DISCOVERY.confirm for a pair indication to arrive.
#define aplcZrcMaxPairIndicationWaitTime   1000   // Time in msec.

// Max time a device shall wait for a response command frame following a request command frame.
#define aplcMaxResponseWaitTime            200   // Time in msec.

// Min value of the KeyExTransferCount parameter passed to a pair request primitive during pairing.
#define aplcMinKeyExchangeTransferCount    3

// Min time a Controller must wait after pairing before attempting command discovery to the Target.
#define aplcMinTargetBlackoutPeriod        500   // Time in msec.

/**************************************************************************************************
 * ZRC specifies the following profile attributes.
 */

// Interval in msec at which user cmd repeat frames will be transmitted.
// The valid range is {0, aplcMaxKeyRepeatInterval}, default is (aplcMaxKeyRepeatInterval / 2).
// Currently, this attribute must be handled and acted upon at the ZRC Application layer.
#define aplKeyRepeatInterval               0x80

// Duration to wait after Rx of user control repeated cmd frame before terminating repeat operation.
// The valid range is >= (aplcMaxKeyRepeatInterval * 2), default is (aplcMaxKeyRepeatInterval * 2).
// Currently, this attribute must be handled and acted upon at the ZRC Application layer.
#define aplKeyRepeatWaitTime               0x81

// Value of KeyExTransferCount parameter passed to pair req primitive during DPP.
#define aplKeyExchangeTransferCount        0x82

#define DPP_DEF_KEY_TRANSFER_COUNT         36

#define RTI_ZRC_KEY_REPEAT_INTERVAL                     50 // ms

// RTI Configuration Interface Item Identifiers

// State Attributes (SA) Table Item Identifiers
#define RTI_SA_ITEM_START                                RCN_NIB_NWK_START                   // 0x60
#define RTI_SA_ITEM_STANDBY_ACTIVE_PERIOD                RCN_NIB_NWK_ACTIVE_PERIOD           // 0x60
#define RTI_SA_ITEM_CURRENT_CHANNEL                      RCN_NIB_NWK_BASE_CHANNEL            // 0x61
#define RTI_SA_ITEM_DISCOVERY_LQI_THRESHOLD              RCN_NIB_NWK_DISCOVERY_LQI_THRESHOLD // 0x62
#define RTI_SA_ITEM_DUTY_CYCLE                           RCN_NIB_NWK_DUTY_CYCLE              // 0x64
#define RTI_SA_ITEM_FRAME_COUNTER                        RCN_NIB_NWK_FRAME_COUNTER           // 0x65
#define RTI_SA_ITEM_IN_POWER_SAVE                        RCN_NIB_NWK_IN_POWER_SAVE           // 0x67
#define RTI_SA_ITEM_MAX_FIRST_ATTEMPT_CSMA_BACKOFFS      RCN_NIB_NWK_MAX_FIRST_ATTEMPT_CSMA_BACKOFFS
#define RTI_SA_ITEM_MAX_FIRST_ATTEMPT_FRAME_RETRIES      RCN_NIB_NWK_MAX_FIRST_ATTEMPT_FRAME_RETRIES
#define RTI_SA_ITEM_RESPONSE_WAIT_TIME                   RCN_NIB_NWK_RESPONSE_WAIT_TIME      // 0x6D
#define RTI_SA_ITEM_SCAN_DURATION                        RCN_NIB_NWK_SCAN_DURATION           // 0x6E
#define RTI_SA_ITEM_USER_STRING                          0x6F	// RCN_NIB_NWK_USER_STRING
#define RTI_SA_ITEM_PAN_ID                               0x85	// RCN_NIB_PAN_ID
#define RTI_SA_ITEM_SHORT_ADDRESS                        0x86	// RCN_NIB_SHORT_ADDRESS
#define RTI_SA_ITEM_AGILITY_ENABLE                       RCN_NIB_AGILITY_ENABLE              // 0x87
#define RTI_SA_ITEM_TRANSMIT_POWER                       RCN_NIB_TRANSMIT_POWER              // 0x88

// Configuration Parameters (CP) Table Item Identifiers
#define RTI_CP_ITEM_START                                0xA0
#define RTI_CP_ITEM_STARTUP_CTRL                         0xA0   // Startup Control
#define RTI_CP_ITEM_NODE_CAPABILITIES                    0xA1   // Node Capabilities
#define RTI_CP_ITEM_NODE_SUPPORTED_TGT_TYPES             0xA2   // Node Capability::Supported Target Types
#define RTI_CP_ITEM_APPL_CAPABILITIES                    0xA3   // Application Capability::User String Specified
#define RTI_CP_ITEM_APPL_DEV_TYPE_LIST                   0xA4   // Application Capability::Device Type List
#define RTI_CP_ITEM_APPL_PROFILE_ID_LIST                 0xA5   // Application Capability::Profile Id List
#define RTI_CP_ITEM_STDBY_DEFAULT_ACTIVE_PERIOD          0xA6   // Standby Information::Default Standby Active Period
#define RTI_CP_ITEM_STDBY_DEFAULT_DUTY_CYCLE             0xA7   // Standby Information::Default Standby Duty Cycle
#define RTI_CP_ITEM_VENDOR_ID                            0xA8   // Vendor Information::Vendor Id
#define RTI_CP_ITEM_VENDOR_NAME                          0xA9   // Vendor Information::Vendor Name
#define RTI_CP_ITEM_DISCOVERY_DURATION                   0xAA   // Discovery Information::Discovery Duration
#define RTI_CP_ITEM_DEFAULT_DISCOVERY_LQI_THRESHOLD      0xAB   // Discovery Information::Default Discovery LQI Threshold
#define RTI_CP_ITEM_END                                  0xAF
// Pairing Table
#define RTI_SA_ITEM_PT_NUMBER_OF_ACTIVE_ENTRIES          0xB0
#define RTI_SA_ITEM_PT_CURRENT_ENTRY_INDEX               0xB1
#define RTI_SA_ITEM_PT_CURRENT_ENTRY                     0xB2
#define RTI_SA_ITEM_PT_END                               0xB3

// Constants (CONST) Table Item Idenifiers
#define RTI_CONST_ITEM_START                             0xC0
#define RTI_CONST_ITEM_SW_VERSION                        0xC0
#define RTI_CONST_ITEM_MAX_PAIRING_TABLE_ENTRIES         0xC1
#define RTI_CONST_ITEM_NWK_PROTOCOL_IDENTIFIER           0xC2
#define RTI_CONST_ITEM_NWK_PROTOCOL_VERSION              0xC3
#define RTI_CONST_ITEM_END                               0xC4

#if defined FEATURE_OAD
#define RTI_CONST_ITEM_OAD_IMAGE_ID                      0xD0
#endif
#define RTI_CONST_ITEM_RNP_IMAGE_ID                      0xD1
#define DPP_CP_ITEM_KEY_TRANSFER_CNT                     0xD2   // TODO: actual of 0x82 conflicts.

// Constants Table Constants
#define RTI_CONST_SW_VERSION                             0x13   // V1.3
#define RTI_CONST_NWK_PROTOCOL_IDENTIFIER                0xCE   // Network layer protocol id
#define RTI_CONST_NWK_PROTOCOL_VERSION                   0x10   // V1.0

// Test Interface
#define RTI_TEST_MODE_TX_RAW_CARRIER                     0
#define RTI_TEST_MODE_TX_RANDOM_DATA                     1
#define RTI_TEST_MODE_RX_AT_FREQ                         2

// Configuration Capacity
#define RTI_MAX_NUM_SUPPORTED_TGT_TYPES                  6

// Bit masks for the Application Capabilities field (3.3.1.5, Figure 18).
#define RTI_APP_CAP_USER_STR_BM                          0x01
#define RTI_APP_CAP_NUM_DEVS_BM                          0x06
#define RTI_APP_CAP_RES_BIT3_BM                          0x08
#define RTI_APP_CAP_NUM_PROF_BM                          0x70
#define RTI_APP_CAP_RES_BIT7_BM                          0x80

// Bit masks for the Node Capabilities field (3.4.2.4, Figure 26).
#define RTI_NODE_CAP_NODE_TYPE_BM                        0x01
#define RTI_NODE_CAP_MAINS_PWR_BM                        0x02
#define RTI_NODE_CAP_SEC_CAP_BM                          0x04
#define RTI_NODE_CAP_CHAN_NORM_BM                        0x08
#define RTI_NODE_CAP_RESERVED_BM                         0xF0

/**************************************************************************************************
 * TYPEDEFS
 **************************************************************************************************/

#if !defined PACK_1
#define PACK_1
#endif

// To be compatible with MS and unix native target
// declare pragma for structure packing
#if defined(_MSC_VER) || defined(unix) || (defined(__ICC430__) && (__ICC430__==1))
#pragma pack(1)
#endif

typedef uint8 rStatus_t;

// Configuration Parameters Table types

// Startup Control
enum
{
  RESTORE_STATE,
  CLEAR_STATE,
  CLEAR_CONFIG_CLEAR_STATE
};


// Pairing table entry
PACK_1 typedef struct ATTR_PACKED
{
  uint8      pairingRef;     // pairing reference
  uint16     srcNwkAddress; // this device's own short address
  uint8      logicalChannel; // expected channel of the peer
  sAddrExt_t ieeeAddress;    // IEEE address of the peer
  uint16     panId;          // PAN identifier of the peer
  uint16     nwkAddress;   // network address of the peer
  uint8      recCapabilities; // capabilities bitmap of the target

#if RCN_FEATURE_SECURITY
  uint8      securityKeyValid; // whether the key is valid or not
  uint8      securityKey[RCN_SEC_KEY_LENGTH]; // security link key
#endif // RCN_FEATURE_SECURITY

#if RCN_FEATURE_EXTRA_PAIR_INFO
  // NOTE that RF4CE spec does not include the following as part of pairing entry
  uint16     vendorIdentifier; // vendor identifier of the target
  uint8      devTypeList[RCN_MAX_NUM_DEV_TYPES]; // list of device types supported by the peer
#endif // RCN_FEATURE_EXTRA_PAIR_INFO

  uint32     recFrameCounter;    // frame counter last received

#if RCN_FEATURE_EXTRA_PAIR_INFO
  // NOTE that RF4CE spec does not include the following as part of pairing entry
  uint8 	 profileDiscs[RCN_PROFILE_DISCS_SIZE];
#endif // RCN_FEATURE_EXTRA_PAIR_INFO

} rcnNwkPairingEntry_t;

// Controller Node Information
PACK_1 typedef struct ATTR_PACKED
{
  uint8 supportedTgtTypes[ RTI_MAX_NUM_SUPPORTED_TGT_TYPES ];
} tgtTypeInfo_s;

// Node Information
PACK_1 typedef struct ATTR_PACKED
{
  tgtTypeInfo_s supportedTgtTypes;
} nodeCap_s;

// Application Information
PACK_1 typedef struct ATTR_PACKED
{
  uint8 appCapabilities;
  uint8 devTypeList[ RTI_MAX_NUM_DEV_TYPES ];
  uint8 profileIdList[ RTI_MAX_NUM_PROFILE_IDS ];
} rtiAppCap_t;

// Standby Information
PACK_1 typedef struct ATTR_PACKED
{
  uint16 standbyActivePeriod;
  uint16 standbyDutyCycle;
} stdByInfo_s;

// Vendor Id
// <please see rti_constants.h>

// Vendor Information
PACK_1 typedef struct ATTR_PACKED
{
  uint16 vendorId;
  uint8  vendorName[ RTI_VENDOR_STRING_LENGTH ];
} vendorInfo_s;

PACK_1 typedef struct ATTR_PACKED
{
  uint16 discDuration; // in miliseconds
  uint8  discLQIThres;
} discInfo_s;

// Configuration Parameters Table
PACK_1 typedef struct ATTR_PACKED
{
  uint8        startupCtrl;                              // RTI_NVID_STARTUP_CONTROL
  nodeCap_s    nodeCap;                                  // RTI_NVID_NODE_CAP
  rtiAppCap_t  appCap;                                   // RTI_NVID_APP_CAP
  stdByInfo_s  stdByInfo;                                // RTI_NVID_STDBY_INFO
  discInfo_s   discoveryInfo;                            // RTI_NVID_DISCOV_INFO
  uint8        nodeCapabilities;
  vendorInfo_s vendorInfo;
} configParams_s;

// State Attributes Table
PACK_1 typedef struct ATTR_PACKED
{
  uint8  curPairTableIndex;
} stateAttribs_s;

// End the packing rule
#ifdef _MSC_VER
#pragma pack()
#endif


// function pointer for RCN event callback function
typedef void (*rtiRcnCbackFn_t)( void *pData );

/**************************************************************************************************
 * GLOBALS
 **************************************************************************************************/

extern uint8 RTI_TaskId;                 // Task ID
extern configParams_s configParamTable;  // Configuration Parameter Table

/*********************************************************************
 * FUNCTIONS
 */

/* These functions are used when creating the OSAL RTI task.  They must not be used for any
 * other purpose.
 */
extern void RTI_Init( uint8 task_id );
extern uint16 RTI_ProcessEvent( uint8 task_id, uint16 events );

// RF4CE Network Processor RemoTI (RTI) API

// Configuration Interface
// Used to access Configuration Parameters, State Attributes, and Constants
extern RTILIB_API rStatus_t RTI_ReadItemEx(uint8 profileId, uint8 itemId, uint8 len, uint8 *pValue);
extern RTILIB_API rStatus_t RTI_WriteItemEx(uint8 profileId, uint8 itemId, uint8 len, uint8 *pValue);
extern RTILIB_API rStatus_t RTI_ReadItem(uint8 itemId, uint8 len, uint8 *pValue);
extern RTILIB_API rStatus_t RTI_WriteItem(uint8 itemId, uint8 len, uint8 *pValue);

// Application Profile Interface
// Used to access RF4CE application profile
extern RTILIB_API void RTI_InitReq( void );
extern RTILIB_API void RTI_PairReq( void );
extern RTILIB_API void RTI_PairAbortReq( void );
extern RTILIB_API void RTI_UnpairReq( uint8 dstIndex );
extern RTILIB_API void RTI_AllowPairReq( void );
extern RTILIB_API void RTI_AllowPairAbortReq( void );
extern RTILIB_API void RTI_SendDataReq( uint8 dstIndex, uint8 profileId, uint16 vendorId, uint8 txOptions, uint8 len, uint8 *pData );
extern RTILIB_API void RTI_StandbyReq( uint8 mode );
extern RTILIB_API void RTI_RxEnableReq( uint16 duration );
extern RTILIB_API void RTI_EnableSleepReq( void );
extern RTILIB_API void RTI_DisableSleepReq( void );
extern RTILIB_API void RTI_EnterBootModeReq( void );

// RTI Callbacks
extern void RTI_InitCnf( rStatus_t status );
extern void RTI_PairCnf( rStatus_t status, uint8 dstIndex, uint8 devType );
extern void RTI_PairAbortCnf( rStatus_t status );
extern void RTI_UnpairCnf( rStatus_t status, uint8 dstIndex );
extern void RTI_UnpairInd( uint8 dstIndex );
extern void RTI_AllowPairCnf( rStatus_t status, uint8 dstIndex, uint8 devType );
extern void RTI_SendDataCnf( rStatus_t status );
extern void RTI_StandbyCnf( rStatus_t status );
extern void RTI_ReceiveDataInd( uint8 srcIndex, uint8 profileId, uint16 vendorId, uint8 rxLQI, uint8 rxFlags, uint8 len, uint8 *pData );
extern void RTI_RxEnableCnf( rStatus_t status );
extern void RTI_EnableSleepCnf( rStatus_t status );
extern void RTI_DisableSleepCnf( rStatus_t status );
extern void RTI_ResetInd( void );

// The following function is used by a module within radio processor.
// The functionsi not intended for use by application in host processor.
extern void RTI_SetBridgeMode(rtiRcnCbackFn_t pCback);


// It is better to compile flag RTI surrogate specific APIs
// with "RNP_HOST" as eventually there could be an SOC built on
// high level OS such as linux.
// For now, to maintain backward compatibility, no such compile
// flag is used.
#ifdef _WIN32
// This function is for windows platform only
 extern uint8 RTI_InitWin32Module(const char *pDevName);
 extern uint8 RTI_CloseWin32Module(void);
# define RTI_InitRNP(_pDevName) RTI_InitWin32Module(_pDevName)
# define RTI_CloseRNP() RTI_CloseWin32Module()
#elif defined unix
 extern int RTIS_Init(const char *pDevName);
 extern void RTIS_Close(void);
# define RTI_InitRNP(_pDevName) RTIS_Init(_pDevName)
# define RTI_CloseRNP() RTIS_Close()
#else
 extern void RTIS_Init( void );
#endif

// Test Interface
// Used to access test modes in the RemoTI stack.
extern RTILIB_API void RTI_SwResetReq( void );
extern RTILIB_API void RTI_TestModeReq( uint8 mode, int8 txPower, uint8 channel );
extern RTILIB_API uint16 RTI_TestRxCounterGetReq(uint8 resetFlag);

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* RTI_LNX_H */
