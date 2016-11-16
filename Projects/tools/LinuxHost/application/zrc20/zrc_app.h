   /**************************************************************************************************
  Filename:       zrc_app.h

  Description:    This file defines linux specific interface to Network Processor Interface
                  module.


  Copyright (C) 2015 Texas Instruments Incorporated - http://www.ti.com/


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

#ifndef ZRC_APP_H
#define ZRC_APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "hal_types.h"
#include "rti_lnx.h"
#include "zrc_profile.h"

//#if defined(_MSC_VER) || defined(unix) || (defined(__ICC430__) && (__ICC430__==1))
//#pragma pack(1)
//#endif

#if !defined PACK_1
#define PACK_1
#endif


#if defined(_MSC_VER) || defined(unix) || (defined(__ICC430__) && (__ICC430__==1))
#pragma pack(1)
#endif

#ifndef ATTR_PACKED
# ifdef __GNUC__
#  define ATTR_PACKED __attribute__ ((__packed__))
# else
#  define ATTR_PACKED
# endif
#endif

#pragma pack(1)

/////////////////////////////////////////////////////////////////////////////
// Defines

// RTI_app Events
#define ZRC_APP_EVT_NONE                               0x00000000 // No event
#define ZRC_APP_EVT_INIT                               0x00000001 // Initial Boot up Event
#define ZRC_APP_EVT_DATA_RCV                           0x00000002 // Data Receive Event
#define ZRC_APP_EVT_OAD_KO                          0x00000004 // OAD unsuccessful
#define ZRC_APP_EVT_OAD_OK                          0x00000008 // OAD successful
#define ZRC_APP_EVT_OAD_REBOOT                      0x00000010 // OAD rebooting
#define ZRC_APP_EVT_OAD_TIMEOUT                     0x00000020 // OAD timed out
#define ZRC_APP_EVT_TOAD_SAFE_NORMAL_TOAD_WINDOW    0x00000040 // TOAD Safe Normal Operation Window expired
#define ZRC_APP_EVT_TEST_RECEIVE_TIMEOUT            0x00000080 // It's been more than 100 ms since we last received a message. This is unusual. Stop the synchronous request test.
#define ZRC_APP_EVT_TEST_RX_ENABLE_TIMEOUT            0x00000100 // Rx is no longer enabled. Return to Idle test state.
#define ZRC_APP_EVT_STRESS_TEST_SYNCH_REQ            0x00000200 // During heavy traffic try to randomly read from RNP, hence testing SREQ
#define ZRC_APP_EVT_PAIRING_TEST                    0x00000400 // Pairing test
#define ZRC_APP_EVT_RESET                            0x00008000 // Reset event
#define ZRC_APP_EVT_STORE_TO_NV                        0x00010000 // Event to postpone NV storage
#define ZRC_APP_EVT_SEND_VOICE_EVENTS                0x00080000

#define ZRC_APP_EVT_SEND_SET_GET_ATTR_RESPONSE        0x00100000 // Event to send Set/Get Attributes Response

#define ZRC_APP_EVT_KEY_RELEASE_LOST_MASK            0xFF000000 // The upper 8 events are reserved for tracking lost key presses
#define ZRC_APP_EVT_KEY_RELEASE_LOST(key)            (1 << (24 + key))

#define ZRC_APP_SEND_GET_ATTR_RESPONSE_WAIT_TIME              1 // Set to 1ms because of delays in serial communication. Wait 20ms before transmitting the response
#define ZRC_APP_SEND_SET_ATTR_RESPONSE_WAIT_TIME            20 // Wait 20ms before transmitting the response
#define ZRC_APP_SEND_RESPONSE_IDLE_TIME                         10 // Set to 10ms because of delays in serial communication. Wait 50ms before transmitting the response
#define ZRC_APP_STORE_TO_NV_TIMEOUT_SLOW                  5000 // Wait 5 seconds after a keypress before updating NV
#define ZRC_APP_STORE_TO_NV_TIMEOUT_FAST            (3 * aplcMaxResponseWaitTime) // Wait after receiving a Set Attributes Request before updating NV. This way a bundle of updates can be grouped together
#define ZRC_APP_KEY_RELEASE_LOST_TIMEOUT            (aplcMaxKeyRepeatInterval * 2)
// Value of ZRC_APP_FREE_DISC_USER_STRING_LL_TIMEOUT must be higher than
// ZRC_ITEM_VALIDATION_INITIAL_WATCHDOG_TIME + (ZRC_ITEM_VALIDATION_SUBSEQUENT_WATCHDOG_TIME * zrcAppNumFailedValidationAttemptsMax) + pairing margin (~2 seconds)
// Default values would mean 8s + 5s * 10 + 2 = 60 seconds.
// TODO: Check if this can be replaced by input to ZRC_AppSetValidationTimeout(). I may have misunderstood this API from the RDK.
#define ZRC_APP_FREE_DISC_USER_STRING_LL_TIMEOUT    120000  // 120000ms --> 2 minutes.

#define NAME_ELEMENT(element) [element] = #element

// Application States
enum {
    AP_STATE_INIT, // Initial state - prior to initialization completion
    AP_STATE_INIT_COLD,
    AP_STATE_RESET, // Initial state - However, RNP is in Serial Boot mode.
    AP_STATE_PAIR,
    AP_STATE_VALIDATION,
    AP_STATE_LATENCY_TEST_MODE,
    AP_STATE_UNPAIR,
    AP_STATE_NDATA,
    AP_STATE_NDATA_PREPARE,
    AP_STATE_WAIT_WAKEUP_FOR_PAIR,
    AP_STATE_WAIT_WAKEUP_FOR_UNPAIR,
    AP_STATE_WAIT_WAKEUP_FOR_NDATA,
    AP_STATE_READY, // Ready state - ready to pair, ready to send data
};

// Send Data States
enum {
    APP_SEND_DATA_STATE_INIT,        // Initial Application Sub State for Configuration of data to send
    APP_SEND_DATA_STATE_PAYLOAD,    // Sub state for configuration of payload
    APP_SEND_DATA_STATE_RATE,    // Sub state for sending ZID Keyboard commands directly
    APP_SEND_DATA_STATE_LATENCY,        // Sub state for sending ZID Mouse commands directly
    APP_SEND_DATA_STATE_THROUGHPUT,    // Sub state for sending ZID Mouse commands directly
    APP_SEND_DATA_STATE_PACKET,    // Sub state for sending ZID Keyboard commands directly

};

/**********************************************************************************
 * Common defines
 */
#define MAX_NUM_OF_PAIRING_ENTRIES_LINUX_SIDE            10
#define ZRC_MAX_BOUND_REMOTES                             8

/**********************************************************************************
 * Application defines
 */
// RNP sleep mode have two dependencies; Rx and NPI
#define ZRC_APP_RNP_POWER_STATE_STANDBY_BIT            BV(0)
#define ZRC_APP_RNP_POWER_STATE_NPI_BIT                BV(1)
// RNP sleep modes in order of current consumption
#define ZRC_APP_RNP_POWER_STATE_ACTIVE                0
#define ZRC_APP_RNP_POWER_STATE_NPI_SLEEP            (ZRC_APP_RNP_POWER_STATE_NPI_BIT)
#define ZRC_APP_RNP_POWER_STATE_STANDBY_ACTIVE        (ZRC_APP_RNP_POWER_STATE_STANDBY_BIT)
#define ZRC_APP_RNP_POWER_STATE_STANDBY_SLEEP        (ZRC_APP_RNP_POWER_STATE_STANDBY_BIT | ZRC_APP_RNP_POWER_STATE_NPI_BIT)

#define ZRC_APP_LQI_TO_DBM_CONVERSION(LQI)            ((LQI - 0xFF) >> 1)

// ZRC variables
#define ZRC_READY               0
#define ZRC_VALIDATION          1

#define ZRC_GOOD                1 //TRUE
#define ZRC_WRONG               2 // ABORT
#define ZRC_INCOMPLETE          0 //FALSE

#define ZRC_VALIDATION_CODE_SIZE            3

/* Binding Parameter Bytes*/
#define ZRC_BASIC_LQI_DEFAULT        0x00        // Corresponds to -128dBm
#define ZRC_STRICT_LQI_DEFAULT       0x00        // Corresponds to -128dBm
/* Binding Parameter bits*/
#define ZRC_PROFILE_BIND_WEIGHTS_PRE_COM_IEEE         0x01
#define ZRC_PROFILE_BIND_WEIGHTS_PDSV                 0x02 // Pairing Description Screen Visible
#define ZRC_PROFILE_BIND_WEIGHTS_LOS                  0x04 // Line Of Sight
#define ZRC_PROFILE_BIND_WEIGHTS_EPT                  0x08 // Empty Pairing Table
#define ZRC_PROFILE_BIND_WEIGHTS_JB                   0x10 // Just Booted
#define ZRC_PROFILE_BIND_WEIGHTS_BP                   0x20 // Button Binding

/* Binding Parameter Bytes*/
#define ZRC_BASIC_LQI_DEFAULT        0x00        // Corresponds to -128dBm
#define ZRC_STRICT_LQI_DEFAULT       0x00        // Corresponds to -128dBm

  /////////////////////////////////////////////////////////////////////////////
  // Typedefs

typedef struct
{
    const char * str;
    uint8 byteCount;
} stringList_t;

// Response buffer
typedef struct ATTR_PACKED
{
    uint8 destIndex;
    uint16 vendorId;
    uint8 profileId;
    uint8 txOptions;
    uint8 msgLen;
    uint8 replyBuf[256];
} response_t;

// Configuration specific

typedef struct ATTR_PACKED
{
      uint8 nodeCapabilities;
      uint16 vendorId;
      uint8 vendorString[RCN_VENDOR_STRING_LENGTH];
      uint8 appCapabilities;
      uint8 userString[RCN_USER_STRING_LENGTH];
      uint8 devTypeList[RCN_MAX_NUM_DEV_TYPES];
      uint8 profileIdList[RCN_MAX_NUM_PROFILE_IDS];
      uint8 tgtTypeList[RTI_MAX_NUM_SUPPORTED_TGT_TYPES];
} appDevInfo_t;

// Configuration specific
typedef struct ATTR_PACKED
{
    uint8 pairType;

} zrc_ctl_cfg_t;

typedef struct ATTR_PACKED
{
    uint8 pairType;

} zrc_tgt_cfg_t;

typedef struct ATTR_PACKED
{
    uint8  length;
    uint16 rate;
    uint32 nbPackets;
} appSendData_t;

typedef struct ZRC_RfChannel
{
    unsigned char           Number;
    unsigned char           Pollution;
} ZRC_RfChannel_t;

/////////////////////////////////////////////////////////////////////////////
// Global variable
extern uint8              zrcAppState; // Public so OEM extensions can access the state.
extern appSendData_t      zrcAppSendData;
extern uint8              appState;
extern const char * const AppState_list[AP_STATE_READY + 1];//18 Application States

/////////////////////////////////////////////////////////////////////////////
// Function declarations

extern int ZRC_AppInit( int mode, char threadId );
extern void ZRCApp_start_timerEx(uint32 event, uint32 timeout);
extern void zrc_appTestModeProcessKey (char* strIn);

typedef void (*zrcAppRtiDataCnfCbackFn_t)( rStatus_t status );
uint8 ZRCApp_SendDataReq( uint8 dstIndex, uint8 profileId, uint16 vendorId, uint8 txOptions, uint8 len, uint8 *pData, zrcAppRtiDataCnfCbackFn_t cBack );

#define NAME_ELEMENT(element) [element] = #element

#ifdef __cplusplus
}
#endif

#endif // MAC_APP_H
