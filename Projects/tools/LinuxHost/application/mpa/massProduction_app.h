/**************************************************************************************************
  Filename:       MASSPRODUCTION_app.h
  Revised:        $Date: 2012-03-21 17:37:33 -0700 (Wed, 21 Mar 2012) $
  Revision:       $Revision: 246 $

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

#ifndef MASSPRODUCTION_APP_H
#define MASSPRODUCTION_APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "rti_lnx.h"

  /////////////////////////////////////////////////////////////////////////////
  // Defines

// MASSPRODUCTION_app Events
#define MASSPRODUCTION_APP_EVT_NONE                   			0x0000 // No event
#define MASSPRODUCTION_APP_EVT_INIT                   			0x0001 // Initial Boot up Event
#define MASSPRODUCTION_APP_EVT_DATA_SEND               			0x0002 // Data Send Event
#define MASSPRODUCTION_APP_EVT_RESPONSE_TIMEOUT       			0x0004 // Wait For Response Timeout Event
#define MASSPRODUCTION_APP_EVT_EXIT               				0x0008 // Exit Event

#define MASSPRODUCTION_APP_WAIT_FOR_RESPONSE_TIMEOUT			1000	// ms to wait for response
#define MASSPRODUCTION_APP_TIMEOUT_MARGIN						50		// 50 ms implementation margin

#define NAME_ELEMENT(element) [element] = #element

// Application States
enum {
	AP_STATE_INIT, // Initial state - prior to initialization completion
	AP_STATE_INIT_COLD, // Initial state - forced cold reset
	AP_STATE_READY, // Ready state - ready to pair, ready to send data
	AP_STATE_UNPAIR, // Unpairing in progress state
	AP_STATE_NDATA, // Send-data in progress state
	AP_STATE_WAIT_FOR_RESPONSE, // Send-data in progress state
	AP_STATE_NDATA_PREPARE, // State to prepare for sending data; next state is normally AP_STATE_NDATA
};

extern const char * const AppState_list[AP_STATE_NDATA_PREPARE + 1];//18 Application States


// Send Data States
enum {
	APP_SEND_DATA_STATE_INIT,		// Initial Application Sub State for Configuration of data to send
	APP_SEND_DATA_STATE_DEST_IDX,	// Sub state for configuration of destination index
	APP_SEND_DATA_STATE_PROFILE,	// Sub state for configuration of profile
	APP_SEND_DATA_STATE_TX_OPTIONS,	// Sub state for configuration of Tx options
	APP_SEND_DATA_STATE_PAYLOAD,	// Sub state for configuration of payload
	APP_SEND_DATA_STATE_KEYBOARD,	// Sub state for sending ZID Keyboard commands directly
	APP_SEND_DATA_STATE_MOUSE,		// Sub state for sending ZID Mouse commands directly
	APP_SEND_DATA_STATE_MOUSE_TEST,	// Sub state for sending ZID Mouse commands directly
	APP_SEND_DATA_STATE_PROFILE_ID
};

  /////////////////////////////////////////////////////////////////////////////
  // Typedefs

// Configuration specific

PACK_1 typedef struct ATTR_PACKED {
	  uint8 nodeCapabilities;
	  uint16 vendorId;
	  uint8 vendorString[RCN_VENDOR_STRING_LENGTH];
	  uint8 appCapabilities;
	  uint8 userString[RCN_USER_STRING_LENGTH];
	  uint8 devTypeList[RCN_MAX_NUM_DEV_TYPES];
	  uint8 profileIdList[RCN_MAX_NUM_PROFILE_IDS];
	  uint8 tgtTypeList[RTI_MAX_NUM_SUPPORTED_TGT_TYPES];
} appDevInfo_t;

// A packed structure that constitutes the data of the ZID_STD_REPORT_MOUSE report.
// This is taken from ZID
typedef struct {
  uint8 btns;  // Bits 0-2 : Left/Right/Center click; bits 3-7 : pad.
  uint8 x;     // -127 to +127 relative X movement.
  uint8 y;     // -127 to +127 relative Y movement.
} app_zid_mouse_data_t;
#define APP_ZID_MOUSE_DATA_LEN                (sizeof(app_zid_mouse_data_t))
#define APP_ZID_MOUSE_DATA_MAX                 127

PACK_1 typedef struct ATTR_PACKED
{
	uint8 dstIndex;
	uint8 profileId;
	uint16 vendorId;
	uint8 txOptions;
	uint8 len;
	uint8 pData[90];
} appSendData_t;

PACK_1 typedef struct ATTR_PACKED
{
	uint8 cycleIdx;
	uint8 superCycleCnt;
	app_zid_mouse_data_t mouseData[4];
} appSendMouse_t;

  /////////////////////////////////////////////////////////////////////////////
  // Function declarations

extern int appInit( int mode, char threadId );


#ifdef __cplusplus
}
#endif

#endif // MASSPRODUCTION_APP_H
