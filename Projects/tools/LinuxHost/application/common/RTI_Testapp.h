/**************************************************************************************************
  Filename:       RTI_Testapp.h
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

#ifndef RTI_TEST_APP_H
#define RTI_TEST_APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "rti_lnx.h"

#if defined(_MSC_VER) || defined(unix) || (defined(__ICC430__) && (__ICC430__==1))
#pragma pack(1)
#endif

//Define:
// Application Test Mode States
enum {
	APP_TEST_MODE_STATE_INIT,					// Initial state. Ready to configure the test mode settings
	APP_TEST_MODE_STATE_CFG_DEST_IDX,			// Sub state for configuration, choice of destination index
	APP_TEST_MODE_STATE_CFG_START_CONDITION,	// Sub state for configuration, choice of start condition
	APP_TEST_MODE_STATE_CFG_TX_POWER,			// Sub state for configuration, choice of Tx Power
	APP_TEST_MODE_STATE_CFG_TX_OPTIONS,			// Sub state for configuration, choice of Tx options
	APP_TEST_MODE_STATE_CFG_PAYLOAD,			// Sub state for configuration, choice of payload size
	APP_TEST_MODE_STATE_CFG_MAX_DELAY,			// Sub state for configuration, choice of max random delay between packets
	APP_TEST_MODE_STATE_CFG_NUM_OF_PACKETS,		// Sub state for configuration, choice of number of packets
	APP_TEST_MODE_STATE_SET_NUM_OF_RUNS,		// Sub state to set the number of automatic runs
//	APP_TEST_MODE_STATE_SET_VERSION,			// Set version of Test Mode. In order to be backwardscompatible
	APP_TEST_MODE_STATE_LAUNCH,					// Launch test for a chosen index
	APP_TEST_MODE_STATE_RUNNING,				// Test is now running, wait for report, or timeout, or if Controller send data
	APP_TEST_MODE_STATE_REPORT,					// Controller is now trying to send the report, try until successful confirmation
	APP_TEST_MODE_STATE_TX_OPTIONS,				// Test Mode Tx options
};

// Application Test Mode States
enum {
	APP_TEST_PAIR_STATE_INIT,					// Initial state. Ready to configure the test mode settings
	APP_TEST_PAIR_STATE_RUN,					// Running pairing test
};

enum
{
  RSA_TEST_TERMINATE,
  RSA_TEST_LATENCY,
  RSA_TEST_PER
};

enum
{
  RSA_TEST_START_IMMEDIATE,
  RSA_TEST_START_ON_BUTTON
};

enum
{
	RSA_TEST_MODE_VERSION_UNTIL_1_3_1,
	RSA_TEST_MODE_VERSION_1_3_1,
	RSA_TEST_MODE_VERSION_END
};

// Test request command
// Test parameters structure should be compliant with the TI vendor specific command format.
typedef struct
{
  uint8 startCondition;
  uint8 testType;
  uint8 txOptions;
  uint8 userDataSize; // user data is one byte plus this size as one byte should be used
                      // to array test command identifier
  uint16 numPackets;
  uint16 maxBackoffDuration; // max delay between each packet in ms
  int8 txPower;	// Tx power as signed byte

} rrtTestReq_t;

PACK_1 typedef struct ATTR_PACKED
{
	uint8 currentIdx;
	uint8 idxArr[RCN_CAP_PAIR_TABLE_SIZE];
	uint8 size;
} testModeDest_t;

typedef struct
{
  uint8 testType;
  struct
  {
    uint16 numPackets;
    uint16 bin[29];
  } latency;
} rrtTestReportRsp_t;

//External Global Variable
//extern uint8 RTI_app_threadId;
//extern appDevInfo_t appCFGParam;
//extern uint8 appState;
//extern uint8 appSendDataState;
//extern uint8 appTestDataPending;
//extern appSendData_t appSendData_s;
//extern char ** const testMode_txPower_list;

//Global share variable
//extern uint8 appTestPairState;
//extern testModeDest_t testModeDest;

#define NAME_ELEMENT(element) [element] = #element

typedef void (*rtiReturnFromTestmodeModuleCbackFn_t)( void );
/////////////////////////////////////////////////////////////////////////////
// Function declarations

void RTI_TestModeInit(rtiReturnFromTestmodeModuleCbackFn_t cback);
void RTI_EnterTestMode(void);
void appTestModeProcessKey(char* strIn);
void RTI_TestModeReceiveDataInd( uint8 srcIndex, uint8 profileId, uint16 vendorId, uint8 rxLQI, uint8 rxFlags, uint8 len, uint8 *pData);
void RTI_TestModePairCnf(rStatus_t status, uint8 dstIndex, uint8 devType);
uint8 RTI_TestModeSendDataCnf_Ndata(rStatus_t status);
void RTI_TestModeSendDataCnf(rStatus_t status);
void testappProcessEvent( uint16 events);
void testappStartPairingtest (void );
void RTI_TestModeAllowPairCnf(rStatus_t status, uint8 dstIndex, uint8 devType);

#ifdef __cplusplus
}
#endif

#endif // RTI_TEST_APP_H
