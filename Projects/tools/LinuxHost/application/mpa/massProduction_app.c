/**************************************************************************************************
 Filename:       massProduction_app.c

 Description:    Linux Host application

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

#include <stdio.h>
#include <stddef.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <getopt.h>
#include <pthread.h>
#include <poll.h>

#include <sys/time.h>

#include "common_app.h"

#include "massProduction_main.h"
#include "massProduction_app.h"
#include "timer.h"
#include "tiLogging.h"
#include "configParser.h"

// Linux surrogate interface, including both RTIS and NPI
#include "npi_ipc_client.h"
#include "npi_lnx_error.h"

#include "hal_rpc.h"
#include "npi_lnx_ipc_rpc.h"

#define SB_DST_ADDR_DIV                    4

// macros
#define PAUSE() { fflush(stdout); while (getchar()!='\n'); }

// Application state variable
uint8 appState;
uint8 supportIndexedAPI = FALSE;

struct timeval curTime, startTime, prevTimeSend, prevTimeRec, prevTime;

#pragma pack(1)
typedef struct ATTR_PACKED
{
	char averageRSSI;
	int averageFREQEST;
	uint8 numOfReceived;
} testReport_t;

// Pairing reference
uint8 destIdx;

appBaseSetting_s baseSettings;
#define NUM_OF_FAKE_P_ENTRIES				3
rcnNwkPairingEntry_t fakePentryTable[NUM_OF_FAKE_P_ENTRIES];
rcnNwkPairingEntry_t originalPairingTable[NUM_OF_FAKE_P_ENTRIES];

// ch from console
char ch;
// string from console
char str[128];

// Toggle Timer Print on Server state variable
static uint8 toggleTimerPrintOnServer = FALSE;
static uint8 toggleBigDebugPrintOnServer = FALSE;

//THREAD and MUTEX
static pthread_mutex_t appThreadMutex;
static pthread_mutex_t appInitMutex;

static pthread_t AppThreadId;
static void *appThreadFunc(void *ptr);
int appThreadTerminate;

static void appProcessEvents(uint32 events);

uint8 MPA_App_threadId;

#define NAME_ELEMENT(element) [element] = #element

const char * const AppState_list[AP_STATE_NDATA_PREPARE + 1] = { // Application States
		[0 ... AP_STATE_NDATA_PREPARE] = NULL, NAME_ELEMENT(AP_STATE_INIT),
		NAME_ELEMENT(AP_STATE_INIT_COLD),
		NAME_ELEMENT(AP_STATE_READY),
		NAME_ELEMENT(AP_STATE_NDATA),
		NAME_ELEMENT(AP_STATE_NDATA_PREPARE)
};

static uint8 vendorName[RTI_VENDOR_STRING_LENGTH] = "TI-LPRF";

appDevInfo_t appCFGParam;
struct
{
	int mpTestStationNumber;
	int mpExpectedNumberOfMessages;
	int mpCurrentMessageCount;
	int8 mpTxPower;
} mpCFG = {0};
uint8 appCFGstate;

appSendData_t appSendData_s;
uint8 appSendDataState;

static void appSetCFGParamOnRNP(void);
static int appVerifyCFGParamAgainstRNP(void);
static int appInitConfigParam( );
static void appInitSyncRes(void);
int appConfigureMassProductionTable(void);
int appRestorePairingTable(void);
int appReadPairingEntry(rcnNwkPairingEntry_t *pEntry, uint8 index);
void appDisplayPairingTable();
int appGetAndPrintExtendedSoftwareVersion(swVerExtended_t *swVerExtended);
static int appGetAndPrintSoftwareVersions(uint8 *baseVersion, swVerExtended_t *swVerExtended);

void RTI_IrInd( uint8 irData ) {}

int appInit(int mode, char threadId)
{
	appThreadTerminate = 0;
	appInitSyncRes();

	MPA_App_threadId = threadId;

	baseSettings.ip_addr = NULL;
	baseSettings.port = NULL;
	baseSettings.freqAgilityEnabled = TRUE;

	if (NPI_LNX_SUCCESS == ConfigParserGetBaseSettings(&baseSettings))
	{
		// Set debug option
		toggleBigDebugPrintOnServer = baseSettings.debug.big;
		LOG_DEBUG("[CFG] toggleBigDebugPrintOnServer = %d\n", toggleBigDebugPrintOnServer);
		// Set debug option
		toggleTimerPrintOnServer = baseSettings.debug.timer;
		LOG_DEBUG("[CFG] toggleTimerPrintOnServer = %d\n", toggleTimerPrintOnServer);
		// Set debug option
		__APP_LOG_LEVEL = baseSettings.debug.app;
		LOG_DEBUG("[CFG] __DEBUG_APP_ACTIVE = %d\n", __APP_LOG_LEVEL);
//		// Set debug option
//		__DEBUG_CLIENT_ACTIVE = baseSettings.debug.client;
//		LOG_DEBUG("[CFG] __DEBUG_CLIENT_ACTIVE = %d\n", __DEBUG_CLIENT_ACTIVE);
	}
	else
	{
		// Turn big debug OFF
		toggleBigDebugPrintOnServer = FALSE;
		// Turn timer debug OFF
		toggleTimerPrintOnServer = FALSE;
	}
	LOG_DEBUG("\n-------------------- START TURN ON DEBUG TRACES -------------------\n");
	npiMsgData_t pMsg;
	pMsg.len = 1;
	pMsg.subSys = RPC_SYS_SRV_CTRL | RPC_CMD_SREQ;
	pMsg.cmdId = NPI_LNX_CMD_ID_CTRL_BIG_DEBUG_PRINT_REQ;

	pMsg.pData[0] = toggleBigDebugPrintOnServer;

	// send debug flag value
	NPI_SendSynchData( &pMsg );
	if (RTI_SUCCESS == pMsg.pData[0])
	{
		LOG_DEBUG("__BIG_DEBUG_ACTIVE set to: 0x%.2X\n", toggleBigDebugPrintOnServer);
	}

	pMsg.cmdId = NPI_LNX_CMD_ID_CTRL_TIME_PRINT_REQ;

	pMsg.pData[0] = toggleTimerPrintOnServer;

	// send debug flag value
	NPI_SendSynchData( &pMsg );
	if (RTI_SUCCESS == pMsg.pData[0])
	{
		LOG_DEBUG("__DEBUG_TIME_ACTIVE set to: 0x%.2X\n", toggleTimerPrintOnServer);
	}
	LOG_DEBUG("\n-------------------- END TURN ON DEBUG TRACES -------------------\n\n");

  LOG_INFO("[Initialization]-------------------- START SOFTWARE VERSION READING-------------------\n");

	uint8           rnpSwVerBase;
	swVerExtended_t rnpSwVerExtended;
	appGetAndPrintSoftwareVersions(&rnpSwVerBase, &rnpSwVerExtended);

	// Check if software version is greater than or equal to RemoTI-1.4.0
	if ((rnpSwVerExtended.major > 1) ||
	    ((rnpSwVerExtended.major == 1) && (rnpSwVerExtended.minor >= 4)))
	{
	  supportIndexedAPI = TRUE;
	}
	LOG_INFO("supportIndexedAPI == %s major-%d minor-%d\n",
		(supportIndexedAPI == TRUE) ? "TRUE" : "FALSE", rnpSwVerExtended.major, rnpSwVerExtended.minor);

	LOG_INFO("-------------------- END SOFTWARE VERSION READING-------------------\n");

	// Setup default configuration
	appInitConfigParam();

	// Configure fake pairing table
	appConfigureMassProductionTable();

	// TODO: it is ideal to make this thread higher priority
	// but linux does not allow realtime of FIFO scheduling policy for
	// non-priviledged threads.
	if (pthread_create(&AppThreadId, NULL, appThreadFunc, NULL))
	{
		// thread creation failed
		LOG_INFO("Failed to create app thread\n");
		return -1;
	}

	return 0;
}

static void appInitSyncRes(void)
{
	// initialize all mutexes
	if (pthread_mutex_init(&appThreadMutex, NULL))
	{
		fprintf(stderr, "Fail To Initialize Mutex appThreadMutex\n");
		exit(-1);
	}

	if (pthread_mutex_init(&appInitMutex, NULL))
	{
		fprintf(stderr, "Fail To Initialize Mutex appInitMutex\n");
		exit(-1);
	}
}

static void *appThreadFunc(void *ptr)
{
	//uint8 readbuf[128];
	//uint8 pollStatus = FALSE;
	uint16 events = 0;
	int mutexRet;

	/* lock mutex in order not to lose signal */
	pthread_mutex_lock(&appThreadMutex);

	// set sample application state to not ready until RTI Init is successfully confirmed
	appState = AP_STATE_INIT;

	// Apply configuration
	appSetCFGParamOnRNP();

	LOG_DEBUG("App Thread Started \n");

	// Initialize node and RF4CE stack
	LOG_INFO("Calling RTI_InitReq...\n");
	//When Launching the Thread, Mutex is unlocked.
	LOG_TRACE("Lock appInit Mutex\n");
	if ( (mutexRet = pthread_mutex_trylock(&appInitMutex)) == EBUSY)
	{
		LOG_TRACE("appInit Mutex busy\n");
	}
	else
	{
		LOG_TRACE("appInit Lock status: %d\n", mutexRet);
	}
	RTI_InitReq();
	LOG_DEBUG("...Waiting for RTI_InitCnf. (can take up to 6s if cold start and target RNP)...\n");

	/* thread loop */
	while (!appThreadTerminate)
	{
		// Wait for event
		sem_wait(&eventSem);

		events = timer_get_event(MPA_App_threadId);
		// Process events
		if (events != 0) {
			appProcessEvents(events);
			LOG_DEBUG("State: %s [0x%.2X]\n", AppState_list[appState], appState);
		}

		// Only process actions if there is a character available
		if (consoleInput.handle == RTI_MAIN_INPUT_READY)
		{
			ch = consoleInput.latestCh;
			strcpy(str, consoleInput.latestStr);

			// 'q' requires special attention, hence it is not part of if/else
			if (ch == 'q')
			{
				//Terminate Thread
				appThreadTerminate = 1;
				//Terminate Thread and exit everything
			}

			// Release handle at the end to indicate to main thread that character is processed
			consoleInput.handle = RTI_MAIN_INPUT_RELEASED;
		}
	}

	pthread_mutex_unlock(&appThreadMutex);

  return NULL;
}


/**************************************************************************************************
 *
 * @fn      appProcessEvents
 *
 * @brief   Process events function
 *
 * @param   events - 16 bit mask
 *
 * @return  void
 */
static void appProcessEvents(uint32 events)
{
	uint16 procEvents = 0;
	int mutexRet = 0;

	if ((events & MASSPRODUCTION_APP_EVT_DATA_SEND) != 0)
	{
		if (appState == AP_STATE_READY)
		{
			appState = AP_STATE_NDATA;
			LOG_DEBUG("State: %s [0x%.2X]\n", AppState_list[appState], appState);

			if (mpCFG.mpExpectedNumberOfMessages != 0)
			{
				LOG_INFO("Starting Test for %d messages\n", mpCFG.mpExpectedNumberOfMessages);
				if (appSendData_s.pData[RTI_TEST_PDATA_IDX_PROTOCOL_CMD] == RTI_CMD_TEST_PRODUCTION_REQUEST)
				{
					// Enable receiver now so it's on in time for the response.
					RTI_RxEnableReq(MASSPRODUCTION_APP_WAIT_FOR_RESPONSE_TIMEOUT + MASSPRODUCTION_APP_TIMEOUT_MARGIN);
				}
				// Send message
				RTI_SendDataReq(appSendData_s.dstIndex, appSendData_s.profileId,
						appSendData_s.vendorId, appSendData_s.txOptions, appSendData_s.len,
						(uint8*)&(appSendData_s.pData[0]));
			}
			else
			{
				LOG_DEBUG("Will not start test for %d messages\n", mpCFG.mpExpectedNumberOfMessages);
			}
		}
		else if (appState == AP_STATE_NDATA_PREPARE)
		{
			appState = AP_STATE_NDATA;
			LOG_DEBUG("Sending message %d of %d\n", mpCFG.mpCurrentMessageCount, mpCFG.mpExpectedNumberOfMessages);
			// Continue transmission
			if (mpCFG.mpCurrentMessageCount < mpCFG.mpExpectedNumberOfMessages)
			{
				appSendData_s.pData[2] = mpCFG.mpCurrentMessageCount;
				if (appSendData_s.pData[RTI_TEST_PDATA_IDX_PROTOCOL_CMD] == RTI_CMD_TEST_PRODUCTION_REQUEST)
				{
					// Enable receiver now so it's on in time for the response.
					RTI_RxEnableReq(MASSPRODUCTION_APP_WAIT_FOR_RESPONSE_TIMEOUT + MASSPRODUCTION_APP_TIMEOUT_MARGIN);
				}
				// Send message
				RTI_SendDataReq(appSendData_s.dstIndex, appSendData_s.profileId,
						appSendData_s.vendorId, appSendData_s.txOptions, appSendData_s.len,
						(uint8*)&(appSendData_s.pData[0]));
			}
			else
			{
				if (appSendData_s.pData[RTI_TEST_PDATA_IDX_PROTOCOL_CMD] == RTI_CMD_TEST_PRODUCTION_REQUEST)
				{
					// Enable receiver now so it's on in time for the response.
					RTI_RxEnableReq(MASSPRODUCTION_APP_WAIT_FOR_RESPONSE_TIMEOUT + MASSPRODUCTION_APP_TIMEOUT_MARGIN);
					// Send message
					uint8 reportRequest[2];
					reportRequest[RTI_TEST_PDATA_IDX_PROTOCOL_ID] = RTI_PROTOCOL_TEST;
					reportRequest[RTI_TEST_PDATA_IDX_PROTOCOL_CMD] = RTI_CMD_TEST_PRODUCTION_REPORT_REQ;
					RTI_SendDataReq(appSendData_s.dstIndex, appSendData_s.profileId,
							appSendData_s.vendorId, appSendData_s.txOptions, sizeof(reportRequest),
							reportRequest);
				}
				else
				{
					appState = AP_STATE_READY;
					// Test completed, exit
					timer_set_event(MPA_App_threadId, MASSPRODUCTION_APP_EVT_EXIT);
				}
			}
		}
		else
		{
			LOG_DEBUG("Can only send data in %s [0x%.2X]\n", AppState_list[AP_STATE_NDATA_PREPARE], AP_STATE_NDATA_PREPARE);
			appState = AP_STATE_READY;
			timer_set_event(MPA_App_threadId, MASSPRODUCTION_APP_EVT_EXIT);
		}

		// Prepare to clear event
		procEvents |= MASSPRODUCTION_APP_EVT_DATA_SEND;
	}
	if ((events & MASSPRODUCTION_APP_EVT_RESPONSE_TIMEOUT) != 0)
	{
		// Return to Ready state
		appState = AP_STATE_READY;
		LOG_DEBUG("Failed to receive response from Server\n");
		timer_set_event(MPA_App_threadId, MASSPRODUCTION_APP_EVT_EXIT);
		// Prepare to clear event
		procEvents |= MASSPRODUCTION_APP_EVT_RESPONSE_TIMEOUT;
	}
	if ((events & MASSPRODUCTION_APP_EVT_INIT) != 0)
	{
		LOG_DEBUG("State: %s [0x%.2X]\n", AppState_list[appState], appState);
		// Prepare to clear event
		procEvents |= MASSPRODUCTION_APP_EVT_INIT;
		if (appState == AP_STATE_INIT_COLD)
		{
			uint8 startupFlg, retVal;

			startupFlg = CLEAR_STATE;
			retVal = RTI_WriteItem(RTI_CP_ITEM_STARTUP_CTRL, 1, &startupFlg);
			if (retVal != RTI_SUCCESS)
			{
				LOG_DEBUG("Could no set Cold Start flag\n");
			}
			else
			{
				LOG_DEBUG("Successfully set Cold Start flag\n");
			}
		}

		appInitConfigParam();

		// Apply changes
		appSetCFGParamOnRNP();
		LOG_DEBUG("Trying to initialize again, as %s\n",
				(appCFGParam.nodeCapabilities & RCN_NODE_CAP_TARGET) ? "Target" : "Controller");
		//When Launching the Thread, Mutex is unlocked.
		if ( (mutexRet = pthread_mutex_trylock(&appInitMutex)) == EBUSY)
		{
			LOG_DEBUG("appInit Mutex busy\n");
		}
		else
		{
			LOG_DEBUG("appInit Lock status: %d\n", mutexRet);
		}
		RTI_InitReq();

		//Mutex is unlocked only inside RTI_Initcnf();
		pthread_mutex_lock(&appInitMutex);

		// Prepare to clear event
		procEvents |= MASSPRODUCTION_APP_EVT_DATA_SEND;
	}
	if ((events & MASSPRODUCTION_APP_EVT_EXIT) != 0)
	{
		LOG_DEBUG("State: %s [0x%.2X]\n", AppState_list[appState], appState);
		if ((appSendData_s.pData[RTI_TEST_PDATA_IDX_PROTOCOL_CMD] == RTI_CMD_TEST_PRODUCTION_REQUEST) &&
			(mpCFG.mpCurrentMessageCount > 1))
		{
			LOG_INFO("Test ended after sending %d messages + report\n", mpCFG.mpCurrentMessageCount - 1);
		}
		else
		{
			LOG_INFO("Test ended after sending %d messages\n", mpCFG.mpCurrentMessageCount);
		}
		//Restore pairing table
		LOG_INFO("Restoring pairing table\n");
		appRestorePairingTable();
		// Safe to exit
		//Terminate Thread
		appThreadTerminate = 1;
		LOG_DEBUG("Thread set for termination\n");
		// Prepare to clear event
		procEvents |= MASSPRODUCTION_APP_EVT_EXIT;
	}
	// Clear event
	timer_clear_event(MPA_App_threadId, procEvents);
}

/**************************************************************************************************
 *
 * @fn      RTI_InitCnf
 *
 * @brief   RTI confirmation callback initiated by client's RTI_InitReq API
 *          call. The client is expected to complete this function.
 *
 *          NOTE: It is possible that this call can be made to the RTI client
 *                before the call to RTI_InitReq has returned.
 *
 * @param   status - Result of RTI_InitReq API call.
 *
 * @return  void
 */
void RTI_InitCnf(rStatus_t status)
{
	if (status == RTI_SUCCESS)
	{
		LOG_INFO("RTI_InitCnf(%s)\n", rtiStatus_list[status]);

		uint8 startupFlg;
		startupFlg = RESTORE_STATE;
		RTI_WriteItem(RTI_CP_ITEM_STARTUP_CTRL, 1, &startupFlg);

		appState = AP_STATE_READY;

		// Get configuration parameters from RNP, to make sure we display the correct settings
		if (appVerifyCFGParamAgainstRNP() != NPI_LNX_SUCCESS)
		{
			fprintf(stderr, "Configuration parameters written to RNP does not correspond to what was requested\n");
			timer_set_event(MPA_App_threadId, MASSPRODUCTION_APP_EVT_EXIT);
		}
		else
		{
			LOG_DEBUG("Entered %s [0x%.2X]\n", AppState_list[appState], appState);

			if (RTI_SUCCESS == RTI_WriteItemEx(RTI_PROFILE_RTI, RTI_SA_ITEM_TRANSMIT_POWER, 1,
					(uint8*)&mpCFG.mpTxPower) )
			{
				LOG_INFO("Tx Power set: %d\n", mpCFG.mpTxPower);
			}
			else
			{
				LOG_DEBUG("Could not set Tx Power!\n");
			}

			// Set logical channel
			if (RTI_SUCCESS == RTI_WriteItemEx(RTI_PROFILE_RTI, RTI_SA_ITEM_TRANSMIT_POWER, 1,
					(uint8*)&fakePentryTable[appSendData_s.dstIndex].logicalChannel) )
			{
				LOG_INFO("Channel set to: %d\n", fakePentryTable[appSendData_s.dstIndex].logicalChannel);
			}
			else
			{
				LOG_DEBUG("Could not set Channel!\n");
			}

			// Schedule event to begin testing.
			timer_set_event(MPA_App_threadId, MASSPRODUCTION_APP_EVT_DATA_SEND);
		}
	}
	else
	{
		LOG_DEBUG("RTI_InitCnf(0x%.2X - %s)\n", status, rtiStatus_list[status]);
		// Try to reset RNP
		RTI_SwResetReq();

		// Schedule event to try again.
		timer_start_timerEx(MPA_App_threadId, MASSPRODUCTION_APP_EVT_INIT, 1200); // Retry initialization after 1.2 seconds
	}

	//Unlock Mutex So that Application can continue
	LOG_DEBUG("Unlock appInit Mutex\n");
	pthread_mutex_unlock(&appInitMutex);
}

/**************************************************************************************************
 *
 * @fn      RTI_PairCnf
 *
 * @brief   RTI confirmation callback initiated by client's RTI_PairReq API
 *          call. The client is expected to complete this function.
 *
 *          NOTE: It is possible that this call can be made to the RTI client
 *                before the call to RTI_PairReq has returned.
 *
 * @param   status - Result of RTI_PairReq API call.
 * @param   dstIndex - Pairing table index of paired device, or invalid.
 * @param   devType  - Pairing table index device type, or invalid.
 * @return  void
 */
void RTI_PairCnf(rStatus_t status, uint8 dstIndex, uint8 devType) {}

/**************************************************************************************************
 *
 * @fn      RTI_AllowPairCnf
 *
 * @brief   RTI confirmation callback initiated by client's RTI_AllowPairReq API
 *          call. The client is expected to complete this function.
 *
 *          NOTE: It is possible that this call can be made to the RTI client
 *                before the call to RTI_AllowPairReq has returned.
 * @param   status   - Result of RTI_PairReq API call.
 * @param   dstIndex - Pairing table entry of paired device, or invalid
 * @param   devType  - Pairing table index device type, or invalid
 *
 * @return  void
 */
void RTI_AllowPairCnf(rStatus_t status, uint8 dstIndex, uint8 devType) {}

/**************************************************************************************************
 *
 * @fn      RTI_SendDataCnf
 *
 * @brief   RTI confirmation callback initiated by client's RTI_SendDataReq API
 *          call. The client is expected to complete this function.
 *
 *          NOTE: It is possible that this call can be made to the RTI client
 *                before the call to RTI_SendDataReq has returned.
 *
 * @param   status - Result of RTI_SendDataReq API call.
 *
 * @return  void
 */
void RTI_SendDataCnf(rStatus_t status)
{
	if (appState == AP_STATE_READY)
	{
		LOG_DEBUG("RTI_SendDataCnf(0x%.2X - %s), but sent in wrong state: %s\n",
				status,
				rtiStatus_list[status],
				AppState_list[appState]);
	}
	else if (appState == AP_STATE_NDATA)
	{
		mpCFG.mpCurrentMessageCount++;
		if (status == RTI_SUCCESS)
		{
			if (appSendData_s.pData[RTI_TEST_PDATA_IDX_PROTOCOL_CMD] == RTI_CMD_TEST_PRODUCTION_REQUEST)
			{
				// Go to Wait For Response state
				appState = AP_STATE_WAIT_FOR_RESPONSE;
				timer_start_timerEx(MPA_App_threadId, MASSPRODUCTION_APP_EVT_RESPONSE_TIMEOUT, MASSPRODUCTION_APP_WAIT_FOR_RESPONSE_TIMEOUT);
			}
			else
			{
				// Return to Send Data Prepare state
				appState = AP_STATE_NDATA_PREPARE;
				LOG_DEBUG("Data %d sent successfully\n", mpCFG.mpCurrentMessageCount);
				// Schedule event to continue testing.
				//			timer_set_event(MPA_App_threadId, MASSPRODUCTION_APP_EVT_DATA_SEND);
				timer_start_timerEx(MPA_App_threadId, MASSPRODUCTION_APP_EVT_DATA_SEND, 5);
			}
		}
		else
		{
			// Return to Ready state
			appState = AP_STATE_READY;
			LOG_INFO("RTI_SendDataCnf(0x%.2X - %s)\n", status, rtiStatus_list[status]);
			timer_set_event(MPA_App_threadId, MASSPRODUCTION_APP_EVT_EXIT);
		}
	}
}

/**************************************************************************************************
 *
 * @fn      RTI_StandbyCnf
 *
 * @brief   RTI confirmation callback initiated by client's RTI_StandbyReq API
 *          call. The client is expected to complete this function.
 *
 *          NOTE: It is possible that this call can be made to the RTI client
 *                before the call to RTI_RxEnableReq has returned.
 *
 * input parameters
 *
 * @param   status - RTI_SUCCESS
 *                   RTI_ERROR_INVALID_PARAMETER
 *                   RTI_ERROR_UNSUPPORTED_ATTRIBUTE
 *                   RTI_ERROR_INVALID_INDEX
 *                   RTI_ERROR_UNKNOWN_STATUS_RETURNED
 *
 * output parameters
 *
 * None.
 *
 * @return  None.
 */
void RTI_StandbyCnf(rStatus_t status) {}

/**************************************************************************************************
 *
 * @fn      RTI_ReceiveDataInd
 *
 * @brief   RTI receive data indication callback asynchronously initiated by
 *          another node. The client is expected to complete this function.
 *
 * input parameters
 *
 * @param   srcIndex:  Pairing table index.
 * @param   profileId: Profile identifier.
 * @param   vendorId:  Vendor identifier.
 * @param   rxLQI:     Link Quality Indication.
 * @param   rxFlags:   Receive flags.
 * @param   len:       Number of bytes to send.
 * @param   *pData:    Pointer to data to be sent.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 */
void RTI_ReceiveDataInd(uint8 srcIndex, uint8 profileId, uint16 vendorId,
		uint8 rxLQI, uint8 rxFlags, uint8 len, uint8 *pData)
{

	// Check if it's wanted data that we should bother process
	if (rxFlags & RTI_RX_FLAGS_VENDOR_SPECIFIC)
	{
		if (vendorId == RTI_VENDOR_TEXAS_INSTRUMENTS)
		{
			if (profileId == RTI_PROFILE_TI_NON_MSO_BACKUP)
			{
				if (len > RTI_TEST_PDATA_IDX_SEQNUM)
				{
					if (pData[RTI_TEST_PDATA_IDX_PROTOCOL_ID] == RTI_PROTOCOL_TEST)
					{
						if (pData[RTI_TEST_PDATA_IDX_PROTOCOL_CMD] == RTI_CMD_TEST_PRODUCTION_RESPONSE)
						{
							LOG_INFO("[MPA] Received Response: %d - %d - %d\n", pData[RTI_TEST_PDATA_IDX_SEQNUM],
									rxLQI, pData[RTI_TEST_PDATA_IDX_FREQEST]);
							// Proceed with test
							appState = AP_STATE_NDATA_PREPARE;
							LOG_DEBUG("Data %d sent successfully\n", mpCFG.mpCurrentMessageCount);
							// Schedule event to continue testing.
							//			timer_set_event(MPA_App_threadId, MASSPRODUCTION_APP_EVT_DATA_SEND);
							timer_start_timerEx(MPA_App_threadId, MASSPRODUCTION_APP_EVT_DATA_SEND, 5);
						}
						else if (pData[RTI_TEST_PDATA_IDX_PROTOCOL_CMD] == RTI_CMD_TEST_PRODUCTION_REPORT_RSP)
						{
							testReport_t *report = (testReport_t *)&pData[RTI_TEST_PDATA_IDX_PROTOCOL_CMD + 1];
							int averageRSSI = report->averageRSSI;
							if (averageRSSI > 127)
							{
								averageRSSI -= 256;
							}
							LOG_INFO("[MPA] Received Report: \n\t\tRSSI:\t\t%ddBm\n\t\tFREQEST:\t%dHz\n\t\tReceived:\t%d\n",
									averageRSSI, report->averageFREQEST, report->numOfReceived);

						}
					}
				}
			}
		}
	}
}

/**************************************************************************************************
 *
 * @fn      RTI_RxEnableCnf
 *
 * @brief   RTI confirmation callback initiated by client's RTI_RxEnableReq API
 *          call. The client is expected to complete this function.
 *
 * @param   status - Result of RTI_EnableRxReqReq API call.
 *
 * @return  void
 */
void RTI_RxEnableCnf(rStatus_t status) {}

/**************************************************************************************************
 *
 * @fn      RTI_EnableSleepCnf
 *
 * @brief   RTI confirmation callback initiated by client's RTI_EnableSleepReq API
 *          call. The client is expected to complete this function.
 *
 * @param   status - Result of RTI_EnableSleepReq API call.
 *
 * @return  void
 *
 */
void RTI_EnableSleepCnf(rStatus_t status) {}

/**************************************************************************************************
 *
 * @fn      RTI_DisableSleepCnf
 *
 * @brief   RTI confirmation callback initiated by client's RTI_DisableSleepReq API
 *          call. The client is expected to complete this function.
 *
 * @param   status - Result of RTI_EnableSleepReq API call.
 *
 * @return  void
 *
 */
void RTI_DisableSleepCnf(rStatus_t status) {}

/**************************************************************************************************
 *
 * @fn      RTI_UnpairInd
 *
 * @brief   RTI indication callback initiated by receiving unpair request command.
 *
 * @param   dstIndex - Pairing table index of paired device.
 *
 * @return  void
 */
void RTI_UnpairInd(uint8 dstIndex) {}

/**************************************************************************************************
 *
 * @fn      RTI_PairAbortCnf
 *
 * @brief   RTI confirmation callback initiated by client's RTI_PairAbortReq API
 *          call. The client is expected to complete this function.
 *
 *          NOTE: It is possible that this call can be made to the RTI client
 *                before the call to RTI_PairAbortReq has returned.
 *
 * @param   status - Result of RTI_PairAbortReq API call.
 * @return  void
 */
void RTI_PairAbortCnf(rStatus_t status) {}


/**************************************************************************************************
 *
 * @fn      RTI_UnpairCnf
 *
 * @brief   RTI confirmation callback initiated by client's RTI_UnpairReq API
 *          call. The client is expected to complete this function.
 *
 *          NOTE: It is possible that this call can be made to the RTI client
 *                before the call to RTI_UnpairReq has returned.
 *
 * @param   status   - Result of RTI_PairReq API call.
 * @param   dstIndex - Pairing table index of paired device, or invalid.
 *
 * @return  void
 */
void RTI_UnpairCnf(rStatus_t status, uint8 dstIndex) {}

/**************************************************************************************************
 *
 * @fn      RTI_ResetInd
 *
 * @brief   RTI indication that is used to notify AP that the NP has been reset.
 *
 * @param   void
 *
 * @return  void
 */
void RTI_ResetInd( void ) {}

/**************************************************************************************************
 *
 * @fn      appConfigureMassProductionTable
 *
 * @brief   Function to add fake pairing
 *
 * @param   none
 *
 * @return  void
 */
int appConfigureMassProductionTable()
{
	uint8 i;
	int ret = NPI_LNX_SUCCESS, numOfpEntries = 0;

	char *strBuf = NULL;
	strBuf = (char*) malloc(128);
	memset(strBuf, 0, 128);

	// Initialize fakePentryTable with invalid pRefs
	fakePentryTable[0].pairingRef = 0xFF;
	fakePentryTable[1].pairingRef = 0xFF;
	fakePentryTable[2].pairingRef = 0xFF;

//	__DEBUG_APP_ACTIVE = TRUE;

	if (NPI_LNX_SUCCESS == ConfigParser("PAIRING_INFO", "NUM_OF_ENTRIES", strBuf))
	{
		numOfpEntries = (uint8) strtol(strBuf, NULL, 16);
		LOG_DEBUG("[CFG_FAKE_PTABLE] pEntry->pairingRef = 0x%.2X\n", numOfpEntries);

		if (numOfpEntries > NUM_OF_FAKE_P_ENTRIES)
		{
			LOG_DEBUG("ERROR: Number of pairing entries %d is too large (> %d)\n", numOfpEntries, NUM_OF_FAKE_P_ENTRIES);
			ret = NPI_LNX_FAILURE;
		}
	}
	else
	{
		LOG_DEBUG("ERROR: %s not defined in configuration file\n", "NUM_OF_ENTRIES");
		ret = NPI_LNX_FAILURE;
	}

	for (i = 0; i < numOfpEntries; i++)
	{
		appReadPairingEntry(&fakePentryTable[i], i);
	}

	// Set these entries in the pairing table
	for (i = 0; i < numOfpEntries; i++)
	{
		if ( fakePentryTable[i].pairingRef != 0xFF)
		{
			LOG_DEBUG("writing fake pairing entry to index 0x%.2X\n", fakePentryTable[i].pairingRef);
      if (supportIndexedAPI == TRUE)
      {
        // Save original entry first
        RTI_ReadIndexedItem(RTI_PROFILE_RTI,
            RTI_SA_ITEM_PAIRING_TABLE_ENTRY,
            fakePentryTable[i].pairingRef,
            sizeof(rcnNwkPairingEntry_t),
            (uint8 *) &originalPairingTable[i]);
        // Try to write back
        RTI_WriteIndexedItem(RTI_PROFILE_RTI,
            RTI_SA_ITEM_PAIRING_TABLE_ENTRY,
            i,
            sizeof(rcnNwkPairingEntry_t),
            (uint8 *) &fakePentryTable[i]);
      }
      else
      {
        // Set current pairing entry
        RTI_WriteItemEx(RTI_PROFILE_RTI, RTI_SA_ITEM_PT_CURRENT_ENTRY_INDEX, 1,
            (uint8 *) &fakePentryTable[i].pairingRef);
        // Save original entry first
        RTI_ReadItemEx(RTI_PROFILE_RTI,
            RTI_SA_ITEM_PT_CURRENT_ENTRY,
            sizeof(rcnNwkPairingEntry_t),
            (uint8 *) &originalPairingTable[i]);
        // Try to write back
        RTI_WriteItemEx(RTI_PROFILE_RTI,
            RTI_SA_ITEM_PT_CURRENT_ENTRY,
            sizeof(rcnNwkPairingEntry_t),
            (uint8 *) &fakePentryTable[i]);
      }
		}
	}

//	__DEBUG_APP_ACTIVE = FALSE;

	if ( (fakePentryTable[0].recCapabilities & 0x01) != 0x01)
	{
		// We are Target in this pairing so set PAN_ID correspondingly
		LOG_DEBUG("Setting PAN ID 0x%.4X\n", fakePentryTable[0].panId);
		RTI_WriteItemEx(RTI_PROFILE_RTI, RTI_SA_ITEM_PAN_ID, 2, (uint8 *) &fakePentryTable[0].panId);
		// Then set SHORT_ADDRESS too
		LOG_DEBUG("Setting SHORT ADDR 0x%.4X\n", fakePentryTable[0].srcNwkAddress);
		RTI_WriteItemEx(RTI_PROFILE_RTI, RTI_SA_ITEM_SHORT_ADDRESS, 2, (uint8 *) &fakePentryTable[0].srcNwkAddress);
	}

	// Verify configuration
	appDisplayPairingTable();

	return ret;
}

/**************************************************************************************************
 *
 * @fn      appRestorePairingTable
 *
 * @brief   Function to restore pairing table after testing
 *
 * @param   none
 *
 * @return  void
 */
int appRestorePairingTable()
{
	uint8 i;
	int ret = NPI_LNX_SUCCESS, numOfpEntries = 0;

	char *strBuf = NULL;
	strBuf = (char*) malloc(128);
	memset(strBuf, 0, 128);

	// Initialize fakePentryTable with invalid pRefs
	fakePentryTable[0].pairingRef = 0xFF;
	fakePentryTable[1].pairingRef = 0xFF;
	fakePentryTable[2].pairingRef = 0xFF;

//	__DEBUG_APP_ACTIVE = TRUE;

	if (NPI_LNX_SUCCESS == ConfigParser("PAIRING_INFO", "NUM_OF_ENTRIES", strBuf))
	{
		numOfpEntries = (uint8) strtol(strBuf, NULL, 16);
		LOG_DEBUG("[CFG_FAKE_PTABLE] pEntry->pairingRef = 0x%.2X\n", numOfpEntries);

		if (numOfpEntries > NUM_OF_FAKE_P_ENTRIES)
		{
			LOG_DEBUG("ERROR: Number of pairing entries %d is too large (> %d)\n", numOfpEntries, NUM_OF_FAKE_P_ENTRIES);
			ret = NPI_LNX_FAILURE;
		}
	}
	else
	{
		LOG_DEBUG("ERROR: %s not defined in configuration file\n", "NUM_OF_ENTRIES");
		ret = NPI_LNX_FAILURE;
	}

	// Set these entries in the pairing table
	for (i = 0; i < numOfpEntries; i++)
	{
		if ( fakePentryTable[i].pairingRef != 0xFF)
		{
			LOG_DEBUG("writing back original pairing entry to index 0x%.2X\n", fakePentryTable[i].pairingRef);
			if (supportIndexedAPI == TRUE)
			{
			  // Restore original entry
			  RTI_WriteIndexedItem(RTI_PROFILE_RTI,
			      RTI_SA_ITEM_PAIRING_TABLE_ENTRY,
			      fakePentryTable[i].pairingRef,
			      sizeof(rcnNwkPairingEntry_t),
			      (uint8 *) &originalPairingTable[i]);
			}
			else
			{
        // Set current pairing entry
        RTI_WriteItemEx(RTI_PROFILE_RTI, RTI_SA_ITEM_PT_CURRENT_ENTRY_INDEX, 1,
            (uint8 *) &fakePentryTable[i].pairingRef);
        // Restore original entry
        RTI_WriteItemEx(RTI_PROFILE_RTI,
            RTI_SA_ITEM_PT_CURRENT_ENTRY,
            sizeof(rcnNwkPairingEntry_t),
            (uint8 *) &originalPairingTable[i]);
			}
		}
	}

//	__DEBUG_APP_ACTIVE = FALSE;

	if ( (originalPairingTable[0].recCapabilities & 0x01) != 0x01)
	{
		// Restore PAN_ID
		LOG_DEBUG("Setting PAN ID 0x%.4X\n", originalPairingTable[0].panId);
		RTI_WriteItemEx(RTI_PROFILE_RTI, RTI_SA_ITEM_PAN_ID, 2, (uint8 *) &originalPairingTable[0].panId);
		// Then restore SHORT_ADDRESS
		LOG_DEBUG("Setting SHORT ADDR 0x%.4X\n", originalPairingTable[0].srcNwkAddress);
		RTI_WriteItemEx(RTI_PROFILE_RTI, RTI_SA_ITEM_SHORT_ADDRESS, 2, (uint8 *) &originalPairingTable[0].srcNwkAddress);
	}

	// Verify configuration
	appDisplayPairingTable();

	return ret;
}

int appReadPairingEntry(rcnNwkPairingEntry_t *pEntry, uint8 index)
{
	char *strBuf = NULL, *pStrCntd = NULL;
	int ret = NPI_LNX_SUCCESS;
	strBuf = (char*) malloc(128);
	memset(strBuf, 0, 128);
	LOG_DEBUG("\n------------------------- READ PAIRING ENTRY %d FROM CFG -------------------------\n", index);
	char tmpStr[128];
	int strLen = 0;
	int j;
	char *strTmp0 = (char*) malloc(128);
	memset(strTmp0, 0, 128);
	snprintf(strTmp0, 128, "IDX%d_PAIRINGREF", index);
	if (NPI_LNX_SUCCESS == ConfigParser("PAIRING_INFO", strTmp0, strBuf))
	{
		pEntry->pairingRef = (uint8) strtol(strBuf, NULL, 16);
		LOG_DEBUG("[CFG_FAKE_PTABLE] pEntry->pairingRef = 0x%.2X\n", pEntry->pairingRef);
	}
	else
	{
		LOG_DEBUG("ERROR: %s not defined in configuration file\n", strTmp0);
		ret = NPI_LNX_FAILURE;
	}
	snprintf(strTmp0, 128, "IDX%d_SRCNWKADDR", index);
	if (NPI_LNX_SUCCESS == ConfigParser("PAIRING_INFO", strTmp0, strBuf))
	{
		pEntry->srcNwkAddress = (uint16) strtol(strBuf, NULL, 16);
		LOG_DEBUG("[CFG_FAKE_PTABLE] pEntry->srcNwkAddress = 0x%.4X\n", pEntry->srcNwkAddress);
	}
	else
	{
		LOG_DEBUG("ERROR: %s not defined in configuration file\n", strTmp0);
		ret = NPI_LNX_FAILURE;
	}
	snprintf(strTmp0, 128, "IDX%d_LOG_CH", index);
	if (NPI_LNX_SUCCESS == ConfigParser("PAIRING_INFO", strTmp0, strBuf))
	{
		pEntry->logicalChannel = (uint16) strtol(strBuf, NULL, 10);
		LOG_DEBUG("[CFG_FAKE_PTABLE] pEntry->logicalChannel = 0x%.2d\n", pEntry->logicalChannel);
	}
	else
	{
		LOG_DEBUG("ERROR: %s not defined in configuration file\n", strTmp0);
		ret = NPI_LNX_FAILURE;
	}
	snprintf(strTmp0, 128, "IDX%d_IEEE", index);
	if (NPI_LNX_SUCCESS == ConfigParser("PAIRING_INFO", strTmp0, strBuf))
	{
		pEntry->ieeeAddress[SADDR_EXT_LEN - 1] = (uint8) strtol(strBuf, &pStrCntd, 16);
	}
	else
	{
		LOG_DEBUG("ERROR: %s not defined in configuration file\n", strTmp0);
		ret = NPI_LNX_FAILURE;
	}
	strLen = 0;
	for (j = SADDR_EXT_LEN - 2; j >= 0; j--)
	{
		pEntry->ieeeAddress[j] = (uint8) strtol(pStrCntd, &pStrCntd, 16);
    snprintf(tmpStr+strLen, sizeof(tmpStr)-strLen, ":%.2X", (pEntry->ieeeAddress[j] & 0x00FF));
//    printf(":%.2X\n", (pEntry->ieeeAddress[j] & 0x00FF));
    strLen += 3;
	}
  LOG_DEBUG("[CFG_FAKE_PTABLE] pEntry->ieeeAddress = %.2hX%s\n", pEntry->ieeeAddress[SADDR_EXT_LEN - 1], tmpStr);

	snprintf(strTmp0, 128, "IDX%d_PANID", index);
	if (NPI_LNX_SUCCESS == ConfigParser("PAIRING_INFO", strTmp0, strBuf))
	{
		pEntry->panId = (uint16) strtol(strBuf, NULL, 16);
		LOG_DEBUG("[CFG_FAKE_PTABLE] pEntry->panId = 0x%.4X\n", pEntry->panId);
	}
	else
	{
		LOG_DEBUG("ERROR: %s not defined in configuration file\n", strTmp0);
		ret = NPI_LNX_FAILURE;
	}
	snprintf(strTmp0, 128, "IDX%d_NWKADDR", index);
	if (NPI_LNX_SUCCESS == ConfigParser("PAIRING_INFO", strTmp0, strBuf))
	{
		pEntry->nwkAddress = (uint16) strtol(strBuf, NULL, 16);
		LOG_DEBUG("[CFG_FAKE_PTABLE] pEntry->nwkAddress = 0x%.4X\n", pEntry->nwkAddress);
	}
	else
	{
		LOG_DEBUG("ERROR: %s not defined in configuration file\n", strTmp0);
		ret = NPI_LNX_FAILURE;
	}
	snprintf(strTmp0, 128, "IDX%d_RECCAP", index);
	if (NPI_LNX_SUCCESS == ConfigParser("PAIRING_INFO", strTmp0, strBuf))
	{
		pEntry->recCapabilities = (uint8) strtol(strBuf, NULL, 16);
		LOG_DEBUG("[CFG_FAKE_PTABLE] pEntry->recCapabilities = 0x%.2X\n", pEntry->recCapabilities);
	}
	else
	{
		LOG_DEBUG("ERROR: %s not defined in configuration file\n", strTmp0);
		ret = NPI_LNX_FAILURE;
	}
	snprintf(strTmp0, 128, "IDX%d_KEY_VALID", index);
	if (NPI_LNX_SUCCESS == ConfigParser("PAIRING_INFO", strTmp0, strBuf))
	{
		pEntry->securityKeyValid = (uint8) strtol(strBuf, &pStrCntd, 16);
		LOG_DEBUG("[CFG_FAKE_PTABLE] pEntry->securityKeyValid = %d\n", pEntry->securityKeyValid);
	}
	else
	{
		LOG_DEBUG("ERROR: %s not defined in configuration file\n", strTmp0);
		ret = NPI_LNX_FAILURE;
	}
	snprintf(strTmp0, 128, "IDX%d_SECKEY", index);
	if (NPI_LNX_SUCCESS == ConfigParser("PAIRING_INFO", strTmp0, strBuf))
	{
		pEntry->securityKey[0] = (uint8) strtol(strBuf, &pStrCntd, 16);
		strLen = 0;
		snprintf(tmpStr+strLen, sizeof(tmpStr)-strLen, "%.2X", pEntry->securityKey[0]);
		strLen += 3;
		for (j = 1; j < RCN_SEC_KEY_LENGTH; j++)
		{
			pEntry->securityKey[j] = (uint8) strtol(pStrCntd, &pStrCntd, 16);
			snprintf(tmpStr+strLen, sizeof(tmpStr)-strLen, ":%.2X", pEntry->securityKey[j]);
			strLen += 3;
		}
		LOG_DEBUG("[CFG_FAKE_PTABLE] pEntry->securityKey = %s\n", tmpStr);
	}
	else
	{
		LOG_DEBUG("ERROR: %s not defined in configuration file\n", strTmp0);
		ret = NPI_LNX_FAILURE;
	}
	snprintf(strTmp0, 128, "IDX%d_VENDORID", index);
	if (NPI_LNX_SUCCESS == ConfigParser("PAIRING_INFO", strTmp0, strBuf))
	{
		pEntry->vendorIdentifier = (uint16) strtol(strBuf, NULL, 16);
		LOG_DEBUG("[CFG_FAKE_PTABLE] pEntry->vendorIdentifier = 0x%.4X\n", pEntry->vendorIdentifier);
	}
	else
	{
		LOG_DEBUG("ERROR: %s not defined in configuration file\n", strTmp0);
		ret = NPI_LNX_FAILURE;
	}
	strLen = 0;
	tmpStr[0] = '\0'; // Initialize with empty string, in case it is not set
	for (j = 0; j < RTI_MAX_NUM_DEV_TYPES; j++)
	{
		snprintf(strTmp0, 128, "IDX%d_DEVTYPE%d", index, j);
		if (NPI_LNX_SUCCESS == ConfigParser("PAIRING_INFO", strTmp0, strBuf))
		{
			pEntry->devTypeList[j] = (uint8) strtol(strBuf, NULL, 16);
			snprintf(tmpStr+strLen, sizeof(tmpStr)-strLen, " 0x%.2X", pEntry->securityKey[0]);
			strLen += 5;
		}
		else
		{
			LOG_DEBUG("ERROR: %s not defined in configuration file\n", strTmp0);
			ret = NPI_LNX_FAILURE;
		}
	}
	LOG_DEBUG("[CFG_FAKE_PTABLE] pEntry->devTypeList =%s\n", tmpStr);
	snprintf(strTmp0, 128, "IDX%d_RECFRAMECOUNTER", index);
	if (NPI_LNX_SUCCESS == ConfigParser("PAIRING_INFO", strTmp0, strBuf))
	{
		pEntry->recFrameCounter = (uint32) strtol(strBuf, NULL, 16);
		LOG_DEBUG("[CFG_FAKE_PTABLE] pEntry->recFrameCounter = 0x%.8X\n", pEntry->recFrameCounter);
	}
	else
	{
		LOG_DEBUG("ERROR: %s not defined in configuration file\n", strTmp0);
		ret = NPI_LNX_FAILURE;
	}
	snprintf(strTmp0, 128, "IDX%d_PROFILE_DISC", index);
	if (NPI_LNX_SUCCESS == ConfigParser("PAIRING_INFO", strTmp0, strBuf))
	{
		pEntry->profileDiscs[0] = (uint8) strtol(strBuf, &pStrCntd, 16);
		LOG_DEBUG("[CFG_FAKE_PTABLE] pEntry->profileDiscs[0] = 0x%.2X\n", pEntry->profileDiscs[0]);
	}
	else
	{
		LOG_DEBUG("ERROR: %s not defined in configuration file\n", strTmp0);
		ret = NPI_LNX_FAILURE;
	}
	LOG_DEBUG("------------------------- PAIRING ENTRY READ -------------------------\n\n");
	if (strTmp0 != NULL)
	{
		free(strTmp0);
	}

	return ret;
}

int appInitConfigParam()
{
	int i, ret = NPI_LNX_SUCCESS, strLen = 0;
	char tmpStr[512];
	// Check if we have a configuration file to read from.
	if (NPI_LNX_SUCCESS == ConfigParserGetBaseSettings(NULL))
	{
		char *strBuf = NULL, *pStrCntd = NULL;
		strBuf = (char*) malloc(128);
		memset(strBuf, 0, 128);
		if (NPI_LNX_SUCCESS == ConfigParser("OWN_INFO", "NODECAP", strBuf))
		{
			appCFGParam.nodeCapabilities = (uint8) strtol(strBuf, NULL, 16);
			LOG_DEBUG("[OWN_CFG] appCFGParam.nodeCapabilities = 0x%.2X\n", appCFGParam.nodeCapabilities);

			if (appCFGParam.nodeCapabilities & RCN_NODE_CAP_TARGET)
			{
				LOG_DEBUG("\nTarget Configuration\n");
			}
			else
			{
				LOG_DEBUG("\nController Configuration\n");
			}
		}
		else
		{
			LOG_DEBUG("ERROR: NODECAP not defined in configuration file\n");
			ret = NPI_LNX_FAILURE;
		}

		if (ret == NPI_LNX_SUCCESS)
		{
			if (NPI_LNX_SUCCESS == ConfigParser("OWN_INFO", "TGTLIST", strBuf))
			{
				// Set up configuration parameters that are different from default values
				appCFGParam.tgtTypeList[0] = (uint8) strtol(strBuf, &pStrCntd, 16);
				LOG_DEBUG("[OWN_CFG] appCFGParam.tgtTypeList[%d] = %d\n", 0, appCFGParam.tgtTypeList[0]);
				for (i = 1; i < RTI_MAX_NUM_SUPPORTED_TGT_TYPES; i++)
				{
					appCFGParam.tgtTypeList[i] = (uint8) strtol(pStrCntd, &pStrCntd, 16);
					LOG_DEBUG("[OWN_CFG] appCFGParam.tgtTypeList[%d] = 0x%.2X\n", i, appCFGParam.tgtTypeList[i]);
				}
			}
			else
			{
				LOG_DEBUG("ERROR: TGTLIST not defined in configuration file\n");
				ret = NPI_LNX_FAILURE;
			}
		}

		if (ret == NPI_LNX_SUCCESS)
		{
			if (NPI_LNX_SUCCESS == ConfigParser("OWN_INFO", "APPCAP", strBuf))
			{
				appCFGParam.appCapabilities = (uint8) strtol(strBuf, NULL, 16);
				LOG_DEBUG("[OWN_CFG] appCFGParam.appCapabilities = 0x%.2X\n", appCFGParam.appCapabilities);
			}
			else
			{
				LOG_DEBUG("ERROR: APPCAP not defined in configuration file\n");
				ret = NPI_LNX_FAILURE;
			}
		}

		if (ret == NPI_LNX_SUCCESS)
		{
			if (NPI_LNX_SUCCESS == ConfigParser("OWN_INFO", "DEVTYPELIST", strBuf))
			{
				appCFGParam.devTypeList[0] = (uint8) strtol(strBuf, &pStrCntd, 16);
				LOG_DEBUG("[OWN_CFG] appCFGParam.devTypeList[%d] = 0x%.2X\n", 0, appCFGParam.devTypeList[0]);
				for (i = 1; i < RTI_MAX_NUM_DEV_TYPES; i++)
				{
					appCFGParam.devTypeList[i] = (uint8) strtol(pStrCntd, &pStrCntd, 16);
					LOG_DEBUG("[OWN_CFG] appCFGParam.devTypeList[%d] = 0x%.2X\n", i, appCFGParam.devTypeList[i]);
				}
			}
			else
			{
				LOG_DEBUG("ERROR: DEVTYPELIST not defined in configuration file\n");
				ret = NPI_LNX_FAILURE;
			}
		}

		if (ret == NPI_LNX_SUCCESS)
		{
			if (NPI_LNX_SUCCESS == ConfigParser("OWN_INFO", "PROFILEIDLIST", strBuf))
			{
				appCFGParam.profileIdList[0] = (uint8) strtol(strBuf, &pStrCntd, 16);
				LOG_DEBUG("[OWN_CFG] appCFGParam.profileIdList[%d] = 0x%.2X\n", 0, appCFGParam.profileIdList[0]);
				for (i = 1; i < RTI_MAX_NUM_PROFILE_IDS; i++)
				{
					appCFGParam.profileIdList[i] = (uint8) strtol(pStrCntd, &pStrCntd, 16);
					LOG_DEBUG("[OWN_CFG] appCFGParam.profileIdList[%d] = 0x%.2X\n", i, appCFGParam.profileIdList[i]);
				}
			}
			else
			{
				LOG_DEBUG("ERROR: PROFILEIDLIST not defined in configuration file\n");
				ret = NPI_LNX_FAILURE;
			}
		}

		if (ret == NPI_LNX_SUCCESS)
		{
			if (NPI_LNX_SUCCESS == ConfigParser("OWN_INFO", "VENDORID", strBuf))
			{
				appCFGParam.vendorId = (uint16) strtol(strBuf, NULL, 16);
				LOG_DEBUG("[OWN_CFG] appCFGParam.vendorId = 0x%.4X\n", appCFGParam.vendorId);
			}
			else
			{
				LOG_DEBUG("ERROR: VENDORID not defined in configuration file\n");
				ret = NPI_LNX_FAILURE;
			}
		}

		if (ret == NPI_LNX_SUCCESS)
		{
			if (NPI_LNX_SUCCESS == ConfigParser("OWN_INFO", "VENDORSTRING", strBuf))
			{
				memcpy(appCFGParam.vendorString, strBuf, strlen(strBuf));
				if (strlen(strBuf) > RTI_VENDOR_STRING_LENGTH)
				{
					LOG_DEBUG("[OWN_CFG] ERROR: Vendor String length > %d\n", RTI_VENDOR_STRING_LENGTH);
				}
				else
				{
					LOG_DEBUG("[OWN_CFG] appCFGParam.vendorString = %s\n", strBuf);
				}
			}
			else
			{
				LOG_DEBUG("ERROR: VENDORSTRING not defined in configuration file\n");
				ret = NPI_LNX_FAILURE;
			}
		}

		if (ret == NPI_LNX_SUCCESS)
		{
			if (NPI_LNX_SUCCESS == ConfigParser("TESTPARAM", "TEST_STATION_NUMBER", strBuf))
			{
				mpCFG.mpTestStationNumber = (int) strtol(strBuf, NULL, 10);
				LOG_DEBUG("[TESTPARAM] mpCFG.mpTestStationNumber = %d\n", mpCFG.mpTestStationNumber);
			}
			else
			{
				LOG_DEBUG("ERROR: TEST_STATION_NUMBER not defined in configuration file\n");
				ret = NPI_LNX_FAILURE;
			}
		}

		if (ret == NPI_LNX_SUCCESS)
		{
			if (NPI_LNX_SUCCESS == ConfigParser("TESTPARAM", "EXPECTED_NUMBER_OF_MESSAGES", strBuf))
			{
				mpCFG.mpExpectedNumberOfMessages = (int) strtol(strBuf, NULL, 10);
				LOG_DEBUG("[TESTPARAM] mpCFG.mpExpectedNumberOfMessages = %d\n", mpCFG.mpExpectedNumberOfMessages);
			}
			else
			{
				LOG_DEBUG("ERROR: EXPECTED_NUMBER_OF_MESSAGES not defined in configuration file\n");
				ret = NPI_LNX_FAILURE;
			}
		}

		if (ret == NPI_LNX_SUCCESS)
		{
			if (NPI_LNX_SUCCESS == ConfigParser("TESTPARAM", "TX_POWER", strBuf))
			{
				mpCFG.mpTxPower = (int) strtol(strBuf, NULL, 10);
				LOG_DEBUG("[TESTPARAM] mpCFG.mpTxPower = %d\n", mpCFG.mpTxPower);
			}
			else
			{
				LOG_DEBUG("ERROR: TX_POWER not defined in configuration file\n");
				ret = NPI_LNX_FAILURE;
			}
		}

		// Configure message to send
		appSendData_s.dstIndex = 0;
		appSendData_s.profileId = RTI_PROFILE_TI; //RTI_PROFILE_TI_NON_MSO_BACKUP
		appSendData_s.txOptions = RTI_TX_OPTION_ACKNOWLEDGED | RTI_TX_OPTION_VENDOR_SPECIFIC;
		appSendData_s.vendorId = RTI_VENDOR_TEXAS_INSTRUMENTS;
		if (ret == NPI_LNX_SUCCESS)
		{
			if (NPI_LNX_SUCCESS == ConfigParser("TESTPARAM", "TOTAL_MESSAGE_LENGTH", strBuf))
			{
				appSendData_s.len = (int) strtol(strBuf, NULL, 10);
				LOG_DEBUG("[TESTPARAM] appSendData_s.len = %d\n", appSendData_s.len);
			}
			else
			{
				LOG_DEBUG("ERROR: TOTAL_MESSAGE_LENGTH not defined in configuration file\n");
				ret = NPI_LNX_FAILURE;
			}
		}
		char *strTmp0 = (char*) malloc(128);
		memset(strTmp0, 0, 128);
		uint8 byte = 0;
		// Bytes hex formatted
		for (byte = 0; byte < 2; byte++)
		{
			if (ret == NPI_LNX_SUCCESS)
			{
				snprintf(strTmp0, 128, "BYTE%d", byte);
				if (NPI_LNX_SUCCESS == ConfigParser("TESTPARAM", strTmp0, strBuf))
				{
					appSendData_s.pData[byte] = (int) strtol(strBuf, NULL, 16);
					LOG_DEBUG("[TESTPARAM] appSendData_s.pData[%d] = %d\n", byte, appSendData_s.pData[byte]);
				}
				else
				{
					LOG_DEBUG("ERROR: %s not defined in configuration file\n", strTmp0);
					ret = NPI_LNX_FAILURE;
				}
			}
		}
		// Bytes decimal formatted
		for (byte = 4; byte < 8; byte++)
		{
			if (ret == NPI_LNX_SUCCESS)
			{
				snprintf(strTmp0, 128, "BYTE%d", byte);
				if (NPI_LNX_SUCCESS == ConfigParser("TESTPARAM", strTmp0, strBuf))
				{
					appSendData_s.pData[byte] = (int) strtol(strBuf, NULL, 10);
					LOG_DEBUG("[TESTPARAM] appSendData_s.pData[%d] = %d\n", byte, appSendData_s.pData[byte]);
				}
				else
				{
					LOG_DEBUG("ERROR: %s not defined in configuration file\n", strTmp0);
					ret = NPI_LNX_FAILURE;
				}
			}
		}

		// Write IEEE address to pData[8:15]
		if (appSendData_s.len > 15)
		{
			strLen = 0;
			uint8 ownIeee[8] = {0};
			// Get IEEE from RNP
			if (RTI_ReadItemEx(RTI_PROFILE_RTI, RTI_SA_ITEM_IEEE_ADDRESS, 8,
					(uint8*)&(ownIeee[0])) != RTI_SUCCESS) {
				//   AP_FATAL_ERROR();
				LOG_DEBUG("Failed to read Node Capabilities\n");
			}
			else
			{
				for (byte = 0; byte < 8; byte++)
				{
					snprintf(tmpStr+strLen, sizeof(tmpStr)-strLen, ":%.2X", ownIeee[byte]);
					strLen += 3;
					appSendData_s.pData[8+byte] = ownIeee[byte];
				}
				LOG_DEBUG("CFGD IEEE: %s\n", tmpStr);
				for (byte = 8; byte < 16; byte++)
				{
					snprintf(tmpStr+strLen, sizeof(tmpStr)-strLen, ":%.2X", appSendData_s.pData[byte]);
					strLen += 3;
				}
				LOG_DEBUG("DATA IEEE: %s\n", tmpStr);

			}
		}
		else
		{
			LOG_DEBUG("Unknown format with data length of %d\n", appSendData_s.len);
		}

		if (strBuf != NULL)
		{
			free(strBuf);
		}
		if (strTmp0 != NULL)
		{
			free(strTmp0);
		}
	}
	else
	{
		ret = NPI_LNX_FAILURE;
	}

	return ret;
}

static void appSetCFGParamOnRNP( void )
{
	////////////////////////////////////
	// Apply Configuration Parameters //
	////////////////////////////////////

	LOG_DEBUG("\n-------------------- SET RNP CONFIGURATION PARAMETERS-------------------\n");
	if (RTI_WriteItemEx(RTI_PROFILE_RTI, RTI_CP_ITEM_NODE_CAPABILITIES, 1,
			(uint8*)&(appCFGParam.nodeCapabilities)) != RTI_SUCCESS) {
		//   AP_FATAL_ERROR();
		LOG_ERROR("Could not write RTI_CP_ITEM_NODE_CAPABILITIES\n");
	}
	else
		LOG_DEBUG("Successfully wrote RTI_CP_ITEM_NODE_CAPABILITIES\n");

	if (RTI_WriteItemEx(RTI_PROFILE_RTI, RTI_CP_ITEM_NODE_SUPPORTED_TGT_TYPES,
			RTI_MAX_NUM_SUPPORTED_TGT_TYPES, appCFGParam.tgtTypeList) != RTI_SUCCESS) {
		///AP_FATAL_ERROR();
		LOG_ERROR("Could not write RTI_CP_ITEM_NODE_SUPPORTED_TGT_TYPES\n");
	}
	else
		LOG_DEBUG("Successfully wrote RTI_CP_ITEM_NODE_SUPPORTED_TGT_TYPES\n");

	if (RTI_WriteItemEx(RTI_PROFILE_RTI, RTI_CP_ITEM_APPL_CAPABILITIES, 1,
			(uint8*)&(appCFGParam.appCapabilities)) != RTI_SUCCESS) {
		// AP_FATAL_ERROR();
		LOG_ERROR("Could not write RTI_CP_ITEM_APPL_CAPABILITIES\n");
	}
	else
		LOG_DEBUG("Successfully wrote RTI_CP_ITEM_APPL_CAPABILITIES\n");

	if (RTI_WriteItemEx(RTI_PROFILE_RTI, RTI_CP_ITEM_APPL_DEV_TYPE_LIST,
			RTI_MAX_NUM_DEV_TYPES, appCFGParam.devTypeList) != RTI_SUCCESS) {
		//   AP_FATAL_ERROR();
		LOG_ERROR("Could not write RTI_CP_ITEM_APPL_DEV_TYPE_LIST\n");
	}
	else
		LOG_DEBUG("Successfully wrote RTI_CP_ITEM_APPL_DEV_TYPE_LIST\n");

	if (RTI_WriteItemEx(RTI_PROFILE_RTI, RTI_CP_ITEM_APPL_PROFILE_ID_LIST,
			RTI_MAX_NUM_PROFILE_IDS, appCFGParam.profileIdList) != RTI_SUCCESS) {
		//    AP_FATAL_ERROR();
		LOG_ERROR("Could not write RTI_CP_ITEM_APPL_PROFILE_ID_LIST\n");
	}
	else
		LOG_DEBUG("Successfully wrote RTI_CP_ITEM_APPL_PROFILE_ID_LIST\n");

	if (RTI_WriteItemEx(RTI_PROFILE_RTI, RTI_CP_ITEM_VENDOR_ID, 2,
			(uint8*)&(appCFGParam.vendorId)) != RTI_SUCCESS) {
		//  AP_FATAL_ERROR();
		LOG_ERROR("Could not write RTI_CP_ITEM_VENDOR_ID\n");
	}
	else
		LOG_DEBUG("Successfully wrote RTI_CP_ITEM_VENDOR_ID\n");

	if (RTI_WriteItemEx(RTI_PROFILE_RTI, RTI_CP_ITEM_VENDOR_NAME,
			sizeof(vendorName), appCFGParam.vendorString) != RTI_SUCCESS) {
		//  AP_FATAL_ERROR();
		LOG_ERROR("Could not write RTI_CP_ITEM_VENDOR_NAME\n");
	}
	else
		LOG_DEBUG("Successfully wrote RTI_CP_ITEM_VENDOR_NAME\n");
}


int appVerifyCFGParamAgainstRNP( void )
{
	///////////////////////////////////////////
	// Verify Configuration Parameters On RNP//
	///////////////////////////////////////////

	int ret = NPI_LNX_SUCCESS;

	appDevInfo_t appCFGParamOnRNP = {0};

	if (RTI_ReadItemEx(RTI_PROFILE_RTI, RTI_CP_ITEM_NODE_CAPABILITIES, 1,
			(uint8*)&(appCFGParamOnRNP.nodeCapabilities)) != RTI_SUCCESS) {
		//   AP_FATAL_ERROR();
		LOG_ERROR("Failed to read Node Capabilities\n");
	}

	if (RTI_ReadItemEx(RTI_PROFILE_RTI, RTI_CP_ITEM_NODE_SUPPORTED_TGT_TYPES,
			RTI_MAX_NUM_SUPPORTED_TGT_TYPES, appCFGParamOnRNP.tgtTypeList) != RTI_SUCCESS) {
		///AP_FATAL_ERROR();
		LOG_ERROR("Failed to read Target Types\n");
	}

	if (RTI_ReadItemEx(RTI_PROFILE_RTI, RTI_CP_ITEM_APPL_CAPABILITIES, 1,
			(uint8*)&(appCFGParamOnRNP.appCapabilities)) != RTI_SUCCESS) {
		// AP_FATAL_ERROR();
		LOG_ERROR("Failed to read Application Capabilities\n");
	}

	if (RTI_ReadItemEx(RTI_PROFILE_RTI, RTI_CP_ITEM_APPL_DEV_TYPE_LIST,
			RTI_MAX_NUM_DEV_TYPES, appCFGParamOnRNP.devTypeList) != RTI_SUCCESS) {
		//   AP_FATAL_ERROR();
		LOG_ERROR("Failed to read Device Types\n");
	}

	if (RTI_ReadItemEx(RTI_PROFILE_RTI, RTI_CP_ITEM_APPL_PROFILE_ID_LIST,
			RTI_MAX_NUM_PROFILE_IDS, appCFGParamOnRNP.profileIdList) != RTI_SUCCESS) {
		//    AP_FATAL_ERROR();
		LOG_ERROR("Failed to read Profile IDs\n");
	}

	if (RTI_ReadItemEx(RTI_PROFILE_RTI, RTI_CP_ITEM_VENDOR_ID, 2,
			(uint8*)&(appCFGParamOnRNP.vendorId)) != RTI_SUCCESS) {
		//  AP_FATAL_ERROR();
		LOG_ERROR("Failed to read Vendor ID\n");
	}

	if (RTI_ReadItemEx(RTI_PROFILE_RTI, RTI_CP_ITEM_VENDOR_NAME,
			sizeof(vendorName), appCFGParamOnRNP.vendorString) != RTI_SUCCESS) {
		//  AP_FATAL_ERROR();
		LOG_ERROR("Failed to read Vendor String\n");
	}

	// Now perform comparison
	if (memcmp(&appCFGParamOnRNP, &appCFGParam, sizeof(appDevInfo_t)) != 0)
	{
		ret = NPI_LNX_FAILURE;
	}

	return ret;
}

/**************************************************************************************************
 *
 * @fn      appDisplayPairingTable
 *
 * @brief   Function to read out, and display, pairing table
 *
 * @param   status - Result of RTI_InitReq API call.
 *
 * @return  void
 */
void appDisplayPairingTable() {
	rcnNwkPairingEntry_t *pEntry;
	// Allocate memory for one pairing entry
	pEntry = (rcnNwkPairingEntry_t *) malloc(sizeof(rcnNwkPairingEntry_t));

	uint8 i, result, atLeastOneEntryFound = 0, maxNumPairingEntries = 1;

	// Initialize memory for backup pairing table
	RTI_ReadItemEx(RTI_PROFILE_RTI, RTI_CONST_ITEM_MAX_PAIRING_TABLE_ENTRIES, 1,&maxNumPairingEntries);
	for (i = 0; i < maxNumPairingEntries; i++) {
    if (supportIndexedAPI == TRUE)
    {
      // Try to read out this entry
      result = RTI_ReadIndexedItem(RTI_PROFILE_RTI,
          RTI_SA_ITEM_PAIRING_TABLE_ENTRY,
          i,
          sizeof(rcnNwkPairingEntry_t),
          (uint8 *) pEntry);
    }
    else
    {
      // Set current pairing entry
      RTI_WriteItemEx(RTI_PROFILE_RTI, RTI_SA_ITEM_PT_CURRENT_ENTRY_INDEX, 1,
          (uint8 *) &i);
      // Try to read out this entry
      result = RTI_ReadItemEx(RTI_PROFILE_RTI,
          RTI_SA_ITEM_PT_CURRENT_ENTRY, sizeof(rcnNwkPairingEntry_t),
          (uint8 *) pEntry);
    }
    if (result == RTI_SUCCESS)
    {
      // Found pairing entry; display this.
      DisplayPairingTable(pEntry);

      atLeastOneEntryFound = 1;
    }
	}

	if (atLeastOneEntryFound != 0) {
		LOG_INFO("*************************************\n");
	} else {
		LOG_INFO("*************************************\n");
		LOG_INFO("* Pairing Table Is Empty\n");
		LOG_INFO("*************************************\n");
	}

	// Free pairing entry buffer
	free(pEntry);
}

static void SoftwareVersionToString(char *retStr, int maxStrLen, swVerExtended_t* swVerExtended)
{
    static char tmpStr[1024];
    size_t      curLen = 0;

    snprintf(tmpStr+curLen, sizeof(tmpStr)-curLen, "Extended Software Version:\n");
    curLen = strlen(tmpStr);
    snprintf(tmpStr+curLen, sizeof(tmpStr)-curLen, "\tMajor:    %d\n", swVerExtended->major);
    curLen = strlen(tmpStr);
    snprintf(tmpStr+curLen, sizeof(tmpStr)-curLen, "\tMinor:    %d\n", swVerExtended->minor);
    curLen = strlen(tmpStr);
    snprintf(tmpStr+curLen, sizeof(tmpStr)-curLen, "\tPatch:    %d\n", swVerExtended->patch);
    curLen = strlen(tmpStr);
    snprintf(tmpStr+curLen, sizeof(tmpStr)-curLen, "\tOptional: %d\n", swVerExtended->svnRev);
    curLen = strlen(tmpStr);
    if (swVerExtended->stack.applies)
    {
        snprintf(tmpStr+curLen, sizeof(tmpStr)-curLen, "\tStack:\n");
        curLen = strlen(tmpStr);
        snprintf(tmpStr+curLen, sizeof(tmpStr)-curLen, "\t\tInterface:\t%d\n", swVerExtended->stack.interface);
        curLen = strlen(tmpStr);
        snprintf(tmpStr+curLen, sizeof(tmpStr)-curLen, "\t\tNode:\t\t%d\n", swVerExtended->stack.node);
        curLen = strlen(tmpStr);
    }
    else
    {
        snprintf(tmpStr+curLen, sizeof(tmpStr)-curLen, "\tStack field doesn't apply (0x%02X)\n",
                ((uint8 *) swVerExtended)[offsetof(swVerExtended_t, stack)]);
        curLen = strlen(tmpStr);
    }
    if (swVerExtended->profiles.applies)
    {
        snprintf(tmpStr+curLen, sizeof(tmpStr)-curLen, "\tProfiles:\n");
        curLen = strlen(tmpStr);
        if (swVerExtended->profiles.zrc11)
        {
            snprintf(tmpStr+curLen, sizeof(tmpStr)-curLen, "\t\t%s\n", "ZRC 1.1");
            curLen = strlen(tmpStr);
        }
        if (swVerExtended->profiles.mso)
        {
            snprintf(tmpStr+curLen, sizeof(tmpStr)-curLen, "\t\t%s\n", "MSO");
            curLen = strlen(tmpStr);
        }
        if (swVerExtended->profiles.zrc20)
        {
            snprintf(tmpStr+curLen, sizeof(tmpStr)-curLen, "\t\t%s\n", "ZRC 2.0");
            curLen = strlen(tmpStr);
        }
    }
    else
    {
        snprintf(tmpStr+curLen, sizeof(tmpStr)-curLen, "\tProfiles field doesn't apply (0x%02X)\n",
                ((uint8 *) swVerExtended)[offsetof(swVerExtended_t, profiles)]);
        curLen = strlen(tmpStr);
    }
    if (swVerExtended->serial.applies)
    {
        snprintf(tmpStr+curLen, sizeof(tmpStr)-curLen, "\tSerial:\n");
        curLen = strlen(tmpStr);
        snprintf(tmpStr+curLen, sizeof(tmpStr)-curLen, "\t\tInterface:\t%d\n", swVerExtended->serial.interface);
        curLen = strlen(tmpStr);
        snprintf(tmpStr+curLen, sizeof(tmpStr)-curLen, "\t\tPort:\t\t%d\n", swVerExtended->serial.port);
        curLen = strlen(tmpStr);
        snprintf(tmpStr+curLen, sizeof(tmpStr)-curLen, "\t\tAlternative:\t%d\n", swVerExtended->serial.alternative);
    }
    else
    {
        snprintf(tmpStr+curLen, sizeof(tmpStr)-curLen, "\tSerial Interface field doesn't apply (0x%02X)\n",
                ((uint8 *) swVerExtended)[offsetof(swVerExtended_t, serial)]);
    }

    strncpy(retStr, tmpStr, maxStrLen);
}

int appGetAndPrintExtendedSoftwareVersion(swVerExtended_t *swVerExtended)
{
    int retVal = RTI_SUCCESS;
    if (retVal == RTI_ReadItem(RTI_CONST_ITEM_EXTENDED_SW_VERSION, 8, (uint8*)swVerExtended))
    {
        char tmpStrForTimePrint[1024];
        SoftwareVersionToString(tmpStrForTimePrint, sizeof(tmpStrForTimePrint), swVerExtended);
        LOG_INFO("[Initialization] - %s\n", tmpStrForTimePrint);
    }

    uint16 shortAddr = 0;
    retVal = RTI_ReadItemEx (RTI_PROFILE_RTI, RTI_SA_ITEM_SHORT_ADDRESS, 2, (uint8*)&shortAddr);
    LOG_INFO("Short Address:\t 0x%4X\n", shortAddr);
    uint16 panId = 0;
    retVal = RTI_ReadItemEx (RTI_PROFILE_RTI, RTI_SA_ITEM_PAN_ID, 2, (uint8*)&panId);
    LOG_INFO("PAN ID:\t\t 0x%4X\n", panId);

    return retVal;
}

static int appGetAndPrintSoftwareVersions(uint8 *baseVersion, swVerExtended_t *swVerExtended)
{
    int result;

    if (RTI_SUCCESS != (result = RTI_ReadItem(RTI_CONST_ITEM_SW_VERSION, 1, baseVersion)))
    {
        *baseVersion = 0;
        LOG_ERROR("Failed to read Base Software Version.\n");
    }
    else
    {
        LOG_INFO("Base Software Version = %u.%u.%u (0x%02x)\n",
                    (*baseVersion & 0xE0) >> 5,
                    (*baseVersion & 0x1C) >> 2,
                    (*baseVersion & 0x03),
                    *baseVersion);
    }

    result = appGetAndPrintExtendedSoftwareVersion(swVerExtended);
    if (RTI_SUCCESS != result)
    {
        memset(swVerExtended, 0, sizeof(*swVerExtended));
    }

    return result;
}

void DisplayPairingTable(rcnNwkPairingEntry_t *pEntry)
{
	int j, strLen = 0;
	char tmpStr[512];
	LOG_INFO("*************************************\n");
	LOG_INFO("* Pairing Index: \t 	0x%.2X\n", pEntry->pairingRef);
	LOG_INFO("* SRC NWK Address: \t 	0x%.4hX\n", pEntry->srcNwkAddress);
	LOG_INFO("* Logical Channel: \t 	0x%.2hX\n", pEntry->logicalChannel); //((pEntry->logicalChannel >> 8) & 0x00FF));
	snprintf(tmpStr+strLen, sizeof(tmpStr)-strLen, "%.2X", (pEntry->ieeeAddress[SADDR_EXT_LEN - 1] & 0x00FF));
	strLen += 2;
	for (j = (SADDR_EXT_LEN - 2); j >= 0; j--)
	{
		snprintf(tmpStr+strLen, sizeof(tmpStr)-strLen, ":%.2X", (pEntry->ieeeAddress[j] & 0x00FF));
		strLen += 3;
	}
	LOG_INFO("* IEEE Address:  		%s\n", tmpStr);
	LOG_INFO("* PAN Id: \t \t \t0x%.4X\n", pEntry->panId);
	LOG_INFO("* NWK Address: \t 	0x%.4hX\n", pEntry->nwkAddress);
	LOG_INFO("* Rec Capabilities: \t 	0x%.2hX\n",
			(pEntry->recCapabilities & 0x00FF));
	LOG_INFO("* Security Key Valid: \t0x%.2hX\n",
			(pEntry->securityKeyValid & 0x00FF));
	LOG_INFO("* Vendor Identifier: \t \t0x%.4hX\n",
			pEntry->vendorIdentifier);
	LOG_INFO("* Device Type List: \t 	[0x%.2hX, 0x%.2hX, 0x%.2hX]\n",
			(pEntry->devTypeList[0] & 0x00FF),
			(pEntry->devTypeList[1] & 0x00FF),
			(pEntry->devTypeList[2] & 0x00FF));
	LOG_INFO("* Received Frame Counter: \t0x%.8X (%u)\n",
			pEntry->recFrameCounter, pEntry->recFrameCounter);
	LOG_INFO("* Profiles Discovered: \t0x%.4X\n",
			pEntry->profileDiscs[0]);
}
