/**************************************************************************************************
 Filename:       lnxsample_main_target.cpp

 Description:    Linux Host application

   Copyright (C) {YEAR} Texas Instruments Incorporated - http://www.ti.com/


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
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <getopt.h>
#include <pthread.h>
#include <poll.h>

#include <sys/time.h>

#include "common_app.h"

#include "simple_app_main.h"
#include "simple_app.h"

// Linux surrogate interface, including both RTIS and NPI
#include "npi_ipc_client.h"

#include "hal_rpc.h"
#include "npi_lnx_ipc_rpc.h"

#define SB_DST_ADDR_DIV                    4

// macros
#define PAUSE() { fflush(stdout); while (getchar()!='\n'); }

#ifdef __BIG_DEBUG__
#define debug_printf(fmt, ...) printf( fmt, ##__VA_ARGS__)
#else
#define debug_printf(fmt, ...)
#endif

// Application state variable
uint8 appState;


struct timeval curTime, startTime, prevTimeSend, prevTimeRec, prevTime;

// Pairing reference
uint8 destIdx;


// ch from console
char ch;
// string from console
char str[128];

// Toggle Timer Print on Server state variable
static uint8 toggleTimerPrintOnServer = FALSE;
static uint8 toggleBigDebugPrintOnServer = FALSE;

// RNP in Standby
uint8 appRNPpowerState = RTI_APP_RNP_POWER_STATE_ACTIVE;

rcnNwkPairingEntry_t pEntryForRecovery;

//THREAD and MUTEX 
static pthread_mutex_t appThreadMutex;
static pthread_mutex_t appInitMutex;

static pthread_t AppThreadId;
static void *appThreadFunc(void *ptr);
static int appThreadTerminate;

static void appProcessEvents(uint16 events);

uint8 RTI_app_threadId;

#define NAME_ELEMENT(element) [element] = #element
static const char * const profile_list[RTI_PROFILE_ID_END + 1] =
		{ [0 ... 2] = NULL, NAME_ELEMENT(RTI_PROFILE_ZRC),
				NAME_ELEMENT(RTI_PROFILE_ZID), };

static const char * const vendor_list[7 + 1] = { //7 Elements for now
		[0 ... 7] = NULL, NAME_ELEMENT(RTI_VENDOR_TEXAS_INSTRUMENTS),
		//NAME_ELEMENT(RTI_PROFILE_ZID),
		};

static const char * const Command_list[5 + 1] = { //5 Elements for now
		[0 ... 3] = NULL, NAME_ELEMENT(RTI_CERC_USER_CONTROL_PRESSED),
				NAME_ELEMENT(RTI_CERC_USER_CONTROL_REPEATED),
				NAME_ELEMENT(RTI_CERC_USER_CONTROL_RELEASED),
				NAME_ELEMENT(RTI_CERC_COMMAND_DISCOVERY_REQUEST),
				NAME_ELEMENT(RTI_CERC_COMMAND_DISCOVERY_RESPONSE), };

const char * const AppState_list[AP_STATE_SIMPLE_TEST_MODE + 1] = { // Application States
		[0 ... AP_STATE_SIMPLE_TEST_MODE] = NULL, NAME_ELEMENT(AP_STATE_INIT),
		NAME_ELEMENT(AP_STATE_INIT_COLD),
		NAME_ELEMENT(AP_STATE_READY),
		NAME_ELEMENT(AP_STATE_PAIR),
		NAME_ELEMENT(AP_STATE_UNPAIR),
		NAME_ELEMENT(AP_STATE_NDATA),
		NAME_ELEMENT(AP_STATE_NDATA_PREPARE),
		NAME_ELEMENT(AP_STATE_WAIT_WAKEUP_FOR_PAIR),
		NAME_ELEMENT(AP_STATE_WAIT_WAKEUP_FOR_UNPAIR),
		NAME_ELEMENT(AP_STATE_WAIT_WAKEUP_FOR_NDATA),
		NAME_ELEMENT(AP_STATE_SIMPLE_TEST_MODE)
};

static const char * const ZRC_Key_list[255 + 1] =
{ //255 Elements for now
		[0 ... 255] = NULL,
		NAME_ELEMENT(RTI_CERC_NUM_1),
		NAME_ELEMENT(RTI_CERC_NUM_2),
		NAME_ELEMENT(RTI_CERC_NUM_3),
		NAME_ELEMENT(RTI_CERC_NUM_4),
		NAME_ELEMENT(RTI_CERC_NUM_5),
		NAME_ELEMENT(RTI_CERC_NUM_6),
		NAME_ELEMENT(RTI_CERC_NUM_6),
		NAME_ELEMENT(RTI_CERC_NUM_7),
		NAME_ELEMENT(RTI_CERC_NUM_8),
		NAME_ELEMENT(RTI_CERC_NUM_9),
		NAME_ELEMENT(RTI_CERC_NUM_0),
		NAME_ELEMENT(RTI_CERC_SELECT),
		NAME_ELEMENT(RTI_CERC_UP),
		NAME_ELEMENT(RTI_CERC_DOWN),
		NAME_ELEMENT(RTI_CERC_LEFT),
		NAME_ELEMENT(RTI_CERC_RIGHT),
		NAME_ELEMENT(RTI_CERC_RIGHT_UP),
		NAME_ELEMENT(RTI_CERC_RIGHT_DOWN),
		NAME_ELEMENT(RTI_CERC_LEFT_UP),
		NAME_ELEMENT(RTI_CERC_LEFT_DOWN),
		NAME_ELEMENT(RTI_CERC_ROOT_MENU),
		NAME_ELEMENT(RTI_CERC_SETUP_MENU),
		NAME_ELEMENT(RTI_CERC_CONTENTS_MENU),
		NAME_ELEMENT(RTI_CERC_FAVORITE_MENU),
		NAME_ELEMENT(RTI_CERC_EXIT),
		NAME_ELEMENT(RTI_CERC_NUM_11),
		NAME_ELEMENT(RTI_CERC_NUM_12),
		NAME_ELEMENT(RTI_CERC_NUM_2),
		NAME_ELEMENT(RTI_CERC_NUM_1),
		NAME_ELEMENT(RTI_CERC_NUM_0),
		NAME_ELEMENT(RTI_CERC_NUM_3),
		NAME_ELEMENT(RTI_CERC_NUM_4),
		NAME_ELEMENT(RTI_CERC_NUM_5),
		NAME_ELEMENT(RTI_CERC_NUM_6),
		NAME_ELEMENT(RTI_CERC_NUM_7),
		NAME_ELEMENT(RTI_CERC_NUM_8),
		NAME_ELEMENT(RTI_CERC_NUM_9),
		NAME_ELEMENT(RTI_CERC_DOT),
		NAME_ELEMENT(RTI_CERC_ENTER),
		NAME_ELEMENT(RTI_CERC_CLEAR),
		NAME_ELEMENT(RTI_CERC_NEXT_FAVORITE),
		NAME_ELEMENT(RTI_CERC_CHANNEL_UP),
		NAME_ELEMENT(RTI_CERC_CHANNEL_DOWN),
		NAME_ELEMENT(RTI_CERC_PREVIOUS_CHANNEL),
		NAME_ELEMENT(RTI_CERC_SOUND_SELECT),
		NAME_ELEMENT(RTI_CERC_INPUT_SELECT),
		NAME_ELEMENT(RTI_CERC_DISPLAY_INFORMATION),
		NAME_ELEMENT(RTI_CERC_HELP),
		NAME_ELEMENT(RTI_CERC_PAGE_UP),
		NAME_ELEMENT(RTI_CERC_PAGE_DOWN),
		NAME_ELEMENT(RTI_CERC_POWER),
		NAME_ELEMENT(RTI_CERC_VOLUME_UP),
		NAME_ELEMENT(RTI_CERC_VOLUME_DOWN),
		NAME_ELEMENT(RTI_CERC_MUTE),
		NAME_ELEMENT(RTI_CERC_PLAY),
		NAME_ELEMENT(RTI_CERC_STOP),
		NAME_ELEMENT(RTI_CERC_PAUSE),
		NAME_ELEMENT(RTI_CERC_RECORD),
		NAME_ELEMENT(RTI_CERC_REWIND),
		NAME_ELEMENT(RTI_CERC_FAST_FORWARD),
		NAME_ELEMENT(RTI_CERC_EJECT),
		NAME_ELEMENT(RTI_CERC_FORWARD),
		NAME_ELEMENT(RTI_CERC_BACKWARD),
		NAME_ELEMENT(RTI_CERC_STOP_RECORD),
		NAME_ELEMENT(RTI_CERC_PAUSE_RECORD),
		NAME_ELEMENT(RTI_CERC_ANGLE),
		NAME_ELEMENT(RTI_CERC_SUB_PICTURE),
		NAME_ELEMENT(RTI_CERC_VIDEO_ON_DEMAND),
		NAME_ELEMENT(RTI_CERC_ELECTRONIC_PROGRAM_GUIDE),
		NAME_ELEMENT(RTI_CERC_TIMER_PROGRAMMING),
		NAME_ELEMENT(RTI_CERC_INITIAL_CONFIGURATION),
		NAME_ELEMENT(RTI_CERC_PLAY_FUNCTION),
		NAME_ELEMENT(RTI_CERC_PAUSE_PLAY_FUNCTION),
		NAME_ELEMENT(RTI_CERC_RECORD_FUNCTION),
		NAME_ELEMENT(RTI_CERC_PAUSE_RECORD_FUNCTION),
		NAME_ELEMENT(RTI_CERC_STOP_FUNCTION),
		NAME_ELEMENT(RTI_CERC_MUTE_FUNCTION),
		NAME_ELEMENT(RTI_CERC_RESTORE_VOLUME_FUNCTION),
		NAME_ELEMENT(RTI_CERC_TUNE_FUNCTION),
		NAME_ELEMENT(RTI_CERC_SELECT_MEDIA_FUNCTION),
		NAME_ELEMENT(RTI_CERC_SELECT_AV_INPUT_FUNCTION),
		NAME_ELEMENT(RTI_CERC_SELECT_AUDIO_INPUT_FUNCTION),
		NAME_ELEMENT(RTI_CERC_POWER_TOGGLE_FUNCTION),
		NAME_ELEMENT(RTI_CERC_POWER_OFF_FUNCTION),
		NAME_ELEMENT(RTI_CERC_POWER_ON_FUNCTION),
		NAME_ELEMENT(RTI_CERC_F1_BLUE),
		NAME_ELEMENT(RTI_CERC_F2_RED),
		NAME_ELEMENT(RTI_CERC_F3_GREEN),
		NAME_ELEMENT(RTI_CERC_F4_YELLOW),
		NAME_ELEMENT(RTI_CERC_F5),
		NAME_ELEMENT(RTI_CERC_DATA),
		NAME_ELEMENT(RTI_CERC_RESERVED_1)
};


static const char * const RNPpowerState_list[4 + 1] = { //4 Application States
		[0 ... 4] = NULL, NAME_ELEMENT(RTI_APP_RNP_POWER_STATE_ACTIVE),
		NAME_ELEMENT(RTI_APP_RNP_POWER_STATE_NPI_SLEEP),
		NAME_ELEMENT(RTI_APP_RNP_POWER_STATE_STANDBY_ACTIVE),
		NAME_ELEMENT(RTI_APP_RNP_POWER_STATE_STANDBY_SLEEP)
};


// Application Test Mode States
enum {
	APP_CFG_STATE_INIT,				// Initial Application Sub State for Configuration
	APP_CFG_STATE_NODE_CAP,			// Sub state for configuration of Node Capabilities
	APP_CFG_STATE_PROFILES,			// Sub state for configuration of profiles
	APP_CFG_STATE_DEV_TYPES,		// Sub state for configuration of supported devices types
	APP_CFG_STATE_TGT_TYPES,		// Sub state for configuration of supported target types
	APP_CFG_STATE_VENDOR_ID,		// Sub state for configuration of vendor ID
	APP_CFG_STATE_VENDOR_STRING,	// Sub state for configuration of vendor string
	APP_CFG_STATE_USER_SRTING,		// Sub state for configuration of user string
	APP_CFG_STATE_KEY_SEED,			// Sub state for configuration of the number of key seeds used during secured pairing
};

appDevInfo_t appCFGParam;
uint8 appCFGstate;

appSendData_t appSendData_s;
uint8 appSendDataState;
void appSendDataProcessKey (char* strIn);

static void appSetCFGParamOnRNP(void);
static void appGetCFGParamFromRNP(void);
static void appInitConfigParam( char tgtSelection );
static void appConfigParamProcessKey(char* strIn);
static void appInitSyncRes(void);
void appDisplayPairingTable(void);
void appClearPairingTable(void);


int appInit(int mode, char threadId) 
{
	appThreadTerminate = 0;
	appInitSyncRes();

	uint8 value[2];
	RTI_app_threadId = threadId;

	printf("\n-------------------- START TURN ON DEBUG TRACES -------------------\n");
	npiMsgData_t pMsg;
	pMsg.len = 1;
	pMsg.subSys = RPC_SYS_SRV_CTRL | RPC_CMD_SREQ;
	pMsg.cmdId = NPI_LNX_CMD_ID_CTRL_BIG_DEBUG_PRINT_REQ;

	// Turn timer debug OFF
	toggleBigDebugPrintOnServer = FALSE;

	pMsg.pData[0] = toggleBigDebugPrintOnServer;

	// send debug flag value
	NPI_SendSynchData( &pMsg );
	if (RTI_SUCCESS == pMsg.pData[0])
	{
		printf("__BIG_DEBUG_ACTIVE set to: 0x%.2X\n", toggleBigDebugPrintOnServer);
	}

	pMsg.cmdId = NPI_LNX_CMD_ID_CTRL_TIME_PRINT_REQ;

	// Turn timer debug OFF
	toggleTimerPrintOnServer = FALSE;

	pMsg.pData[0] = toggleTimerPrintOnServer;

	// send debug flag value
	NPI_SendSynchData( &pMsg );
	if (RTI_SUCCESS == pMsg.pData[0])
	{
		printf("__DEBUG_TIME_ACTIVE set to: 0x%.2X\n", toggleTimerPrintOnServer);
	}
	printf("\n-------------------- END TURN ON DEBUG TRACES -------------------\n\n");

	printf("\n-------------------- START SOFTWARE VERSION READING-------------------\n");

	if (RTI_SUCCESS != RTI_ReadItem(RTI_CONST_ITEM_SW_VERSION, 1, value))
	{
		fprintf(stderr, "Failed to read Software Version.\n");

		fprintf(stderr, " Please check connection.\n");
		exit(-1);
	}
	else
	{
		printf("- Software Version = 0x%x\n", value[0]);
		printf("-------------------- END SOFTWARE VERSION READING-------------------\n");
	}


	// Setup default configuration
	appInitConfigParam('t');

	// Initialize default Send Data parameters
	appSendData_s.dstIndex = 0x00;
	appSendData_s.len = 2;
	appSendData_s.profileId = RTI_PROFILE_ZRC;
	appSendData_s.txOptions = RTI_TX_OPTION_ACKNOWLEDGED | RTI_TX_OPTION_VENDOR_SPECIFIC | RTI_TX_OPTION_SECURITY;
	appSendData_s.vendorId = RTI_VENDOR_TEXAS_INSTRUMENTS;
	appSendData_s.pData[0] = RTI_CERC_USER_CONTROL_PRESSED;
	appSendData_s.pData[1] = RTI_CERC_NUM_0;

	// TODO: it is ideal to make this thread higher priority
	// but linux does not allow realtime of FIFO scheduling policy for
	// non-priviledged threads.
	if (pthread_create(&AppThreadId, NULL, appThreadFunc, NULL)) 
	{
		// thread creation failed
		printf("Failed to create app thread\n");
		return -1;
	}

	return 0;
}

static void appInitSyncRes(void)
{
	// initialize all mutexes
	if (pthread_mutex_init(&appThreadMutex, NULL))
	{
		printf("Fail To Initialize Mutex appThreadMutex\n");
		exit(-1);
	}

	if (pthread_mutex_init(&appInitMutex, NULL))
	{
		printf("Fail To Initialize Mutex appInitMutex\n");
		exit(-1);
	}
}

static void *appThreadFunc(void *ptr)
{
	//uint8 readbuf[128];
	//uint8 pollStatus = FALSE;
	uint16 events = 0;

	/* lock mutex in order not to lose signal */
	pthread_mutex_lock(&appThreadMutex);


	// set sample application state to not ready until RTI Init is successfully confirmed
	appState = AP_STATE_INIT;

	printf("App Thread Started \n");


	// Display current configuration
	DispCFGCurrentCfg(appCFGParam);
	//Display menu...
	DispMenuInit();

	/* thread loop */
	while (!appThreadTerminate) 
	{
		// Wait for event
		sem_wait(&event_mutex);

		events = RTI_main_get_event(RTI_app_threadId);
		// Process events
		if (events != 0) {
			appProcessEvents(events);
			debug_printf("State: %s [0x%.2X]\n", AppState_list[appState], appState);
		}

		// Only process actions if there is a character available
		if (consoleInput.handle == RTI_MAIN_INPUT_READY)
		{
			ch = consoleInput.latestCh;
			strcpy(str, consoleInput.latestStr);

			// 'q' has highest priority
			if (appState == AP_STATE_INIT)
			{
				// Let user configure the RNP
				appConfigParamProcessKey(str);
			}
			else if (appState == AP_STATE_NDATA_PREPARE)
			{
				// Pass key onto Send Data processing
				appSendDataProcessKey(str);
			}
			// Other states
			else if (ch == '0')
			{
				// Enter Init state and allow user to configure
				appState = AP_STATE_INIT;
				// Display configuration menu
				DispMenuInit();
			}
			else if (ch == '1')
			{
				if (appState == AP_STATE_READY)
				{
					// Check if we're Target or Controller
					if (appCFGParam.nodeCapabilities & RCN_NODE_CAP_TARGET)
					{
						// Allow pairing
						printf("Calling RTI_AllowPairReq\n");
						RTI_AllowPairReq();
					}
					else // We must be Controller
					{
						// Pairing
						printf("Calling RTI_PairReq\n");
						RTI_PairReq();
					}

					// Go to Allow Pair state
					appState = AP_STATE_PAIR;
				}
				else
				{
					// Check if we're Target or Controller
					if (appCFGParam.nodeCapabilities & RCN_NODE_CAP_TARGET)
					{
						// Abort AllowPair
						printf("Calling RTI_AllowPairAbortReq\n");
						RTI_AllowPairAbortReq();
					}
					else // We must be Controller
					{
						// Abort Pair
						printf("Calling RTI_PairAbortReq\n");
						RTI_PairAbortReq();
					}

					// Return to READY state. There is no confirmation for this call, so
					// make sure you call it when NPI is active, as is done here.
					appState = AP_STATE_READY;
				}
			}
			else if (ch == '2')
			{
				if (appState == AP_STATE_READY)
				{
					appState = AP_STATE_UNPAIR;
					// Unpair
					printf("Calling RTI_UnpairReq  for index 0\n");
					RTI_UnpairReq(0);
				}
				else
				{
					// Simply remain in whatever state we're in
					printf("Cannot call RTI_UnpairReq, because we're in state: 0x%.2X\n", appState);
				}
			}
			else if (ch == '7')
			{
				if (appState == AP_STATE_READY)
				{
					// Go to AP_STATE_NDATA_PREPARE
					appState = AP_STATE_NDATA_PREPARE;
					DispSendDataMenu();
				}
				else
				{
					printf("ERR: Cannot send data in this state: %s [0x%.2X]\nRNP Power State: \t %s [0x%.2X]\n",
							AppState_list[appState], appState,
							RNPpowerState_list[appRNPpowerState], appRNPpowerState);
				}
			}
			else if (ch == '8')
			{
				// Read out and display all pairing entries
				appClearPairingTable();
			}
			else if (ch == '9')
			{
				// Read out and display all pairing entries
				appDisplayPairingTable();
			}
			else if (ch == 't')
			{
				npiMsgData_t pMsg;
				pMsg.len = 1;
				pMsg.subSys = RPC_SYS_SRV_CTRL | RPC_CMD_AREQ;
				pMsg.cmdId = NPI_LNX_CMD_ID_CTRL_TIME_PRINT_REQ;

				if (toggleTimerPrintOnServer == FALSE)
				{
					// Turn timer debug ON
					toggleTimerPrintOnServer = TRUE;
				}
				else
				{
					// Turn timer debug OFF
					toggleTimerPrintOnServer = FALSE;
				}

				pMsg.pData[0] = toggleTimerPrintOnServer;

				// send debug flag value
				NPI_SendAsynchData( &pMsg );
				if (RTI_SUCCESS == pMsg.pData[0])
				{
					printf("__DEBUG_TIME_ACTIVE set to: 0x%.2X\n", toggleTimerPrintOnServer);
				}
			}
			else if (ch == 'y')
			{
				npiMsgData_t pMsg;
				pMsg.len = 1;
				pMsg.subSys = RPC_SYS_SRV_CTRL | RPC_CMD_SREQ;
				pMsg.cmdId = NPI_LNX_CMD_ID_CTRL_BIG_DEBUG_PRINT_REQ;

				if (toggleBigDebugPrintOnServer == FALSE)
				{
					// Turn timer debug ON
					toggleBigDebugPrintOnServer = TRUE;
				}
				else
				{
					// Turn timer debug OFF
					toggleBigDebugPrintOnServer = FALSE;
				}

				pMsg.pData[0] = toggleBigDebugPrintOnServer;

				// send debug flag value
				NPI_SendSynchData( &pMsg );
				if (RTI_SUCCESS == pMsg.pData[0])
				{
					printf("__BIG_DEBUG_ACTIVE set to: 0x%.2X\n", toggleBigDebugPrintOnServer);
				}
			}
			else if (ch == 'a')
			{
				printf("Application State: \t %s [0x%.2X]\nRNP Power State: \t %s [0x%.2X]\n",
						AppState_list[appState], appState,
						RNPpowerState_list[appRNPpowerState], appRNPpowerState);
			}
			else if (ch == 'm')
			{
				// Display menu
				DispMenuReady();
			}
			else if (ch == 'r')
			{
				// Set INIT_STATE
				appState = AP_STATE_INIT_COLD;

				// Reset RNP
				RTI_SwResetReq();

				// Schedule event to Init RNP.
				RTI_main_start_timerEx(RTI_app_threadId, RTI_APP_EVT_INIT, 1200); // Retry initialization after 1.2 seconds
			}
			else if (ch == 'c')
			{
				if (appState == AP_STATE_READY)
				{
					appState = AP_STATE_SIMPLE_TEST_MODE;
					printf("Display messages for simple testing\n");
				}
				else if (appState == AP_STATE_SIMPLE_TEST_MODE)
				{
					appState = AP_STATE_READY;
					DispMenuReady();
					printf("Back to normal state\n");
				}
			}
			else if (ch == 'g')
			{
				// Get MAC Channel
				uint8 macChannel = 0;

				RTI_ReadItemEx(RTI_PROFILE_RTI, 0x61, 1, &macChannel); // RTI_SA_ITEM_CURRENT_CHANNEL == 0x61

				printf ("Current MAC Channel %.2d\n", macChannel);

//				// Get Latest Energy Samples
//				uint8 energySamples[20] = {0};
//
//				RTI_ReadItemEx(RTI_PROFILE_RTI, 0x8A, sizeof(energySamples), energySamples); // RCN_NIB_ENERGY_SAMPLES == 0x8A
//
//				printf ("[");
//				for (macChannel = 1; macChannel < (sizeof(energySamples)-1); macChannel++)
//				{
//					printf(" %d,", (91*energySamples[macChannel])/255 - 91);
//				}
//				printf(" %d]\n\n", (91*energySamples[sizeof(energySamples)-1])/255 - 91);
			}
			else if (ch != '\n')
			{
//				printf("unknown command %c (0x%.2X) \n", ch, ch);
//				DispMenuReady();
			}

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
static void appProcessEvents(uint16 events)
{
	uint16 procEvents = 0;
	int mutexRet = 0;

	if ((events & RTI_APP_EVT_DATA_RCV) != 0)
	{
		printf("State: %s [0x%.2X]\n", AppState_list[appState], appState);
		// Prepare to clear event
		procEvents |= RTI_APP_EVT_DATA_RCV;
	}
	if ((events & RTI_APP_EVT_INIT) != 0)
	{
		printf("State: %s [0x%.2X]\n", AppState_list[appState], appState);
		// Prepare to clear event
		procEvents |= RTI_APP_EVT_INIT;
		if (appState == AP_STATE_INIT_COLD)
		{
			uint8 startupFlg, retVal;

			startupFlg = CLEAR_STATE;
			retVal = RTI_WriteItem(RTI_CP_ITEM_STARTUP_CTRL, 1, &startupFlg);
			if (retVal != RTI_SUCCESS)
			{
				printf ("Could no set Cold Start flag\n");
			}
			else
			{
				printf ("Successfully set Cold Start flag\n");
			}
		}
		if (appCFGParam.nodeCapabilities & RCN_NODE_CAP_TARGET)
			appInitConfigParam('t');
		else
			appInitConfigParam('c');
		// Apply changes
		appSetCFGParamOnRNP();
		printf("Trying to initialize again, as %s\n",
				(appCFGParam.nodeCapabilities & RCN_NODE_CAP_TARGET) ? "Target" : "Controller");
		//When Launching the Thread, Mutex is unlocked.
		if ( (mutexRet = pthread_mutex_trylock(&appInitMutex)) == EBUSY)
		{
			debug_printf("[MUTEX] appInit Mutex busy\n");
		}
		else
		{
			debug_printf("[MUTEX] appInit Lock status: %d\n", mutexRet);
		}
		RTI_InitReq();

		//Mutex is unlocked only inside RTI_Initcnf();
		pthread_mutex_lock(&appInitMutex);

	}
	// Clear event
	RTI_main_clear_event(RTI_app_threadId, procEvents);
}

/**************************************************************************************************
 *
 * @fn      appClearPairingTable
 *
 * @brief   Function to clear pairing table
 *
 * @param   status - Result of RTI_InitReq API call.
 *
 * @return  void
 */
void appClearPairingTable()
{
	rcnNwkPairingEntry_t *pEntry;
	// Allocate memory for one pairing entry
	pEntry = (rcnNwkPairingEntry_t *) malloc(sizeof(rcnNwkPairingEntry_t));

	uint8 i, result, pairingTableSize;

	(void)RTI_ReadItemEx(RTI_PROFILE_RTI, RTI_CONST_ITEM_MAX_PAIRING_TABLE_ENTRIES, 1, (uint8*)&pairingTableSize);

	for (i = 0; i < pairingTableSize; i++)
	{
		// Set current pairing entry
		RTI_WriteItemEx(RTI_PROFILE_RTI, RTI_SA_ITEM_PT_CURRENT_ENTRY_INDEX, 1,
				(uint8 *) &i);
		// Try to read out this entry
		if ((result = RTI_ReadItemEx(RTI_PROFILE_RTI,
				RTI_SA_ITEM_PT_CURRENT_ENTRY,
				sizeof(rcnNwkPairingEntry_t),
				(uint8 *) pEntry)) == RTI_SUCCESS)
		{
			// Invalidate item
			pEntry->pairingRef = RTI_INVALID_PAIRING_REF;
			RTI_WriteItemEx(RTI_PROFILE_RTI,
				RTI_SA_ITEM_PT_CURRENT_ENTRY,
				sizeof(rcnNwkPairingEntry_t),
				(uint8 *) pEntry);
		}
	}

	printf("*************************************\n");
	printf("* Pairing Table Is Empty\n");
	printf("*************************************\n");

	// Free pairing entry buffer
	free(pEntry);
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
void appDisplayPairingTable() 
{
	rcnNwkPairingEntry_t *pEntry;
	// Allocate memory for one pairing entry
	pEntry = (rcnNwkPairingEntry_t *) malloc(sizeof(rcnNwkPairingEntry_t));

	uint8 i, result, atLeastOneEntryFound = 0, numOfEntries;

	RTI_ReadItemEx(RTI_PROFILE_RTI, RTI_CONST_ITEM_MAX_PAIRING_TABLE_ENTRIES,
			1, (uint8 *) &numOfEntries);

	printf("*************************************\n");
	printf("* Max number of pairing entries: %d\n", numOfEntries);
	for (i = 0; i < numOfEntries; i++)
	{
		// Set current pairing entry
		RTI_WriteItemEx(RTI_PROFILE_RTI, RTI_SA_ITEM_PT_CURRENT_ENTRY_INDEX, 1,
				(uint8 *) &i);
		// Try to read out this entry
		if ((result = RTI_ReadItemEx(RTI_PROFILE_RTI,
				RTI_SA_ITEM_PT_CURRENT_ENTRY, sizeof(rcnNwkPairingEntry_t),
				(uint8 *) pEntry)) == RTI_SUCCESS) {

			// Found pairing entry; display this.
			DisplayPairingTable(pEntry);

			atLeastOneEntryFound = 1;
		}
	}

	if (atLeastOneEntryFound != 0) 
	{
		printf("*************************************\n");
	}
	else
	{
		printf("*************************************\n");
		printf("* Pairing Table Is Empty\n");
		printf("*************************************\n");
	}

	// Free pairing entry buffer
	free(pEntry);
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
		printf("RTI_InitCnf(0x%.2X - %s)\n", status, rtiStatus_list[status]);

		uint8 startupFlg;
		startupFlg = RESTORE_STATE;
		RTI_WriteItem(RTI_CP_ITEM_STARTUP_CTRL, 1, &startupFlg);

		appState = AP_STATE_READY;

		// Get configuration parameters from RNP, to make sure we display the correct settings
		appGetCFGParamFromRNP();
		// Display what we have configured
		DispCFGCurrentCfg(appCFGParam);

		printf("Entered %s [0x%.2X]\n", AppState_list[appState], appState);
		//Display menu...
		DispMenuReady();

		//			 Schedule event to try again.
		//			RTI_main_start_timerEx(RTI_app_threadId, RTI_APP_EVT_INIT, 20); // Retry initialization after 20ms for stress testing
	}
	else
	{
		printf("ERR: RTI_InitCnf(0x%.2X - %s)\n", status, rtiStatus_list[status]);
		// Try to reset RNP
		RTI_SwResetReq();

		// Schedule event to try again.
		RTI_main_start_timerEx(RTI_app_threadId, RTI_APP_EVT_INIT, 1200); // Retry initialization after 1.2 seconds
//		exit(-1);
	}

	//Unlock Mutex So that Application can continue
	debug_printf("[MUTEX] Unlock appInit Mutex\n");
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
void RTI_PairCnf(rStatus_t status, uint8 dstIndex, uint8 devType) 
{
	// Return to READY STATE
	appState = AP_STATE_READY;

	if (status == RTI_SUCCESS)
	{
		printf("RTI_PairCnf(0x%.2X - %s), dstIndex: 0x%.2X, devType: %s[0x%.2X]\n",
				status,
				rtiStatus_list[status],
				dstIndex,
				(devType < RTI_DEVICE_TARGET_TYPE_END) ? rtiDevType_list[devType] : "Unknown",
						devType);
	}
	else
	{
		printf("ERR: RTI_PairCnf(0x%.2X - %s)\n", status, rtiStatus_list[status]);
	}
}

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
void RTI_AllowPairCnf(rStatus_t status, uint8 dstIndex, uint8 devType) 
{
	// set paring reference (destination index)
	destIdx = dstIndex;

	// Return to READY STATE
	appState = AP_STATE_READY;

	if (0 == status)
	{
		printf("\tPaired!! Waiting for data from RC, press the <q> key followed by <enter> at any time to quit\n\n");
	}
}

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
		printf("ERR: RTI_SendDataCnf(0x%.2X - %s), but sent in wrong state: %s\n",
				status,
				rtiStatus_list[status],
				AppState_list[appState]);
	}
	else if (appState == AP_STATE_NDATA)
	{
		if (status == RTI_SUCCESS)
			printf("Data sent successfully\n");
		else
			printf("ERR: RTI_SendDataCnf(0x%.2X - %s)\n", status, rtiStatus_list[status]);
		// Return to Send Data Prepare state
		appState = AP_STATE_NDATA_PREPARE;
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
void RTI_StandbyCnf(rStatus_t status)
{
	if (status == RTI_SUCCESS)
	{
		if (appRNPpowerState & RTI_APP_RNP_POWER_STATE_STANDBY_BIT)
		{
			printf("\nExited Standby\n");
			appRNPpowerState &= ~(RTI_APP_RNP_POWER_STATE_STANDBY_BIT);
		}
		else
		{
			printf("\nEntered Standby\n");
			appRNPpowerState |= RTI_APP_RNP_POWER_STATE_STANDBY_BIT;

			// Enable Sleep here, NPI is not responsive after this
			RTI_EnableSleepReq();
		}
	}
	else
	{
		printf("ERR: RTI_StandbyCnf(0x%.2X - %s)\n", status, rtiStatus_list[status]);
	}
}

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
		uint8 rxLQI, uint8 rxFlags, uint8 len, uint8 *pData) {
	int i, error = FALSE;
	static uint8 lastSource = RTI_INVALID_PAIRING_REF;

	if ( appState != AP_STATE_SIMPLE_TEST_MODE)
	{
		//check Basic Range to avoid Seg Fault
		if ((profileId < 0) || (profileId > RTI_PROFILE_ID_END))
			error = TRUE;

		//Vendor Id Meaningful only if Vendor Specific Data
		if (rxFlags & RTI_RX_FLAGS_VENDOR_SPECIFIC)
			if ((vendorId < 0) || (vendorId > 7))
				error = TRUE;

		if (RTI_PROFILE_ZRC == profileId)
		{
			if (error)
			{
				printf("UNDEFINED PACKET Source Idx: %d, profileId %d , vendorId: %d , rxFlags: 0x%x, rxLQI %d \n",
						srcIndex, profileId, vendorId, rxFlags, rxLQI);
				profileId = 0xff;
			} else
			{
				printf("Source Idx: %d, profileId %d (%s), vendorId: %d (%s), rxFlags 0x%x, rxLQI %d \n",
						srcIndex, profileId,
						profile_list[profileId] ? profile_list[profileId] : "?",
								vendorId,
								(rxFlags & RTI_RX_FLAGS_VENDOR_SPECIFIC) ?
										(vendor_list[vendorId] ? vendor_list[vendorId] : "?") :
										"N/A", rxFlags, rxLQI);
			}

			if ((pData[0] < 0) || (pData[0] > 5))
				error = TRUE;
			if ((pData[0] < 0) || (pData[0] > 255))
				error = TRUE;
			if (error)
			{
				printf("Raw ZRC Data: ");
				for (i = 0; i < len; i++)
					printf("%d ", pData[i]);
				printf("\n");
			} else
				printf("ZRC Data: Cmd: %d (%s), Key %d (%s) \n", pData[0],
						Command_list[pData[0]] ? Command_list[pData[0]] : "?",
								pData[1],
								ZRC_Key_list[pData[1]] ? ZRC_Key_list[pData[1]] : "?");
		}
		else
		{
			printf("Raw Data: ");
			for (i = 0; i < len; i++)
				printf("%d ", pData[i]);
			printf("\n");
		}
	}
	else if ( appState == AP_STATE_SIMPLE_TEST_MODE )
	{
		if (srcIndex != lastSource)
			printf("\n");
		printf("Source Idx: %d, profileId %d (%s), rxLQI %d (-%3d dBm)",
				srcIndex, profileId,
				profile_list[profileId] ? profile_list[profileId] : "?",
						rxLQI, (0xFF - rxLQI)/2);

		printf("\tData: ");
		for (i = 0; i < len; i++)
			printf("%d ", pData[i]);
		printf("\n");
	}

	lastSource = srcIndex;

//	//For Debug Purpose!!!
//	if (error)
//		exit(-1);
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
void RTI_RxEnableCnf(rStatus_t status)
{
	if (status == RTI_SUCCESS)
		printf("RTI_RxEnableCnf(0x%.2X - %s)\n", status, rtiStatus_list[status]);
	else
		printf("ERR: RTI_RxEnableCnf(0x%.2X - %s)\n", status, rtiStatus_list[status]);
}

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
void RTI_EnableSleepCnf(rStatus_t status)
{
	if (status == RTI_SUCCESS)
	{
		printf("\nSleep enabled\n");
		appRNPpowerState |= RTI_APP_RNP_POWER_STATE_NPI_BIT;
	}
	else
	{
		printf("ERR: RTI_EnableSleepCnf(0x%.2X - %s)\n", status, rtiStatus_list[status]);
	}
}

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
void RTI_DisableSleepCnf(rStatus_t status)
{
	if (status == RTI_SUCCESS)
	{
		printf("\nSleep disabled\n");
		appRNPpowerState &= ~(RTI_APP_RNP_POWER_STATE_NPI_BIT);
		// Toggle StandBy mode
		if (appRNPpowerState & RTI_APP_RNP_POWER_STATE_STANDBY_BIT)
		{
			// We are in Standby, disable it
			RTI_StandbyReq(RTI_STANDBY_OFF);
		}
		else
		{
			printf("Already exited Standby\n");
		}
	}
	else
	{
		printf("ERR: RTI_DisableSleepCnf(0x%.2X - %s)\n", status, rtiStatus_list[status]);
	}
}

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
void RTI_UnpairInd(uint8 dstIndex)
{
	printf("RTI_UnpairInd, dstIndex : %d \n", dstIndex);
}

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
void RTI_PairAbortCnf(rStatus_t status)
{
	if (status == RTI_SUCCESS)
	{
		printf("RTI_PairAbortCnf(0x%.2X - %s)\n", status, rtiStatus_list[status]);
	}
	else
	{
		printf("ERR: RTI_DisableSleepCnf(0x%.2X - %s)\n", status, rtiStatus_list[status]);
	}
}


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
void RTI_UnpairCnf(rStatus_t status, uint8 dstIndex)
{
	if (appState == AP_STATE_UNPAIR)
	{
		appState = AP_STATE_READY;
		printf("RTI_UnpairCnf(0x%.2X - %s), dstIndex: 0x%2X\n",
				status,
				rtiStatus_list[status],
				dstIndex);
	}
	else
	{
		printf("ERR: RTI_UnpairCnf(0x%.2X - %s), dstIndex : %d , \n\tcalled from illegal state: %s\n",
				status,
				rtiStatus_list[status],
				dstIndex,
				AppState_list[appState]);
	}
}

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
void RTI_ResetInd( void )
{
	int mutexRet;

	if (appState == AP_STATE_INIT_COLD)
	{
		uint8 startupFlg, retVal;
		startupFlg = CLEAR_STATE;
		retVal = RTI_WriteItem(RTI_CP_ITEM_STARTUP_CTRL, 1, &startupFlg);
		if (retVal != RTI_SUCCESS)
		{
			printf ("Could no set Cold Start flag\n");
		}
		else
		{
			printf ("Successfully set Cold Start flag\n");
		}

		appState = AP_STATE_INIT;

		// Disable event to initialize since we caught the reset from the RNP,
		// note that this is a new feature as of RemoTI-1.3.1
		RTI_main_start_timerEx(RTI_app_threadId, RTI_APP_EVT_INIT, 0);
	}
	else
	{
		// Warn user of reset
		printf("[WARNING] RNP reset unexpectedly\n");
	}

	// RNP is now back in default state
	appRNPpowerState = RTI_APP_RNP_POWER_STATE_ACTIVE;

	// Initialize node and RF4CE stack
	printf("Calling RTI_InitReq...\n");
	//When Launching the Thread, Mutex is unlocked.
	debug_printf("[MUTEX] Lock appInit Mutex\n");
	if ( (mutexRet = pthread_mutex_trylock(&appInitMutex)) == EBUSY)
	{
		debug_printf("[MUTEX] appInit Mutex busy\n");
	}
	else
	{
		debug_printf("[MUTEX] appInit Lock status: %d\n", mutexRet);
	}
	RTI_InitReq();
	printf("...Waiting for RTI_InitCnf. (can take up to 6s if cold start and target RNP)...\n");
}


// List of supported target device types: maximum up to 6 device types.
static uint8 tgtListCTL[RTI_MAX_NUM_SUPPORTED_TGT_TYPES] =
{
		RTI_DEVICE_TELEVISION, RTI_DEVICE_SET_TOP_BOX,
		RTI_DEVICE_MEDIA_CENTER_PC, RTI_DEVICE_RESERVED_INVALID,
		RTI_DEVICE_RESERVED_INVALID, RTI_DEVICE_RESERVED_INVALID
};
// List of supported target device types: maximum up to 6 device types.
static uint8 tgtListTGT[RTI_MAX_NUM_SUPPORTED_TGT_TYPES] =
{
		RTI_DEVICE_REMOTE_CONTROL, RTI_DEVICE_RESERVED_INVALID,
		RTI_DEVICE_RESERVED_INVALID, RTI_DEVICE_RESERVED_INVALID,
		RTI_DEVICE_RESERVED_INVALID, RTI_DEVICE_RESERVED_INVALID
};

// List of implemented device types: maximum up to 3 device types.
static uint8 devListCTL[RTI_MAX_NUM_DEV_TYPES] =
{
		RTI_DEVICE_REMOTE_CONTROL,
		RTI_DEVICE_RESERVED_INVALID, RTI_DEVICE_RESERVED_INVALID };
// List of implemented device types: maximum up to 3 device types.
static uint8 devListTGT[RTI_MAX_NUM_DEV_TYPES] =
{
		RTI_DEVICE_TELEVISION,
		RTI_DEVICE_RESERVED_INVALID, RTI_DEVICE_RESERVED_INVALID };

// List of implemented device types: maximum up to 3 device types.
static uint8 profileList[RTI_MAX_NUM_PROFILE_IDS] =
{
		RTI_PROFILE_ZRC, RTI_PROFILE_ZID, 0, 0, 0, 0, 0
};
static uint8 vendorName[RTI_VENDOR_STRING_LENGTH] = "TI-LPRF";

static void appConfigParamProcessKey(char* strIn)
{
	int tmp, i, mutexRet;
	uint8 pData[1];
	char* pStr;
	switch (appCFGstate)
	{
	case APP_CFG_STATE_INIT:
		switch (ch)
		{
		case '1':
			// Toggle Node type bit
			appCFGParam.nodeCapabilities ^= RTI_NODE_CAP_NODE_TYPE_BM;
			break;
		case '2':
			// Enter sub state to configure profiles
			appCFGstate = APP_CFG_STATE_NODE_CAP;
			DispCFGNodeCapMenu();
			break;
		case '3':
			// Enter sub state to configure profiles
			appCFGstate = APP_CFG_STATE_PROFILES;
			DispCFGProfileIDMenu();
			break;
		case '4':
			// Enter sub state to configure device types
			appCFGstate = APP_CFG_STATE_DEV_TYPES;
			DispCFGdevTypesMenu();
			break;
		case '5':
			// Enter sub state to configure target types
			appCFGstate = APP_CFG_STATE_TGT_TYPES;
			DispCFGtgtTypesMenu();
			break;
		case 'k':
			// Enter number of key seeds
			appCFGstate = APP_CFG_STATE_KEY_SEED;
			printf("Set # key seed\n");
			break;
		case 'i':
			// Initialize without configuration
			RTI_InitReq();
			printf("...Waiting for RTI_InitCnf. (can take up to 6s if cold start and target RNP)...\n");
			break;
		case 'g':
			// Get configuration parameters from RNP, to make sure we display the correct settings
			appGetCFGParamFromRNP();
			// Display what we have configured
			DispCFGCurrentCfg(appCFGParam);
			break;
		case 'r':
			appState = AP_STATE_READY;

			printf("Entered %s [0x%.2X]\n", AppState_list[appState], appState);
			//Display menu...
			DispMenuReady();
			break;
		case 's':
			// Apply changes
			appSetCFGParamOnRNP();

			// Initialize node and RF4CE stack
			printf("Calling RTI_InitReq...\n");
			//When Launching the Thread, Mutex is unlocked.
			//When Launching the Thread, Mutex is unlocked.
			debug_printf("[MUTEX] Lock appInit Mutex\n");
			if ( (mutexRet = pthread_mutex_trylock(&appInitMutex)) == EBUSY)
			{
				debug_printf("[MUTEX] appInit Mutex busy\n");
			}
			else
			{
				debug_printf("[MUTEX] appInit Lock status: %d\n", mutexRet);
			}
			RTI_InitReq();
			printf("...Waiting for RTI_InitCnf. (can take up to 6s if cold start and target RNP)...\n");

			break;
		case 'c':
		case 't':
			// Set Default settings
			appInitConfigParam(ch);

			// Apply changes
			appSetCFGParamOnRNP();

			// Initialize node and RF4CE stack
			printf("Calling RTI_InitReq...\n");
			//When Launching the Thread, Mutex is unlocked.
			debug_printf("[MUTEX] Lock appInit Mutex\n");
			if ( (mutexRet = pthread_mutex_trylock(&appInitMutex)) == EBUSY)
			{
				debug_printf("[MUTEX] appInit Mutex busy\n");
			}
			else
			{
				debug_printf("[MUTEX] appInit Lock status: %d\n", mutexRet);
			}
			RTI_InitReq();
			printf("...Waiting for RTI_InitCnf. (can take up to 6s if cold start and target RNP)...\n");

			break;
		case 'l':
			// List current chosen configuration (not the one programmed to RNP)
			DispCFGCurrentCfg(appCFGParam);
			break;
		default:
			appCFGstate = APP_CFG_STATE_INIT;
			DispMenuInit();
			break;
		}
		break;

	case APP_CFG_STATE_NODE_CAP:
		// use strtok to split string and process each individual Profile ID
		i = 0;
		// Get first token
		pStr = strtok (strIn, " ,:;-|");
		while ( pStr != NULL)
		{
			// Convert string to int
			tmp = atoi(pStr);
			// OR in node capabilities
			if (tmp)
				appCFGParam.nodeCapabilities |= BV(i);
			else
				appCFGParam.nodeCapabilities &= ~(uint8)(BV(i));
			i++;
			if (i >= 4)
				break;
			// Now get next token
			pStr = strtok (NULL, " ,:;-|");
		}
		// Display current configuration to give feedback to user about the changes
		DispCFGCurrentCfg(appCFGParam);
		// Return to CFG state
		appCFGstate = APP_CFG_STATE_INIT;
		DispMenuInit();
		break;
	case APP_CFG_STATE_PROFILES:
		// use strtok to split string and process each individual Profile ID
		i = 0;
		// Get first token
		pStr = strtok (strIn, " ,:;-|");
		while ( pStr != NULL)
		{
			// Convert string to int
			tmp = atoi(pStr);
			// Add it to the list
			appCFGParam.profileIdList[i] = (uint8) tmp;
			i++;
			if (i >= RCN_MAX_NUM_PROFILE_IDS)
				break;
			// Now get next token
			pStr = strtok (NULL, " ,:;-|");
		}
		// Update the number of supported Profile IDs (max 7)
		RCN_APP_CAPA_SET_NUM_PROFILES(appCFGParam.appCapabilities, i);
		// Display current configuration to give feedback to user about the changes
		DispCFGCurrentCfg(appCFGParam);
		// Return to CFG state
		appCFGstate = APP_CFG_STATE_INIT;
		DispMenuInit();
		break;
	case APP_CFG_STATE_DEV_TYPES:
		// use strtok to split string and process each individual Profile ID
		i = 0;
		// Get first token
		pStr = strtok (strIn, " ,:;-|");
		while ( pStr != NULL)
		{
			// Convert string to int
			tmp = atoi(pStr);
			// Add it to the list
			appCFGParam.devTypeList[i] = (uint8) tmp;
			i++;
			if (i >= RTI_MAX_NUM_DEV_TYPES)
				break;
			// Now get next token
			pStr = strtok (NULL, " ,:;-|");
		}
		// Fill in the missing Target Types
		if (i < RTI_MAX_NUM_DEV_TYPES)
		{
			for (; i < RTI_MAX_NUM_DEV_TYPES; i ++)
			{
				appCFGParam.devTypeList[i] = RTI_DEVICE_RESERVED_INVALID;
			}
		}
		// Display current configuration to give feedback to user about the changes
		DispCFGCurrentCfg(appCFGParam);
		// Return to CFG state
		appCFGstate = APP_CFG_STATE_INIT;
		DispMenuInit();
		break;
	case APP_CFG_STATE_TGT_TYPES:
		// use strtok to split string and process each individual Profile ID
		i = 0;
		// Get first token
		pStr = strtok (strIn, " ,:;-|");
		while ( pStr != NULL)
		{
			// Convert string to int
			tmp = atoi(pStr);
			// Add it to the list
			appCFGParam.tgtTypeList[i] = (uint8) tmp;
			i++;
			if (i >= RTI_MAX_NUM_SUPPORTED_TGT_TYPES)
				break;
			// Now get next token
			pStr = strtok (NULL, " ,:;-|");
		}
		// Fill in the missing Target Types
		if (i < RTI_MAX_NUM_SUPPORTED_TGT_TYPES)
		{
			for (; i < RTI_MAX_NUM_SUPPORTED_TGT_TYPES; i ++)
			{
				appCFGParam.tgtTypeList[i] = RTI_DEVICE_RESERVED_INVALID;
			}
		}
		// Display current configuration to give feedback to user about the changes
		DispCFGCurrentCfg(appCFGParam);
		// Return to CFG state
		appCFGstate = APP_CFG_STATE_INIT;
		DispMenuInit();
		break;
	case APP_CFG_STATE_KEY_SEED:
		/* init key exchange transfer count to something small so key ex doesn't dominate logs */
		pData[0] = atoi(strIn);
		RTI_WriteItemEx( RTI_PROFILE_GDP, aplKeyExchangeTransferCount, 1, pData );
		printf("Set key seeds %d\n", pData[0]);
		// Return to CFG state
		appCFGstate = APP_CFG_STATE_INIT;
		DispMenuInit();
		break;
	}
}

void appInitConfigParam( char tgtSelection )
{
	int i;
	if (tgtSelection == 'c')
	{
		// Controll Type; Battery Powered; Security capable; Channel Normalization capable.
		debug_printf("\nController Configuration\n");
		appCFGParam.nodeCapabilities = RTI_BUILD_NODE_CAPABILITIES(0, 0, 1, 1);

		// Set up configuration parameters that are different from default values
		for (i = 0; i < sizeof(tgtListCTL); i++) {
			appCFGParam.tgtTypeList[i] = tgtListCTL[i];
		}

		debug_printf("ZID and ZRC profile activated\n");
		// No User String pairing; 1 Device (Remote Controller); 2 Profiles (ZRC & ZID)
		appCFGParam.appCapabilities = RTI_BUILD_APP_CAPABILITIES(0, 1, 2);

		for (i = 0; i < sizeof(devListCTL); i++) {
			appCFGParam.devTypeList[i] = devListCTL[i];
		}

		for (i = 0; i < sizeof(profileList); i++) {
			appCFGParam.profileIdList[i] = profileList[i];
		}

		appCFGParam.vendorId = RTI_VENDOR_TEXAS_INSTRUMENTS;

		for (i = 0; i < sizeof(vendorName); i++) {
			appCFGParam.vendorString[i] = vendorName[i];
		}

	}
	else if (tgtSelection == 't')
	{
		// Target Type; A/C Pwr; Security capable; Channel Normalization capable.
		debug_printf("\nTarget Configuration\n");
		appCFGParam.nodeCapabilities = RTI_BUILD_NODE_CAPABILITIES(1, 1, 1, 1);

		// Set up configuration parameters that are different from default values
		for (i = 0; i < sizeof(tgtListTGT); i++) {
			appCFGParam.tgtTypeList[i] = tgtListTGT[i];
		}

		debug_printf("ZRC profile activated\n");
		// No User String pairing; 1 Device (Television); 1 Profile (ZRC)
		appCFGParam.appCapabilities = RTI_BUILD_APP_CAPABILITIES(0, 1, 1);

		for (i = 0; i < sizeof(devListTGT); i++) {
			appCFGParam.devTypeList[i] = devListTGT[i];
		}

		for (i = 0; i < sizeof(profileList); i++) {
			appCFGParam.profileIdList[i] = profileList[i];
		}

		appCFGParam.vendorId = RTI_VENDOR_TEXAS_INSTRUMENTS;

		for (i = 0; i < sizeof(vendorName); i++) {
			appCFGParam.vendorString[i] = vendorName[i];
		}


	}
}

static void appSetCFGParamOnRNP( void )
{
	////////////////////////////////////
	// Apply Configuration Parameters //
	////////////////////////////////////

	printf("\n-------------------- SET RNP CONFIGURATION PARAMETERS-------------------\n");
	if (RTI_WriteItemEx(RTI_PROFILE_RTI, RTI_CP_ITEM_NODE_CAPABILITIES, 1,
			(uint8*)&(appCFGParam.nodeCapabilities)) != RTI_SUCCESS) {
		//   AP_FATAL_ERROR();
		debug_printf("[ERR] Could not write RTI_CP_ITEM_NODE_CAPABILITIES\n");
	}
	else
		debug_printf("[DEB] Successfully wrote RTI_CP_ITEM_NODE_CAPABILITIES\n");

	if (RTI_WriteItemEx(RTI_PROFILE_RTI, RTI_CP_ITEM_NODE_SUPPORTED_TGT_TYPES,
			RTI_MAX_NUM_SUPPORTED_TGT_TYPES, appCFGParam.tgtTypeList) != RTI_SUCCESS) {
		///AP_FATAL_ERROR();
		debug_printf("[ERR] Could not write RTI_CP_ITEM_NODE_SUPPORTED_TGT_TYPES\n");
	}
	else
		debug_printf("[DEB] Successfully wrote RTI_CP_ITEM_NODE_SUPPORTED_TGT_TYPES\n");

	if (RTI_WriteItemEx(RTI_PROFILE_RTI, RTI_CP_ITEM_APPL_CAPABILITIES, 1,
			(uint8*)&(appCFGParam.appCapabilities)) != RTI_SUCCESS) {
		// AP_FATAL_ERROR();
		debug_printf("[ERR] Could not write RTI_CP_ITEM_APPL_CAPABILITIES\n");
	}
	else
		debug_printf("[DEB] Successfully wrote RTI_CP_ITEM_APPL_CAPABILITIES\n");

	if (RTI_WriteItemEx(RTI_PROFILE_RTI, RTI_CP_ITEM_APPL_DEV_TYPE_LIST,
			RTI_MAX_NUM_DEV_TYPES, appCFGParam.devTypeList) != RTI_SUCCESS) {
		//   AP_FATAL_ERROR();
		debug_printf("[ERR] Could not write RTI_CP_ITEM_APPL_DEV_TYPE_LIST\n");
	}
	else
		debug_printf("[DEB] Successfully wrote RTI_CP_ITEM_APPL_DEV_TYPE_LIST\n");

	if (RTI_WriteItemEx(RTI_PROFILE_RTI, RTI_CP_ITEM_APPL_PROFILE_ID_LIST,
			RTI_MAX_NUM_PROFILE_IDS, appCFGParam.profileIdList) != RTI_SUCCESS) {
		//    AP_FATAL_ERROR();
		debug_printf("[ERR] Could not write RTI_CP_ITEM_APPL_PROFILE_ID_LIST\n");
	}
	else
		debug_printf("[DEB] Successfully wrote RTI_CP_ITEM_APPL_PROFILE_ID_LIST\n");

	if (RTI_WriteItemEx(RTI_PROFILE_RTI, RTI_CP_ITEM_VENDOR_ID, 2,
			(uint8*)&(appCFGParam.vendorId)) != RTI_SUCCESS) {
		//  AP_FATAL_ERROR();
		debug_printf("[ERR] Could not write RTI_CP_ITEM_VENDOR_ID\n");
	}
	else
		debug_printf("[DEB] Successfully wrote RTI_CP_ITEM_VENDOR_ID\n");

	if (RTI_WriteItemEx(RTI_PROFILE_RTI, RTI_CP_ITEM_VENDOR_NAME,
			sizeof(vendorName), appCFGParam.vendorString) != RTI_SUCCESS) {
		//  AP_FATAL_ERROR();
		debug_printf("[ERR] Could not write RTI_CP_ITEM_VENDOR_NAME\n");
	}
	else
		debug_printf("[DEB] Successfully wrote RTI_CP_ITEM_VENDOR_NAME\n");

}


void appGetCFGParamFromRNP( void )
{
	///////////////////////////////////////////
	// Read Configuration Parameters From RNP//
	///////////////////////////////////////////

	if (RTI_ReadItemEx(RTI_PROFILE_RTI, RTI_CP_ITEM_NODE_CAPABILITIES, 1,
			(uint8*)&(appCFGParam.nodeCapabilities)) != RTI_SUCCESS) {
		//   AP_FATAL_ERROR();
		printf("ERR: Failed to read Node Capabilities\n");
	}

	if (RTI_ReadItemEx(RTI_PROFILE_RTI, RTI_CP_ITEM_NODE_SUPPORTED_TGT_TYPES,
			RTI_MAX_NUM_SUPPORTED_TGT_TYPES, appCFGParam.tgtTypeList) != RTI_SUCCESS) {
		///AP_FATAL_ERROR();
		printf("ERR: Failed to read Target Types\n");
	}

	if (RTI_ReadItemEx(RTI_PROFILE_RTI, RTI_CP_ITEM_APPL_CAPABILITIES, 1,
			(uint8*)&(appCFGParam.appCapabilities)) != RTI_SUCCESS) {
		// AP_FATAL_ERROR();
		printf("ERR: Failed to read Application Capabilities\n");
	}

	if (RTI_ReadItemEx(RTI_PROFILE_RTI, RTI_CP_ITEM_APPL_DEV_TYPE_LIST,
			RTI_MAX_NUM_DEV_TYPES, appCFGParam.devTypeList) != RTI_SUCCESS) {
		//   AP_FATAL_ERROR();
		printf("ERR: Failed to read Device Types\n");
	}

	if (RTI_ReadItemEx(RTI_PROFILE_RTI, RTI_CP_ITEM_APPL_PROFILE_ID_LIST,
			RTI_MAX_NUM_PROFILE_IDS, appCFGParam.profileIdList) != RTI_SUCCESS) {
		//    AP_FATAL_ERROR();
		printf("ERR: Failed to read Profile IDs\n");
	}

	if (RTI_ReadItemEx(RTI_PROFILE_RTI, RTI_CP_ITEM_VENDOR_ID, 2,
			(uint8*)&(appCFGParam.vendorId)) != RTI_SUCCESS) {
		//  AP_FATAL_ERROR();
		printf("ERR: Failed to read Vendor ID\n");
	}

	if (RTI_ReadItemEx(RTI_PROFILE_RTI, RTI_CP_ITEM_VENDOR_NAME,
			sizeof(vendorName), appCFGParam.vendorString) != RTI_SUCCESS) {
		//  AP_FATAL_ERROR();
		printf("ERR: Failed to read Vendor String\n");
	}

}
/**************************************************************************************************
 * @fn          appTestModeProcessKey
 *
 * @brief       This function executes the Test Mode
 *
 * input parameters
 *
 * @param key - key from key scanner to be processed
 * @param strIn - string from console to be processed
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void appSendDataProcessKey (char* strIn)
{

//	printf("------------------------------------------------------\n");
//	printf("Send Data MENU:");
//	printf("r- Return to Main Menu\n");
//	printf("s- Send Data\n");
//	printf("1- Set Destination Index\n");
//	printf("2- Set Payload\n");
//	printf("3- Set Tx Options\n");
//	printf("4- Send ZID Keyboard data\n");
//	printf("l- List current configuration\n");
//	printf("m- Show This Menu\n");

//	// Send Data States
//	enum {
//		APP_SEND_DATA_STATE_INIT,		// Initial Application Sub State for Configuration of data to send
//		APP_SEND_DATA_STATE_DEST_IDX,	// Sub state for configuration of destination index
//		APP_SEND_DATA_STATE_PROFILE,	// Sub state for configuration of profile
//		APP_SEND_DATA_STATE_TX_OPTIONS,	// Sub state for configuration of Tx options
//		APP_SEND_DATA_STATE_PAYLOAD,	// Sub state for configuration of payload
//		APP_SEND_DATA_STATE_KEYBOARD,// Sub state for sending ZID Keyboard commands directly
//	};

	if (appSendDataState == APP_SEND_DATA_STATE_INIT)
	{
		switch (strIn[0])
		{
		case 'r':
			appState = AP_STATE_READY;
			// Display menu
			DispMenuReady();
			return;
		case 'm':
			// Display Test Mode menu
			DispSendDataMenu();
			return;
		case 's':
			// Go to NDATA state to send data
			appState = AP_STATE_NDATA;
			// Send Data
			RTI_SendDataReq(appSendData_s.dstIndex,
					appSendData_s.profileId,
					appSendData_s.vendorId,
					appSendData_s.txOptions,
					appSendData_s.len,
					appSendData_s.pData );
			break;
		case '1':
			// Go to configuration state (of destination index) to allow setting start condition
			appSendDataState = APP_SEND_DATA_STATE_DEST_IDX;
			DispSendDataDestIndexMenu();
			break;
		case '2':
			// Go to configuration state (of payload)
			appSendDataState = APP_SEND_DATA_STATE_PAYLOAD;
			DispSendDataPayloadMenu();
			break;
		case '3':
			// Go to configuration state (of Tx Options)
			appSendDataState = APP_SEND_DATA_STATE_TX_OPTIONS;
			DispSendDataTxOptionsMenu();
			break;
		case '4':
			// Go to configuration state (of Tx Options)
			appSendDataState = APP_SEND_DATA_STATE_PROFILE_ID;
			DispSendDataProfileIDMenu();
			break;
		case 'l':
			DispSendDataCurrentCfg(appSendData_s);
			break;
		default:
			printf("unknown command %c (0x%.2X) \n", strIn[0], strIn[0]);
			DispSendDataMenu();
			return;
		}
	}
	else if (appSendDataState == APP_SEND_DATA_STATE_DEST_IDX)
	{
		// TODO: Add protection against illegal destination index
		appSendData_s.dstIndex = atoi(strIn);

		// Return to Send Data Init state
		appSendDataState = APP_SEND_DATA_STATE_INIT;
	}
	else if (appSendDataState == APP_SEND_DATA_STATE_PAYLOAD)
	{
		// TODO: Treat data based on the set profile ID
		char *pEnd, *pEndOld;
		uint8 i = 0;
		appSendData_s.pData[0] = strtol(strIn, &pEnd, 16);
		debug_printf("payload[%d] 0x%.2X (int)%d\n", i, appSendData_s.pData[i], (int)appSendData_s.pData[i]);
		while ( strlen(pEnd) != 0 )
		{
			pEndOld = pEnd;
			appSendData_s.pData[++i] = strtol(pEnd, &pEnd, 16);
			debug_printf("payload[%d] 0x%.2X (int)%d\n", i, appSendData_s.pData[i], (int)appSendData_s.pData[i]);

			// Check for global error code
			if (errno == ERANGE)
				break;

			// Another error can occur if strtol cannot convert. It will then return 0
			// 0 is a legal value, so to determine if it's actually an error we have to
		// compare to the string which should be '00' in this case.

			// Or simply check if the pointer has increased, or if the number of bytes exceeds the legal input
			if ( (pEndOld == pEnd) || (i >= 90) )
			{
				printf("ERR: Illegal input");
				break;
			}
		}
		appSendData_s.len = ++i;
		// Return to Send Data Init state
		appSendDataState = APP_SEND_DATA_STATE_INIT;
	}
	else if (appSendDataState == APP_SEND_DATA_STATE_TX_OPTIONS)
	{
		appSendData_s.txOptions = strtol(strIn, NULL, 16);

		printf("Tx Options set to: 0x%.2X\n", appSendData_s.txOptions);

		// Return to Send Data Init state
		appSendDataState = APP_SEND_DATA_STATE_INIT;
	}
	else if (appSendDataState == APP_SEND_DATA_STATE_PROFILE_ID)
	{
		appSendData_s.profileId = atoi(strIn);

		printf("Set Profile ID: 0x%.2X\n", appSendData_s.profileId);

		// Return to Send Data Init state
		appSendDataState = APP_SEND_DATA_STATE_INIT;
	}
}
