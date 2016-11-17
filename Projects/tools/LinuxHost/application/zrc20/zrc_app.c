/**************************************************************************************************
 Filename:       zrc_app.c

 Description:    Linux Host application for ZRC profile target node

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
#include <time.h>
#include <sys/time.h>
#include <unistd.h>

#include "common_app.h"
#include "timer.h"
#include "tiLogging.h"

#ifndef RTI_TESTMODE
#define RTI_TESTMODE
#endif //RTI_TESTMODE

#include "npi_lnx.h"
#include "npi_rti.h"


// Linux surrogate interface, including both RTIS and NPI
#include "npi_ipc_client.h"
#include "zrc_app_main.h"
#include "zrc_app.h"
#include "zrc_configuration.h"
#include "zrc_vendor_specific.h"

// Profile includes
#include "gdp.h"

#ifdef RTI_TESTMODE
#include "RTI_Testapp.h"
#endif //RTI_TESTMODE

#include "hal_rpc.h"
#include "npi_lnx_ipc_rpc.h"
#include "npi_lnx_error.h"

// macros
#define PAUSE() { fflush(stdout); while (getchar()!='\n'); }

// Application state variable
uint8 zrcAppState = AP_STATE_INIT; // Public so OEM extensions can access the state.

static uint8 zrcBindingType = BINDING_TYPE_NORMAL;
// Track power state of RNP (only one Client can control and track this!)
static uint8 zrcAppRNPpowerState;
struct timeval curTime, startTime, prevTimeSend, prevTimeRec;
struct timeval curTime, prevTime;

/**********************************************************************************
 * Local type defs and other defines
 */
typedef struct
{
  uint8 pairIndex;
  uint16 vendorId;
  uint8 rxLQI;
  uint8 rxFlags;
  uint8 reportId;
} zrcDongleReportInfo_t;

// Pairing reference
uint8 stbDstIndex;

// ch from console
char ch;
// string from console
char str[128];

// Toggle Timer Print on Server state variable
static uint8 toggleTimerPrintOnServer = FALSE;
static uint8 toggleBigDebugPrintOnServer = FALSE;

uint8 zrcAppTestDataPending = 0;

/* Used to save last report sent info in case we need to send a NULL report later */
static zrcDongleReportInfo_t zrcDongleNullReportInfo;

/**********************************************************************************
 * Common variables
 */
response_t responseBuf;	// Used for Set/Get Attributes Response

/**********************************************************************************
 * Validation state variables
 */
static uint8 zrcAppValidationStatus = GDP_CHECK_VALIDATION_STATUS_PENDING;
static uint8 zrcAppValidationSourceIndex = RTI_INVALID_PAIRING_REF;
static uint8 zrcAppValidationState;
static uint8 zrcAppNumValidationDigitsPressed = 0;
static uint8 zrcAppNumFailedValidationAttempts = 0, zrcAppNumFailedValidationAttemptsMax = 10;
static uint8 zrcAppValidationDigits[ZRC_VALIDATION_CODE_SIZE] = {0};
static uint8 zrcAppGoldenDigits[ZRC_VALIDATION_CODE_SIZE] = {0};

/***********************************************************************************
 * Polling
 */

// outgoing message queue for polling
typedef struct zrcMsgQueue
{
    uint8 profileId;
    uint8 msgLen;
    uint8 *pMsg;
    struct zrcMsgQueue *pNext;
} zrcMsgQueue_t;

zrcMsgQueue_t *pRsaMsgQueueHead = NULL;

//THREAD and MUTEX
static pthread_mutex_t zrcAppThreadMutex;
static pthread_mutex_t zrcAppInitMutex;

static pthread_t AppThreadId;
static void *zrcAppThreadFunc(void *ptr);
static int zrcAppThreadTerminate;

static void zrcAppProcessEvents(uint32 events);

uint8 ZRC_App_threadId;

uint8 zrcAppSendDataState;

/***************************************************************
 * Local function declarations
 */
static void zrcAppProcessZRC20ProfileData(uint8 srcIndex, uint8 len, uint8 *pData, uint8 rxLQI);
static void zrcAppProcessValidationCommand(uint8 srcIndex, uint8 actionCode);
static void zrcAppProcessGDPProfileData(uint8 srcIndex, uint8 len, uint8 *pData, uint8 rxLQI);
static void zrcAppProcessGDPPushAttributes(uint8 srcIndex, uint8 len, uint8 *pData);
static void zrcAppInitStack();
static char * zrcGetActionTypeStringBuf( uint8 type );
void zrcBuildIdentifyClientNotificationCmd( uint8 *pLen, uint8 **ppBuf );
void zrcMsgQueue_push( uint8 *pMsg, uint8 len, uint8 profileId);
void zrcMsgQueue_pop( uint8 **ppMsg, uint8 *pLen, uint8 *pProfileId );
bool zrcMsgQueue_isEmpty( void );
static int zrcAppInitSyncRes(void);

int zrcAppGetAndPrintExtendedSoftwareVersion(swVerExtended_t *swVerExtended);
static int zrcAppGetAndPrintSoftwareVersions(uint8 *baseVersion, swVerExtended_t *swVerExtended);
static void zrcAppReturnFromSubmodule(void);

// The following file includes a large set of string defines used for verbose logging.
#include "zrc_app_log_constants.c"

appSendData_t  zrcAppSendData;
static zrcAppRtiDataCnfCbackFn_t zrcAppSendDataCnfCb = NULL;

static char tmpStrForTimePrint[1024];

// Unused APIs required by npi_rti.c
void RTI_IrInd( uint8 irData ) {}
void RTI_AllowPairCnf(rStatus_t status, uint8 dstIndex, uint8 devType) {}

int ZRC_AppInit(int mode, char threadId)
{
    zrcAppThreadTerminate = 0;

    //Initialize Mutex
    if (NPI_LNX_SUCCESS != zrcAppInitSyncRes())
    {
        return NPI_LNX_FAILURE;
    }

    ZRC_App_threadId = threadId;

    LOG_INFO("-------------------- START TOGGLE DEBUG TRACES on SERVER/DAEMON SIDE-------------------\n");
    npiMsgData_t pMsg;
    pMsg.len = 1;
    pMsg.subSys = RPC_SYS_SRV_CTRL | RPC_CMD_SREQ;
    pMsg.cmdId = NPI_LNX_CMD_ID_CTRL_BIG_DEBUG_PRINT_REQ;

    if ( (mode == 2) || (mode == 3))
    {
        toggleBigDebugPrintOnServer = TRUE;
    }

    pMsg.pData[0] = toggleBigDebugPrintOnServer;

    // send debug flag value
    NPI_SendSynchData( &pMsg );
    if (RTI_SUCCESS == pMsg.pData[0])
        LOG_INFO("__BIG_DEBUG_ACTIVE on server set to 0x%.2X\n", toggleBigDebugPrintOnServer);
    else
        LOG_WARN("Failed to set __BIG_DEBUG_ACTIVE on server to 0x%.2X\n", toggleBigDebugPrintOnServer);

    pMsg.cmdId = NPI_LNX_CMD_ID_CTRL_TIME_PRINT_REQ;

    if ( (mode == 1) || (mode == 3))
    {
        toggleTimerPrintOnServer = TRUE;
    }
    pMsg.pData[0] = toggleTimerPrintOnServer;

    // send debug flag value
    NPI_SendSynchData( &pMsg );
    if (RTI_SUCCESS == pMsg.pData[0])
        LOG_INFO("__DEBUG_TIME_ACTIVE on server set to 0x%.2X\n", toggleTimerPrintOnServer);
    else
        LOG_WARN("Failed to set __DEBUG_TIME_ACTIVE on server to 0x%.2X\n", toggleTimerPrintOnServer);

    LOG_INFO("-------------------- END TOGGLE DEBUG TRACES on SERVER/DAEMON SIDE -------------------\n");

    /********************************************************************************************
     * Initialize hosts non-volatile memory
     */
//    if (NPI_LNX_SUCCESS != ZRC_NVInit())
//    {
//        // NV File not existing
//        LOG_ERROR("[Initialization] Failed to open NV file\n");
//        return NPI_LNX_FAILURE;
//    }

    LOG_INFO("[Initialization]-------------------- START SOFTWARE VERSION READING-------------------\n");

	uint8           rnpSwVerBase;
    swVerExtended_t rnpSwVerExtended;
    int readSoftwareVersionStatus = zrcAppGetAndPrintSoftwareVersions(&rnpSwVerBase, &rnpSwVerExtended);
    if (RTI_SUCCESS != readSoftwareVersionStatus)
    {
//            LOG_ERROR("[Initialization] Failed to read software version. Checking if the chip was reset into Serial Bootloader mode.\n");
//            int tmpRetVal = SBL_IsDeviceInSBLMode();
//            if (tmpRetVal == NPI_LNX_SUCCESS)
//            {
//                zrcAppState = AP_STATE_INIT_SBL;
//                // Go straight to check if we are in serial bootloader mode and potentially load new image
//                if    (NPI_LNX_SUCCESS != SBL_Execute())
//                    LOG_ERROR("[Initialization] Firmware update failed.\n");
//                else
//                    LOG_INFO("[Initialization] Firmware update success.\n");
//            }
//            else
//            {
//                // RTI_ResetInd() may be called after resetting, so make sure stack is not unintentionally initialized.
//                zrcAppState = AP_STATE_RESET;
//
//                // Attempt a reset, then read the software version again
//                npiMsgData_t pMsg;
//                pMsg.len = 0;
//                pMsg.subSys = RPC_SYS_SRV_CTRL | RPC_CMD_AREQ;
//                pMsg.cmdId = NPI_LNX_CMD_ID_RESET_DEVICE;
//                // Send command
//                NPI_SendAsynchData( &pMsg );
//
//                // Allow up to 1.4 seconds to reset
//                int sleepCount = 7;
//                while ((zrcAppState == AP_STATE_RESET) && (sleepCount > 0))
//                {
//                    sleepCount--;
//                    usleep(200000);
//                }
//
//                // The try to read software version again
//                readSoftwareVersionStatus = RTI_ReadItem(RTI_CONST_ITEM_SW_VERSION, 1, value);
//
//                if (readSoftwareVersionStatus != RTI_SUCCESS)
//                {
//                    LOG_ERROR("[Initialization] Failed to read software version after resetting. Attempt a firmware upgrade\n");
//                    zrcAppState = AP_STATE_INIT_SBL;
//                    // Go straight to check if we are in serial bootloader mode and potentially load new image
//                    if    (NPI_LNX_SUCCESS != SBL_Execute())
//                        LOG_ERROR("[Initialization] Firmware update failed.\n");
//                    else
//                        LOG_INFO("[Initialization] Firmware update success.\n");
//                }
//            }
//            readSoftwareVersionStatus = RTI_ReadItem(RTI_CONST_ITEM_SW_VERSION, 1, value);
//            zrcAppState = AP_STATE_INIT;
    }
    LOG_INFO("[Initialization]-------------------- END SOFTWARE VERSION READING-------------------\n");

//    zrcAppState = AP_STATE_INIT_SBL;
//    // Check if we should update software
//    SBL_CheckForUpdate(&rnpSwVerExtended);
    zrcAppState = AP_STATE_INIT;

    LOG_INFO("[Initialization]------------------- BEGIN IEEE ADDRESS READING------------------\n");
    uint8 ieeeAddr[SADDR_EXT_LEN], strLen;
    int j;
    char tmpStr[128];
    RTI_ReadItem(RTI_SA_ITEM_IEEE_ADDRESS, SADDR_EXT_LEN, ieeeAddr);
    strLen = 0;
    for (j = (SADDR_EXT_LEN - 2); j >= 0; j--)
    {
        snprintf(tmpStr+strLen, sizeof(tmpStr)-strLen, ":%.2X", (ieeeAddr[j] & 0x00FF));
        strLen += 3;
    }
    LOG_INFO("[Initialization] Our IEEE is: %.2hX%s\n", (ieeeAddr[SADDR_EXT_LEN - 1] & 0x00FF), tmpStr);
    LOG_INFO("[Initialization]-------------------- END IEEE ADDRESS READING-------------------\n");

    //    /********************************************************************************************
    //     * Update RAM version of RF Statistics from Non-Volatile memory
    //     */
    //    ZRC_NVInit();
    //    zrcCfgRestoreFromNV();

#ifdef RTI_TESTMODE
    RTI_TestModeInit(zrcAppReturnFromSubmodule);
#endif //RTI_TESTMODE

    if (pthread_create(&AppThreadId, NULL, zrcAppThreadFunc, NULL))
    {
        // thread creation failed
        LOG_ERROR("Failed to create app thread\n");
        return NPI_LNX_FAILURE;
    }

    return NPI_LNX_SUCCESS;
}

void ZRCApp_start_timerEx(uint32 event, uint32 timeout)
{
    timer_start_timerEx(ZRC_App_threadId, event, timeout);
}

static int zrcAppInitSyncRes(void)
{
    // initialize all mutexes
    if (pthread_mutex_init(&zrcAppThreadMutex, NULL))
    {
        LOG_ERROR("Fail To Initialize Mutex appThreadMutex\n");
        return NPI_LNX_FAILURE;
    }

    if (pthread_mutex_init(&zrcAppInitMutex, NULL))
    {
        LOG_ERROR("Fail To Initialize Mutex appInitMutex\n");
        return NPI_LNX_FAILURE;
    }

    return NPI_LNX_SUCCESS;
}

static void *zrcAppThreadFunc(void *ptr)
{
    uint32 events = 0;

    /* lock mutex in order not to lose signal */
    pthread_mutex_lock(&zrcAppThreadMutex);
    int semRet = 0;

    zrcAppState = AP_STATE_INIT;

    LOG_INFO("[ZRC Initialization] ZRC App Thread Started \n");

    //Display menu...
    DispMenuInit();

    /* thread loop */
    while (!zrcAppThreadTerminate)
    {
        // Wait for event
        while ((semRet = sem_wait(&eventSem)) == EINTR);
        if ( semRet != 0)
        {
            if (errno != EINVAL)
            {
                LOG_ERROR("sem_wait errno %d", errno);
            }
            else
            {
                usleep(1000);
                events = timer_get_event(ZRC_App_threadId);
            }
        }
        else
        {
            events = timer_get_event(ZRC_App_threadId);
        }

        // Process events
        if (events != 0)
        {
            zrcAppProcessEvents(events);
            LOG_INFO("State: %s [0x%.2X]\n", AppState_list[zrcAppState], zrcAppState);
        }

        // Only process actions if there is a character available
        if (consoleInput.handle == MAIN_INPUT_READY)
        {
            ch = consoleInput.latestCh;
            strcpy(str, consoleInput.latestStr);

            // 'q' has highest priority
            if (zrcAppState == AP_STATE_INIT)
            {
                // Let user configure the RNP
                if ( (ch == 'c') || (ch == 't') )
                {
                    zrcCfgSetTarget((ch == 't') ? TRUE : FALSE);

                    zrcCfgInitConfigParam();

                    zrcCfgSetCFGParamOnRNP();

                    // Initialize stack
                    zrcAppInitStack();
                }
                else
                {
                    LOG_WARN("Invalid selection\n");
                    DispMenuInit();
                }
            }
            else if (zrcAppState == AP_STATE_VALIDATION)
            {
                uint8 txBuf[5] = {0, 0, 0, 0, 0};
                txBuf[0] = ZRC_CMD_ID_ACTIONS;
                txBuf[1] = ZRC_ACTION_CTRL_TYPE_START;
                txBuf[2] = 0; // action payload = 0
                txBuf[3] = 0; // action bank = HDMI-CEC
                txBuf[4] = (ch - 0x30) + RTI_CERC_NUM_0;
                // Send validation digits
                ZRCApp_SendDataReq( stbDstIndex, RTI_PROFILE_ZRC20,
                        RTI_VENDOR_TEST_VENDOR,
                        (RTI_TX_OPTION_ACKNOWLEDGED | RTI_TX_OPTION_SECURITY),
                        sizeof(txBuf), txBuf, NULL );
            }
            else if (zrcAppState == AP_STATE_RESET)
            {

            }
#ifdef RTI_TESTMODE
            else if (zrcAppState == AP_STATE_LATENCY_TEST_MODE)
            {
                appTestModeProcessKey(str);
            }
#endif //RTI_TESTMODE
            else if (ch == 'm')
            {
                // Display menu
                DispMenuReady();
            }
            else if (zrcAppState == AP_STATE_NDATA)
            {
                // Do not allow key presses during transmission
                LOG_WARN("Busy transmitting, please wait\n");
            }
            else if (zrcAppState == AP_STATE_NDATA_PREPARE)
            {
                if ( (ch >= 0x30) && (ch <= 0x39) )
                {
                    uint8 txBuf[5] = {0, 0, 0, 0, 0};
                    txBuf[0] = ZRC_CMD_ID_ACTIONS;
                    txBuf[1] = ZRC_ACTION_CTRL_TYPE_START;
                    txBuf[2] = 0; // action payload = 0
                    txBuf[3] = 0; // action bank = HDMI-CEC
                    txBuf[4] = (ch - 0x30) + RTI_CERC_NUM_0;
                    zrcAppState = AP_STATE_NDATA;
                    // Send digits
                    ZRCApp_SendDataReq( stbDstIndex, RTI_PROFILE_ZRC20,
                            RTI_VENDOR_TEST_VENDOR,
                            (RTI_TX_OPTION_ACKNOWLEDGED | RTI_TX_OPTION_SECURITY),
                            sizeof(txBuf), txBuf, NULL );
                }
                else
                {
                    zrcAppState = AP_STATE_READY;

                    LOG_INFO("Entered %s [0x%.2X]\n", AppState_list[zrcAppState], zrcAppState);

                    //Display menu...
                    DispMenuReady();

                }
            }
            else if (ch == '1')
            {
                if (zrcAppState == AP_STATE_READY)
                {
                    // Enter pairing state
                    zrcAppState = AP_STATE_PAIR;
                    if (TRUE == zrcCfgIsTarget())
                    {
                        // Then issue pairing request
                        RTI_AllowBindReq();
                    }
                    else
                    {
                        // Then issue pairing request
                        RTI_BindReq( zrcBindingType );
                    }
                }
            }
            else if (ch == '2')
            {
                if (zrcAppState == AP_STATE_READY)
                {
                    if (zrcAppRNPpowerState & ZRC_APP_RNP_POWER_STATE_NPI_BIT)
                    {
                        // Warn user of sleep
                        LOG_WARN("Cannot call RTI_UnpairReq because NPI is in sleep\n");
                    }
                    else {
                        zrcAppState = AP_STATE_UNPAIR;
                        // Unpair
                        LOG_INFO("Calling RTI_UnpairReq for index %d\n", stbDstIndex);
                        RTI_UnpairReq(stbDstIndex);
                    }
                }
                else
                {
                    // Simply remain in whatever state we're in
                    LOG_INFO("Cannot call RTI_UnpairReq, because we're in state: 0x%.2X\n", zrcAppState);
                }
            }
            else if (ch == '3')
            {
                if (zrcAppState == AP_STATE_READY)
                {
                    zrcAppState = AP_STATE_LATENCY_TEST_MODE;
                    LOG_INFO("Entering Test Mode\n");
                    RTI_EnterTestMode();
                }
                else
                {
                    LOG_WARN("Cannot enter testmode, because we're in state: 0x%.2X\n", zrcAppState);
                }
            }
            else if ((ch == 'h') ||
                    (ch == 'j') ||
                    (ch == 'k') ||
                    (ch == 'l'))
            {
                // Set channel
                uint8 channel, faEnable = FALSE, status = RTI_SUCCESS;
                if (ch == 'l')
                {
                    faEnable = TRUE;
                    status = RTI_WriteItemEx(RTI_PROFILE_RTI, RTI_SA_ITEM_AGILITY_ENABLE, 1, (uint8 *)&faEnable);
                    LOG_INFO("Frequency Agility re-enabled (%s)\n", rtiStatus_list[status]);
                }
                else
                {
                    if (ch == 'h')
                    {
                        channel = 15;
                    }
                    else if (ch == 'j')
                    {
                        channel = 20;
                    }
                    else if (ch == 'k')
                    {
                        channel = 25;
                    }
                    status = RTI_WriteItemEx(RTI_PROFILE_RTI, RTI_SA_ITEM_AGILITY_ENABLE, 1, (uint8 *)&faEnable);
                    LOG_INFO("Frequency Agility disabled (%s)\n", rtiStatus_list[status]);
                    if (status == RTI_SUCCESS)
                    {
                        RTI_WriteItemEx(RTI_PROFILE_RTI, RTI_SA_ITEM_CURRENT_CHANNEL, 1, (uint8 *)&channel);
                        LOG_INFO("Channel set to %d\n", channel);
                    }
                }
            }
            else if (ch == 'f')
            {
                // Find my remote feature
                uint8 len;
                uint8 *pMsg;
                if (zrcCfgGetIdentificationClientCapabilities())
                {
                    LOG_INFO("[POLL] Preparing outgoing data\n");
                    zrcBuildIdentifyClientNotificationCmd( &len, &pMsg );
                    zrcMsgQueue_push( pMsg, len, RTI_PROFILE_GDP);
                }
                else
                {
                    LOG_WARN("[POLL] Cannot preparing outgoing data as no polling is configured\n");
                }
            }
            else if (ch == 's')
            {
                // Let RNP know we enter sleep
                LOG_INFO("Calling RTI_EnableSleepReq\n");
                RTI_EnableSleepReq();
            }
            else if (ch == 'p')
            {
                // Acknowledge waking up on GPIO
                LOG_INFO("Calling RTI_PingReq\n");
                RTI_PingReq();
            }
            else if (ch == 'c')
            {
                // Prepare connection request
                npiMsgData_t pMsg;
                pMsg.subSys = RPC_SYS_SRV_CTRL;
                pMsg.cmdId  = NPI_LNX_CMD_ID_CONNECT_DEVICE;
                pMsg.len    = 1;

                pMsg.pData[0] = NPI_SERVER_DEVICE_INDEX_SPI;
                LOG_INFO("Connecting to %d ...", pMsg.pData[0]);

                NPI_SendSynchData( &pMsg );
                LOG_INFO("status %d\n", pMsg.pData[0]);
            }
            else if (ch == 'd')
            {
                // Set debug level
                char *strDigit;
                // First call will give 'd'
                strDigit = strtok(str, " -:");
                if (strDigit != NULL)
                {
                    // Second call will give the level to use for debugging
                    strDigit = strtok(NULL, " -:");

                    if (strDigit != NULL)
                    {
                        __APP_LOG_LEVEL = strtol(strDigit, NULL, 10);
                        LOG_INFO("Debug level set to %d\n", __APP_LOG_LEVEL);
                    }
                    else
                    {
                        LOG_WARN("Debug level could not be set, please provide valid input\n");
                    }
                }
                else
                {
                    LOG_WARN("Debug level could not be set, please provide valid input\n");
                }
            }
            else if (ch == '8')
            {
                if (zrcAppRNPpowerState & ZRC_APP_RNP_POWER_STATE_NPI_BIT)
                {
                    // Warn user of sleep
                    LOG_WARN("Cannot clear pairing table because NPI is in sleep\n");
                }
                else
                {
                    // Read out and display all pairing entries
                    zrcCfgClearPairingTable();
                }
            }
            else if (ch == '9')
            {
                if (zrcAppRNPpowerState & ZRC_APP_RNP_POWER_STATE_NPI_BIT)
                {
                    // Warn user of sleep
                    LOG_WARN("Cannot display pairing table because NPI is in sleep\n");
                }
                else
                {
                    // Read out and display all pairing entries
                    zrcCfgDisplayPairingTable();
                }
            }
            else if (ch == '4')
            {
                uint8           baseVersion;
                swVerExtended_t rnpSwVerExtended;

                LOG_INFO("-------------------- START SOFTWARE VERSION READING-------------------\n");
                zrcAppGetAndPrintSoftwareVersions(&baseVersion, &rnpSwVerExtended);
                LOG_INFO("-------------------- END SOFTWARE VERSION READING-------------------\n");

                DispMenuReady();
            }
            else if (ch == 'r')
            {
                timer_set_event(ZRC_App_threadId, ZRC_APP_EVT_RESET);

                // Schedule event to Init RNP.
                //                DispMenuReady();
            }
            else if (ch == 't')
            {
                npiMsgData_t pMsg;
                pMsg.len = 1;
                pMsg.subSys = RPC_SYS_SRV_CTRL | RPC_CMD_SREQ;
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
                NPI_SendSynchData( &pMsg );
                if (RTI_SUCCESS == pMsg.pData[0])
                {
                    LOG_INFO("__DEBUG_TIME_ACTIVE set to: 0x%.2X\n", toggleTimerPrintOnServer);
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
                    LOG_INFO("__BIG_DEBUG_ACTIVE set to: 0x%.2X\n", toggleBigDebugPrintOnServer);
                }
            }
            else if (ch != '\n')
            {
                LOG_WARN("unknown command %c (0x%.2X) \n", ch, ch);
                DispMenuReady();
            }

            // 'q' requires special attention, hence it is not part of if/else
            if (ch == 'q')
            {
                //Terminate Thread
                zrcAppThreadTerminate = 1;
                //Terminate Thread and exit everything
            }

            // Release handle at the end to indicate to main thread that character is processed
            consoleInput.handle = MAIN_INPUT_RELEASED;
        }

    }
    pthread_mutex_unlock(&zrcAppThreadMutex);

    return NULL;
}

/**************************************************************************************************
 *
 * @fn      zrcAppInitStack
 *
 * @brief   Safely initialize stack. This function is blocking on RTI_InitCnf()
 *
 * @param   none
 *
 * @return  void
 */
static void zrcAppInitStack()
{
    int mutexRet = 0;
    // Initialize node and RF4CE stack
    LOG_INFO("Calling RTI_InitReq...\n");
    //When Launching the Thread, Mutex is unlocked.
    LOG_DEBUG("[MUTEX] Lock zrcAppInit Mutex\n");
    if ((mutexRet = pthread_mutex_trylock(&zrcAppInitMutex)) == EBUSY)
    {
        LOG_ERROR("[MUTEX] zrcAppInit Mutex busy\n");
    }
    else
    {
        LOG_DEBUG("[MUTEX] zrcAppInit Lock status: %d\n", mutexRet);
    }
    // Check if the firmware was previously incorrectly initialized. This is a specific spot check with limited scope.
    uint8 routeToApp = FALSE, startupFlg = CLEAR_STATE, retVal;
    retVal = RTI_ReadItemEx(RTI_PROFILE_RTI, RTI_SA_ITEM_ROUTE_DISCOVERY_TO_APP, 1, (uint8 *)&routeToApp);
    if (retVal != RTI_SUCCESS)
    {
        LOG_ERROR("Failed to read important attribute\n");
    }
    else if (routeToApp == FALSE)
    {
        // This attribute should always be TRUE for all ZRC images. Perform cold start to set things right.
        retVal = RTI_WriteItem(RTI_CP_ITEM_STARTUP_CTRL, 1, &startupFlg);
        if (retVal != RTI_SUCCESS)
        {
            LOG_ERROR("Could not set Cold Start flag\n");
        }
        else
        {
            LOG_INFO("Successfully set Cold Start flag\n");
        }
    }
    // Then call initialize stack
    RTI_InitReq();
    LOG_INFO("...Waiting for RTI_InitCnf. (can take up to 6s if cold start and target RNP)...\n");
    //Mutex is unlocked only inside RTI_Initcnf(); So this would effectively prevent application from continuing until initialization is complete
    pthread_mutex_lock(&zrcAppInitMutex);
    pthread_mutex_unlock(&zrcAppInitMutex);
}

/**************************************************************************************************
 *
 * @fn      zrcAppProcessEvents
 *
 * @brief   Process events function
 *
 * @param   events - 32 bit mask
 *
 * @return  void
 */
static void zrcAppProcessEvents(uint32 events)
{
    uint32 procEvents = 0;
    //    uint8 *pBuf;

    if (events & ZRC_APP_EVT_DATA_RCV)
    {
        LOG_INFO("Received Data event. State: %s [0x%.2X]\n", AppState_list[zrcAppState], zrcAppState);
        // Prepare to clear event
        procEvents |= ZRC_APP_EVT_DATA_RCV;
    }

    if (events & ZRC_APP_EVT_INIT)
    {
        LOG_INFO("Initialization event. State: %s [0x%.2X]\n", AppState_list[zrcAppState], zrcAppState);
        // Prepare to clear event
        procEvents |= ZRC_APP_EVT_INIT;
        if (zrcAppState == AP_STATE_INIT_COLD)
        {
            uint8 startupFlg, retVal;

            startupFlg = CLEAR_STATE;
            retVal = RTI_WriteItem(RTI_CP_ITEM_STARTUP_CTRL, 1, &startupFlg);
            if (retVal != RTI_SUCCESS)
            {
                LOG_WARN("Could not set Cold Start flag\n");
            }
            else
            {
                LOG_INFO("Successfully set Cold Start flag\n");
            }
        }
        // In case we reset from TOAD failed state, we need to
        // reconfigure back so we support all required Profiles
        // TODO: Keep previously user configured values. For now use default

        zrcCfgInitConfigParam();
        // Apply changes
        zrcCfgSetCFGParamOnRNP();
        LOG_INFO("Trying to initialize again, as %s\n",
                (TRUE == zrcCfgIsTarget()) ?
                        "Target" : "Controller");
        // Initialize stack
        zrcAppInitStack();

    }

    if (events & ZRC_APP_EVT_TEST_RX_ENABLE_TIMEOUT)
    {
        // Mark this event as processed
        procEvents |= ZRC_APP_EVT_TEST_RX_ENABLE_TIMEOUT;
    }

    if (events & ZRC_APP_EVT_RESET)
    {
        // Prepare to clear event
        procEvents |= ZRC_APP_EVT_RESET;

        // Set INIT_STATE
        zrcAppState = AP_STATE_INIT_COLD;

        // Reset RNP
        //        RTI_SwResetReq();
        RTI_ResetInd();
    }

    if (events & ZRC_APP_EVT_KEY_RELEASE_LOST_MASK)
    {
        // At least one release key was missed, report them all to IARM
        uint8 remoteIndex;
        for (remoteIndex = 0; remoteIndex < ZRC_MAX_BOUND_REMOTES; remoteIndex++)
        {
            if (events & ZRC_APP_EVT_KEY_RELEASE_LOST(remoteIndex))
            {
                /*********************************************************
                 * Indicate lost Key release
                 */
                LOG_INFO("Lost Event Key: code 0x%.2x, remoteId %d\n",
                        zrcCfgGetLastKeypressCommand(remoteIndex),
                        remoteIndex);

                // Prepare to clear event
                procEvents |= ZRC_APP_EVT_KEY_RELEASE_LOST(remoteIndex);
            }
        }
    }

    if (events & ZRC_APP_EVT_STORE_TO_NV)
    {
        // Mark event as processed so that it will be cleared.
        procEvents |= ZRC_APP_EVT_STORE_TO_NV;

        //        LOG_INFO("STORE_TO_NV event triggered\n");
        //
        //        zrcCfgStoreToNV(ZRC_CFG_STORE_NV);
    }

    if (events & ZRC_APP_EVT_SEND_SET_GET_ATTR_RESPONSE)
    {
        LOG_INFO("Sending Get/Set Attribute Response Command: %d (ProfileID: 0x%.4X, len: %d)\n", responseBuf.destIndex, responseBuf.profileId, responseBuf.msgLen);
        zrcAppState = AP_STATE_NDATA;
        RTI_SendDataReq( responseBuf.destIndex, responseBuf.profileId, responseBuf.vendorId, responseBuf.txOptions, responseBuf.msgLen, responseBuf.replyBuf );
        // Prepare to clear event
        procEvents |= ZRC_APP_EVT_SEND_SET_GET_ATTR_RESPONSE;
    }

    // Clear event
    timer_clear_event(ZRC_App_threadId, procEvents);
    LOG_DEBUG("0x%.8X events processed\n", procEvents);

}

/**************************************************************************************************
 *
 * @fn      ZRCApp_SendDataReq
 *
 * @brief   Abstraction of RTI_SendDataReq() to route callbacks.
 *
 *          NOTE: It is possible that this call can be made to the RTI client
 *                before the call to RTI_InitReq has returned.
 *
 * @param   status - Result of RTI_InitReq API call.
 *
 * @return  void
 */
uint8 ZRCApp_SendDataReq( uint8 dstIndex, uint8 profileId, uint16 vendorId, uint8 txOptions, uint8 len, uint8 *pData, zrcAppRtiDataCnfCbackFn_t cBack )
{
    uint8 retVal = TRUE;
    if (zrcAppState == AP_STATE_READY)
    {
        // Set state
        zrcAppState = AP_STATE_NDATA;
        // Setup callback
        zrcAppSendDataCnfCb = cBack;
        char responseStr[512];
        size_t strLen;
        snprintf(responseStr, sizeof(responseStr), "%.2X", pData[0]);
        strLen = 2;
        int i;
        for (i = 1; i < len; i++)
        {
            snprintf(responseStr+strLen, sizeof(responseStr)-strLen, " %.2X", pData[i]);
        strLen += 3;
        }
        LOG_DEBUG("[Data] Calling RTI_SendDataReq (Profile %d, Data: %s)\n", profileId, responseStr);
        // Send Data
        RTI_SendDataReq( dstIndex, profileId, vendorId, txOptions, len, pData);
    }
    else
    {
        retVal = FALSE;
        LOG_WARN("[Data] Failed to call RTI_SendDataReq (State %d -- %s)\n", zrcAppState, AppState_list[zrcAppState]);
    }
    return retVal;
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
        LOG_INFO("RTI_InitCnf(0x%.2X - %s)\n", status, rtiStatus_list[status]);

        uint8 startupFlg;
        // Next time we want to restore state from NV, in other words perform a Warm Start as specified in the RF4CE specification.
        startupFlg = RESTORE_STATE;
        RTI_WriteItem(RTI_CP_ITEM_STARTUP_CTRL, 1, &startupFlg);

        zrcAppState = AP_STATE_READY;

        zrcCfgPostInitializationConfiguration();

        // Check for pairing entries
        uint8 activePairingTableSize = 0;
        (void) RTI_ReadItemEx(RTI_PROFILE_RTI,
                RTI_SA_ITEM_PT_NUMBER_OF_ACTIVE_ENTRIES, 1,
                (uint8*) &activePairingTableSize);

        // Initialize/Align RF statistics
        //        zrcAppUpdateRFstatistics();

        LOG_INFO("Entered %s [0x%.2X]\n", AppState_list[zrcAppState], zrcAppState);
        //Display menu...
        DispMenuReady();

        LOG_INFO("Display messages for simple testing\n");
    }
    else
    {
        LOG_ERROR("RTI_InitCnf(0x%.2X - %s)\n", status, rtiStatus_list[status]);
        // Try to reset RNP
        RTI_SwResetReq();

        // Schedule event to try again.
        timer_start_timerEx(ZRC_App_threadId, ZRC_APP_EVT_INIT, 1200); // Retry initialization after 1.2 seconds
    }

    //Unlock Mutex So that Application can continue
    LOG_DEBUG("[MUTEX] Unlock zrcAppInit Mutex\n");
    pthread_mutex_unlock(&zrcAppInitMutex);
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
    zrcAppState = AP_STATE_READY;

    if (status == RTI_SUCCESS)
    {
        LOG_INFO("RTI_PairCnf(0x%.2X - %s), dstIndex: 0x%.2X, devType: %s[0x%.2X]\n",
                status, rtiStatus_list[status], dstIndex,
                (devType < RTI_DEVICE_TARGET_TYPE_END) ?
                        rtiDevType_list[devType] : "Unknown", devType);
    }
    else
    {
        LOG_ERROR("RTI_PairCnf(0x%.2X - %s)\n", status, rtiStatus_list[status]);
    }

    zrcAppValidationState = ZRC_READY;
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
    // Check if callback should be routed to sub-modules
    if (zrcAppSendDataCnfCb)
    {
        zrcAppRtiDataCnfCbackFn_t tmp = zrcAppSendDataCnfCb;
        zrcAppSendDataCnfCb = NULL;
        tmp(status);
    }

    if (zrcAppState == AP_STATE_READY)
    {
        LOG_INFO("RTI_SendDataCnf(0x%.2X - %s), but sent in wrong state: %s\n",
                status, rtiStatus_list[status], AppState_list[zrcAppState]);
    }
    else if ( zrcAppValidationState == ZRC_VALIDATION )
    {
        if (status == RTI_ERROR_NOT_PERMITTED)
        {
            LOG_WARN("[Validation] Attempt collided with stack sending Get Validation Status Request, please try same digit again!\n");
        }
        else
        {
            LOG_INFO("[Validation] Ready for new digit (0x%.2X - %s)\n", status, rtiStatus_list[status]);
        }
        // Go back to validation state to allow new digit to be transmitted
        zrcAppState = AP_STATE_VALIDATION;
    }
#ifdef RTI_TESTMODE
    else if (zrcAppState == AP_STATE_LATENCY_TEST_MODE)
    {
        RTI_TestModeSendDataCnf(status);
    }
#endif //RTI_TESTMODE
    else
    {
        if (status != 0x00)
        {
            LOG_INFO("RTI_SendDataCnf(0x%.2X - %s) (%s)\n", status,
                    rtiStatus_list[status], (TRUE == zrcCfgIsTarget()) ? "Target" : "Controller");
        }
        else
        {
            LOG_INFO("RTI_SendDataCnf(SUCCESS) (%s)\n", (TRUE == zrcCfgIsTarget()) ? "Target" : "Controller");
        }
        // Return to Send Data Prepare state, only if we're Controller

        if (TRUE == zrcCfgIsTarget())
        {
            zrcAppState = AP_STATE_READY;
        }
        else
        {
            zrcAppState = AP_STATE_NDATA_PREPARE;
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
void RTI_StandbyCnf(rStatus_t status)
{
    if (status == RTI_SUCCESS)
    {
        if (zrcAppRNPpowerState & ZRC_APP_RNP_POWER_STATE_STANDBY_BIT)
        {
            LOG_INFO("Exited Standby\n");
            zrcAppRNPpowerState &= ~(ZRC_APP_RNP_POWER_STATE_STANDBY_BIT);
        }
        else
        {
            LOG_INFO("Entered Standby\n");
            zrcAppRNPpowerState |= ZRC_APP_RNP_POWER_STATE_STANDBY_BIT;

            // Enable Sleep here, NPI is not responsive after this
            RTI_EnableSleepReq();
        }
    }
    else
    {
        LOG_ERROR("RTI_StandbyCnf(0x%.2X - %s)\n", status, rtiStatus_list[status]);
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
        uint8 rxLQI, uint8 rxFlags, uint8 len, uint8 *pData)
{
    int i, error = TRUE;
    //    static uint8 lastSource = RTI_INVALID_PAIRING_REF;

    //check Basic Range to avoid Seg Fault
    if ( ((profileId >= 0) && (profileId < RTI_PROFILE_ID_STD_END)) ||
            ((profileId >= RTI_PROFILE_ZRC20) && (profileId < RTI_PROFILE_MSO_END) ) )
    {
        error = FALSE;
    }

    //Vendor Id Meaningful only if Vendor Specific Data
    if (rxFlags & RTI_RX_FLAGS_VENDOR_SPECIFIC)
    {
        if ( (vendorId < 0) || ((vendorId > 7) && (vendorId != RTI_VENDOR_TEST_VENDOR)) )
        {
            error = TRUE;
        }
    }

    if ( (RTI_PROFILE_GDP == profileId) && (error != TRUE) )
    {
        zrcAppProcessGDPProfileData(srcIndex, len, pData, rxLQI);
    }
    else if ( (RTI_PROFILE_ZRC20 == profileId) && (error != TRUE) )
    {
        zrcAppProcessZRC20ProfileData(srcIndex, len, pData, rxLQI);
    }
    else if (RTI_PROFILE_ZRC == profileId)
    {
#ifdef RTI_TESTMODE
        if (zrcAppState == AP_STATE_LATENCY_TEST_MODE)
        {
            RTI_TestModeReceiveDataInd(srcIndex, profileId, vendorId, rxLQI, rxFlags, len, pData);
        }
        else 
#endif //RTI_TESTMODE
        if (pData[0] == RTI_PROTOCOL_TEST)
        {
#ifdef RTI_TESTMODE
            zrcAppState = AP_STATE_LATENCY_TEST_MODE;
            RTI_EnterTestMode();
            // We received test data, enter testmode and let test module handle data
            RTI_TestModeReceiveDataInd(srcIndex, profileId, vendorId, rxLQI, rxFlags, len, pData);
#else
            char tmpStr[512];
            size_t strLen = 0;
            for (i = 0; i < len; i++)
            {
                snprintf(tmpStr+strLen, sizeof(tmpStr)-strLen, "%.2X ", pData[i]);
                strLen += 3;
            }
            LOG_INFO("Unknown ZRC Data: %s (len = %d)\n", tmpStr, len);
#endif //RTI_TESTMODE
        }
        else if (pData[0] <= RTI_CERC_USER_CONTROL_RELEASED)
        {
            LOG_INFO("[ZRC 1.1] ZRC Data: Cmd: %d (%s), Key %d (%s) \n", pData[0],
                    Command_list[pData[0]] ? Command_list[pData[0]] : "?",
                            pData[1],
                            ZRC_Key_list[pData[1]] ? ZRC_Key_list[pData[1]] : "?");
            if ((pData[0] == RTI_CERC_USER_CONTROL_PRESSED) || (pData[0] == RTI_CERC_USER_CONTROL_REPEATED))
            {
                // Track lost release
                timer_start_timerEx(ZRC_App_threadId, ZRC_APP_EVT_KEY_RELEASE_LOST(srcIndex), ZRC_APP_KEY_RELEASE_LOST_TIMEOUT);
            }
            else
            {
                // Release received, stop timer by setting timer to 0
                timer_start_timerEx(ZRC_App_threadId, ZRC_APP_EVT_KEY_RELEASE_LOST(srcIndex), 0);
            }
        }
        else if (pData[0] == 0x50)
        {
            static uint8 faEnable = TRUE, blocks = 0;
            // As a minimum; stop FA on reception of Start
            if (pData[1] == 0x00)
            {
                if (faEnable != FALSE)
                {
                    faEnable = FALSE;
                    RTI_WriteItemEx(RTI_PROFILE_RTI, RTI_SA_ITEM_AGILITY_ENABLE, 1, (uint8 *)&faEnable);
                    LOG_INFO("Frequency Agility disabled as Voice starts\n");
                }
                blocks = 0;
            }
            // and re-enable on reception of Stop
            else if (pData[1] == 0x02)
            {
                faEnable = TRUE;
                RTI_WriteItemEx(RTI_PROFILE_RTI, RTI_SA_ITEM_AGILITY_ENABLE, 1, (uint8 *)&faEnable);
                LOG_INFO("Frequency Agility re-enabled as Voice ends\n");
            }
            else if (pData[1] == 0x01)
            {
                LOG_INFO("Voice Data, %d\n", blocks++);
            }
        }
    }
    else
    {
        char tmpStr[512];
        size_t strLen = 0;
        for (i = 0; i < len; i++)
        {
            snprintf(tmpStr+strLen, sizeof(tmpStr)-strLen, "%.2X ", pData[i]);
            strLen += 3;
            if ((strLen + 3) > len)
            {
                break;
            }
        }
        LOG_INFO("Raw Data: %s (len = %d)\n", tmpStr, len);
    }
}

/**************************************************************************************************
 *
 * @fn      zrcAppProcessZRC20ProfileData
 *
 * @brief   Local function to handle ZRC Profile data
 *
 * @param   srcIndex    - pairing table index of Controller sending the command
 *             len            - length of incoming data
 *             pData        - pointer to data sent by Controller
 *             rxLQI        - received link quality
 *
 * @return  void
 */
static void zrcAppProcessZRC20ProfileData(uint8 srcIndex, uint8 len, uint8 *pData, uint8 rxLQI)
{
    uint8 i;
    struct timeval    tv;
    char tmpStr[512];
    size_t strLen = 0;
    for (i = 0; i < len; i++)
    {
        snprintf(tmpStr+strLen, sizeof(tmpStr)-strLen, "%.2X ", pData[i]);
        strLen += 3;
    }
    LOG_DEBUG("[ZRC 2.0] Raw Data: %s (len = %d)\n", tmpStr, len);
    LOG_DEBUG("[ZRC 2.0] ZRC Application Validation state: %d \n", zrcAppValidationState);

    uint8 cmd;
    uint8 actionType;
    uint8 actionPayloadLen;
    uint8 actionBank;
    uint8 actionCode;
    uint8 *pBuf = pData;
    uint8 *pEndBuf = pBuf + len;

    cmd = *pBuf++;
    switch (cmd)
    {
    case ZRC_CMD_ID_ACTIONS:
        if (pBuf == pEndBuf)
        {
            LOG_INFO("[ZRC 2.0][%d][%3d dBm] Key released.\n", srcIndex, ZRC_APP_LQI_TO_DBM_CONVERSION(rxLQI));
            // Release received, stop timer by setting timer to 0
            timer_start_timerEx(ZRC_App_threadId, ZRC_APP_EVT_KEY_RELEASE_LOST(srcIndex), 0);
        }
        else
        {
            while (pBuf < pEndBuf)
            {
                actionType = *pBuf++;
                actionType &= ZRC_ACTION_TYPE_MASK;
                actionPayloadLen = *pBuf++;
                actionBank = *pBuf++;
                actionCode = *pBuf++;

                strLen = 0;
                if (actionPayloadLen > 0)
                {
                    snprintf(tmpStr, sizeof(tmpStr)-strLen, "Payload: ");
                    strLen += 9;
                }
                i = 0;
                while (actionPayloadLen > 0)
                {
                    snprintf(tmpStr+strLen, sizeof(tmpStr)-strLen, "%2.2X ", pBuf[i]);
                    strLen += 3;
                    i++;

                    actionPayloadLen--;
                }

                if ( (actionType == ZRC_ACTION_CTRL_TYPE_START) ||
                        (actionType == ZRC_ACTION_CTRL_TYPE_REPEAT) ||
                        (actionType == ZRC_ACTION_CTRL_TYPE_ATOMIC) )
                {
                    if ( (actionBank == ZRC_ACTION_BANK_TI_HID) &&
                            (actionCode == ZRC_TI_ACTION_CODE_HID_REPORT_DATA) )
                    {
                        // TI HID report
                        uint16 vendorId = BUILD_UINT16( *pBuf, *(pBuf+1) );
                        pBuf += 2;
                        if (vendorId == RTI_VENDOR_TEXAS_INSTRUMENTS)
                        {
                            // Save info needed in case we need to send a NULL report later
                            zrcDongleNullReportInfo.pairIndex = srcIndex;
                            zrcDongleNullReportInfo.reportId = ((zrc_report_record_t *)pBuf)->id;

                            // process HID report
                            if (((zrc_report_record_t *)pBuf)->id == ZRC_STD_REPORT_KEYBOARD)
                            {
                                LOG_INFO("[ZRC 2.0][%d][%3d dBm] Keyboard HID report received: 0x%.2X 0x%.2X\n", srcIndex,
                                        ZRC_APP_LQI_TO_DBM_CONVERSION(rxLQI), ((zrc_report_record_t *)pBuf)->data[0],
                                        ((zrc_report_record_t *)pBuf)->data[2]);
                            }
                            else if (((zrc_report_record_t *)pBuf)->id == ZRC_STD_REPORT_MOUSE)
                            {
                                LOG_INFO("[ZRC 2.0][%d][%3d dBm] Mouse HID report received: 0x%.2X x:%d y:%d\n", srcIndex,
                                        ZRC_APP_LQI_TO_DBM_CONVERSION(rxLQI), ((zrc_report_record_t *)pBuf)->data[0],
                                        (int8)((zrc_report_record_t *)pBuf)->data[1], (int8)((zrc_report_record_t *)pBuf)->data[2]);
                            }
                            pBuf += ((zrc_report_record_t *)pBuf)->len + offsetof(zrc_report_record_t, data);
                        }
                    }
                    else if (actionBank == ZRC_ACTION_BANK_HDMI_CEC)
                    {
                        if ( (zrcAppValidationState == ZRC_VALIDATION) && (actionType != ZRC_ACTION_CTRL_TYPE_REPEAT) )
                        {
                            LOG_INFO("[Validation] Key %s [0x%x] pressed during Validation process\n", ZRC_Key_list[actionCode], actionCode);
                            zrcAppProcessValidationCommand(srcIndex, actionCode);
                        }
                        else
                        {
                            if (actionType == ZRC_ACTION_CTRL_TYPE_START)
                            {
                                LOG_INFO("[ZRC 2.0][%d][%3d dBm] Key %s [0x%x] pressed. \t%s\n", srcIndex,
                                        ZRC_APP_LQI_TO_DBM_CONVERSION(rxLQI), ZRC_Key_list[actionCode], actionCode, tmpStr);
                            }
                            else if (actionType == ZRC_ACTION_CTRL_TYPE_REPEAT)
                            {
                                LOG_INFO("[ZRC 2.0][%d][%3d dBm] Key %s [0x%x] repeated. \t%s\n", srcIndex,
                                        ZRC_APP_LQI_TO_DBM_CONVERSION(rxLQI), ZRC_Key_list[actionCode], actionCode, tmpStr);
                            }
                            else
                            {
                                LOG_INFO("[ZRC 2.0][%d][%3d dBm] Key %s [0x%x] released. \t%s\n", srcIndex,
                                        ZRC_APP_LQI_TO_DBM_CONVERSION(rxLQI), ZRC_Key_list[actionCode], actionCode, tmpStr);
                            }
                            /*********************************************************
                             * Add info in boundRemotes table
                             */
                            gettimeofday(&tv, NULL);
                            zrcCfgUpdateLastKeypress(srcIndex, actionCode, (unsigned long)tv.tv_sec, rxLQI);

                            //                    /************************************************************
                            //                     * We want to update the information in NV, but not too often
                            //                     * Only Power key should be acted upon fast.
                            //                     */
                            //                    if (actionCode == ZRC_KEY_CMD_POWER_TOGGLE)
                            //                    {
                            //                        timer_set_event(ZRC_App_threadId, ZRC_APP_EVT_STORE_TO_NV);
                            //                    }
                            //                    else
                            //                    {
                            //                        timer_start_timerEx(ZRC_App_threadId, ZRC_APP_EVT_STORE_TO_NV, ZRC_APP_STORE_TO_NV_TIMEOUT_SLOW);
                            //                    }

                            if (actionType == ZRC_ACTION_CTRL_TYPE_ATOMIC)
                            {
                                // Release received, stop timer by setting timer to 0
                                timer_start_timerEx(ZRC_App_threadId, ZRC_APP_EVT_KEY_RELEASE_LOST(srcIndex), 0);
                            }
                            else
                            {
                                // Key repeat timer handled by IR Manager, but not lost release
                                timer_start_timerEx(ZRC_App_threadId, ZRC_APP_EVT_KEY_RELEASE_LOST(srcIndex), ZRC_APP_KEY_RELEASE_LOST_TIMEOUT);
                            }
                        }
                    }
                }
                else
                {
                    LOG_INFO("[ZRC 2.0] type: %s  bank: %2.2x, code: %2.2x. %s\n", zrcGetActionTypeStringBuf(actionType), actionBank, actionCode, tmpStr);
                }
            }
        }
        break;

    default:
        LOG_WARN("[ZRC 2.0][WARNING] Unknown ZRC 2.0 command received = 0x%.2X -- data = 0x%.2X\n", pData[0], pData[4] );
        break;
    }
}

/**************************************************************************************************
 *
 * @fn      zrcAppProcessGDPProfileData
 *
 * @brief   Local function to handle GDP Profile data
 *
 * @param   srcIndex    - pairing table index of Controller sending the command
 *             len            - length of incoming data
 *             pData        - pointer to data sent by Controller
 *             rxLQI        - received link quality
 *
 * @return  void
 */
static void zrcAppProcessGDPProfileData(uint8 srcIndex, uint8 len, uint8 *pData, uint8 rxLQI)
{
    uint8 i;
    char tmpStr[512];
    size_t strLen = 0;
    for (i = 0; i < len; i++)
    {
        snprintf(tmpStr+strLen, sizeof(tmpStr)-strLen, "%.2X ", pData[i]);
        strLen += 3;
    }
    LOG_DEBUG("[GDP] Raw Data: %s (len = %d)\n", tmpStr, len);
    LOG_DEBUG("[GDP] ZRC Application Validation state: %d \n", zrcAppValidationState);

    uint8 cmd = ((*pData) & GDP_FRM_CNTRL_CMD_CODE_MASK);
    switch (cmd)
    {
    case GDP_CMD_ID_PUSH_ATTRIBUTES:
        {
            zrcAppProcessGDPPushAttributes(srcIndex, len, pData);
            break;
        }
    default:
        LOG_WARN("[GDP] Unknown GDP command received = 0x%.2X -- data = 0x%.2X\n", pData[0], pData[4] );
        break;
    }
}

/**************************************************************************************************
 *
 * @fn      zrcAppProcessGDPPushAttributes
 *
 * @brief   Local function to handle received GDP Push Attributes command
 *
 * @param   srcIndex    - pairing table index of node sending the command
 *             len            - length of incoming data
 *             pData        - pointer to data sent by node
 *
 * @return  void
 */
static void zrcAppProcessGDPPushAttributes(uint8 srcIndex, uint8 len, uint8 *pData)
{
    gdpGenericRspCmd_t *pRsp = (gdpGenericRspCmd_t *)responseBuf.replyBuf;
    responseBuf.msgLen = sizeof(gdpGenericRspCmd_t);
    responseBuf.destIndex = srcIndex;
    responseBuf.profileId = RTI_PROFILE_GDP;
    responseBuf.txOptions = RTI_TX_OPTION_ACKNOWLEDGED | RTI_TX_OPTION_SECURITY;

    // - Save Attribute into the RIB inside the RNP.
    pRsp->rspCode = zrcCfgProcessGDPPushAttributesRequest(srcIndex, len, pData);

    // - Build the Response for later transmission
    pRsp->hdr = GDP_FRM_CNTRL_GDP_CMD_MASK | GDP_CMD_ID_GENERIC_RESPONSE;
    timer_start_timerEx(ZRC_App_threadId, ZRC_APP_EVT_SEND_SET_GET_ATTR_RESPONSE, ZRC_APP_SEND_SET_ATTR_RESPONSE_WAIT_TIME);
}

/**************************************************************************************************
 *
 * @fn      zrcAppProcessValidationCommand
 *
 * @brief   Local function to handled received commands during validation. It also handles
 *             the validation itself and notifies the UI through broadcasting events.
 *
 * @param   srcIndex    - pairing table index of Controller sending the command
 *             actionCode    - action code sent by Controller
 *
 * @return  void
 */
static void zrcAppProcessValidationCommand(uint8 srcIndex, uint8 actionCode)
{
    uint8 currentValidationDigit = 0xFF, digitValid = FALSE;

    // First make sure that we're talking to the expected remote
    if (srcIndex == zrcAppValidationSourceIndex)
    {
        if (actionCode == RTI_CERC_EXIT)
        {
            LOG_INFO("[Validation] Received Exit command during validation --> Full Abort\n");
            // Exit key was pressed during validation -- do a full abort
            zrcAppValidationState = ZRC_READY;
            zrcAppValidationStatus = GDP_CHECK_VALIDATION_STATUS_FAILURE;
            zrcAppValidationSourceIndex = RTI_INVALID_PAIRING_REF;
        }
        else if ((actionCode >= RTI_CERC_NUM_0) &&
                (actionCode <= RTI_CERC_NUM_9))
        {
            LOG_INFO("[Validation] Received Digit (%d) during validation\n", actionCode - RTI_CERC_NUM_0);
            if (zrcAppNumValidationDigitsPressed < sizeof(zrcAppValidationDigits))
            {
                currentValidationDigit = actionCode - RTI_CERC_NUM_0;
                // Validate digit
                LOG_INFO("[Validation] Validating digit %d against %d\n", currentValidationDigit, zrcAppGoldenDigits[zrcAppNumValidationDigitsPressed]);
                if (zrcAppGoldenDigits[zrcAppNumValidationDigitsPressed] == currentValidationDigit)
                {
                    digitValid = TRUE;
                }
                // Add digit to list
                zrcAppValidationDigits[zrcAppNumValidationDigitsPressed] = currentValidationDigit;
                LOG_INFO("[Validation] Digit list: [%d, %d, %d] vs [%d, %d, %d]\n", zrcAppValidationDigits[0],
                        zrcAppValidationDigits[1], zrcAppValidationDigits[2], zrcAppGoldenDigits[0],
                        zrcAppGoldenDigits[1], zrcAppGoldenDigits[2]);
            }
            // Increment number of validation digits pressed. This number should increment even for invalid digits
            // as invalid digits are also passed to UI.
            // If the digit was indeed invalid then it reset later on.
            zrcAppNumValidationDigitsPressed++;
            LOG_INFO("Number of digits pressed %d\n", zrcAppNumValidationDigitsPressed);

            if ( digitValid == FALSE )
            {
                // Give priority to maximum number of failed attempts
                // It is OK to always increment failed attempts as each incorrect digit will reset
                // from scratch, not from the last valid digit.
                if (++zrcAppNumFailedValidationAttempts >= zrcAppNumFailedValidationAttemptsMax)
                {
                    snprintf(tmpStrForTimePrint, sizeof(tmpStrForTimePrint), "[Validation][INFO] Invalid digit, and failed more than the maximum allowed attempts %d --> Failed Validation\n", zrcAppNumFailedValidationAttempts);
                    // Return to ready state
                    zrcAppValidationState = ZRC_READY;
                    zrcAppValidationStatus = GDP_CHECK_VALIDATION_STATUS_FAILURE;
                    zrcAppValidationSourceIndex = RTI_INVALID_PAIRING_REF;
                }
                else
                {
                    LOG_INFO("[Validation] Invalid digit, failed %d times --> Failure\n", zrcAppNumFailedValidationAttempts);

                    /**********************************************************
                     * Start over from beginning of validation. Increment number
                     * of failed attempts
                     */
                    zrcAppNumValidationDigitsPressed = 0;
                    memset(zrcAppValidationDigits, -1, sizeof(zrcAppValidationDigits));
                }
            }
            else if (zrcAppNumValidationDigitsPressed >= ZRC_VALIDATION_CODE_SIZE)
            {
                LOG_INFO("[Validation] Validation Completed --> Success\n");
                // If last digit was valid then we have completed validation successfully
                zrcAppValidationStatus = GDP_CHECK_VALIDATION_STATUS_SUCCESS;
            }
        }
        else
        {
            LOG_INFO("[Validation] Received User Command during validation\n");
        }
    }
    else
    {
        // Received command from remote that is not the one we're currently validating.

        if ((actionCode >= RTI_CERC_NUM_0) && (actionCode <= RTI_CERC_NUM_9))
        {
            // Received digit from another remote during validation. This is cause for collision
            LOG_INFO("[Validation] Received validation digit from unexpected Controller --> Collision\n");
            zrcAppValidationState = ZRC_READY;
            zrcAppValidationStatus = GDP_CHECK_VALIDATION_STATUS_FAILURE;
            zrcAppValidationSourceIndex = RTI_INVALID_PAIRING_REF;
        }
        else
        {
            LOG_INFO("[ZRC Validation] Received User Command during validation, but not from Controller currently validating\n");
        }
    }
}

void RTI_BindCnf( rStatus_t status, uint8 dstIndex )
{
    if (zrcAppState == AP_STATE_PAIR)
    {
        // Return to ready state
        zrcAppState = AP_STATE_READY;
    }
    LOG_INFO("RTI_BindCnf called with status 0x%.2X, dstIndex 0x%.2X\n", status, dstIndex);
}

void RTI_SendProfileCommandCnf( rStatus_t status )
{
    LOG_INFO("RTI_SendProfileCommandCnf called with status 0x%.2X\n", status);
}

void RTI_BindInd( rStatus_t status, uint8 dstIndex )
{
    if (zrcAppState == AP_STATE_PAIR)
    {
        zrcAppState = AP_STATE_READY;
        LOG_INFO("Binding completed for %d (Status 0x%.2X)\n", dstIndex, status);
    }
    else if ((dstIndex == zrcAppValidationSourceIndex) && (dstIndex != RTI_INVALID_PAIRING_REF))
    {
        LOG_INFO("Binding completed for %d (Status 0x%.2X)\n", dstIndex, status);

        // If pairing is successful add info in boundRemotes
        if (zrcAppValidationStatus == GDP_CHECK_VALIDATION_STATUS_SUCCESS)
        {
            // Perform post pairing updates to pairing table and statistics
            zrcCfgPostPairingProcessing(zrcAppValidationSourceIndex);

            // Update NV for recently bound remote
            timer_start_timerEx(ZRC_App_threadId, ZRC_APP_EVT_STORE_TO_NV, ZRC_APP_STORE_TO_NV_TIMEOUT_FAST);
        }

        zrcAppValidationState = ZRC_READY;
        zrcAppValidationStatus = GDP_CHECK_VALIDATION_STATUS_PENDING;
        zrcAppValidationSourceIndex = RTI_INVALID_PAIRING_REF;
    }
    else
    {
        LOG_WARN("Binding completed for unexpected node %d (Status 0x%.2X - %s)\n", dstIndex, status, rtiStatus_list[status]);
        zrcAppValidationStatus = GDP_CHECK_VALIDATION_STATUS_FAILURE;
    }
    zrcAppState = AP_STATE_READY;
}
void RTI_PollConfigCnf( rStatus_t status, uint8 *pPollConfig )
{

}
void RTI_IdentificationConfigCnf( rStatus_t status )
{

}
void RTI_PollCnf( rStatus_t status )
{

}
void RTI_KeyExchangeCnf( rStatus_t status )
{

}
void RTI_SendIrdbVendorSupportCnf( rStatus_t status )
{

}
void RTI_SendMappableActionsCnf( rStatus_t status )
{

}
void RTI_GetActionMappingsCnf( rStatus_t status, uint8 len, uint8 *pAttrData )
{

}
void RTI_HaSupportedAnnounceCnf( rStatus_t status )
{

}
void RTI_PullHaAttributesCnf( rStatus_t status )
{

}
void RTI_PollInd( uint8 pairIndex, uint8 trigger )
{
    if (zrcMsgQueue_isEmpty() == FALSE)
    {
        uint8 len;
        uint8 *pMsg;
        uint8 profileId;

        zrcMsgQueue_pop( &pMsg, &len, &profileId );
        if (zrcMsgQueue_isEmpty() == FALSE)
        {
            // tell originator device there's more to come
            *pMsg |= GDP_FRM_CNTRL_DATA_PENDING_MASK;
        }

        // send it
        RTI_PollRsp( profileId, len, pMsg );

        // free message memory
        free( pMsg );
    }
    else
    {
        RTI_PollRsp( RTI_PROFILE_GDP, 0, NULL );
    }

    LOG_INFO("Heartbeat!\n");
}
void RTI_UnbindCnf( rStatus_t status, uint8 dstIndex )
{

}
void RTI_UnbindInd( uint8 dstIndex )
{

}
void RTI_BindAbortCnf( rStatus_t status )
{

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
    {
        LOG_INFO("RTI_RxEnableCnf(0x%.2X - %s)\n", status, rtiStatus_list[status]);
    }
    else
    {
        LOG_ERROR("RTI_RxEnableCnf(0x%.2X - %s)\n", status, rtiStatus_list[status]);
    }
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
        LOG_INFO("Sleep enabled\n");
        zrcAppRNPpowerState |= ZRC_APP_RNP_POWER_STATE_NPI_BIT;
    }
    else
    {
        LOG_ERROR("RTI_EnableSleepCnf(0x%.2X - %s)\n", status, rtiStatus_list[status]);
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
        LOG_INFO("Sleep disabled\n");
        zrcAppRNPpowerState &= ~(ZRC_APP_RNP_POWER_STATE_NPI_BIT);
        // Toggle StandBy mode
        if (zrcAppRNPpowerState & ZRC_APP_RNP_POWER_STATE_STANDBY_BIT)
        {
            // We are in Standby, disable it
            RTI_StandbyReq(RTI_STANDBY_OFF);
        }
        else
        {
            LOG_WARN("Already exited Standby\n");
        }
    }
    else
    {
        LOG_ERROR("RTI_DisableSleepCnf(0x%.2X - %s)\n", status, rtiStatus_list[status]);
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
    if (zrcAppState == AP_STATE_PAIR)
    {
        zrcAppState = AP_STATE_READY;
        LOG_INFO("Controller %d did not accept pairing request. Check configuration, note that security is required for ZRC and ZID profiles. \n", dstIndex);
    }
    else
    {
        LOG_INFO("RTI_UnpairInd, dstIndex : %d \n", dstIndex);
    }
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
        LOG_INFO("RTI_PairAbortCnf(0x%.2X - %s)\n", status, rtiStatus_list[status]);
    }
    else
    {
        LOG_ERROR("RTI_DisableSleepCnf(0x%.2X - %s)\n", status, rtiStatus_list[status]);
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
    if (zrcAppState == AP_STATE_UNPAIR)
    {
        zrcAppState = AP_STATE_READY;
        LOG_INFO("RTI_UnpairCnf(0x%.2X - %s), dstIndex: 0x%2X\n",
                status,
                rtiStatus_list[status],
                dstIndex);
    }
    else
    {
        LOG_ERROR("RTI_UnpairCnf(0x%.2X - %s), dstIndex : %d , \n\tcalled from illegal state: %s\n",
                status,
                rtiStatus_list[status],
                dstIndex,
                AppState_list[zrcAppState]);
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
void RTI_ResetInd(void)
{
    static int numUnexpectedResets = 0;

    if (zrcAppState == AP_STATE_INIT_COLD)
    {
        uint8 startupFlg, retVal;
        startupFlg = CLEAR_STATE;
        retVal = RTI_WriteItem(RTI_CP_ITEM_STARTUP_CTRL, 1, &startupFlg);
        if (retVal != RTI_SUCCESS)
        {
            LOG_INFO("Could no set Cold Start flag\n");
        }
        else
        {
            LOG_INFO("Successfully set Cold Start flag\n");
        }

        zrcAppState = AP_STATE_INIT;

        // Disable event to initialize since we caught the reset from the RNP,
        // note that this is a new feature as of RemoTI-1.3.1
        timer_start_timerEx(ZRC_App_threadId, ZRC_APP_EVT_INIT, 100);
        return;
    }
//    else if (zrcAppState == AP_STATE_INIT_SBL)
//    {
//        LOG_INFO("RNP reset during Serial Update\n");
//    }
    else if (zrcAppState == AP_STATE_RESET)
    {
        LOG_INFO("RNP reset as requested\n");
        zrcAppState = AP_STATE_INIT;
        // We don't want to initialize just yet.
        return;
    }
    else if (zrcAppState == AP_STATE_INIT)
    {
        LOG_INFO("RNP reset during init, not unexpected\n");
        // Continue initialization.
        return;
    }
    else
    {
        ++numUnexpectedResets;
        // Warn user of reset
        LOG_WARN("RNP reset unexpectedly! (#%u)\n", numUnexpectedResets);
    }

    // RNP is now back in default state
    zrcAppRNPpowerState = ZRC_APP_RNP_POWER_STATE_ACTIVE;

    // Perform initialization from timer thread context, not AREQ thread context (which would prevent RTI_InitCnf() to be called)
    timer_set_event(ZRC_App_threadId, ZRC_APP_EVT_INIT);
    // Warn user of reset
    LOG_INFO("Timed init request started\n");
}

/**************************************************************************************************
 *
 * @fn      RTI_PairInd
 *
 * @brief   RTI indication that is used to notify AP about pairing during ZRC validation
 *
 * @param   void
 *
 *
 * @return  void
 */
void RTI_PairInd( uint8 status, uint8 srcIndex, uint8 devType)
{
    (void) devType;

    // If we are currently in validation and status is unsuccessful then it is because
    // it timed out
    if ( zrcAppValidationState == ZRC_VALIDATION )
    {
        LOG_INFO("[Validation] Pairing indication received, status 0x%.2X, from %d\n", status, srcIndex);
        /****************************************************************************
         *  If zrcAppValidationStatus is still ZRC_CHECK_VALIDATION_STATUS_PENDING it
         *  means that the validation failed because of timeout on the RNP
         *  All other failure causes are captured in zrcAppProcessValidationCommand
         *  and hence no other message is required to the UI.
         */
        if (zrcAppValidationStatus == GDP_CHECK_VALIDATION_STATUS_PENDING)
        {
            LOG_INFO("[Validation] RNP Timed Out --> Timeout\n");
        }

        // Return to READY state regardless of success/failure
        zrcAppValidationState = ZRC_READY;
        zrcAppValidationSourceIndex = RTI_INVALID_PAIRING_REF;

        // Go back to ready state
        zrcAppState = AP_STATE_READY;
        // Buffers are cleared as we begin next validation
    }
    else
    {
        LOG_WARN("Pairing indication received, but we're not in validation state, status 0x%.2X, from %d\n", status, srcIndex);
    }
}

/**************************************************************************************************
 *
 * @fn      RTI_StartValidationInd
 *
 * @brief   used to indicate the start of the ZRC validation
 *
 * @param   void
 *
 *
 * @return  void
 * */
void RTI_StartValidationInd( uint8 srcIndex )
{
    //We have a request to Start UI
    // change State of app
    zrcAppValidationState = ZRC_VALIDATION;
    zrcAppValidationSourceIndex = srcIndex;

    if (TRUE == zrcCfgIsTarget())
    {
        LOG_INFO("[Validation] Validate ZRC pairing\n");
        zrcAppValidationStatus = GDP_CHECK_VALIDATION_STATUS_PENDING;
        // re-init digits counters
        zrcAppNumValidationDigitsPressed = 0;
        memset(zrcAppValidationDigits, -1 , sizeof(zrcAppValidationDigits));
        zrcAppNumFailedValidationAttempts = 0;

        /*********************************************************
         * Generate random 3-digit validation code.
         */
        static bool randomSeedGenerated = FALSE;

        // generate random seed if necessary
        if (randomSeedGenerated == FALSE)
        {
            srand( (unsigned int)time( NULL ) );
            randomSeedGenerated = TRUE;
        }

        // generate random 3-digit code
        int randVal = rand();
        zrcAppGoldenDigits[0] = (uint8)((randVal % 1000) / 100);
        zrcAppGoldenDigits[1] = (uint8)((randVal % 100) / 10);
        zrcAppGoldenDigits[2] = (uint8)(randVal % 10);
        LOG_INFO("[Validation] Golden code: [%d, %d, %d]\n", zrcAppGoldenDigits[0], zrcAppGoldenDigits[1], zrcAppGoldenDigits[2]);
    }
    zrcAppState = AP_STATE_VALIDATION;
}

/**************************************************************************************************
 *
 * @fn      RTI_GetValidationStatusInd
 *
 * @brief   used to request the validation status
 *
 * @param   void
 *
 *
 * @return  void
 * */
void RTI_GetValidationStatusInd( )
{
    LOG_DEBUG("[Validation] validation status: 0x%x\n", zrcAppValidationStatus);
    RTI_GetValidationStatusRsp( zrcAppValidationStatus );
    if (zrcAppValidationStatus != GDP_CHECK_VALIDATION_STATUS_PENDING)
    {
        LOG_INFO("[Validation] Validation Completed - status = %2.2x\n", zrcAppValidationStatus);
    }
}

static char * zrcGetActionTypeStringBuf( uint8 type )
{
    static char *stringBuf[4] =
    {
            "RESERVED",
            "START",
            "REPEAT",
            "ATOMIC"
    };

    if (type < 4)
    {
        return stringBuf[type];
    }
    else
    {
        return "INVALID";
    }
}

void zrcBuildIdentifyClientNotificationCmd( uint8 *pLen, uint8 **ppBuf )
{
    gdpClientNotificationCmd_t *pMsg;
    uint8 *pPayload;
    static uint16 identifyTime = 1; // in seconds

    // get memory for message
    *pLen = sizeof( gdpClientNotificationCmd_t ) + sizeof(gdpClientNotificationIdentify_t);
    pMsg = (gdpClientNotificationCmd_t *)malloc( *pLen );
    *ppBuf = (uint8 *)pMsg;

    // fill in common fields
    pMsg->hdr = GDP_CMD_BUILD_CMD_ID(GDP_CMD_ID_CLIENT_NOTIFICATION);
    pMsg->subType = GDP_CLIENT_NOTIFICATION_SUB_TYPE_IDENTIFY;
    pPayload = &pMsg->payload[0];
    *pPayload++ = GDP_CLIENT_NOTIFICATION_FLAGS_FLASH_LIGHT_MASK;
    *pPayload++ = LO_UINT16( identifyTime );
    *pPayload++ = HI_UINT16( identifyTime );
    identifyTime++;

    if (identifyTime > 3)
    {
        identifyTime = 1;
    }
}

void zrcMsgQueue_push( uint8 *pMsg, uint8 len, uint8 profileId)
{
    zrcMsgQueue_t *pNode = (zrcMsgQueue_t *)malloc( sizeof(zrcMsgQueue_t) );
    pNode->msgLen = len;
    pNode->pMsg = pMsg;
    pNode->profileId = profileId;
    pNode->pNext = NULL;

    if (pRsaMsgQueueHead == NULL)
    {
        pRsaMsgQueueHead = pNode;
    }
    else
    {
        zrcMsgQueue_t *pTemp = pRsaMsgQueueHead;
        while (pTemp->pNext != NULL)
        {
            pTemp = pTemp->pNext;
        }
        pTemp->pNext = pNode;
    }
}

void zrcMsgQueue_pop( uint8 **ppMsg, uint8 *pLen, uint8 *pProfileId )
{
    zrcMsgQueue_t *pTemp = pRsaMsgQueueHead;
    if (pRsaMsgQueueHead != NULL)
    {
        *ppMsg = pRsaMsgQueueHead->pMsg;
        *pLen = pRsaMsgQueueHead->msgLen;
        *pProfileId = pRsaMsgQueueHead->profileId;
        pRsaMsgQueueHead = pRsaMsgQueueHead->pNext;
        free( pTemp );
    }
    else
    {
        *ppMsg = NULL;
        *pLen = 0;
    }
}

bool zrcMsgQueue_isEmpty( void )
{
    return (pRsaMsgQueueHead == NULL);
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

int zrcAppGetAndPrintExtendedSoftwareVersion(swVerExtended_t *swVerExtended)
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

static int zrcAppGetAndPrintSoftwareVersions(uint8 *baseVersion, swVerExtended_t *swVerExtended)
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

    if ((*baseVersion <= 0x2D) ||
        (RTI_SUCCESS != (result = zrcAppGetAndPrintExtendedSoftwareVersion(swVerExtended))))
    {
        memset(swVerExtended, 0, sizeof(*swVerExtended));
    }

    return result;
}

#ifdef RTI_TESTMODE
static void zrcAppReturnFromSubmodule()
{
    zrcAppState = AP_STATE_READY;
    DispMenuReady();
}
#endif //RTI_TESTMODE

void DispMenuReady(void)
{
    LOG_INFO("------------------------------------------------------\n");
    LOG_INFO("Main MENU:\n");
    LOG_INFO("q- exit\n");
    if (TRUE == zrcCfgIsTarget())
    {
        LOG_INFO("1- Enable Button Pairing (by calling RTI_AllowBindReq())\n");
    }
    else
    {
        LOG_INFO("1- Initiate Binding Procedure (by calling RTI_BindReq())\n");
    }
    if (TRUE != zrcCfgIsTarget())
    {
        LOG_INFO("2- Unpairing\n");
        LOG_INFO("3- Enter Send Data State\n");
    }
    else
    {
        LOG_INFO("3- Enter Test Mode\n");
    }
    LOG_INFO("4- NP software version\n");
    LOG_INFO("f- Find Remotes\n");
    LOG_INFO("8- Clear Pairing Table\n");
    LOG_INFO("9- Display Pairing Table\n");
    LOG_INFO("Set Channel 15 ('h'), 20 ('j') or 25 '(k'). To re-enable FA ('l')\n");
    LOG_INFO("s- Let RNP know host goes to sleep\n");
    LOG_INFO("p- Acknowledge waking up on GPIO\n");
    LOG_INFO("d- Ask NPI Server to disconnect from RNP\n");
    LOG_INFO("c- Ask NPI Server to connect to RNP\n");
    LOG_INFO("t- Toggle __DEBUG_TIME_ACTIVE on Server\n");
    LOG_INFO("y- Toggle __BIG_DEBUG on Server\n");
    LOG_INFO("r- Reset RNP - Cold Start\n");
    LOG_INFO("m- Show This Menu\n");
}

void DispMenuInit(void)
{
    LOG_INFO("------------------------------------------------------\n");
    LOG_INFO("Init MENU:\n");
    LOG_INFO("q- exit\n");
    LOG_INFO("c- Start as Controller\n");
    LOG_INFO("t- Start as Target\n");
}
