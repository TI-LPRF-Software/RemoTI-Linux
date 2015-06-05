/**************************************************************************************************
 Filename:       lnxsample_main_target.cpp

 Description:    Linux SimpleConsole application (target node)

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
#include <string.h>
#include <getopt.h>
#include <poll.h>

#include "simple_app_main.h"
#include "simple_app.h"
#include "common_app.h"
#include "timer.h"
#include "lprfLogging.h"

#include "rti_lnx.h"
// Linux surrogate interface
#include "npi_ipc_client.h"
#include "npi_lnx_error.h"

#include "hal_rpc.h"
#define SB_DST_ADDR_DIV                    4

// macros
#define PAUSE() { fflush(stdout); while (getchar()!='\n'); }

#define NAME_ELEMENT(element) [element] = #element

const char * const rtiStatus_list[256] =
{
        NAME_ELEMENT(RTI_SUCCESS),
        NAME_ELEMENT(RTI_ERROR_INVALID_INDEX),
        NAME_ELEMENT(RTI_ERROR_INVALID_PARAMETER),
        NAME_ELEMENT(RTI_ERROR_UNSUPPORTED_ATTRIBUTE),
        NAME_ELEMENT(RTI_ERROR_NO_ORG_CAPACITY),                        //0xB0
        NAME_ELEMENT(RTI_ERROR_NO_REC_CAPACITY),                        //0xB1
        NAME_ELEMENT(RTI_ERROR_NO_PAIRING_INDEX),                       //0xB2
        NAME_ELEMENT(RTI_ERROR_NO_RESPONSE),                            //0xB3
        NAME_ELEMENT(RTI_ERROR_NOT_PERMITTED),                          //0xB4
        NAME_ELEMENT(RTI_ERROR_FRAME_COUNTER_EXPIRED),                  //0xB6
        NAME_ELEMENT(RTI_ERROR_DISCOVERY_ERROR),                        //0xB7
        NAME_ELEMENT(RTI_ERROR_DISCOVERY_TIMEOUT),                      //0xB8
        NAME_ELEMENT(RTI_ERROR_SECURITY_TIMEOUT),                       //0xB9
        NAME_ELEMENT(RTI_ERROR_SECURITY_FAILURE),                       //0xBA
        NAME_ELEMENT(RTI_ERROR_NO_SECURITY_KEY),                        //0xBD
        NAME_ELEMENT(RTI_ERROR_OUT_OF_MEMORY),                          //0xBE
        NAME_ELEMENT(RTI_ERROR_OSAL_NO_TIMER_AVAIL),                    //0x08
        NAME_ELEMENT(RTI_ERROR_OSAL_NV_OPER_FAILED),                    //0x0A
        NAME_ELEMENT(RTI_ERROR_OSAL_NV_ITEM_UNINIT),                    //0x09
        NAME_ELEMENT(RTI_ERROR_OSAL_NV_BAD_ITEM_LEN),                   //0x0C
        NAME_ELEMENT(RTI_ERROR_MAC_TRANSACTION_EXPIRED),                //0xF0
        NAME_ELEMENT(RTI_ERROR_MAC_TRANSACTION_OVERFLOW),               //0xF1
        NAME_ELEMENT(RTI_ERROR_MAC_NO_RESOURCES),                       //0x1A
        NAME_ELEMENT(RTI_ERROR_MAC_UNSUPPORTED),                        //0x18
        NAME_ELEMENT(RTI_ERROR_MAC_BAD_STATE),                          //0x19
        NAME_ELEMENT(RTI_ERROR_MAC_CHANNEL_ACCESS_FAILURE),             //0xE1
        NAME_ELEMENT(RTI_ERROR_MAC_NO_ACK),                             //0xE9
        NAME_ELEMENT(RTI_ERROR_MAC_BEACON_LOST),                        //0xE0
        NAME_ELEMENT(RTI_ERROR_MAC_PAN_ID_CONFLICT),                    //0xEE
        NAME_ELEMENT(RTI_ERROR_MAC_SCAN_IN_PROGRESS),                   //0xFC
        NAME_ELEMENT(RTI_ERROR_UNKNOWN_STATUS_RETURNED),                //0x20
        NAME_ELEMENT(RTI_ERROR_FAILED_TO_DISCOVER),                     //0x21
        NAME_ELEMENT(RTI_ERROR_FAILED_TO_PAIR),                         //0x22
        NAME_ELEMENT(RTI_ERROR_ALLOW_PAIRING_TIMEOUT),                  //0x23
        NAME_ELEMENT(RTI_ERROR_FAILED_TO_CONFIGURE_ZRC),                //0x41
        NAME_ELEMENT(RTI_ERROR_FAILED_TO_CONFIGURE_ZID),                //0x42
        NAME_ELEMENT(RTI_ERROR_FAILED_TO_CONFIGURE_Z3D),                //0x43
        NAME_ELEMENT(RTI_ERROR_FAILED_TO_CONFIGURE_INV_MASK),           //(0x40)
  // reserve failure IDs 0x44-0x4A for future profiles
        NAME_ELEMENT(RTI_ERROR_SYNCHRONOUS_NPI_TIMEOUT),                //0xFF
};

const char * const rtiProfileId_list[0xFF + 1] =
{
		[0 ... RTI_PROFILE_RTI] = NULL,
		NAME_ELEMENT(RTI_PROFILE_GDP),
		NAME_ELEMENT(RTI_PROFILE_ZRC),
		NAME_ELEMENT(RTI_PROFILE_ZID),
		NAME_ELEMENT(RTI_PROFILE_ZRC20),
		NAME_ELEMENT(RTI_PROFILE_TI),
		NAME_ELEMENT(RTI_PROFILE_TOAD)
};

const char * const rtiVendorId_list[RTI_VENDOR_TEXAS_INSTRUMENTS + 1] =
{
		[0 ... RTI_VENDOR_TEXAS_INSTRUMENTS] = NULL,
		NAME_ELEMENT(RTI_VENDOR_PANASONIC),
		NAME_ELEMENT(RTI_VENDOR_SONY),
		NAME_ELEMENT(RTI_VENDOR_SAMSUNG),
		NAME_ELEMENT(RTI_VENDOR_PHILIPS),
		NAME_ELEMENT(RTI_VENDOR_FREESCALE),
		NAME_ELEMENT(RTI_VENDOR_OKI),
		NAME_ELEMENT(RTI_VENDOR_TEXAS_INSTRUMENTS)
};

const char * const rtiDevType_list[RTI_DEVICE_TARGET_TYPE_END] =
{
		[0 ... (RTI_DEVICE_TARGET_TYPE_END - 1)] = NULL,
		NAME_ELEMENT(RTI_DEVICE_RESERVED_INVALID),			//0x00
        NAME_ELEMENT(RTI_DEVICE_REMOTE_CONTROL),			//0x01
        NAME_ELEMENT(RTI_DEVICE_TARGET_TYPE_START),			//0x02
        NAME_ELEMENT(RTI_DEVICE_TELEVISION),				//0x02
        NAME_ELEMENT(RTI_DEVICE_PROJECTOR),					//0x03
        NAME_ELEMENT(RTI_DEVICE_PLAYER),					//0x04
        NAME_ELEMENT(RTI_DEVICE_RECORDER),					//0x05
        NAME_ELEMENT(RTI_DEVICE_VIDEO_PLAYER_RECORDER),		//0x06
        NAME_ELEMENT(RTI_DEVICE_AUDIO_PLAYER_RECORDER),		//0x07
        NAME_ELEMENT(RTI_DEVICE_AUDIO_VIDEO_RECORDER),		//0x08
        NAME_ELEMENT(RTI_DEVICE_SET_TOP_BOX),				//0x09
        NAME_ELEMENT(RTI_DEVICE_HOME_THEATER_SYSTEM),		//0x0A
        NAME_ELEMENT(RTI_DEVICE_MEDIA_CENTER_PC),			//0x0B
        NAME_ELEMENT(RTI_DEVICE_GAME_CONSOLE),				//0x0C
        NAME_ELEMENT(RTI_DEVICE_SATELLITE_RADIO_RECEIVER),	//0x0D
        NAME_ELEMENT(RTI_DEVICE_IR_EXTENDER),				//0x0E
        NAME_ELEMENT(RTI_DEVICE_MONITOR)					//0x0F
};

const char * const testMode_txPower_list[28] =
{ " \0407\tdBm",
		" \0407\tdBm",
		" \0407\tdBm",
		" \0404.5\tdBm",
		" \0403\tdBm",
		" \0403\tdBm",
		" \0401.7\tdBm",
		" \0400.3\tdBm",
		" -1\tdBm",
		" -1\tdBm",
		" -2.8\tdBm",
		" -2.8\tdBm",
		" -4.1\tdBm",
		" -5.9\tdBm",
		" -5.9\tdBm",
		" -7.7\tdBm",
		" -7.7\tdBm",
		" -9.9\tdBm",
		" -9.9\tdBm",
		" -9.9\tdBm",
		"-12.8\tdBm",
		"-12.8\tdBm",
		"-14.9\tdBm",
		"-14.9\tdBm",
		"-16.6\tdBm",
		"-16.6\tdBm",
		"-18.7\tdBm",
		"-18.7\tdBm"
};

// Pairing reference
uint8 destIdx;

struct pollfd fds[1];

const char *device = "";
const char *debugOption = "";

/* Global variable definitions. Declared as externs in common_app.h */
sem_t eventSem;
int __APP_LOG_LEVEL = LOG_LEVEL_INFO;

enum 
{
	RTI_main_linux_threadId, SIMPLE_App_threadId, RTI_main_threadId_tblSize
};

static void print_usage(const char *prog) {
	printf("Usage: %s [-DlHOLC3]\n", prog);
	puts(
			"  -D --device      device to use (default /dev/spidev4.0). For Socket use format IPaddress:port\n"
			"  -d --debugOption debugAll: both time and big, debugTime: only timestamps, debugBig: verbose debug\n"
		);
	exit(1);
}

static void parse_opts(int argc, char *argv[])
{
	while (1)
	{
		static const struct option lopts[] =
		{
			{ "device", 1, 0, 'D' },
			{ "debug", 1, 0, 'd' },
			{ NULL, 0, 0, 0 },
		};
		int c;

		c = getopt_long(argc, argv, "D:d:", lopts, NULL);

		if (c == -1)
			break;

		switch (c)
		{
		case 'D':
			device = optarg;
			break;
		case 'd':
			debugOption = optarg;
			break;
		default:
			print_usage(argv[0]);
			break;
		}
	}
}

int main(int argc, char **argv) 
{

	LOG_INFO("Starting...\n");

	// Return value for main
	int ret = 0;

	consoleInput.latestCh = ' ';
	consoleInput.handle = RTI_MAIN_INPUT_RELEASED;

	// setup filedescriptor for stdin
	fds[0].fd = fileno(stdin);
	fds[0].events = POLLIN;

	parse_opts(argc, argv);

	// Initialize shared semaphore. Must happen before program begins execution
	sem_init(&eventSem,0,1);

//	FILE *tmpFile;

	if ((ret = NPI_ClientInit(device)) != NPI_LNX_SUCCESS)
	{
		fprintf(stderr, "Failed to start RTI library module, device; %s\n", device);
		print_usage(argv[0]);
		return ret;
	}

	// Toggle Timer Print on Server state variable
	uint8 mode = 0;
	if (strcmp(debugOption, "debugAll") == 0)
	{
		LOG_INFO("!!! 1\n");
		mode = 3;
	}
	else if (strcmp(debugOption,"debugBig") == 0)
	{
		LOG_INFO("!!! 2\n");
		mode = 2;
	}
	else if (strcmp(debugOption,"debugTime") == 0)
	{
		LOG_INFO("!!! 3\n");
		mode = 1;
	}
	//Start RTI thread, management of RTI command in separate thread.
	if ((ret = SimpleAppInit(mode, SIMPLE_App_threadId)) != 0)
	{
		return ret;
	}

	LOG_INFO("Starting timer thread\n");

	//Start RTI timer thread, management of timer in separate thread.
	if ((ret = timer_init(RTI_main_threadId_tblSize)) != 0)
	{
		printf("Failed to start timer thread. Exiting...");
		return ret;
	}

	//first Menu will be display at the end of the RNP initialization.

	int pollRet;

	//MANAGE DISPLAY HERE
	while (1) 
	{
		//Wait for Display Mutex release before displaying anything.
		//pthread_mutex_lock(&appDisplayMutex);
		//pthread_mutex_unlock(&appDisplayMutex);

		//Check Display buffer, if not empty display one element

		// Check for input characters, timeout after 500ms
		pollRet = poll(fds, 1, 500);
		if ((pollRet == 1) && (consoleInput.handle == RTI_MAIN_INPUT_RELEASED))
		{
//			consoleInput.latestCh = getchar();
			fgets(consoleInput.latestStr, sizeof(consoleInput.latestStr), stdin);
			// Remove \n character from string
			char* p;
			if ( (p= strchr(consoleInput.latestStr, '\n')) != NULL)
				*p = '\0';
			consoleInput.latestCh = consoleInput.latestStr[0];
			if (consoleInput.latestCh == 'q')
			{
				ret = 0;
				break;
			}
			// Do not act on -1, . and new line (\n)
			if ( (consoleInput.latestCh != -1)
					&& (consoleInput.latestCh != '.')
					&& (consoleInput.latestCh != '\n') )
			{
				// Indicate to application thread that the input is ready
				consoleInput.handle = RTI_MAIN_INPUT_READY;
				// Release resources waiting for this event
				if (sem_post(&eventSem) < 0)
				{
					LOG_ERROR("[MAIN] Failed to post semaphore %p\n", &eventSem);
				}
			}
//			LOG_DEBUG("Character read: \t%c, int: %d\n", consoleInput.latestCh,
//					consoleInput.latestCh);
//			LOG_DEBUG("String read: \t%s\n", consoleInput.latestStr,
//					consoleInput.latestStr);
		}
//		LOG_DEBUG("poll returned: %d; console handle: %d\n", pollRet, consoleInput.handle);
	}

	// Destroy semaphores
	sem_destroy(&eventSem);

	NPI_ClientClose();

	return ret;
}

void DispMenuInit(void) 
{
	LOG_INFO("------------------------------------------------------\n");
	LOG_INFO("Init MENU:\n");
	LOG_INFO("1- Toggle Target / Controller\n");
	LOG_INFO("2- Set Node Capabilities\n");
	LOG_INFO("3- Supported Profiles\n");
	LOG_INFO("4- Supported Devices \n");
	LOG_INFO("5- Supported Target Types \n");
	LOG_INFO("i- Initialize without configuration. (Restore from NV).\n");
	LOG_INFO("g- Get current configuration from RNP.\n");
	LOG_INFO("l- Show configuration. Note! Not Necessarily the One Written To RNP\n");
	LOG_INFO("s- Apply Configuration and Move On To Application\n");
	LOG_INFO("r- Back To Application, Do Not Apply Changes\n");
	LOG_INFO("\tc- Start As Controller (default settings)\n");
	LOG_INFO("\tt- Start As Target (default settings)\n");
}

void DispCFGCurrentCfg(appDevInfo_t appCfg, uint16 nwkAddr, uint16 panId, uint8 *ieeAddr)
{
	int16 i;
	char* tmpStr[4];
	int strLen;
	char tmpStrLong[512];
	LOG_INFO("------------------------------------------------------\n");
	LOG_INFO("- Current Configuration:\n");
	LOG_INFO("- \tNode Capabilities: \t \t \t 0x%.2X\n", appCfg.nodeCapabilities);
	if (appCfg.nodeCapabilities & RTI_NODE_CAP_NODE_TYPE_BM)
		tmpStr[0] = "Target";
	else
		tmpStr[0] = "Controller";

	if (appCfg.nodeCapabilities & RTI_NODE_CAP_MAINS_PWR_BM)
		tmpStr[1] = "Mains Powered";
	else
		tmpStr[1] = "Battery Powered";

	if (appCfg.nodeCapabilities & RTI_NODE_CAP_SEC_CAP_BM)
		tmpStr[2] = "Security Enabled";
	else
		tmpStr[2] = "-";

	if (appCfg.nodeCapabilities & RTI_NODE_CAP_CHAN_NORM_BM)
		tmpStr[3] = "Channel Normalization Supported";
	else
		tmpStr[3] = "-";

	for (i = 0; i < 4; i++)
		LOG_INFO("- \t \t%s\n", tmpStr[i]);


	LOG_INFO("- \tApplication Capabilities: \t \t 0x%.2X\n", appCfg.appCapabilities);
	LOG_INFO("- \t \t Number of supported profiles:\t 0x%.2X\n",
			RCN_APP_CAPA_GET_NUM_PROFILES(appCfg.appCapabilities));
	LOG_INFO("- \t \t Number of supported devices:\t 0x%.2X\n",
			RCN_APP_CAPA_GET_NUM_DEV_TYPES(appCfg.appCapabilities));
	if (RCN_APP_CAPA_GET_USER_STRING(appCfg.appCapabilities))
		LOG_INFO("- \t \t User String supported:\t%s\n", appCfg.userString);
	LOG_INFO("- \tSupported Profiles:\n");
	for (i = 0; i < RCN_APP_CAPA_GET_NUM_PROFILES(appCfg.appCapabilities); i++)
	{
		LOG_INFO("- \t \t \t \t %s\n", rtiProfileId_list[appCfg.profileIdList[i]]);
	}
	if (appCfg.vendorId <= RTI_VENDOR_TEXAS_INSTRUMENTS)
	{
		LOG_INFO("- \tVendor ID: \t \t 0x%.2X (%s)\n", appCfg.vendorId, rtiVendorId_list[appCfg.vendorId]);
	}
	else
	{
		LOG_INFO("- \tVendor ID: \t \t 0x%.2X (Unknown)\n", appCfg.vendorId);
	}

	LOG_INFO("- \tDevice Types:\n");
	for (i = 0; i < RTI_MAX_NUM_DEV_TYPES; i++)
	{
		if (appCfg.devTypeList[i] <= RTI_DEVICE_TARGET_TYPE_END)
		{
			LOG_INFO("- \t \t \t \t 0x%.2X (%s)\n", appCfg.devTypeList[i], rtiDevType_list[appCfg.devTypeList[i]]);
		}
		else
		{
			LOG_INFO("- \t \t \t \t 0x%.2X (Unknown)\n", appCfg.devTypeList[i]);
		}
	}

	LOG_INFO("- \tTarget Device Types:\n");
	for (i = 0; i < RTI_MAX_NUM_SUPPORTED_TGT_TYPES; i++)
	{
		if (appCfg.tgtTypeList[i] <= RTI_DEVICE_TARGET_TYPE_END)
		{
			LOG_INFO("- \t \t \t \t 0x%.2X (%s)\n", appCfg.tgtTypeList[i], rtiDevType_list[appCfg.tgtTypeList[i]]);
		}
		else
		{
			LOG_INFO("- \t \t \t \t 0x%.2X (Unknown)\n", appCfg.tgtTypeList[i]);
		}
	}
	LOG_INFO("- \tNWK Address: \t \t 0x%.4X\n", nwkAddr);
	LOG_INFO("- \tPAN ID: \t \t 0x%.4X\n", panId);
	strLen = 0;
	for (i = (SADDR_EXT_LEN - 2); i >= 0; i--)
	{
		snprintf(tmpStrLong+strLen, sizeof(tmpStrLong)-strLen, ":%.2X", (ieeAddr[i] & 0x00FF));
		strLen += 3;
	}
	LOG_INFO("* IEEE Address:  		 %.2hX%s\n", (ieeAddr[SADDR_EXT_LEN - 1] & 0x00FF), tmpStrLong);
	LOG_INFO("------------------------------------------------------\n");
}

void DispCFGNodeCapMenu(void)
{
	LOG_INFO("------------------------------------------------------\n");
	LOG_INFO("- Configure Node Capabilites\n");
	LOG_INFO("- \tEnter as string of bytes separated by any of \n");
	LOG_INFO("- \tthese delimiters: ' ' , ; : - |\n");
	LOG_INFO("- Target/Controller, Mains/Battery, Security On/Off, Channel Normalization On/Off\n");
	LOG_INFO("- Example:\n");
	LOG_INFO("- \t 1, 1, 1, 0 \t yields:\n");
	LOG_INFO("- Target , Mains Powered, Security Enabled, Channel Normalization Off\n");
}

void DispCFGProfileIDMenu(void)
{
	LOG_INFO("------------------------------------------------------\n");
	LOG_INFO("- Configure Supported Profile IDs (Max %d)\n", RCN_MAX_NUM_PROFILE_IDS);
	LOG_INFO("- \tEnter as string of bytes separated by any of \n");
	LOG_INFO("- \tthese delimiters: ' ' , ; : - |\n");
	LOG_INFO("- Available Profiles:\n");
	DispNumberedStringList((char**)rtiProfileId_list, RTI_PROFILE_RTI + 1);
}

void DispCFGVendorIDMenu(void)
{
	LOG_INFO("------------------------------------------------------\n");
	LOG_INFO("- Configure Vendor ID\n");
	LOG_INFO("- \tEnter as 16 bit hexadecimal value \n");
}

void DispCFGdevTypesMenu(void)
{
	LOG_INFO("------------------------------------------------------\n");
	LOG_INFO("- Configure Supported Device Types (Max %d)\n", RTI_MAX_NUM_DEV_TYPES);
	LOG_INFO("- \tEnter as string of bytes separated by any of \n");
	LOG_INFO("- \tthese delimiters: ' ' , ; : - |\n");
}

void DispCFGtgtTypesMenu(void)
{
	LOG_INFO("------------------------------------------------------\n");
	LOG_INFO("- Configure Supported Target Types (Max %d)\n", RTI_MAX_NUM_SUPPORTED_TGT_TYPES);
	LOG_INFO("- \tEnter as string of bytes separated by any of \n");
	LOG_INFO("- \tthese delimiters: ' ' , ; : - |\n");
}

void DispNumberedStringList(char** strList, uint16 len)
{
	int i;
	LOG_INFO("------------------------------------------------------\n");
	for (i = 0; i < len; i++)
	{
		if (strList[i] != NULL)
			LOG_INFO("- %d \t%s\n", i, strList[i]);
	}
	LOG_INFO("------------------------------------------------------\n");
}

void DispStringList(char** strList, uint16 len)
{
	int i;
	LOG_INFO("------------------------------------------------------\n");
	for (i = 0; i < len; i++)
	{
		if (strList[i] != NULL)
			LOG_INFO("-\t%s\n", strList[i]);
	}
	LOG_INFO("------------------------------------------------------\n");
}

void DispMenuReady(void)
{
	LOG_INFO("------------------------------------------------------\n");
	LOG_INFO("Main MENU:\n");
	LOG_INFO("q- exit\n");
	LOG_INFO("0- Config \n");
	LOG_INFO("1- Pairing\n");
	LOG_INFO("2- Unpairing\n");
	LOG_INFO("6- Physically Reset RNP (does not work for USB)\n");
	LOG_INFO("7- Send Data\n");
	LOG_INFO("8- Clear Pairing Table\n");
	LOG_INFO("9- Display Pairing Table\n");
	LOG_INFO("Set Channel 15 ('h'), 20 ('j') or 25 '(k'). To re-enable FA ('l')\n");
	LOG_INFO("t- Toggle __DEBUG_TIME_ACTIVE on Server\n");
	LOG_INFO("y- Toggle __BIG_DEBUG on Server\n");
	LOG_INFO("c- Toggle simple data display\n");
	LOG_INFO("s- Toggle standby mode\n");
	LOG_INFO("a- Check States\n");
	LOG_INFO("g- Get MAC Channel\n");
	LOG_INFO("v- Control Attenuator\n");
	LOG_INFO("r- Reset RNP\n");
	LOG_INFO("m- Show This Menu\n");
}

void DispPhyTestModeMenu(void)
{
	LOG_INFO("------------------------------------------------------\n");
	LOG_INFO("Physical Test Mode MENU:\n");
	LOG_INFO("r- Return to Main Menu\n");
	LOG_INFO("1- Tx Raw Carrier\n");
	LOG_INFO("2- Tx Modulated Carrier\n");
	LOG_INFO("3- Rx Test\n");
	LOG_INFO("m- Show This Menu\n");
}

void DispPhyTestModeActiveMenu(void)
{
	LOG_INFO("------------------------------------------------------\n");
	LOG_INFO("Test Running:\n");
	LOG_INFO("s- Stop Test\n");
}

void DispPhyTestModeRxMenu(void)
{
	LOG_INFO("------------------------------------------------------\n");
	LOG_INFO("Physical Test Mode Rx MENU:\n");
	LOG_INFO("r- Return to Previous Menu\n");
	LOG_INFO("1- Channel 15 (2425MHz)\n");
	LOG_INFO("2- Channel 20 (2450MHz)\n");
	LOG_INFO("3- Channel 25 (2475MHz)\n");
	LOG_INFO("m- Show This Menu\n");
}

void DispPhyTestModeTxRawMenu(void)
{
	LOG_INFO("------------------------------------------------------\n");
	LOG_INFO("Physical Test Mode Tx Raw Carrier MENU:\n");
	LOG_INFO("r- Return to Previous Menu\n");
	LOG_INFO("1-  +7dBm @2425MHz\n");
	LOG_INFO("2-  +7dBm @2450MHz\n");
	LOG_INFO("3-  +7dBm @2475MHz\n");
	LOG_INFO("4-   0dBm @2425MHz\n");
	LOG_INFO("5-   0dBm @2450MHz\n");
	LOG_INFO("6-   0dBm @2475MHz\n");
	LOG_INFO("7- -20dBm @2425MHz\n");
	LOG_INFO("8- -20dBm @2450MHz\n");
	LOG_INFO("9- -20dBm @2475MHz\n");
	LOG_INFO("m- Show This Menu\n");
}

void DispPhyTestModeTxModulatedMenu(void)
{
	LOG_INFO("------------------------------------------------------\n");
	LOG_INFO("Physical Test Mode Tx Modulated Carrier MENU:\n");
	LOG_INFO("r- Return to Previous Menu\n");
	LOG_INFO("1-  +7dBm @2425MHz\n");
	LOG_INFO("2-  +7dBm @2450MHz\n");
	LOG_INFO("3-  +7dBm @2475MHz\n");
	LOG_INFO("4-   0dBm @2425MHz\n");
	LOG_INFO("5-   0dBm @2450MHz\n");
	LOG_INFO("6-   0dBm @2475MHz\n");
	LOG_INFO("7- -20dBm @2425MHz\n");
	LOG_INFO("8- -20dBm @2450MHz\n");
	LOG_INFO("9- -20dBm @2475MHz\n");
	LOG_INFO("m- Show This Menu\n");
}

void DispControlAttenuatorMenu(void)
{
	LOG_INFO("------------------------------------------------------\n");
	LOG_INFO("Control Attenuator MENU:\n");
	LOG_INFO("r- Return to Main Menu\n");
	LOG_INFO("Attenuator 1, Attenuator 2\n");
	LOG_INFO("m- Show This Menu\n");
}

void DispSendDataMenu(void)
{
	LOG_INFO("------------------------------------------------------\n");
	LOG_INFO("Send Data MENU:\n");
	LOG_INFO("r- Return to Main Menu\n");
	LOG_INFO("s- Send Data\n");
	LOG_INFO("1- Set Destination Index\n");
	LOG_INFO("2- Set Payload\n");
	LOG_INFO("3- Set Tx Options\n");
	LOG_INFO("4- Set Profile ID\n");
	LOG_INFO("l- List current configuration\n");
	LOG_INFO("m- Show This Menu\n");
}

void DispSendDataDestIndexMenu(void)
{
	LOG_INFO("------------------------------------------------------\n");
	LOG_INFO("Send Data - Set Destination Index\n");
	LOG_INFO("- \tEnter destination index as integer (max 255) \n");
}

void DispSendDataTxOptionsMenu(void)
{
	LOG_INFO("------------------------------------------------------\n");
	LOG_INFO("Send Data - Set Tx Options\n");
	LOG_INFO("  TODO: add atoh() equivalent function \n");
}

void DispSendDataProfileIDMenu(void)
{
	LOG_INFO("------------------------------------------------------\n");
	LOG_INFO("Choose Profile ID\n");
}

void DispSendDataCurrentCfg(appSendData_t appSendData_s)
{
	uint8 i;
	int strLen;
	char tmpStr[512];
	LOG_INFO("------------------------------------------------------\n");
	LOG_INFO("Current Send Data Configuration\n");
	LOG_INFO("- \t Destination Index: \t 0x%.2X \n", appSendData_s.dstIndex);
	LOG_INFO("- \t Profile: \t \t %s [0x%.2X] \n", rtiProfileId_list[appSendData_s.profileId], appSendData_s.profileId);
	LOG_INFO("- \t Tx Options: \t 0x%.2X \n", appSendData_s.txOptions);
	LOG_INFO("- \t Payload: \n");
	for (i = 1; i < appSendData_s.len; i++)
	{
		snprintf(tmpStr+strLen, sizeof(tmpStr)-strLen, ", 0x%.2X", appSendData_s.pData[i]);
		strLen += 4;
	}
	LOG_INFO("- \t \t 0x%.2X%s\n", appSendData_s.pData[0], tmpStr);
}

void DispSendDataPayloadMenu(void)
{
	LOG_INFO("------------------------------------------------------\n");
	LOG_INFO("Send Data - Set Payload\n");
	LOG_INFO("- \tEnter payload as a succession of bytes given  \n");
	LOG_INFO("- \twith radix 16. Example:\n");
	LOG_INFO("- \t \t 12 34 56 78 AB CD\n");
}

void DisplayPairingTable(rcnNwkPairingEntry_t *pEntry)
{
	int8 j;
	int strLen;
	char tmpStr[512];
	LOG_INFO("*************************************\n");
	LOG_INFO("* Pairing Index: \t 	0x%.2X\n", pEntry->pairingRef);
	LOG_INFO("* SRC NWK Address: \t 	0x%.4hX\n", pEntry->srcNwkAddress);
	LOG_INFO("* Logical Channel: \t 	0x%.2hX\n", pEntry->logicalChannel); //((pEntry->logicalChannel >> 8) & 0x00FF));
	strLen = 0;
	for (j = (SADDR_EXT_LEN - 2); j >= 0; j--)
	{
		snprintf(tmpStr+strLen, sizeof(tmpStr)-strLen, ":%.2X", (pEntry->ieeeAddress[j] & 0x00FF));
		strLen += 3;
	}
	LOG_INFO("* IEEE Address:  		%.2hX%s\n", (pEntry->ieeeAddress[SADDR_EXT_LEN - 1] & 0x00FF), tmpStr);
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
	LOG_INFO("* Profiles Discovered:  	0x%.4X\n",
			pEntry->profileDiscs[0]);
}

