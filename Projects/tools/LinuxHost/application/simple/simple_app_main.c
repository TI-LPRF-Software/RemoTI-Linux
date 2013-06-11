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
#include "timer.h"

#include "rti_lnx.h"
// Linux surrogate interface
#include "npi_ipc_client.h"

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

const char * const rtiProfileId_list[RTI_PROFILE_RTI + 1] =
{
		[0 ... RTI_PROFILE_RTI] = NULL,
		NAME_ELEMENT(RTI_PROFILE_GDP),
		NAME_ELEMENT(RTI_PROFILE_ZRC),
		NAME_ELEMENT(RTI_PROFILE_ZID),
		NAME_ELEMENT(RTI_PROFILE_Z3S),
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

enum 
{
	RTI_main_linux_threadId, RTI_App_threadId, RTI_main_threadId_tblSize
};

RTI_main_thread_s *RTI_main_thread_tbl;

sem_t event_mutex;

static void print_usage(const char *prog) {
	printf("Usage: %s [-DlHOLC3]\n", prog);
	puts(
			"  -D --device     device to use (default /dev/spidev4.0). For Socket use format IPaddress:port\n"
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
			{ NULL, 0, 0, 0 },
		};
		int c;

		c = getopt_long(argc, argv, "D:", lopts, NULL);

		if (c == -1)
			break;

		switch (c)
		{
		case 'D':
			device = optarg;
			break;
		default:
			print_usage(argv[0]);
			break;
		}
	}
}

int main(int argc, char **argv) 
{

	printf("Starting...\n");

	// Return value for main
	int ret = 0;

	consoleInput.latestCh = ' ';
	consoleInput.handle = RTI_MAIN_INPUT_RELEASED;

	RTI_main_thread_tbl = (RTI_main_thread_s *) malloc(
			sizeof(RTI_main_thread_s) * RTI_main_threadId_tblSize);

	// setup filedescriptor for stdin
	fds[0].fd = fileno(stdin);
	fds[0].events = POLLIN;

	parse_opts(argc, argv);

	// Initialize shared semaphore. Must happen before program begins execution
	sem_init(&event_mutex,0,1);

//	FILE *tmpFile;

	if ((ret = NPI_ClientInit(device)) == FALSE)
	{
		fprintf(stderr, "Failed to start RTI library module, device; %s\n", device);
		print_usage(argv[0]);
		return ret;
	}

	//Start RTI thread, management of RTI command in separate thread.
	if ((ret = appInit(0, RTI_App_threadId)) != 0) 
	{
		return ret;
	}

	printf("Starting timer thread\n");

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
				sem_post(&event_mutex);
			}
//			printf("Character read: \t%c, int: %d\n", consoleInput.latestCh,
//					consoleInput.latestCh);
//			printf("String read: \t%s\n", consoleInput.latestStr,
//					consoleInput.latestStr);
		}
//		printf("poll returned: %d; console handle: %d\n", pollRet, consoleInput.handle);
	}

	// Destroy semaphores
	sem_destroy(&event_mutex);

	NPI_ClientClose();

	return ret;
}

/**************************************************************************************************
 *
 * @fn      RTI_main_start_timerEx
 *
 * @brief   Modeled after OSAL_start_timerEx.
 * 			To stop timer call with same threadId, and event, and
 * 			set timeout to 0.
 *
 * @param   threadId - Id of the thread to call at the event.
 * @param	event - event bitmask
 * @param	timeout - number of milliseconds to count down
 *
 * @return  void
 */
uint8 RTI_main_start_timerEx(uint8 threadId, uint16 event, uint16 timeout) 
{
	uint8 i;
	for (i = 0; i < 16; i++) 
	{
		if (event & BV(i))
			break;
	}
	RTI_main_thread_tbl[threadId].timeoutValue[i] = timeout * 1000;
	// Use a 0 value of timeout to disable timer
	if (timeout)
	{
		RTI_main_thread_tbl[threadId].timerEnabled |= event;
	}
	else
	{
		RTI_main_thread_tbl[threadId].timerEnabled &= ~event;
	}
#ifdef TIMER_DEBUG
	printf("Timer started\n");
#endif //TIMER_DEBUG
	return 1;
}

uint8 RTI_main_set_event(uint8 threadId, uint16 event) 
{
#ifdef TIMER_DEBUG
	printf("Setting event 0x%.2X\n", event);
#endif //TIMER_DEBUG
	RTI_main_thread_tbl[threadId].eventFlag |= event;
	return TRUE;
}

uint8 RTI_main_clear_event(uint8 threadId, uint16 event) 
{
	RTI_main_thread_tbl[threadId].eventFlag &= ~event;
	return TRUE;
}

uint16 RTI_main_get_event(uint8 threadId) 
{
	return RTI_main_thread_tbl[threadId].eventFlag;
}

void DispMenuInit(void) 
{
	printf("------------------------------------------------------\n");
	printf("Init MENU:\n");
	printf("1- Toggle Target / Controller\n");
	printf("2- Set Node Capabilities\n");
	printf("3- Supported Profiles\n");
	printf("4- Supported Devices \n");
	printf("5- Supported Target Types \n");
	printf("6- Vendor ID\n");
	printf("7- Vendor String\n");
	printf("8- Toggle User String\n");
	printf("9- User String\n");
	printf("i- Initialize without configuration. (Restore from NV).\n");
	printf("g- Get current configuration from RNP.\n");
	printf("l- Show configuration. Note! Not Necessarily the One Written To RNP\n");
	printf("s- Apply Configuration and Move On To Application\n");
	printf("r- Back To Application, Do Not Apply Changes\n");
	printf("\nc- Start As Controller (default settings)\n");
	printf("\nt- Start As Target (default settings)\n");
}

void DispCFGCurrentCfg(appDevInfo_t appCfg)
{
	int16 i;
	char* tmpStr[4];
	printf("------------------------------------------------------\n");
	printf("- Current Configuration:\n");
	printf("- \tNode Capabilities: \t \t \t 0x%.2X\n", appCfg.nodeCapabilities);
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
		printf("- \t \t%s\n", tmpStr[i]);


	printf("- \tApplication Capabilities: \t \t 0x%.2X\n", appCfg.appCapabilities);
	printf("- \t \t Number of supported profiles:\t 0x%.2X\n",
			RCN_APP_CAPA_GET_NUM_PROFILES(appCfg.appCapabilities));
	printf("- \t \t Number of supported devices:\t 0x%.2X\n",
			RCN_APP_CAPA_GET_NUM_DEV_TYPES(appCfg.appCapabilities));
	if (RCN_APP_CAPA_GET_USER_STRING(appCfg.appCapabilities))
		printf("- \t \t User String supported\n");
	printf("- \tSupported Profiles:\n");
	for (i = 0; i < RCN_APP_CAPA_GET_NUM_PROFILES(appCfg.appCapabilities); i++)
	{
		printf("- \t \t \t \t %s\n", rtiProfileId_list[appCfg.profileIdList[i]]);
	}
	if (appCfg.vendorId <= RTI_VENDOR_TEXAS_INSTRUMENTS)
	{
		printf("- \tVendor ID: \t \t \t \t 0x%.2X (%s)\n", appCfg.vendorId, rtiVendorId_list[appCfg.vendorId]);
	}
	else
	{
		printf("- \tVendor ID: \t \t \t \t 0x%.2X (Unknown)\n", appCfg.vendorId);
	}

	printf("- \tDevice Types:\n");
	for (i = 0; i < RTI_MAX_NUM_DEV_TYPES; i++)
	{
		if (appCfg.devTypeList[i] <= RTI_DEVICE_TARGET_TYPE_END)
		{
			printf("- \t \t \t \t 0x%.2X (%s)\n", appCfg.devTypeList[i], rtiDevType_list[appCfg.devTypeList[i]]);
		}
		else
		{
			printf("- \t \t \t \t 0x%.2X (Unknown)\n", appCfg.devTypeList[i]);
		}
	}

	printf("- \tTarget Device Types:\n");
	for (i = 0; i < RTI_MAX_NUM_SUPPORTED_TGT_TYPES; i++)
	{
		if (appCfg.tgtTypeList[i] <= RTI_DEVICE_TARGET_TYPE_END)
		{
			printf("- \t \t \t \t 0x%.2X (%s)\n", appCfg.tgtTypeList[i], rtiDevType_list[appCfg.tgtTypeList[i]]);
		}
		else
		{
			printf("- \t \t \t \t 0x%.2X (Unknown)\n", appCfg.tgtTypeList[i]);
		}
	}
	printf("------------------------------------------------------\n");
}

void DispCFGNodeCapMenu(void)
{
	printf("------------------------------------------------------\n");
	printf("- Configure Node Capabilites\n");
	printf("- \tEnter as string of bytes separated by any of \n");
	printf("- \tthese delimiters: ' ' , ; : - |\n");
	printf("- Target/Controller, Mains/Battery, Security On/Off, Channel Normalization On/Off\n");
	printf("- Example:\n");
	printf("- \t 1, 1, 1, 0 \t yields:\n");
	printf("- Target , Mains Powered, Security Enabled, Channel Normalization Off\n");
}

void DispCFGProfileIDMenu(void)
{
	printf("------------------------------------------------------\n");
	printf("- Configure Supported Profile IDs (Max %d)\n", RCN_MAX_NUM_PROFILE_IDS);
	printf("- \tEnter as string of bytes separated by any of \n");
	printf("- \tthese delimiters: ' ' , ; : - |\n");
	printf("- Available Profiles:\n");
	DispNumberedStringList((char**)rtiProfileId_list, RTI_PROFILE_RTI + 1);
}

void DispCFGVendorIDMenu(void)
{
	printf("------------------------------------------------------\n");
	printf("- Configure Vendor ID\n");
	printf("- \tEnter as 16 bit hexadecimal value \n");
}

void DispCFGdevTypesMenu(void)
{
	printf("------------------------------------------------------\n");
	printf("- Configure Supported Device Types (Max %d)\n", RTI_MAX_NUM_DEV_TYPES);
	printf("- \tEnter as string of bytes separated by any of \n");
	printf("- \tthese delimiters: ' ' , ; : - |\n");
}

void DispCFGtgtTypesMenu(void)
{
	printf("------------------------------------------------------\n");
	printf("- Configure Supported Target Types (Max %d)\n", RTI_MAX_NUM_SUPPORTED_TGT_TYPES);
	printf("- \tEnter as string of bytes separated by any of \n");
	printf("- \tthese delimiters: ' ' , ; : - |\n");
}

void DispNumberedStringList(char** strList, uint16 len)
{
	int i;
	printf("------------------------------------------------------\n");
	for (i = 0; i < len; i++)
	{
		if (strList[i] != NULL)
			printf("- %d \t%s\n", i, strList[i]);
	}
	printf("------------------------------------------------------\n");
}

void DispStringList(char** strList, uint16 len)
{
	int i;
	printf("------------------------------------------------------\n");
	for (i = 0; i < len; i++)
	{
		if (strList[i] != NULL)
			printf("-\t%s\n", strList[i]);
	}
	printf("------------------------------------------------------\n");
}

void DispMenuReady(void)
{
	printf("------------------------------------------------------\n");
	printf("Main MENU:\n");
	printf("q- exit\n");
	printf("0- Config \n");
	printf("1- Pairing\n");
	printf("2- Unpairing\n");
	printf("7- Send Data\n");
	printf("8- Clear Pairing Table\n");
	printf("9- Display Pairing Table\n");
	printf("t- Toggle __DEBUG_TIME_ACTIVE on Server\n");
	printf("y- Toggle __BIG_DEBUG on Server\n");
	printf("c- Toggle simple data display\n");
	printf("a- Check States\n");
	printf("g- Get MAC Channel\n");
	printf("r- Reset RNP\n");
	printf("m- Show This Menu\n");
}

void DispSendDataMenu(void)
{
	printf("------------------------------------------------------\n");
	printf("Send Data MENU:\n");
	printf("r- Return to Main Menu\n");
	printf("s- Send Data\n");
	printf("1- Set Destination Index\n");
	printf("2- Set Payload\n");
	printf("3- Set Tx Options\n");
	printf("4- Set Profile ID\n");
	printf("l- List current configuration\n");
	printf("m- Show This Menu\n");
}

void DispSendDataDestIndexMenu(void)
{
	printf("------------------------------------------------------\n");
	printf("Send Data - Set Destination Index\n");
	printf("- \tEnter destination index as integer (max 255) \n");
}

void DispSendDataTxOptionsMenu(void)
{
	printf("------------------------------------------------------\n");
	printf("Send Data - Set Tx Options\n");
	printf("  TODO: add atoh() equivalent function \n");
}

void DispSendDataProfileIDMenu(void)
{
	printf("------------------------------------------------------\n");
	printf("Choose Profile ID\n");
}

void DispSendDataCurrentCfg(appSendData_t appSendData_s)
{
	uint8 i;
	printf("------------------------------------------------------\n");
	printf("Current Send Data Configuration\n");
	printf("- \t Destination Index: \t 0x%.2X \n", appSendData_s.dstIndex);
	printf("- \t Profile: \t \t %s [0x%.2X] \n", rtiProfileId_list[appSendData_s.profileId], appSendData_s.profileId);
	printf("- \t Tx Options: \t 0x%.2X \n", appSendData_s.txOptions);
	printf("- \t Payload: \n");
	printf("- \t \t 0x%.2X", appSendData_s.pData[0]);
	for (i = 1; i < appSendData_s.len; i++)
	{
		printf(", 0x%.2X", appSendData_s.pData[i]);
	}
	printf("\n");
}

void DispSendDataPayloadMenu(void)
{
	printf("------------------------------------------------------\n");
	printf("Send Data - Set Payload\n");
	printf("- \tEnter payload as a succession of bytes given  \n");
	printf("- \twith radix 16. Example:\n");
	printf("- \t \t 12 34 56 78 AB CD\n");
}

void DisplayPairingTable(rcnNwkPairingEntry_t *pEntry)
{
	int8 j;
	printf("*************************************\n");
	printf("* Pairing Index: \t 	0x%.2X\n", pEntry->pairingRef);
	printf("* SRC NWK Address: \t 	0x%.4hX\n", pEntry->srcNwkAddress);
	printf("* Logical Channel: \t 	0x%.2hX\n", pEntry->logicalChannel); //((pEntry->logicalChannel >> 8) & 0x00FF));
	printf("* IEEE Address:  		%.2hX", (pEntry->ieeeAddress[SADDR_EXT_LEN - 1] & 0x00FF));
	for (j = (SADDR_EXT_LEN - 2); j >= 0; j--)
	{
		printf(":%.2X", (pEntry->ieeeAddress[j] & 0x00FF));
	}
	printf("\n");
	printf("* PAN Id: \t \t \t0x%.4X\n", pEntry->panId);
	printf("* NWK Address: \t \t 	0x%.4hX\n", pEntry->nwkAddress);
	printf("* Rec Capabilities: \t 	0x%.2hX\n",
			(pEntry->recCapabilities & 0x00FF));
	printf("* Security Key Valid: \t \t0x%.2hX\n",
			(pEntry->securityKeyValid & 0x00FF));
	printf("* Vendor Identifier: \t \t0x%.4hX\n",
			pEntry->vendorIdentifier);
	printf("* Device Type List: \t 	[0x%.2hX, 0x%.2hX, 0x%.2hX]\n",
			(pEntry->devTypeList[0] & 0x00FF),
			(pEntry->devTypeList[1] & 0x00FF),
			(pEntry->devTypeList[2] & 0x00FF));
	printf("* Received Frame Counter: \t0x%.8X (%u)\n",
			pEntry->recFrameCounter, pEntry->recFrameCounter);
	printf("* Profiles Discovered: \t 	0x%.4X\n",
			pEntry->profileDiscs[0]);
}

