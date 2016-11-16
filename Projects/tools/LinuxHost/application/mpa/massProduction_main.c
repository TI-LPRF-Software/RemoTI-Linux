/**************************************************************************************************
 Filename:       massProduction_main.c

 Description:    Linux SimpleConsole application (target node)

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
#include <stdlib.h>
#include <string.h>
#include <getopt.h>
#include <poll.h>

#include "massProduction_main.h"
#include "massProduction_app.h"
#include "timer.h"
#include "configParser.h"
#include "tiLogging.h"

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

extern int appThreadTerminate;

// Pairing reference
uint8 destIdx;

appBaseSetting_s baseSettings;

struct pollfd fds[1];

char device[512];
int connectedPort;

/* Global variable definitions. Declared as externs in common_app.h */
sem_t eventSem;

static int configFilePresent = FALSE;
char *configFile;

enum
{
	FPA_main_threadId, MPA_App_threadId, MPA_App_threadId_tblSize
};

static void print_usage(const char *prog) {
	fprintf(stderr, "Usage: %s [-DlHOLC3]\n", prog);
	fprintf(stderr,
			"  -c --configFilePath \tpath to configuration file. Should contain mass production configuration\n"
		);
	exit(1);
}

static void parse_opts(int argc, char *argv[])
{
	while (1)
	{
		static const struct option lopts[] =
		{
			{ "configFilePath", 1, 0, 'c' },
			{ NULL, 0, 0, 0 },
		};
		int c;

		c = getopt_long(argc, argv, "c:", lopts, NULL);

		if (c == -1)
			break;

		switch (c)
		{
		case 'c':
			configFilePresent = TRUE;
			configFile = optarg;
			break;
		default:
			print_usage(argv[0]);
			break;
		}

	}
}

int main(int argc, char **argv)
{
	baseSettings.ip_addr = (char *) malloc(128);
	baseSettings.port = (char *) malloc(32);

	// IMPORTANT: This call to tiLogging_Init() MUST BE DONE FIRST.
	// One thing it does is changes the buffering of stdout to line-based,
	// and that works ONLY if the stream has not been used yet, so we
	// MUST NOT printf/log anything before calling it...
	tiLogging_Init(NULL);

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

	// Initialize ConfigParser
	if (configFilePresent)
	{
		LOG_INFO("Initializing the Config Parser from: %s\n", configFile);
		ConfigParserInit(configFile, NULL);
		LOG_INFO("Initializing base settings\n");
		ConfigParserGetBaseSettings(&baseSettings);
		LOG_INFO("Got base settings. IP Address: %s, Port: %s\n", baseSettings.ip_addr, baseSettings.port);
		strcpy(device, baseSettings.ip_addr);
		strcat(device, ":");
		strcat(device, baseSettings.port);
		LOG_INFO("Device: %s\n", device);
	}
	else
	{
		LOG_WARN("You must define a valid configuration file\n");
		print_usage(argv[0]);
	}

//	FILE *tmpFile;

	if ((ret = NPI_ClientInit(device)) != NPI_LNX_SUCCESS)
	{
		LOG_FATAL("Failed to start RTI library module, device; %s\n", device);
		print_usage(argv[0]);
		return ret;
	}

	//Start RTI thread, management of RTI command in separate thread.
	if ((ret = appInit(0, MPA_App_threadId)) != 0)
	{
		return ret;
	}

	LOG_INFO("Starting timer thread\n");

	//Start RTI timer thread, management of timer in separate thread.
	if ((ret = timer_init(MPA_App_threadId_tblSize)) != 0)
	{
		LOG_FATAL("Failed to start timer thread. Exiting...");
		return ret;
	}

	//first Menu will be display at the end of the RNP initialization.

	int pollRet;

	//MANAGE DISPLAY HERE, WAIT FOR APPLICATION THREAD TO EXIT
	while (!appThreadTerminate)
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

	if (baseSettings.ip_addr != NULL)
	{
		free(baseSettings.ip_addr);
	}
	if (baseSettings.port != NULL)
	{
		free(baseSettings.port);
	}


	NPI_ClientClose();

	return ret;
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
	LOG_INFO("t- Toggle __DEBUG_TIME_ACTIVE on Server\n");
	LOG_INFO("y- Toggle __BIG_DEBUG on Server\n");
	LOG_INFO("r- Reset RNP\n");
	LOG_INFO("m- Show This Menu\n");
}
