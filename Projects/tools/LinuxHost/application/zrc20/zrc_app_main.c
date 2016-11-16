/**************************************************************************************************
 Filename:       zrc_app_main.cpp

 Description:    Linux serial boot loader application (target node)

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


// Linux surrogate interface
#include "npi_ipc_client.h"
#include "npi_lnx_error.h"


#include "zrc_app_main.h"
#include "timer.h"
#include "tiLogging.h"

#include "hal_rpc.h"

// macros
#define PAUSE() { fflush(stdout); while (getchar()!='\n'); }

#define NAME_ELEMENT(element) [element] = #element

struct pollfd fds[1];
const char *device = "";
const char *debugOption = "";
char *imagePath = "";

/* Global variable definitions. Declared as externs in common_app.h */
sem_t eventSem;

enum 
{
    ZRC_main_linux_threadId,
    ZRC_App_threadId,
    ZRC_main_threadId_tblSize
};

static void print_usage(const char *prog) {
    fprintf(stderr, "Usage: %s [-DlHOLC3]\n", prog);
    fprintf(stderr,
            "  -D --device      device to use (default /dev/spidev4.0). For Socket use format IPaddress:port\n"
            "  -d --debugOption debugAll: both time and big, debugTime: only timestamps, debugBig: verbose debug\n"
            "  -i --image        path to image that can be uploaded to remote\n"
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
            { "image", 1, 0, 'i' },
            { NULL, 0, 0, 0 },
        };
        int c;

        c = getopt_long(argc, argv, "D:d:i:", lopts, NULL);

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
        case 'i':
            imagePath = optarg;
            break;
        default:
            print_usage(argv[0]);
            break;
        }
    }
}

int main(int argc, char **argv)
{
	// IMPORTANT: This call to tiLogging_Init() MUST BE DONE FIRST.
	// One thing it does is changes the buffering of stdout to line-based,
	// and that works ONLY if the stream has not been used yet, so we
	// MUST NOT printf/log anything before calling it...
	tiLogging_Init(NULL);

	LOG_INFO("Starting...\n");

    // Return value for main
    int ret = 0;
//    static int first_start = 1;

    consoleInput.latestCh = ' ';
    consoleInput.handle = MAIN_INPUT_RELEASED;

    // setup filedescriptor for stdin
    fds[0].fd = fileno(stdin);
    fds[0].events = POLLIN;

    parse_opts(argc, argv);

    // Initialize shared semaphore. Must happen before program begins execution
    sem_init(&eventSem,0,1);

    // Initialize Network Processor Interface.
    if ((ret = NPI_ClientInit(device)) != NPI_LNX_SUCCESS)
    {
        LOG_FATAL("Failed to start NPI library module, device; %s\n", device);
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

    //Start ZRC thread, management of RTI command in separate thread.
    if ((ret = ZRC_AppInit(mode, ZRC_App_threadId)) != 0)
    {
        return ret;
    }

    LOG_INFO("Starting timer thread\n");

    //Start MAC timer thread, management of timer in separate thread.
    // No Timer for Mac for now...
    if ((ret = timer_init(ZRC_main_threadId_tblSize)) != 0)
    {
        LOG_FATAL("Failed to start timer thread. Exiting...");
        return ret;
    }

    //first Menu will be display at the end of the RNP initialization.

    int pollRet;

    //MANAGE MAIN DISPLAY HERE
    // Equivalent to OSAL Main Loop
    while (1) 
    {
        //Wait for Display Mutex release before displaying anything.
        //pthread_mutex_lock(&appDisplayMutex);
        //pthread_mutex_unlock(&appDisplayMutex);

        //Check Display buffer, if not empty display one element
//        if (first_start)
//        {
//            consoleInput.handle = MAIN_INPUT_READY;
//            sem_post(&eventSem);
//            first_start=0;
//        }

        // Check for input characters, timeout after 500ms
        pollRet = poll(fds, 1, 500);
        if ((pollRet == 1) && (consoleInput.handle == MAIN_INPUT_RELEASED))
        {
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
                consoleInput.handle = MAIN_INPUT_READY;
                // Release resources waiting for this event
                LOG_DEBUG("POSTING EVENT\n");
                if (sem_post(&eventSem) < 0)
                {
                    LOG_ERROR("[MAIN] Failed to post semaphore %p\n", &eventSem);
                }
            }
//            LOG_DEBUG("Character read: \t%c, int: %d\n", consoleInput.latestCh,
//                    consoleInput.latestCh);
//            LOG_DEBUG("String read: \t%s\n", consoleInput.latestStr,
//                    consoleInput.latestStr);
        }
//        LOG_DEBUG("poll returned: %d; console handle: %d\n", pollRet, consoleInput.handle);
    }

    // Destroy semaphores
    if (sem_destroy(&eventSem) < 0)
    {
        perror("Failed to destroy mutex");
    }
    // Remember to terminate all threads.
    // They are automatically terminated by exiting the application

    // Close Network Processor Connection
    LOG_INFO("[APP]Close Socket Connection\n");
    NPI_ClientClose();

    return ret;
}
