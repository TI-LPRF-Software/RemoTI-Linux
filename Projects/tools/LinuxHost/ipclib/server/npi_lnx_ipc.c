/**************************************************************************************************
 Filename:       npi_lnx_ipc.c
 Revised:        $Date: 2012-03-21 17:37:33 -0700 (Wed, 21 Mar 2012) $
 Revision:       $Revision: 246 $

 Description:    This file contains Linux platform specific NPI socket server
 implementation

 Copyright (C) {2016} Texas Instruments Incorporated - http://www.ti.com/

 Beej's Guide to Unix IPC was used in the development of this software:
 http://beej.us/guide/bgipc/output/html/multipage/intro.html#audience
 A small portion of the code from the associated code was also used. This
 code is Public Domain.


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

/**************************************************************************************************
 *                                           Includes
 **************************************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <ctype.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

// For stress testing data dump
#include <fcntl.h>
#include <sys/stat.h>
#include <time.h>

#ifndef NPI_UNIX
#include <ifaddrs.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#endif

#include <sys/time.h>

#include "OEM_NpiStartupHook.h"

/* NPI includes */
#include "npi_lnx.h"
#include "npi_lnx_error.h"
#include "npi_lnx_ipc_rpc.h"
#include "tiLogging.h"
#include "npi_lnx_serial_configuration.h"

#if (defined NPI_SPI) && (NPI_SPI == TRUE)
#include "npi_lnx_spi.h"
#include "hal_spi.h"
#endif

#if (defined NPI_I2C) && (NPI_I2C == TRUE)
#include "npi_lnx_i2c.h"
#include "hal_i2c.h"
#endif

#if (defined NPI_UART) && (NPI_UART == TRUE)
#include "npi_lnx_uart.h"
#endif

// The following is only necessary because we always read out GPIO configuration
#include "hal_gpio.h"

#if (!defined NPI_SPI) || (NPI_SPI == FALSE)
#if (!defined NPI_I2C) || (NPI_I2C == FALSE)
#if (!defined NPI_UART) || (NPI_UART == FALSE)
#error "neither NPI_I2C, NPI_SPI, NPI_UART defined to TRUE, at least one mandatory. verify your makefile"
#endif
#endif
#endif

#include "hal_dbg_ifc.h"
#include "hal_dbg_ifc_rpc.h"


/**************************************************************************************************
 *                                        Externals
 **************************************************************************************************/

/**************************************************************************************************
 *                                        Defines
 **************************************************************************************************/
#define NPI_SERVER_CONNECTION_QUEUE_SIZE        20

#define MAX(a,b)								((a > b) ? a : b)

/**************************************************************************************************
 *                                           Constant
 **************************************************************************************************/
enum {
	enumSRDY,
	enumMRDY,
	enumRESET,
	enumDD,
	enumDC
};

const pNPI_OpenDeviceFn NPI_OpenDeviceFnArr[] =
{
#if (defined NPI_UART) && (NPI_UART == TRUE)
		NPI_UART_OpenDevice,
#else
		NULL,
#endif
#if (defined NPI_SPI) && (NPI_SPI == TRUE)
		NPI_SPI_OpenDevice,
#else
		NULL,
#endif
#if (defined NPI_I2C) && (NPI_I2C == TRUE)
		NPI_I2C_OpenDevice
#else
		NULL,
#endif
};

const pNPI_CloseDeviceFn NPI_CloseDeviceFnArr[] =
{
#if (defined NPI_UART) && (NPI_UART == TRUE)
		NPI_UART_CloseDevice,
#else
		NULL,
#endif
#if (defined NPI_SPI) && (NPI_SPI == TRUE)
		NPI_SPI_CloseDevice,
#else
		NULL,
#endif
#if (defined NPI_I2C) && (NPI_I2C == TRUE)
		NPI_I2C_CloseDevice
#else
		NULL,
#endif
};
const pNPI_SendAsynchDataFn NPI_SendAsynchDataFnArr[] =
{
#if (defined NPI_UART) && (NPI_UART == TRUE)
		NPI_UART_SendAsynchData,
#else
		NULL,
#endif
#if (defined NPI_SPI) && (NPI_SPI == TRUE)
		NPI_SPI_SendAsynchData,
#else
		NULL,
#endif
#if (defined NPI_I2C) && (NPI_I2C == TRUE)
		NPI_I2C_SendAsynchData,
#else
		NULL,
#endif
#if (defined NPI_UART_USB) && (NPI_UART_USB == TRUE)
		NPI_UART_SendAsynchData,
#else
		NULL,
#endif
};
const pNPI_SendSynchDataFn NPI_SendSynchDataFnArr[] =
{
#if (defined NPI_UART) && (NPI_UART == TRUE)
		NPI_UART_SendSynchData,
#else
		NULL,
#endif
#if (defined NPI_SPI) && (NPI_SPI == TRUE)
		NPI_SPI_SendSynchData,
#else
		NULL,
#endif
#if (defined NPI_I2C) && (NPI_I2C == TRUE)
		NPI_I2C_SendSynchData,
#else
		NULL,
#endif
#if (defined NPI_UART_USB) && (NPI_UART_USB == TRUE)
		NPI_UART_SendSynchData,
#else
		NULL,
#endif
};

const pNPI_ResetSlaveFn NPI_ResetSlaveFnArr[] =
{
		NULL,
#if (defined NPI_SPI) && (NPI_SPI == TRUE)
		NPI_SPI_ResetSlave,
#else
		NULL,
#endif
#if (defined NPI_I2C) && (NPI_I2C == TRUE)
		NPI_I2C_ResetSlave,
#else
		NULL,
#endif
};

const pNPI_SynchSlaveFn NPI_SynchSlaveFnArr[] =
{
		NULL,
#if (defined NPI_SPI) && (NPI_SPI == TRUE)
		NPI_SPI_SynchSlave,
#else
		NULL,
#endif
#if (defined NPI_I2C) && (NPI_I2C == TRUE)
		NULL,
#else
		NULL,
#endif
};

/**************************************************************************************************
 *                                        Type definitions
 **************************************************************************************************/

/**************************************************************************************************
 *                                        Global Variables
 **************************************************************************************************/

int npi_ipc_errno;

/**************************************************************************************************
 *                                        Local Variables
 **************************************************************************************************/

// Socket variables
static struct sockaddr_storage their_addr;
static struct addrinfo *servinfo;

// Socket handles
static int  sNPIlisten;

// Socket connection file descriptors
static fd_set activeConnectionsFDs;
static int fdmax;
static struct
{
	int list[NPI_SERVER_CONNECTION_QUEUE_SIZE];
	int size;
} activeConnections;

// Variables for Configuration
npiSerialCfg_t serialCfg;

#if (defined __DEBUG_TIME__) || (__STRESS_TEST__)
static struct timespec gStartTime = {0,0};
#endif //__DEBUG_TIME__

#ifdef __STRESS_TEST__
#define INDEX_RECV 0
#define INDEX_SEND 1
#define TIMING_STATS_SIZE                                                     500
#define TIMING_STATS_MS_DIV                                                    10
static unsigned int timingStats[2][(TIMING_STATS_SIZE / TIMING_STATS_MS_DIV) + 1];
FILE *fpStressTestData;
#define STRESS_TEST_SUPPORTED_NUM_PAIRING_ENTRIES                              10
static struct
{
	uint32 currentSeqNumber[STRESS_TEST_SUPPORTED_NUM_PAIRING_ENTRIES];
	struct{
		uint32 errorInSeqNum;
		uint32 seqNumIdentical;
	} recErrors;
} ST_Parameters_t[2] =
{
		{
				{0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
				{0, 0}
		},
		{
				{0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
				{0, 0}
		}
};
#endif //__STRESS_TEST__

/**************************************************************************************************
 *                                     Local Function Prototypes
 **************************************************************************************************/
static void NPI_LNX_IPC_Exit(int ret, uint8 freeSerial);

static int NPI_LNX_IPC_SendData(npiMsgData_t const *sendBuf, int connection);
static int NPI_LNX_IPC_ConnectionHandle(int connection, npiMsgData_t *recvBuf);

static int removeFromActiveList(int c);
static int addToActiveList(int c);

static int setupSocket(npiSerialCfg_t *serialCfg);
static int configureDebugInterface(void);
static void writeToNpiLnxLog(const char* str);

static int npi_ServerCmdHandle(npiMsgData_t *npi_ipc_buf);

/**************************************************************************************************
 * @fn          halDelay
 *
 * @brief       Delay for milliseconds.
 *              Do not invoke with zero.
 *              Do not invoke with greater than 500 msecs.
 *              Invoking with very high frequency and/or with long delays will start to
 *              significantly impact the real time performance of TimerA tasks because this will
 *              invisibly overrun the period when the TimerA count remaining, when this function
 *              is invoked, is less than the delay requested.
 *
 * input parameters
 *
 * @param       msecs - Milliseconds to delay in low power mode.
 * @param       sleep - Enforces blocking delay in low power mode if set.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
//void halDelay(uint8 msecs, uint8 sleep)
//{
//	if (sleep)
//	{
//	    usleep(msecs * 1000);
//	}
//}

static void writeToNpiLnxLog(const char* str)
{
	static const int kStrSize=255;
	int npiLnxLogFd;
	size_t ix = 0;
	char *fullStr = (char *)malloc(kStrSize);
	char *inStr = (char *)malloc(kStrSize);

	time_t timeNow;
	struct tm * timeNowinfo;

	time ( &timeNow );
	timeNowinfo = localtime ( &timeNow );

	snprintf(fullStr, kStrSize, "[%s", asctime(timeNowinfo));
	snprintf(inStr, kStrSize, "%s", str);
	// Remove \n characters
	fullStr[strlen(fullStr) - 2] = 0;
	for (ix = strlen(str) - 1; ix > MAX(strlen(str), 4); ix--)
	{
		if (inStr[ix] == '\n')
			inStr[ix] = 0;
	}
	// Concatenate and add global error code
	ix = strlen(fullStr);
	snprintf(fullStr+ix, kStrSize-ix, "] %s. Error: %.8X\n", inStr, npi_ipc_errno);

	// Write error message to /dev/npiLnxLog
	if (*serialCfg.logPath)
		npiLnxLogFd = open(serialCfg.logPath,  O_WRONLY | O_APPEND | O_CREAT, S_IRWXU);
	else
		npiLnxLogFd = STDERR_FILENO; // Empty string for log path means use stderr

	if (npiLnxLogFd > 0)
	{
		write(npiLnxLogFd, fullStr, strlen(fullStr));
//		LOG_DEBUG("Wrote:\n%s\n to npiLnxLog.log\n", fullStr, errno);
	}
	else
	{
		LOG_ERROR("Could not write \n%s\n to npiLnxLog. Error: %.8X\n", str, errno);
		perror("open");
	}

	if (*serialCfg.logPath)
		close(npiLnxLogFd);

	free(fullStr);
	free(inStr);
}

static void print_usage_and_exit(const char *prog)
{
	fflush(stdout);// Get anything logged to stdout ahead of us flushed.
	fprintf(stderr, "\nUsage: %s [config_file_name] [debug]\n", prog);
	fprintf(stderr, "  config_file_name: the path/name of the config file to use.\n");
	fprintf(stderr, "                    If NOT specified, the default is RemoTI_RNP.cfg and it will first\n");
	fprintf(stderr, "                    look in the current directory, and if not found there, will look in\n");
	fprintf(stderr, "                    the directory of wherever this binary is located.\n");
	fprintf(stderr, "  debug: set debug options. 'debugAll' for both BIG and TIME, 'debugTime' for just TIME or 'debugBig' for just BIG \n");
	exit(1);
}

/**************************************************************************************************
 *
 * @fn          NPI Linux IPC Socket Server
 *
 * @brief       This is the main function
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 *
 **************************************************************************************************/
int main(int argc, char *argv[])
{
	static npiMsgData_t   npiIpcRecvBuf;
	int                   ret = NPI_LNX_SUCCESS;

	// IMPORTANT: This call to tiLogging_Init() MUST BE DONE FIRST.
	// One thing it does is changes the buffering of stdout to line-based,
	// and that works ONLY if the stream has not been used yet, so we
	// MUST NOT printf/log anything before calling it...
	tiLogging_Init(NULL);

#ifdef __BUILT_TIME_STRING__
	LOG_ALWAYS("%s: Built %s\n\n", argv[0], __BUILT_TIME_STRING__);
#endif


	LOG_ALWAYS("*********************************************************************\n");
	LOG_ALWAYS("*\tTexas Instruments\n");
	LOG_ALWAYS("*\tNPI Server version %d.%d.%d\n",  NPI_LNX_MAJOR_VERSION, NPI_LNX_MINOR_VERSION, NPI_LNX_REVISION);
	LOG_ALWAYS("*\tCopyright 2016\n");
	LOG_ALWAYS("*********************************************************************\n\n");

	if (!OEM_NpiStartupHook(argc, argv))
	{
		npi_ipc_errno = -1;
		LOG_FATAL("%s(): OEM_NpiStartupHook() failed.\n", __FUNCTION__);
		NPI_LNX_IPC_Exit(NPI_LNX_FAILURE, TRUE);
	}

#ifdef NPI_UNIX
	LOG_DEBUG("NPI_UNIX\n");
#else
	LOG_DEBUG("NOT NPI_UNIX\n");
#endif

	/**********************************************************************
	 * Initialize timed print module
	 **********************************************************************/
   time_printf_start();

	/**********************************************************************
	 * First step is to Configure the serial interface
	 **********************************************************************/

	// Variables for Configuration. Some are declared global to be used in unified graceful
	// exit function.
	char const * configFilePath = "RemoTI_RNP.cfg";

	if (argc > 3)
	{
		LOG_FATAL("Too many arguments\n");
		print_usage_and_exit(argv[0]);
	}
	else if (argc > 2)
	{
		configFilePath = argv[1];
		if (strcmp(argv[2], "debugAll") == 0)
		{
			__BIG_DEBUG_ACTIVE = TRUE;
#ifdef __DEBUG_TIME__
			__DEBUG_TIME_ACTIVE = TRUE;
#endif
		}
		else if (strcmp(argv[2], "debugBig") == 0)
		{
			__BIG_DEBUG_ACTIVE = TRUE;
		}
#ifdef __DEBUG_TIME__
		else if (strcmp(argv[2], "debugTime") == 0)
		{
			__DEBUG_TIME_ACTIVE = TRUE;
		}
#endif
		else
		{
			print_usage_and_exit(argv[0]);
		}
	}
	else if (argc > 1)
	{
		configFilePath = argv[1];
	}
	else
	{
		// No config file path specified.  Try defaulting it...

		FILE *fpTemp = fopen(configFilePath, "r");
		if (fpTemp)
		{
			// Config file exists in working directory.  Use it.
			fclose(fpTemp);
		}
		else
		{
			// Config file does not exist in working directory.
			// Presume it's located in the same directory as
			// the binary, so point there.
			char *lastSlash = strrchr(argv[0], '/');
			if (lastSlash)
			{
				char     *pathToUse;
				size_t   pathLen;

				++lastSlash;
				pathLen = (size_t)(lastSlash-argv[0]) + strlen(configFilePath) + 1; /* +1 is for null term */

				pathToUse = malloc(pathLen);
				snprintf(pathToUse, pathLen, "%.*s%s", (int)(lastSlash-argv[0]), argv[0], configFilePath);
				configFilePath = pathToUse;
			}
		}
	}

	LOG_INFO("configFilePath is \"%s\"\n", configFilePath);

	if (NPI_LNX_SUCCESS == getSerialConfiguration(configFilePath, &serialCfg))
	{
		LOG_ALWAYS("Successfully read configuration parameters\n");
	}
	else
	{
		NPI_LNX_IPC_Exit(NPI_LNX_FAILURE, FALSE);
	}

	/**********************************************************************
	 * Open the serial interface
	 */

	switch(serialCfg.devIdx)
	{
		case NPI_SERVER_DEVICE_INDEX_UART_USB:
			// Initialization of UART for USB is the same as for physical UART.
			// Except for Reset GPIO
		case NPI_SERVER_DEVICE_INDEX_UART:
	#if (defined NPI_UART) && (NPI_UART == TRUE)
		{
			ret = (NPI_OpenDeviceFnArr[NPI_SERVER_DEVICE_INDEX_UART])(serialCfg.devPath, (npiUartCfg_t *)&serialCfg.serial.npiUartCfg);

			// Now configure reset GPIO for physical UART
			if (serialCfg.devIdx == (unsigned)NPI_SERVER_DEVICE_INDEX_UART)
			{
				if ( NPI_LNX_FAILURE == (ret = HalGpioResetInit((halGpioCfg_t *)&serialCfg.gpioCfg[2])))
				{
					return ret;
				}
			}
		}
	#endif
		break;
		case NPI_SERVER_DEVICE_INDEX_SPI:
	#if (defined NPI_SPI) && (NPI_SPI == TRUE)
		{
			// Now open device for processing
			LOG_DEBUG("Opening SPI device...\n");
			ret = (NPI_OpenDeviceFnArr[serialCfg.devIdx])(serialCfg.devPath, (npiSpiCfg_t *) &serialCfg.serial.npiSpiCfg);

			if (ret != NPI_LNX_SUCCESS)
				LOG_ERROR("%s(): Failed to open SPI device!\n", __FUNCTION__);
			else
			{
				// Perform Reset of the RNP
				LOG_INFO("%s(): Resetting RNP...\n", __FUNCTION__);
				(NPI_ResetSlaveFnArr[serialCfg.devIdx])();

				// Do the Hw Handshake
				LOG_INFO("%s(): Synch slave...\n", __FUNCTION__);
				(NPI_SynchSlaveFnArr[serialCfg.devIdx])();
			}
		}
	#endif
		break;

		case NPI_SERVER_DEVICE_INDEX_I2C:
	#if (defined NPI_I2C) && (NPI_I2C == TRUE)
		{
			// Open the Device and perform a reset
			ret = (NPI_OpenDeviceFnArr[serialCfg.devIdx])(serialCfg.devPath, (npiI2cCfg_t *) &serialCfg.serial.npiI2cCfg);
			if (ret != NPI_LNX_SUCCESS)
				LOG_ERROR("%s(): Failed to open I2C device!\n", __FUNCTION__);
		}
	#endif
		break;
		default:
			ret = NPI_LNX_FAILURE;
		break;
	}

	/**********************************************************************
	 * Configure Debug Interface if supported
	 */
	if (serialCfg.debugSupported)
	{
		ret = configureDebugInterface();
	}

	// The following will exit if ret != SUCCESS
	NPI_LNX_IPC_Exit(ret, FALSE);


#ifdef __STRESS_TEST__
	/**********************************************************************
	 * Setup StressTesting
	 **********************************************************************/

	int i = 0, fdStressTestData, done=0;
	char pathName[128];
	do
	{
		snprintf(pathName, sizeof(pathName), "results/stressTestData%.4d.txt", i++);
		LOG_ALWAYS("%s\n", pathName);
		fdStressTestData = open( pathName , O_CREAT | O_EXCL | O_WRONLY, S_IWRITE | S_IREAD );
		LOG_ALWAYS("fd = %d\n", fdStressTestData);
		if (fdStressTestData >= 0)
			done = 1;
		else
			close(fdStressTestData);
	} while (done == 0);
	// Now it's safe to open the file
	fpStressTestData = fopen(pathName, "w");

	time_t rawTime;
	time(&rawTime);
	fprintf(fpStressTestData, "*******************************************************************\n");
	fprintf(fpStressTestData, "\nTiming Statistics file created on %s\n\n", ctime(&rawTime));
	fprintf(fpStressTestData, "*******************************************************************\n");
#endif //__STRESS_TEST__


	/**********************************************************************
	 * Now that everything has been initialized and configured, let's open
	 * a socket and begin listening.
	 **********************************************************************/
	sNPIlisten = setupSocket(&serialCfg);

	fd_set activeConnectionsFDsSafeCopy;
	int justConnected;
	int c;

	// Connection main loop. Cannot get here with ret != SUCCESS

	char *toNpiLnxLog = (char *)malloc(AP_MAX_BUF_LEN);

	// Clear file descriptor sets
	FD_ZERO(&activeConnectionsFDs);
	FD_ZERO(&activeConnectionsFDsSafeCopy);

	// Add the listener to the set
	FD_SET(sNPIlisten, &activeConnectionsFDs);
	fdmax = sNPIlisten;

#if (defined __DEBUG_TIME__) || (__STRESS_TEST__)
	clock_gettime(CLOCK_MONOTONIC, &gStartTime);
#endif // (defined __DEBUG_TIME__) || (__STRESS_TEST__)
	//                                            debug_
	LOG_INFO("Waiting for first connection on #%d...\n", sNPIlisten);

	while (ret == NPI_LNX_SUCCESS)
	{
		activeConnectionsFDsSafeCopy = activeConnectionsFDs;

		// First use select to find activity on the sockets
		if (select (fdmax + 1, &activeConnectionsFDsSafeCopy, NULL, NULL, NULL) == -1)
		{
			if (errno != EINTR)
			{
				perror("select");
				npi_ipc_errno = NPI_LNX_ERROR_IPC_SOCKET_SELECT_CHECK_ERRNO;
				ret = NPI_LNX_FAILURE;
				break;
			}
			continue;
		}

		// Then process this activity
		for (c = 0; c <= fdmax; c++)
		{
			if (FD_ISSET(c, &activeConnectionsFDsSafeCopy))
			{
				if (c == sNPIlisten)
				{
					int addrLen = 0;
					// Accept a connection from a client.
					addrLen = sizeof(their_addr);
					justConnected = accept(sNPIlisten,
							(struct sockaddr *) &their_addr,
							(socklen_t *) &addrLen);

					if (justConnected == -1)
					{
						perror("accept");
						npi_ipc_errno = NPI_LNX_ERROR_IPC_SOCKET_ACCEPT;
						ret = NPI_LNX_FAILURE;
						break;
					}
					else
					{
#ifndef NPI_UNIX
						char ipstr[INET6_ADDRSTRLEN];
						char ipstr2[INET6_ADDRSTRLEN];
#endif //NPI_UNIX
						FD_SET(justConnected, &activeConnectionsFDs);
						if (justConnected > fdmax)
							fdmax = justConnected;
#ifdef NPI_UNIX
						snprintf(toNpiLnxLog, AP_MAX_BUF_LEN, "Connected to #%d.", justConnected);
#else
						//                                            debug_
						inet_ntop(AF_INET, &((struct sockaddr_in *) &their_addr)->sin_addr, ipstr, sizeof ipstr);
						inet_ntop(AF_INET6, &((struct sockaddr_in6 *)&their_addr)->sin6_addr, ipstr2, sizeof ipstr2);
						snprintf(toNpiLnxLog, AP_MAX_BUF_LEN, "Connected to #%d.(%s / %s)", justConnected, ipstr, ipstr2);
#endif //NPI_UNIX
						writeToNpiLnxLog(toNpiLnxLog);
						LOG_INFO("%s\n", toNpiLnxLog);
						ret = addToActiveList(justConnected);

#ifdef __DEBUG_TIME__
						if (__DEBUG_TIME_ACTIVE == TRUE)
						{
							clock_gettime(CLOCK_MONOTONIC, &gStartTime);
						}
#endif //__DEBUG_TIME__
					}
				}
				else
				{
					ret = NPI_LNX_IPC_ConnectionHandle(c, &npiIpcRecvBuf);
					if (ret == NPI_LNX_SUCCESS)
					{
						// Everything is ok
					}
					else
					{
						uint8 childThread;
						switch (npi_ipc_errno)
						{
						case NPI_LNX_ERROR_IPC_RECV_DATA_DISCONNECT:
							close(c);
							LOG_INFO("Removing connection #%d due to disconnect.\n", c);
							// Connection closed. Remove from set
							FD_CLR(c, &activeConnectionsFDs);
							// We should now set ret to NPI_SUCCESS, but there is still one fatal error
							// possibility so simply set ret = to return value from removeFromActiveList().
							ret = removeFromActiveList(c);
							snprintf(toNpiLnxLog, AP_MAX_BUF_LEN, "Removed connection #%d", c);
							//							LOG_WARN("%s\n", toNpiLnxLog);
							writeToNpiLnxLog(toNpiLnxLog);
							break;
						case NPI_LNX_ERROR_UART_SEND_SYNCH_TIMEDOUT:
							//This case can happen in some particular condition:
							// if the network is in BOOT mode, it will not answer any synchronous request other than SYS_BOOT request.
							// if we exit immediately, we will never be able to recover the NP device.
							// This may be replace in the future by an update of the RNP behavior
							LOG_WARN("Synchronous Request Timeout...");
							snprintf(toNpiLnxLog, AP_MAX_BUF_LEN, "Removed connection #%d", c);
							LOG_WARN("%s\n", toNpiLnxLog);
							writeToNpiLnxLog(toNpiLnxLog);
							ret = NPI_LNX_SUCCESS;
							npi_ipc_errno = NPI_LNX_SUCCESS;
							break;

						case NPI_LNX_ERROR_HAL_DBG_IFC_WAIT_DUP_READY:
							// Device did not respond, it may be that it's not in debug mode anymore.
							LOG_WARN("Chip failed to respond\n");
							// This error should not be considered critical at this stage.
							snprintf(toNpiLnxLog, AP_MAX_BUF_LEN, "Could not get chip ID, device not in debug mode as it failed to respond\n");
							writeToNpiLnxLog(toNpiLnxLog);
							npi_ipc_errno = NPI_LNX_SUCCESS;
							ret = NPI_LNX_SUCCESS;
							break;
						case NPI_LNX_ERROR_HAL_DBG_IFC_ASYNCH_INVALID_CMDID:
							// This is not a critical error, so don't cause server to exit.
							// It simply tells that an invalid AREQ CMD was requested.
							snprintf(toNpiLnxLog, AP_MAX_BUF_LEN, "Invalid asynchronous request to debug interface #%c", c);
							writeToNpiLnxLog(toNpiLnxLog);
							ret = NPI_LNX_SUCCESS;
							npi_ipc_errno = NPI_LNX_SUCCESS;
							break;
						default:
							if (npi_ipc_errno == NPI_LNX_SUCCESS)
							{
								// Do not report and abort if there is no real error.
								ret = NPI_LNX_SUCCESS;
							}
							else if (NPI_LNX_ERROR_JUST_WARNING(npi_ipc_errno))
							{
								// This may be caused by an unexpected reset. Write it to the log,
								// but keep going.
								// Everything about the error can be found in the message, and in npi_ipc_errno:
								childThread = npiIpcRecvBuf.cmdId;
								snprintf(toNpiLnxLog, AP_MAX_BUF_LEN, "Child thread with ID %d in module %d reported error:\t%.*s",
										NPI_LNX_ERROR_THREAD(childThread),
										NPI_LNX_ERROR_MODULE(childThread),
										(int)sizeof(npiIpcRecvBuf.pData),
										(char *)(npiIpcRecvBuf.pData));
								//							LOG_WARN("%s\n", toNpiLnxLog);
								writeToNpiLnxLog(toNpiLnxLog);
								// Force continuation
								ret = NPI_LNX_SUCCESS;
							}
							else
							{
								//							debug_
								LOG_ERROR("npi_ipc_errno 0x%.8X\n", npi_ipc_errno);
								// Everything about the error can be found in the message, and in npi_ipc_errno:
								childThread = npiIpcRecvBuf.cmdId;
								snprintf(toNpiLnxLog, AP_MAX_BUF_LEN, "Child thread with ID %d in module %d reported error:\t%.*s",
										NPI_LNX_ERROR_THREAD(childThread),
										NPI_LNX_ERROR_MODULE(childThread),
										(int)sizeof(npiIpcRecvBuf.pData),
										(char *)(npiIpcRecvBuf.pData));
								//							LOG_ERROR("%s\n", toNpiLnxLog);
								writeToNpiLnxLog(toNpiLnxLog);
							}
							break;
						}

						// Check if error requested a reset
						if (NPI_LNX_ERROR_RESET_REQUESTED(npi_ipc_errno))
						{
							// Yes, utilize server control API to reset current device
							// Do it by reconnecting so that threads are kept synchronized
							npiMsgData_t npi_ipc_buf_tmp;
							int localRet = NPI_LNX_SUCCESS;
							LOG_WARN("Reset was requested, so try to disconnect device %d\n", serialCfg.devIdx);
							npi_ipc_buf_tmp.cmdId = NPI_LNX_CMD_ID_DISCONNECT_DEVICE;
							localRet = npi_ServerCmdHandle((npiMsgData_t *)&npi_ipc_buf_tmp);
							LOG_WARN("Disconnection from device %d was %s\n", serialCfg.devIdx, (localRet == NPI_LNX_SUCCESS) ? "successful" : "unsuccessful");
							if (localRet == NPI_LNX_SUCCESS)
							{
								LOG_WARN("Then try to connect device %d again\n", serialCfg.devIdx);
								int bigDebugWas = __BIG_DEBUG_ACTIVE;
								if (bigDebugWas == FALSE)
								{
									__BIG_DEBUG_ACTIVE = TRUE;
									LOG_ALWAYS("__BIG_DEBUG_ACTIVE set to TRUE\n");
								}
								npi_ipc_buf_tmp.cmdId = NPI_LNX_CMD_ID_CONNECT_DEVICE;
								localRet = npi_ServerCmdHandle((npiMsgData_t *)&npi_ipc_buf_tmp);
								LOG_WARN("Reconnection to device %d was %s\n", serialCfg.devIdx, (localRet == NPI_LNX_SUCCESS) ? "successful" : "unsuccessful");
								if (bigDebugWas == FALSE)
								{
									__BIG_DEBUG_ACTIVE = FALSE;
									LOG_ALWAYS("__BIG_DEBUG_ACTIVE set to FALSE\n");
								}
							}
						}

						// If this error was sent through socket; close this connection
						if ((npiIpcRecvBuf.subSys & RPC_CMD_TYPE_MASK) == RPC_CMD_NOTIFY_ERR)
						{
							close(c);
							LOG_ERROR("Removing connection #%d due to RPC_CMD_NOTIFY_ERR\n", c);
							// Connection closed. Remove from set
							FD_CLR(c, &activeConnectionsFDs);
						}
					}
				}
			}
		}
	}
	free(toNpiLnxLog);

	LOG_WARN("Exit socket while loop\n");
	/**********************************************************************
	 * Remember to close down all connections
	 *********************************************************************/

#ifndef NPI_UNIX
	freeaddrinfo(servinfo); // free the linked-list
#endif //NPI_UNIX
	(NPI_CloseDeviceFnArr[serialCfg.devIdx])();

	// Free all remaining memory
	NPI_LNX_IPC_Exit(NPI_LNX_SUCCESS + 1, TRUE);

#if (defined __STRESS_TEST__) && (__STRESS_TEST__ == TRUE)
	//            close(fpStressTestData);
	//            close(fdStressTestData);
#endif //(defined __STRESS_TEST__) && (__STRESS_TEST__ == TRUE)

	return ret;
}


/**************************************************************************************************
 *
 * @fn          addToActiveList
 *
 * @brief       Manage active connections, add to list
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      -1 if something went wrong, 0 if success
 *
 **************************************************************************************************/

static int addToActiveList(int c)
{
	if (activeConnections.size <= NPI_SERVER_CONNECTION_QUEUE_SIZE)
	{
		// Entry at position activeConnections.size is always the last available entry
		activeConnections.list[activeConnections.size] = c;

		// Increment size
		activeConnections.size++;

		return NPI_LNX_SUCCESS;
	}
	else
	{
		// There's no more room in the list
		npi_ipc_errno = NPI_LNX_ERROR_IPC_ADD_TO_ACTIVE_LIST_NO_ROOM;
		return NPI_LNX_FAILURE;
	}
}

/**************************************************************************************************
 *
 * @fn          removeFromActiveList
 *
 * @brief       Manage active connections, remove from list. Re organize so list is full
 * 				up to its declared size
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      -1 if something went wrong, 0 if success
 *
 **************************************************************************************************/

static int removeFromActiveList(int c)
{
	int i;
	// Find entry
	for (i = 0; i < activeConnections.size; i++)
	{
		if (activeConnections.list[i] == c)
			break;
	}


	if (i < activeConnections.size)
	{
		//Check if the last active conection has been removed
		if (activeConnections.size == 1)
		{
			//continue to wait for new connection
			activeConnections.size = 0;
			activeConnections.list[0] = 0;
			LOG_DEBUG("No  Active Connections");
		}
		else
		{

			// Found our entry, replace this entry by the last entry
			activeConnections.list[i] = activeConnections.list[activeConnections.size - 1];

			// Decrement size
			activeConnections.size--;
#ifdef __BIG_DEBUG__
			LOG_DEBUG("Remaining Active Connections: #%d", activeConnections.list[0]);
			// Send data to all connections, except listener
			for (i = 1; i < activeConnections.size; i++)
			{
				LOG_DEBUG(", #%d", activeConnections.list[i]);
			}
			LOG_DEBUG("\n");
#endif //__BIG_DEBUG__
		}
		return NPI_LNX_SUCCESS;
	}
	else
	{
		// Could not find entry
		npi_ipc_errno = NPI_LNX_ERROR_IPC_REMOVE_FROM_ACTIVE_LIST_NOT_FOUND;
		return NPI_LNX_FAILURE;
	}
}

#ifdef __DEBUG_TIME__
static void time_print_npi_ipc_buf(const char *strDirection, const npiMsgData_t *npiMsgData, struct timespec const *callersStartTime, struct timespec const *callersCurrentTime, struct timespec *callersPreviousTime)
{
	char 		tmpStr[128 + (sizeof(npiMsgData_t) * 3)];
	size_t	tmpLen = 0;
	int      i;

	snprintf(tmpStr, sizeof(tmpStr), "[%s %2d bytes, subSys 0x%02X, cmdId 0x%02X, pData: \040 ",
		strDirection,
		npiMsgData->len,
		npiMsgData->subSys,
		npiMsgData->cmdId);
	tmpLen = strlen(tmpStr);

	for (i = 0; i < npiMsgData->len && (tmpLen < sizeof(tmpStr)); i++)
	{
		snprintf(tmpStr+tmpLen, sizeof(tmpStr)-tmpLen, " %02X", npiMsgData->pData[i]);
		tmpLen += 3;
	}
	snprintf(tmpStr+tmpLen, sizeof(tmpStr)-tmpLen, "]");
	time_printf_always_localized(callersStartTime, callersCurrentTime, callersPreviousTime, "%s\n", tmpStr);
}
#endif


/**************************************************************************************************
 *
 * @fn          NPI_LNX_IPC_ConnectionHandle
 *
 * @brief       Handle connections
 *
 * input parameters
 *
 *    connection - connection index or -1 if no specific connection
 *		recvBuf - buffer to put received data into
 *
 * output parameters
 *		recvBuf - upon return, buffer will contain the received data
 *
 * None.
 *
 * @return      STATUS
 *
 **************************************************************************************************/
static int NPI_LNX_IPC_ConnectionHandle(int connection, npiMsgData_t *recvBuf)
{
	npiMsgData_t sendBuf;
	char tmpStr[512];
	size_t strLen;
	strLen = 0;
	int          n, i, ret = NPI_LNX_SUCCESS;

	// Handle the connection
	LOG_DEBUG("Receive message...\n");

	// Receive only NPI header first. Then then number of bytes indicated by length.
	n = recv(connection, recvBuf, RPC_FRAME_HDR_SZ, 0);
	if (n <= 0)
	{
		if (n < 0)
		{
			perror("recv");
			LOG_ERROR("%s(): Receive message (n = %d)...\n", __FUNCTION__, n);
			if ( (errno == ENOTSOCK) || (errno == EPIPE))
			{
				LOG_ERROR("Tried to read #%d as socket\n", connection);
				LOG_ERROR("Will disconnect #%d\n", connection);
				npi_ipc_errno = NPI_LNX_ERROR_IPC_RECV_DATA_DISCONNECT;
				ret = NPI_LNX_FAILURE;
			}
			else if (errno == ECONNRESET)
			{
//				debug_
				LOG_WARN("Client disconnect while attempting to send to it\n");
				LOG_WARN("Will disconnect #%d\n", connection);
				npi_ipc_errno = NPI_LNX_ERROR_IPC_RECV_DATA_DISCONNECT;
				ret = NPI_LNX_FAILURE;
			}
			else
			{
				LOG_ERROR("%s(): Receive message (fail) with errno=%d.\n", __FUNCTION__, errno);
				npi_ipc_errno = NPI_LNX_ERROR_IPC_RECV_DATA_CHECK_ERRNO;
				ret = NPI_LNX_FAILURE;
			}
		}
		else
		{
			LOG_WARN("Client #%d disconnected.\n", connection);
			npi_ipc_errno = NPI_LNX_ERROR_IPC_RECV_DATA_DISCONNECT;
			ret = NPI_LNX_FAILURE;
		}
	}
	else if (n == RPC_FRAME_HDR_SZ)
	{
		// LOG_DEBUG("%s(): Receive message header (good)...\n", __FUNCTION__);
		// Now read out the payload of the NPI message, if it exists
		if (recvBuf->len > 0)
		{
			n = recv(connection, recvBuf->pData, recvBuf->len , 0);
//			LOG_DEBUG("Receive payload len %d\n", n);
			if (n != (int)recvBuf->len)
			{
				LOG_ERROR("%s(): Could not read out the NPI payload. Requested %d, but read %d!\n",
						__FUNCTION__,
						recvBuf->len, n);
				npi_ipc_errno = NPI_LNX_ERROR_IPC_RECV_DATA_TOO_FEW_BYTES;
				ret = NPI_LNX_FAILURE;
				if (n < 0)
				{
					perror("recv");
					// Disconnect this
					npi_ipc_errno = NPI_LNX_ERROR_IPC_RECV_DATA_DISCONNECT;
					ret = NPI_LNX_FAILURE;
				}
			}
			else
			{
				ret = NPI_LNX_SUCCESS;
			}
			// n is only used by debug traces from here on, but add header length
			// so the whole message is written out
			n += RPC_FRAME_HDR_SZ;
		}
		/*
		 * Take the message from the client and pass it to the NPI
		 */
	#ifdef __DEBUG_TIME__
		static struct timespec prevTimeRec = {0,0};
		if (__DEBUG_TIME_ACTIVE == TRUE)
		{
			//            debug_
			struct timespec currentTime;
			clock_gettime(CLOCK_MONOTONIC, &currentTime);

		#ifdef __STRESS_TEST__
			long int diffPrevMillisecs;
			int      t;
			if (currentTime.tv_nsec >= prevTimeRec.tv_nsec)
			{
				diffPrevMillisecs = (currentTime.tv_nsec - prevTimeRec.tv_nsec) / 1000000;
				t = 0;
			}
			else
			{
				diffPrevMillisecs = ((currentTime.tv_nsec + 1000000000) - prevTimeRec.tv_nsec) / 1000000;
				t = 1;
			}

			if (diffPrevMillisecs < TIMING_STATS_SIZE)
				timingStats[INDEX_RECV][diffPrevMillisecs / TIMING_STATS_MS_DIV]++;
			else
				timingStats[INDEX_RECV][TIMING_STATS_SIZE / TIMING_STATS_MS_DIV]++;
		#endif //__STRESS_TEST__

			time_print_npi_ipc_buf("<--", recvBuf, &gStartTime, &currentTime, &prevTimeRec);
		}
	#endif //__DEBUG_TIME__

		if ((recvBuf->subSys & RPC_CMD_TYPE_MASK) == RPC_CMD_SREQ)
		{
			for (i = 0; i < recvBuf->len; i++)
			{
				snprintf(tmpStr+strLen, sizeof(tmpStr)-strLen, "%02X ", recvBuf->pData[i]);
				strLen += 3;
			}
			LOG_DEBUG("NPI SREQ:  (Total Len %d, Data Len %d, subSys 0x%02x, cmdId 0x%02x) PAYLOAD: %s\n", n, recvBuf->len, recvBuf->subSys, recvBuf->cmdId, tmpStr);

			if ((recvBuf->subSys & RPC_SUBSYSTEM_MASK) == RPC_SYS_DEBUG)
			{
				if (serialCfg.debugSupported)
				{
					// Synchronous Call to Debug Interface
					ret = Hal_DebugInterface_SynchMsgCback(recvBuf);
				}
				else
				{
					LOG_DEBUG("Debug Interface SREQ received, but not supported\n");
					// Debug not supported, return 0xFF
					recvBuf->pData[0] = 0xFF;
					ret = NPI_LNX_SUCCESS;
				}
			}
			else if ((recvBuf->subSys & RPC_SUBSYSTEM_MASK) == RPC_SYS_SRV_CTRL)
			{

				//SREQ Command send to this server.
				ret = npi_ServerCmdHandle(recvBuf);
			}
			else
			{
				uint8 sreqHdr[RPC_FRAME_HDR_SZ] = {0};
				// Retain the header for later integrity check
				memcpy(sreqHdr, recvBuf, RPC_FRAME_HDR_SZ);
				// Synchronous request requires an answer...
				ret = (NPI_SendSynchDataFnArr[serialCfg.devIdx])(recvBuf);
				if ( (ret != NPI_LNX_SUCCESS) &&
						( (npi_ipc_errno == NPI_LNX_ERROR_HAL_GPIO_WAIT_SRDY_CLEAR_POLL_TIMEDOUT) ||
							(npi_ipc_errno == NPI_LNX_ERROR_HAL_GPIO_WAIT_SRDY_SET_POLL_TIMEDOUT) ))
				{
					// Report this error to client through a pseudo response
					recvBuf->len = 1;
					recvBuf->pData[0] = 0xFF;
				}
				else
				{
					// Capture incoherent SRSP, check type and subsystem
					if ( (( recvBuf->subSys & ~(RPC_SUBSYSTEM_MASK)) != RPC_CMD_SRSP )
						||
						  (( recvBuf->subSys & (RPC_SUBSYSTEM_MASK)) != (sreqHdr[RPC_POS_CMD0] & RPC_SUBSYSTEM_MASK))
						)
					{
						// Report this error to client through a pseudo response
						recvBuf->len = 1;
						recvBuf->subSys = (sreqHdr[RPC_POS_CMD0] & RPC_SUBSYSTEM_MASK) | RPC_CMD_SRSP;
						recvBuf->cmdId = sreqHdr[RPC_POS_CMD1];
						recvBuf->pData[0] = 0xFF;
					}
				}
			}

			if ( (ret == NPI_LNX_SUCCESS) ||
						(npi_ipc_errno == NPI_LNX_ERROR_HAL_GPIO_WAIT_SRDY_CLEAR_POLL_TIMEDOUT) ||
						(npi_ipc_errno == NPI_LNX_ERROR_HAL_GPIO_WAIT_SRDY_SET_POLL_TIMEDOUT) ||
					(npi_ipc_errno == NPI_LNX_ERROR_HAL_DBG_IFC_WAIT_DUP_READY) )
			{
				n = ( (int)recvBuf->len + RPC_FRAME_HDR_SZ );

				// Copy response into transmission buffer
				memcpy(&sendBuf, recvBuf, n);

				// Command type is not set, so set it here
				sendBuf.subSys |= RPC_CMD_SRSP;

				strLen = 0;
				for (i = 0; i < sendBuf.len; i++)
				{
					snprintf(tmpStr+strLen, sizeof(tmpStr)-strLen, "%02X ", sendBuf.pData[i]);
					strLen += 3;
				}
				LOG_DEBUG("NPI SRSP:  (Total Len %d, Data Len %d, subSys 0x%02x, cmdId 0x%02x) PAYLOAD: %s\n", n, sendBuf.len, sendBuf.subSys, sendBuf.cmdId, tmpStr);

				if (sendBuf.len == 0)
				{
					LOG_ERROR("SRSP is 0!\n");
				}

				//			pthread_mutex_lock(&npiSyncRespLock);
				// Send bytes
				ret = NPI_LNX_IPC_SendData(&sendBuf, connection);
			}
			else
			{
				// Keep status from NPI_SendSynchDataFnArr
				LOG_ERROR("SRSP: ret = 0x%.8X, npi_ipc_errno 0x%.8X\n", ret, npi_ipc_errno);
			}
		}
		else if ((recvBuf->subSys & RPC_CMD_TYPE_MASK) == RPC_CMD_AREQ)
		{
			strLen = 0;
			for (i = 0; i < recvBuf->len; i++)
			{
				snprintf(tmpStr+strLen, sizeof(tmpStr)-strLen, "%02X ", recvBuf->pData[i]);
				strLen += 3;
			}
			LOG_DEBUG("NPI AREQ:  (Total Len %d, Data Len %d, subSys 0x%02x, cmdId 0x%02x) PAYLOAD: %s\n", n, recvBuf->len, recvBuf->subSys, recvBuf->cmdId, tmpStr);

			if ((recvBuf->subSys & RPC_SUBSYSTEM_MASK) == RPC_SYS_DEBUG)
			{
				if (serialCfg.debugSupported)
				{
					// Asynchronous Call to Debug Interface
					ret = Hal_DebugInterface_AsynchMsgCback(recvBuf);
				}
				else
				{
					LOG_DEBUG("Debug Interface AREQ received, but not supported\n");
					// Debug not supported, do nothing
					ret = NPI_LNX_SUCCESS;
				}
			}
			else if ((recvBuf->subSys & RPC_SUBSYSTEM_MASK) == RPC_SYS_SRV_CTRL)
			{
				// Print caller ID
				LOG_INFO("AREQ received from %d to control NPI Server\n", connection);
				//AREQ Command send to this server.
				ret = npi_ServerCmdHandle(recvBuf);
			}
			else
			{
				// Asynchronous request may just be sent
				ret = (NPI_SendAsynchDataFnArr[serialCfg.devIdx])(recvBuf);
			}
		}
		else if ((recvBuf->subSys & RPC_CMD_TYPE_MASK)  == RPC_CMD_NOTIFY_ERR)
		{
			// An error occurred in a child thread.
			ret = NPI_LNX_FAILURE;
		}
		else
		{
			LOG_WARN("Can only accept AREQ or SREQ for now...\n");
			strLen = 0;
			for (i = 0; i < recvBuf->len; i++)
			{
				snprintf(tmpStr+strLen, sizeof(tmpStr)-strLen, "%02X ", recvBuf->pData[i]);
				strLen += 3;
			}
			LOG_DEBUG("Unknown:  (Total Len %d, Data Len %d, subSys 0x%02x, cmdId 0x%02x) PAYLOAD: %s\n", n, recvBuf->len, recvBuf->subSys, recvBuf->cmdId, tmpStr);

			npi_ipc_errno = NPI_LNX_ERROR_IPC_RECV_DATA_INCOMPATIBLE_CMD_TYPE;
			// Ignore error. It's not deadly
			ret = NPI_LNX_SUCCESS;
		}
	}
	else
	{
		LOG_ERROR("%s(): Received %d bytes when asked for %d (RPC_FRAME_HDR_SZ)\n", __FUNCTION__, n, RPC_FRAME_HDR_SZ);
	}

#if (defined __BIG_DEBUG__) && (__BIG_DEBUG__ == TRUE)
	// This will effectively result in an echo
	memcpy(&sendBuf, recvBuf, sizeof(sendBuf));
#endif

	if ((ret == (int)NPI_LNX_FAILURE) && (npi_ipc_errno == (int)NPI_LNX_ERROR_IPC_RECV_DATA_DISCONNECT))
	{
		LOG_DEBUG("Done with %d\n", connection);
	}
	else
	{
		LOG_DEBUG("!Done\n");
	}

	return ret;
}

/**************************************************************************************************
 *
 * @fn          NPI_LNX_IPC_SendData
 *
 * @brief       Send data from NPI to client
 *
 * input parameters
 *
 * @param          sendBuf                            - message to send
 * @param          connection                         - connection to send message (for synchronous response) otherwise -1 for all connections
 *
 * output parameters
 *
 * None.
 *
 * @return      STATUS
 *
 **************************************************************************************************/
static int NPI_LNX_IPC_SendData(npiMsgData_t const *sendBuf, int connection)
{
	int bytesSent = 0, ix=0, ret = NPI_LNX_SUCCESS;
	int len = (int)(sendBuf->len) + RPC_FRAME_HDR_SZ;

#ifdef __DEBUG_TIME__
	static struct timespec prevTimeSend = {0,0};

	if (__DEBUG_TIME_ACTIVE == TRUE)
	{
		struct timespec currentTime;
		clock_gettime(CLOCK_MONOTONIC, &currentTime);

	#ifdef __STRESS_TEST__
		long int diffPrevMillisecs;
		int      t;
		if (currentTime.tv_nsec >= prevTimeSend.tv_nsec)
		{
			diffPrevMillisecs = (currentTime.tv_nsec - prevTimeSend.tv_nsec) / 1000000;
			t = 0;
		}
		else
		{
			diffPrevMillisecs = ((curtentTime.tv_nsec + 1000000000) - prevTimeSend.tv_nsec) / 1000000;
			t = 1;
		}

		if (diffPrevMillisecs < TIMING_STATS_SIZE)
			timingStats[INDEX_SEND][diffPrevMillisecs / TIMING_STATS_MS_DIV]++;
		else
			timingStats[INDEX_SEND][TIMING_STATS_SIZE / TIMING_STATS_MS_DIV]++;

		// Save timingStats if inactive for > 10 seconds
		if ((currentTime.tv_sec - prevTimeSend.tv_sec) > 10)
		{
			time_t rawTime;
			time(&rawTime);
			LOG_ALWAYS("Timing Statistics as of %s:\n", ctime(&rawTime));
			fprintf(fpStressTestData, "\nTiming Statistics as of %s:\n", ctime(&rawTime));
			for (ix = 0; ix < (TIMING_STATS_SIZE / TIMING_STATS_MS_DIV); i++ )
			{
				LOG_ALWAYS(" %4d: \t %8d\n", ix * TIMING_STATS_MS_DIV, timingStats[INDEX_SEND][ix]);
				fprintf(fpStressTestData, " %4d: \t %8d\n", ix * TIMING_STATS_MS_DIV, timingStats[INDEX_SEND][ix]);
			}
			LOG_ALWAYS(" More than %u: \t %8u\n", TIMING_STATS_SIZE, timingStats[INDEX_SEND][TIMING_STATS_SIZE]);
			fprintf(fpStressTestData, " More than %u: \t %8u\n", TIMING_STATS_SIZE, timingStats[INDEX_SEND][TIMING_STATS_SIZE]);

			// Then clear statistics for next set.
			memset(timingStats[INDEX_SEND], 0, (TIMING_STATS_SIZE / TIMING_STATS_MS_DIV) + 1);
		}

	#endif //__STRESS_TEST__

		time_print_npi_ipc_buf("-->", sendBuf, &gStartTime, &currentTime, &prevTimeSend);
	}
#endif //__DEBUG_TIME__

	if (connection < 0)
	{
#ifdef __BIG_DEBUG__
		LOG_ALWAYS("Dispatch AREQ to all active connections: #%d", activeConnections.list[0]);
		// Send data to all connections, except listener
		for (ix = 1; ix < activeConnections.size; ix++)
		{
			LOG_ALWAYS(", %d", activeConnections.list[ix]);
		}
		LOG_ALWAYS(".\n");
#endif //__BIG_DEBUG__
		// Send data to all connections, except listener
		for (ix = 0; ix < activeConnections.size; ix++)
		{
			if (activeConnections.list[ix] != sNPIlisten)
			{
//				LOG_DEBUG("[AREQ] Sending message...\n");
				bytesSent = send(activeConnections.list[ix], (char *)sendBuf, len, MSG_NOSIGNAL);
//				LOG_DEBUG("[AREQ] Sent %d bytes...\n", bytesSent);

				LOG_DEBUG("...sent %d bytes to Client #%d\n", bytesSent, activeConnections.list[ix]);

				if (bytesSent < 0)
				{
					if (errno != ENOTSOCK)
					{
						char errorStr[30];
						snprintf(errorStr, sizeof(errorStr), "send %d, %d", activeConnections.list[ix], errno);
						perror(errorStr);
						// Remove from list if detected bad file descriptor, or broken pipe
						if ( (errno == EBADF) || (errno == EPIPE) )
						{
							LOG_ERROR("Send to all: Removing connection #%d (errno=%d)\n", activeConnections.list[ix], errno);
							close(activeConnections.list[ix]);
							// Connection closed. Remove from set
							FD_CLR(activeConnections.list[ix], &activeConnectionsFDs);
							ret = removeFromActiveList(activeConnections.list[ix]);
						}
						else
						{
							npi_ipc_errno = NPI_LNX_ERROR_IPC_SEND_DATA_ALL;
							ret = NPI_LNX_FAILURE;
						}
					}
				}
				else if (bytesSent != len)
				{
					LOG_ERROR("Failed to send all %d bytes on socket\n", len);
				}
			}
		}
	}
	else
	{
		// Send to specific connection only
//		LOG_DEBUG("[AREQ] Sending message...\n");
		bytesSent = send(connection, (char *)sendBuf, len, MSG_NOSIGNAL);
//		LOG_DEBUIG("[AREQ] Sent %d byte message...\n", bytesSent);

		LOG_DEBUG("...sent %d bytes to Client #%d\n", bytesSent, connection);

		if (bytesSent < 0)
		{
			perror("send");
			// Remove from list if detected bad file descriptor
			if (errno == EBADF)
			{
				LOG_ERROR("Removing connection #%d\n", connection);
				close(connection);
				// Connection closed. Remove from set
				FD_CLR(connection, &activeConnectionsFDs);
				ret = removeFromActiveList(connection);
				if (ret == NPI_LNX_SUCCESS)
				{
					npi_ipc_errno = NPI_LNX_ERROR_IPC_SEND_DATA_SPECIFIC_CONNECTION_REMOVED;
					ret = NPI_LNX_FAILURE;
				}
			}
			else
			{
				npi_ipc_errno = NPI_LNX_ERROR_IPC_SEND_DATA_SPECIFIC;
				ret = NPI_LNX_FAILURE;
			}
		}
	}

	return ret;
}

/**************************************************************************************************
 * @fn          NPI_AsynchMsgCback
 *
 * @brief       This function is an NPI callback to the client that indicates an
 *              asynchronous message has been received. The client software is
 *              expected to complete this call.
 *
 *              Note: The client must copy this message if it requires it
 *                    beyond the context of this call.
 *
 * input parameters
 *
 * @param       *pMsg - A pointer to an asynchronously received message.
 *
 * output parameters
 *
 * None.
 *
 * @return      STATUS
 **************************************************************************************************
 */
int NPI_AsynchMsgCback(npiMsgData_t *pMsg)
{
	int i;
	//	int ret = NPI_LNX_SUCCESS;

	LOG_DEBUG("[-->] %d bytes, subSys 0x%.2X, cmdId 0x%.2X, pData:",
			pMsg->len,
			pMsg->subSys,
			pMsg->cmdId);
	for (i = 0; i < pMsg->len; i++)
	{
		LOG_DEBUG(" 0x%.2X", pMsg->pData[i]);
	}
	LOG_DEBUG("\n");


#ifdef __STRESS_TEST__

	// If packet is an AREQ RTI_ReceiveDataInd use first byte in payload as sequence number
	if ((pMsg->cmdId == 0x05) && (pMsg->pData[7] == 0x03)) //RTIS_CMD_ID_RTI_REC_DATA_IND && RTI_CMD_TEST_DATA_SEQUENCED
	{
		uint32 *incomingSeqNum = (uint32 *) &pMsg->pData[8];
		if (*incomingSeqNum != (ST_Parameters_t[1].currentSeqNumber[pMsg->pData[0]] + 1))
		{
			if (*incomingSeqNum == ST_Parameters_t[1].currentSeqNumber[pMsg->pData[0]])
				ST_Parameters_t[1].recErrors.seqNumIdentical++;
			else
				ST_Parameters_t[1].recErrors.errorInSeqNum++;

			LOG_ALWAYS("[ERR] Sequence Number \t (==: %d, !=: %d)\n",
					ST_Parameters_t[1].recErrors.seqNumIdentical,
					ST_Parameters_t[1].recErrors.errorInSeqNum);
			fprintf(fpStressTestData, " [ERR] Sequence Number \t (==: %d, !=: %d)\n",
					ST_Parameters_t[1].recErrors.seqNumIdentical,
					ST_Parameters_t[1].recErrors.errorInSeqNum);

			LOG_ALWAYS("\tLast Sequence Number: (srcIdx: 0x%.2X) \t %d\n", pMsg->pData[0], ST_Parameters_t[1].currentSeqNumber[pMsg->pData[0]]);
			fprintf(fpStressTestData, "\tLast Sequence Number: (srcIdx: 0x%.2X) \t %d\n", pMsg->pData[0], ST_Parameters_t[1].currentSeqNumber[pMsg->pData[0]]);

			LOG_ALWAYS("\tNew \040 Sequence Number: (srcIdx: 0x%.2X) \t %d", pMsg->pData[0], *incomingSeqNum);
			fprintf(fpStressTestData, "\tNew \040 Sequence Number: (srcIdx: 0x%.2X) \t %d", pMsg->pData[0], *incomingSeqNum);

			LOG_ALWAYS("\n");
			fprintf(fpStressTestData, "\n");
		}

		ST_Parameters_t[1].currentSeqNumber[pMsg->pData[0]] = *incomingSeqNum;
	}
#endif //__STRESS_TEST__

	return NPI_LNX_IPC_SendData(pMsg, -1);
}


/**************************************************************************************************
 * @fn          NPI_LNX_IPC_Exit
 *
 * @brief       This function will exit gracefully
 *
 *
 * input parameters
 *
 * @param       ret	-	exit condition. Return on Success, exit on Failure
 *
 * output parameters
 *
 * None.
 *
 * @return      None
 **************************************************************************************************
 */
static void NPI_LNX_IPC_Exit(int ret, uint8 freeSerial)
{
	(void)freeSerial;

	if (ret == (int)NPI_LNX_FAILURE)
	{
		// Don't even bother open a socket; device opening failed..
		LOG_FATAL("Could not open device... exiting (%d)\n", npi_ipc_errno);

		// Write error message to /dev/npiLnxLog
		writeToNpiLnxLog("Could not open device");

		exit(npi_ipc_errno);
	}
}

/**************************************************************************************************
 * @fn          NPI_LNX_IPC_NotifyError
 *
 * @brief       This function allows other threads to notify of error conditions.
 *
 *
 * input parameters
 *
 * @param       source		- source identifier
 * @param       *errorMsg 	- A string containing the error message.
 *
 * output parameters
 *
 * None.
 *
 * @return      None
 **************************************************************************************************
 */
int NPI_LNX_IPC_NotifyError(uint16 source, const char* errorMsg)
{
	int ret = NPI_LNX_SUCCESS;
	int sNPIconnected;
#ifndef NPI_UNIX
	struct addrinfo *resAddr;
#endif //NPI_UNIX

	const char *ipAddress = "127.0.0.1";


	/**********************************************************************
	 * Connect to the NPI server
	 **********************************************************************/

#ifdef NPI_UNIX
	int len;
	struct sockaddr_un remote;

	if ((sNPIconnected = socket(AF_UNIX, SOCK_STREAM, 0)) == -1)
	{
		perror("socket");
		ret = NPI_LNX_ERROR_IPC_NOTIFY_ERR_CREATE_SOCKET;
	}
#else
	struct addrinfo hints;

	memset(&hints, 0, sizeof(hints));
	hints.ai_family = AF_UNSPEC;
	hints.ai_socktype = SOCK_STREAM;

	//    ipAddress = "192.168.128.133";
	//    if ((res = getaddrinfo(NULL, ipAddress, &hints, &resAddr)) != 0)
	if (serialCfg.port == NULL)
	{
		// Fall back to default if port was not found in the configuration file
		LOG_WARN("Port not sent to NPI_LNX_IPC_NotifyError. Will use default port: %s", NPI_PORT);
		LOG_INFO("getaddrinfo: %s\n", gai_strerror(ret));
		ret = NPI_LNX_ERROR_IPC_NOTIFY_ERR_GET_ADDR_INFO;
	}
	else
	{
		LOG_DEBUG("[NOTIFY_ERROR] Port: %s\n", serialCfg.port);
		if ((ret = getaddrinfo(ipAddress, serialCfg.port, &hints, &resAddr)) != 0)
		{
			LOG_INFO("getaddrinfo: %s\n", gai_strerror(ret));
			ret = NPI_LNX_ERROR_IPC_NOTIFY_ERR_GET_ADDR_INFO;
		}
		else
		{
			ret = NPI_LNX_SUCCESS;
		}
	}


	if ((sNPIconnected = socket(resAddr->ai_family, resAddr->ai_socktype, resAddr->ai_protocol)) == -1)
	{
		perror("socket");
		ret = NPI_LNX_ERROR_IPC_NOTIFY_ERR_CREATE_SOCKET;
	}
#endif

	LOG_DEBUG("[NOTIFY] Trying to connect...\n");

#ifdef NPI_UNIX
	remote.sun_family = AF_UNIX;
	strcpy(remote.sun_path, ipAddress);
	len = strlen(remote.sun_path) + sizeof(remote.sun_family);
	if (connect(sNPIconnected, (struct sockaddr *)&remote, len) == -1)
	{
		perror("connect");
		ret = NPI_LNX_ERROR_IPC_NOTIFY_ERR_CONNECT;
	}
#else
	if (connect(sNPIconnected, resAddr->ai_addr, resAddr->ai_addrlen) == -1)
	{
		perror("connect");
		ret = NPI_LNX_ERROR_IPC_NOTIFY_ERR_CONNECT;
	}
#endif

	if (ret == NPI_LNX_SUCCESS)
		LOG_DEBUG("[NOTIFY] Connected.\n");


	int no = 0;
	// allow out-of-band data
	if (setsockopt(sNPIconnected, SOL_SOCKET, SO_OOBINLINE, &no, sizeof(int)) == -1)
	{
		perror("setsockopt");
		ret = NPI_LNX_ERROR_IPC_NOTIFY_ERR_SET_SOCKET_OPTIONS;
	}

	npiMsgData_t msg;

	if (strlen(errorMsg) <= AP_MAX_BUF_LEN)
	{
		memcpy(msg.pData, errorMsg, strlen(errorMsg));
	}
	else
	{
		errorMsg = "Default msg. Requested msg too long.\n";
		memcpy(msg.pData, errorMsg, strlen(errorMsg));
		LOG_WARN("[NOTIFY_ERROR] Size of error message too long (%zd, max %d).\n",
				strlen(errorMsg),
				AP_MAX_BUF_LEN);
	}
	// If last character is \n then remove it.
	if ((msg.pData[strlen(errorMsg) - 1]) == '\n')
	{
		msg.pData[strlen(errorMsg) - 1] = 0;
		msg.len = strlen(errorMsg);
	}
	else
	{
		msg.pData[strlen(errorMsg)] = 0;
		msg.len = strlen(errorMsg) + 1;
	}

	// For now the only required info here is command type.
	msg.subSys = RPC_CMD_NOTIFY_ERR;
	// CmdId is filled with the source identifier.
	msg.cmdId = source;

	send(sNPIconnected, &msg, msg.len + RPC_FRAME_HDR_SZ, MSG_NOSIGNAL);

	return ret;
}

static int npi_ServerCmdHandle(npiMsgData_t *pNpi_ipc_buf)
{
	int ret = NPI_LNX_SUCCESS;

	LOG_DEBUG("[NPI SERVER] Control: cmdId 0x%.2X\n", pNpi_ipc_buf->cmdId);

	switch(pNpi_ipc_buf->cmdId)
	{
		case NPI_LNX_CMD_ID_CTRL_TIME_PRINT_REQ:
		#ifdef __DEBUG_TIME__
			__DEBUG_TIME_ACTIVE = pNpi_ipc_buf->pData[0];
			LOG_ALWAYS("__DEBUG_TIME_ACTIVE set to %d (%s)\n",__DEBUG_TIME_ACTIVE,  __DEBUG_TIME_ACTIVE ? "TRUE" : "FALSE");
			// Set return status
			pNpi_ipc_buf->len = 1;
			pNpi_ipc_buf->pData[0] = NPI_LNX_SUCCESS;
		#else //__DEBUG_TIME__
			LOG_ALWAYS("NPI_Server not compiled to support time stamps\n");
			// Set return status
			pNpi_ipc_buf->len = 1;
			pNpi_ipc_buf->pData[0] = (uint8) NPI_LNX_FAILURE;
		#endif //__DEBUG_TIME__
			pNpi_ipc_buf->subSys = RPC_SYS_SRV_CTRL;
			ret = NPI_LNX_SUCCESS;
			break;

		case NPI_LNX_CMD_ID_CTRL_BIG_DEBUG_PRINT_REQ:
			__BIG_DEBUG_ACTIVE = pNpi_ipc_buf->pData[0];
			LOG_ALWAYS("__BIG_DEBUG_ACTIVE set to %d (%s). Current __APP_LOG_LEVEL=%d\n",__BIG_DEBUG_ACTIVE,  __BIG_DEBUG_ACTIVE? "TRUE" : "FALSE", __APP_LOG_LEVEL);
			// Set return status
			pNpi_ipc_buf->len = 1;
			pNpi_ipc_buf->pData[0] = NPI_LNX_SUCCESS;
			ret = NPI_LNX_SUCCESS;
			break;

		case NPI_LNX_CMD_ID_VERSION_REQ:
			// Set return status
			pNpi_ipc_buf->len = 4;
			pNpi_ipc_buf->pData[0] = NPI_LNX_SUCCESS;
			pNpi_ipc_buf->pData[1] = NPI_LNX_MAJOR_VERSION;
			pNpi_ipc_buf->pData[2] = NPI_LNX_MINOR_VERSION;
			pNpi_ipc_buf->pData[3] = NPI_LNX_REVISION;
			ret = NPI_LNX_SUCCESS;
			break;

		case NPI_LNX_CMD_ID_GET_PARAM_REQ:
			// Set return status
			switch(pNpi_ipc_buf->pData[0])
			{
				case NPI_LNX_PARAM_NB_CONNECTIONS:
					pNpi_ipc_buf->len = 3;
					pNpi_ipc_buf->pData[0] = NPI_LNX_SUCCESS;
					//Number of Active Connections
					pNpi_ipc_buf->pData[1] = activeConnections.size;
					//Max number of possible connections.
					pNpi_ipc_buf->pData[2] = NPI_SERVER_CONNECTION_QUEUE_SIZE;

					ret = NPI_LNX_SUCCESS;
					break;

				case NPI_LNX_PARAM_DEVICE_USED:
					pNpi_ipc_buf->len = 2;
					pNpi_ipc_buf->pData[0] = NPI_LNX_SUCCESS;
					//device open and used by the sever
					pNpi_ipc_buf->pData[1] = serialCfg.devIdx;

					ret = NPI_LNX_SUCCESS;
					break;

				default:
					npi_ipc_errno = NPI_LNX_ERROR_IPC_RECV_DATA_INVALID_GET_PARAM_CMD;
					ret = NPI_LNX_FAILURE;
					break;
			} // end inner switch
			break;

		case NPI_LNX_CMD_ID_RESET_DEVICE:
			if (serialCfg.devIdx == NPI_SERVER_DEVICE_INDEX_SPI)
			{
				// Perform Reset of the RNP
				(NPI_ResetSlaveFnArr[serialCfg.devIdx])();

				// Do the Hw Handshake
				(NPI_SynchSlaveFnArr[serialCfg.devIdx])();
			}
			else if (serialCfg.devIdx == NPI_SERVER_DEVICE_INDEX_I2C)
			{
				// Perform Reset of the RNP
				(NPI_ResetSlaveFnArr[serialCfg.devIdx])();
			}
			else
			{
				LOG_WARN("Resetting device upon request by client\n");
				ret = HalGpioResetSet(FALSE);
				usleep(1);
				ret = HalGpioResetSet(TRUE);
			}

			// If SRDY is shared with debug interface we need to re-init this GPIO.
			if ( (ret == NPI_LNX_SUCCESS) && serialCfg.debugSupported)
			{
				LOG_DEBUG("Resetting SRDY pin\n");
				// We may have left debug mode, so make sure DD data line is reset to input
				ret = halGpioDDSetDirection(0);
			}
			break;

		case NPI_LNX_CMD_ID_DISCONNECT_DEVICE:
			LOG_DEBUG("Trying to disconnect device %d\n", serialCfg.devIdx);
			(NPI_CloseDeviceFnArr[serialCfg.devIdx])();
			LOG_DEBUG("Preparing return message after disconnecting device %d\n", serialCfg.devIdx);
			pNpi_ipc_buf->len = 1;
			pNpi_ipc_buf->pData[0] = NPI_LNX_SUCCESS;
			break;

		case NPI_LNX_CMD_ID_CONNECT_DEVICE:
			LOG_DEBUG("Trying to connect to device %d, %s\n", serialCfg.devIdx, serialCfg.devPath);
			switch(serialCfg.devIdx)
			{
				case NPI_SERVER_DEVICE_INDEX_UART_USB:
					// Initialization of UART for USB is the same as for physical UART.
					// Except for Reset GPIO
				case NPI_SERVER_DEVICE_INDEX_UART:
				#if (defined NPI_UART) && (NPI_UART == TRUE)
					ret = (NPI_OpenDeviceFnArr[NPI_SERVER_DEVICE_INDEX_UART])(serialCfg.devPath, (npiUartCfg_t *)&serialCfg.serial.npiUartCfg);

					// Now configure reset GPIO for physical UART
					if (serialCfg.devIdx == NPI_SERVER_DEVICE_INDEX_UART)
					{
						if ( NPI_LNX_FAILURE == (ret = HalGpioResetInit((halGpioCfg_t *)&serialCfg.gpioCfg[2])))
						{
							return ret;
						}
					}
				#endif
					break;

				case NPI_SERVER_DEVICE_INDEX_SPI:
				#if (defined NPI_SPI) && (NPI_SPI == TRUE)
					// Now open device for processing
					ret = (NPI_OpenDeviceFnArr[serialCfg.devIdx])(serialCfg.devPath, (npiSpiCfg_t *) &serialCfg.serial.npiSpiCfg);

					// Must also reset and synch

					// Perform Reset of the RNP
					(NPI_ResetSlaveFnArr[serialCfg.devIdx])();

					// Do the Hw Handshake
					(NPI_SynchSlaveFnArr[serialCfg.devIdx])();

//					// Since SPI does not indicate reset to host we should notify here
//					// but there's no unified way of doing it for RNP and ZNP...
//					// For RemoTI we can send RTI_ResetInd(). This message should just
//					// be discarded by anything but RNP, so should be safe.
//
//					// We only need space for the header; there is no payload.
//					resetBuf = malloc(sizeof(*resetBuf) - sizeof(resetBuf->pData));
//					resetBuf->len = 0;
//					resetBuf->subSys = 0x4A;
//					resetBuf->cmdId = 0x0D;
//					NPI_LNX_IPC_SendData(resetBuf, -1);
//					free(resetBuf);
				#endif
					break;

				case NPI_SERVER_DEVICE_INDEX_I2C:
				#if (defined NPI_I2C) && (NPI_I2C == TRUE)
					// Open the Device and perform a reset
					ret = (NPI_OpenDeviceFnArr[serialCfg.devIdx])(serialCfg.devPath, (npiI2cCfg_t *) &serialCfg.serial.npiI2cCfg);
				#endif
					break;

				default:
					ret = NPI_LNX_FAILURE;
					break;
			} // end inner switch

			LOG_DEBUG("Preparing return message after connecting to device %d (ret == 0x%.2X, npi_ipc_errno == 0x%.2X)\n",
					serialCfg.devIdx, ret, npi_ipc_errno);
			pNpi_ipc_buf->len = 1;
			pNpi_ipc_buf->pData[0] = ret;
			break; // End case NPI_LNX_CMD_ID_CONNECT_DEVICE

		default:
			npi_ipc_errno = NPI_LNX_ERROR_IPC_RECV_DATA_INVALID_SREQ;
			ret = NPI_LNX_FAILURE;
			break;
	} // end outer switch
	return ret;
}

static int setupSocket(npiSerialCfg_t *serialCfg)
{
	int socketInt = -1;

#ifdef NPI_UNIX
	int len;
	struct sockaddr_un local, their_addr;
#else
	int status;
	struct addrinfo hints;

	memset(&hints, 0, sizeof(hints));
	hints.ai_family = AF_UNSPEC;
	hints.ai_socktype = SOCK_STREAM;
	hints.ai_flags = AI_PASSIVE;

	LOG_INFO("Listen port: %s\n", serialCfg->port);

#if FORCE_LOCALHOST_ONLY
	(void)status;
	getaddrinfo("localhost", serialCfg->port, &hints, &servinfo);
#else
	if ((status = getaddrinfo(NULL, serialCfg->port, &hints, &servinfo)) != 0)
	{
		LOG_ERROR("getaddrinfo error: %s\n", gai_strerror(status));
		//                port = NPI_PORT;
		strncpy(serialCfg->port, NPI_PORT, sizeof(serialCfg->port)-1);
		LOG_INFO("Trying default port: %s instead\n", serialCfg->port);
		if ((status = getaddrinfo(NULL, serialCfg->port, &hints, &servinfo)) != 0)
		{
			LOG_ERROR("getaddrinfo error: %s\n", gai_strerror(status));
			npi_ipc_errno = NPI_LNX_ERROR_IPC_SOCKET_GET_ADDRESS_INFO;
			NPI_LNX_IPC_Exit(NPI_LNX_FAILURE, TRUE);
		}
	}

	LOG_INFO("Following IP addresses are available:\n");
	{
		struct ifaddrs * ifAddrStruct=NULL;
		struct ifaddrs * ifa=NULL;
		void * tmpAddrPtr=NULL;

		getifaddrs(&ifAddrStruct);

		for (ifa = ifAddrStruct; ifa != NULL; ifa = ifa->ifa_next)
		{
			if (ifa ->ifa_addr != NULL)
			{
				if (ifa ->ifa_addr->sa_family==AF_INET)
				{ // check it is IP4
					// is a valid IP4 Address
					tmpAddrPtr=&((struct sockaddr_in *)ifa->ifa_addr)->sin_addr;
					char addressBuffer[INET_ADDRSTRLEN];
					inet_ntop(AF_INET, tmpAddrPtr, addressBuffer, INET_ADDRSTRLEN);
					LOG_INFO(" IPv4: interface: %s\t IP Address %s\n", ifa->ifa_name, addressBuffer);
				}
				else if (ifa->ifa_addr->sa_family==AF_INET6)
				{ // check it is IP6
					// is a valid IP6 Address
					tmpAddrPtr=&((struct sockaddr_in6 *)ifa->ifa_addr)->sin6_addr;
					char addressBuffer[INET6_ADDRSTRLEN];
					inet_ntop(AF_INET6, tmpAddrPtr, addressBuffer, INET6_ADDRSTRLEN);
					LOG_INFO(" IPv6: interface: %s\t IP Address %s\n", ifa->ifa_name, addressBuffer);
				}
			}
		}
		if (ifAddrStruct!=NULL) freeifaddrs(ifAddrStruct);
	}

#endif //FORCE_LOCALHOST_ONLY

	LOG_INFO("The socket will listen on the following IP addresses:\n");

	struct addrinfo *p;
	char ipstr[INET6_ADDRSTRLEN];
	for (p = servinfo; p != NULL; p = p->ai_next)
	{
		void *addr;
		char const *ipver;

		// get the pointer to the address itself,
		// different fields in IPv4 and IPv6:
		if (p->ai_family == AF_INET)
		{ // IPv4
			struct sockaddr_in *ipv4 = (struct sockaddr_in *) p->ai_addr;
			addr = &(ipv4->sin_addr);
			ipver = "IPv4";
		} else
		{ // IPv6
			struct sockaddr_in6 *ipv6 = (struct sockaddr_in6 *) p->ai_addr;
			addr = &(ipv6->sin6_addr);
			ipver = "IPv6";
		}

		// convert the IP to a string and print it:
		inet_ntop(p->ai_family, addr, ipstr, sizeof ipstr);
		LOG_INFO("  %s: %s\n", ipver, ipstr);
	}
	LOG_DEBUG("0.0.0.0 means it will listen to all available IP address\n\n");

#endif //NPI_UNIX

#ifdef NPI_UNIX
	// Create the socket
	socketInt = socket(AF_UNIX, SOCK_STREAM, 0);

	// Bind socket to a Unix domain address
	local.sun_family = AF_UNIX;
	strcpy(local.sun_path, "echo_socket");
	unlink(local.sun_path);
	len = strlen(local.sun_path) + sizeof(local.sun_family);
	if (bind(socketInt, (struct sockaddr *)&local, len) == -1)
	{
		perror("bind");
		writeToNpiLnxLog("Port is probably already in use, please select an available port\n");
		LOG_ERROR("Port is probably already in use, please select an available port\n");
		npi_ipc_errno = NPI_LNX_ERROR_IPC_SOCKET_BIND;
		NPI_LNX_IPC_Exit(NPI_LNX_FAILURE, TRUE);
	}

#else
	socketInt = socket(servinfo->ai_family, servinfo->ai_socktype,
			servinfo->ai_protocol);

	int yes = 1;
	// avoid "Address already in use" error message
	if (setsockopt(socketInt, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(int))
			== -1)
	{
		perror("setsockopt");
		npi_ipc_errno = NPI_LNX_ERROR_IPC_SOCKET_SET_REUSE_ADDRESS;
		NPI_LNX_IPC_Exit(NPI_LNX_FAILURE, TRUE);
	}

	// Bind socket
	if (bind(socketInt, servinfo->ai_addr, servinfo->ai_addrlen) == -1)
	{
		perror("bind");
		npi_ipc_errno = NPI_LNX_ERROR_IPC_SOCKET_BIND;
		NPI_LNX_IPC_Exit(NPI_LNX_FAILURE, TRUE);
	}

#endif

	// Listen, allow 20 connections in the queue
	if (listen(socketInt, NPI_SERVER_CONNECTION_QUEUE_SIZE) == -1)
	{
		perror("listen");
		npi_ipc_errno = NPI_LNX_ERROR_IPC_SOCKET_LISTEN;
		NPI_LNX_IPC_Exit(NPI_LNX_FAILURE, TRUE);
	}

	return socketInt;
}

static int configureDebugInterface()
{
	int ret = NPI_LNX_SUCCESS;

	// Configure DD
	if ( ret == NPI_LNX_SUCCESS )
	{
		ret = HalGpioDDInit((halGpioCfg_t *)&serialCfg.gpioCfg[3]);
	}

	// Configure DC
	if ( ret == NPI_LNX_SUCCESS )
	{
		ret = HalGpioDCInit((halGpioCfg_t *)&serialCfg.gpioCfg[4]);
	}
	else
		LOG_ERROR("2 here\n");

	// Now enter debug mode.
	if ( ret == NPI_LNX_SUCCESS )
	{
		ret = Hal_debug_init();
	}
	else
		LOG_ERROR("3 here\n");

	if ( ret == NPI_LNX_SUCCESS )
	{
		// Now get chip ID.
		uint8 chipId = 0;
		ret = Hal_read_chip_id(&chipId);
		if ( ret != NPI_LNX_SUCCESS )
		{
			if (npi_ipc_errno == NPI_LNX_ERROR_HAL_DBG_IFC_WAIT_DUP_READY)
			{
				// Device did not respond, it may be that it's not in debug mode anymore.
				LOG_ERROR("Could not get chip ID, device not in debug mode as it failed to respond\n");						// This error should not be considered critical at this stage.
				npi_ipc_errno = NPI_LNX_SUCCESS;
				ret = NPI_LNX_SUCCESS;
			}
			else
			{
				LOG_ERROR("Could not get chip ID, an error occurred\n");
			}
		}
		else
		{
			LOG_INFO("Chip ID: 0x%.2X\n", chipId);
		}
	}

	return ret;
}

/**************************************************************************************************
 **************************************************************************************************/

