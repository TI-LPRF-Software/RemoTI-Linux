 /**************************************************************************************************
  Filename:       npi_ipc_client.c
  Revised:        $Date: 2012-03-21 17:37:33 -0700 (Wed, 21 Mar 2012) $
  Revision:       $Revision: 246 $

  Description:    This file contains Linux platform specific RemoTI (RTI) API
                  Surrogate implementation

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
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <pthread.h>
#include <poll.h>
#include <sys/time.h>
#include <unistd.h>
#include <syscall.h>

#ifndef NPI_UNIX
#include <netdb.h>
#include <arpa/inet.h>
#endif

/* NPI includes */
#include "npi_lnx.h"

#include "npi_lnx_error.h"
#include "tiLogging.h"

#ifdef NPI_SYS
#include "npi_sys.h"
#endif

#ifdef NPI_BOOT
#include "npi_mac.h"
#endif

#include "npi_rti.h"
#include "npi_lnx_ipc_rpc.h"

#ifdef NPI_BOOT
#include "npi_boot.h"
#endif

#ifdef NPI_DBG
#include "npi_dbg.h"
#endif

#ifdef NPI_PERIPHERALS
#include "npi_peripherals.h"
#endif

#ifdef NPI_NFC
#include "npi_nfc.h"
#endif

#ifdef NPI_XR_TESTIF
#include "npi_xrTestif.h"
#endif

#include "npi_ipc_client.h"

#define NPI_PORT "2533"

#define msg_memcpy(src, dst, len)	memcpy(src, dst, len)

/**************************************************************************************************
 *                                        Externals
 **************************************************************************************************/

/**************************************************************************************************
 *                                           Constant
 **************************************************************************************************/

#define NPI_MSG_AREQ_READY			0x0000
#define NPI_MSG_AREQ_BUSY			0x0001

#define NPI_IPC_BUF_SIZE			(2 * (sizeof(npiMsgData_t)))

typedef int (*npiProcessMsg_t)(npiMsgData_t *pBuf);

//if Value, max number of RPC command type change, this table needs to be updated.
npiProcessMsg_t NpiAsyncMsgCbackParserTable[] =
{
		  NULL,   					//RPC_SYS_RES0        0
#ifdef NPI_SYS
		  SYS_AsynchMsgCback,   	//RPC_SYS_SYS         1
#else
		NULL,
#endif
#ifdef NPI_MAC
		  MAC_AsynchMsgCback,   	//RPC_SYS_MAC         2
#else
		NULL,
#endif
		  NULL,   					//RPC_SYS_NWK         3
		  NULL,   					//RPC_SYS_AF          4
		  NULL,   					//RPC_SYS_ZDO         5
		  NULL,   					//RPC_SYS_SAPI        6    // Simple API
		  NULL,   					//RPC_SYS_UTIL        7
		  NULL,   					//RPC_SYS_DBG         8
		  NULL,   					//RPC_SYS_APP         9
#ifdef NPI_RTI
		  RTI_AsynchMsgCback,		//RPC_SYS_RCAF        10   // Remote Control Application Framework
#else
		NULL,
#endif
		  NULL,   					//RPC_SYS_RCN         11   // Remote Control Network Layer
		  NULL,   					//RPC_SYS_RCN_CLIENT  12   // Remote Control Network Layer Client
#ifdef NPI_BOOT
		  BOOT_AsynchMsgCback,		//RPC_SYS_BOOT        13   // Serial Bootloader
#else
		NULL,
#endif
#ifdef NPI_XR_TESTIF
		XR_TESTIF_AsynchMsgCback,	// XR Test Interface  14
#else //NPI_XR_TESTIF
		NULL,   					//RPC_SYS_ZIPTEST	  14   // Used by ZIP
#endif //NPI_XR_TESTIF
#ifdef NPI_DBG
		DBG_AsynchMsgCback,   		//RPC_SYS_DEBUG	      15   // Debug Interface for Flash Programming
#else
		NULL,
#endif
#ifdef NPI_PERIPHERALS
		  PER_AsynchMsgCback,		// RPC_SYS_PERIPHERALS	  16   // Sub system for all peripherals
#else
		  NULL,
#endif
#ifdef NPI_NFC
		  NFC_AsynchMsgCback,		// RPC_SYS_NFC	  	  17   // Interface for NFC
#else
		  NULL,
#endif
		  NULL   					//RPC_SYS_MAX         18   // Maximum value, must be last
};

#define NAME_ELEMENT(element) [element] = #element


const char * const RpcCmdType_list[RPC_SYS_MAX + 1] = { //18 Cmd Type
		[0 ... RPC_SYS_MAX] = NULL,
		NAME_ELEMENT(RPC_SYS_RES0),
		NAME_ELEMENT(RPC_SYS_SYS),
		NAME_ELEMENT(RPC_SYS_MAC),
		NAME_ELEMENT(RPC_SYS_NWK),
		NAME_ELEMENT(RPC_SYS_AF),
		NAME_ELEMENT(RPC_SYS_ZDO),
		NAME_ELEMENT(RPC_SYS_SAPI),
		NAME_ELEMENT(RPC_SYS_UTIL),
		NAME_ELEMENT(RPC_SYS_DBG),
		NAME_ELEMENT(RPC_SYS_APP),
		NAME_ELEMENT(RPC_SYS_RCAF),
		NAME_ELEMENT(RPC_SYS_RCN),
		NAME_ELEMENT(RPC_SYS_RCN_CLIENT),
		NAME_ELEMENT(RPC_SYS_BOOT),
		NAME_ELEMENT(RPC_SYS_DEBUG),
		NAME_ELEMENT(RPC_SYS_PERIPHERALS),
		NAME_ELEMENT(RPC_SYS_NFC),
		NAME_ELEMENT(RPC_SYS_MAX)
};

/**************************************************************************************************
 *                                        Type definitions
 **************************************************************************************************/

struct linkedAreqMsg
{
	npiMsgData_t message;
	struct linkedAreqMsg *nextMessage;
};

typedef struct linkedAreqMsg areqMsg;


/**************************************************************************************************
 *                                        Global Variables
 **************************************************************************************************/

/**************************************************************************************************
 *                                        Local Variables
 **************************************************************************************************/

// Client socket handle
int sNPIconnected;
// Client data transmission buffers
char npi_ipc_buf[2][NPI_IPC_BUF_SIZE];
// Client data AREQ received buffer
areqMsg *npi_ipc_areq_rec_buf;
// Client data AREQ processing buffer
areqMsg *npi_ipc_areq_proc_buf;

// Message count to keep track of incoming and processed messages
static int messageCount = 0;

// Client data SRSP reception buffer
char npi_ipc_srsp_buf[(sizeof(npiMsgData_t))];

// variable to store the number of received synchronous response bytes
static int numOfReceievedSRSPbytes = 0;

static pthread_t NPIThreadId;
static void *npi_ipc_readThreadFunc (void *ptr);
static void *npi_ipc_handleThreadFunc (void *ptr);

// Mutex to handle synchronous response
pthread_mutex_t npiLnxClientSREQmutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t npiLnxClientSREQSerializationMutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t npiLnxClientAREQmutex = PTHREAD_MUTEX_INITIALIZER;
// conditional variable to notify Synchronous response
static pthread_cond_t npiLnxClientSREQcond;
// conditional variable to notify that the AREQ is handled
static pthread_cond_t npiLnxClientAREQcond;

// mutex variable, AREQ message processing status
static uint8 areqMsgProcessStatus = NPI_MSG_AREQ_READY;

#ifndef NPI_UNIX
struct addrinfo *resAddr;
#endif

// current state
#if defined _WIN32 || defined unix
// windows does not require RTI_Init() call
static uint8 npiState = NPI_STATE_READY;
#else
static uint8 npiState;
#endif // _WIN32

/**************************************************************************************************
 *                                     Local Function Prototypes
 **************************************************************************************************/

static void npi_ipc_initsyncres(void);
static void npi_ipc_delsyncres(void);

/**************************************************************************************************
 *
 * @fn          NPI_ClientInit
 *
 * @brief       This function initializes RTI Surrogate
 *
 * input parameters
 *
 * @param       ipAddress - path to the NPI Server Socket
 *
 * output parameters
 *
 * None.
 *
 * @return      TRUE if the surrogate module started off successfully.
 *              FALSE, otherwise.
 *
 **************************************************************************************************/

int NPI_ClientInit(const char *devPath)
{
	int res = NPI_LNX_SUCCESS;
	int connectAttemptsLeft = NPI_SERVER_CONNECT_ATTEMPTS_MAX;
#ifndef NPI_UNIX
	const char *ipAddress = "", *port = "";
	int i = 0;
	char *pStr;
#else
	const char *unixPath = "";
#endif //NPI_UNIX

	char strTmp[128];
	strncpy(strTmp, devPath, 128);
	// use strtok to split string and find IP address and port;
	// the format is = IPaddress:port
	// Get first token
#ifndef NPI_UNIX
	pStr = strtok (strTmp, ":");
	while ( pStr != NULL)
	{
		if (i == 0)
		{
			// First part is the IP address
			ipAddress = pStr;
		}
		else if (i == 1)
		{
			// Second part is the port
			port = pStr;
		}
		i++;
		if (i > 2)
			break;
		// Now get next token
		pStr = strtok (NULL, " ,:;-|");
	}
#else
	unixPath = strTmp;
#endif //NPI_UNIX

	/**********************************************************************
	 * Initiate synchronization resources
	 */
	npi_ipc_initsyncres();

	/**********************************************************************
	 * Connect to the NPI server
	 **********************************************************************/

#ifdef NPI_UNIX
    int len;
    struct sockaddr_un remote;

    if ((sNPIconnected = socket(AF_UNIX, SOCK_STREAM, 0)) == -1)
    {
        LOG_FATAL("%s(): Unable to create a socket! Exiting app!\n", __FUNCTION__);
        exit(1);
    }
#else
    struct addrinfo hints;

    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;

//    ipAddress = "192.168.128.133";
//    if ((res = getaddrinfo(NULL, ipAddress, &hints, &resAddr)) != 0)
	if (port == NULL)
	{
		// Fall back to default if port was not found in the configuration file
		LOG_WARN("%s() Warning! Port not sent to NPI. Will use default port: %s", __FUNCTION__, NPI_PORT);

	    if ((res = getaddrinfo(ipAddress, NPI_PORT, &hints, &resAddr)) != 0)
	    {
	    	LOG_ERROR("getaddrinfo: %s\n", gai_strerror(res));
	    	res = NPI_LNX_ERROR_IPC_SOCKET_GET_ADDRESS_INFO;
	    }
	}
	else
	{
	    LOG_DEBUG("[NPI Client] Port: %s\n\n", port);
	    if ((res = getaddrinfo(ipAddress, port, &hints, &resAddr)) != 0)
	    {
	    	LOG_ERROR("%s(): getaddrinfo: %s\n", __FUNCTION__, gai_strerror(res));
	    	res = NPI_LNX_ERROR_IPC_SOCKET_GET_ADDRESS_INFO;
	    }
	}

    LOG_DEBUG("[NPI Client] IP addresses for %s:\n", ipAddress);

    struct addrinfo *p;
    char ipstr[INET6_ADDRSTRLEN];
    for(p = resAddr;p != NULL; p = p->ai_next)
    {
        void *addr;
        char const *ipver;

        // get the pointer to the address itself,
        // different fields in IPv4 and IPv6:
        if (p->ai_family == AF_INET) { // IPv4
            struct sockaddr_in *ipv4 = (struct sockaddr_in *)p->ai_addr;
            addr = &(ipv4->sin_addr);
            ipver = "IPv4";
        } else { // IPv6
            struct sockaddr_in6 *ipv6 = (struct sockaddr_in6 *)p->ai_addr;
            addr = &(ipv6->sin6_addr);
            ipver = "IPv6";
        }

        // convert the IP to a string and print it:
        inet_ntop(p->ai_family, addr, ipstr, sizeof ipstr);
        LOG_DEBUG("  %s: %s\n", ipver, ipstr);
    }


    if ((sNPIconnected = socket(resAddr->ai_family, resAddr->ai_socktype, resAddr->ai_protocol)) == -1)
    {
        LOG_FATAL("%s(): Unable to create a socket! Exiting app!\n", __FUNCTION__);
        res = NPI_LNX_ERROR_IPC_SOCKET_CREATE;
        exit(1);
    }
#endif

    LOG_INFO("[NPI Client] Connecting to NPI Server...\n");

    do
    {
       LOG_DEBUG("%s(): Trying to connect...\n", __FUNCTION__);

#ifdef NPI_UNIX
        remote.sun_family = AF_UNIX;
        strcpy(remote.sun_path, unixPath);
        len = strlen(remote.sun_path) + sizeof(remote.sun_family);
        if (connect(sNPIconnected, (struct sockaddr *)&remote, len) == -1)
        {
            res = NPI_LNX_ERROR_IPC_SOCKET_CONNECT;;
            LOG_DEBUG("[NPI Client] %s(): Not connected. res = 0x%.2X\n", __FUNCTION__, res);
        }
#else
        if (connect(sNPIconnected, resAddr->ai_addr, resAddr->ai_addrlen) == -1)
        {
            res = NPI_LNX_ERROR_IPC_SOCKET_CONNECT;;
            LOG_DEBUG("[NPI Client] %s(): Not connected. res = 0x%.2X\n", __FUNCTION__, res);
        }
#endif
        else
        {
            res =  NPI_LNX_SUCCESS;
        }

        if (res != NPI_LNX_SUCCESS)
        {
            if (--connectAttemptsLeft)
            {
                LOG_WARN("[NPI Client] Unable to connect to NPI server.  Will retry again in %d second%s.  (Will give up after %d more attempts.)\n",
                     NPI_SERVER_CONNECT_RETRY_SECONDS, NPI_SERVER_CONNECT_RETRY_SECONDS==1 ? "" : "s", connectAttemptsLeft);
                sleep(NPI_SERVER_CONNECT_RETRY_SECONDS);
            }
            else
            {
                LOG_ERROR("[NPI Client] Unable to connect to NPI server after %d seconds.  Giving up!\n", NPI_SERVER_CONNECT_RETRY_SECONDS * NPI_SERVER_CONNECT_ATTEMPTS_MAX);
            }
        }

    } while (res != NPI_LNX_SUCCESS && connectAttemptsLeft > 0);

    if (res == NPI_LNX_SUCCESS)
    {
       LOG_INFO("[NPI Client] Connected to NPI Server.\n");
    }


	int no = 0;
    if (res == NPI_LNX_SUCCESS)
    {
    	// allow out-of-band data
    	if (setsockopt(sNPIconnected, SOL_SOCKET, SO_OOBINLINE, &no, sizeof(int)) == -1)
    	{
    		LOG_ERROR("[NPI Client] setsockopt");
    		LOG_ERROR("[NPI Client] %s(): Failed to set socket options!\n", __FUNCTION__);
    		res = NPI_LNX_ERROR_IPC_SOCKET_SET_SOCKET_OPTIONS;
    	}
    }

	/**********************************************************************
	 * Create thread which can read new messages from the NPI server
	 **********************************************************************/

    if (res == NPI_LNX_SUCCESS)
    {
    	if (pthread_create(&NPIThreadId, NULL, npi_ipc_readThreadFunc, NULL))
    	{
    		// thread creation failed
    		LOG_ERROR("[NPI Client] %s(): Failed to create NPI IPC Client read thread\n", __FUNCTION__);
            res = NPI_LNX_ERROR_IPC_THREAD_CREATION_FAILED;
    	}
    }

	/**********************************************************************
	 * Create thread which can handle new messages from the NPI server
	 **********************************************************************/

    if (res == NPI_LNX_SUCCESS)
    {
    	if (pthread_create(&NPIThreadId, NULL, npi_ipc_handleThreadFunc, NULL))
    	{
    		// thread creation failed
    		LOG_ERROR("[NPI Client] Failed to create NPI IPC Client handle thread\n");
            res = NPI_LNX_ERROR_IPC_THREAD_CREATION_FAILED;
    	}
    }

    if (res == NPI_LNX_SUCCESS)
    {
    	uint8 version[3];
    	uint8 param[2];
		//Read Software Version.
		NPI_ReadVersionReq(version);
		LOG_INFO("[NPI Client] Connected to Server v%d.%d.%d\n", version[0], version[1], version[2]);

		//Read Number of Active Connection Version.
		NPI_ReadParamReq(NPI_LNX_PARAM_NB_CONNECTIONS,2, param);
		LOG_INFO("[NPI Client] %d active connection , out of %d maximum connections\n", param[0], param[1]);
		//Check Which interface is used.
		NPI_ReadParamReq(NPI_LNX_PARAM_DEVICE_USED, 1 , param);
		LOG_DEBUG("[NPI Client] Interface used y server: %d (0 = UART, 1 = SPI, 2 = I2C, 3 = HID, 4 = ZID)\n", param[0]);

	}

	return res;
}

/**************************************************************************************************
 *
 * @fn          npi_ipc_handleThreadFunc
 *
 * @brief       This function initializes RTI Surrogate
 *
 * input parameters
 *
 * @param       ipAddress - path to the NPI Server Socket
 *
 * output parameters
 *
 * None.
 *
 * @return      TRUE if the surrogate module started off successfully.
 *              FALSE, otherwise.
 *
 **************************************************************************************************/
static void *npi_ipc_handleThreadFunc (void *ptr)
{
	int done = 0, tryLockFirstTimeOnly = 0;

	// Handle message from socket
	do {

		if (tryLockFirstTimeOnly == 0)
		{
			// Lock mutex
			LOG_TRACE("[NPI Client HANDLE][MUTEX] Lock AREQ Mutex (Handle)\n");
			int mutexRet = 0, writeOnce = 0;
			while ( (mutexRet = pthread_mutex_trylock(&npiLnxClientAREQmutex)) == EBUSY)
			{
				if (writeOnce == 0)
				{
					LOG_TRACE("[NPI Client HANDLE][MUTEX] AREQ Mutex (Handle) busy");
					fflush(LOG_DESTINATION_FP);
					writeOnce++;
				}
				else
				{
					writeOnce++;
					if ( (writeOnce % 1000) == 0)
					{
						LOG_TRACE(".");
					}
					if (writeOnce > 0xEFFFFFF0)
						writeOnce = 1;
					fflush(LOG_DESTINATION_FP);
				}
			}
			LOG_TRACE("[NPI Client HANDLE][MUTEX] AREQ Lock (Handle) status: %d\n", mutexRet);
			tryLockFirstTimeOnly = 1;
		}

		// Conditional wait for the response handled in the AREQ handling thread,
		LOG_TRACE("[NPI Client HANDLE][MUTEX] Wait for AREQ Cond (Handle) signal... (areqMsgProcessStatus = %d), effectively releasing lock\n", areqMsgProcessStatus);
		pthread_cond_wait(&npiLnxClientAREQcond, &npiLnxClientAREQmutex);
		LOG_TRACE("[NPI Client HANDLE][MUTEX] AREQ (Handle) has lock\n");

		// Walk through all received AREQ messages before releasing MUTEX
		areqMsg *searchList = npi_ipc_areq_proc_buf, *clearList;
		while (searchList != NULL)
		{
			LOG_TRACE("[NPI Client HANDLE][DBG] Processing \t@ %p next \t@ %p\n",
					(void *)searchList,
					(void *)(searchList->nextMessage));

			// Must remove command type before calling NPI_AsynchMsgCback
			searchList->message.subSys &= ~(RPC_CMD_TYPE_MASK);

			LOG_TRACE("[NPI Client HANDLE][MUTEX] AREQ Calling NPI_AsynchMsgCback (Handle)...\n");

			NPI_AsynchMsgCback((npiMsgData_t *)&(searchList->message));

			LOG_TRACE("[NPI Client HANDLE][MUTEX] AREQ (Handle) (message @ %p)...\n", (void *)searchList);

			clearList = searchList;
			// Set search list to next message
			searchList = searchList->nextMessage;
			// Free processed buffer
			if (clearList == NULL)
			{
				// Impossible error, must abort
				done = 1;
				LOG_ERROR("[NPI Client HANDLE][ERR] clearList buffer was already free\n");
				break;
			}
			else
			{
				messageCount--;
				LOG_TRACE("[NPI Client HANDLE][DBG] Clearing \t\t@ %p (processed %d messages)...\n",
						(void *)clearList,
						messageCount);
				memset(clearList, 0, sizeof(areqMsg));
				free(clearList);
			}
		}
		LOG_TRACE("[NPI Client HANDLE][MUTEX] AREQ Signal message(s) handled (Handle) (processed %d messages)...\n", messageCount);
		// Signal to the read thread that we're ready for more
		//			pthread_cond_signal(&npiLnxClientAREQcond);
		areqMsgProcessStatus = NPI_MSG_AREQ_READY;

		LOG_DEBUG("[NPI Client HANDLE][DBG] Finished processing (processed %d messages)...\n",
				messageCount);

	} while (!done);

	LOG_INFO("[NPI Client HANDLE] %s Exiting\n", __FUNCTION__);
	return ptr;
}
/**************************************************************************************************
 *
 * @fn          npi_ipc_handleThreadFunc
 *
 * @brief       This function initializes RTI Surrogate
 *
 * input parameters
 *
 * @param       ipAddress - path to the NPI Server Socket
 *
 * output parameters
 *
 * None.
 *
 * @return      TRUE if the surrogate module started off successfully.
 *              FALSE, otherwise.
 *
 **************************************************************************************************/
static void *npi_ipc_readThreadFunc (void *ptr)
{
	int done = 0, n = 0;

	int mutexRet = 0;
#ifdef __BIG_DEBUG__
	static int writeOnceAREQMutexTry = 0, writeOnceAREQReady = 0;
#endif //__BIG_DEBUG__

	/* thread loop */

	struct pollfd ufds[1];
	int pollRet;
	ufds[0].fd = sNPIconnected;
	ufds[0].events = POLLIN | POLLPRI;

	// Read from socket
	do {
		// To be able to continue processing previous AREQ we need to perform a non-blocking read of the socket,
		// this is solved by using poll() to see if there are bytes to read.

		if (npi_ipc_areq_rec_buf != NULL)
		{
#ifdef __DEBUG_TIME__
			LOG_TRACE("[NPI Client READ] Read thread 1ms timeout \n");
#endif //__DEBUG_TIME__
			// In case there are messages received yet to be processed allow processing by timing out after 1ms.
			pollRet = poll((struct pollfd*)&ufds, 1, 1);
		}
		else
		{
			// In case there are no messages received to be processed wait forever.
			LOG_TRACE("[NPI Client READ] Read thread Wait forever\n");
			pollRet = poll((struct pollfd*)&ufds, 1, -1);
		}

		if (pollRet == -1)
		{
			// Error occured in poll()
			// We simply want to continue on EINTR since we don't have a signal handler anyway
			if (errno != EINTR)
			{
				LOG_ERROR("[NPI Client] poll");
			}
		}
		else if (pollRet == 0)
		{
			// Timeout, could still be AREQ to process
		}
		else
		{
			if (ufds[0].revents & POLLIN) {
				n = recv(sNPIconnected,
						npi_ipc_buf[0],
						RPC_FRAME_HDR_SZ,
						0); // normal data
			}
			if (ufds[0].revents & POLLPRI) {
				n = recv(sNPIconnected,
						npi_ipc_buf[0],
						RPC_FRAME_HDR_SZ,
						MSG_OOB); // out-of-band data
			}
			if (n <= 0)
			{
				if (n < 0)
					LOG_ERROR("[NPI Client] recv");
				done = 1;
				LOG_ERROR("Error: RECEIVED %d bytes.. other side might have closed connection\n", n);
			}
			else if  (n != RPC_FRAME_HDR_SZ)
			{
				// Invalid length received
				LOG_ERROR("[NPI Client READ] Received invalid number of bytes %d, expected %d. Errno: %d\n",
						n, RPC_FRAME_HDR_SZ, errno);
			}
			else
			{
				// We have received the header, now read out length bytes and process it,
				// if there are bytes to receive.
				if (((npiMsgData_t *)&(npi_ipc_buf[0][0]))->len > 0)
				{
					n = recv(sNPIconnected,
							(uint8*)&(npi_ipc_buf[0][RPC_FRAME_HDR_SZ]),
							((npiMsgData_t *)&(npi_ipc_buf[0][0]))->len,
							0);
				}
				else
				{
					// There are no payload bytes; which is also valid.
					n = ((npiMsgData_t *)&(npi_ipc_buf[0][0]))->len;
				}

				if (n != ((npiMsgData_t *)&(npi_ipc_buf[0][0]))->len)
				{
					// Invalid length received
					LOG_ERROR("[NPI Client READ] Received invalid number of bytes %d, expected %d. Errno: %d\n",
							n, ((npiMsgData_t *)&(npi_ipc_buf[0][0]))->len, errno);
				}
				else
				{
					int i;
				    char str[256];
				    uint8 charWritten;
				    charWritten = snprintf(str, sizeof(str), "[NPI Client READ] Received %d bytes,\t subSys 0x%.2X, cmdId 0x%.2X, pData:\t",
											((npiMsgData_t *)&(npi_ipc_buf[0][0]))->len,
											((npiMsgData_t *)&(npi_ipc_buf[0][0]))->subSys,
											((npiMsgData_t *)&(npi_ipc_buf[0][0]))->cmdId);
				    charWritten += snprintf(&str[charWritten], sizeof(str) - charWritten, "\t");
					for (i = 3; i < (n + RPC_FRAME_HDR_SZ); i++)
				    {
				       charWritten += snprintf(&str[charWritten], sizeof(str) - charWritten, " 0x%.2X", (uint8)npi_ipc_buf[0][i]);
				    }
				    charWritten += snprintf(&str[charWritten], sizeof(str) - charWritten, "\n");
				    LOG_DEBUG("%s", str);

					if ( ( (uint8)(((npiMsgData_t *)&(npi_ipc_buf[0][0]))->subSys) & (uint8)RPC_CMD_TYPE_MASK) == RPC_CMD_SRSP )
					{
						numOfReceievedSRSPbytes = ((npiMsgData_t *)&(npi_ipc_buf[0][0]))->len + RPC_FRAME_HDR_SZ;

						// Copy buffer so that we don't clear it before it's handled.
						memcpy(npi_ipc_srsp_buf,
								(uint8*)&(npi_ipc_buf[0][0]),
								numOfReceievedSRSPbytes);

						// and signal the synchronous reception
						LOG_DEBUG("[NPI Client READ][MUTEX] SRSP Cond signal set\n");
						LOG_TRACE("[NPI Client READ] Client Read SRSP: (len %d)\n", numOfReceievedSRSPbytes);
						fflush(LOG_DESTINATION_FP);
						// Get mutex first
						pthread_mutex_lock(&npiLnxClientSREQmutex);   // Make sure the receiver has begin the wait on the cond so it doesn't miss the signal.
						pthread_cond_signal(&npiLnxClientSREQcond);   // Signal the receiver, unblocking it on the cond.
						pthread_mutex_unlock(&npiLnxClientSREQmutex); // Release the mutex so the receiver can re-acquire it.
					}
					else if ( ( (uint8)(((npiMsgData_t *)&(npi_ipc_buf[0][0]))->subSys) & (uint8)RPC_CMD_TYPE_MASK) == RPC_CMD_AREQ )
					{
						LOG_DEBUG("[NPI Client READ] RPC_CMD_AREQ cmdId: 0x%.2X\n", ((npiMsgData_t *)&(npi_ipc_buf[0][0]))->cmdId);
						// Verify the size of the incoming message before passing it
						if ( (((npiMsgData_t *)&(npi_ipc_buf[0][0]))->len + RPC_FRAME_HDR_SZ) <= sizeof(npiMsgData_t) )
						{
							// Allocate memory for new message
							areqMsg *newMessage = (areqMsg *) malloc(sizeof(areqMsg));
							if (newMessage == NULL)
							{
								// Serious error, must abort
								done = 1;
								LOG_ERROR("[NPI Client READ] Could not allocate memory for AREQ message\n");
								break;
							}
							else
							{
								messageCount++;
								memset(newMessage, 0, sizeof(areqMsg));
								LOG_TRACE("[NPI Client READ][DBG] Allocated \t@ %p (received\040 %d messages)...\n",
										(void *)newMessage,
										messageCount);
							}

							LOG_TRACE("[NPI Client READ] Filling new message (@ %p)...\n", (void *)newMessage);

							// Copy AREQ message into AREQ buffer
							memcpy(&(newMessage->message),
									(uint8*)&(npi_ipc_buf[0][0]),
									(((npiMsgData_t *)&(npi_ipc_buf[0][0]))->len + RPC_FRAME_HDR_SZ));

							// Place message in read list
							if (npi_ipc_areq_rec_buf == NULL)
							{
								// First message in list
								npi_ipc_areq_rec_buf = newMessage;
							}
							else
							{
								areqMsg *searchList = npi_ipc_areq_rec_buf;
								// Find last entry and place it here
								while (searchList->nextMessage != NULL)
								{
									searchList = searchList->nextMessage;
								}
								searchList->nextMessage = newMessage;
							}
						}
						else
						{
							// Serious error
							LOG_ERROR("[NPI Client READ] ERR: Incoming AREQ has incorrect length field; %d\n",
									((npiMsgData_t *)&(npi_ipc_buf[0][0]))->len);

							LOG_TRACE("[NPI Client READ][MUTEX] Unlock AREQ Mutex (Read)\n");
							// Then unlock the thread so the handle can handle the AREQ
							pthread_mutex_unlock(&npiLnxClientAREQmutex);
						}

					}
					else
					{
						// Cannot handle synchronous requests from RNP
						LOG_ERROR("[NPI Client READ] ERR: Received unknown subsystem: 0x%.2X\n", ((npiMsgData_t *)&(npi_ipc_buf[0][0]))->subSys);
					}

					// Clear buffer for next message
					memset(npi_ipc_buf[0], 0, NPI_IPC_BUF_SIZE);
				}
			}
		}

		// Place processing of AREQ message here so it can be retried without receiving a new message
		// TODO: Can we replace this while with something a little safer?
		if (areqMsgProcessStatus != NPI_MSG_AREQ_READY)
		{
#ifdef __BIG_DEBUG__
			// Can only process 1 AREQ at a time...
			if (writeOnceAREQReady == 0)
			{
				LOG_TRACE("[NPI Client READ][MUTEX] Waiting for AREQ Handle to acknowledge processing the message\n");
				fflush(LOG_DESTINATION_FP);
				writeOnceAREQReady++;
			}
			else
			{
				writeOnceAREQReady++;
				if ( (writeOnceAREQReady % 1000) == 0)
				{
					LOG_TRACE("[NPI Client READ][MUTEX] Still waiting for AREQ Handle to acknowledge processing the message\n");
				}
				else if ( (writeOnceAREQReady % 10) == 0)
				{
					LOG_TRACE("*");
				}
				if (writeOnceAREQReady > 0xEFFFFFF0)
					writeOnceAREQReady = 1;
				fflush(LOG_DESTINATION_FP);
			}
#endif //__BIG_DEBUG__
			if ( (messageCount > 100) && ( (messageCount % 100) == 0) )
			{
				LOG_WARN("[NPI Client READ] AREQ message count: %4d\n", messageCount);
				if (messageCount > 500)
				{
					LOG_WARN("[NPI Client READ] SEVERE: More than 500 messages in the buffer requires attention!\n");
				}
			}
		}
		// Handle thread must make sure it has finished its list. See if there are new messages to move over
		else if (npi_ipc_areq_rec_buf != NULL)
		{
#ifdef __BIG_DEBUG__
			if (writeOnceAREQMutexTry == 0)
				LOG_TRACE("[NPI Client READ][MUTEX] Lock AREQ Mutex (Read)\n");
#endif //__BIG_DEBUG__
			if ( (mutexRet = pthread_mutex_trylock(&npiLnxClientAREQmutex)) == EBUSY)
			{
#ifdef __BIG_DEBUG__
				if (writeOnceAREQMutexTry == 0)
				{
					LOG_TRACE("[NPI Client READ][MUTEX] AREQ Mutex (Read) busy");
					fflush(LOG_DESTINATION_FP);
					writeOnceAREQMutexTry++;
				}
				else
				{
					writeOnceAREQMutexTry++;
					if ( (writeOnceAREQMutexTry % 100) == 0)
					{
						LOG_TRACE(".");
					}
					if (writeOnceAREQMutexTry > 0xEFFFFFF0)
						writeOnceAREQMutexTry = 1;
					fflush(LOG_DESTINATION_FP);
				}
#endif //__BIG_DEBUG__
			}
			else if (mutexRet == 0)
			{
#ifdef __BIG_DEBUG__
				writeOnceAREQMutexTry = 0;
				LOG_TRACE("[NPI Client READ][MUTEX] AREQ Lock (Read), status: %d\n", mutexRet);
#endif //__BIG_DEBUG__

				areqMsgProcessStatus = NPI_MSG_AREQ_BUSY;

				// Move list over for processing
				npi_ipc_areq_proc_buf = npi_ipc_areq_rec_buf;
				// Clear receiving buffer for new messages
				npi_ipc_areq_rec_buf = NULL;

				LOG_TRACE("[NPI Client READ][DBG] Copied message list (processed %d messages)...\n",
						messageCount);

				LOG_TRACE("[NPI Client READ][MUTEX] Unlock AREQ Mutex (Read)\n");
				// Then unlock the thread so the handle can handle the AREQ
				pthread_mutex_unlock(&npiLnxClientAREQmutex);

				LOG_TRACE("[NPI Client READ][MUTEX] AREQ Signal message read (Read)...\n");
				// Signal to the handle thread an AREQ message is ready
				pthread_cond_signal(&npiLnxClientAREQcond);
				LOG_TRACE("[NPI Client READ][MUTEX] AREQ Signal message read (Read)... sent\n");

#ifdef __BIG_DEBUG__
				writeOnceAREQReady = 0;
#endif //__BIG_DEBUG__
			}
		}

	} while (!done);


	return ptr;
}

/**************************************************************************************************
 *
 * @fn          npi_ipc_initsyncres
 *
 * @brief
 *
 * input parameters
 *
 * @param       none
 *
 * output parameters
 *
 * None.
 *
 * @return      none
 *
 **************************************************************************************************/
static void npi_ipc_initsyncres(void)
{
  // initialize all mutexes
  pthread_mutex_init(&npiLnxClientSREQmutex, NULL);
  pthread_mutex_init(&npiLnxClientAREQmutex, NULL);

  // initialize all conditional variables
  pthread_cond_init(&npiLnxClientSREQcond, NULL);
  pthread_cond_init(&npiLnxClientAREQcond, NULL);
}

/**************************************************************************************************
 *
 * @fn          npi_ipc_delsyncres
 *
 * @brief
 *
 * input parameters
 *
 * @param       none
 *
 * output parameters
 *
 * None.
 *
 * @return      none
 *
 **************************************************************************************************/
static void npi_ipc_delsyncres(void)
{
  // In Linux, there is no dynamically allocated resources
  // and hence the following calls do not actually matter.

  // destroy all conditional variables
  pthread_cond_destroy(&npiLnxClientSREQcond);
  pthread_cond_destroy(&npiLnxClientAREQcond);

  // destroy all mutexes
  pthread_mutex_destroy(&npiLnxClientSREQmutex);
  pthread_mutex_destroy(&npiLnxClientAREQmutex);

}

/**************************************************************************************************
 *
 * @fn          NPI_ClientClose
 *
 * @brief       This function stops RTI surrogate module
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 *
 **************************************************************************************************/
void NPI_ClientClose(void)
{
	// Close the NPI socket connection

	close(sNPIconnected);

	// Delete synchronization resources
	npi_ipc_delsyncres();

#ifndef NPI_UNIX
	freeaddrinfo(resAddr); // free the linked-list
#endif //NPI_UNIX

}

/**************************************************************************************************
 *
 * @fn          NPI_SendSynchData
 *
 * @brief       This function sends a message synchronously over the socket
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 *
 **************************************************************************************************/
void NPI_SendSynchData (npiMsgData_t *pMsg)
{
	int result = 0;
	int mutexRet = 0;
	int bytesSent;
	long remainingMicroseconds;
	struct timespec expiryTime, monotonicStart;
	long callingThreadID = syscall(SYS_gettid);

	// Add Proper RPC type to header
	((uint8*)pMsg)[RPC_POS_CMD0] = (((uint8*)pMsg)[RPC_POS_CMD0] & RPC_SUBSYSTEM_MASK) | RPC_CMD_SREQ;

	int i;
	char str[256];
	uint8 charWritten = 0;
	charWritten = snprintf(str, sizeof(str), "[NPI Client SEND SYNCH] Thread %ld Preparing to send %d bytes, subSys 0x%.2X, cmdId 0x%.2X, pData:",
							callingThreadID,
							pMsg->len,
							pMsg->subSys,
							pMsg->cmdId);
	charWritten += snprintf(&str[charWritten], sizeof(str) - charWritten, "\t");
	for (i = 0; i < pMsg->len; i++)
	{
		charWritten += snprintf(&str[charWritten], sizeof(str) - charWritten, " 0x%.2X", pMsg->pData[i]);
	}
	charWritten += snprintf(&str[charWritten], sizeof(str) - charWritten, "\n");
	LOG_DEBUG("%s", str);

	LOG_TRACE("[NPI Client SEND SYNCH][MUTEX] Thread %ld: Locking npiLnxClientSREQSerializationMutex\n", callingThreadID);
	if ((mutexRet = pthread_mutex_lock(&npiLnxClientSREQSerializationMutex)) != 0)
	{
		// Should be "impossible" to get here, given the nature of the mutex, but handle case anyway.
		// Unfortunately, there is no way to return an error from this function as-coded.
		LOG_ERROR("[NPI Client SEND SYNCH][MUTEX] Thread %ld: Error %d waiting for npiLnxClientSREQSerializationMutex! Cannot send!\n", callingThreadID, mutexRet);
	}
	else
	{
		// Got serialization mutex, now
		// Lock the tx/rx synchronization mutex
		LOG_TRACE("[NPI Client SEND SYNCH][MUTEX] Thread %ld: Got serialization mutex, now Lock SRSP Mutex\n", callingThreadID);
		int writeOnce = 0;
		while ( (mutexRet = pthread_mutex_trylock(&npiLnxClientSREQmutex)) == EBUSY)
		{
			if (writeOnce == 0)
			{
				LOG_TRACE("[NPI Client SEND SYNCH][MUTEX] Thread %ld: SRSP Mutex busy\n", callingThreadID);
				writeOnce++;
			}
			else
			{
				writeOnce++;
				if ( (writeOnce % 1000) == 0)
				{
					LOG_TRACE("%d.", writeOnce);
				}
				if (writeOnce > 0xFFFFFFF0)
					writeOnce = 1;
				fflush(LOG_DESTINATION_FP);
			}
		}
		LOG_TRACE("[NPI Client SEND SYNCH][MUTEX] Thread %ld: SRSP Lock status: %d\n", callingThreadID, mutexRet);


		bytesSent = send(sNPIconnected, (uint8 *)pMsg, pMsg->len + RPC_FRAME_HDR_SZ, 0);

		if (bytesSent == -1)
		{
			LOG_FATAL("[NPI Client] send");
			exit(1);
		}
		else if (bytesSent != (pMsg->len + RPC_FRAME_HDR_SZ))
		{
			LOG_ERROR("[NPI Client] Sent only %d of %d bytes!\n", bytesSent, (pMsg->len + RPC_FRAME_HDR_SZ));
		}

		LOG_TRACE("[NPI Client SEND SYNCH] Sent %d bytes.  Waiting for synchronous response...\n", bytesSent);

		// Conditional wait for the response handled in the receiving thread,
		// wait maximum NPI_IPC_CLIENT_SYNCH_TIMEOUT seconds.  Since this is
		// vulnerable to clock adjustments and pthread_cond_timedwait requires
		// a realtime clock, wrap this in a monotonic clock check and retry if
		// timed out before actual timeout in monotonic time was hit.
		clock_gettime(CLOCK_MONOTONIC, &monotonicStart);
		clock_gettime(CLOCK_REALTIME, &expiryTime);
		expiryTime.tv_sec += NPI_IPC_CLIENT_SYNCH_TIMEOUT;
		LOG_TRACE("[NPI Client SEND SYNCH][MUTEX] Thread %ld: Wait for SRSP Cond signal...\n", callingThreadID);
		do
		{
			result = pthread_cond_timedwait(&npiLnxClientSREQcond, &npiLnxClientSREQmutex, &expiryTime);
			if (result != ETIMEDOUT)
				remainingMicroseconds = 0;
			else
			{
				// Check monotonic clock to see if timeout time actually elapsed or clock jumped.
				struct timespec monotonicCurrent;
				long elapsedMicroseconds;

				clock_gettime(CLOCK_MONOTONIC, &monotonicCurrent);
				elapsedMicroseconds = ((long)((long)(monotonicCurrent.tv_sec) - (long)(monotonicStart.tv_sec)) * 1000000L) +
				                      ((long)(((long)(monotonicCurrent.tv_nsec) - (long)(monotonicStart.tv_nsec))/1000L));

				remainingMicroseconds = (NPI_IPC_CLIENT_SYNCH_TIMEOUT * 1000000L) - elapsedMicroseconds;
				if (remainingMicroseconds > 0)
				{
					// Realtime clock must have jumped.  Set expiry time to current time + remaining timeout
					// and loop back to try again.
					clock_gettime(CLOCK_REALTIME, &expiryTime);
					expiryTime.tv_sec += (remainingMicroseconds / 1000000);
					expiryTime.tv_nsec += ((remainingMicroseconds % 1000000) * 1000);

					if (expiryTime.tv_nsec > 1000000000)
					{
						expiryTime.tv_sec += 1;
						expiryTime.tv_nsec -= 1000000000;
					}
				}
			}
		} while(remainingMicroseconds > 0);

		if (result == ETIMEDOUT)
		{
			// TODO: Indicate synchronous transaction error
			LOG_WARN("[NPI Client SEND SYNCH] Thread %ld: SRSP Cond Wait timed out!\n", callingThreadID);

			// Clear response buffer before processing
			memset(npi_ipc_srsp_buf, 0, sizeof(npiMsgData_t));
			// Set Timeout Error to rsp buffer
//			pMsg->pData[0]=0xFF;
		}
		// Wait for response
		else if ( numOfReceievedSRSPbytes > 0)
		{
			charWritten = snprintf(str, sizeof(str), "[NPI Client] Thread %ld received data:", callingThreadID);
			for (i = 0; i < numOfReceievedSRSPbytes; i++)
			{
				charWritten += snprintf(&str[charWritten], sizeof(str) - charWritten, " 0x%.2X", (uint8)npi_ipc_srsp_buf[i]);
			}
			LOG_TRACE("%s", str);

			// Sanity check on length
			if (numOfReceievedSRSPbytes != ( ((npiMsgData_t *)npi_ipc_srsp_buf)->len + RPC_FRAME_HDR_SZ))
			{
				LOG_FATAL("[NPI Client SEND SYNCH] Mismatch in read IPC data and the expected length\n. \n\tread bytes = %d \n\tnpiMsgLength(+ 3) = %d \n\t\n",
						numOfReceievedSRSPbytes,
						((npiMsgData_t *)npi_ipc_srsp_buf)->len + RPC_FRAME_HDR_SZ);
				exit(1);
			}

			// Copy response back in transmission buffer for processing
			memcpy((uint8*)pMsg, npi_ipc_srsp_buf, numOfReceievedSRSPbytes);
		}
		else
		{
			if ( numOfReceievedSRSPbytes < 0)
				LOG_ERROR("[NPI Client] recv");
			else
				LOG_WARN("[NPI Client SEND SYNCH] Server closed connection\n");
		}

		numOfReceievedSRSPbytes = 0;

		// Now unlock the mutex before returning
		LOG_TRACE("[NPI Client SEND SYNCH][MUTEX] Thread %ld: Unlock SRSP Mutex\n", callingThreadID);
		pthread_mutex_unlock(&npiLnxClientSREQmutex);
		LOG_TRACE("[NPI Client SEND SYNCH][MUTEX] Thread %ld: Unlock npiLnxClientSREQSerializationMutex\n", callingThreadID);
		pthread_mutex_unlock(&npiLnxClientSREQSerializationMutex);
	}
}


/**************************************************************************************************
 *
 * @fn          NPI_SendAsynchData
 *
 * @brief       This function sends a message asynchronously over the socket
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 *
 **************************************************************************************************/
void NPI_SendAsynchData (npiMsgData_t *pMsg)
{
	// Add Proper RPC type to header
	if ((((uint8*)pMsg)[RPC_POS_CMD0] & RPC_CMD_TYPE_MASK) == 0)
		((uint8*)pMsg)[RPC_POS_CMD0] = (((uint8*)pMsg)[RPC_POS_CMD0] & RPC_SUBSYSTEM_MASK) | RPC_CMD_AREQ;

	int i;
    char str[256];
    uint8 charWritten;
    charWritten = snprintf(str, sizeof(str), "[NPI Client SEND ASYNCH] trying to send %d bytes,\t subSys 0x%.2X, cmdId 0x%.2X, pData:",
							pMsg->len,
							pMsg->subSys,
							pMsg->cmdId);
    charWritten += snprintf(&str[charWritten], sizeof(str) - charWritten, "\t");
    for (i = 0; i < pMsg->len; i++)
    {
       charWritten += snprintf(&str[charWritten], sizeof(str) - charWritten, " 0x%.2X", pMsg->pData[i]);
    }
    LOG_DEBUG("%s\n", str);

	int bytesSent = send(sNPIconnected, (uint8 *)pMsg, pMsg->len + RPC_FRAME_HDR_SZ, 0);
	if (bytesSent == -1)
	{
		LOG_FATAL("[NPI Client SEND ASYNCH] send");
		exit(1);
	}
	else if (bytesSent < pMsg->len + RPC_FRAME_HDR_SZ)
	{
		LOG_ERROR("[NPI Client SEND ASYNCH] Sent %d bytes, of expected %d\n", bytesSent, pMsg->len + RPC_FRAME_HDR_SZ);
	}
}

int NPI_AsynchMsgCback(npiMsgData_t *pMsg )
{
  npiProcessMsg_t  func;
  int res = -1;

  if (npiState == NPI_STATE_READY)
  {
    /* check subsystem range */
    if (pMsg->subSys < RPC_SYS_MAX)
    {
      /* look up processing function */
      func = NpiAsyncMsgCbackParserTable[pMsg->subSys];
      if (func)
      {
#ifdef   __DEBUG_TIME__
         char str[256];
         uint8 charWritten, i;
         charWritten = snprintf(str, sizeof(str), "[NPI Client CBACK START] subSys 0x%.2X, cmdId 0x%.2X, pData:",
                           pMsg->subSys,
                           pMsg->cmdId);
         charWritten += snprintf(&str[charWritten], sizeof(str) - charWritten, "\t");
         for (i = 0; i < pMsg->len; i++)
         {
            charWritten += snprintf(&str[charWritten], sizeof(str) - charWritten, " 0x%.2X", pMsg->pData[i]);
         }
         charWritten += snprintf(&str[charWritten], sizeof(str) - charWritten, "\n");
         LOG_DEBUG("%s", str);
#endif //__DEBUG_TIME__
        /* execute processing function */
        res = (*func)(pMsg);
#ifdef __DEBUG_TIME__
         LOG_DEBUG("[NPI Client CBACK ENDED] \n");
#endif //__DEBUG_TIME__
      }
      else
      {
        LOG_TRACE("Warning! Asynch Msg received not handled! Did you register the parser callback for ?: %s \n", RpcCmdType_list[pMsg->subSys]);
      }
    }
    else
    {
        LOG_TRACE("Warning! Asynch Msg received does not correspond to any module: %d >=  RPC_SYS_MAX: \n", pMsg->subSys);
    }
  }
  return res;

}
/**************************************************************************************************
 *
 * @fn          NPI_ReadVersion
 *
 * @brief       This API is used to read the NPI server version.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 *
 **************************************************************************************************/
void NPI_ReadVersionReq( uint8 *pValue )
{
  npiMsgData_t pMsg;

  // Prepare Read Version Request
  pMsg.subSys = RPC_SYS_SRV_CTRL;
  pMsg.cmdId  = NPI_LNX_CMD_ID_VERSION_REQ;
  pMsg.len    = 0;

  NPI_SendSynchData( &pMsg );

  // copy the reply data to the client's buffer
  // Note: the first byte of the payload is reserved for the status
  msg_memcpy( pValue, &pMsg.pData[1], 3 );
}
/**************************************************************************************************
 *
 * @fn          NPI_ReadParamReq
 *
 * @brief       This API is used to read NPI server parameters.
 *
 * input parameters
 *
 * @param       paramId - The parameter item identifier.
 * @param       len - The length in bytes of the item identifier's data.
 *
 * output parameters
 *
 * @param       *pValue - Pointer to buffer where read data is placed.
 *
 * None.
 *
 * @return      None.
 *
 **************************************************************************************************/
void NPI_ReadParamReq( uint8 paramId, uint8 len, uint8 *pValue )
{
  npiMsgData_t pMsg;

  // Prepare parameter request
  pMsg.subSys = RPC_SYS_SRV_CTRL;
  pMsg.cmdId  = NPI_LNX_CMD_ID_GET_PARAM_REQ;
  pMsg.len    = 2;
  pMsg.pData[0] = paramId;
  pMsg.pData[1] = len;

  NPI_SendSynchData( &pMsg );

  // copy the reply data to the client's buffer
  // Note: the first byte of the payload is reserved for the status
  msg_memcpy( pValue, &pMsg.pData[1], len );
}

/**************************************************************************************************
 *
 * @fn          NPI_ConnectReq
 *
 * @brief       This API is used to ask NPI server to connect to device.
 *
 * input parameters
 *
 * @param		*pStatus - Pointer to device to connect to
 * @param		length	 - length of device path
 * @param		devPath	 - device path string
 *
 * output parameters
 *
 * @param       *pStatus - Pointer to buffer where status is read.
 *
 * None.
 *
 * @return      None.
 *
 **************************************************************************************************/
void NPI_ConnectReq( uint8 *pStatus, uint8 length, uint8 *devPath )
{
  npiMsgData_t pMsg;

  // Prepare connection request
  pMsg.subSys = RPC_SYS_SRV_CTRL;
  pMsg.cmdId  = NPI_LNX_CMD_ID_CONNECT_DEVICE;
  pMsg.len    = length + 2;

  pMsg.pData[0] = *pStatus;
  pMsg.pData[1] = length;
  memcpy(pMsg.pData + 2, devPath, length);

  NPI_SendSynchData( &pMsg );

  // copy the reply data to the client's buffer
  // Note: the first byte of the payload is reserved for the status
  msg_memcpy( pStatus, &pMsg.pData[0], pMsg.len );
}

/**************************************************************************************************
 *
 * @fn          NPI_DisconnectReq
 *
 * @brief       This API is used to ask NPI server to disconnect device.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * @param       *pValue - Pointer to buffer where status is read.
 *
 * None.
 *
 * @return      None.
 *
 **************************************************************************************************/
void NPI_DisconnectReq( uint8 *pStatus )
{
  npiMsgData_t pMsg;

  // Prepare disconnection request
  pMsg.subSys = RPC_SYS_SRV_CTRL;
  pMsg.cmdId  = NPI_LNX_CMD_ID_DISCONNECT_DEVICE;
  pMsg.len    = 0;

  NPI_SendSynchData( &pMsg );

  // copy the reply data to the client's buffer
  // Note: the first byte of the payload is reserved for the status
  msg_memcpy( pStatus, &pMsg.pData[0], pMsg.len );
}

/**************************************************************************************************
 *
 * @fn          NPI_SetWorkaroundReq
 *
 * @brief       This API is used to ask NPI server to set workaround.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * @param       item		- Workaround ID
 * @param       *pStatus 	- Pointer to buffer where status is read.
 *
 * None.
 *
 * @return      None.
 *
 **************************************************************************************************/
void NPI_SetWorkaroundReq( uint8 workaroundID, uint8 *pStatus )
{
  npiMsgData_t pMsg;

  // Prepare workaround request
  pMsg.subSys = RPC_SYS_SRV_CTRL;
//  pMsg.cmdId  = NPI_LNX_CMD_ID_WORKAROUND;
  pMsg.cmdId  = 2;
  pMsg.len    = 2;

  pMsg.pData[0] = workaroundID;
  pMsg.pData[1] = *pStatus;

  NPI_SendSynchData( &pMsg );

  // copy the reply data to the client's buffer
  // Note: the first byte of the payload is reserved for the status
  msg_memcpy( pStatus, &pMsg.pData[0], pMsg.len );
}


// -- utility porting --

// These utility functions are called from RTI surrogate module

// -- dummy functions --

// These dummies are called by RTIS module, but they are irrelevant in windows
// environment.
void NPI_Init(void)
{
  // NPI_Init() is a function supposed to be called in OSAL environment
  // Application processor RTIS module calls this function directly
  // to make NPI work for non-OSAL application processor but
  // in case of Windows port of NPI module, this function call is not
  // implemented.
}


/**************************************************************************************************
 **************************************************************************************************/
