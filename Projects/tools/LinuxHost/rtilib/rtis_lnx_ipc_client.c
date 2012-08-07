 /**************************************************************************************************
  Filename:       rtis_lnx_ipc_client.c
  Revised:        $Date: 2012-03-21 17:37:33 -0700 (Wed, 21 Mar 2012) $
  Revision:       $Revision: 246 $

  Description:    This file contains Linux platform specific RemoTI (RTI) API
                  Surrogate implementation

  Copyright (C) {2012} Texas Instruments Incorporated - http://www.ti.com/


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

#ifndef NPI_UNIX
#include <netdb.h>
#include <arpa/inet.h>
#endif

/* NPI includes */
#include "npi_lnx.h"

/* RTI includes */
#include "rti_lnx_constants.h"
#include "rti_lnx.h"
#include "rtis_lnx.h"


#ifdef __BIG_DEBUG__
#define debug_printf(fmt, ...) printf( fmt, ##__VA_ARGS__)
#else
#define debug_printf(fmt, ...)
#endif

#define msg_memcpy(src, dst, len)	memcpy(src, dst, len)

/**************************************************************************************************
 *                                        Externals
 **************************************************************************************************/

/**************************************************************************************************
 *                                           Constant
 **************************************************************************************************/

#define RTIS_MSG_AREQ_READY			0x0000
#define RTIS_MSG_AREQ_BUSY			0x0001

#define RTIS_IPC_BUF_SIZE			(2 * (sizeof(npiMsgData_t)))

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
char rtis_ipc_buf[2][RTIS_IPC_BUF_SIZE];
// Client data AREQ received buffer
areqMsg *rtis_ipc_areq_rec_buf;
// Client data AREQ processing buffer
areqMsg *rtis_ipc_areq_proc_buf;

// Message count to keep track of incoming and processed messages
static int messageCount = 0;

// Client data SRSP reception buffer
char rtis_ipc_srsp_buf[(sizeof(npiMsgData_t))];

// variable to store the number of received synchronous response bytes
static int numOfReceievedSRSPbytes = 0;

static pthread_t RTISThreadId;
static void *rtis_lnx_ipc_readThreadFunc (void *ptr);
static void *rtis_lnx_ipc_handleThreadFunc (void *ptr);

// Mutex to handle synchronous response
pthread_mutex_t rtisLnxClientSREQmutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t rtisLnxClientAREQmutex = PTHREAD_MUTEX_INITIALIZER;
// conditional variable to notify Synchronous response
static pthread_cond_t rtisLnxClientSREQcond;
// conditional variable to notify that the AREQ is handled
static pthread_cond_t rtisLnxClientAREQcond;

// mutex variable, AREQ message processing status
static uint8 areqMsgProcessStatus = RTIS_MSG_AREQ_READY;

#ifndef NPI_UNIX
struct addrinfo *resAddr;
#endif

// current state
#if defined _WIN32 || defined unix
// windows does not require RTI_Init() call
static uint8 rtisState = RTIS_STATE_READY;
#else
static uint8 rtisState;
#endif // _WIN32
static uint8 rtisBE; // big endian machine flag

/**************************************************************************************************
 *                                     Local Function Prototypes
 **************************************************************************************************/

//// RTI Task Related
//void   RTI_Init( uint8 taskId );
//uint16 RTI_ProcessEvent( uint8 taskId, uint16 events );

// NPI Callback Related
void NPI_AsynchMsgCback( npiMsgData_t *pMsg );

// Endianness conversion
static void rtisAttribEConv( uint8 attrib, uint8 len, uint8 *pValue );

// Endianness conversion macros
#define RTI_ECONV16(_value) ((((_value) & 0xff)<<8)|(((_value) & 0xff00)>>8))
#define RTI_ECONV32(_value) \
  ((((_value) & 0xff)<<24)|(((_value) & 0xff00)<<8)| \
   (((_value) & 0xff0000)>>8)|(((_value) & 0xff000000)>>24))
#define RTI_PAIRING_ENTRY_REQUIRED_LEN(_field) \
  ((uint16) (&((rcnNwkPairingEntry_t *) 0)->_field) + sizeof(((rcnNwkPairingEntry_t *) 0)->_field))


static void rtis_lnx_ipc_initsyncres(void);
static void rtis_lnx_ipc_delsyncres(void);


/**************************************************************************************************
 *
 * @fn          RTIS_Init_Socket
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

int RTIS_Init(const char *devPath)
{
	int res = TRUE, i = 0;
	const char *ipAddress = "", *port = "";

	char *pStr, strTmp[128];
	strncpy(strTmp, devPath, 128);
	// use strtok to split string and find IP address and port;
	// the format is = IPaddress:port
	// Get first token
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

	/**********************************************************************
	 * Initiate synchronization resources
	 */
	rtis_lnx_ipc_initsyncres();

	/**********************************************************************
	 * Connect to the NPI server
	 **********************************************************************/

#ifdef NPI_UNIX
    int len;
    struct sockaddr_un remote;

    if ((sNPIconnected = socket(AF_UNIX, SOCK_STREAM, 0)) == -1) {
        perror("socket");
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
		printf("Warning! Port not sent to RTIS. Will use default port: %s", NPI_PORT);

	    if ((res = getaddrinfo(ipAddress, NPI_PORT, &hints, &resAddr)) != 0)
	    {
	    	fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(res));
	    	res = 2;
	    }
	    else
	    {
	    	// Because of inverted logic on return value
	    	res = TRUE;
	    }
	}
	else
	{
	    printf("Port: %s\n\n", port);
	    if ((res = getaddrinfo(ipAddress, port, &hints, &resAddr)) != 0)
	    {
	    	fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(res));
	    	res = 2;
	    }
	    else
	    {
	    	// Because of inverted logic on return value
	    	res = TRUE;
	    }
	}

    printf("IP addresses for %s:\n\n", ipAddress);

    struct addrinfo *p;
    char ipstr[INET6_ADDRSTRLEN];
    for(p = resAddr;p != NULL; p = p->ai_next) {
        void *addr;
        char *ipver;

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
        printf("  %s: %s\n", ipver, ipstr);
    }


    if ((sNPIconnected = socket(resAddr->ai_family, resAddr->ai_socktype, resAddr->ai_protocol)) == -1) {
        perror("socket");
        exit(1);
    }
#endif

    printf("Trying to connect...\n");

#ifdef NPI_UNIX
    remote.sun_family = AF_UNIX;
    strcpy(remote.sun_path, ipAddress);
    len = strlen(remote.sun_path) + sizeof(remote.sun_family);
    if (connect(sNPIconnected, (struct sockaddr *)&remote, len) == -1) {
        perror("connect");
        res = FALSE;
    }
#else
    if (connect(sNPIconnected, resAddr->ai_addr, resAddr->ai_addrlen) == -1) {
        perror("connect");
        res = FALSE;
    }
#endif

    if (res == TRUE)
    	printf("Connected.\n");


	int no = 0;
	// allow out-of-band data
	if (setsockopt(sNPIconnected, SOL_SOCKET, SO_OOBINLINE, &no, sizeof(int)) == -1)
	{
		perror("setsockopt");
		res = FALSE;
	}

	/**********************************************************************
	 * Create thread which can read new messages from the NPI server
	 **********************************************************************/

	if (pthread_create(&RTISThreadId, NULL, rtis_lnx_ipc_readThreadFunc, NULL))
	{
		// thread creation failed
		printf("Failed to create RTIS LNX IPC Client read thread\n");
		return -1;
	}

	/**********************************************************************
	 * Create thread which can handle new messages from the NPI server
	 **********************************************************************/

	if (pthread_create(&RTISThreadId, NULL, rtis_lnx_ipc_handleThreadFunc, NULL))
	{
		// thread creation failed
		printf("Failed to create RTIS LNX IPC Client handle thread\n");
		return -1;
	}

	return res;
}

static void *rtis_lnx_ipc_handleThreadFunc (void *ptr)
{
	int done = 0, tryLockFirstTimeOnly = 0;
//	, result = 0;
//	struct timespec expirytime;
//	struct timeval curtime;

	// Handle message from socket
	do {

		if (tryLockFirstTimeOnly == 0)
		{
			// Lock mutex
			debug_printf("[MUTEX] Lock AREQ Mutex (Handle)\n");
			int mutexRet = 0, writeOnce = 0;
			while ( (mutexRet = pthread_mutex_trylock(&rtisLnxClientAREQmutex)) == EBUSY)
			{
				if (writeOnce == 0)
				{
					debug_printf("[MUTEX] AREQ Mutex (Handle) busy");
					fflush(stdout);
					writeOnce++;
				}
				else
				{
					writeOnce++;
					if ( (writeOnce % 1000) == 0)
					{
						debug_printf(".");
					}
					if (writeOnce > 0xEFFFFFF0)
						writeOnce = 1;
					fflush(stdout);
				}
			}
			debug_printf("\n[MUTEX] AREQ Lock (Handle) status: %d\n", mutexRet);
			tryLockFirstTimeOnly = 1;
		}

		// Conditional wait for the response handled in the AREQ handling thread,
//		// wait maximum 10 seconds
//		gettimeofday(&curtime, NULL);
//		expirytime.tv_sec = curtime.tv_sec + 10;
//		expirytime.tv_nsec = curtime.tv_usec * 1000;
		debug_printf("[MUTEX] Wait for AREQ Cond (Handle) signal... (areqMsgProcessStatus = %d), effectively releasing lock\n", areqMsgProcessStatus);
//		result = pthread_cond_timedwait(&rtisLnxClientAREQcond, &rtisLnxClientAREQmutex, &expirytime);
		pthread_cond_wait(&rtisLnxClientAREQcond, &rtisLnxClientAREQmutex);
//		if (result == ETIMEDOUT)
//		{
//			debug_printf("[MUTEX] AREQ Cond Wait (Handle) timed out!\n");
//		}
//		else
//		{
//			debug_printf("[MUTEX] AREQ Cond Wait (Handle), status: %d\n", result);
		debug_printf("[MUTEX] AREQ (Handle) has lock\n");

		// Walk through all received AREQ messages before releasing MUTEX
		areqMsg *searchList = rtis_ipc_areq_proc_buf, *clearList;
		while (searchList != NULL)
		{
			debug_printf("\n\n[DBG] Processing \t@ 0x%.16X next \t@ 0x%.16X\n",
					(unsigned int)searchList,
					(unsigned int)(searchList->nextMessage));

			// Must remove command type before calling NPI_AsynchMsgCback
			searchList->message.subSys &= ~(RPC_CMD_TYPE_MASK);

			debug_printf("[MUTEX] AREQ Calling NPI_AsynchMsgCback (Handle)...\n");

			NPI_AsynchMsgCback((npiMsgData_t *)&(searchList->message));

			debug_printf("[MUTEX] AREQ (Handle) (message @ 0x%.16X)...\n", (unsigned int)searchList);

			clearList = searchList;
			// Set search list to next message
			searchList = searchList->nextMessage;
			// Free processed buffer
			if (clearList == NULL)
			{
				// Impossible error, must abort
				done = 1;
				printf("[ERR] clearList buffer was already free\n");
				break;
			}
			else
			{
	//			clearList = NULL;
				messageCount--;
				debug_printf("[DBG] Clearing \t\t@ 0x%.16X (processed %d messages)...\n",
						(unsigned int)clearList,
						messageCount);
				memset(clearList, 0, sizeof(areqMsg));
				free(clearList);
			}
		}
		debug_printf("[MUTEX] AREQ Signal message(s) handled (Handle) (processed %d messages)...\n", messageCount);
		// Signal to the read thread that we're ready for more
		//			pthread_cond_signal(&rtisLnxClientAREQcond);
		areqMsgProcessStatus = RTIS_MSG_AREQ_READY;

		debug_printf("[DBG] Finished processing (processed %d messages)...\n",
				messageCount);

//		debug_printf("[MUTEX] Unlock AREQ Mutex (Handle)\n\n");
//		pthread_mutex_unlock(&rtisLnxClientAREQmutex);

	} while (!done);

	return ptr;
}

static void *rtis_lnx_ipc_readThreadFunc (void *ptr)
{
	int done = 0, n;
//	, result = 0;
//	struct timespec expirytime;
//	struct timeval curtime;

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

		// TODO: Maybe only poll in case there are AREQ messages to process?
		pollRet = poll((struct pollfd*)&ufds, 1, 1);
		if (pollRet == -1)
		{
			// Error occured in poll()
			perror("poll");
		}
		else if (pollRet == 0)
		{
			// Timeout, could still be AREQ to process
		}
		else
		{
			if (ufds[0].revents & POLLIN) {
				n = recv(sNPIconnected,
						rtis_ipc_buf[0],
						RPC_FRAME_HDR_SZ,
						0); // normal data
			}
			if (ufds[0].revents & POLLPRI) {
				n = recv(sNPIconnected,
						rtis_ipc_buf[0],
						RPC_FRAME_HDR_SZ,
						MSG_OOB); // out-of-band data
			}
			if (n <= 0)
			{
				if (n < 0)
					perror("recv");
				done = 1;
			}
			else if (n == 3)
			{
				// We have received the header, now read out length byte and process it
				n = recv(sNPIconnected,
						(uint8*)&(rtis_ipc_buf[0][RPC_FRAME_HDR_SZ]),
						((npiMsgData_t *)&(rtis_ipc_buf[0][0]))->len,
						0);
				if (n == ((npiMsgData_t *)&(rtis_ipc_buf[0][0]))->len)
				{
					int i;
					debug_printf("Received %d bytes,\t subSys 0x%.2X, cmdId 0x%.2X, pData:\t",
							((npiMsgData_t *)&(rtis_ipc_buf[0][0]))->len,
							((npiMsgData_t *)&(rtis_ipc_buf[0][0]))->subSys,
							((npiMsgData_t *)&(rtis_ipc_buf[0][0]))->cmdId);
					for (i = 3; i < (n + RPC_FRAME_HDR_SZ); i++)
					{
						debug_printf(" 0x%.2X", (uint8)rtis_ipc_buf[0][i]);
					}
					debug_printf("\n");

					if ( ( (uint8)(((npiMsgData_t *)&(rtis_ipc_buf[0][0]))->subSys) & (uint8)RPC_CMD_TYPE_MASK) == RPC_CMD_SRSP )
					{
						numOfReceievedSRSPbytes = ((npiMsgData_t *)&(rtis_ipc_buf[0][0]))->len + RPC_FRAME_HDR_SZ;

						// Copy buffer so that we don't clear it before it's handled.
						memcpy(rtis_ipc_srsp_buf,
								(uint8*)&(rtis_ipc_buf[0][0]),
								numOfReceievedSRSPbytes);

						// and signal the synchronous reception
						debug_printf("[MUTEX] SRSP Cond signal set\n");
						debug_printf("Client Read: (len %d): ", numOfReceievedSRSPbytes);
						fflush(stdout);
						pthread_cond_signal(&rtisLnxClientSREQcond);
					}
					else if ( ( (uint8)(((npiMsgData_t *)&(rtis_ipc_buf[0][0]))->subSys) & (uint8)RPC_CMD_TYPE_MASK) == RPC_CMD_AREQ )
					{
						debug_printf("RPC_CMD_AREQ cmdId: 0x%.2X, %.6d\n", ((npiMsgData_t *)&(rtis_ipc_buf[0][0]))->cmdId, counter++);
						// Verify the size of the incoming message before passing it
						if ( (((npiMsgData_t *)&(rtis_ipc_buf[0][0]))->len + RPC_FRAME_HDR_SZ) <= sizeof(npiMsgData_t) )
						{
							// Allocate memory for new message
							areqMsg *newMessage = (areqMsg *) malloc(sizeof(areqMsg));
							if (newMessage == NULL)
							{
								// Serious error, must abort
								done = 1;
								printf("[ERR] Could not allocate memory for AREQ message\n");
								break;
							}
							else
							{
								messageCount++;
								memset(newMessage, 0, sizeof(areqMsg));
								debug_printf("\n[DBG] Allocated \t@ 0x%.16X (received\040 %d messages)...\n",
										(unsigned int)newMessage,
										messageCount);
							}

							debug_printf("Filling new message (@ 0x%.16X)...\n", (unsigned int)newMessage);

							// Copy AREQ message into AREQ buffer
							memcpy(&(newMessage->message),
									(uint8*)&(rtis_ipc_buf[0][0]),
									(((npiMsgData_t *)&(rtis_ipc_buf[0][0]))->len + RPC_FRAME_HDR_SZ));

							// Place message in read list
							if (rtis_ipc_areq_rec_buf == NULL)
							{
								// First message in list
								rtis_ipc_areq_rec_buf = newMessage;
							}
							else
							{
								areqMsg *searchList = rtis_ipc_areq_rec_buf;
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
							printf("ERR: Incoming AREQ has incorrect length field; %d\n",
									((npiMsgData_t *)&(rtis_ipc_buf[0][0]))->len);

							debug_printf("[MUTEX] Unlock AREQ Mutex (Read)\n");
							// Then unlock the thread so the handle can handle the AREQ
							pthread_mutex_unlock(&rtisLnxClientAREQmutex);
						}

					}
					else
					{
						// Cannot handle synchronous requests from RNP
						printf("ERR: Received SREQ\n");
					}

					// Clear buffer for next message
					memset(rtis_ipc_buf[0], 0, RTIS_IPC_BUF_SIZE);
				}
				else
				{
					// Impossible. ... ;)
				}
			}
			else
			{
				// Impossible. ... ;)
			}
		}

		// Place processing of AREQ message here so it can be retried without receiving a new message
		// TODO: Can we replace this while with something a little safer?
		if (areqMsgProcessStatus != RTIS_MSG_AREQ_READY)
		{
#ifdef __BIG_DEBUG__
			// Can only process 1 AREQ at a time...
			if (writeOnceAREQReady == 0)
			{
				debug_printf("[MUTEX] Waiting for AREQ Handle to acknowledge processing the message\n");
				fflush(stdout);
				writeOnceAREQReady++;
			}
			else
			{
				writeOnceAREQReady++;
				if ( (writeOnceAREQReady % 1000) == 0)
				{
					debug_printf("[MUTEX] Still waiting for AREQ Handle to acknowledge processing the message\n");
				}
				else if ( (writeOnceAREQReady % 10) == 0)
				{
					debug_printf("*");
				}
				if (writeOnceAREQReady > 0xEFFFFFF0)
					writeOnceAREQReady = 1;
				fflush(stdout);
			}
#endif //__BIG_DEBUG__
			if (messageCount > 50)
			{
				printf("[WARNING] AREQ message count: %4d", messageCount);
			}
		}
		// Handle thread must make sure it has finished its list. See if there are new messages to move over
		else if (rtis_ipc_areq_rec_buf != NULL)
		{
#ifdef __BIG_DEBUG__
			if (writeOnceAREQMutexTry == 0)
				debug_printf("[MUTEX] Lock AREQ Mutex (Read)\n");
#endif //__BIG_DEBUG__
			if ( (mutexRet = pthread_mutex_trylock(&rtisLnxClientAREQmutex)) == EBUSY)
			{
#ifdef __BIG_DEBUG__
				if (writeOnceAREQMutexTry == 0)
				{
					debug_printf("[MUTEX] AREQ Mutex (Read) busy");
					fflush(stdout);
					writeOnceAREQMutexTry++;
				}
				else
				{
					writeOnceAREQMutexTry++;
					if ( (writeOnceAREQMutexTry % 100) == 0)
					{
						debug_printf(".");
					}
					if (writeOnceAREQMutexTry > 0xEFFFFFF0)
						writeOnceAREQMutexTry = 1;
					fflush(stdout);
				}
#endif //__BIG_DEBUG__
			}
			else if (mutexRet == 0)
			{
#ifdef __BIG_DEBUG__
				writeOnceAREQMutexTry = 0;
				debug_printf("\n[MUTEX] AREQ Lock (Read), status: %d\n", mutexRet);
#endif //__BIG_DEBUG__

				areqMsgProcessStatus = RTIS_MSG_AREQ_BUSY;

				// Move list over for processing
				rtis_ipc_areq_proc_buf = rtis_ipc_areq_rec_buf;
				// Clear receiving buffer for new messages
				rtis_ipc_areq_rec_buf = NULL;

				debug_printf("[DBG] Copied message list (processed %d messages)...\n",
						messageCount);

				debug_printf("[MUTEX] Unlock AREQ Mutex (Read)\n");
				// Then unlock the thread so the handle can handle the AREQ
				pthread_mutex_unlock(&rtisLnxClientAREQmutex);

				debug_printf("[MUTEX] AREQ Signal message read (Read)...\n");
				// Signal to the handle thread an AREQ message is ready
				pthread_cond_signal(&rtisLnxClientAREQcond);
				debug_printf("[MUTEX] AREQ Signal message read (Read)... sent\n");

				//							debug_printf("[MUTEX] Lock AREQ Mutex (Read)\n");
				//							if ( (mutexRet = pthread_mutex_trylock(&rtisLnxClientAREQmutex)) == EBUSY)
				//							{
				//								debug_printf("[MUTEX] AREQ Mutex (Read) busy");
				//								// If mutex is busy it means data the AREQ handler has it, which
				//								// is good! We can't, and won't have to, wait for the 'AREQ handled' signal
				//							}
				//							else if (mutexRet == 0)
				//							{
				//								debug_printf("\n[MUTEX] AREQ Lock (Read), status: %d\n", mutexRet);
				//
				//								// Conditional wait for the response handled in the AREQ handling thread,
				//								// wait maximum 10 seconds
				//								gettimeofday(&curtime, NULL);
				//								expirytime.tv_sec = curtime.tv_sec + 10;
				//								expirytime.tv_nsec = curtime.tv_usec * 1000;
				//								debug_printf("[MUTEX] Wait for AREQ Cond (Read) signal...\n");
				//								result = pthread_cond_timedwait(&rtisLnxClientAREQcond, &rtisLnxClientAREQmutex, &expirytime);
				//								if (result == ETIMEDOUT)
				//								{
				//									debug_printf("[MUTEX] AREQ Cond Wait (Read) timed out!\n");
				//								}
				//								else
				//								{
				//									debug_printf("[MUTEX] AREQ Cond Wait (Read) timed out!\n");
				//								}
				//								debug_printf("[MUTEX] Unlock AREQ Mutex (Read)\n");
				//								// Then unlock the thread so the handle can handle the AREQ
				//								pthread_mutex_unlock(&rtisLnxClientAREQmutex);
				//							}

#ifdef __BIG_DEBUG__
				writeOnceAREQReady = 0;
#endif //__BIG_DEBUG__
			}
		}

	} while (!done);


	return ptr;
}

/* Initialize thread synchronization resources */
static void rtis_lnx_ipc_initsyncres(void)
{
  // initialize all mutexes
  pthread_mutex_init(&rtisLnxClientSREQmutex, NULL);
  pthread_mutex_init(&rtisLnxClientAREQmutex, NULL);

  // initialize all conditional variables
  pthread_cond_init(&rtisLnxClientSREQcond, NULL);
  pthread_cond_init(&rtisLnxClientAREQcond, NULL);
}

/* Destroy thread synchronization resources */
static void rtis_lnx_ipc_delsyncres(void)
{
  // In Linux, there is no dynamically allocated resources
  // and hence the following calls do not actually matter.

  // destroy all conditional variables
  pthread_cond_destroy(&rtisLnxClientSREQcond);
  pthread_cond_destroy(&rtisLnxClientAREQcond);

  // destroy all mutexes
  pthread_mutex_destroy(&rtisLnxClientSREQmutex);
  pthread_mutex_destroy(&rtisLnxClientAREQmutex);

}

/**************************************************************************************************
 *
 * @fn          RTIS_Close
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
void RTIS_Close(void)
{
	// Close the NPI socket connection

	close(sNPIconnected);

	// Delete synchronization resources
	rtis_lnx_ipc_delsyncres();

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
	struct timespec expirytime;
	struct timeval curtime;

	// Add Proper RPC type to header
	((uint8*)pMsg)[RPC_POS_CMD0] = (((uint8*)pMsg)[RPC_POS_CMD0] & RPC_SUBSYSTEM_MASK) | RPC_CMD_SREQ;

	int i;
	debug_printf("preparing to send %d bytes, subSys 0x%.2X, cmdId 0x%.2X, pData:",
			pMsg->len,
			pMsg->subSys,
			pMsg->cmdId);
	debug_printf("\t");
	for (i = 0; i < pMsg->len; i++)
	{
		debug_printf(" 0x%.2X", pMsg->pData[i]);
	}
	debug_printf("\n");

	// Lock mutex
	debug_printf("[MUTEX] Lock SRSP Mutex");
	int mutexRet = 0, writeOnce = 0;
	while ( (mutexRet = pthread_mutex_trylock(&rtisLnxClientSREQmutex)) == EBUSY)
	{
		if (writeOnce == 0)
		{
			debug_printf("\n[MUTEX] SRSP Mutex busy");
			fflush(stdout);
			writeOnce++;
		}
		else
		{
			writeOnce++;
			if ( (writeOnce % 1000) == 0)
			{
				debug_printf(".");
			}
			if (writeOnce > 0xFFFFFFF0)
				writeOnce = 1;
			fflush(stdout);
		}
	}
	debug_printf("\n[MUTEX] SRSP Lock status: %d\n", mutexRet);

	if (send(sNPIconnected, (uint8 *)pMsg, pMsg->len + RPC_FRAME_HDR_SZ, 0) == -1)
	{
		perror("send");
		exit(1);
	}

	debug_printf("Waiting for synchronous response...\n");

	// Conditional wait for the response handled in the receiving thread,
	// wait maximum 10 seconds
	gettimeofday(&curtime, NULL);
	expirytime.tv_sec = curtime.tv_sec + 10;
	expirytime.tv_nsec = curtime.tv_usec * 1000;
	debug_printf("[MUTEX] Wait for SRSP Cond signal...\n");
	result = pthread_cond_timedwait(&rtisLnxClientSREQcond, &rtisLnxClientSREQmutex, &expirytime);
	if (result == ETIMEDOUT)
	{
		// TODO: Indicate synchronous transaction error
		debug_printf("[MUTEX] SRSP Cond Wait timed out!\n");
		printf("[ERR] SRSP Cond Wait timed out!\n");
	}

//	int pollOK = 0;
//	struct pollfd ufds;
//	if (firstAttempt > 0)
//	{
//		printf("Received data out of bounds!...\n");
//		numOfReceievedSRSPbytes = recv(sNPIconnected, rtis_ipc_buf[1], RTIS_IPC_BUF_SIZE, MSG_OOB);
//
//		ufds.fd = sNPIconnected;
//		ufds.events = POLLIN | POLLOUT;	// Poll for incoming data and out-of-band data
//
//		// Poll the socket for 3 seconds at a time
//		do {
//			numOfReceievedSRSPbytes = poll((struct pollfd*)&ufds, 1, 3000);
//
//			if (numOfReceievedSRSPbytes < 0)
//			{
//				perror("poll");
//			}
//			else if (t == 0)
//			{
//				printf("Still polling for synchronous response...\n");
//			}
//			else
//			{
//				if (ufds.revents &  POLLIN)
//				{
//					numOfReceievedSRSPbytes = recv(sNPIconnected, rtis_ipc_buf[1], RTIS_IPC_BUF_SIZE, 0);
//					// Indicate that we are finished polling.
//					pollOK = 1;
//				}
//				else if (ufds.revents &  POLLOUT)
//				{
//					// Receive data out-of-bounds
//					printf("Received data out of bounds!...\n");
//					numOfReceievedSRSPbytes = recv(sNPIconnected, rtis_ipc_buf[1], RTIS_IPC_BUF_SIZE, MSG_OOB);
//					// Indicate that we are finished polling.
//					//				if (numOfReceievedSRSPbytes > 0)
//					pollOK = 1;
//				}
//				else
//				{
//					printf("Unknown error occurred while polling for synchronous response...\n");
//				}
//			}
//		} while (!pollOK);
//	}
//	else
//	{
//		numOfReceievedSRSPbytes = recv(sNPIconnected, rtis_ipc_buf[0], RTIS_IPC_BUF_SIZE, 0);
//		firstAttempt++;
//	}

	// Wait for response
	if ( numOfReceievedSRSPbytes > 0)
	{
		for (i = 0; i < numOfReceievedSRSPbytes; i++)
		{
			debug_printf(" 0x%.2X", (uint8)rtis_ipc_srsp_buf[i]);
		}
		debug_printf("\n");

		// Sanity check on length
		if (numOfReceievedSRSPbytes != ( ((npiMsgData_t *)rtis_ipc_srsp_buf)->len + RPC_FRAME_HDR_SZ))
		{
			printf("Mismatch in read IPC data and the expected length\n. \n\tread bytes = %d \n\tnpiMsgLength(+ 3) = %d \n\t\n",
					numOfReceievedSRSPbytes,
					((npiMsgData_t *)rtis_ipc_srsp_buf)->len + RPC_FRAME_HDR_SZ);
			exit(1);
		}

		// Copy response back in transmission buffer for processing
		memcpy((uint8*)pMsg, rtis_ipc_srsp_buf, numOfReceievedSRSPbytes);
	}
	else
	{
		if ( numOfReceievedSRSPbytes < 0)
			perror("recv");
		else
			printf("Server closed connection\n");
		exit(1);
	}

	// Now unlock the mutex before returning
	debug_printf("[MUTEX] Unlock SRSP Mutex\n");
	pthread_mutex_unlock(&rtisLnxClientSREQmutex);
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
	((uint8*)pMsg)[RPC_POS_CMD0] = (((uint8*)pMsg)[RPC_POS_CMD0] & RPC_SUBSYSTEM_MASK) | RPC_CMD_AREQ;

	int i;
	debug_printf("trying to send %d bytes,\t subSys 0x%.2X, cmdId 0x%.2X, pData:",
			pMsg->len,
			pMsg->subSys,
			pMsg->cmdId);
	debug_printf("\t");
	for (i = 0; i < pMsg->len; i++)
	{
		debug_printf(" 0x%.2X", pMsg->pData[i]);
	}
	debug_printf("\n");

	if (send(sNPIconnected, ((uint8*)pMsg), pMsg->len + RPC_FRAME_HDR_SZ , 0) == -1)
	{
		perror("send");
		exit(1);
	}
}

/**************************************************************************************************
 *
 * @fn          RTI_ReadItemEx
 *
 * @brief       This API is used to read an item from a Profile's Configuration Interface.
 *
 * input parameters
 *
 * @param       profileId - The Profile identifier.
 * @param       itemId - The Configuration Interface item identifier.
 * @param       len - The length in bytes of the item identifier's data.
 *
 * output parameters
 *
 * @param       *pValue - Pointer to buffer where read data is placed.
 *
 * @return      RTI_SUCCESS, RTI_ERROR_NOT_PERMITTED, RTI_ERROR_INVALID_INDEX,
 *              RTI_ERROR_INVALID_PARAMETER, RTI_ERROR_UNKNOWN_PARAMETER,
 *              RTI_ERROR_UNSUPPORTED_ATTRIBUTE, RTI_ERROR_OSAL_NV_OPER_FAILED,
 *              RTI_ERROR_OSAL_NV_ITEM_UNINIT, RTI_ERROR_OSAL_NV_BAD_ITEM_LEN
 *
 **************************************************************************************************/
rStatus_t RTI_ReadItemEx( uint8 profileId, uint8 itemId, uint8 len, uint8 *pValue )
{
  npiMsgData_t pMsg;

  // prep Read Item request
  // Note: no need to send pValue over the NPI
  pMsg.subSys   = RPC_SYS_RCAF;
  pMsg.cmdId    = RTIS_CMD_ID_RTI_READ_ITEM_EX;
  pMsg.len      = 3;
  pMsg.pData[0] = profileId;
  pMsg.pData[1] = itemId;
  pMsg.pData[2] = len;


  // send Read Item request to NPI socket synchronously
  NPI_SendSynchData( &pMsg );


  // DEBUG
  if ( pMsg.pData[0] == RTI_ERROR_SYNCHRONOUS_NPI_TIMEOUT )
  {
//    rtisFatalError( RTI_ERROR_SYNCHRONOUS_NPI_TIMEOUT );
  }

  // copy the reply data to the client's buffer
  // Note: the first byte of the payload is reserved for the status
  msg_memcpy( pValue, &pMsg.pData[1], len );

  // perform endianness change
  rtisAttribEConv( itemId, len, pValue );

  // return the status, which is stored is the first byte of the payload
  return( (rStatus_t)pMsg.pData[0] );
}

/**************************************************************************************************
 *
 * @fn          RTI_ReadItem
 *
 * @brief       This API is used to read the RTI Configuration Interface item
 *              from the Configuration Parameters table, the State Attributes
 *              table, or the Constants table.
 *
 * input parameters
 *
 * @param       itemId - The Configuration Interface item identifier.
 * @param       len - The length in bytes of the item identifier's data.
 *
 * output parameters
 *
 * @param       *pValue - Pointer to buffer where read data is placed.
 *
 * @return      RTI_SUCCESS, RTI_ERROR_NOT_PERMITTED, RTI_ERROR_INVALID_INDEX,
 *              RTI_ERROR_INVALID_PARAMETER, RTI_ERROR_UNKNOWN_PARAMETER,
 *              RTI_ERROR_UNSUPPORTED_ATTRIBUTE, RTI_ERROR_OSAL_NV_OPER_FAILED,
 *              RTI_ERROR_OSAL_NV_ITEM_UNINIT, RTI_ERROR_OSAL_NV_BAD_ITEM_LEN
 *
 **************************************************************************************************/
rStatus_t RTI_ReadItem( uint8 itemId, uint8 len, uint8 *pValue )
{
  npiMsgData_t pMsg;

  // prep Read Item request
  // Note: no need to send pValue over the NPI
  pMsg.subSys   = RPC_SYS_RCAF;
  pMsg.cmdId    = RTIS_CMD_ID_RTI_READ_ITEM;
  pMsg.len      = 2;
  pMsg.pData[0] = itemId;
  pMsg.pData[1] = len;

  // send Read Item request to NP RTIS synchronously
  NPI_SendSynchData( &pMsg );

  // DEBUG
  if ( pMsg.pData[0] == RTI_ERROR_SYNCHRONOUS_NPI_TIMEOUT )
  {
//    rtisFatalError( RTI_ERROR_SYNCHRONOUS_NPI_TIMEOUT );
  }

  // copy the reply data to the client's buffer
  // Note: the first byte of the payload is reserved for the status
  msg_memcpy( pValue, &pMsg.pData[1], len );

  // perform endianness change
  rtisAttribEConv( itemId, len, pValue );

  // return the status, which is stored is the first byte of the payload
  return( (rStatus_t)pMsg.pData[0] );
}

/**************************************************************************************************
 *
 * @fn          RTI_WriteItemEx
 *
 * @brief       This API is used to write an item to a Profile's Configuration Interface.
 *
 * input parameters
 *
 * @param       profileId - The Profile identifier.
 * @param       itemId - The Configuration Interface item identifier.
 * @param       len - The length in bytes of the item identifier's data.
 * @param       *pValue - Pointer to buffer where write data is stored.
 *
 * input parameters
 *
 * None.
 *
 * @return      RTI_SUCCESS, RTI_ERROR_NOT_PERMITTED, RTI_ERROR_INVALID_INDEX,
 *              RTI_ERROR_INVALID_PARAMETER, RTI_ERROR_UNKNOWN_PARAMETER,
 *              RTI_ERROR_UNSUPPORTED_ATTRIBUTE, RTI_ERROR_OSAL_NV_OPER_FAILED,
 *              RTI_ERROR_OSAL_NV_ITEM_UNINIT, RTI_ERROR_OSAL_NV_BAD_ITEM_LEN
 *
 **************************************************************************************************/
rStatus_t RTI_WriteItemEx( uint8 profileId, uint8 itemId, uint8 len, uint8 *pValue )
{
  npiMsgData_t pMsg;

  // prep Write Item request
  pMsg.subSys   = RPC_SYS_RCAF;
  pMsg.cmdId    = RTIS_CMD_ID_RTI_WRITE_ITEM_EX;
  pMsg.len      = 3+len;
  pMsg.pData[0] = profileId;
  pMsg.pData[1] = itemId;
  pMsg.pData[2] = len;

  // copy the client's data to be sent
  msg_memcpy( &pMsg.pData[3], pValue, len );

  // perform endianness change
  rtisAttribEConv( itemId, len, &pMsg.pData[3] );

  // send Write Item request to NP RTIS synchronously
  NPI_SendSynchData( &pMsg );

  // DEBUG
  if ( pMsg.pData[0] == RTI_ERROR_SYNCHRONOUS_NPI_TIMEOUT )
  {
//    rtisFatalError( RTI_ERROR_SYNCHRONOUS_NPI_TIMEOUT );
  }

  // return the status, which is stored is the first byte of the payload
  return( (rStatus_t)pMsg.pData[0] );
}

/**************************************************************************************************
 *
 * @fn          RTI_WriteItem
 *
 * @brief       This API is used to write RTI Configuration Interface parameters
 *              to the Configuration Parameters table, and permitted attributes
 *              to the State Attributes table.
 *
 * input parameters
 *
 * @param       itemId  - The Configuration Interface item identifier.
 * @param       len - The length in bytes of the item identifier's data.
 * @param       *pValue - Pointer to buffer where write data is stored.
 *
 * input parameters
 *
 * None.
 *
 * @return      RTI_SUCCESS, RTI_ERROR_NOT_PERMITTED, RTI_ERROR_INVALID_INDEX,
 *              RTI_ERROR_INVALID_PARAMETER, RTI_ERROR_UNKNOWN_PARAMETER,
 *              RTI_ERROR_UNSUPPORTED_ATTRIBUTE, RTI_ERROR_OSAL_NV_OPER_FAILED,
 *              RTI_ERROR_OSAL_NV_ITEM_UNINIT, RTI_ERROR_OSAL_NV_BAD_ITEM_LEN
 *
 **************************************************************************************************/
rStatus_t RTI_WriteItem( uint8 itemId, uint8 len, uint8 *pValue )
{
  npiMsgData_t pMsg;

  // prep Write Item request
  pMsg.subSys   = RPC_SYS_RCAF;
  pMsg.cmdId    = RTIS_CMD_ID_RTI_WRITE_ITEM;
  pMsg.len      = 2+len;
  pMsg.pData[0] = itemId;
  pMsg.pData[1] = len;

  // copy the client's data to be sent
  msg_memcpy( &pMsg.pData[2], pValue, len );

  // perform endianness change
  rtisAttribEConv( itemId, len, &pMsg.pData[2] );

  // send Write Item request to NP RTIS synchronously
  NPI_SendSynchData( &pMsg );

  // DEBUG
  if ( pMsg.pData[0] == RTI_ERROR_SYNCHRONOUS_NPI_TIMEOUT )
  {
//    rtisFatalError( RTI_ERROR_SYNCHRONOUS_NPI_TIMEOUT );
  }
  // DEBUG, test if RNP not lock in boot mode
  if ( pMsg.subSys == RPC_SYS_BOOT )
  {
    return( 1 );
  }

  // return the status, which is stored is the first byte of the payload
  return( (rStatus_t)pMsg.pData[0] );
}

/**************************************************************************************************
 *
 * @fn          RTI_InitReq
 *
 * @brief       This API is used to initialize the RemoTI stack and begin
 *              network operation. A RemoTI confirmation callback is generated
 *              and handled by the client.
 *
 *              The first thing this function does is take a snapshot of the
 *              Configuration Parameters (CP) table stored in NV memory, and
 *              only the snapshot will be used by RTI until another call is made
 *              to this function (presumably due to a reset). Therefore, any
 *              changes to the CP table must be made prior to calling this
 *              function. Once the RTI is started, subsequent changes by the
 *              client to the CP table can be made, but they will have no affect
 *              on RTI operation. The CP table is stored in NV memory and will
 *              persist across a device reset. The client can restore the
 *              the CP table to its default settings by setting the Startup
 *              Option parameter accordingly.
 *
 *              The client's confirm callback will provide a status, which can
 *              be one of the following:
 *
 *              RTI_SUCCESS
 *              RTI_ERROR_INVALID_PARAMTER
 *              RTI_ERROR_UNSUPPORTED_ATTRIBUTE
 *              RTI_ERROR_INVALID_INDEX
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
RTILIB_API void RTI_InitReq( void )
{
  npiMsgData_t pMsg;

  // prep Init request
  pMsg.subSys = RPC_SYS_RCAF;
  pMsg.cmdId  = RTIS_CMD_ID_RTI_INIT_REQ;
  pMsg.len    = 0;

  // send Init request to NP RTIS asynchronously as a confirm is due back
  NPI_SendAsynchData( &pMsg );
}

/**************************************************************************************************
 *
 * @fn          RTI_PairReq
 *
 * @brief       This API is used to initiate a pairing process. Note that this
 *              call actually consists of a discovery followed by pairing. That
 *              is a NLME-DISCOVERY.request followed by NLME-PAIR.request.
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
RTILIB_API void RTI_PairReq( void )
{
  npiMsgData_t pMsg;

  // prep Pair request
  pMsg.subSys = RPC_SYS_RCAF;
  pMsg.cmdId  = RTIS_CMD_ID_RTI_PAIR_REQ;
  pMsg.len    = 0;

  // send Pair request to NP RTIS asynchronously as a confirm is due back
  NPI_SendAsynchData( &pMsg );
}

/**************************************************************************************************
 *
 * @fn          RTI_PairAbortReq
 *
 * @brief       This API is used to abort an on-going pairing process.
 *
 *              The client's confirm callback will provide a status, which can
 *              be one of the following:
 *
 *              RTI_SUCCESS
 *              RTI_ERROR_PAIR_COMPLETE
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
RTILIB_API void RTI_PairAbortReq( void )
{
  npiMsgData_t pMsg;

  // prep Pair request
  pMsg.subSys = RPC_SYS_RCAF;
  pMsg.cmdId  = RTIS_CMD_ID_RTI_PAIR_ABORT_REQ;
  pMsg.len    = 0;

  // send Pair request to NP RTIS asynchronously as a confirm is due back
  NPI_SendAsynchData( &pMsg );
}

/**************************************************************************************************
 *
 * @fn          RTI_AllowPairReq
 *
 * @brief       This function is used by the Target application to ready the
 *              node for a pairing request, and thereby allow this node to
 *              respond.
 *
 *              The client's confirm callback will provide a status, which can
 *              be one of the following:
 *
 *              RTI_SUCCESS
 *              RTI_ERROR_OSAL_NO_TIMER_AVAIL
 *              RTI_ERROR_ALLOW_PAIRING_TIMEOUT
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
RTILIB_API void RTI_AllowPairReq( void )
{
  npiMsgData_t pMsg;

  // prep Pair request
  pMsg.subSys = RPC_SYS_RCAF;
  pMsg.cmdId  = RTIS_CMD_ID_RTI_ALLOW_PAIR_REQ;
  pMsg.len    = 0;

  // send Pair request to NP RTIS asynchronously as a confirm is due back
  NPI_SendAsynchData( &pMsg );
}

/**************************************************************************************************
 *
 * @fn          RTI_AllowPairAbortReq
 *
 * @brief       This API is used to attempt to abort an on-going allow-pairing process.
 *
 *              It is possible that allow pair is at a state of no return (no aborting).
 *              There is no callback associated to this function call.
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
RTILIB_API void RTI_AllowPairAbortReq( void )
{
  npiMsgData_t pMsg;

  // prep Pair request
  pMsg.subSys = RPC_SYS_RCAF;
  pMsg.cmdId  = RTIS_CMD_ID_RTI_ALLOW_PAIR_ABORT_REQ;
  pMsg.len    = 0;

  // send Pair request to NP RTIS asynchronously as a confirm is due back
  NPI_SendAsynchData( &pMsg );
}

/**************************************************************************************************
 *
 * @fn          RTI_UnpairReq
 *
 * @brief       This API is used to trigger un-pairing of a pair entry
 *
 * input parameters
 *
 * @param      dstIndex - destination index
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 *
 **************************************************************************************************/
RTILIB_API void RTI_UnpairReq( uint8 dstIndex )
{
  npiMsgData_t pMsg;

  // prep Pair request
  pMsg.subSys = RPC_SYS_RCAF;
  pMsg.cmdId  = RTIS_CMD_ID_RTI_UNPAIR_REQ;
  pMsg.len    = 1;
  pMsg.pData[0] = dstIndex;

  // send Pair request to NP RTIS asynchronously as a confirm is due back
  NPI_SendAsynchData( &pMsg );
}

/**************************************************************************************************
 *
 * @fn          RTI_SendDataReq
 *
 * @brief       This function sends data to the destination specified by the
 *              pairing table index.
 *
 * input parameters
 *
 * @param       dstIndex  - Pairing table index.
 * @param       profileId - Profile identifier.
 * @param       vendorId  - Vendor identifier.
 * @param       txOptions - Transmission options, as specified in Table 2 of the
 *                          RF4CE specification.
 * @param       len       - Number of bytes to send.
 * @param       *pData    - Pointer to buffer of data to be sent.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 *
 **************************************************************************************************/
RTILIB_API void RTI_SendDataReq( uint8 dstIndex, uint8 profileId, uint16 vendorId, uint8 txOptions, uint8 len, uint8 *pData )
{
  npiMsgData_t pMsg;

  // prep Send Data request
  pMsg.subSys   = RPC_SYS_RCAF;
  pMsg.cmdId    = RTIS_CMD_ID_RTI_SEND_DATA_REQ;
  pMsg.len      = 6+len;
  pMsg.pData[0] = dstIndex;
  pMsg.pData[1] = profileId;
  RTI_SET_ITEM_HALFWORD( &pMsg.pData[2], vendorId );
  pMsg.pData[4] = txOptions;
  pMsg.pData[5] = len;

  // copy the client's data to be sent
  msg_memcpy( &pMsg.pData[6], pData, len );

  // send Send Data request to NP RTIS synchronously
  NPI_SendAsynchData( &pMsg );
}

/**************************************************************************************************
 *
 * @fn          RTI_StandbyReq
 *
 * @brief       This API is used by the Target client to place this node into
 *              standby mode. Th properties of the standby consist of the active
 *              period and the duty cycle. These values are set in the
 *              Configuration Parameters table using the RTI_WriteItemReq API,
 *              and go into effect when standby is enabled for this node.
 *
 * input parameters
 *
 * @param       mode - RTI_STANDBY_ON, RTI_STANDBY_OFF
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 *
 **************************************************************************************************/
RTILIB_API void RTI_StandbyReq( uint8 mode )
{
  npiMsgData_t pMsg;

  // prep Standby request
  pMsg.subSys   = RPC_SYS_RCAF;
  pMsg.cmdId    = RTIS_CMD_ID_RTI_STANDBY_REQ;
  pMsg.len      = 1;
  pMsg.pData[0] = mode;

  // send Standby request to NP RTIS asynchronously as a confirm is due back
  NPI_SendAsynchData( &pMsg );
}

/**************************************************************************************************
 *
 * @fn          RTI_RxEnableReq
 *
 * @brief       This API is used to enable the radio receiver, enable the radio
 *              receiver for a specified amount of time, or disable the radio
 *              receiver.
 *
 * input parameters
 *
 * @param       duration - RTI_RX_ENABLE_ON, RTI_RX_ENABLE_OFF, 1..0xFFFE
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 *
 **************************************************************************************************/
RTILIB_API void RTI_RxEnableReq( uint16 duration )
{
  npiMsgData_t pMsg;

  // prep Rx Enable request
  pMsg.subSys = RPC_SYS_RCAF;
  pMsg.cmdId  = RTIS_CMD_ID_RTI_RX_ENABLE_REQ;
  pMsg.len    = 4;
  RTI_SET_ITEM_WORD( &pMsg.pData[0], (duration & 0x00FFFFFF) ); // max duration is 0x00FF_FFFF

  // send Rx Enable request to NP RTIS asynchronously as a confirm is due back
  NPI_SendAsynchData( &pMsg );
}

/**************************************************************************************************
 *
 * @fn          RTI_EnableSleepReq
 *
 * @brief       This API is used to enable sleep on the target.
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
RTILIB_API void RTI_EnableSleepReq( void )
{
  npiMsgData_t pMsg;

  // prep Enable Sleep request
  pMsg.subSys = RPC_SYS_RCAF;
  pMsg.cmdId  = RTIS_CMD_ID_RTI_ENABLE_SLEEP_REQ;
  pMsg.len    = 0;

  // send Enable Sleep request to NP RTIS asynchronously
  NPI_SendAsynchData( &pMsg );
}

/**************************************************************************************************
 *
 * @fn          RTI_DisableSleepReq
 *
 * @brief       This API is used to disable sleep on the target.
 *
 *              Note: When used from the RTIS, no actual message is sent to the
 *                    RTI, but wakeup bytes are sent instead. The RTI will
 *                    disable sleep as a result.
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
RTILIB_API void RTI_DisableSleepReq( void )
{
  npiMsgData_t pMsg;

  // ping NP; ping request will be discarded
  pMsg.subSys   = RPC_SYS_RCAF;
  pMsg.cmdId    = RTIS_CMD_ID_RTI_DISABLE_SLEEP_REQ; //RTIS_CMD_ID_TEST_PING_REQ;
  pMsg.len      = 2;
  pMsg.pData[0] = 0xAA;
  pMsg.pData[1] = 0xCC;

  // send command to slave
  NPI_SendAsynchData( &pMsg );
}

/**************************************************************************************************
 *
 * @fn          RTI_SwResetReq
 *
 * @brief       This function resets the radio processor CPU by way of software triggering.
 *              Implementation of this function is target (CPU) dependent.
 *              Note that in production platform, the reset could be done by chip reset signal
 *              (halResetSlave) and hence use of this function should be restricted to development
 *              phase.
 *
 * input parameters
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 *
 **************************************************************************************************/
RTILIB_API void RTI_SwResetReq( void )
{
  npiMsgData_t pMsg;

  pMsg.subSys   = RPC_SYS_RCAF;
  pMsg.cmdId    = RTIS_CMD_ID_RTI_SW_RESET_REQ;
  pMsg.len      = 0;

  // send command to slave
  NPI_SendAsynchData( &pMsg );

  // wait for 200ms.
//  halDelay(200, 1);
}

/**************************************************************************************************
 *
 * @fn          RTI_TestModeReq
 *
 * @brief       This function is used to place the radio in test modes.
 *              Note that implementation is chip dependent. HAL is not used to reduce code
 *              size overhead.
 *
 * input parameters
 *
 * @param       mode - test mode: RTI_TEST_MODE_TX_RAW_CARRIER, RTI_TEST_MODE_TX_RANDOM_DATA
 *                     or RTI_TEST_MODE_RX_AT_FREQ
 * @param       txPower - transmit power as negative dBm value. That is, 20 implies -20dBm.
 * @param       channel - MAC channel number
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 *
 **************************************************************************************************/
RTILIB_API void RTI_TestModeReq( uint8 mode, int8 txPower, uint8 channel )
{
  npiMsgData_t pMsg;

  pMsg.subSys   = RPC_SYS_RCAF;
  pMsg.cmdId    = RTIS_CMD_ID_RTI_TEST_MODE_REQ;
  pMsg.len      = 3;
  pMsg.pData[0] = mode;
  pMsg.pData[1] = (uint8) txPower;
  pMsg.pData[2] = channel;

  // send command to slave
  NPI_SendAsynchData( &pMsg );
}

/**************************************************************************************************
 *
 * @fn          RTI_TestRxCounterGetReq
 *
 * @brief       This function is used to obtain received packet counter value.
 *
 * input parameters
 *
 * @param       resetFlag - whether or not to reset the counter after reading the value
 *
 * output parameters
 *
 * None.
 *
 * @return      counter value
 *
 **************************************************************************************************/
RTILIB_API uint16 RTI_TestRxCounterGetReq(uint8 resetFlag)
{
  npiMsgData_t pMsg;

  // serialize the request
  pMsg.subSys   = RPC_SYS_RCAF;
  pMsg.cmdId    = RTIS_CMD_ID_RTI_RX_COUNTER_GET_REQ;
  pMsg.len      = 1;
  pMsg.pData[0] = resetFlag;

  // send serialized request to NP RTIS synchronously
  NPI_SendSynchData( &pMsg );

  // return the status, which is stored is the first byte of the payload
  return (pMsg.pData[0] + ((uint16)pMsg.pData[1] << 8));
}

/**************************************************************************************************
 * @fn          NPI_AsynchMsgCback
 *
 * @brief       This function is a NPI callback to the client that inidcates an
 *              asynchronous message has been received. The client software is
 *              expected to complete this call.
 *
 *              Note: The client must copy this message if it requires it
 *                    beyond the context of this call.
 *
 * input parameters
 *
 * @param       *pMsg - A pointer to an asychronously received message.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void NPI_AsynchMsgCback( npiMsgData_t *pMsg )
{
	if (rtisState != RTIS_STATE_READY)
	{
		return;
	}

	if (pMsg->subSys == RPC_SYS_RCAF)
	{
		switch( pMsg->cmdId )
		{
		// confirmation to init request
		case RTIS_CMD_ID_RTI_INIT_CNF:
			RTI_InitCnf( (rStatus_t)pMsg->pData[0] );
			break;

			// confirmation to pair request
		case RTIS_CMD_ID_RTI_PAIR_CNF:
			// status, pairing ref table index, pairing table device type
			RTI_PairCnf( (rStatus_t)pMsg->pData[0], pMsg->pData[1], pMsg->pData[2] );
			break;

			// confirmation to pair abort request
		case RTIS_CMD_ID_RTI_PAIR_ABORT_CNF:
			RTI_PairAbortCnf( (rStatus_t) pMsg->pData[0] );
			break;

			// confirmation to allow pair request
		case RTIS_CMD_ID_RTI_ALLOW_PAIR_CNF:
			RTI_AllowPairCnf( (rStatus_t) pMsg->pData[0], pMsg->pData[1], pMsg->pData[2]);
			break;

			// confirmation to send data request
		case RTIS_CMD_ID_RTI_SEND_DATA_CNF:
			RTI_SendDataCnf( (rStatus_t)pMsg->pData[0] );
			break;

			// indication of received data
		case RTIS_CMD_ID_RTI_REC_DATA_IND:
			RTI_ReceiveDataInd( pMsg->pData[0], pMsg->pData[1],
					pMsg->pData[2] | (pMsg->pData[3] << 8), // vendor Id
					pMsg->pData[4],
					pMsg->pData[5],
					pMsg->pData[6],
					&pMsg->pData[7]);
			break;

		case RTIS_CMD_ID_RTI_STANDBY_CNF:
			RTI_StandbyCnf( (rStatus_t) pMsg->pData[0] );
			break;

			// confirmation to send data request
		case RTIS_CMD_ID_RTI_ENABLE_SLEEP_CNF:
			RTI_EnableSleepCnf( (rStatus_t)pMsg->pData[0] );
			break;

			// confirmation to send data request
		case RTIS_CMD_ID_RTI_DISABLE_SLEEP_CNF:
			RTI_DisableSleepCnf( (rStatus_t)pMsg->pData[0] );
			break;

		case RTIS_CMD_ID_RTI_RX_ENABLE_CNF:
			RTI_RxEnableCnf( (rStatus_t ) pMsg->pData[0] );
			break;

		case RTIS_CMD_ID_RTI_UNPAIR_CNF:
			RTI_UnpairCnf( (rStatus_t) pMsg->pData[0],
					pMsg->pData[1] ); // dstIndex
			break;

		case RTIS_CMD_ID_RTI_UNPAIR_IND:
			RTI_UnpairInd( pMsg->pData[0] ); // dstIndex
			break;

		default:
			// nothing can be done here!
			break;
		}
	}
	else if (pMsg->subSys == RPC_SYS_RCN_CLIENT)
	{
#ifdef _WIN32 // TODO: remove this compile flag once RCNS is ported to an app processor
		RCNS_AsynchMsgCback(pMsg);
#endif
	}
}


/**************************************************************************************************
 * @fn          rtisAttribEConv
 *
 * @brief       This function converts endianness of an attribute value if the current machine
 *              is a big endian machine.
 *              This function will not do anything if the machine is a little endian machine.
 *
 * input parameters
 *
 * @param       attrib - attribute identifier
 * @param       len    - length of the value buffer
 * @param       pValue - buffer where attribute value is stored
 *
 * output parameters
 *
 * @param       pValue - the buffer is rewritten with swapped endianness value.
 *
 * @return      None.
 **************************************************************************************************
 */
static void rtisAttribEConv( uint8 attrib, uint8 len, uint8 *pValue )
{
  if (rtisBE)
  {
    if (attrib == RTI_SA_ITEM_PT_CURRENT_ENTRY)
    {
      if (len >= sizeof(rcnNwkPairingEntry_t))
      {
        // Note that endianness conversion will not occur if the retrieved attribute length is
        // smaller.
        rcnNwkPairingEntry_t *pEntry = (rcnNwkPairingEntry_t *) pValue;

        pEntry->srcNwkAddress = RTI_ECONV16(pEntry->srcNwkAddress);
        // Note that IEEE address is not converted. It is always supposed to be little endian
        pEntry->panId = RTI_ECONV16(pEntry->panId);
        pEntry->nwkAddress = RTI_ECONV16(pEntry->nwkAddress);
        pEntry->recFrameCounter = RTI_ECONV32(pEntry->recFrameCounter);
        pEntry->vendorIdentifier = RTI_ECONV16(pEntry->vendorIdentifier);
      }
    }
    else
    {
      // all other attributes are natural number
      uint8 i, j, buf;

      for (i = 0, j = len - 1; i < j; i++, j--)
      {
        buf = pValue[i];
        pValue[i] = pValue[j];
        pValue[j] = buf;
      }
    }
  }
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
