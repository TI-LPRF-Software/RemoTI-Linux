/**************************************************************************************************
  Filename:       npi_lnx_uart.c
  Revised:        $Date: 2012-05-10 09:15:45 -0700 (Thu, 10 May 2012) $
  Revision:       $Revision: 310 $

  Description:    This file contains linux specific implementation of Network Processor Interface
                  module.


  Copyright (C) {2015} Texas Instruments Incorporated - http://www.ti.com/


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

#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/signal.h>
#include <fcntl.h>
#include <termios.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <errno.h>
#include <unistd.h>
#include <stdio.h>
#include <semaphore.h>

#include "aic.h"
#include "npi_lnx.h"
#include "npi_lnx_uart.h"
#include "time_printf.h"

#include "npi_lnx_error.h"

#define NPI_UART	TRUE

#if (defined NPI_UART) && (NPI_UART == TRUE)
// -- macros --

#ifndef TRUE
# define TRUE (1)
#endif

#ifndef FALSE
# define FALSE (0)
#endif

#define error_printf(fmt, ...) fprintf(stderr, fmt, ##__VA_ARGS__)

//#ifdef __BIG_DEBUG__
//#define debug_printf(fmt, ...) fprintf( stderr, fmt, ##__VA_ARGS__); fflush(stdout);
//#else
#define debug_printf(fmt) st (if (__BIG_DEBUG_ACTIVE == TRUE) time_printf(fmt);)
//#endif

// -- Constants --

// Header portion of npiMsgData_t data structure
// The constant may have to change as npiMsgData_t structure changes.
#define NPI_MSG_DATA_HDR_LEN (sizeof(npiMsgData_t) - AP_MAX_BUF_LEN)

// Size of header for callback data buffer, which should include linked list
// pointers for queuing
#define NPI_CBACK_BUF_HDR_LEN (sizeof(npiAsyncDataHdr_t) + NPI_MSG_DATA_HDR_LEN)

// Time out value for response from RNP
#define NPI_RNP_TIMEOUT 2 // 6 seconds

// UART Baud rate
#define NPI_BAUDRATE B115200
// UART Flow Control
#define NPI_FLOWCONTROL	0

// State values for UART frame parsing
#define SOP_STATE      0x00
#define CMD_STATE1     0x01
#define CMD_STATE2     0x02
#define LEN_STATE1     0x03
#define LEN_STATE2     0x04
#define DATA_STATE     0x05
#define FCS_STATE      0x06

#define NAME_ELEMENT(element) [element] = #element
const char *parseStateStr[FCS_STATE + 1] =
{
	NAME_ELEMENT(SOP_STATE),
	NAME_ELEMENT(CMD_STATE1),
	NAME_ELEMENT(CMD_STATE2),
	NAME_ELEMENT(LEN_STATE1),
	NAME_ELEMENT(LEN_STATE2),
	NAME_ELEMENT(DATA_STATE),
	NAME_ELEMENT(FCS_STATE),
};


// -- Typedefs --
typedef struct _npiAsyncDataHdr_t
{
	struct _npiAsyncDataHdr_t *pNext;
} npiAsyncDataHdr_t;

typedef struct _npi_parseinfo_str {
	int state;
	uint16 LEN_Token;
	uint8 CMD0_Token;
	uint8 CMD1_Token;
	uint8 FSC_Token;
	uint16 tempDataLen;
	uint8 *pMsg;
} npi_parseinfo_t;

// -- Global Variables --
npi_tracehook_t npi_tracehook_rx = NULL;
npi_tracehook_t npi_tracehook_tx = NULL;

// -- Local Variables --

// State variable used to indicate that a device is open.
static int npiOpenFlag = FALSE;

static npiUartCfg_t uartCfg = {NPI_BAUDRATE, NPI_FLOWCONTROL};

// mutex to protect write calls
static pthread_mutex_t npi_write_mutex;

// pointer to synchronous response data buffer
static npiMsgData_t *pNpiSyncData;
// conditional variable for synchronous response
static pthread_cond_t npiSyncRespCond;
static pthread_mutex_t npiSyncRespLock;

// conditional variable to wake up asynchronous callback thread
static pthread_cond_t npiAsyncCond;
static pthread_mutex_t npiAsyncLock;
static int npiAsyncTerminate;

// conditional variable to wake up UART sleep disabling thread
static pthread_cond_t npiUartWakeupCond;
static pthread_mutex_t npiUartWakeupLock;

// conditional variable to wake up UART receive thread
static pthread_cond_t npi_rx_cond;
static pthread_mutex_t npi_rx_mutex;
static pthread_mutex_t npi_rx_signal_mutex;
//static sem_t signal_mutex;
// callback thread
static pthread_t npiAsyncCbackThread;

// UART receive thread
static pthread_t npiRxThread;

// linked list pointers for asynchronous message reception
static npiAsyncDataHdr_t *pNpiAsyncQueueHead;
static npiAsyncDataHdr_t *pNpiAsyncQueueTail;

// received frame parsing state
static int npi_rx_terminate;
static npi_parseinfo_t npi_parseinfo;

// UART device related variables
static int npi_fd;
static struct termios npi_oldtio;

// -- Forward references of local functions --

// mutex and conditional variable initialization and destruction
static void npi_initsyncres(void);
static void npi_delsyncres(void);

// thread termination subroutines
static void npi_termasync(void);
static void npi_termrx(void);

// uart subroutines
static int npi_opentty(const char *devpath);
static void npi_closetty(void);
static int npi_write(const void *buf, size_t count);
static int npi_sendframe(uint8 subsystem, uint8 cmd, uint8 *data, uint16 len);
static int npi_parseframe(const unsigned char *buf, int len);
static int npi_procframe( uint8 subsystemId, uint8 commandId, uint8 *pBuf, uint16 length);
static uint8 npi_calcfcs(uint16 len, uint8 cmd0, uint8 cmd1, uint8 *data);
static void npi_iohandler(int status);
static void npi_installsig(void);

// thread entry routines
static void *npiAsyncCbackProc(void *ptr);
static void *npi_rx_entry(void *ptr);

// -- Public functions --

/******************************************************************************
 * @fn         NPI_UART_OpenDevice
 *
 * @brief      This function establishes a serial communication connection with
 *             a network processor device.
 *             As windows machine does not have a single dedicated serial
 *             interface, this function will designate which serial port shall
 *             be used for communication.
 *
 * input parameters
 *
 * @param      portName � name of the serial port
 *
 * output parameters
 *
 * None.
 *
 * @return     TRUE if the connection is established successfully.
 *             FALSE, otherwise.
 ******************************************************************************
 */
int NPI_UART_OpenDevice(const char *portName, void *pCfg)
{

	if (pCfg != NULL)
	{
		uartCfg.speed = ((npiUartCfg_t *)pCfg)->speed;
		uartCfg.flowcontrol = ((npiUartCfg_t *)pCfg)->flowcontrol;
	}
	else
	{
		uartCfg.speed = NPI_BAUDRATE;
		uartCfg.flowcontrol = NPI_FLOWCONTROL;
	}

	if (npiOpenFlag)
	{
		npi_ipc_errno = NPI_LNX_ERROR_UART_OPEN_ALREADY_OPEN;
		return NPI_LNX_FAILURE;
	}

	npiOpenFlag = TRUE;

	// initialize thread synchronization resources
	npi_initsyncres();

	// initialize UART receive thread related variables
	npi_parseinfo.state = SOP_STATE;
	npi_rx_terminate = 0;

	// initialize async callback thread variables
	pNpiAsyncQueueHead = NULL;
	pNpiAsyncQueueTail = NULL;
	npiAsyncTerminate = 0;

	// initialize sync call variable
	pNpiSyncData = NULL;

	// create asynchronous callback thread
	if (pthread_create(&npiAsyncCbackThread, NULL, npiAsyncCbackProc, NULL )) {
		// thread creation failed
		npiOpenFlag = FALSE;

		npi_delsyncres();

		npi_ipc_errno = NPI_LNX_ERROR_UART_OPEN_FAILED_ASYNCH_CB_THREAD;
		return NPI_LNX_FAILURE;
	}

	// Open UART port
	char tmpStr[256];
	snprintf(tmpStr, sizeof(tmpStr), "[UART] Opening device %s\n", portName);
	debug_printf(tmpStr);
	if (npi_opentty(portName)) {
		// device open failed
		npi_termasync();
		npiOpenFlag = FALSE;

		npi_ipc_errno = NPI_LNX_ERROR_UART_OPEN_FAILED_DEVICE;
		return NPI_LNX_FAILURE;
	}


	// create UART receive thread

	// TODO: it is ideal to make this thread higher priority
	// but linux does not allow realtime of FIFO scheduling policy for
	// non-priviledged threads.
	if (pthread_create(&npiRxThread, NULL, npi_rx_entry, NULL)) {
		// thread creation failed
		npi_termasync();
		npi_closetty();

		npi_delsyncres();

		npiOpenFlag = FALSE;

		npi_ipc_errno = NPI_LNX_ERROR_UART_OPEN_FAILED_RX_THREAD;
		return NPI_LNX_FAILURE;
	}

	return NPI_LNX_SUCCESS;
}

/******************************************************************************
 * @fn         NPI_UART_CloseDevice
 *
 * @brief      This function closes connection with a network processor device
 *
 * input parameters
 *
 * @param      pDevice   - pointer to a device data structure
 *
 * output parameters
 *
 * None.
 *
 * @return     STATUS
 ******************************************************************************
 */
void NPI_UART_CloseDevice(void)
{
	debug_printf("[UART] UART device closing... \n");
	npi_termrx();
	debug_printf("[UART] UART thread closed... \n");
	npi_closetty();
	npi_termasync();

	npi_delsyncres();

	npiOpenFlag = FALSE;

	debug_printf("[UART] UART device closed\n");

}

/**************************************************************************************************
 * @fn          NPI_UART_SendAsynchData
 *
 * @brief       This function is called by the client when it has data ready to
 *              be sent asynchronously. This routine allocates an AREQ buffer,
 *              copies the client's payload, and sets up the send.
 *
 * input parameters
 *
 * @param *pMsg  - Pointer to data to be sent asynchronously (i.e. AREQ).
 *
 * output parameters
 *
 * None.
 *
 * @return      STATUS
 **************************************************************************************************
 */
int NPI_UART_SendAsynchData( npiMsgData_t *pMsg )
{
	return npi_sendframe(pMsg->subSys, pMsg->cmdId, pMsg->pData, pMsg->len);
}


/**************************************************************************************************
 * @fn          NPI_UART_SendSynchData
 *
 * @brief       This function is called by the client when it has data ready to
 *              be sent synchronously. This routine allocates a SREQ buffer,
 *              copies the client's payload, sends the data, and waits for the
 *              reply. The input buffer is used for the output data.
 *
 * input parameters
 *
 * @param *pMsg  - Pointer to data to be sent synchronously (i.e. the SREQ).
 *
 * output parameters
 *
 * @param *pMsg  - Pointer to replay data (i.e. the SRSP).
 *
 * @return      None.
 **************************************************************************************************
 */
int NPI_UART_SendSynchData( npiMsgData_t *pMsg )
{
	char tmpStr[AP_MAX_BUF_LEN * 3];
	int result, ret = NPI_LNX_SUCCESS;
	struct timespec expirytime;
	struct timeval currentTime;

	pthread_mutex_lock(&npiSyncRespLock);
	pNpiSyncData = pMsg;
	ret = npi_sendframe(pMsg->subSys | RPC_CMD_SREQ, pMsg->cmdId, pMsg->pData, pMsg->len);
	// Clear buffer
	pMsg->subSys = RPC_SYS_RES0 | RPC_CMD_RES6;

	if (ret == NPI_LNX_SUCCESS)
	{
		gettimeofday(&currentTime, NULL);
		expirytime.tv_sec = currentTime.tv_sec + NPI_RNP_TIMEOUT;
		expirytime.tv_nsec = currentTime.tv_usec * 1000;
		snprintf(tmpStr, sizeof(tmpStr), "[%s] Conditional wait %d.%ld\n",
				__FUNCTION__,
				(int)(expirytime.tv_sec - currentTime.tv_sec),
				expirytime.tv_nsec - (currentTime.tv_usec * 1000));
		debug_printf(tmpStr);
		// Response may have already come in. This may be the case for Serial Bootloader which is really fast.
		if ( ((pMsg->subSys & RPC_CMD_TYPE_MASK) == RPC_CMD_SRSP) ||
			 ((pMsg->subSys & RPC_SUBSYSTEM_MASK) == RPC_SYS_BOOT) )
		{
			snprintf(tmpStr, sizeof(tmpStr),
					"[%s] Synchronous Response received early, subSys 0x%.2X, cmdId 0x%.2X, pData[0] 0x%.2X\n",
					__FUNCTION__, pMsg->subSys, pMsg->cmdId, pMsg->pData[0]);
			debug_printf(tmpStr);
		}
		else
		{
			result = pthread_cond_timedwait(&npiSyncRespCond, &npiSyncRespLock, &expirytime);
			if (ETIMEDOUT == result)
			{
				// TODO: Indicate synchronous transaction error
				// Uncommenting the following line will cause assert when connection is not there
				// Uncomment it only during debugging.
				pMsg->pData[0] = 0xff;
				snprintf(tmpStr, sizeof(tmpStr), "[%s] Send synch data timed out\n", __FUNCTION__);
				debug_printf(tmpStr);
				npi_ipc_errno = NPI_LNX_ERROR_UART_SEND_SYNCH_TIMEDOUT;
				ret = NPI_LNX_FAILURE;
			}
			else
			{
				snprintf(tmpStr, sizeof(tmpStr), "[%s] Did not time out\n", __FUNCTION__);
				debug_printf(tmpStr);
			}
		}
		if ( ((pMsg->subSys & RPC_SUBSYSTEM_MASK) == RPC_SYS_BOOT) &&
			 ((pMsg->subSys & RPC_CMD_TYPE_MASK) == RPC_CMD_SREQ) )
		{
			// There is a bug in early versions of the UART serial bootloader where the request type is SREQ,
			// although it should be SRSP
			pMsg->subSys = RPC_SYS_BOOT | RPC_CMD_SRSP;
		}
	}
	// Clear the pointer.
	pNpiSyncData = NULL;
	pthread_mutex_unlock(&npiSyncRespLock);

	return ret;
}

/**************************************************************************************************
 * @fn          npiUartDisableSleep
 *
 * @brief       This function takes the NP node out of sleep and gives it time
 *              to prepare to receive its first UART data.
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void npiUartDisableSleep( void )
{
	static uint8 pBuf[] = { 0x00 };
	struct timespec expirytime;
	struct timespec curtime;

	// wait for a signal triggered by a character received only after
	// sending wakeup character.
	pthread_mutex_lock(&npiUartWakeupLock);

	// Send wakeup character
	npi_write(pBuf, sizeof(pBuf));

	// wait 20 seconds for wakeup
	// The timeout is handy for the PC host application to move on when RNP goes wrong.
	clock_gettime(CLOCK_MONOTONIC, &curtime);
	expirytime.tv_sec = curtime.tv_sec + NPI_RNP_TIMEOUT;
	expirytime.tv_nsec = curtime.tv_nsec;
	pthread_cond_timedwait(&npiUartWakeupCond, &npiUartWakeupLock, &expirytime);

	pthread_mutex_unlock(&npiUartWakeupLock);
}

// -- private functions --

/* Initialize thread synchronization resources */
static void npi_initsyncres(void)
{
	// initialize all mutexes
	pthread_mutex_init(&npi_write_mutex, NULL);
	pthread_mutex_init(&npiSyncRespLock, NULL);
	pthread_mutex_init(&npiAsyncLock, NULL);
	pthread_mutex_init(&npiUartWakeupLock, NULL);
	pthread_mutex_init(&npi_rx_mutex, NULL);
	pthread_mutex_init(&npi_rx_signal_mutex, NULL);
//	sem_init(&signal_mutex,0,1);

	// initialize all conditional variables
	pthread_cond_init(&npiSyncRespCond, NULL);
	pthread_cond_init(&npiAsyncCond, NULL);
	pthread_cond_init(&npiUartWakeupCond, NULL);
	pthread_cond_init(&npi_rx_cond, NULL);
}

/* Destroy thread synchronization resources */
static void npi_delsyncres(void)
{
	// In Linux, there is no dynamically allocated resources
	// and hence the following calls do not actually matter.

	// destroy all conditional variables
	pthread_cond_destroy(&npiSyncRespCond);
	pthread_cond_destroy(&npiAsyncCond);
	pthread_cond_destroy(&npiUartWakeupCond);
	pthread_cond_destroy(&npi_rx_cond);

	// destroy all mutexes
	pthread_mutex_destroy(&npi_write_mutex);
	pthread_mutex_destroy(&npiSyncRespLock);
	pthread_mutex_destroy(&npiAsyncLock);
	pthread_mutex_destroy(&npiUartWakeupLock);
	pthread_mutex_destroy(&npi_rx_mutex);
	pthread_mutex_destroy(&npi_rx_signal_mutex);
//	sem_destroy(&signal_mutex);

}

// Entry function for a thread calling NPI_AsynchMsgCback() functions
// upon receipt of asynchronous messages
// This separate thread is required to prevent deadlock situation where a
// AIC callback thread is locked as the callback function called another
// NPI function.
static void *npiAsyncCbackProc(void *ptr)
{
	char tmpStr[AP_MAX_BUF_LEN * 3];
	npiAsyncDataHdr_t *pElement = NULL;
	int ret = NPI_LNX_SUCCESS;
	((void)ptr);
	pthread_mutex_lock(&npiAsyncLock);
	for (;;)
	{
		do
		{
			if (npiAsyncTerminate)
			{
				// termination was signaled
				break;
			}
			pElement = pNpiAsyncQueueTail;
			if (pElement)
			{
				pNpiAsyncQueueTail = pElement->pNext;
				if (!pNpiAsyncQueueTail)
				{
					pNpiAsyncQueueHead = pNpiAsyncQueueTail;
				}
				snprintf(tmpStr, sizeof(tmpStr), "[%s] about to call NPI_AsynchMsgCback \n\tpElement @%p\n\tpNpiAsyncQueueHead @%p\n\tpNpiAsyncQueueTail @%p\n",
						__FUNCTION__, pElement, pNpiAsyncQueueHead, pNpiAsyncQueueTail);
				debug_printf(tmpStr);
				// unlock mutex so that UART thread can still signal conditional variable
				// and does not get blocked till NPI async callback function returns.
				pthread_mutex_unlock(&npiAsyncLock);

				// callback
				ret = NPI_AsynchMsgCback((npiMsgData_t *)(pElement + 1));
				free(pElement);
				pElement = NULL;

				snprintf(tmpStr, sizeof(tmpStr), "[%s] finished NPI_AsynchMsgCback \n\tpElement @%p\n\tpNpiAsyncQueueHead @%p\n\tpNpiAsyncQueueTail @%p\n",
						__FUNCTION__, pElement, pNpiAsyncQueueHead, pNpiAsyncQueueTail);
				debug_printf(tmpStr);

				// lock the mutex again before checking the terminate condition and queue
				// so that it is OK to miss conditional variable signal while the mutex was
				// unlocked.
				pthread_mutex_lock(&npiAsyncLock);

				if (ret != NPI_LNX_SUCCESS)
					npiAsyncTerminate = 1;
			}
		} while (pNpiAsyncQueueTail);

		if (npiAsyncTerminate)
		{
			// break out of outer loop, too.
			break;
		}

		// wait for signal
		pthread_cond_wait(&npiAsyncCond, &npiAsyncLock);
		snprintf(tmpStr, sizeof(tmpStr), "[%s] got signal \n\tpNpiAsyncQueueHead @%p\n\tpNpiAsyncQueueTail @%p\n", __FUNCTION__,
				pNpiAsyncQueueHead, pNpiAsyncQueueTail);
		debug_printf(tmpStr);
	}

	// thread is to be terminated
	pthread_mutex_unlock(&npiAsyncLock);


	char *errorMsg;
	if (ret == NPI_LNX_FAILURE)
		errorMsg = "UART Asynch call back thread exited with error. Please check global error message\n";
	else
		errorMsg = "UART Asynch call back thread exited without error\n";

	NPI_LNX_IPC_NotifyError(NPI_LNX_ERROR_MODULE_MASK(NPI_LNX_ERROR_UART_ASYNCH_CB_PROC_THREAD), errorMsg);

	return NULL;
}

/* UART RX thread entry routine */
static void *npi_rx_entry(void *ptr)
{
	unsigned char readbuf[AP_MAX_BUF_LEN];
	char tmpStr[AP_MAX_BUF_LEN * 3];
	int ret = NPI_LNX_SUCCESS;

	/* install signal handler */
	//npi_installsig();

	debug_printf("[UART] Wait for mutex in rx entry:\n");
	
	/* lock mutex in order not to lose signal */
	pthread_mutex_lock(&npi_rx_mutex);

	/* thread loop */
	while (!npi_rx_terminate)
	{
		int readcount;

		do
		{
			readcount = read(npi_fd, readbuf, sizeof(readbuf));
			if (readcount > 0)
			{
				snprintf(tmpStr, sizeof(tmpStr), "[%s] Read %d bytes\n", __FUNCTION__, readcount);
				debug_printf(tmpStr);
				ret = npi_parseframe(readbuf, readcount);
				if (ret == NPI_LNX_FAILURE)
					break;
			}
			else if (readcount < 0)
			{
				snprintf(tmpStr, sizeof(tmpStr), "[%s] Errno %d\n", __FUNCTION__, errno);
				debug_printf(tmpStr);
				// Ignore error EAGAIN; Resource temporarily unavailable.
				// EINTR; System Call interrupted, try again.
				// Try again if this was the first attempt after the conditional signal npi_rx_cond.
				if ((errno != EAGAIN) && (errno != EINTR))
				{
					perror("read npi_fd (1)");
					npi_ipc_errno = NPI_LNX_ERROR_UART_RX_THREAD;
					ret = NPI_LNX_FAILURE;
				}
			}
		} while (readcount > 0);

		if (ret == NPI_LNX_FAILURE)
		{
			npi_rx_terminate = 1;
			break;
		}
		else
		{
#ifdef NPI_UNRELIABLE_SIGACTION
			/* In some system, sigaction is not reliable hence poll reading even without signal. */
			{
				struct timespec expirytime;
				struct timespec curtime;

				int waitTime = 10000000;
				clock_gettime(CLOCK_MONOTONIC, &curtime);
				expirytime.tv_sec = curtime.tv_sec;
				expirytime.tv_nsec = (curtime.tv_nsec) + waitTime;
				if (expirytime.tv_nsec >= 1000000000) {
					expirytime.tv_nsec -= 1000000000;
					expirytime.tv_sec++;
				}
				debug_printf("[UART] (read loop) Conditional wait %d ns\n", waitTime);
				pthread_cond_timedwait(&npi_rx_cond, &npi_rx_mutex, &expirytime);
			}
#else
			snprintf(tmpStr, sizeof(tmpStr), "[%s] Going to wait for more data\n", __FUNCTION__);
			debug_printf(tmpStr);
			pthread_cond_wait(&npi_rx_cond, &npi_rx_mutex);
			snprintf(tmpStr, sizeof(tmpStr), "[%s] Got signal to get more data\n", __FUNCTION__);
			debug_printf(tmpStr);
//			sem_wait(&signal_mutex);
#endif
		}
	}
	snprintf(tmpStr, sizeof(tmpStr), "[%s] npi_rx_terminate == %d, unlocking mutex\n", __FUNCTION__, npi_rx_terminate);
	debug_printf(tmpStr);
	pthread_mutex_unlock(&npi_rx_mutex);

	char *errorMsg;
	if (ret == NPI_LNX_FAILURE)
		errorMsg = "UART Rx thread exited with error. Please check global error message\n";
	else
		errorMsg = "UART Rx thread exited without error\n";

	NPI_LNX_IPC_NotifyError(NPI_LNX_ERROR_MODULE_MASK(NPI_LNX_ERROR_UART_RX_THREAD), errorMsg);

	return NULL;
}

/* Terminate Asynchronous thread */
static void npi_termasync(void)
{
	// send terminate signal
	pthread_mutex_lock(&npiAsyncLock);
	npiAsyncTerminate = 1;
	pthread_cond_signal(&npiAsyncCond);
	pthread_mutex_unlock(&npiAsyncLock);

	// wait till the thread terminates
	pthread_join(npiAsyncCbackThread, NULL);
}

/* Terminate UART rx thread */
static void npi_termrx(void)
{
	// send terminate signal
	npi_rx_terminate = 1;
	debug_printf("[UART] [MUTEX] Signaling thread that we have terminated\n");
#ifdef NPI_UNRELIABLE_SIGACTION
	pthread_cond_signal(&npi_rx_cond);
#else
	pthread_mutex_lock(&npi_rx_mutex);
	pthread_cond_signal(&npi_rx_cond);
	pthread_mutex_unlock(&npi_rx_mutex);
//	sem_post(&signal_mutex);
#endif

	// wait till the thread terminates
	debug_printf("[UART] [MUTEX] Waiting for thread to finish termination\n");
	pthread_join(npiRxThread, NULL);
}

/* Open TTY device
 * return non-zero when failed to open the device. */
static int npi_opentty(const char *devpath)
{
	struct termios newtio;

	/* NOCTTY so that RNP cannot kill the current process by ^C */
	npi_fd = open(devpath, O_RDWR | O_NOCTTY | O_NONBLOCK);
	if (npi_fd < 0) {
		return npi_fd;
	}

	/* install signal handler */
	npi_installsig();

	/* make the file descriptor asynchronous */
	fcntl(npi_fd, F_SETFL, FASYNC | FNDELAY);

	/* save current port settings */
	tcgetattr(npi_fd, &npi_oldtio);

	/* clear new port setting structure */
	bzero(&newtio, sizeof(newtio));

	/* CRTSCTS : in case hardware flow control is used
	 * CS8     : 8n1 (8 bit, no parity, 1 stopbit)
	 * CLOCAL  : local connection, no modem control
	 * CREAD   : enable receiving characters
	 */
	unsigned short int bRate = B115200;
	switch (uartCfg.speed)
	{
	case 57600:
		bRate = B57600;
		break;
	case 115200:
		bRate = B115200;
		break;
	case 230400:
		bRate = B230400;
		break;
	default:
		bRate = B115200;
		break;
	}
	char tmpStr[256];
	snprintf(tmpStr, sizeof(tmpStr), "[UART] Baud rate set to %d (0x%.6X)\n", uartCfg.speed, bRate);
	debug_printf(tmpStr);
	if (uartCfg.flowcontrol == 1)
	{
		newtio.c_cflag = bRate | CS8 | CLOCAL | CREAD | CRTSCTS;
	}
	else
	{
		newtio.c_cflag = bRate | CS8 | CLOCAL | CREAD;
	}
	snprintf(tmpStr, sizeof(tmpStr), "[UART] c_cflag set to 0x%.6X\n", newtio.c_cflag);
	debug_printf(tmpStr);

	/* IGNPAR  : ignore bytes with parity errors
	 * ICRNL   : map CR to NL (not in use)
	 */
	newtio.c_iflag = IGNPAR;

	/* raw output */
	newtio.c_oflag = 0;

	newtio.c_cc[VTIME] = 0; /* inter-character timer unused */
	newtio.c_cc[VMIN]  = 1; /* blocking read until 1 char received */

	/* clean the modem line */
	tcflush(npi_fd, TCIFLUSH);

	/* set new attribute */
	tcsetattr(npi_fd, TCSANOW, &newtio);

	return 0;
}

/* Close open device */
static void npi_closetty(void)
{
	/* revert to old settings */
	tcsetattr(npi_fd, TCSANOW, &npi_oldtio);

	/* close the device */
	close(npi_fd);
}

/* wrapper of write function to make safe write operation
 * Maybe this function is not necessary if write function itself is thread-safe.
 */
static int npi_write(const void *buf, size_t count)
{
	int result = -1;

#ifdef __DEBUG_TIME__
	if (__DEBUG_TIME_ACTIVE == TRUE)
	{
		char tmpStr[512];
		snprintf(tmpStr, sizeof(tmpStr), "[UART] write %d bytes to %d\n", (int)count, npi_fd);
		debug_printf(tmpStr);
	}
#endif //__DEBUG_TIME__

	pthread_mutex_lock(&npi_write_mutex);
	do
	{
		result = write(npi_fd, buf, count);
	}
	while ((errno == EINTR) && (result == -1));
	pthread_mutex_unlock(&npi_write_mutex);

#ifdef __DEBUG_TIME__
	if (__DEBUG_TIME_ACTIVE == TRUE)
	{
		char tmpStr[512];
		snprintf(tmpStr, sizeof(tmpStr), "[UART] result: %d\n", (int)result);
		debug_printf(tmpStr);
	}
#endif //__DEBUG_TIME__

	return result;
}

/* build and send an NPI frame
 */
static int npi_sendframe(uint8 subsystem, uint8 cmd, uint8 *data, uint16 len)
{
	// Build a frame from the primitive ID and primitive content

	unsigned char *pBuf;
	size_t frlen = AIC_UART_FRAME_OVHD + AIC_FRAME_HDR_SZ + len; // frame length

	pBuf = (unsigned char *) malloc(frlen);

	if (!pBuf)
	{
		npi_ipc_errno = NPI_LNX_ERROR_UART_SEND_FRAME_FAILED_TO_ALLOCATE;
		return NPI_LNX_FAILURE;
	}

	pBuf[0] = AIC_UART_SOF;
	pBuf[1 + AIC_POS_LEN] = (len & 0x00FF);
	pBuf[1 + AIC_POS_LEN + 1] = (((len & 0xFF00) >> 8) & 0x00FF);
	pBuf[1 + AIC_POS_CMD0] = subsystem;
	pBuf[1 + AIC_POS_CMD1] = cmd;
	memcpy(&pBuf[1 + AIC_POS_DAT0], data, len);
	pBuf[1 + len + AIC_FRAME_HDR_SZ] = npi_calcfcs(len, subsystem, cmd, data);

	// Send the data over to the serial com device
	if (npi_write(pBuf, frlen) < 0) {
		perror("ERR:");
		free(pBuf);
		npi_ipc_errno = NPI_LNX_ERROR_UART_SEND_FRAME_FAILED_TO_WRITE;
		return NPI_LNX_FAILURE;
	}

	// Trace the sent data
	if (npi_tracehook_tx) {
		npi_tracehook_tx(subsystem, cmd, data, len);
	}

	free(pBuf);

	return NPI_LNX_SUCCESS;
}

/* Parse an NPI frame */
static int npi_parseframe(const unsigned char *buf, int len)
{
	int ret = NPI_LNX_SUCCESS;
	uint8 ch;
	char tmpStr[AP_MAX_BUF_LEN * 3];
	int  bytesInRxBuffer;

	if (len)
	{
		// fire UART wakeup signal
		pthread_mutex_lock(&npiUartWakeupLock);
		pthread_cond_signal(&npiUartWakeupCond);
		pthread_mutex_unlock(&npiUartWakeupLock);
	}

	while (len > 0)
	{
		ch = *buf++;
		len--;
		snprintf(tmpStr, sizeof(tmpStr), "[%s] ch: 0x%.2X, state: %s\n", __FUNCTION__, ch, parseStateStr[npi_parseinfo.state]);
		debug_printf(tmpStr);

		switch (npi_parseinfo.state)
		{
		case SOP_STATE:
			if (ch == AIC_UART_SOF)
			{
				npi_parseinfo.state = LEN_STATE1;
			}
			else
			{
				snprintf(tmpStr, sizeof(tmpStr), "[%s] non-SOF found: 0x%.2X\n", __FUNCTION__, ch);
				debug_printf(tmpStr);
			}
			break;

		case LEN_STATE1:
			npi_parseinfo.LEN_Token = ch;
			npi_parseinfo.state = LEN_STATE2;
			break;

		case LEN_STATE2:
			npi_parseinfo.LEN_Token += ((((uint16)ch) << 8) & 0xFF00);
			snprintf(tmpStr, sizeof(tmpStr), "Length: 0x%.6X, malloc(%d)\n",
					npi_parseinfo.LEN_Token, (int)(NPI_CBACK_BUF_HDR_LEN + npi_parseinfo.LEN_Token));
			debug_printf(tmpStr);

			npi_parseinfo.tempDataLen = 0;

			/* Allocate memory for the data */
			npi_parseinfo.pMsg = (uint8 *)malloc(
					NPI_CBACK_BUF_HDR_LEN + npi_parseinfo.LEN_Token );

			snprintf(tmpStr, sizeof(tmpStr), "npi_parseinfo.pMsg @%p\n", npi_parseinfo.pMsg);
			debug_printf(tmpStr);

			if (npi_parseinfo.pMsg)
			{
				/* Fill up what we can */
				npi_parseinfo.state = CMD_STATE1;
				break;
			}
			else
			{
				npi_parseinfo.state = SOP_STATE;
				break;
			}
			break;

		case CMD_STATE1:
			npi_parseinfo.CMD0_Token = ch;
			npi_parseinfo.state = CMD_STATE2;
			break;

		case CMD_STATE2:
			npi_parseinfo.CMD1_Token = ch;
			/* If there is no data, skip to FCS state */
			if (npi_parseinfo.LEN_Token)
			{
				npi_parseinfo.state = DATA_STATE;
			}
			else
			{
				npi_parseinfo.state = FCS_STATE;
			}
			break;

		case DATA_STATE:

			/* Fill in the buffer the first byte of the data */
			npi_parseinfo.pMsg[NPI_CBACK_BUF_HDR_LEN + npi_parseinfo.tempDataLen++] = ch;

			/* Check number of bytes left in the Rx buffer */
			bytesInRxBuffer = len;

			/* If the remainder of the data is there, read them all, otherwise, just read enough */
			if (bytesInRxBuffer <= npi_parseinfo.LEN_Token - npi_parseinfo.tempDataLen)
			{
				snprintf(tmpStr, sizeof(tmpStr), "[%s, %d] copying %d bytes, missing %d, already have %d of total %d\n",
						__FUNCTION__, __LINE__, bytesInRxBuffer, npi_parseinfo.LEN_Token - npi_parseinfo.tempDataLen,
						npi_parseinfo.tempDataLen, npi_parseinfo.LEN_Token);
				debug_printf(tmpStr);
				memcpy(&npi_parseinfo.pMsg[NPI_CBACK_BUF_HDR_LEN + npi_parseinfo.tempDataLen],
						buf, bytesInRxBuffer);
				buf += bytesInRxBuffer;
				len -= bytesInRxBuffer;
				npi_parseinfo.tempDataLen += (uint8) bytesInRxBuffer;
			}
			else
			{
				snprintf(tmpStr, sizeof(tmpStr), "[%s, %d] copying %d bytes of %d, already have %d of total %d\n",
						__FUNCTION__, __LINE__, npi_parseinfo.LEN_Token - npi_parseinfo.tempDataLen, bytesInRxBuffer,
						npi_parseinfo.tempDataLen, npi_parseinfo.LEN_Token);
				debug_printf(tmpStr);
				memcpy(&npi_parseinfo.pMsg[NPI_CBACK_BUF_HDR_LEN + npi_parseinfo.tempDataLen],
						buf, npi_parseinfo.LEN_Token - npi_parseinfo.tempDataLen);
				buf += npi_parseinfo.LEN_Token - npi_parseinfo.tempDataLen;
				len -= npi_parseinfo.LEN_Token - npi_parseinfo.tempDataLen;
				npi_parseinfo.tempDataLen += (npi_parseinfo.LEN_Token - npi_parseinfo.tempDataLen);
			}
			snprintf(tmpStr, sizeof(tmpStr), "[%s, %d] tempLen: %d of total %d\n", __FUNCTION__, __LINE__, npi_parseinfo.tempDataLen, npi_parseinfo.LEN_Token);
			debug_printf(tmpStr);

			/* If number of bytes read is equal to data length, time to move on to FCS */
			if ( npi_parseinfo.tempDataLen == npi_parseinfo.LEN_Token )
			{
				npi_parseinfo.state = FCS_STATE;
			}

			break;

		case FCS_STATE:

			npi_parseinfo.FSC_Token = ch;
			uint8 fcsCalculated = npi_calcfcs(
					npi_parseinfo.LEN_Token,
					npi_parseinfo.CMD0_Token,
					npi_parseinfo.CMD1_Token,
					&npi_parseinfo.pMsg[NPI_CBACK_BUF_HDR_LEN]);

			/* Make sure it's correct */
			if (fcsCalculated == npi_parseinfo.FSC_Token)
			{
				// Trace the received data
				if (npi_tracehook_rx)
				{
					npi_tracehook_rx(npi_parseinfo.CMD0_Token, npi_parseinfo.CMD1_Token,
							&npi_parseinfo.pMsg[NPI_CBACK_BUF_HDR_LEN],
							npi_parseinfo.LEN_Token);
				}
				snprintf(tmpStr, sizeof(tmpStr), "[%s] found frame, going to npi_procframe\n", __FUNCTION__);
				debug_printf(tmpStr);

				// process the received frame
				ret = npi_procframe(npi_parseinfo.CMD0_Token, npi_parseinfo.CMD1_Token,
						npi_parseinfo.pMsg, npi_parseinfo.LEN_Token);
			}
			else
			{
				snprintf(tmpStr, sizeof(tmpStr), "[%s] fcs failed 0x%.2X vs 0x%.2X, destroying the message:\n", __FUNCTION__,
						fcsCalculated, npi_parseinfo.FSC_Token);
				debug_printf(tmpStr);
				snprintf(tmpStr, sizeof(tmpStr), "[%s] len: 0x%.2X, cmd0: 0x%.2X, cmd1: 0x%.2X, FCS: 0x%.2X\n", __FUNCTION__,
						npi_parseinfo.LEN_Token,
						npi_parseinfo.CMD0_Token,
						npi_parseinfo.CMD1_Token,
						npi_parseinfo.FSC_Token);
				debug_printf(tmpStr);
				/* deallocate the msg */
				free ( (uint8 *)npi_parseinfo.pMsg );
			}

			/* Reset the state, send or discard the buffers at this point */
			npi_parseinfo.state = SOP_STATE;

			break;

		default:
			break;
		}
	}

	return ret;
}

/* Process received frame */
static int npi_procframe( uint8 subsystemId, uint8 commandId, uint8 *pBuf, uint16 length)
{
	int ret = NPI_LNX_SUCCESS;
	int i;
	char tmpStr[AP_MAX_BUF_LEN * 3];
	int strIndex = 0;

#ifndef SREQ_VIA_AREQ
	if ( ((subsystemId & RPC_CMD_TYPE_MASK) == RPC_CMD_SRSP) ||
			((subsystemId & RPC_SUBSYSTEM_MASK) == RPC_SYS_BOOT))
	{
		// synchronous response

		pthread_mutex_lock(&npiSyncRespLock);
		npiMsgData_t *pMsg = pNpiSyncData;
		if (pMsg)
		{
			// Fill in the synchronous response buffer
			memcpy(&pMsg->pData, pBuf + NPI_CBACK_BUF_HDR_LEN, length);
			pMsg->len = length;
			pMsg->subSys = subsystemId;
			pMsg->cmdId = commandId;
		}
		// if pointer is NULL, no action should be taken

		// the buffer usage is done, no more use expected
		free(pBuf);

		snprintf(tmpStr, sizeof(tmpStr), "[%s] signal synch response received (invoked by read loop)\n", __FUNCTION__);
		debug_printf(tmpStr);
		// Unblock the synchronous request call
		pthread_cond_signal(&npiSyncRespCond);
		pthread_mutex_unlock(&npiSyncRespLock);
	}
	else
#endif //SREQ_VIA_AREQ
	{
		// must be an asynchronous message

		// type cast to asynchronous message queue element data structure
		npiAsyncDataHdr_t *pElement = (npiAsyncDataHdr_t *) pBuf;

		// type cast the message structure
		npiMsgData_t *pMsg = (npiMsgData_t *) (pBuf + sizeof(npiAsyncDataHdr_t));

		if (__BIG_DEBUG_ACTIVE == TRUE)
		{
			snprintf(tmpStr, sizeof(tmpStr), "[%s] subsys: 0x%.2x, Cmd ID: 0x%.2X, length: %d, Data: ", __FUNCTION__, subsystemId, commandId, length);
			strIndex = strlen(tmpStr);
			for (i = 0 ; i < length; i++ )
			{
				snprintf(tmpStr + strIndex, sizeof(tmpStr) - strIndex, "%.2X ", pMsg->pData[i]);
				strIndex += 3;
			}
			snprintf(tmpStr + strIndex, sizeof(tmpStr) - strIndex, "\n");
			debug_printf(tmpStr);
		}

		// Fill in the message structure
		pMsg->len = length;
		pMsg->subSys = subsystemId;
		pMsg->cmdId = commandId;
		pElement->pNext = NULL;

		// queue the message
		pthread_mutex_lock(&npiAsyncLock);
		if (pNpiAsyncQueueHead)
		{
			pNpiAsyncQueueHead->pNext = pElement;
		}
		else
		{
			pNpiAsyncQueueTail = pElement;
		}
		pNpiAsyncQueueHead = pElement;

		snprintf(tmpStr, sizeof(tmpStr), "[%s] signal areq callback thread (invoked by read loop). \n\tpNpiAsyncQueueHead @%p\n\tpNpiAsyncQueueTail @%p\n",
				__FUNCTION__, pNpiAsyncQueueHead, pNpiAsyncQueueTail);
		debug_printf(tmpStr);
		/* wake up the asynchronous callback thread */
		pthread_cond_signal(&npiAsyncCond);
		pthread_mutex_unlock(&npiAsyncLock);
		snprintf(tmpStr, sizeof(tmpStr), "[%s] unlocked npiAsyncLock (invoked by read loop)\n", __FUNCTION__);
		debug_printf(tmpStr);
	}

	return ret;
}

/* Calculate NPI frame FCS */
static uint8 npi_calcfcs( uint16 len, uint8 cmd0, uint8 cmd1, uint8 *data_ptr )
{
	uint16 i;
	uint8 xorResult;

	uint8 len_lo = (len & 0x00FF);
	uint8 len_hi = (((len & 0xFF00) >> 8) & 0x00FF);
	xorResult = len_lo ^ len_hi ^ cmd0 ^ cmd1;
//	printf("\nlen_lo: 0x%.2X len_hi: 0x%.2X cmd0: 0x%.2X cmd1: 0x%.2X,", len_lo, len_hi, cmd0, cmd1);

	for ( i = 0; i < len; i++, data_ptr++ )
	{
		xorResult = xorResult ^ *data_ptr;
//		printf(" (%d)0x%.2X", x, *data_ptr);
	}
//	printf(" FCS: 0x%.2X -- 0x%.2X", *data_ptr, xorResult);
//	printf("\n");

	return ( xorResult );
}

static void npi_iohandler(int status)
{
	// This is a potential reentrant function.
	// Therefore used semaphore instead of mutex.

	((void)status);
	/* send wakeup signal */
	if (0 == pthread_mutex_trylock(&npi_rx_signal_mutex))
	{
#ifdef __DEBUG_TIME__
		if (__DEBUG_TIME_ACTIVE == TRUE)
		{
			debug_printf("Signal from UART, grabbing mutex\n");
		}
#endif //__DEBUG_TIME__

		pthread_mutex_lock(&npi_rx_mutex);
		pthread_cond_signal(&npi_rx_cond);
		pthread_mutex_unlock(&npi_rx_mutex);
		//	sem_post(&signal_mutex);
		pthread_mutex_unlock(&npi_rx_signal_mutex);

#ifdef __DEBUG_TIME__
		if (__DEBUG_TIME_ACTIVE == TRUE)
		{
			debug_printf("Signal read handle, owning mutex\n");
		}
#endif //__DEBUG_TIME__
	}
	else
	{
#ifdef __DEBUG_TIME__
		if (__DEBUG_TIME_ACTIVE == TRUE)
		{
			debug_printf("Got signal from UART, but busy processing previous signal\n");
		}
#endif //__DEBUG_TIME__
	}
}

/* Install signal handler for UART IO */
static void npi_installsig(void)
{
	struct sigaction ioaction;

	/* install signal handler */
	debug_printf("[UART] Install IO signal handler \n");
	ioaction.sa_handler = npi_iohandler;
	sigemptyset(&ioaction.sa_mask);
	ioaction.sa_flags = 0;
	ioaction.sa_restorer = NULL;
	if (sigaction(SIGIO, &ioaction, NULL)) {
		perror("sigaction");
	}

	/* allow the process to receive SIGIO */
	if (fcntl(npi_fd, F_SETOWN, getpid()) < 0) {
		perror("fcntl");
	}
}
#endif // #if (defined NPI_UART) && (NPI_UART == TRUE)

/**************************************************************************************************
 */
