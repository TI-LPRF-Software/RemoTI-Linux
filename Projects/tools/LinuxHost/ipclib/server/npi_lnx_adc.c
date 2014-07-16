/**************************************************************************************************
  Filename:       npi_lnx_adc.c
  Revised:        $Date: 2012-03-21 17:37:33 -0700 (Wed, 21 Mar 2012) $
  Revision:       $Revision: 246 $

  Description:    This file contains platform independent implementation
  	  	  	  	  of convenience calls for ADC reading.


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

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <getopt.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <poll.h>
#include <linux/input.h>

#include "aic.h"
#include "npi_lnx.h"
#include "npi_lnx_adc.h"
#include "hal_rpc.h"

#include "npi_lnx_error.h"

//#if (defined NPI_ADC_CONTROL) && (NPI_ADC_CONTROL == TRUE)

// -- macros --

#ifndef TRUE
# define TRUE (1)
#endif

#ifndef FALSE
# define FALSE (0)
#endif

#ifdef __BIG_DEBUG__
#define debug_printf(fmt, ...) printf( fmt, ##__VA_ARGS__)
#else
#define debug_printf(fmt, ...) st (if (__BIG_DEBUG_ACTIVE == TRUE) printf( fmt, ##__VA_ARGS__);)
#endif

#ifdef __DEBUG_TIME__ADC
struct timeval curTime, initTime;
#endif
#if (defined __DEBUG_TIME__)
struct timeval curTime, prevTime;
extern struct timeval startTime;
#endif
// -- Constants --

// State variable used to indicate that a device is open.
static int npiOpenFlag = FALSE;

// NPI device related variables
static int              	npi_poll_terminate;
static pthread_mutexattr_t 	npiPollLock_attr;
static pthread_mutex_t  	npi_poll_mutex;
static pthread_mutex_t		npi_adcRead_mutex;
static int 					GpioSrdyFd;
static pthread_cond_t   	npi_adcRead_cond;
// Polling thread
static pthread_t        	npiPollThread;
// Event thread

// For debugging mainly
uint8 writeOnce = 0;

// thread termination subroutines
static void npi_termpoll(void);
static void 			*npi_poll_entry(void *ptr);

// -- Public functions --

#ifdef __STRESS_TEST__
extern struct timeval curTime, startTime;
struct timeval prevTimeADC;
#endif //__STRESS_TEST__

// -- Private functions --
static int npi_initsyncres(void);
static int npi_initThreads(void);
static int npi_i2c_pollData(npiMsgData_t *pMsg);

/******************************************************************************
 * @fn         NPI_OpenDevice
 *
 * @brief      This function establishes a serial communication connection with
 *             a network processor device.
 *             As windows machine does not have a single dedicated serial
 *             interface, this function will designate which serial port shall
 *             be used for communication.
 *
 * input parameters
 *
 * @param   portName 	– name of the serial port
 * @param	gpioCfg		– GPIO settings for SRDY, MRDY and RESET
 *
 * output parameters
 *
 * None.
 *
 * @return     TRUE if the connection is established successfully.
 *             FALSE, otherwise.
 ******************************************************************************
 */
int NPI_ADC_OpenDevice(const char *portName, void *pCfg)
{
	int ret = NPI_LNX_SUCCESS;
	if(npiOpenFlag)
	{
	    npi_ipc_errno = NPI_LNX_ERROR_ADC_OPEN_ALREADY_OPEN;
	    return NPI_LNX_FAILURE;
	}

	npiOpenFlag = TRUE;

	debug_printf("Opening Device File: %s\n", portName);

	ret = HalI2cInit(portName);
	if (ret != NPI_LNX_SUCCESS)
		return ret;

	debug_printf("((npiI2cCfg_t *)pCfg)->gpioCfg[0] \t @%p\n",
			(void *)&(((npiI2cCfg_t *)pCfg)->gpioCfg[0]));

  	if ( NPI_LNX_FAILURE == (GpioSrdyFd = HalGpioSrdyInit(((npiI2cCfg_t *)pCfg)->gpioCfg[0])))
		return GpioSrdyFd;
	if ( NPI_LNX_FAILURE == (ret = HalGpioMrdyInit(((npiI2cCfg_t *)pCfg)->gpioCfg[1])))
		return ret;
	if ( NPI_LNX_FAILURE == (ret = HalGpioResetInit(((npiI2cCfg_t *)pCfg)->gpioCfg[2])))
		return ret;

	// initialize thread synchronization resources
	if ( NPI_LNX_FAILURE == (ret = npi_initsyncres()))
		return ret;

#ifdef __DEBUG_TIME__ADC
	gettimeofday(&initTime, NULL);
	printf("NPI ADC timer started\n");
#endif //__DEBUG_TIME__ADC

	// Reset The RNP
	ret = NPI_ADC_ResetSlave();
	if (ret != NPI_LNX_SUCCESS)
		return ret;

	ret = npi_initThreads();

	return ret;
}

NPI_ADC_SynchRequest(npiMsgData_t *msg)
{
	switch (msg->pData[NPI_LNX_ADC_CONTROL_CMD_ID_IDX])
	{
	case NPI_LNX_ADC_CONTROL_CMD_ID_READ_REQ:
		msg->len = sizeof(int) + 2;
		msg->cmdId = NPI_LNX_CMD_ID_ADC_CONTROL;
		ret = HalADCRead(msg->pData[NPI_LNX_ADC_CONTROL_CMD_READ_REQ_PORT_IDX], (int *)&msg->pData[NPI_LNX_ADC_CONTROL_CMD_READ_RSP_DATA_IDX]);
		if (ret == NPI_LNX_SUCCESS)
		{
			msg->pData[NPI_LNX_ADC_CONTROL_CMD_READ_RSP_PORT_IDX] = msg->pData[NPI_LNX_ADC_CONTROL_CMD_READ_REQ_PORT_IDX];
		}
		else
		{
			msg->pData[NPI_LNX_ADC_CONTROL_CMD_READ_RSP_PORT_IDX] = 0xFF; // Report invalid port to indicate error
		}
		// Command ID is always the same for the response, regardless of result
		msg->pData[NPI_LNX_ADC_CONTROL_CMD_ID_IDX] = NPI_LNX_ADC_CONTROL_CMD_ID_READ_RSP;
	}
}

/******************************************************************************
 * @fn         NPI_ADC_CloseDevice
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
 * @return     None
 ******************************************************************************
 */
void NPI_ADC_CloseDevice(void)
{
  npi_termpoll();
  HalI2cClose();
  HalGpioSrdyClose();
  HalGpioMrdyClose();
  HalGpioResetClose();
  npiOpenFlag = FALSE;
}

/* Initialize thread synchronization resources */
static int npi_initThreads(void)
{
  // create Polling thread
  // initialize ADC receive thread related variables
  npi_poll_terminate = 0;

  // TODO: it is ideal to make this thread higher priority
  // but linux does not allow realtime of FIFO scheduling policy for
  // non-priviledged threads.

  if(pthread_create(&npiPollThread, NULL, npi_poll_entry, NULL))
  {
    // thread creation failed
    NPI_ADC_CloseDevice();
    npi_ipc_errno = NPI_LNX_ERROR_ADC_OPEN_FAILED_POLL_THREAD;
    return NPI_LNX_FAILURE;
  }

  return NPI_LNX_SUCCESS;
}

/* Initialize thread synchronization resources */
static int npi_initsyncres(void)
{
	printf("[MUTEX] Initializing mutexes\n");
	// initialize all mutexes
	pthread_mutexattr_init(&npiPollLock_attr);
	pthread_mutexattr_settype(&npiPollLock_attr, PTHREAD_MUTEX_ERRORCHECK);
	if ( pthread_mutex_init(&npi_adcRead_mutex, &npiPollLock_attr) )
//	if ( pthread_mutex_init(&npi_adcRead_mutex, NULL) )
	{
		printf("Fail To Initialize Mutex npi_adcRead_mutex\n");
	    npi_ipc_errno = NPI_LNX_ERROR_ADC_OPEN_FAILED_POLL_LOCK_MUTEX;
	    return NPI_LNX_FAILURE;
	}

	if(pthread_mutex_init(&npi_poll_mutex, NULL))
	{
		printf("Fail To Initialize Mutex npi_poll_mutex\n");
	    npi_ipc_errno = NPI_LNX_ERROR_ADC_OPEN_FAILED_POLL_MUTEX;
	    return NPI_LNX_FAILURE;
	}

	if(pthread_cond_init(&npi_adcRead_cond, NULL))
	{
		printf("Fail To Initialize Condition npi_adcRead_cond\n");
	    npi_ipc_errno = NPI_LNX_ERROR_ADC_OPEN_FAILED_POLL_COND;
	    return NPI_LNX_FAILURE;
	}

	return NPI_LNX_SUCCESS;
}

/* ADC read thread entry routine */
static void *npi_poll_entry(void *ptr)
{
	int ret = NPI_LNX_SUCCESS;
	uint8 readbuf[128];
	uint8 pollStatus = FALSE;

	/* lock mutex in order not to lose signal */
	pthread_mutex_lock(&npi_poll_mutex);

	printf("POLL: Thread Started \n");

	/* thread loop */
	while(!npi_poll_terminate)
	{
		pthread_cond_wait(&npi_adcRead_cond, &npi_adcRead_mutex);

		//Ready SRDY Status
		if(1)
		{
			//   exit(-1);
			//RNP is polling, retrieve the data
			*readbuf = 0; //Poll Command has zero data bytes.
			*(readbuf+1) = RPC_CMD_POLL;
			*(readbuf+2) = 0;

			// Read ADC data
			ret = npi_i2c_pollData((npiMsgData_t *)readbuf);

			if(0 == pthread_mutex_unlock(&npi_adcRead_mutex))
			{
#if (defined __DEBUG_MUTEX__)

				debug_printf("[MUTEX] npi_i2c_pollData released (%d @ 0x%.16X)\n",
						npi_adcRead_mutex,
						&npi_adcRead_mutex);
#endif //__DEBUG_TIME__ADC
				pollStatus = TRUE;
			}
			else
			{
#if (defined __DEBUG_MUTEX__)
				printf("[MUTEX-ERR] npi_i2c_pollData released (%d @ 0x%.16X)\n", npi_adcRead_mutex, &npi_adcRead_mutex);
#endif //(defined __DEBUG_MUTEX__)
				perror("Mutex unlock Poll:");
				npi_ipc_errno = NPI_LNX_ERROR_ADC_POLL_THREAD_FAILED_UNLOCK;
				ret = NPI_LNX_FAILURE;
				// Exit clean so main knows...
				npi_poll_terminate = 1;
			}

			//Check if polling was successful, if so send message to host.
			if((readbuf[RPC_POS_CMD0] & RPC_CMD_TYPE_MASK) == RPC_CMD_AREQ)
			{
				((uint8 *)readbuf)[RPC_POS_CMD0] &=  RPC_SUBSYSTEM_MASK;
				ret = NPI_AsynchMsgCback((npiMsgData_t *)(readbuf));
				writeOnce = 0;
				if ( ret != NPI_LNX_SUCCESS)
				{
					// An error has occurred
					// Exit clean so main knows...
					npi_poll_terminate = 1;
				}
			}
		}
		else
		{
			pollStatus = FALSE;

			if (pollStatus == FALSE)
			{
				if(0 == pthread_mutex_unlock(&npi_adcRead_mutex))
				{
					if (writeOnce < 4)
					{
#if (defined __DEBUG_MUTEX__)
						debug_printf("[MUTEX] npi_i2c_pollData released (%d @ 0x%.16X)\n", npi_adcRead_mutex, &npi_adcRead_mutex);
#endif //__DEBUG_TIME__ADC
						writeOnce++;
					}
				}
				else
				{
#if (defined __DEBUG_MUTEX__)
					printf("[MUTEX-ERR] npi_i2c_pollData released (%d @ 0x%.16X)\n", npi_adcRead_mutex, &npi_adcRead_mutex);
#endif //(defined __DEBUG_MUTEX__)
					perror("Mutex unlock Poll:");
					npi_ipc_errno = NPI_LNX_ERROR_ADC_POLL_THREAD_FAILED_UNLOCK;
					ret = NPI_LNX_FAILURE;
					// Exit clean so main knows...
					npi_poll_terminate = 1;
				}
			}
		}
	}
	pthread_mutex_unlock(&npi_poll_mutex);

	char *errorMsg;
	if (ret == NPI_LNX_FAILURE)
		errorMsg = "ADC Poll thread exited with error. Please check global error message\n";
	else
		errorMsg = "ADC Poll thread exited without error\n";

	NPI_LNX_IPC_NotifyError(NPI_LNX_ERROR_MODULE_MASK(NPI_LNX_ERROR_ADC_POLL_THREAD_FAILED_LOCK), errorMsg);

	return ptr;
}

/* Terminate Polling thread */
static void npi_termpoll(void)
{
  //This will cause the Thread to exit
  npi_poll_terminate = 1;

  // Wait for lock before destroying the mutex
  pthread_mutex_lock(&npi_poll_mutex);
  pthread_mutex_unlock(&npi_poll_mutex);

  pthread_mutex_destroy(&npi_poll_mutex);

  // wait till the thread terminates
  pthread_join(npiPollThread, NULL);
}


//#endif // NPI_ADC_CONTROL
/**************************************************************************************************
*/
