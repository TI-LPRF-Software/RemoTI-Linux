/**************************************************************************************************
  Filename:       npi_lnx_spi.c
  Revised:        $Date: 2012-03-15 13:45:31 -0700 (Thu, 15 Mar 2012) $
  Revision:       $Revision: 237 $

  Description:    This file contains linux specific implementation of Network Processor Interface
                  module.


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
#include <linux/spi/spidev.h>
#include <linux/input.h>
#include <poll.h>

#include "aic.h"
#include "npi_lnx.h"
#include "npi_lnx_spi.h"
#include "hal_rpc.h"
#include "hal_gpio.h"

#include "npi_lnx_error.h"
#include "time_printf.h"

#ifdef __STRESS_TEST__
#include <sys/time.h>
#elif (defined __DEBUG_TIME__)
#include <sys/time.h>
#else
#include <sys/time.h>
#endif // __STRESS_TEST__

// -- macros --
#if (defined NPI_SPI) && (NPI_SPI == TRUE)

#ifndef TRUE
# define TRUE (1)
#endif

#ifndef FALSE
# define FALSE (0)
#endif

#define error_printf(fmt, ...) fprintf(stderr, fmt, ##__VA_ARGS__)

#ifdef __BIG_DEBUG__
#define debug_printf(fmt, ...) fprintf( stderr, fmt, ##__VA_ARGS__); fflush(stdout);
#else
#define debug_printf(fmt, ...) st (if (__BIG_DEBUG_ACTIVE == TRUE) fprintf(stderr, fmt, ##__VA_ARGS__);)
#endif

// -- Constants --

// -- Local Variables --

// State variable used to indicate that a device is open.
static int npiOpenFlag = FALSE;
static uint8 earlyMrdyDeAssert = TRUE;
static uint8 detectResetFromSlowSrdyAssert = TRUE;
static uint8 forceRun = NPI_LNX_UINT8_ERROR;
static uint8 srdyMrdyHandshakeSupport = TRUE;

// NPI device related variables
static int              npi_poll_terminate;
static pthread_mutex_t  npiPollLock;
static pthread_mutex_t  npi_poll_mutex;
static int 				GpioSrdyFd;
#ifndef SRDY_INTERRUPT
static pthread_cond_t   npi_poll_cond;
#endif

// Polling thread
//---------------
static pthread_t        npiPollThread;
static int     			PollLockVar = 0;

// thread subroutines
static void npi_termpoll(void);
static void *npi_poll_entry(void *ptr);


#ifdef SRDY_INTERRUPT
// Event thread
//---------------
static pthread_t        npiEventThread;
static void *npi_event_entry(void *ptr);
static int global_srdy;

static pthread_cond_t   npi_srdy_H2L_poll;

static pthread_mutex_t  npiSrdyLock;
#define INIT 0
#define READY 1
#endif

// -- Forward references of local functions --

// -- Public functions --

struct timespec curTimeSPIisrPoll, prevTimeSPIisrPoll;

// -- Private functions --
static int npi_initsyncres(void);
static int npi_initThreads(void);

/******************************************************************************
 * @fn         PollLockVarError
 *
 * @brief      This function kill the program due to a major Mutex problem.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return
 *
 * None.
 ******************************************************************************
 */
int PollLockVarError(int originator, int shouldNotBe)
{
    error_printf("ERROR! PollLock Var is %d, it should be %d. Called by 0x%.8X\n", !shouldNotBe, shouldNotBe, originator);
    npi_ipc_errno = NPI_LNX_ERROR_SPI_POLL_LOCK_VAR_ERROR;
    return NPI_LNX_FAILURE;
}
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
int NPI_SPI_OpenDevice(const char *portName, void *pCfg)
{
	int ret = NPI_LNX_SUCCESS;
#ifndef PERFORM_SW_RESET_INSTEAD_OF_HARDWARE_RESET
	int funcID = NPI_LNX_ERROR_FUNC_ID_OPEN_DEVICE;
#endif //PERFORM_SW_RESET_INSTEAD_OF_HARDWARE_RESET

	if(npiOpenFlag)
	{
		npi_ipc_errno = NPI_LNX_ERROR_SPI_OPEN_ALREADY_OPEN;
		return NPI_LNX_FAILURE;
	}

	npiOpenFlag = TRUE;

	debug_printf("Opening Device File: %s\n", portName);


	// Setup parameters that differ between ZNP and RNP
	earlyMrdyDeAssert = ((npiSpiCfg_t *)pCfg)->earlyMrdyDeAssert;
	detectResetFromSlowSrdyAssert = ((npiSpiCfg_t *)pCfg)->detectResetFromSlowSrdyAssert;
	forceRun = ((npiSpiCfg_t *)pCfg)->forceRunOnReset;
	srdyMrdyHandshakeSupport = ((npiSpiCfg_t *)pCfg)->srdyMrdyHandshakeSupport;

	debug_printf("((npiSpiCfg_t *)pCfg)->gpioCfg[0] \t @%p\n",
			(void *)&(((npiSpiCfg_t *)pCfg)->gpioCfg[0]));

	// Set up GPIO properly BEFORE setting up SPI to ensure SPI init doesn't have side-effect on SPI bus.
	if	( NPI_LNX_FAILURE	==	(ret = HalGpioResetInit(((npiSpiCfg_t *)pCfg)->gpioCfg[2])))
	{
		error_printf("%s(): ERROR returned from HalGpioResetInit!\n", __FUNCTION__);
	}
	else if ( NPI_LNX_FAILURE == (ret = HalGpioMrdyInit(((npiSpiCfg_t	*)pCfg)->gpioCfg[1])))
	{
		error_printf("%s(): ERROR returned from HalGpioMrdyInit!\n", __FUNCTION__);
	}
	else if ( NPI_LNX_FAILURE == (GpioSrdyFd = HalGpioSrdyInit(((npiSpiCfg_t *)pCfg)->gpioCfg[0])))
	{
		error_printf("%s(): ERROR returned from HalGpioSrdyInit!\n", __FUNCTION__);
		ret = GpioSrdyFd;
	}
	else if ( NPI_LNX_FAILURE == (ret = HalSpiInit(portName,	((npiSpiCfg_t*)pCfg)->spiCfg)))
	{
		error_printf("%s(): ERROR returned from HalSpiInit!\n", __FUNCTION__);
	}


	if	(ret == NPI_LNX_SUCCESS)
	{
		// initialize thread synchronization resources
		if	( NPI_LNX_FAILURE	==	(ret = npi_initsyncres()))
		{
			error_printf("%s(): ERROR returned from npi_initsyncres!\n", __FUNCTION__);
		}
		else
		{
		#ifndef PERFORM_SW_RESET_INSTEAD_OF_HARDWARE_RESET
			//Polling forbidden until the Reset and Sync is done
			debug_printf("LOCK POLL WHILE INIT\n");
			pthread_mutex_lock(&npiPollLock);
			if	(PollLockVar)
			{
				ret = PollLockVarError(funcID++,	PollLockVar);
			}
			else
			{
				PollLockVar	= 1;
				debug_printf("[SPI POLL] PollLockVar set to %d\n",	PollLockVar);
			}

			debug_printf("PollLockVar = %d\n", PollLockVar);
		#endif // ! PERFORM_SW_RESET_INSTEAD_OF_HARDWARE_RESET

			// TODO: it is ideal to make this thread higher priority
			// but Linux does not allow real time of FIFO scheduling policy for
			// non-privileged threads.

			if	(ret == NPI_LNX_SUCCESS)
			{
				// create Polling thread
				ret = npi_initThreads();
			}
			else
			{
				error_printf("%s() ERROR: Did not attempt to start Threads\n", __FUNCTION__);
			}
		}
	}

   debug_printf("%s() returning %d\n", __FUNCTION__, ret);
   return ret;
}


/******************************************************************************
 * @fn         NPI_SPI_CloseDevice
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
void NPI_SPI_CloseDevice(void)
{
  error_printf("Shutting down threads\n");
  npi_termpoll();
  error_printf("Closing SPI\n"); 
  HalSpiClose();
  error_printf("Closing GPIO-SRDY\n");
  HalGpioSrdyClose();
  error_printf("Closing GPIO-MRDY\n");
  HalGpioMrdyClose();
  error_printf("Closing GPIO-RESET\n");
  HalGpioResetClose();
  error_printf("Closing completed\n");
  npiOpenFlag = FALSE;
}

/**************************************************************************************************
 * @fn          NPI_SPI_SendAsynchData
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
int NPI_SPI_SendAsynchData( npiMsgData_t *pMsg )
{
	int ret = NPI_LNX_SUCCESS, funcID = NPI_LNX_ERROR_FUNC_ID_SEND_ASYNCH;
	debug_printf("[ASYNCH] Sync Lock SRDY ...\n");
	fflush(stdout);
	//Lock the polling until the command is send
	pthread_mutex_lock(&npiPollLock);
#ifdef SRDY_INTERRUPT
	pthread_mutex_lock(&npiSrdyLock);
#endif
	if (PollLockVar)
	{
		ret = PollLockVarError(funcID++, PollLockVar);
	}
	else
	{
		PollLockVar = 1;
		debug_printf("[ASYNCH][SPI POLL] PollLockVar set to %d\n", PollLockVar);
	}

	if	(ret == NPI_LNX_SUCCESS)
	{
		debug_printf("[ASYNCH] (Sync) success \n");
	}

	debug_printf("\n******************** START SEND ASYNC DATA ********************\n");
	// Add Proper RPC type to header
	((uint8*)pMsg)[RPC_POS_CMD0] = (((uint8*)pMsg)[RPC_POS_CMD0] & RPC_SUBSYSTEM_MASK) | RPC_CMD_AREQ;

	if	(ret == NPI_LNX_SUCCESS)
	{
		ret = HAL_RNP_MRDY_CLR();

		if	( NPI_LNX_SUCCESS	!=	ret)
		{
			return ret;
		}
	}

	debug_printf("[AREQ]\n");

	//Wait for SRDY Clear
	ret = HalGpioWaitSrdyClr();

	if (ret == NPI_LNX_SUCCESS)
		ret = HalSpiWrite( 0, (uint8*) pMsg, (pMsg->len)+RPC_FRAME_HDR_SZ);

	if (ret == NPI_LNX_SUCCESS)
		ret = HAL_RNP_MRDY_SET();
	else
		(void)HAL_RNP_MRDY_SET();

	if (!PollLockVar)
	{
		ret = PollLockVarError(funcID++, !PollLockVar);
	}
	else
	{
		PollLockVar = 0;
		debug_printf("[ASYNCH] [SPI POLL] PollLockVar set to %d\n", PollLockVar);
	}
	pthread_mutex_unlock(&npiPollLock);
#ifdef SRDY_INTERRUPT
	pthread_mutex_unlock(&npiSrdyLock);
#endif
	debug_printf("[ASYNCH] Sync unLock SRDY ...\n\n");
	debug_printf("\n******************** STOP SEND ASYNC DATA ********************\n");

	return ret;
}

/**************************************************************************************************
 * @fn          npi_spi_pollData
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
 * @return      STATUS
 **************************************************************************************************
 */
int npi_spi_pollData(npiMsgData_t *pMsg)
{
	int	ret = NPI_LNX_SUCCESS;
	bool  mRdyAsserted = FALSE;
	debug_printf("\n-------------------- START POLLING DATA --------------------\n");

#ifdef __BIG_DEBUG__
	printf("Polling Command ...");

	for(i = 0 ; i < (RPC_FRAME_HDR_SZ+pMsg->len); i++)
		printf(" 0x%.2x", ((uint8*)pMsg)[i]);

	printf("\n");
#endif

#ifdef __STRESS_TEST__
	//	debug_
	static struct timeval prevTimePoll;
	time_printf_always_localized("MRDY Low\n", NULL, NULL, &prevTimePoll)
#endif //__STRESS_TEST__

	ret = HAL_RNP_MRDY_CLR();

	if (NPI_LNX_SUCCESS == ret)
	{
		struct timespec t1, t2;

		mRdyAsserted = TRUE;
		ret = HalSpiWrite( 0, (uint8*)pMsg, (pMsg->len)+RPC_FRAME_HDR_SZ);

		clock_gettime(CLOCK_MONOTONIC, &t1);
#if __BIG_DEBUG__
		printf("[POLL] %.5ld.%09ld]\n", t1.tv_sec, t1.tv_nsec);
#endif

		if (ret == NPI_LNX_SUCCESS)
		{
			int bigDebugWas = __BIG_DEBUG_ACTIVE;
			if (!bigDebugWas)
			{
				__BIG_DEBUG_ACTIVE = FALSE;
			}

			//Wait for SRDY set
			ret = HalGpioWaitSrdySet();

			if (ret != NPI_LNX_SUCCESS)
				fprintf(stderr, "%s() ERROR: HalGpioWaitSrdySet() failed!\n", __FUNCTION__);

			// Check how long it took to wait for SRDY to go High. May indicate that this Poll was considered
			// a handshake by the RNP.
			clock_gettime(CLOCK_MONOTONIC, &t2);

			if (earlyMrdyDeAssert)
			{
				//We Set MRDY here to avoid GPIO latency with the beagle board
				// if we do here later, the RNP see it low at the end of the transaction and
				// therefore think a new transaction is starting and lower its SRDY...
				int mRet = HAL_RNP_MRDY_SET();

				if (mRet == NPI_LNX_SUCCESS)
					mRdyAsserted = FALSE;
				else if (ret == NPI_LNX_SUCCESS)
					ret = mRet;
			}

			if (detectResetFromSlowSrdyAssert)
			{
				// NOTE: Below casting shouldn't be needed, but is protecting in case the structure elements
				//       are unsigned types.  Converting values such that diffusecs is microseconds
				long int diffusecs = (((long)((long)t2.tv_sec - (long)t1.tv_sec) * 1000000L) + (long)(((long)t2.tv_nsec - (long)t1.tv_nsec)/1000L));

				// If it took more than NPI_LNX_SPI_NUM_OF_MS_TO_DETECT_RESET_AFTER_SLOW_SRDY_ASSERT ms
				// then it's likely a reset handshake.
				if (diffusecs > (NPI_LNX_SPI_NUM_OF_MS_TO_DETECT_RESET_AFTER_SLOW_SRDY_ASSERT * 1000) )
				{
					error_printf("[POLL] SRDY took %ld us to go high (%ld.%09ld, %ld.%09ld)\n", diffusecs, t1.tv_sec, t1.tv_nsec, t2.tv_sec, t2.tv_nsec);
					npi_ipc_errno = NPI_LNX_ERROR_SPI_POLL_DATA_SRDY_CLR_TIMEOUT_POSSIBLE_RESET;
					ret = NPI_LNX_FAILURE;
				}
			}

			__BIG_DEBUG_ACTIVE = bigDebugWas;

			if (ret == NPI_LNX_SUCCESS)
			{
				//Do a Three Byte Dummy Write to read the RPC Header (initialize to 0 first)
				memset((uint8*)pMsg, 0, RPC_FRAME_HDR_SZ);
				if (ret == NPI_LNX_SUCCESS)
				{
					ret = HalSpiRead( 0, (uint8*)pMsg, RPC_FRAME_HDR_SZ);

					if (ret == NPI_LNX_SUCCESS)
					{
						// If we read 0xFF, 0xFF, 0xFF then it's an illegal header
						if ( (pMsg->len == 0xFF) &&
							  (pMsg->subSys == 0xFF) &&
							  (pMsg->cmdId == 0xFF) )
						{
							// Do nothing
							error_printf("[POLL] WARNING: Invalid header (FF FF FF) received!\n");
						}
						else if (pMsg->len > 0)
						{
							//Zero buffer for length about to read, then do a write/read of the corresponding length
							memset(pMsg->pData, 0, ((uint8*)pMsg)[0]);
							ret = HalSpiRead( 0, pMsg->pData, pMsg->len);
						}
					}

#ifdef __BIG_DEBUG__
					if (HAL_RNP_SRDY_CLR())
						printf("SRDY is set\n");
					else
						printf("SRDY is clear\n");
#endif

#ifdef __BIG_DEBUG__
					printf("Poll Response Received (ret=0x%x) ...", (unsigned int)ret);
					for (i = 0 ; i < (RPC_FRAME_HDR_SZ+pMsg->len); i++ ) printf(" 0x%.2x", ((uint8*)pMsg)[i]);
					printf("\n");
#endif
				}
			}
		}
	}

	if (mRdyAsserted)
	{
		int mRet = HAL_RNP_MRDY_SET();
		if (ret == NPI_LNX_SUCCESS)
			ret = mRet;
	}

	debug_printf("\n-------------------- END POLLING DATA --------------------\n");

	return ret;
}

/**************************************************************************************************
 * @fn          NPI_SPI_SendSynchData
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
 * @return      STATUS
 **************************************************************************************************
 */
int NPI_SPI_SendSynchData( npiMsgData_t *pMsg )
{
	int i, ret = NPI_LNX_SUCCESS, funcID = NPI_LNX_ERROR_FUNC_ID_SEND_SYNCH;
	int lockRetPoll = 0, lockRetSrdy = 0;

	// Do not attempt to send until polling is finished

	if ( (__DEBUG_TIME_ACTIVE == TRUE) &&  (__BIG_DEBUG_ACTIVE == TRUE) )
	{
		time_printf("[SYNCH] Lock POLL mutex");
	}
	//Lock the polling until the command is send
	lockRetPoll = pthread_mutex_lock(&npiPollLock);
	if (lockRetPoll)
	{
		error_printf("[SYNCH] [ERR] Error %d getting POLL mutex lock\n", lockRetPoll);
		perror("mutex lock");
		ret = NPI_LNX_FAILURE;
	}
	else
	{
		if ( (__DEBUG_TIME_ACTIVE == TRUE) &&  (__BIG_DEBUG_ACTIVE == TRUE) )
		{
			time_printf("[SYNCH] POLL mutex locked\n");
		}
#ifdef SRDY_INTERRUPT
		if ( (__DEBUG_TIME_ACTIVE == TRUE) &&  (__BIG_DEBUG_ACTIVE == TRUE) )
		{
			time_printf("[SYNCH] Lock SRDY mutex\n");
		}
		lockRetSrdy = pthread_mutex_lock(&npiSrdyLock);
		if (lockRetSrdy)
		{
			error_printf("[SYNCH] [ERR] Error %d getting SRDY mutex lock\n", lockRetSrdy);
			perror("mutex lock");
			ret = NPI_LNX_FAILURE;
		}
		else
		{
			if ( (__DEBUG_TIME_ACTIVE == TRUE) &&  (__BIG_DEBUG_ACTIVE == TRUE) )
			{
				time_printf("[SYNCH] SRDY mutex locked\n");
			}
		}
#endif
	}

	if (ret == NPI_LNX_SUCCESS)
	{
		if (PollLockVar)
		{
			ret = PollLockVarError(funcID++, PollLockVar);
		}
		else
		{
			PollLockVar = 1;
			debug_printf("[SYNCH] PollLockVar set to %d\n", PollLockVar);
		}
		debug_printf("[SYNCH] (Sync) success \n");
		debug_printf("==================== START SEND SYNC DATA ====================\n");
	}

	if	(ret == NPI_LNX_SUCCESS)
	{
#ifdef __BIG_DEBUG__
		if (TRUE == HAL_RNP_SRDY_CLR())
			printf("[SYNCH] SRDY set\n");
		else
			printf("[SYNCH] SRDY Clear\n");
#endif

		// Add Proper RPC type to header
		((uint8*)pMsg)[RPC_POS_CMD0] = (((uint8*)pMsg)[RPC_POS_CMD0] & RPC_SUBSYSTEM_MASK) | RPC_CMD_SREQ;

		ret = HAL_RNP_MRDY_CLR();
	}

	if	( NPI_LNX_SUCCESS	==	ret)
	{
		//Wait for SRDY Clear
		ret = HalGpioWaitSrdyClr();

		if (ret != NPI_LNX_SUCCESS)
			error_printf("[SYNCH] [SREQ] ERROR! Waiting for SRDY assert failed, ret=0x%x\n", ret);
		else
		{
			debug_printf("[SYNCH] Sync Data Command ...");
			for (i = 0 ; i < (RPC_FRAME_HDR_SZ+pMsg->len); i++ ) debug_printf(" 0x%.2x", ((uint8*)pMsg)[i]);
			debug_printf("\n");

			ret = HalSpiWrite( 0, (uint8*) pMsg, (pMsg->len)+RPC_FRAME_HDR_SZ);

			if (ret != NPI_LNX_SUCCESS)
				error_printf("[SYNCH] [SREQ], SPI Write Failed, ret=0x%x\n", ret);
			else
			{
				//Wait for SRDY set
				ret = HalGpioWaitSrdySet();
				if (ret != NPI_LNX_SUCCESS)
				{
					error_printf("[SYNCH] [SREQ], [ERR] ret=0x%x, line %d, errno=0x%x\n", ret, __LINE__, npi_ipc_errno);
					(void)HAL_RNP_MRDY_SET();
				}
				else if (earlyMrdyDeAssert == TRUE)
				{
					//We Set MRDY here to avoid GPIO latency with the beagle board
					// if we do here later, the RNP see it low at the end of the transaction and
					// therefore think a new transaction is starting and lower its SRDY...
					if (ret == NPI_LNX_SUCCESS)
						ret = HAL_RNP_MRDY_SET();
					else
					{
						error_printf("[SYNCH] [SREQ], [ERR] ret=0x%x, line %d, errno=0x%x\n", ret, __LINE__, npi_ipc_errno);
						(void)HAL_RNP_MRDY_SET();
					}
				}

				if (ret == NPI_LNX_SUCCESS)
				{
					//Do a Three Byte Dummy Write to read the RPC Header
					memset(pMsg, 0, RPC_FRAME_HDR_SZ);
					ret = HalSpiRead( 0, (uint8*) pMsg, RPC_FRAME_HDR_SZ);

					debug_printf("[SYNCH] [ret=0x%x] Read %d bytes ...", ret, RPC_FRAME_HDR_SZ);
					for (i = 0 ; i < (RPC_FRAME_HDR_SZ); i++ ) debug_printf(" 0x%.2x", ((uint8*)pMsg)[i]);
					debug_printf("\n");

					if (ret != NPI_LNX_SUCCESS)
					{
						error_printf("[SYNCH] [SREQ], [ERR] ret=0x%x, line %d, errno=0x%x\n", ret, __LINE__, npi_ipc_errno);
						(void)HAL_RNP_MRDY_SET();
					}
					else if (pMsg->len > 0)
					{
						if (pMsg->len == 0xFF && pMsg->subSys == 0xFF && pMsg->cmdId == 0xFF)
						{
							error_printf("[SYNCH] Received 0xFF 0xFF 0xFF.  Ignoring it and returning an error!\n");
							ret = NPI_LNX_FAILURE;
						}
						else
						{
							// Do a write/read of the corresponding length
							// Fill data with 0s first (aids in debugging):
							memset(pMsg->pData, 0, pMsg->len);
							ret = HalSpiRead( 0, pMsg->pData, pMsg->len);

							if (earlyMrdyDeAssert == FALSE)
							{
								//End of transaction
								if (ret == NPI_LNX_SUCCESS)
								{
									ret = HAL_RNP_MRDY_SET();
								}
								else
								{
									(void)HAL_RNP_MRDY_SET();
								}
							}

							debug_printf("[SYNCH] [ret=0x%x] Read %d more bytes ...", ret, pMsg->len);
							for (i = 0; i < pMsg->len; i++ ) debug_printf(" 0x%.2x", pMsg->pData[i]);
							debug_printf("\n");
						}
					}
				}
			}
		}
	}
#ifdef __BIG_DEBUG__
	if (TRUE == HAL_RNP_SRDY_CLR())
		printf("[SYNCH] SRDY set\n");
	else
		printf("[SYNCH] SRDY Clear\n");
#endif

	if (!PollLockVar)
	{
		ret = PollLockVarError(funcID++, !PollLockVar);
	}
	else
	{
		PollLockVar = 0;
		debug_printf("[SYNCH] PollLockVar set to %d\n", PollLockVar);
	}

	if (!lockRetPoll)
		pthread_mutex_unlock(&npiPollLock);

#ifdef SRDY_INTERRUPT
	if (!lockRetSrdy)
	{
		pthread_mutex_unlock(&npiSrdyLock);
		debug_printf("[SYNCH] Sync unLock SRDY ...\n\n");
	}
#endif

#ifdef __BIG_DEBUG__
	printf("[SYNCH] Sync Data Receive [ret=%d] ...", ret);
	for (i = 0 ; i < (RPC_FRAME_HDR_SZ+pMsg->len); i++ ) printf(" 0x%.2x", ((uint8*)pMsg)[i]);
	printf("\n");
#endif

	//Release the polling lock
	debug_printf("\n==================== END SEND SYNC DATA ====================\n");
	return ret;
}


/**************************************************************************************************
 * @fn          NPI_SPI_ResetSlave
 *
 * @brief       do the HW synchronization between the host and the RNP
 *
 * input parameters
 *
 * @param      none
 *
 * output parameters
 *
 * None.
 *
 * @return      STATUS
 **************************************************************************************************
 */
int NPI_SPI_ResetSlave( void )
{
	int ret = NPI_LNX_SUCCESS;

#ifdef __DEBUG_TIME__
	time_printf_always("----- START RESET SLAVE ------------\n");
#else	//(!defined __DEBUG_TIME__)
	printf("\n\n-------------------- START RESET SLAVE -------------------\n");
#endif //(defined __DEBUG_TIME__)


#ifdef PERFORM_SW_RESET_INSTEAD_OF_HARDWARE_RESET
	if (HalGpioSrdyCheck(1) != FALSE) // TRUE, FALSE, or ERROR.  If we're not coming up from a cold boot where SRDY would already be low...
	{
		npiMsgData_t pMsg;

		pMsg.subSys	  = RPC_SYS_RCAF;
		pMsg.cmdId	  = RTIS_CMD_ID_RTI_SW_RESET_REQ;
		pMsg.len		  = 0;
		printf("---------- %s WARNING: CANNOT RESET SLAVE VIA GPIO. ATTEMPTING SW RESET. ---------\n", __FUNCTION__);

		// send command to slave
		ret = NPI_SPI_SendAsynchData(	&pMsg	);

		// If the chip was already in the bootloader when this was called, then we need to clock 3 bytes to synch up the bootloader.
		//Do a Three Byte Dummy Write to read the RPC Header
		memset(pMsg.pData, 0, RPC_FRAME_HDR_SZ);

		HAL_RNP_MRDY_CLR();
		HalSpiWrite( 0, pMsg.pData, RPC_FRAME_HDR_SZ);
		HAL_RNP_MRDY_SET();
	}
#else
	ret = HalGpioReset();

	if	(forceRun != NPI_LNX_UINT8_ERROR)
	{
	  if (ret == NPI_LNX_SUCCESS)
	  {
		  ret = HalGpioWaitSrdyClr();
	  }
	  //Send force run command
	  if (ret == NPI_LNX_SUCCESS)
	  {
		  ret = HalSpiWrite( 0, &forceRun, 1);
	  }
	  //Wait for SRDY High, do this regardless of error
	  if (ret == NPI_LNX_SUCCESS)
	  {
		  ret = HalGpioWaitSrdySet();
	  }
	  else
	  {
		  // Keep previous error message, but still de-assert to unlock Network Processor
		  HalGpioWaitSrdySet();
	  }
  }
#endif // PERFORM_SW_RESET_INSTEAD_OF_HARDWARE_RESET

  printf("Wait 500us for RNP to initialize after a Reset... This may change in the future, check for RTI_ResetInd()...\n");
  usleep(500); //wait 500us for RNP to initialize

#ifdef __DEBUG_TIME__
	time_printf_always("----- END RESET SLAVE --------------\n");
#else //(!defined __DEBUG_TIME__)
  printf("-------------------- END RESET SLAVE -------------------\n");
#endif //(defined __DEBUG_TIME__)

  return ret;
}

/* Initialize thread synchronization resources */
static int npi_initThreads(void)
{
	int ret = NPI_LNX_SUCCESS;
	// create Polling thread
  // initialize SPI receive thread related variables
	npi_poll_terminate = 0;

	// TODO: it is ideal to make this thread higher priority
	// but linux does not allow realtime of FIFO scheduling policy for
	// non-priviledged threads.

	if(pthread_create(&npiPollThread, NULL, npi_poll_entry, NULL))
	{
		// thread creation failed
		NPI_SPI_CloseDevice();
    	npi_ipc_errno = NPI_LNX_ERROR_SPI_OPEN_FAILED_POLL_THREAD;
    	return NPI_LNX_FAILURE;
	}
#ifdef SRDY_INTERRUPT

	if(pthread_create(&npiEventThread, NULL, npi_event_entry, NULL))
	{
		// thread creation failed
		NPI_SPI_CloseDevice();
    	npi_ipc_errno = NPI_LNX_ERROR_SPI_OPEN_FAILED_EVENT_THREAD;
    	return NPI_LNX_FAILURE;
	}
#endif

	return ret;

}

/**************************************************************************************************
 * @fn          NPI_SPI_SynchSlave
 *
 * @brief       do the HW synchronization between the host and the RNP
 *
 * input parameters
 *
 * @param      none
 *
 * output parameters
 *
 * None.
 *
 * @return      STATUS
 **************************************************************************************************
 */
int NPI_SPI_SynchSlave( void )
{
	int ret = NPI_LNX_SUCCESS;
#ifndef PERFORM_SW_RESET_INSTEAD_OF_HARDWARE_RESET
	int funcID = NPI_LNX_ERROR_FUNC_ID_SYNCH_SLAVE;
#endif //!PERFORM_SW_RESET_INSTEAD_OF_HARDWARE_RESET;

	if (srdyMrdyHandshakeSupport == TRUE)
	{
		printf("\n\n-------------------- START GPIO HANDSHAKE -------------------\n");

		// At this point we already have npiPollMutex lock
		int lockRetSrdy = 0;
#ifdef SRDY_INTERRUPT
		if ( (__DEBUG_TIME_ACTIVE == TRUE) &&  (__BIG_DEBUG_ACTIVE == TRUE) )
		{
			time_printf("[HANDSHAKE] Lock SRDY mutex\n");
		}
		lockRetSrdy = pthread_mutex_lock(&npiSrdyLock);
		if ( (__DEBUG_TIME_ACTIVE == TRUE) &&  (__BIG_DEBUG_ACTIVE == TRUE) )
		{
			time_printf("[HANDSHAKE] SRDY mutex locked\n");
		}
#endif
#ifndef PERFORM_SW_RESET_INSTEAD_OF_HARDWARE_RESET
		if (!PollLockVar)
		{
			ret = PollLockVarError(funcID, !PollLockVar);
		}
		else
		{
			PollLockVar = 1;
			debug_printf("[HANDSHAKE] PollLockVar set to %d\n", PollLockVar);
		}
#endif //PERFORM_SW_RESET_INSTEAD_OF_HARDWARE_RESET
		if (lockRetSrdy != 0)
		{
			error_printf("%s() [HANDSHAKE] ERROR! Could not get SRDY mutex lock\n", __FUNCTION__);
			perror("mutex lock");
		}

#ifdef __DEBUG_TIME__
		time_printf_always("Handshake Lock SRDY... Wait for SRDY to go Low\n");
#else //(!defined __DEBUG_TIME__)
		printf("Handshake Lock SRDY ...\n");
#endif // defined __DEBUG_TIME__

		// Check that SRDY is low
		ret = HalGpioWaitSrdyClr();
#ifdef PERFORM_SW_RESET_INSTEAD_OF_HARDWARE_RESET
		if ((ret == NPI_LNX_FAILURE) && (npi_ipc_errno == NPI_LNX_ERROR_HAL_GPIO_WAIT_SRDY_CLEAR_POLL_TIMEDOUT))
		{
			printf("[HANDSHAKE] We may have attempted a soft reset while in bootloader. In this case a timeout is expected. Writing 3 bytes to release situation\n");
		}
#endif //PERFORM_SW_RESET_INSTEAD_OF_HARDWARE_RESET

#ifdef __DEBUG_TIME__
		time_printf_always("Set MRDY Low\n");
#endif // defined __DEBUG_TIME__

		// set MRDY to Low
		if (ret == NPI_LNX_SUCCESS)
		{
			if ( NPI_LNX_SUCCESS != (ret = HAL_RNP_MRDY_CLR()))
			{
				return ret;
			}
		}

#ifdef __DEBUG_TIME__
		time_printf_always("Wait for SRDY to go High\n");
#endif // defined __DEBUG_TIME__

		// Wait for SRDY to go High
		ret = HalGpioWaitSrdySet();

#ifdef __DEBUG_TIME__
		time_printf_always("Set MRDY High\n");
#endif // defined __DEBUG_TIME__
		// Set MRDY to High
		if (ret == NPI_LNX_SUCCESS)
			ret = HAL_RNP_MRDY_SET();
		else
			(void)HAL_RNP_MRDY_SET();

		if (ret == NPI_LNX_SUCCESS)
			ret = HalGpioSrdyCheck(1);

#ifndef PERFORM_SW_RESET_INSTEAD_OF_HARDWARE_RESET
		if (!PollLockVar)
		{
			ret = PollLockVarError(funcID++, !PollLockVar);
		}
		else
		{
			PollLockVar = 0;
			debug_printf("[HANDSHAKE] PollLockVar set to %d\n", PollLockVar);
		}

		printf("[HANDSHAKE] unLock Poll ...\n");
		pthread_mutex_unlock(&npiPollLock);
#endif //PERFORM_SW_RESET_INSTEAD_OF_HARDWARE_RESET
		printf("(Handshake) success \n");
#ifdef SRDY_INTERRUPT
		pthread_mutex_unlock(&npiSrdyLock);
#endif
		printf("-------------------- END GPIO HANDSHAKE -------------------\n");
	}
	else
	{
		printf("\n\n----------------- SYNCHRONISING MUTEX'S ----------------\n");
		pthread_mutex_unlock(&npiPollLock);
		printf("unLock Poll ...\n");
#ifdef SRDY_INTERRUPT
		pthread_mutex_unlock(&npiSrdyLock);
		printf("unLock SRDY ...\n");
#endif
		printf("----------------- END SYNCHRONISING MUTEX'S ----------------\n");
	}

	return ret;
}

/**************************************************************************************************
 * @fn          npi_initsyncres
 *
 * @brief       Thread initialization
 *
 * input parameters
 *
 * @param      none
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
static int npi_initsyncres(void)
{
  // initialize all mutexes
	debug_printf("LOCK POLL CREATED\n");
	if (pthread_mutex_init(&npiPollLock, NULL))
	{
		error_printf("ERROR: Fail To Initialize Mutex npiPollLock\n");
		npi_ipc_errno = NPI_LNX_ERROR_SPI_OPEN_FAILED_POLL_LOCK_MUTEX;
		return NPI_LNX_FAILURE;
	}

	if(pthread_mutex_init(&npi_poll_mutex, NULL))
	{
		error_printf("ERROR: Fail To Initialize Mutex npi_poll_mutex\n");
		npi_ipc_errno = NPI_LNX_ERROR_SPI_OPEN_FAILED_POLL_MUTEX;
		return NPI_LNX_FAILURE;
	}
#ifdef SRDY_INTERRUPT
	if(pthread_cond_init(&npi_srdy_H2L_poll, NULL))
	{
		error_printf("ERROR: Fail To Initialize Condition npi_srdy_H2L_poll\n");
		npi_ipc_errno = NPI_LNX_ERROR_SPI_OPEN_FAILED_SRDY_COND;
		return NPI_LNX_FAILURE;
	}
	if (pthread_mutex_init(&npiSrdyLock, NULL))
	{
		error_printf("ERROR: Fail To Initialize Mutex npiSrdyLock\n");
		npi_ipc_errno = NPI_LNX_ERROR_SPI_OPEN_FAILED_SRDY_LOCK_MUTEX;
		return NPI_LNX_FAILURE;
	}
#else
	if(pthread_cond_init(&npi_poll_cond, NULL))
	{
		error_printf("ERROR: Fail To Initialize Condition npi_poll_cond\n");
		npi_ipc_errno = NPI_LNX_ERROR_SPI_OPEN_FAILED_POLL_COND;
		return NPI_LNX_FAILURE;
	}
#endif
  return NPI_LNX_SUCCESS;
}


/**************************************************************************************************
 * @fn          npi_poll_entry
 *
 * @brief       Poll Thread entry function
 *
 * input parameters
 *
 * @param      ptr
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
static void *npi_poll_entry(void *ptr)
{
	int ret = NPI_LNX_SUCCESS, funcID = NPI_LNX_ERROR_FUNC_ID_POLL_ENTRY;
	uint8 readbuf[128];
#ifndef SRDY_INTERRUPT
	uint8 pollStatus = FALSE;
#endif //SRDY_INTERRUPT

	printf("[POLL] Locking Mutex for Poll Thread \n");

	/* lock mutex in order not to lose signal */
	pthread_mutex_lock(&npi_poll_mutex);

	printf("[POLL] Thread Started \n");

	//This lock wait for Initialization to finish (reset+sync)
	pthread_mutex_lock(&npiPollLock);

	printf("[POLL] Thread Continues After Synchronization\n");

#ifdef SRDY_INTERRUPT
	debug_printf("[POLL] Lock Poll mutex (SRDY=%d) \n", global_srdy);
	pthread_cond_wait(&npi_srdy_H2L_poll, &npiPollLock);
	debug_printf("[POLL] Locked Poll mutex (SRDY=%d) \n", global_srdy);
#else
	pthread_mutex_unlock(&npiPollLock);
#endif

	/* thread loop */
	while(!npi_poll_terminate)
	{

#ifndef SRDY_INTERRUPT
		pthread_mutex_lock(&npiPollLock);
#endif
		if (PollLockVar)
		{
			ret = PollLockVarError(funcID+1, PollLockVar);
		}
		else
		{
			PollLockVar = 1;
			debug_printf("[POLL] PollLockVar set to %d\n", PollLockVar);
		}

		//Ready SRDY Status
		// This Test check if RNP has asserted SRDY line because it has some Data pending.
		// If SRDY is not Used, then this line need to be commented, and the Poll command need
		// to be sent regularly to check if any data is pending. this is done every 10ms (see below npi_poll_cond)
#ifndef SRDY_INTERRUPT
		ret =  HAL_RNP_SRDY_CLR();
		if(TRUE == ret)
#else
		//Interruption case, In case of a SREQ, SRDY will go low a end generate an event.
		// the npiPollLock will prevent us to arrive to this test,
		// BUT an AREQ can immediately follow  a SREQ: SRDY will stay low for the whole process
		// In this case, we need to check that the SRDY line is still LOW or is HIGH.
		if (HalGpioSrdyCheck(0) == TRUE)
#endif
		{
			debug_printf("[POLL] Polling received...\n");

			//RNP is polling, retrieve the data
			*readbuf = 0; //Poll Command has zero data bytes.
			*(readbuf+1) = RPC_CMD_POLL;
			*(readbuf+2) = 0;
			ret = npi_spi_pollData((npiMsgData_t *)readbuf);
			if (ret == NPI_LNX_SUCCESS)
			{
				//Check if polling was successful
				if ((readbuf[RPC_POS_CMD0] & RPC_CMD_TYPE_MASK) == RPC_CMD_AREQ)
				{
//					((uint8 *)readbuf)[RPC_POS_CMD0] =  RPC_SUBSYSTEM_MASK;
					ret = NPI_AsynchMsgCback((npiMsgData_t *)(readbuf));
					if (ret != NPI_LNX_SUCCESS)
					{
						// Exit thread to invoke report to main thread
						npi_poll_terminate = 1;
						error_printf("%s:%d: ERROR! Terminating poll because RPC_CMD_AREQ.\n", __FUNCTION__, __LINE__);
					}
				}
			}
			else
			{
				// Exit thread to invoke report to main thread
				npi_poll_terminate = 1;
				if (ret == NPI_LNX_ERROR_SPI_POLL_DATA_SRDY_CLR_TIMEOUT_POSSIBLE_RESET)
				{
					error_printf("[POLL][WARNING] Unexpected handshake received. RNP may have reset. \n");
				}
				error_printf("%s:%d: ERROR! Terminating poll because error return (ret=%d, npi_ipc_errno=%d).\n", __FUNCTION__, __LINE__, ret, npi_ipc_errno);
			}

			if (!PollLockVar)
			{
				ret = PollLockVarError(funcID+2, !PollLockVar);
			}
			else
			{
				PollLockVar = 0;
				debug_printf("[POLL] PollLockVar set to %d\n", PollLockVar);
			}

#ifndef SRDY_INTERRUPT
			if ( 0 == pthread_mutex_unlock(&npiPollLock))
			{
				pollStatus = TRUE;
				debug_printf("[POLL] UnLock SRDY ...\n");
			}
			else
			{
				debug_printf("[POLL] UnLock SRDY FAILED...\n");
			    npi_ipc_errno = NPI_LNX_ERROR_SPI_POLL_THREAD_POLL_UNLOCK;
			    ret = NPI_LNX_FAILURE;
				npi_poll_terminate = 1;
				error_printf("%s:%d: ERROR! Terminating poll because POLL mutex unlock failed.\n", __FUNCTION__, __LINE__);
			}
#endif //SRDY_INTERRUPT
		}
		else
		{
			if (!PollLockVar)
			{
				ret = PollLockVarError(funcID+3, !PollLockVar);
			}
			else
			{
				PollLockVar = 0;
				debug_printf("[POLL] PollLockVar set to %d\n", PollLockVar);
			}

#ifdef SRDY_INTERRUPT
			debug_printf("[POLL] SRDY was not 0 when we expected!\n");
#else
			if ( 0 == pthread_mutex_unlock(&npiPollLock))
			{
				debug_printf("[POLL] UnLock SRDY ...\n");
			}
			else
			{
				debug_printf("[POLL] UnLock SRDY FAILED...\n");
			    npi_ipc_errno = NPI_LNX_ERROR_SPI_POLL_THREAD_POLL_UNLOCK;
			    ret = NPI_LNX_FAILURE;
				npi_poll_terminate = 1;
				error_printf("%s:%d: ERROR! Terminating poll because POLL mutex unlock failed.\n", __FUNCTION__, __LINE__);
			}
			pollStatus = FALSE;
#endif //SRDY_INTERRUPT
		}

#ifdef SRDY_INTERRUPT
		debug_printf("[POLL] Unlock POLL mutex by conditional wait (SRDY=%d) \n", global_srdy);
		if (!npi_poll_terminate)
		{
			pthread_cond_wait(&npi_srdy_H2L_poll, &npiPollLock);
		}
		else
		{
			// Just unlock mutex, while loop will exit next
			pthread_mutex_unlock(&npiPollLock);
		}
		debug_printf("[POLL] Locked POLL mutex because condition was met (SRDY=%d) \n", global_srdy);
#else
		if (!pollStatus) //If previous poll failed, wait 10ms to do another one, else do it right away to empty the RNP queue.
		{
			struct timespec expirytime;

			clock_gettime(CLOCK_REALTIME, &expiryTime);

			expirytime.tv_nsec += 10000000; // 10ms
			if (expirytime.tv_nsec >= 1000000000) {
				expirytime.tv_nsec -= 1000000000;
				expirytime.tv_sec++;
			}
			pthread_cond_timedwait(&npi_poll_cond, &npi_poll_mutex, &expirytime);
		}
#endif
	}
	error_printf("[POLL] WARNING. Thread exiting with ret=%d, npi_ipc_errno0x%x...\n", ret, npi_ipc_errno);
	pthread_mutex_unlock(&npi_poll_mutex);

	char const *errorMsg;
	if ( (ret != NPI_LNX_SUCCESS) && (npi_ipc_errno != NPI_LNX_ERROR_SPI_POLL_THREAD_SREQ_CONFLICT) )
	{
		errorMsg = "[POLL] Thread exited with error. Please check global error message\n";
	}
	else
	{
		errorMsg = "[POLL] Thread exited without error\n";
	}

	NPI_LNX_IPC_NotifyError(NPI_LNX_ERROR_MODULE_MASK(NPI_LNX_ERROR_SPI_POLL_THREAD_POLL_LOCK), errorMsg);

	return NULL;
}

/**************************************************************************************************
 * @fn          npi_termpoll
 *
 * @brief       Poll Thread terminate function
 *
 * input parameters
 *
 * @param      ptr
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
static void npi_termpoll(void)
{
  //This will cause the Thread to exit
  npi_poll_terminate = 1;
  error_printf("%s:%d: Terminating poll because...well, we're %s().\n", __FUNCTION__, __LINE__, __FUNCTION__);

#ifdef SRDY_INTERRUPT
	pthread_cond_signal(&npi_srdy_H2L_poll);
#else
	// In case of polling mechanism, send the Signal to continue
	pthread_cond_signal(&npi_poll_cond);
#endif

#ifdef SRDY_INTERRUPT
	pthread_mutex_destroy(&npiSrdyLock);
#endif
  pthread_mutex_destroy(&npi_poll_mutex);

  // wait till the thread terminates
  pthread_join(npiPollThread, NULL);

#ifdef SRDY_INTERRUPT
  pthread_join(npiEventThread, NULL);
#endif //SRDY_INTERRUPT
}

#ifdef SRDY_INTERRUPT
/**************************************************************************************************
 * @fn          npi_event_entry
 *
 * @brief       Poll Thread entry function
 *
 * input parameters
 *
 * @param      ptr
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
static void *npi_event_entry(void *ptr)
{
#define SPI_ISR_POLL_TIMEOUT_MS_MIN		3
#define SPI_ISR_POLL_TIMEOUT_MS_MAX		100
	int result = -1;
	int missedInterrupt = 0;
	int consecutiveTimeout = 0;
	int whileIt = 0;
	int ret = NPI_LNX_SUCCESS;
	int timeout = SPI_ISR_POLL_TIMEOUT_MS_MAX;
	/* Timeout in msec. Drop down to SPI_ISR_POLL_TIMEOUT_MS_MIN if two consecutive interrupts are missed */
	struct pollfd pollfds[1];
	int val;
	char tmpStr[512];

	printf("[INT]: Interrupt Event Thread Started \n");

	/* thread loop */
	while (!npi_poll_terminate)
	{
		whileIt++;
		memset((void*) pollfds, 0, sizeof(pollfds));
		pollfds[0].fd = GpioSrdyFd; /* Wait for input */
		pollfds[0].events = POLLPRI; /* Wait for input */
		result = poll(pollfds, 1, timeout);

		// Make sure we're not in Asynch data or Synch data, so check if npiSrdyLock is available
		if (pthread_mutex_trylock(&npiSrdyLock) != 0)
		{
			if ( __BIG_DEBUG_ACTIVE == TRUE )
			{
				// We are in Asynch or Synch, so return to poll
				if (HAL_RNP_SRDY_SET() == TRUE)
				{
					time_printf("[INT]: SRDY found to be de-asserted while we are transmitting\n");
				}
				else
				{
					time_printf("[INT]: SRDY found to be asserted while we are transmitting\n");
				}
			}
			continue;
		}
		else
		{
			if ( __BIG_DEBUG_ACTIVE == TRUE)
			{
				snprintf(tmpStr, sizeof(tmpStr), "[INT]: Event thread has SRDY mutex lock, result = %d\n", result);
				time_printf(tmpStr);
			}
			// We got lock, move on
			switch (result)
			{
			case 0:
			{
				//Should not happen by default no Timeout.
				int bigDebugWas = __BIG_DEBUG_ACTIVE;
				if (bigDebugWas == TRUE)
				{
					__BIG_DEBUG_ACTIVE = FALSE;
				}
				if (__BIG_DEBUG_ACTIVE == TRUE)
				{
					snprintf(tmpStr, sizeof(tmpStr), "[INT]: poll() timeout (timeout set to %d), poll() returned %d\n", timeout, result);
					time_printf(tmpStr);
				}
				result = 2; //Force wrong result to avoid deadlock caused by timeout
				//#ifdef __BIG_DEBUG__
				if (  NPI_LNX_FAILURE == (val = HalGpioSrdyCheck(1)))
				{
					ret = val;
					npi_poll_terminate = 1;
					error_printf("%s:%d: ERROR! Terminating poll because HalGpioSrdyCheck() returned error %d.\n", __FUNCTION__, __LINE__, ret);
				}
				else
				{
					// Accept this case as a missed interrupt. We may stall if not attempting to handle asserted SRDY
					if (!val)
					{
						if ( __BIG_DEBUG_ACTIVE == TRUE )
						{
							snprintf(tmpStr, sizeof(tmpStr), "Missed interrupt: %d (it #%d)\n", missedInterrupt, whileIt);
							time_printf(tmpStr);
						}
						missedInterrupt++;
						consecutiveTimeout = 0;
					}
					else
					{
						// Timed out. If we're in rapid poll mode, i.e. timeout == SPI_ISR_POLL_TIMEOUT_MS_MIN
						// then we should only allow 100 consecutive such timeouts before resuming normal
						// timeout
						consecutiveTimeout++;
						if ( (timeout == SPI_ISR_POLL_TIMEOUT_MS_MIN) &&
								(consecutiveTimeout > 100) )
						{
							// Timed out 100 times, for 300ms, without a single
							// SRDY assertion. Set back to 100ms timeout.
							consecutiveTimeout = 0;
							// Set timeout back to 100ms
							timeout = SPI_ISR_POLL_TIMEOUT_MS_MAX;
						}

						missedInterrupt = 0;
					}
					result = val;
				}
				//#endif
				debug_printf("[INT]: SRDY: %d\n", val);
				__BIG_DEBUG_ACTIVE = bigDebugWas;
				break;
			}
			case -1:
			{
				debug_printf("[INT]:poll() error \n");
				npi_ipc_errno = NPI_LNX_ERROR_SPI_EVENT_THREAD_FAILED_POLL;
				ret = NPI_LNX_FAILURE;
				consecutiveTimeout = 0;
				// Exit clean so main knows...
				npi_poll_terminate = 1;
				error_printf("%s:%d: ERROR! Terminating poll because poll() error.\n", __FUNCTION__, __LINE__);
				break;
			}
			default:
			{
				consecutiveTimeout = 0;
				//				char * buf[64];
				//				read(pollfds[0].fd, buf, 64);
				if (missedInterrupt)
				{
					if ( (pollfds[0].revents & (POLLERR | POLLHUP | POLLNVAL)) != 0 )
					{
						if ( __BIG_DEBUG_ACTIVE == TRUE )
						{
							snprintf(tmpStr, sizeof(tmpStr), "[INT]:Poll returned error (it #%d), revent[0] = %d",
								whileIt,
								pollfds[0].revents);
							time_printf(tmpStr);
						}
					}
					else
					{
						if ( __BIG_DEBUG_ACTIVE == TRUE )
						{
							snprintf(tmpStr, sizeof(tmpStr), "[INT]:Clearing missed INT (it #%d), results = %d, revent[0] = %d",
								whileIt,
								result,
								pollfds[0].revents);
							time_printf(tmpStr);
						}
						missedInterrupt = 0;
						// Set timeout back to 100ms

						if (timeout != SPI_ISR_POLL_TIMEOUT_MS_MAX)
						{
							timeout = SPI_ISR_POLL_TIMEOUT_MS_MAX;
						}
					}
				}
				result = global_srdy = HalGpioSrdyCheck(1);
				debug_printf("[INT]:Set global SRDY: %d\n", global_srdy);

				break;
			} //default:
			} //switch (result)
		} // else of if (pthread_mutex_trylock(npiSrdyLock) != 0)

		fflush(stdout);

		if (FALSE == result) //Means SRDY switch to low state
		{
			if (clock_gettime(CLOCK_MONOTONIC, &curTimeSPIisrPoll) == 0)
				// Adjust poll timeout based on time between packets, limited downwards
				// to SPI_ISR_POLL_TIMEOUT_MS_MIN and upwards to SPI_ISR_POLL_TIMEOUT_MS_MAX
			{
				// Calculate delta
				long int diffPrev;
				if (curTimeSPIisrPoll.tv_nsec >= prevTimeSPIisrPoll.tv_nsec)
				{
					diffPrev = (curTimeSPIisrPoll.tv_nsec - prevTimeSPIisrPoll.tv_nsec) / 1000;
				}
				else
				{
					diffPrev = ((curTimeSPIisrPoll.tv_nsec + 1000000000) - prevTimeSPIisrPoll.tv_nsec) / 1000;
				}
				prevTimeSPIisrPoll = curTimeSPIisrPoll;

				if (diffPrev < (SPI_ISR_POLL_TIMEOUT_MS_MIN * 1000) )
				{
					timeout = SPI_ISR_POLL_TIMEOUT_MS_MIN;
				}
				else if (diffPrev > (SPI_ISR_POLL_TIMEOUT_MS_MAX * 1000) )
				{
					if (timeout != SPI_ISR_POLL_TIMEOUT_MS_MAX)
					{
						timeout = SPI_ISR_POLL_TIMEOUT_MS_MAX;
					}
				}
				else
				{
					timeout = diffPrev / 1000;
				}
			}
			else if (timeout != SPI_ISR_POLL_TIMEOUT_MS_MAX)
			{
				// Not good, can't trust time. Set timeout to its max
				timeout = SPI_ISR_POLL_TIMEOUT_MS_MAX;
			}

			if ( (NPI_LNX_FAILURE == (ret = HalGpioMrdyCheck(1))))
			{
				debug_printf("[INT]:Fail to check MRDY \n");
				// Exit clean so main knows...
				npi_poll_terminate = 1;
				error_printf("%s:%d: ERROR! Terminating poll because HalGpioMrdyCheck() returned error %d.\n", __FUNCTION__, __LINE__, ret);
			}

			if (ret != NPI_LNX_FAILURE)
			{
				if (missedInterrupt > 0)
				{
					// Two consecutive interrupts; turn down timeout to SPI_ISR_POLL_TIMEOUT_MS_MIN
					if (timeout > SPI_ISR_POLL_TIMEOUT_MS_MIN)
					{
						timeout = SPI_ISR_POLL_TIMEOUT_MS_MIN;
					}
	
					if ( __BIG_DEBUG_ACTIVE == TRUE )
					{
						snprintf(tmpStr, sizeof(tmpStr), "[INT] Missed interrupt, but SRDY is asserted! %d (it #%d)\n",
									missedInterrupt, whileIt);
						time_printf(tmpStr);
					}
				}

				if ( __BIG_DEBUG_ACTIVE == TRUE )
				{
					snprintf(tmpStr, sizeof(tmpStr), "[INT]: Event thread is releasing SRDY mutex lock\n");
					time_printf(tmpStr);
				}

				// Unlock before signaling poll thread
				pthread_mutex_unlock(&npiSrdyLock);
				if ( __BIG_DEBUG_ACTIVE == TRUE )
				{
					//MRDY High, This is a request from the RNP
					snprintf(tmpStr, sizeof(tmpStr), "[INT]: MRDY High??: %d, send H2L to POLL (srdy = %d)\n", ret, global_srdy);
					time_printf(tmpStr);
				}
				// Before we can signal poll thread we need to make sure SynchData has completed
				pthread_mutex_lock(&npiPollLock);        // Wait for it to call cond_wait, which releases mutex
				pthread_cond_signal(&npi_srdy_H2L_poll); // Signal it (let it get cond)
				pthread_mutex_unlock(&npiPollLock);      // Release mutex so it can re-acquire it.
			}
			else
			{
				if ( __BIG_DEBUG_ACTIVE == TRUE )
				{
					snprintf(tmpStr, sizeof(tmpStr), "[INT]: Event thread is releasing SRDY mutex lock\n");
					time_printf(tmpStr);
				}
				pthread_mutex_unlock(&npiSrdyLock);
			}
		}
		else
		{
			//Unknown Event
			//Do nothing for now ...
			//debug_printf("Unknown Event or timeout, ignore it, result:%d \n",result);
			if ( __BIG_DEBUG_ACTIVE == TRUE )
			{
				snprintf(tmpStr, sizeof(tmpStr), "[INT]: Event thread is releasing SRDY mutex lock, result = %d",result);
				time_printf(tmpStr);
			}
			pthread_mutex_unlock(&npiSrdyLock);
		}
	}

	return NULL;
}
#endif //SRDY_INTERRUPT

#endif //#if (defined NPI_SPI) && (NPI_SPI == TRUE)

/**************************************************************************************************
*/
