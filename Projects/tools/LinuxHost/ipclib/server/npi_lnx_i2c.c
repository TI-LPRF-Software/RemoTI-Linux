/**************************************************************************************************
  Filename:       npi_lnx_i2c.c
  Revised:        $Date: 2012-03-21 17:37:33 -0700 (Wed, 21 Mar 2012) $
  Revision:       $Revision: 246 $

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
#include <poll.h>
#include <linux/input.h>

#include "aic.h"
#include "npi_lnx.h"
#include "npi_lnx_i2c.h"
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

#if (defined NPI_I2C) && (NPI_I2C == TRUE)

// -- macros --

#ifndef TRUE
# define TRUE (1)
#endif

#ifndef FALSE
# define FALSE (0)
#endif

#define error_printf(fmt, ...) fprintf(stderr, fmt, ##__VA_ARGS__)

// -- Constants --

// -- Local Variables --

// State variable used to indicate that a device is open.
static int npiOpenFlag = FALSE;

// NPI device related variables
static int              	npi_poll_terminate;
static pthread_mutex_t  	npiPollLock;
static pthread_mutexattr_t 	npiPollLock_attr;
static pthread_mutex_t  	npi_poll_mutex;
static int 					GpioSrdyFd;
#ifndef SRDY_INTERRUPT
static pthread_cond_t   	npi_poll_cond;
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

struct timespec curTimeI2CisrPoll, prevTimeI2CisrPoll;

// -- Private functions --
static int npi_initsyncres(void);
static int npi_initThreads(void);
static int npi_i2c_pollData(npiMsgData_t *pMsg);

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
static int PollLockVarError(int originator, int shouldNotBe)
{
    error_printf("ERROR! PollLock Var is %d, it should be %d. Called by %d\n", !shouldNotBe, shouldNotBe, originator);
    npi_ipc_errno = NPI_LNX_ERROR_I2C_POLL_LOCK_VAR_ERROR;
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
int NPI_I2C_OpenDevice(const char *portName, void *pCfg)
{
	int ret = NPI_LNX_SUCCESS;
	char tmpStr[256];
	if(npiOpenFlag)
	{
	    npi_ipc_errno = NPI_LNX_ERROR_I2C_OPEN_ALREADY_OPEN;
	    return NPI_LNX_FAILURE;
	}

	npiOpenFlag = TRUE;

	if ( __BIG_DEBUG_ACTIVE == TRUE )
	{
		snprintf(tmpStr, sizeof(tmpStr), "[%s] Opening Device File: %s\n", __FUNCTION__, portName);
		time_printf(tmpStr);
	}

	ret = HalI2cInit(portName);
	if (ret != NPI_LNX_SUCCESS)
		return ret;

	if ( __BIG_DEBUG_ACTIVE == TRUE )
	{
		snprintf(tmpStr, sizeof(tmpStr), "[%s] ((npiI2cCfg_t *)pCfg)->gpioCfg[0] \t @%p\n", __FUNCTION__, (void *)&(((npiI2cCfg_t *)pCfg)->gpioCfg[0]));
		time_printf(tmpStr);
	}

  	if ( NPI_LNX_FAILURE == (GpioSrdyFd = HalGpioSrdyInit(((npiI2cCfg_t *)pCfg)->gpioCfg[0])))
	{
		error_printf("%s(): ERROR returned from HalGpioSrdyInit!\n", __FUNCTION__);
		ret = GpioSrdyFd;
	}
	if ( NPI_LNX_FAILURE == (ret = HalGpioMrdyInit(((npiI2cCfg_t *)pCfg)->gpioCfg[1])))
	{
		error_printf("%s(): ERROR returned from HalGpioMrdyInit!\n", __FUNCTION__);
	}
	if ( NPI_LNX_FAILURE == (ret = HalGpioResetInit(((npiI2cCfg_t *)pCfg)->gpioCfg[2])))
	{
		error_printf("%s(): ERROR returned from HalGpioResetInit!\n", __FUNCTION__);
	}

	if	(ret == NPI_LNX_SUCCESS)
	{
		// initialize thread synchronization resources
		if ( NPI_LNX_FAILURE == (ret = npi_initsyncres()))
			return ret;


		// Reset The RNP
		ret = NPI_I2C_ResetSlave();

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

	if ( __BIG_DEBUG_ACTIVE == TRUE )
	{
		snprintf(tmpStr, sizeof(tmpStr), "[%s] returning %d\n", __FUNCTION__, ret);
		time_printf(tmpStr);
	}
   return ret;
}

/******************************************************************************
 * @fn         NPI_I2C_CloseDevice
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
void NPI_I2C_CloseDevice(void)
{
  error_printf("Shutting down threads\n");
  npi_termpoll();
  error_printf("Closing I2C\n"); 
  HalI2cClose();
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
 * @fn          NPI_I2C_SendAsynchData
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
int NPI_I2C_SendAsynchData(npiMsgData_t *pMsg)
{
	int ret = NPI_LNX_SUCCESS;
	char tmpStr[256];
	if ( __BIG_DEBUG_ACTIVE == TRUE )
	{
		snprintf(tmpStr, sizeof(tmpStr), "[%s] Locking POLL and SRDY\n", __FUNCTION__);
		time_printf(tmpStr);
	}
	fflush(stdout);
	//Lock the polling until the command is send
	pthread_mutex_lock(&npiPollLock);
#ifdef SRDY_INTERRUPT
	pthread_mutex_lock(&npiSrdyLock);
#endif
	if (PollLockVar)
	{
		ret = PollLockVarError(__LINE__, PollLockVar);
	}
	else
	{
		PollLockVar = 1;
		if ( __BIG_DEBUG_ACTIVE == TRUE )
		{
			snprintf(tmpStr, sizeof(tmpStr), "[%s] PollLockVar set to %d\n", __FUNCTION__, PollLockVar);
			time_printf(tmpStr);
		}
	}

	if	(ret == NPI_LNX_SUCCESS)
	{
		if ( __BIG_DEBUG_ACTIVE == TRUE )
		{
			snprintf(tmpStr, sizeof(tmpStr), "[%s] (Sync) success\n", __FUNCTION__);
			time_printf(tmpStr);
		}
	}

	if ( __BIG_DEBUG_ACTIVE == TRUE )
	{
		snprintf(tmpStr, sizeof(tmpStr), "[%s] ******************** START SEND ASYNC DATA ********************\n", __FUNCTION__);
		time_printf(tmpStr);
	}
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

	if ( __BIG_DEBUG_ACTIVE == TRUE )
	{
		snprintf(tmpStr, sizeof(tmpStr), "[%s] AREQ\n", __FUNCTION__);
		time_printf(tmpStr);
	}

	//Wait for SRDY Clear
	ret = HalGpioWaitSrdyClr();

	//Send LEN, CMD0 and CMD1 (command Header)
	if (ret == NPI_LNX_SUCCESS)
		ret = HalI2cWrite(0, (uint8*) pMsg, RPC_FRAME_HDR_SZ + (pMsg->len));

	if (ret == NPI_LNX_SUCCESS)
		ret = HAL_RNP_MRDY_SET();
	else
		(void)HAL_RNP_MRDY_SET();

	if (!PollLockVar)
	{
		ret = PollLockVarError(__LINE__, !PollLockVar);
	}
	else
	{
		PollLockVar = 0;
		if ( __BIG_DEBUG_ACTIVE == TRUE )
		{
			snprintf(tmpStr, sizeof(tmpStr), "[%s] PollLockVar set to %d\n", __FUNCTION__, PollLockVar);
			time_printf(tmpStr);
		}
	}
	// Also signal poll thread so it can poll for a NULL message from the RNP. Otherwise we won't later get
	// the interrupt from the RNP, since the RNP will keep SRDY low until it has answered NULL to a poll.
	if ( __BIG_DEBUG_ACTIVE == TRUE )
	{
		snprintf(tmpStr, sizeof(tmpStr), "[%s] Signaling poll thread to perform a final 0 response poll ...\n", __FUNCTION__);
		time_printf(tmpStr);
	}
	pthread_cond_signal(&npi_srdy_H2L_poll); // Signal it (let it get cond)
	pthread_mutex_unlock(&npiPollLock);      // Release mutex so it can re-acquire it.
#ifdef SRDY_INTERRUPT
	pthread_mutex_unlock(&npiSrdyLock);
#endif
	if ( __BIG_DEBUG_ACTIVE == TRUE )
	{
		snprintf(tmpStr, sizeof(tmpStr), "[%s] Unlock SRDY mutex\n", __FUNCTION__);
		time_printf(tmpStr);
		snprintf(tmpStr, sizeof(tmpStr), "[%s] ******************** STOP SEND ASYNC DATA ********************\n", __FUNCTION__);
		time_printf(tmpStr);
	}

	return ret;
}

/**************************************************************************************************
 * @fn          npi_i2c_pollData
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
int npi_i2c_pollData(npiMsgData_t *pMsg)
{
	int	ret = NPI_LNX_SUCCESS;
	char tmpStr[256];
	if ( __BIG_DEBUG_ACTIVE == TRUE )
	{
		snprintf(tmpStr, sizeof(tmpStr), "[%s] -------------------- START POLLING DATA --------------------\n", __FUNCTION__);
		time_printf(tmpStr);
	}

#ifdef __BIG_DEBUG__
	printf("Polling Command ...");
	int i;
	for(i = 0 ; i < (RPC_FRAME_HDR_SZ+pMsg->len); i++)
		printf(" 0x%.2x", ((uint8*)pMsg)[i]);

	printf("\n");
#endif
#ifdef SRDY_INTERRUPT
	pthread_mutex_lock(&npiSrdyLock);
#endif

#ifdef __STRESS_TEST__
	//	debug_
	static struct timeval prevTimePoll;
	time_printf_always_localized("MRDY Low\n", NULL, NULL, &prevTimePoll)
#endif //__STRESS_TEST__

	ret = HAL_RNP_MRDY_CLR();

	if (NPI_LNX_SUCCESS == ret)
	{
		//This wait is only valid if RNP manage MRDY line, if not SRDY will never be set low on SREQ.
		// To avoid any problem, just add a timeout, or ignore it.
		ret = HalGpioWaitSrdyClr();

		//Send LEN, CMD0 and CMD1 (command Header) and payload
		if (ret == NPI_LNX_SUCCESS)
			ret = HalI2cWrite(0, (uint8*) pMsg, RPC_FRAME_HDR_SZ + (pMsg->len));

		if (ret == NPI_LNX_SUCCESS)
			ret = HalI2cRead(0, (uint8*) pMsg, RPC_FRAME_HDR_SZ);

		if (ret == NPI_LNX_SUCCESS)
		{
			if ( (pMsg->len == 0xFF) &&
					(pMsg->subSys == 0xFF) &&
					(pMsg->cmdId == 0xFF) )
			{
				// Do nothing
				error_printf("[POLL] WARNING: Invalid header (FF FF FF) received!\n");
			}
			else if (pMsg->len > 0)
			{
				if ( __BIG_DEBUG_ACTIVE == TRUE )
				{
					snprintf(tmpStr, sizeof(tmpStr), "[%s] I2C Read Response \n", __FUNCTION__);
					time_printf(tmpStr);
				}
				//Zero buffer for length about to read, then do a write/read of the corresponding length
				memset(pMsg->pData, 0, ((uint8*)pMsg)[0]);
				ret = HalI2cRead(0, pMsg->pData, pMsg->len);
			}
		}
	}
	if (ret == NPI_LNX_SUCCESS)
		ret = HAL_RNP_MRDY_SET();
	else
		(void)HAL_RNP_MRDY_SET();

	if ( __BIG_DEBUG_ACTIVE == TRUE )
	{
		snprintf(tmpStr, sizeof(tmpStr), "[%s] -------------------- END POLLING DATA --------------------\n", __FUNCTION__);
		time_printf(tmpStr);
	}
#ifdef SRDY_INTERRUPT
	pthread_mutex_unlock(&npiSrdyLock);
#endif

	return ret;
}

/**************************************************************************************************
 * @fn          NPI_I2C_SendSynchData
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
int NPI_I2C_SendSynchData(npiMsgData_t *pMsg)
{
	int i, ret = NPI_LNX_SUCCESS;
	int lockRetPoll = 0, lockRetSrdy = 0, strIndex = 0;
	char tmpStr[256];

	// Do not attempt to send until polling is finished

	if (__BIG_DEBUG_ACTIVE == TRUE)
	{
		snprintf(tmpStr, sizeof(tmpStr), "[%s] Lock Poll mutex \n", __FUNCTION__);
		time_printf(tmpStr);
	}
	//Lock the polling until the command is send
	lockRetPoll = pthread_mutex_lock(&npiPollLock);
	if (lockRetPoll)
	{
		error_printf("[SYNCH] [ERR] Error %d getting POLL mutex lock\n", lockRetPoll);
		perror("mutex lock");
		npi_ipc_errno = NPI_LNX_ERROR_I2C_SEND_SYNCH_FAILED_LOCK;
		ret = NPI_LNX_FAILURE;
	}
	else
	{
		if (__BIG_DEBUG_ACTIVE == TRUE)
		{
			snprintf(tmpStr, sizeof(tmpStr), "[%s] Poll mutex locked\n", __FUNCTION__);
			time_printf(tmpStr);
		}
#ifdef SRDY_INTERRUPT
		if (__BIG_DEBUG_ACTIVE == TRUE)
		{
			snprintf(tmpStr, sizeof(tmpStr), "[%s] Lock SRDY mutex \n", __FUNCTION__);
			time_printf(tmpStr);
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
			if (__BIG_DEBUG_ACTIVE == TRUE)
			{
				snprintf(tmpStr, sizeof(tmpStr), "[%s] SRDY mutex locked\n", __FUNCTION__);
				time_printf(tmpStr);
			}
		}
#endif
	}

	if (ret == NPI_LNX_SUCCESS)
	{
		if (PollLockVar)
		{
			ret = PollLockVarError(__LINE__, PollLockVar);
		}
		else
		{
			PollLockVar = 1;
			if (__BIG_DEBUG_ACTIVE == TRUE)
			{
				snprintf(tmpStr, sizeof(tmpStr), "[%s] PollLockVar set to %d\n", __FUNCTION__, PollLockVar);
				time_printf(tmpStr);
			}
		}
		if (__BIG_DEBUG_ACTIVE == TRUE)
		{
			snprintf(tmpStr, sizeof(tmpStr), "[%s] =================== START SEND SYNCH DATA ====================\n", __FUNCTION__);
			time_printf(tmpStr);
		}
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
			if (__BIG_DEBUG_ACTIVE == TRUE)
			{
				snprintf(tmpStr, sizeof(tmpStr), "[%s] Synch Data Command ...", __FUNCTION__);
				strIndex = strlen(tmpStr);
				for (i = 0 ; i < (RPC_FRAME_HDR_SZ+pMsg->len); i++ )
				{
					snprintf(tmpStr + strIndex, sizeof(tmpStr) - strIndex, " 0x%.2X", ((uint8*)pMsg)[i]);
					strIndex += 5;
				}
				snprintf(tmpStr + strIndex, sizeof(tmpStr) - strIndex, "%s \n", tmpStr);
				time_printf(tmpStr);
			}

			//This wait is only valid if RNP manage MRDY line, if not SRDY will never be set low on SREQ.
			// To avoid any problem, just add a timeout, or ignore it.
			ret = HalGpioWaitSrdyClr();

			//Send LEN, CMD0 and CMD1 (comand Header)
			if (ret == NPI_LNX_SUCCESS)
				ret = HalI2cWrite(0, (uint8*) pMsg, RPC_FRAME_HDR_SZ + (pMsg->len));

			if (ret == NPI_LNX_SUCCESS)
			{
				//Do a Three Byte Dummy Write to read the RPC Header
				memset(pMsg, 0, RPC_FRAME_HDR_SZ);
				ret = HalI2cRead( 0, (uint8*) pMsg, RPC_FRAME_HDR_SZ);

				if (__BIG_DEBUG_ACTIVE == TRUE)
				{
					snprintf(tmpStr, sizeof(tmpStr), "[%s] Synch Data Command ...", __FUNCTION__);
					strIndex = strlen(tmpStr);
					for (i = 0 ; i < (RPC_FRAME_HDR_SZ); i++ )
					{
						snprintf(tmpStr + strIndex, sizeof(tmpStr) - strIndex, " 0x%.2X", ((uint8*)pMsg)[i]);
						strIndex += 5;
					}
					snprintf(tmpStr + strIndex, sizeof(tmpStr) - strIndex, "\n");
					time_printf(tmpStr);
				}

				if (ret != NPI_LNX_SUCCESS)
				{
						error_printf("[%s] HalI2cRead() returned 0x%x, line %d, errno=0x%x\n", __FUNCTION__, ret, __LINE__, npi_ipc_errno);
				}
				else if (pMsg->len > 0)
				{
					if (pMsg->len == 0xFF && pMsg->subSys == 0xFF && pMsg->cmdId == 0xFF)
					{
						error_printf("[%s] Received 0xFF 0xFF 0xFF.  Ignoring it and returning an error!\n", __FUNCTION__);
						ret = NPI_LNX_FAILURE;
					}
					else
					{
						// Do a write/read of the corresponding length
						// Fill data with 0s first (aids in debugging):
						memset(pMsg->pData, 0, pMsg->len);
						ret = HalI2cRead( 0, pMsg->pData, pMsg->len);

						if (__BIG_DEBUG_ACTIVE == TRUE)
						{
							snprintf(tmpStr, sizeof(tmpStr), "[%s] Read %d bytes more", __FUNCTION__, pMsg->len);
							strIndex = strlen(tmpStr);
							for (i = 0 ; i < (pMsg->len); i++ )
							{
								snprintf(tmpStr, sizeof(tmpStr) - strIndex, " 0x%.2X", pMsg->pData[i]);
								strIndex += 5;
							}
							snprintf(tmpStr + strIndex, sizeof(tmpStr) - strIndex, "%s \n", tmpStr);
							time_printf(tmpStr);
						}
					}
				}
			}
		}
	}

	//Release the polling lock

	if (ret == NPI_LNX_SUCCESS)
		ret = HAL_RNP_MRDY_SET();
	else
		(void)HAL_RNP_MRDY_SET();

	if (!PollLockVar)
	{
		ret = PollLockVarError(__LINE__, !PollLockVar);
	}
	else
	{
		PollLockVar = 0;
		if (__BIG_DEBUG_ACTIVE == TRUE)
		{
			snprintf(tmpStr, sizeof(tmpStr), "[%s] PollLockVar set to %d\n", __FUNCTION__, PollLockVar);
			time_printf(tmpStr);
		}
	}


	//Release the polling lock
	if (__BIG_DEBUG_ACTIVE == TRUE)
	{
		snprintf(tmpStr, sizeof(tmpStr), "[%s] =================== END SEND SYNCH DATA ====================\n", __FUNCTION__);
		time_printf(tmpStr);
	}

	if (!lockRetPoll)
	{
		// Also signal poll thread so it can poll for a NULL message from the RNP. Otherwise we won't later get
		// the interrupt from the RNP, since the RNP will keep SRDY low until it has answered NULL to a poll.
		if (__BIG_DEBUG_ACTIVE == TRUE)
		{
			snprintf(tmpStr, sizeof(tmpStr), "[%s] Signaling poll thread to perform a final 0 response poll ...\n", __FUNCTION__);
			time_printf(tmpStr);
		}
		pthread_cond_signal(&npi_srdy_H2L_poll); // Signal it (let it get cond)
		pthread_mutex_unlock(&npiPollLock);      // Release mutex so it can re-acquire it.
	}

#ifdef SRDY_INTERRUPT
	if (!lockRetSrdy)
	{
		pthread_mutex_unlock(&npiSrdyLock);
		if (__BIG_DEBUG_ACTIVE == TRUE)
		{
			snprintf(tmpStr, sizeof(tmpStr), "[%s] Unlocked SRDY mutex\n", __FUNCTION__);
			time_printf(tmpStr);
		}
	}
#endif
	return ret;
}

/**************************************************************************************************
 * @fn          NPI_I2C_ResetSlave
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
int NPI_I2C_ResetSlave(void)
{
	int ret = NPI_LNX_SUCCESS;
	char tmpStr[128];

	snprintf(tmpStr, sizeof(tmpStr), "[%s] -------------------- START RESET SLAVE -------------------\n", __FUNCTION__);
	time_printf(tmpStr);
	ret = HalGpioReset();
	snprintf(tmpStr, sizeof(tmpStr), "[%s] Wait 1.2s for RNP to initialize after a Reset... This may change in the future, check for RTI_ResetInd()...\n", __FUNCTION__);
	time_printf(tmpStr);
	usleep(1200000); //wait 1.2s for RNP to initialize
	snprintf(tmpStr, sizeof(tmpStr), "[%s] ---------------------- END RESET SLAVE -------------------\n", __FUNCTION__);
	time_printf(tmpStr);

	return ret;
}

/* Initialize thread synchronization resources */
static int npi_initThreads(void)
{
	// create Polling thread
	// initialize I2C receive thread related variables
	npi_poll_terminate = 0;

	// TODO: it is ideal to make this thread higher priority
	// but linux does not allow realtime of FIFO scheduling policy for
	// non-priviledged threads.

	if(pthread_create(&npiPollThread, NULL, npi_poll_entry, NULL))
	{
		// thread creation failed
		NPI_I2C_CloseDevice();
		npi_ipc_errno = NPI_LNX_ERROR_I2C_OPEN_FAILED_POLL_THREAD;
		return NPI_LNX_FAILURE;
	}
#ifdef SRDY_INTERRUPT

	if(pthread_create(&npiEventThread, NULL, npi_event_entry, NULL))
	{
		// thread creation failed
		NPI_I2C_CloseDevice();
		npi_ipc_errno = NPI_LNX_ERROR_I2C_OPEN_FAILED_EVENT_THREAD;
		return NPI_LNX_FAILURE;
	}
#endif

	return NPI_LNX_SUCCESS;
}

/* Initialize thread synchronization resources */
static int npi_initsyncres(void)
{
	printf("[MUTEX] Initializing mutexes\n");
	// initialize all mutexes
	pthread_mutexattr_init(&npiPollLock_attr);
	pthread_mutexattr_settype(&npiPollLock_attr, PTHREAD_MUTEX_ERRORCHECK);
	if ( pthread_mutex_init(&npiPollLock, &npiPollLock_attr) )
//	if ( pthread_mutex_init(&npiPollLock, NULL) )
	{
		error_printf("Fail To Initialize Mutex npiPollLock\n");
	    npi_ipc_errno = NPI_LNX_ERROR_I2C_OPEN_FAILED_POLL_LOCK_MUTEX;
	    return NPI_LNX_FAILURE;
	}

	if(pthread_mutex_init(&npi_poll_mutex, NULL))
	{
		error_printf("Fail To Initialize Mutex npi_poll_mutex\n");
	    npi_ipc_errno = NPI_LNX_ERROR_I2C_OPEN_FAILED_POLL_MUTEX;
	    return NPI_LNX_FAILURE;
	}
#ifdef SRDY_INTERRUPT
	if(pthread_cond_init(&npi_srdy_H2L_poll, NULL))
	{
		error_printf("Fail To Initialize Condition npi_srdy_H2L_poll\n");
	    npi_ipc_errno = NPI_LNX_ERROR_I2C_OPEN_FAILED_SRDY_COND;
	    return NPI_LNX_FAILURE;
	}
	if (pthread_mutex_init(&npiSrdyLock, NULL))
	{
		error_printf("Fail To Initialize Mutex npiSrdyLock\n");
	    npi_ipc_errno = NPI_LNX_ERROR_I2C_OPEN_FAILED_SRDY_LOCK_MUTEX;
	    return NPI_LNX_FAILURE;
	}
#else
	if(pthread_cond_init(&npi_poll_cond, NULL))
	{
		error_printf("Fail To Initialize Condition npi_poll_cond\n");
	    npi_ipc_errno = NPI_LNX_ERROR_I2C_OPEN_FAILED_POLL_COND;
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
	int ret = NPI_LNX_SUCCESS;
	uint8 readbuf[128];
	char tmpStr[512];
#ifndef SRDY_INTERRUPT
	uint8 pollStatus = FALSE;
#endif //SRDY_INTERRUPT

	((void)ptr);
	snprintf(tmpStr, sizeof(tmpStr), "[%s] Locking Mutex for Poll Thread \n", __FUNCTION__);
	time_printf(tmpStr);

	/* lock mutex in order not to lose signal */
	pthread_mutex_lock(&npi_poll_mutex);

	snprintf(tmpStr, sizeof(tmpStr), "[%s] Poll Thread Started\n", __FUNCTION__);
	time_printf(tmpStr);

	//This lock wait for Initialization to finish (reset)
	pthread_mutex_lock(&npiPollLock);

	snprintf(tmpStr, sizeof(tmpStr), "[%s] Poll Thread Continues After Synchronization\n", __FUNCTION__);
	time_printf(tmpStr);

#ifdef SRDY_INTERRUPT
	if ( __BIG_DEBUG_ACTIVE == TRUE )
	{
		snprintf(tmpStr, sizeof(tmpStr), "[%s] Lock Poll mutex (SRDY=%d) \n", __FUNCTION__, global_srdy);
		time_printf(tmpStr);
	}
	pthread_cond_wait(&npi_srdy_H2L_poll, &npiPollLock);
	if ( __BIG_DEBUG_ACTIVE == TRUE )
	{
		snprintf(tmpStr, sizeof(tmpStr), "[%s] Locked Poll mutex (SRDY=%d) \n",	__FUNCTION__, global_srdy);
		time_printf(tmpStr);
	}
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
			ret = PollLockVarError(__LINE__, PollLockVar);
		}
		else
		{
			PollLockVar = 1;
			if ( __BIG_DEBUG_ACTIVE == TRUE )
			{
				snprintf(tmpStr, sizeof(tmpStr), "[%s] PollLockVar set to %d\n",
						__FUNCTION__, PollLockVar);
				time_printf(tmpStr);
			}
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
			if ( __BIG_DEBUG_ACTIVE == TRUE )
			{
				snprintf(tmpStr, sizeof(tmpStr), "[%s] Polling received... \n", __FUNCTION__);
				time_printf(tmpStr);
			}

			//RNP is polling, retrieve the data
			*readbuf = 0; //Poll Command has zero data bytes.
			*(readbuf+1) = RPC_CMD_POLL;
			*(readbuf+2) = 0;
			ret = npi_i2c_pollData((npiMsgData_t *)readbuf);
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
				// An error has occurred
				// Check what error it is, some errors are expected
				char *errorMsg;
				switch (npi_ipc_errno)
				{
				case NPI_LNX_ERROR_HAL_GPIO_WAIT_SRDY_CLEAR_POLL_TIMEDOUT:
					errorMsg = "npi_i2c_pollData timed out waiting for SRDY. Could be SBL which only responds to SREQ. Please check global error message\n";
					NPI_LNX_IPC_NotifyError(NPI_LNX_ERROR_MODULE_MASK(NPI_LNX_ERROR_HAL_GPIO_WAIT_SRDY_CLEAR_POLL_TIMEDOUT), errorMsg);
					// The RNP may be in SBL or may only respond to SREQ. We should not exit POLL thread.
					ret = NPI_LNX_SUCCESS;
				break;
					case  NPI_LNX_ERROR_HAL_I2C_READ_TIMEDOUT:
					// The RNP may have reset unexpectedly, keep going
					ret = NPI_LNX_SUCCESS;
					break;
				case  NPI_LNX_ERROR_HAL_I2C_WRITE_TIMEDOUT:
					// The RNP may have reset unexpectedly, keep going
						ret = NPI_LNX_SUCCESS;
					break;
				default:
					// Exit clean so main knows...
					npi_poll_terminate = 1;
					error_printf("%s:%d: ERROR! Terminating poll because error return (ret=%d, npi_ipc_errno=%d).\n", __FUNCTION__, __LINE__, ret, npi_ipc_errno);
					break;
				}
			}

			if (!PollLockVar)
			{
				ret = PollLockVarError(__LINE__, !PollLockVar);
			}
			else
			{
				PollLockVar = 0;
				if ( __BIG_DEBUG_ACTIVE == TRUE )
				{
					snprintf(tmpStr, sizeof(tmpStr), "[%s] PollLockVar set to %d\n", __FUNCTION__, PollLockVar);
					time_printf(tmpStr);
				}
			}

#ifndef SRDY_INTERRUPT
			if ( 0 == pthread_mutex_unlock(&npiPollLock))
			{
				pollStatus = TRUE;
				if ( __BIG_DEBUG_ACTIVE == TRUE )
				{
					snprintf(tmpStr, sizeof(tmpStr), "[%s] Unlock SRDY mutex \n", __FUNCTION__);
					time_printf(tmpStr);
				}
			}
			else
			{
				if ( __BIG_DEBUG_ACTIVE == TRUE )
				{
					snprintf(tmpStr, sizeof(tmpStr), "[%s] Failed to unlock SRDY mutex \n", __FUNCTION__);
					time_printf(tmpStr);
				}
			    npi_ipc_errno = NPI_LNX_ERROR_I2C_POLL_THREAD_POLL_UNLOCK;
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
				ret = PollLockVarError(__LINE__, !PollLockVar);
			}
			else
			{
				PollLockVar = 0;
				if ( __BIG_DEBUG_ACTIVE == TRUE )
				{
					snprintf(tmpStr, sizeof(tmpStr), "[%s] PollLockVar set to %d\n", __FUNCTION__, PollLockVar);
					time_printf(tmpStr);
				}
			}

#ifdef SRDY_INTERRUPT
			if ( __BIG_DEBUG_ACTIVE == TRUE )
			{
				snprintf(tmpStr, sizeof(tmpStr), "[%s] SRDY was not 0 when we expected!\n", __FUNCTION__);
				time_printf(tmpStr);
			}
#else
			if ( 0 == pthread_mutex_unlock(&npiPollLock))
			{
				if ( __BIG_DEBUG_ACTIVE == TRUE )
				{
					snprintf(tmpStr, sizeof(tmpStr), "[%s] Unlock SRDY mutex \n", __FUNCTION__);
					time_printf(tmpStr);
				}
			}
			else
			{
				if ( __BIG_DEBUG_ACTIVE == TRUE )
				{
					snprintf(tmpStr, sizeof(tmpStr), "[%s] Failed to unlock SRDY mutex \n", __FUNCTION__);
					time_printf(tmpStr);
				}
			    npi_ipc_errno = NPI_LNX_ERROR_I2C_POLL_THREAD_POLL_UNLOCK;
			    ret = NPI_LNX_FAILURE;
				npi_poll_terminate = 1;
				error_printf("%s:%d: ERROR! Terminating poll because POLL mutex unlock failed.\n", __FUNCTION__, __LINE__);
			}
			pollStatus = FALSE;
#endif //SRDY_INTERRUPT
		}

#ifdef SRDY_INTERRUPT
		snprintf(tmpStr, sizeof(tmpStr), "[%s] Unlock POLL mutex by conditional wait (SRDY=%d) \n", __FUNCTION__, global_srdy);
		time_printf(tmpStr);
		if (TRUE == HAL_RNP_SRDY_SET())
		{
			snprintf(tmpStr, sizeof(tmpStr), "[%s] Waiting for SRDY to go low\n", __FUNCTION__);
			time_printf(tmpStr);
			int condWaitRetVal = pthread_cond_wait(&npi_srdy_H2L_poll, &npiPollLock);
			if (condWaitRetVal)
			{
				snprintf(tmpStr, sizeof(tmpStr), "[%s] pthread_cond_wait returned with %d (%s, %s)\n", __FUNCTION__,
						condWaitRetVal, strerror(condWaitRetVal), strerror(errno));
				time_printf(tmpStr);
				if (condWaitRetVal == 1) exit(-1);
			}
		}
		else if (npi_poll_terminate)
		{
			// Just unlock mutex, while loop will exit next
			pthread_mutex_unlock(&npiPollLock);
		}
		else
		{
			if (PollLockVar)
			{
				ret = PollLockVarError(__LINE__, PollLockVar);
			}
			snprintf(tmpStr, sizeof(tmpStr), "[%s] SRDY found high after poll, poll again\n", __FUNCTION__);
			time_printf(tmpStr);
			// After a synchronous request we need to poll RNP again. The RNP will only release SRDY after
			// replying with 0x00 0x00 0x00 to a poll request.
		}
		snprintf(tmpStr, sizeof(tmpStr), "[%s] Locked POLL mutex because condition was met (SRDY=%d) \n", __FUNCTION__, global_srdy);
		time_printf(tmpStr);
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
	if (ret == NPI_LNX_FAILURE)
	{
		errorMsg = "[POLL] Thread exited with error. Please check global error message\n";
	}
	else
	{
		errorMsg = "[POLL] Thread exited without error\n";
	}

	NPI_LNX_IPC_NotifyError(NPI_LNX_ERROR_MODULE_MASK(NPI_LNX_ERROR_I2C_POLL_THREAD_FAILED_LOCK), errorMsg);

	return ptr;
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
#define I2C_ISR_POLL_TIMEOUT_MS_MIN		3
#define I2C_ISR_POLL_TIMEOUT_MS_MAX		100
	int result = -1;
	int missedInterrupt = 0;
	int consecutiveTimeout = 0;
	int whileIt = 0;
	int ret = NPI_LNX_SUCCESS;
	int timeout = I2C_ISR_POLL_TIMEOUT_MS_MAX;
	/* Timeout in msec. Drop down to I2C_ISR_POLL_TIMEOUT_MS_MIN if two consecutive interrupts are missed */
	struct pollfd pollfds[1];
	int val;
	char tmpStr[512];

	snprintf(tmpStr, sizeof(tmpStr), "[%s] Interrupt Event Thread Started \n", __FUNCTION__);
	time_printf(tmpStr);
	((void)ptr);
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
					snprintf(tmpStr, sizeof(tmpStr), "[%s] SRDY found to be de-asserted while we are transmitting\n", __FUNCTION__);
					time_printf(tmpStr);
				}
				else
				{
					snprintf(tmpStr, sizeof(tmpStr), "[%s] SRDY found to be asserted while we are transmitting\n", __FUNCTION__);
					time_printf(tmpStr);
				}
			}
			continue;
		}
		else
		{
			if ( __BIG_DEBUG_ACTIVE == TRUE && (consecutiveTimeout % 50 == 0) )
			{
				snprintf(tmpStr, sizeof(tmpStr), "[%s] Event thread has SRDY mutex lock, result = %d\n", __FUNCTION__, result);
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
					snprintf(tmpStr, sizeof(tmpStr), "[%s] poll() timeout (timeout set to %d), poll() returned %d\n", __FUNCTION__, timeout, result);
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
							snprintf(tmpStr, sizeof(tmpStr), "[%s] Missed interrupt: %d (it #%d)\n", __FUNCTION__, missedInterrupt, whileIt);
							time_printf(tmpStr);
						}
						missedInterrupt++;
						consecutiveTimeout = 0;
					}
					else
					{
						// Timed out. If we're in rapid poll mode, i.e. timeout == I2C_ISR_POLL_TIMEOUT_MS_MIN
						// then we should only allow 100 consecutive such timeouts before resuming normal
						// timeout
						consecutiveTimeout++;
						if ( (timeout < I2C_ISR_POLL_TIMEOUT_MS_MAX) &&
								(consecutiveTimeout > 100) )
						{
							// Timed out 100 times, for 300ms, without a single
							// SRDY assertion. Set back to 100ms timeout.
							consecutiveTimeout = 0;
							// Set timeout back to 100ms
							timeout = I2C_ISR_POLL_TIMEOUT_MS_MAX;
						}

						missedInterrupt = 0;
					}
					result = global_srdy = val; // Update global SRDY tracker here as well, as no errors has occurred.
				}
				//#endif

				if ( __BIG_DEBUG_ACTIVE == TRUE )
				{
					snprintf(tmpStr, sizeof(tmpStr), "[%s] SRDY: %d\n", __FUNCTION__, global_srdy);
					time_printf(tmpStr);
				}
				__BIG_DEBUG_ACTIVE = bigDebugWas;
				break;
			}
			case -1:
			{
				if ( __BIG_DEBUG_ACTIVE == TRUE )
				{
					snprintf(tmpStr, sizeof(tmpStr), "[%s] poll() error (%s)\n", __FUNCTION__, strerror(errno));
					time_printf(tmpStr);
				}
				npi_ipc_errno = NPI_LNX_ERROR_I2C_EVENT_THREAD_FAILED_POLL;
				ret = NPI_LNX_FAILURE;
				consecutiveTimeout = 0;
				// Exit clean so main knows...
				npi_poll_terminate = 1;
				error_printf("%s:%d: ERROR! Terminating poll because poll() error (%s).\n", __FUNCTION__, __LINE__, strerror(errno));
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
							snprintf(tmpStr, sizeof(tmpStr), "[%s] Poll returned error (it #%d), revent[0] = %d",
									__FUNCTION__, whileIt, pollfds[0].revents);
							time_printf(tmpStr);
						}
					}
					else
					{
						if ( __BIG_DEBUG_ACTIVE == TRUE )
						{
							snprintf(tmpStr, sizeof(tmpStr), "[%s] Clearing missed INT (it #%d), results = %d, revent[0] = %d",
									__FUNCTION__, whileIt, result, pollfds[0].revents);
							time_printf(tmpStr);
						}
						missedInterrupt = 0;
						// Set timeout back to 100ms

						if (timeout != I2C_ISR_POLL_TIMEOUT_MS_MAX)
						{
							timeout = I2C_ISR_POLL_TIMEOUT_MS_MAX;
						}
					}
				}
				result = global_srdy = HalGpioSrdyCheck(1);
				if ( __BIG_DEBUG_ACTIVE == TRUE )
				{
					snprintf(tmpStr, sizeof(tmpStr), "[%s] Set global SRDY: %d\n", __FUNCTION__, global_srdy);
					time_printf(tmpStr);
				}

				break;
			} //default:
			} //switch (result)
		} // else of if (pthread_mutex_trylock(npiSrdyLock) != 0)

		fflush(stdout);

		if (FALSE == result) //Means SRDY switch to low state
		{
			if (clock_gettime(CLOCK_MONOTONIC, &curTimeI2CisrPoll) == 0)
				// Adjust poll timeout based on time between packets, limited downwards
				// to I2C_ISR_POLL_TIMEOUT_MS_MIN and upwards to I2C_ISR_POLL_TIMEOUT_MS_MAX
			{
				// Calculate delta
				long int diffPrev;
				if (curTimeI2CisrPoll.tv_nsec >= prevTimeI2CisrPoll.tv_nsec)
				{
					diffPrev = (curTimeI2CisrPoll.tv_nsec - prevTimeI2CisrPoll.tv_nsec) / 1000;
				}
				else
				{
					diffPrev = ((curTimeI2CisrPoll.tv_nsec + 1000000000) - prevTimeI2CisrPoll.tv_nsec) / 1000;
				}
				prevTimeI2CisrPoll = curTimeI2CisrPoll;

				if (diffPrev < (I2C_ISR_POLL_TIMEOUT_MS_MIN * 1000) )
				{
					timeout = I2C_ISR_POLL_TIMEOUT_MS_MIN;
				}
				else if (diffPrev > (I2C_ISR_POLL_TIMEOUT_MS_MAX * 1000) )
				{
					if (timeout != I2C_ISR_POLL_TIMEOUT_MS_MAX)
					{
						timeout = I2C_ISR_POLL_TIMEOUT_MS_MAX;
					}
				}
				else
				{
					timeout = diffPrev / 1000;
				}
			}
			else if (timeout != I2C_ISR_POLL_TIMEOUT_MS_MAX)
			{
				// Not good, can't trust time. Set timeout to its max
				timeout = I2C_ISR_POLL_TIMEOUT_MS_MAX;
			}

			if ( (NPI_LNX_FAILURE == (ret = HalGpioMrdyCheck(1))))
			{
				if ( __BIG_DEBUG_ACTIVE == TRUE )
				{
					snprintf(tmpStr, sizeof(tmpStr), "[%s] Failed to check MRDY\n", __FUNCTION__);
					time_printf(tmpStr);
				}
				// Exit clean so main knows...
				npi_poll_terminate = 1;
				error_printf("%s:%d: ERROR! Terminating poll because HalGpioMrdyCheck() returned error %d.\n", __FUNCTION__, __LINE__, ret);
			}

			if (ret != NPI_LNX_FAILURE)
			{
				if (missedInterrupt > 0)
				{
					// Two consecutive interrupts; turn down timeout to I2C_ISR_POLL_TIMEOUT_MS_MIN
					if (timeout > I2C_ISR_POLL_TIMEOUT_MS_MIN)
					{
						timeout = I2C_ISR_POLL_TIMEOUT_MS_MIN;
					}
	
					if ( __BIG_DEBUG_ACTIVE == TRUE )
					{
						snprintf(tmpStr, sizeof(tmpStr), "[%s] Missed interrupt, but SRDY is asserted! %d (it #%d)\n",
								__FUNCTION__, missedInterrupt, whileIt);
						time_printf(tmpStr);
					}
				}

				if ( __BIG_DEBUG_ACTIVE == TRUE )
				{
					snprintf(tmpStr, sizeof(tmpStr), "[%s] Event thread is releasing SRDY mutex lock, line %d\n", __FUNCTION__, __LINE__);
					time_printf(tmpStr);
				}

				// Unlock before signaling poll thread
				pthread_mutex_unlock(&npiSrdyLock);
				if ( __BIG_DEBUG_ACTIVE == TRUE )
				{
					//MRDY High, This is a request from the RNP
					snprintf(tmpStr, sizeof(tmpStr), "[%s] MRDY High??: %d, send H2L to POLL (srdy = %d)\n", __FUNCTION__, ret, global_srdy);
					time_printf(tmpStr);
				}
				// Before we can signal poll thread we need to make sure SynchData has completed
				pthread_mutex_lock(&npiPollLock);        // Wait for it to call cond_wait, which releases mutex
				if (__BIG_DEBUG_ACTIVE == TRUE)
				{
					snprintf(tmpStr, sizeof(tmpStr), "[%s] Signaling poll thread to perform a poll ...\n", __FUNCTION__);
					time_printf(tmpStr);
				}
				pthread_cond_signal(&npi_srdy_H2L_poll); // Signal it (let it get cond)
				pthread_mutex_unlock(&npiPollLock);      // Release mutex so it can re-acquire it.
			}
			else
			{
				if ( __BIG_DEBUG_ACTIVE == TRUE )
				{
					snprintf(tmpStr, sizeof(tmpStr), "[%s] Event thread is releasing SRDY mutex lock, line %d\n", __FUNCTION__, __LINE__);
					time_printf(tmpStr);
				}
				pthread_mutex_unlock(&npiSrdyLock);
			}
		}
		else
		{
			//Unknown Event
			//Do nothing for now ...
			if ( __BIG_DEBUG_ACTIVE == TRUE && (consecutiveTimeout % 50 == 0) )
			{
				snprintf(tmpStr, sizeof(tmpStr), "[%s] Event thread is releasing SRDY mutex lock, result = %d, line %d\n", __FUNCTION__, result, __LINE__);
				time_printf(tmpStr);
			}
			pthread_mutex_unlock(&npiSrdyLock);
		}
	}

	pthread_cond_signal(&npi_srdy_H2L_poll);

	char const *errorMsg;
	if (ret == NPI_LNX_FAILURE)
		errorMsg = "I2C Event thread exited with error. Please check global error message\n";
	else
		errorMsg = "I2C Event thread exited without error\n";

	NPI_LNX_IPC_NotifyError(NPI_LNX_ERROR_MODULE_MASK(NPI_LNX_ERROR_I2C_EVENT_THREAD_FAILED_LOCK), errorMsg);

	return ptr;
}
#endif //SRDY_INTERRUPT

#endif // NPI_I2C
/**************************************************************************************************
*/
