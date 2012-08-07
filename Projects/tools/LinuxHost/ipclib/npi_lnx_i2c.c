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
#include <linux/poll.h>
#include <linux/input.h>

#include "aic.h"
#include "npi_lnx.h"
#include "npi_lnx_i2c.h"
#include "hal_rpc.h"
#include "hal_gpio.h"

#if (defined __STRESS_TEST__) || (defined __DEBUG_TIME__I2C)
#include <sys/time.h>
#endif // __STRESS_TEST__

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
#define debug_printf(fmt, ...)
#endif

#ifdef __DEBUG_TIME__I2C
struct timeval curTime, initTime;
#endif
// -- Constants --

// State variable used to indicate that a device is open.
static int npiOpenFlag = FALSE;

// NPI device related variables
static int              	npi_poll_terminate;
static pthread_mutex_t  	npiPollLock;
static pthread_mutexattr_t 	npiPollLock_attr;
static pthread_mutex_t  	npi_poll_mutex;
static pthread_cond_t   	npi_poll_cond;
// Polling thread
static pthread_t        	npiPollThread;
// Event thread
static pthread_t        	npiEventThread;

// For debugging mainly
uint8 writeOnce = 0;

// thread termination subroutines
static void npi_termpoll(void);
static void *npi_poll_entry(void *ptr);
#ifdef SRDY_INTERRUPT
static void *npi_event_entry(void *ptr);
static pthread_mutex_t  npi_Srdy_mutex;
#endif
// received frame parsing state
static int npi_poll_terminate;

// I2C device related variables
// -- Forward references of local functions --

// -- Public functions --

#ifdef __STRESS_TEST__
extern struct timeval curTime, startTime;
struct timeval prevTimeI2C;
#endif //__STRESS_TEST__

// -- Private functions --
static void npi_initsyncres(void);
static int npi_initThreads(void);
void npi_i2c_pollData(npiMsgData_t *pMsg);

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

  if(npiOpenFlag)
  {
    return FALSE;
  }

  npiOpenFlag = TRUE;

  debug_printf("Opening Device File: %s\n", portName);

  HalI2cInit(portName);

  debug_printf("((npiI2cCfg_t *)pCfg)->gpioCfg[0] \t @0x%.8X\n",
		  (unsigned int)&(((npiI2cCfg_t *)pCfg)->gpioCfg[0]));

  if ( -1 == HalGpioSrdyInit(((npiI2cCfg_t *)pCfg)->gpioCfg[0]))
    HalGpioUsage();
  if ( -1 == HalGpioMrdyInit(((npiI2cCfg_t *)pCfg)->gpioCfg[1]))
    HalGpioUsage();
  if ( -1 == HalGpioResetInit(((npiI2cCfg_t *)pCfg)->gpioCfg[2]))
    HalGpioUsage();

  // initialize thread synchronization resources
  npi_initsyncres();

#ifdef __DEBUG_TIME__I2C
  gettimeofday(&initTime, NULL);
  printf("NPI I2C timer started\n");
#endif //__DEBUG_TIME__I2C

  // Reset The RNP
  NPI_I2C_ResetSlave();

  if(!npi_initThreads()) return FALSE;


  return TRUE;
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
  npi_termpoll();
  HalI2cClose();
  HalGpioSrdyClose();
  HalGpioMrdyClose();
  HalGpioResetClose();
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
 * @return      None.
 **************************************************************************************************
 */
void NPI_I2C_SendAsynchData(npiMsgData_t *pMsg)
{
	debug_printf("Sync Lock SRDY ...");
	//Lock the polling until the command is send
	if (0 == pthread_mutex_lock(&npiPollLock))
	{
		writeOnce = 0;

#ifdef __DEBUG_TIME__I2C
		//	debug_
		gettimeofday(&curTime, NULL);
		printf("[MUTEX %.5ld.%.6ld] NPI_I2C_SendAsynchData has lock (%d cnt: %d) [tid: %ld]\n",
				curTime.tv_sec - initTime.tv_sec,
				curTime.tv_usec,
				npiPollLock,
				((int *)&npiPollLock)[1],
				syscall(224));
#elif (defined __DEBUG_MUTEX__)
	printf("[MUTEX] NPI_I2C_SendAsynchData has lock (%d @ 0x%.16X)\n", npiPollLock, &npiPollLock);
#endif //__DEBUG_TIME__I2C
	}
    else
    {
#if (defined __DEBUG_MUTEX__)
		printf("[MUTEX-ERR] NPI_I2C_SendAsynchData has lock (%d @ 0x%.16X)\n", npiPollLock, &npiPollLock);
#endif //(defined __DEBUG_MUTEX__)
    	perror("Mutex unlock Poll:");
    }
	debug_printf("(Sync) success \n");

	// Add Proper RPC type to header
	((uint8*)pMsg)[RPC_POS_CMD0] = (((uint8*)pMsg)[RPC_POS_CMD0] & RPC_SUBSYSTEM_MASK) | RPC_CMD_AREQ;

	HAL_RNP_MRDY_CLR();

	//This wait is only valid if RNP manage MRDY line, if not SRDY will never be set low on SREQ.
	// To avoid any problem, just add a timeout, or ignore it.
	HalGpioWaitSrdyClr();
//	usleep(1000);	// NOTE! This function seems unreliable.

	//Send LEN, CMD0 and CMD1 (comand Header)
	HalI2cWrite(0, (uint8*) pMsg, RPC_FRAME_HDR_SZ + (pMsg->len));

	HAL_RNP_MRDY_SET();

	if (0 == pthread_mutex_unlock(&npiPollLock))
	{
#ifdef __DEBUG_TIME__I2C
		//	debug_
		gettimeofday(&curTime, NULL);
		printf("[MUTEX %.5ld.%.6ld] NPI_I2C_SendAsynchData released (%d cnt: %d) [tid: %ld]\n",
				curTime.tv_sec - initTime.tv_sec,
				curTime.tv_usec,
				npiPollLock,
				((int *)&npiPollLock)[1],
				syscall(224));
#elif (defined __DEBUG_MUTEX__)
		printf("[MUTEX] NPI_I2C_SendAsynchData released (%d @ 0x%.16X)\n", npiPollLock, &npiPollLock);
#endif //__DEBUG_TIME__I2C
	}
    else
    {
#if (defined __DEBUG_MUTEX__)
		printf("[MUTEX-ERR] NPI_I2C_SendAsynchData released (%d @ 0x%.16X)\n", npiPollLock, &npiPollLock);
#endif //(defined __DEBUG_MUTEX__)
    	perror("Mutex unlock Poll:");
    }
	debug_printf("Sync unLock SRDY ...");

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
 * @return      None.
 **************************************************************************************************
 */
void npi_i2c_pollData(npiMsgData_t *pMsg)
{

#ifdef __BIG_DEBUG__
  printf("Polling Command ...");

  int i;
  for(i = 0 ; i < (RPC_FRAME_HDR_SZ+pMsg->len); i++)
    printf(" 0x%.2x", ((uint8*)pMsg)[i]);

  printf("\n");
#endif

#ifdef __STRESS_TEST__
  //	debug_
  gettimeofday(&curTime, NULL);
  long int diffPrev;
  int t = 0;
  if (curTime.tv_usec >= prevTimeI2C.tv_usec)
  {
	  diffPrev = curTime.tv_usec - prevTimeI2C.tv_usec;
  }
  else
  {
	  diffPrev = (curTime.tv_usec + 1000000) - prevTimeI2C.tv_usec;
	  t = 1;
  }

  prevTimeI2C = curTime;

  printf("[POLL %.5ld.%.6ld (+%ld.%6ld)] MRDY Low \n",
		  curTime.tv_sec - startTime.tv_sec,
		  curTime.tv_usec,
		  curTime.tv_sec - prevTimeI2C.tv_sec - t,
		  diffPrev);
#endif //__STRESS_TEST__
  HAL_RNP_MRDY_CLR();

//This wait is only valid if RNP manage MRDY line, if not SRDY will never be set low on SREQ.
// To avoid any problem, just add a timeout, or ignore it.
  HalGpioWaitSrdyClr();
//  usleep(1000);

  //Send LEN, CMD0 and CMD1 (comand Header)
  HalI2cWrite(0, (uint8*) pMsg, RPC_FRAME_HDR_SZ + (pMsg->len));

  HalI2cRead(0, (uint8*) pMsg, RPC_FRAME_HDR_SZ);

  if(pMsg->len != 0)
    //Read RSP Data
    HalI2cRead(0, (uint8*) &(pMsg->pData[0]), (pMsg->len));

  HAL_RNP_MRDY_SET();

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
 * @return      None.
 **************************************************************************************************
 */
void NPI_I2C_SendSynchData(npiMsgData_t *pMsg)
{
	// Do not attempt to send until polling is finished

	int lockRet = 0;
	debug_printf("Sync Lock SRDY ...");
	//Lock the polling until the command is send
	lockRet = pthread_mutex_lock(&npiPollLock);
	debug_printf("(Sync) success \n");
	if (lockRet != 0)
	{
		printf("[ERR] Could not get lock\n");
		perror("mutex lock");
	}
	else
	{
		writeOnce = 0;
#ifdef __DEBUG_TIME__I2C
		//	debug_
		gettimeofday(&curTime, NULL);
		printf("[MUTEX %.5ld.%.6ld] NPI_I2C_SendSynchData has lock (%d cnt: %d) [tid: %ld]\n",
				curTime.tv_sec - initTime.tv_sec,
				curTime.tv_usec,
				npiPollLock,
				((int *)&npiPollLock)[1],
				syscall(224));
#elif (defined __DEBUG_MUTEX__)
		printf("[MUTEX] NPI_I2C_SendSynchData has lock (%d @ 0x%.16X)\n", npiPollLock, &npiPollLock);
#endif //__DEBUG_TIME__I2C
	}

	// Add Proper RPC type to header
	((uint8*)pMsg)[RPC_POS_CMD0] = (((uint8*)pMsg)[RPC_POS_CMD0] & RPC_SUBSYSTEM_MASK) | RPC_CMD_SREQ;

#ifdef __BIG_DEBUG__
	printf("Sync Data Command ...");

	int i;
	for(i = 0 ; i < (RPC_FRAME_HDR_SZ+pMsg->len); i++)
		printf(" 0x%.2x", ((uint8*)pMsg)[i]);

	printf("\n");
#endif

#ifdef __STRESS_TEST__
	//	debug_
	gettimeofday(&curTime, NULL);
	long int diffPrev;
	int t = 0;
	if (curTime.tv_usec >= prevTimeI2C.tv_usec)
	{
		diffPrev = curTime.tv_usec - prevTimeI2C.tv_usec;
	}
	else
	{
		diffPrev = (curTime.tv_usec + 1000000) - prevTimeI2C.tv_usec;
		t = 1;
	}

	prevTimeI2C = curTime;

	printf("[--> %.5ld.%.6ld (+%ld.%6ld)] MRDY Low \n",
			curTime.tv_sec - startTime.tv_sec,
			curTime.tv_usec,
			curTime.tv_sec - prevTimeI2C.tv_sec - t,
			diffPrev);
#endif //__STRESS_TEST__
	HAL_RNP_MRDY_CLR();

	//This wait is only valid if RNP manage MRDY line, if not SRDY will never be set low on SREQ.
	// To avoid any problem, just add a timeout, or ignore it.
	HalGpioWaitSrdyClr();
//	usleep(1000);

	//Send LEN, CMD0 and CMD1 (comand Header)
	HalI2cWrite(0, (uint8*) pMsg, RPC_FRAME_HDR_SZ + (pMsg->len));

	HalI2cRead(0, (uint8*) pMsg, RPC_FRAME_HDR_SZ);

	if(pMsg->len != 0)
		//Read RSP Data
		HalI2cRead(0, (uint8*) &(pMsg->pData[0]), (pMsg->len));

	//Release the polling lock
	//This is the SRSP, clear out the PC type in header
	((uint8 *)pMsg)[RPC_POS_CMD0] &=  RPC_SUBSYSTEM_MASK;

	HAL_RNP_MRDY_SET();

	if (0 == pthread_mutex_unlock(&npiPollLock))
	{
#ifdef __DEBUG_TIME__I2C
		//	debug_
		gettimeofday(&curTime, NULL);
		printf("[MUTEX %.5ld.%.6ld] NPI_I2C_SendSynchData released (%d cnt: %d) [tid: %ld]\n",
				curTime.tv_sec - initTime.tv_sec,
				curTime.tv_usec,
				npiPollLock,
				((int *)&npiPollLock)[1],
				syscall(224));
#elif (defined __DEBUG_MUTEX__)
		printf("[MUTEX] NPI_I2C_SendSynchData released (%d @ 0x%.16X)\n", npiPollLock, &npiPollLock);
#endif //__DEBUG_TIME__I2C
	}
    else
    {
#if (defined __DEBUG_MUTEX__)
		printf("[MUTEX-ERR] NPI_I2C_SendAsynchData released (%d @ 0x%.16X)\n", npiPollLock, &npiPollLock);
#endif //(defined __DEBUG_MUTEX__)
    	perror("Mutex unlock Poll:");
    }
	debug_printf("Sync unLock SRDY ...");
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
 * @return      None.
 **************************************************************************************************
 */
void NPI_I2C_ResetSlave(void)
{

  printf("\n\n-------------------- START RESET SLAVE -------------------\n");
  HalGpioReset();
  printf("Wait 1.2s for RNP to initialize after a Reset... This may change in the future, check for RTI_ResetInd()...\n");
  usleep(1200000); //wait 1.2s for RNP to initialize
  printf("-------------------- END RESET SLAVE -------------------\n");

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
    return FALSE;
  }
#ifdef SRDY_INTERRUPT

  if(pthread_create(&npiEventThread, NULL, npi_event_entry, NULL))
  {
    // thread creation failed
    NPI_I2C_CloseDevice();
    return FALSE;
  }
#endif

  return TRUE;


}

/* Initialize thread synchronization resources */
static void npi_initsyncres(void)
{
	printf("[MUTEX] Initializing mutexes\n");
	// initialize all mutexes
	pthread_mutexattr_init(&npiPollLock_attr);
	pthread_mutexattr_settype(&npiPollLock_attr, PTHREAD_MUTEX_ERRORCHECK);
	if ( pthread_mutex_init(&npiPollLock, &npiPollLock_attr) )
//	if ( pthread_mutex_init(&npiPollLock, NULL) )
	{
		printf("Fail To Initialize Mutex npiPollLock\n");
		exit(-1);
	}

	if(pthread_mutex_init(&npi_poll_mutex, NULL))
	{
		printf("Fail To Initialize Mutex npi_poll_mutex\n");
		exit(-1);
	}
#ifdef SRDY_INTERRUPT
	if(pthread_mutex_init(&npi_Srdy_mutex, NULL))
	{
		printf("Fail To Initialize Mutex npi_Srdy_mutex\n");
		exit(-1);
	}
#endif

	if(pthread_cond_init(&npi_poll_cond, NULL))
	{
		printf("Fail To Initialize Condition npi_poll_cond\n");
		exit(-1);
	}

}


/* I2C RX thread entry routine */
static void *npi_poll_entry(void *ptr)
{
  uint8 readbuf[128];
  uint8 pollStatus = FALSE;

  /* lock mutex in order not to lose signal */
  pthread_mutex_lock(&npi_poll_mutex);

  printf("POLL: Thread Started \n");

  /* thread loop */
  while(!npi_poll_terminate)
  {

#ifdef SRDY_INTERRUPT
    debug_printf("POLL: Lock SRDY mutex \n");
    pthread_mutex_lock(&npi_Srdy_mutex);
    debug_printf("POLL: Locked SRDY mutex \n");
#endif
    if(0 == pthread_mutex_lock(&npiPollLock))
    {
    	if (writeOnce < 4)
    	{
#ifdef __DEBUG_TIME__I2C
    		//	debug_
    		gettimeofday(&curTime, NULL);
    		printf("[MUTEX %.5ld.%.6ld] npi_i2c_pollData has lock (%d cnt: %d) [tid: %ld]\n",
				curTime.tv_sec - initTime.tv_sec,
				curTime.tv_usec,
				npiPollLock,
				((int *)&npiPollLock)[1],
				syscall(224));
#elif (defined __DEBUG_MUTEX__)
    		printf("[MUTEX] npi_i2c_pollData has lock (%d @ 0x%.16X)\n", npiPollLock, &npiPollLock);
#endif //__DEBUG_TIME__I2C
    	}

    	//Ready SRDY Status
    	// This Test check if RNP has asserted SRDY line because it has some Data pending.
    	// If SRDY is not Used, thgen this line need to be commented, and the Poll command need
    	// to be send regulary to check if any data is pending. this is done every 10ms (see below npi_poll_cond)
#ifndef SRDY_INTERRUPT
    	if(TRUE == HAL_RNP_SRDY_CLR())
#else
    		//Interruption case, In case of a SREQ, SRDY will go low a end generate an event.
    		// the npiPollLock will prevent us to arrive to this test,
    		// BUT an AREQ can immediately follow  a SREQ: SRDY will stay low for the whole process
    		// In this case, we need to check that the SRDY line is still LOW or is HIGH.
    		if(1)
#endif
    		{
    			//   exit(-1);
    			//RNP is polling, retrieve the data
    			*readbuf = 0; //Poll Command has zero data bytes.
    			*(readbuf+1) = RPC_CMD_POLL;
    			*(readbuf+2) = 0;

    			//Send the Polling Command,
    			npi_i2c_pollData((npiMsgData_t *)readbuf);

#ifdef __DEBUG_TIME__I2C
    			  //	debug_
    			  gettimeofday(&curTime, NULL);
#endif //__DEBUG_TIME__I2C

    	        if(0 == pthread_mutex_unlock(&npiPollLock))
    	        {
#ifdef __DEBUG_TIME__I2C
    	        	printf("[MUTEX %.5ld.%.6ld] npi_i2c_pollData released (%d cnt: %d) [tid: %ld]\n",
    	        			curTime.tv_sec - initTime.tv_sec,
    	        			curTime.tv_usec,
    	        			npiPollLock,
    	        			((int *)&npiPollLock)[1],
    	        			syscall(224));
#elif (defined __DEBUG_MUTEX__)

    	        	printf("[MUTEX] npi_i2c_pollData released (%d @ 0x%.16X)\n",
    	        			npiPollLock,
    	        			&npiPollLock);
#endif //__DEBUG_TIME__I2C
    	        	pollStatus = TRUE;
    	        }
    	        else
    	        {
#if (defined __DEBUG_MUTEX__)
    	    		printf("[MUTEX-ERR] npi_i2c_pollData released (%d @ 0x%.16X)\n", npiPollLock, &npiPollLock);
#endif //(defined __DEBUG_MUTEX__)
    	        	perror("Mutex unlock Poll:");
    	        }

    			//Check if polling was successful
    			if((readbuf[RPC_POS_CMD0] & RPC_CMD_TYPE_MASK) == RPC_CMD_AREQ)
    			{
    				((uint8 *)readbuf)[RPC_POS_CMD0] &=  RPC_SUBSYSTEM_MASK;
    				NPI_AsynchMsgCback((npiMsgData_t *)(readbuf));
    				writeOnce = 0;
    			}
    		}
    		else
    		{
    			pollStatus = FALSE;
    		}


    	if (pollStatus == FALSE)
    	{
    		if(0 == pthread_mutex_unlock(&npiPollLock))
    		{
    			if (writeOnce < 4)
    			{
#ifdef __DEBUG_TIME__I2C
    				//	debug_
    				gettimeofday(&curTime, NULL);
    				printf("[MUTEX %.5ld.%.6ld] npi_i2c_pollData released (%d cnt: %d) [tid: %ld]\n",
    	        			curTime.tv_sec - initTime.tv_sec,
    	        			curTime.tv_usec,
    	        			npiPollLock,
    	        			((int *)&npiPollLock)[1],
    	        			syscall(224));
#elif (defined __DEBUG_MUTEX__)
    				printf("[MUTEX] npi_i2c_pollData released (%d @ 0x%.16X)\n", npiPollLock, &npiPollLock);
#endif //__DEBUG_TIME__I2C
    				writeOnce++;
    			}
    		}
    		else
    		{
#if (defined __DEBUG_MUTEX__)
    			printf("[MUTEX-ERR] npi_i2c_pollData released (%d @ 0x%.16X)\n", npiPollLock, &npiPollLock);
#endif //(defined __DEBUG_MUTEX__)
    			perror("Mutex unlock Poll:");
    		}
    	}
    }
    else
    {
      // this case is reach if a SREQ has been send:
      // the SRDY will go LOW and trigger a POLL CMD.
      // We Need to continue Checking the SRDY line until it is set HIGH Again.
      // this is done in the Event Thread
      pollStatus = FALSE;
      debug_printf("(Poll) failed \n");
    }

#ifdef SRDY_INTERRUPT
    debug_printf("POLL: unLock SRDY mutex \n");
     pthread_mutex_unlock(&npi_Srdy_mutex);
    debug_printf("POLL: unLocked SRDY mutex \n");
#else
    if (!pollStatus) //If previous poll failed, wait 10ms to do another one, else do it right away to empty the RNP queue.
    {
      struct timespec expirytime;
      struct timeval curtime;

      gettimeofday(&curtime, NULL);
      expirytime.tv_sec = curtime.tv_sec;
      expirytime.tv_nsec = (curtime.tv_usec * 1000) + 1000000; //Wait 1000us for next polling

      if(expirytime.tv_nsec >= 1000000000)
      {
        expirytime.tv_nsec -= 1000000000;
        expirytime.tv_sec++;
      }

      pthread_cond_timedwait(&npi_poll_cond, &npi_poll_mutex, &expirytime);
    }
#endif
  }
  pthread_mutex_unlock(&npi_poll_mutex);

  return NULL;
}

/* Terminate Polling thread */
static void npi_termpoll(void)
{
  //This will cause the Thread to exit
  npi_poll_terminate = 1;

  // In case of polling mechanism, send the Signal to continue
  pthread_mutex_lock(&npi_poll_mutex);
  pthread_cond_signal(&npi_poll_cond);
  pthread_mutex_unlock(&npi_poll_mutex);

#ifdef SRDY_INTERRUPT
  pthread_mutex_destroy(&npi_Srdy_mutex);
#endif
  pthread_mutex_destroy(&npi_poll_mutex);

  // wait till the thread terminates
  pthread_join(npiPollThread, NULL);

#ifdef SRDY_INTERRUPT
  pthread_join(npiEventThread, NULL);
#endif //SRDY_INTERRUPT
}

#ifdef SRDY_INTERRUPT
static int handle_event (int fd)
{
  struct input_event ev[64];
  int i, rd,res=-1;

#ifdef __BIG_DEBUG__
  struct timeval Previous_EventTime;
  time_t nowtime;
  struct tm *nowtm;
  char tmbuf[64], buf[64];
#endif

  rd = read(fd, ev, sizeof(struct input_event) * 64);

  if (rd < (int) sizeof(struct input_event))
  {
    printf("expected %d bytes, got %d\n", (int) sizeof(struct input_event), rd);
    perror("\nevtest: error reading");
    return res;
  }

    //For Loop in case several Event are Present
  for (i = 0; i < rd / sizeof(struct input_event); i++)
  {
    //check Only SW_KEY type
    if (1 == ev[i].type)
    {
#ifdef __BIG_DEBUG__
        nowtime = ev[i].time.tv_sec;
        nowtm = localtime(&nowtime);
        strftime(tmbuf, sizeof tmbuf, "%H:%M:%S", nowtm);
        snprintf(buf, sizeof buf, "%s.%06d, delta (ms) %d", tmbuf, (int) ev[i].time.tv_usec, (int) ((ev[i].time.tv_usec- Previous_EventTime.tv_usec)/1000));
        printf("\t #%d Event: time %s, ", i, buf);
        printf("type %d , code %d, ",
                  ev[i].type,
                  ev[i].code);
        printf("value %d\n", ev[i].value);
        Previous_EventTime = ev[i].time;
#endif
        if (ev[i].code == BTN_EXTRA)
        {
            if (ev[i].value == 0)
             res = TRUE;
            else
             res = FALSE;
        }
    }
  }
  return res;
}
static void *npi_event_entry(void *ptr)
{
  int result = -1;
  struct pollfd pollfds[2];
  int timeout = 2000; /* Timeout in msec. */
  int SrdyAsserted = TRUE;

  pollfds[0].fd = open("/dev/input/event0",O_RDWR);

  if ( pollfds[0].fd  < 0 ) {
    printf(" failed to open /dev/input/event0 \n");
    exit (1);
  }
  pollfds[0].events = POLLIN;      /* Wait for input */

  printf("EVENT: Thread Started \n");

  debug_printf("EVENT: unLock SRDY mutex \n");
  pthread_mutex_unlock(&npi_Srdy_mutex);
  debug_printf("EVENT: unLocked SRDY mutex \n");

  /* thread loop */
  while(!npi_poll_terminate)
  {
    {
      result = poll (&pollfds, 1, timeout);
      switch (result)
      {
        case 0:
          //Should not happen by default no Timeout.
          result = 2; //FORCE WRONG RESULT TO AVOID DEADLOCK CAUSE BY TIMEOUT
          printf ("poll() timeout\n");
          break;
        case -1:
          printf ("poll() error \n");
          exit (1);

         default:
          result = handle_event(pollfds[0].fd);
      }
    }
    if (TRUE == result)
    {
      //Allow POLL Cmd to be send
      if (SrdyAsserted == FALSE) //This test to prevent 2 consecutive unlock
      {
          SrdyAsserted=TRUE;
          debug_printf("EVENT: unLock SRDY mutex \n");
          pthread_mutex_unlock(&npi_Srdy_mutex);
          debug_printf("EVENT: unLocked SRDY mutex \n");
      }
      else
          debug_printf("EVENT: Already unlocked \n");

    }
    else if (FALSE == result)
    {
      //Block POLL Cmd until next SRDY assertion
      if (SrdyAsserted == TRUE) //This test to prevent 2 consecutive lock
      {
          SrdyAsserted=FALSE;
          debug_printf("EVENT: Lock SRDY mutex \n");
          pthread_mutex_lock(&npi_Srdy_mutex);
          debug_printf("EVENT: Locked SRDY mutex \n");
      }
      else
          debug_printf("EVENT: Already unlocked \n");
    }
    else
    {
        //Unknow Event
        //Do nothing for now ...
        debug_printf ("Unknow Event, ignore it\n");
    }

  }

  return NULL;
}

#endif
/**************************************************************************************************
*/
