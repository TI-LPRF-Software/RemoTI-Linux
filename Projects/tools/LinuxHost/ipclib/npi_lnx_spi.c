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

#include "aic.h"
#include "npi_lnx.h"
#include "npi_lnx_spi.h"
#include "hal_rpc.h"
#include "hal_gpio.h"

#ifdef __STRESS_TEST__
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

// -- Constants --

// -- Local Variables --

// State variable used to indicate that a device is open.
static int npiOpenFlag = FALSE;
static int polling = FALSE;


// NPI device related variables
static int              npi_poll_terminate;
static pthread_mutex_t  npiPollLock;
static pthread_mutex_t  npi_poll_mutex;
static pthread_cond_t   npi_poll_cond;
// Polling thread
static pthread_t        npiPollThread;
// Event thread
static pthread_t        npiEventThread;
static int     PollLockVar = 0;
// thread termination subroutines
static void npi_termpoll(void);
static void *npi_poll_entry(void *ptr);
#ifdef SRDY_INTERRUPT
static void *npi_event_entry(void *ptr);
static pthread_mutex_t  npi_Srdy_mutex;
#endif

// -- Forward references of local functions --

// -- Public functions --

#ifdef __STRESS_TEST__
extern struct timeval curTime, startTime;
struct timeval prevTimeI2C;
#endif //__STRESS_TEST__

// -- Private functions --
static void npi_initsyncres(void);
static int npi_initThreads(void);

void PollLockVarError(void)
{
    printf("PollLock Var ERROR, it is %d, it should be %d", PollLockVar, !PollLockVar);
    exit (-1);
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

  if(npiOpenFlag)
  {
    return FALSE;
  }

  npiOpenFlag = TRUE;

  debug_printf("Opening Device File: %s\n", portName);

  HalSpiInit(portName, ((npiSpiCfg_t*)pCfg)->speed);

  debug_printf("((npiSpiCfg_t *)pCfg)->gpioCfg[0] \t @0x%.8X\n",
		  (unsigned int)&(((npiSpiCfg_t *)pCfg)->gpioCfg[0]));
  if ( -1 == HalGpioSrdyInit(((npiSpiCfg_t *)pCfg)->gpioCfg[0]))
    HalGpioUsage();
  if ( -1 == HalGpioMrdyInit(((npiSpiCfg_t *)pCfg)->gpioCfg[1]))
    HalGpioUsage();
  if ( -1 == HalGpioResetInit(((npiSpiCfg_t *)pCfg)->gpioCfg[2]))
    HalGpioUsage();

  // initialize thread synchronization resources
  npi_initsyncres();

  // initialize SPI receive thread related variables
  npi_poll_terminate = 0;

  //Polling forbid until the Reset and Sync is done
  pthread_mutex_lock(&npiPollLock);
  if (PollLockVar) PollLockVarError(); else PollLockVar=1;

  // TODO: it is ideal to make this thread higher priority
  // but linux does not allow realtime of FIFO scheduling policy for
  // non-priviledged threads.

  // create Polling thread
  if(!npi_initThreads()) return FALSE;

  return TRUE;
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
  npi_termpoll();
  HalSpiClose();
  HalGpioSrdyClose();
  HalGpioMrdyClose();
  HalGpioResetClose();
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
 * @return      None.
 **************************************************************************************************
 */
void NPI_SPI_SendAsynchData( npiMsgData_t *pMsg )
{

 	debug_printf("Sync Lock SRDY ...");
	//Lock the polling until the command is send
	pthread_mutex_lock(&npiPollLock);
    if (PollLockVar) PollLockVarError(); else PollLockVar=1;
	debug_printf("(Sync) success \n");

  // Add Proper RPC type to header
  ((uint8*)pMsg)[RPC_POS_CMD0] = (((uint8*)pMsg)[RPC_POS_CMD0] & RPC_SUBSYSTEM_MASK) | RPC_CMD_AREQ;

  HAL_RNP_MRDY_CLR();
  //Wait for SRDY Clear
  HalGpioWaitSrdyClr();

  HalSpiWrite( 0, (uint8*) pMsg, (pMsg->len)+RPC_FRAME_HDR_SZ);

  //Wait for SRDY set
//  HalGpioWaitSrdySet();

  HAL_RNP_MRDY_SET();

  if (!PollLockVar) PollLockVarError(); else PollLockVar=0;
  pthread_mutex_unlock(&npiPollLock);
  debug_printf("Sync unLock SRDY ...");

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
 * @return      None.
 **************************************************************************************************
 */
void npi_spi_pollData(npiMsgData_t *pMsg)
  {

  int i;
#ifdef __BIG_DEBUG__
  printf("Polling Command ...");

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
  //Wait for SRDY Clear
  if (FALSE == polling)  HalGpioWaitSrdyClr();

  HalSpiWrite( 0, (uint8*) pMsg, (pMsg->len)+RPC_FRAME_HDR_SZ);

  //Wait for SRDY set, maximum 100 ms
  if (HalGpioWaitSrdySet(100) == 1)
  {
	  //We Set MRDY here to avoid GPIO latency with the beagle board
	  // if we do here later, the RNP see it low at the end of the transaction and
	  // therefore think a new transaction is starting and lower its SRDY...
	  HAL_RNP_MRDY_SET();

	  //Do a Three Byte Dummy Write to read the RPC Header
	  for (i = 0 ;i < RPC_FRAME_HDR_SZ; i++ ) ((uint8*)pMsg)[i] = 0;
	  HalSpiWrite( 0, (uint8*) pMsg, RPC_FRAME_HDR_SZ);


	  //Do a write/read of the corresponding length
	  for (i = 0 ;i < ((uint8*)pMsg)[0]; i++ ) ((uint8*)pMsg)[i+RPC_FRAME_HDR_SZ] = 0;
	  HalSpiWrite( 0, pMsg->pData, ((uint8*)pMsg)[0]);

  }
  else
  {
	  // Release MRDY as part of handshake
	  HAL_RNP_MRDY_SET();
	  pMsg->len = 0;
	  pMsg->subSys = RPC_CMD_RES7;

	  // Now don't let go until SRDY is cleared
	  HalGpioWaitSrdySet(0xFFFF);
  }
#ifdef __BIG_DEBUG__
  if (TRUE == HAL_RNP_SRDY_CLR())
    printf("SRDY set\n");
  else
    printf("SRDY Clear\n");
#endif

#ifdef __BIG_DEBUG__
  printf("Poll Response Received ...");
  for (i = 0 ; i < (RPC_FRAME_HDR_SZ+pMsg->len); i++ ) printf(" 0x%.2x", ((uint8*)pMsg)[i]);
  printf("\n");
#endif


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
 * @return      None.
 **************************************************************************************************
 */
void NPI_SPI_SendSynchData( npiMsgData_t *pMsg )
{
    int i;
	// Do not attempt to send until polling is finished

	int lockRet = 0;
	debug_printf("Sync Lock SRDY ...");
	//Lock the polling until the command is send
	lockRet = pthread_mutex_lock(&npiPollLock);
    if (PollLockVar) PollLockVarError(); else PollLockVar=1;
	debug_printf("(Sync) success \n");
	if (lockRet != 0)
    {
		printf("[ERR] Could not get lock\n");
		perror("mutex lock");
	}

    // Add Proper RPC type to header
    ((uint8*)pMsg)[RPC_POS_CMD0] = (((uint8*)pMsg)[RPC_POS_CMD0] & RPC_SUBSYSTEM_MASK) | RPC_CMD_SREQ;


#ifdef __BIG_DEBUG__
  if (TRUE == HAL_RNP_SRDY_CLR())
    printf("SRDY set\n");
  else
    printf("SRDY Clear\n");
#endif

#ifdef __BIG_DEBUG__
  printf("Sync Data Command ...");
  for (i = 0 ; i < (RPC_FRAME_HDR_SZ+pMsg->len); i++ ) printf(" 0x%.2x", ((uint8*)pMsg)[i]);
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
  //Wait for SRDY Clear
  if (FALSE == polling)  HalGpioWaitSrdyClr();

  HalSpiWrite( 0, (uint8*) pMsg, (pMsg->len)+RPC_FRAME_HDR_SZ);

  //Wait for SRDY set
  HalGpioWaitSrdySet(0xFFFF);

  //We Set MRDY here to avoid GPIO latency with the beagle board
  // if we do here later, the RNP see it low at the end of the transaction and
  // therefore think a new transaction is starting and lower its SRDY...
  HAL_RNP_MRDY_SET();

  //Do a Three Byte Dummy Write to read the RPC Header
  for (i = 0 ;i < RPC_FRAME_HDR_SZ; i++ ) ((uint8*)pMsg)[i] = 0;
    HalSpiWrite( 0, (uint8*) pMsg, RPC_FRAME_HDR_SZ);


  //Do a write/read of the corresponding length
  for (i = 0 ;i < ((uint8*)pMsg)[0]; i++ ) ((uint8*)pMsg)[i+RPC_FRAME_HDR_SZ] = 0;
  HalSpiWrite( 0, pMsg->pData, ((uint8*)pMsg)[0]);

  //End of transaction
  HAL_RNP_MRDY_SET();

#ifdef __BIG_DEBUG__
  if (TRUE == HAL_RNP_SRDY_CLR())
    printf("SRDY set\n");
  else
    printf("SRDY Clear\n");
#endif

#ifdef __BIG_DEBUG__
  printf("Sync Data Receive ...");
  for (i = 0 ; i < (RPC_FRAME_HDR_SZ+pMsg->len); i++ ) printf(" 0x%.2x", ((uint8*)pMsg)[i]);
  printf("\n");
#endif

	//Release the polling lock
    //This is the SRSP, clear out the PC type in header
    ((uint8 *)pMsg)[RPC_POS_CMD0] &=  RPC_SUBSYSTEM_MASK;

    if (!PollLockVar) PollLockVarError(); else PollLockVar=0;
	pthread_mutex_unlock(&npiPollLock);
	debug_printf("Sync unLock SRDY ...");

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
 * @return      None.
 **************************************************************************************************
 */
void NPI_SPI_ResetSlave( void )
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
    NPI_SPI_CloseDevice();
    return FALSE;
  }
#ifdef SRDY_INTERRUPT

  if(pthread_create(&npiEventThread, NULL, npi_event_entry, NULL))
  {
    // thread creation failed
    NPI_SPI_CloseDevice();
    return FALSE;
  }
#endif

  return TRUE;


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
 * @return      None.
 **************************************************************************************************
 */
void NPI_SPI_SynchSlave( void )
{

   printf("\n\n-------------------- START GPIO HANDSHAKE -------------------\n");

  printf("Handhake Lock SRDY ...");
  // Check that SRDY is low
  HalGpioWaitSrdyClr();
  // set MRDY to Low
  HAL_RNP_MRDY_CLR();
  // Wait for SRDY to go High
  HalGpioWaitSrdySet(0xFFFF);
  // Set MRDY to High
  HAL_RNP_MRDY_SET();

  HalGpioSrdyCheck(1);
  if (!PollLockVar) PollLockVarError(); else PollLockVar=0;
  pthread_mutex_unlock(&npiPollLock);
  printf("Handhake unLock SRDY ...");
  printf("(Handhake) success \n");
  printf("\n\n-------------------- END GPIO HANDSHAKE -------------------\n");

}


/* Initialize thread synchronization resources */
static void npi_initsyncres(void)
{
  // initialize all mutexes
  if (pthread_mutex_init(&npiPollLock, NULL))
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


/* SPI RX thread entry routine */
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
    if(0 == pthread_mutex_trylock(&npiPollLock))
    {
    if (PollLockVar) PollLockVarError(); else PollLockVar=1;

        debug_printf("(Poll) success \n");
        //Ready SRDY Status
      // This Test check if RNP has asserted SRDY line because it has some Data pending.
      // If SRDY is not Used, then this line need to be commented, and the Poll command need
      // to be sent regularly to check if any data is pending. this is done every 10ms (see below npi_poll_cond)
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
         debug_printf("Polling received...\n");

         //RNP is polling, retrieve the data
         *readbuf = 0; //Poll Command has zero data bytes.
         *(readbuf+1) = RPC_CMD_POLL;
         *(readbuf+2) = 0;
         npi_spi_pollData((npiMsgData_t *)readbuf);
         //pthread_mutex_unlock(&npiPollLock);
         debug_printf("Poll unLock SRDY ...\n");
        //Check if polling was successful
         if ((readbuf[RPC_POS_CMD0] & RPC_CMD_TYPE_MASK) == RPC_CMD_AREQ)
         {
        	 ((uint8 *)readbuf)[RPC_POS_CMD0] &=  RPC_SUBSYSTEM_MASK;
        	 NPI_AsynchMsgCback((npiMsgData_t *)(readbuf));
         }
         else if ((readbuf[RPC_POS_CMD0] & RPC_CMD_TYPE_MASK) == RPC_CMD_RES7)
         {
        	 printf("[WARNING] Unexpected handshake received. RNP may have reset. \n");
         }
        if (!PollLockVar) PollLockVarError(); else PollLockVar=0;
        if ( 0 == pthread_mutex_unlock(&npiPollLock))
        {
            pollStatus = TRUE;
            debug_printf("Poll unLock SRDY ...\n");
        }
        else
            debug_printf("Poll unLock SRDY FAILED...\n");
         }
         else
         {
        if (!PollLockVar) PollLockVarError(); else PollLockVar=0;
        if ( 0 == pthread_mutex_unlock(&npiPollLock))
        {
            debug_printf("Poll unLock SRDY ...\n");
         }
        else
            debug_printf("Poll unLock SRDY FAILED...\n");
	    pollStatus = FALSE;
        }
    }
    else
    {
      // this case is reached if an SREQ has been sent:
      // the SRDY will go LOW and trigger a POLL CMD.
      // We Need to continue Checking the SRDY line until it is set HIGH Again.
      // this is done in the Event Thread
      pollStatus = FALSE;
    //  debug_printf("(Poll) failed to get Lock\n");
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
      expirytime.tv_nsec = (curtime.tv_usec * 1000) + 10000000;
      if (expirytime.tv_nsec >= 1000000000) {
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
