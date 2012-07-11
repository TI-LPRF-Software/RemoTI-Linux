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

// -- macros --

#ifndef TRUE
# define TRUE (1)
#endif

#ifndef FALSE
# define FALSE (0)
#endif

// -- Constants --

// -- Local Variables --

// State variable used to indicate that a device is open.
static int npiOpenFlag = FALSE;
static int polling = FALSE;


static pthread_mutex_t npiSrdyLock;
static pthread_mutex_t npi_poll_mutex;


static pthread_cond_t npi_poll_cond;


// Polling thread
static pthread_t npiPollThread;

// received frame parsing state
static int npi_poll_terminate;

// -- Forward references of local functions --

// mutex and conditional variable initialization and destruction
static void npi_initsyncres(void);
//static void npi_delsyncres(void);

// thread termination subroutines
static void npi_termpoll(void);
static void *npi_poll_entry(void *ptr);

// subroutines
// -- Public functions --

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
int NPI_SPI_OpenDevice(const char *portName, void *pCfg)
{

  if (npiOpenFlag)
  {
    return FALSE;
  }
  npiOpenFlag = TRUE;

  HalSpiInit(portName, ((npiSpiCfg_t*)pCfg)->speed);

  HalGpioSrdyInit(((npiSpiCfg_t*)pCfg)->gpioCfg[0]);

  HalGpioMrdyInit(((npiSpiCfg_t*)pCfg)->gpioCfg[1]);

  HalGpioResetInit(((npiSpiCfg_t*)pCfg)->gpioCfg[2]);

  // initialize thread synchronization resources
  npi_initsyncres();

  // initialize SPI receive thread related variables
  npi_poll_terminate = 0;

  // create Polling thread

  // TODO: it is ideal to make this thread higher priority
  // but linux does not allow realtime of FIFO scheduling policy for
  // non-priviledged threads.

  if (pthread_create(&npiPollThread, NULL, npi_poll_entry, NULL))
  {
    // thread creation failed
    HalSpiClose();
    HalGpioSrdyClose();
    HalGpioMrdyClose();
    HalGpioResetClose();
    npiOpenFlag = FALSE;
    return FALSE;
  }

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
//  npi_sendframe(pMsg->subSys | RPC_CMD_AREQ, pMsg->cmdId, pMsg->pData, pMsg->len);
#ifdef __BIG_DEBUG__
  int i;
#endif

  if (FALSE == polling)
  {
#ifdef __BIG_DEBUG__
    printf("Sync Lock SRDY ...");
#endif
    pthread_mutex_lock(&npiSrdyLock);
#ifdef __BIG_DEBUG__
    printf("(Sync) success \n");
#endif
  }

#ifdef __BIG_DEBUG__
  printf("Async Data Command ...");
  for (i = 0 ; i < (RPC_FRAME_HDR_SZ+pMsg->len); i++ ) printf(" 0x%.2x", ((uint8*)pMsg)[i]);
  printf("\n");
#endif
  // Add Proper RPC type to header
  ((uint8*)pMsg)[RPC_POS_CMD0] = (((uint8*)pMsg)[RPC_POS_CMD0] & RPC_SUBSYSTEM_MASK) | RPC_CMD_AREQ;

  HAL_RNP_MRDY_CLR();
  //Wait for SRDY Clear
  HalGpioWaitSrdyClr();

  HalSpiWrite( 0, (uint8*) pMsg, (pMsg->len)+RPC_FRAME_HDR_SZ);

  //Wait for SRDY set
  HalGpioWaitSrdySet();

  HAL_RNP_MRDY_SET();

  if (FALSE == polling)
  {
    pthread_mutex_unlock(&npiSrdyLock);
#ifdef __BIG_DEBUG__
    printf("Sync unLock SRDY ...");
#endif
  }

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

  if (FALSE == polling)
  {
#ifdef __BIG_DEBUG__
    printf("Sync Lock SRDY ...");
#endif
    pthread_mutex_lock(&npiSrdyLock);
#ifdef __BIG_DEBUG__
    printf("(Sync) success \n");
#endif

    // Add Proper RPC type to header
    ((uint8*)pMsg)[RPC_POS_CMD0] = (((uint8*)pMsg)[RPC_POS_CMD0] & RPC_SUBSYSTEM_MASK) | RPC_CMD_SREQ;
  }

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

  HAL_RNP_MRDY_CLR();
  //Wait for SRDY Clear
  if (FALSE == polling)  HalGpioWaitSrdyClr();

  HalSpiWrite( 0, (uint8*) pMsg, (pMsg->len)+RPC_FRAME_HDR_SZ);

  //Wait for SRDY set
  HalGpioWaitSrdySet();

  //We Set MRDY here to avoid GPIO latency with the beagle board
  // if we do here later, the RNP see it low at the end of the transaction and
  // therefore think a new transaction is starting and lower its SRDY...
  HAL_RNP_MRDY_SET();

  //Do a Three Byte Dummy Write to read the RPC Header
  for (i = 0 ;i < RPC_FRAME_HDR_SZ; i++ ) ((uint8*)pMsg)[i] = 0;
    HalSpiWrite( 0, (uint8*) pMsg, RPC_FRAME_HDR_SZ);


  //Do a write/read of the corresponding lenght
  for (i = 0 ;i < ((uint8*)pMsg)[0]; i++ ) ((uint8*)pMsg)[i+RPC_FRAME_HDR_SZ] = 0;
  HalSpiWrite( 0, pMsg->pData, ((uint8*)pMsg)[0]);

  //End of transaction
  //HAL_RNP_MRDY_SET();

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

  if (FALSE == polling)
  {
    //This is the SRSP, clear out the PC type in header
    ((uint8 *)pMsg)[RPC_POS_CMD0] &=  RPC_SUBSYSTEM_MASK;

    pthread_mutex_unlock(&npiSrdyLock);
#ifdef __BIG_DEBUG__
    printf("Sync unLock SRDY ...");
#endif
  }

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
  printf("\n\n-------------------- END RESET SLAVE -------------------\n");

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
  pthread_mutex_lock(&npiSrdyLock);
  printf("(Handhake) success \n");
  // Check that SRDY is low
  HalGpioWaitSrdyClr();
  // set MRDY to Low
  HAL_RNP_MRDY_CLR();
  // Wait for SRDY to go High
  HalGpioWaitSrdySet();
  // Set MRDY to High
  HAL_RNP_MRDY_SET();

  HalGpioSrdyCheck(1);
  pthread_mutex_unlock(&npiSrdyLock);
  printf("Handhake unLock SRDY ...");
  printf("\n\n-------------------- END GPIO HANDSHAKE -------------------\n");

}



// -- private functions --
/* Initialize thread synchronization resources */
static void npi_initsyncres(void)
{
  // initialize all mutexes
  pthread_mutex_init(&npiSrdyLock, NULL);
  pthread_mutex_init(&npi_poll_mutex, NULL);
  pthread_cond_init(&npi_poll_cond, NULL);

}

/* SPI RX thread entry routine */
static void *npi_poll_entry(void *ptr)
{
  uint8 readbuf[128];

  /* lock mutex in order not to lose signal */
  pthread_mutex_lock(&npi_poll_mutex);

  /* thread loop */
  while (!npi_poll_terminate)
  {

#ifdef __BIG_DEBUG__
    printf("Poll Lock SRDY ...");
#endif
    if ( 0 == pthread_mutex_trylock(&npiSrdyLock))
    {
#ifdef __BIG_DEBUG__
        printf("(Poll) success \n");
#endif
        //Ready SRDY Status
        if (TRUE == HAL_RNP_SRDY_CLR())
        {
         //   exit(-1);
#ifdef __BIG_DEBUG__
         printf("Polling received...\n");
#endif
         //RNP is polling, retrieve the data
         *readbuf = 0; //Poll Command has zero data bytes.
         *(readbuf+1) = RPC_CMD_POLL;
         *(readbuf+2) = 0;
         polling = TRUE;
         NPI_SPI_SendSynchData( (npiMsgData_t *)readbuf );
         polling = FALSE;
         pthread_mutex_unlock(&npiSrdyLock);
#ifdef __BIG_DEBUG__
         printf("Poll unLock SRDY ...\n");
#endif
         if ((readbuf[RPC_POS_CMD0] & RPC_CMD_TYPE_MASK) == RPC_CMD_AREQ)
         {
            ((uint8 *)readbuf)[RPC_POS_CMD0] &=  RPC_SUBSYSTEM_MASK;
            NPI_AsynchMsgCback((npiMsgData_t *)(readbuf));
         }
         else
         {
            printf("Polling receive but not for an ASYNC request...");
            exit(-1);
         }
        }
        pthread_mutex_unlock(&npiSrdyLock);
#ifdef __BIG_DEBUG__
        printf("Poll unLock SRDY ...\n");
#endif
    }
#ifdef __BIG_DEBUG__
    else
        printf("(Poll) failed \n");
#endif


    /* In some system, sigaction is not reliable hence poll reading even without signal. */
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
  }
  pthread_mutex_unlock(&npi_poll_mutex);

  return NULL;
}

/* Terminate Polling thread */
static void npi_termpoll(void)
{
  // send terminate signal
  pthread_mutex_lock(&npi_poll_mutex);
  npi_poll_terminate = 1;
  pthread_cond_signal(&npi_poll_cond);
  pthread_mutex_unlock(&npi_poll_mutex);

  // wait till the thread terminates
  pthread_join(npiPollThread, NULL);
}

/**************************************************************************************************
*/
