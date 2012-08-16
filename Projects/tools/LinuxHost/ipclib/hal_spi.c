/**************************************************************************************************
  Filename:       hal_spi.c
  Revised:        $Date: 2012-03-02 08:19:21 -0800 (Fri, 02 Mar 2012) $
  Revision:       $Revision: 217 $

  Description:    This file contains the interface to the HAL SPI.


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
 *                                           INCLUDES
 **************************************************************************************************/
#include <errno.h>
#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

#include "pthread.h"

#include  "hal_types.h"
#include  "hal_spi.h"


#if (defined HAL_SPI) && (HAL_SPI == TRUE)
/**************************************************************************************************
 *                                            CONSTANTS
 **************************************************************************************************/

/**************************************************************************************************
 *                                              MACROS
 **************************************************************************************************/

/**************************************************************************************************
 *                                            TYPEDEFS
 **************************************************************************************************/

/**************************************************************************************************
 *                                         GLOBAL VARIABLES
 **************************************************************************************************/
static int spiDevFd;


static uint8 bits = 8;
static uint32 speed = 500000; //Hz
static uint16 delay = 0;

pthread_mutex_t spiMutex1 = PTHREAD_MUTEX_INITIALIZER;
/**************************************************************************************************
 *                                          FUNCTIONS - API
 **************************************************************************************************/

/**************************************************************************************************
 * @fn      HalSpiFlush
 *
 * @brief   Write a buffer to the SPI.
 *
 * @param   port - SPI port.
 *          len - Number of bytes to flush.
 *
 * @return  None
 **************************************************************************************************/
void HalSpiFlush(uint8 port, uint8 len)
{

}

/**************************************************************************************************
 * @fn      HalSpiInit
 *
 * @brief   Initialize SPI Service
 *
 * @param   None
 *
 * @return  None
 **************************************************************************************************/
void HalSpiInit(const char *devpath, uint32 speedIn)
{
  /* open the device */
  int ret;
  uint8_t mode=0;
  uint8_t bits = 8;
  speed = speedIn;

#ifdef __BIG_DEBUG__
  printf("Opening %s ...\n",devpath);
#endif
  spiDevFd = open(devpath, O_RDWR );
  if (spiDevFd <0)
  {
    perror(devpath);
    printf("%s open failed\n",devpath);
    exit(-1);
  }
  /*
   * spi mode
   */
  ret = ioctl(spiDevFd, SPI_IOC_WR_MODE, &mode);
  if (ret < 0 )
  {
    perror("can't set spi mode\n");
    exit(-1);
  }

  ret = ioctl(spiDevFd, SPI_IOC_RD_MODE, &mode);
  if (ret < 0 )
  {
    perror("can't set spi mode\n");
    exit(-1);
  }

/*
 * bits per word
 */
  ret = ioctl(spiDevFd, SPI_IOC_WR_BITS_PER_WORD, &bits);
  if (ret < 0 )
  {
    perror("can't set bits per word\n");
    exit(-1);
  }

  ret = ioctl(spiDevFd, SPI_IOC_RD_BITS_PER_WORD, &bits);
  if (ret < 0 )
  {
    perror("can't get bits per word\n");
    exit(-1);
  }

/*
 * max speed hz
 */
  ret = ioctl(spiDevFd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
  if (ret < 0 )
  {
    perror("can't set max speed hz\n");
    exit(-1);
  }

  ret = ioctl(spiDevFd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
  if (ret < 0 )
  {
    perror("can't get max speed hz\n");
    exit(-1);
  }

#ifdef __BIG_DEBUG__
  printf("spi mode: %d\n", mode);
  printf("bits per word: %d\n", bits);
  printf("max speed: %d Hz (%d KHz)\n", speed, speed/1000);
#endif


}

/**************************************************************************************************
 * @fn      HalSpiClose
 *
 * @brief   Close SPI Service
 *
 * @param   None
 *
 * @return  None
 **************************************************************************************************/
void HalSpiClose( void )
{
    close(spiDevFd);
}

/**************************************************************************************************
 * @fn      HalSpiPoll
 *
 * @brief   Poll the SPI.
 *
 * @param   None
 *
 * @return  None
 **************************************************************************************************/
void HalSpiPoll(void)
{
}
/**************************************************************************************************
 * @fn      HalSpiWrite
 *
 * @brief   Write a buffer to the SPI.
 *
 * @param   port - SPI port.
 *          pBuf - Pointer to the buffer that will be written.
 *          len - Number of bytes to write/read.
 *
 * @return  None
 **************************************************************************************************/
void HalSpiWrite(uint8 port, uint8 *pBuf, uint8 len)
{
  uint8* tx;
  uint8* rx;
  int ret;
#ifdef __BIG_DEBUG__
  uint8 i;
#endif
  (void)port;
  struct spi_ioc_transfer tr;
  tx = pBuf;
  rx = malloc (len);

  tr.tx_buf = (unsigned long)tx;
  tr.rx_buf = (unsigned long)rx;
  tr.len = len;
  tr.delay_usecs = delay;
  tr.speed_hz = speed;
  tr.bits_per_word = bits;

  pthread_mutex_lock(&spiMutex1);
#ifdef __BIG_DEBUG__
  printf("SPI: Sending ...");
  for (i = 0 ; i < len; i++ ) printf(" 0x%.2x",tx[i]);
  printf("\n");
#endif

  ret = ioctl(spiDevFd, SPI_IOC_MESSAGE(1), &tr);
  if (ret < 0 )
  {
    perror("can't write to SPI \n");
  }

  memcpy(pBuf, rx, len);
#ifdef __BIG_DEBUG__
  printf("SPI: Receive ...");
  for (i = 0 ; i < len; i++ ) printf(" 0x%.2x",rx[i]);
  printf("\n");
#endif
  free(rx);
  pthread_mutex_unlock(&spiMutex1);
}

#endif

/**************************************************************************************************
**************************************************************************************************/
