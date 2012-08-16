/**************************************************************************************************
  Filename:       hal_gpio.c
  Revised:        $Date: 2012-03-15 13:45:31 -0700 (Thu, 15 Mar 2012) $
  Revision:       $Revision: 237 $

  Description:    This file contains the interface to the HAL GPIO.


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
#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <sys/poll.h>

#include "pthread.h"

//#include  "hal_board.h"
#include  "hal_types.h"
#include  "hal_gpio.h"

#ifdef __STRESS_TEST__
#include <sys/time.h>
#endif // __STRESS_TEST__
#elif defined __DEBUG_TIME__
#include <sys/time.h>
#endif //__DEBUG_TIME__

/**************************************************************************************************
 *                                            CONSTANTS
 **************************************************************************************************/

/**************************************************************************************************
 *                                              MACROS
 **************************************************************************************************/

#ifdef __BIG_DEBUG__
#define debug_printf(fmt, ...) printf( fmt, ##__VA_ARGS__)
#else
#define debug_printf(fmt, ...)
#endif

/**************************************************************************************************
 *                                            TYPEDEFS
 **************************************************************************************************/

/**************************************************************************************************
 *                                         GLOBAL VARIABLES
 **************************************************************************************************/
static int gpioSrdyFd;
static int gpioMrdyFd;
static int gpioResetFd;

static halGpioCfg_t srdyGpioCfg;
static halGpioCfg_t mrdyGpioCfg;
static halGpioCfg_t resetGpioCfg;

#ifdef __STRESS_TEST__
extern struct timeval curTime, startTime;
extern struct timeval prevTimeI2C;
#elif defined __DEBUG_TIME__
struct timeval curTime, startTime;
struct timeval prevTime;
#endif //__DEBUG_TIME__

/**************************************************************************************************
 *                                          FUNCTIONS - API
 **************************************************************************************************/
/**************************************************************************************************
 * @fn      HalGpioUsage
 *
 * @brief   Close SRDY IO
 *
 * @param   None
 *
 * @return  None
 **************************************************************************************************/
void HalGpioUsage(void)
{
	debug_printf("\n*********************\n");
	debug_printf("*********************\n");
	debug_printf(" !! GPIO Failed to initialize !!\n");
	debug_printf(" This software is provide as a example and should be used on the following Plateform:\n");
	debug_printf(" Beagle Board xM Rev C with a TinCanTools.com Trainee REV-B board\n");
	debug_printf(" If you used another Plateform or do not SRDY/MRDY/RESET, you need to update the following to match your plateform technical characteristic:\n");
	debug_printf(" hal_gpio.c, hal_gpio., file  npi_lnx_i2c can also be modified if you do not used thoses GPIOs.\n");
	debug_printf(" The beagle plateform is using the following static configuration: \n");
	debug_printf(" MRDY:n");
	debug_printf("      Level Shifter (out, high) dir: %s value: %s\n", mrdyGpioCfg.levelshifter.direction, mrdyGpioCfg.levelshifter.value);
	debug_printf("      MRDY GPIO dir: %s value: %s\n", mrdyGpioCfg.gpio.direction, mrdyGpioCfg.gpio.value);
	debug_printf(" SRDY:n");
	debug_printf("      Level Shifter (out, high) dir: %s value: %s\n", srdyGpioCfg.levelshifter.direction, srdyGpioCfg.levelshifter.value);
	debug_printf("      SRDY GPIO dir: %s value: %s\n", srdyGpioCfg.gpio.direction, srdyGpioCfg.gpio.value);
	debug_printf(" RESET:n");
	debug_printf("      Level Shifter (out, high) dir: %s value: %s\n", resetGpioCfg.levelshifter.direction, resetGpioCfg.levelshifter.value);
	debug_printf("      SRDY GPIO dir: %s value: %s\n", resetGpioCfg.gpio.direction, resetGpioCfg.gpio.value);
	debug_printf("\n*********************\n");
}

/**************************************************************************************************
 * @fn      HalGpioSrdyClose
 *
 * @brief   Close SRDY IO
 *
 * @param   None
 *
 * @return  None
 **************************************************************************************************/
void HalGpioSrdyClose(void)
{
	close(gpioSrdyFd);
}

/**************************************************************************************************
 * @fn      HalGpioMrdyClose
 *
 * @brief   Close SRDY IO
 *
 * @param   None
 *
 * @return  None
 **************************************************************************************************/
void HalGpioMrdyClose(void)
{
	close(gpioMrdyFd);
}

/**************************************************************************************************
 * @fn      HalGpioResetClose
 *
 * @brief   Close SRDY IO
 *
 * @param   None
 *
 * @return  None
 **************************************************************************************************/
void HalGpioResetClose(void)
{
	close(gpioResetFd);
}

/**************************************************************************************************
 * @fn      HalGpioSrdyInit
 *
 *
 * @brief   Initialise SRDY GPIO.
 *
 * @param   gpioCfg - SRDY pin configuration parameters
 *
 * @return  0: success ; -1 error
 **************************************************************************************************/
uint8 HalGpioSrdyInit(halGpioCfg_t *gpioCfg)
{
	memcpy(srdyGpioCfg.gpio.value,
			gpioCfg->gpio.value,
			strlen(gpioCfg->gpio.value));
	debug_printf("resetGpioCfg.gpio.value = '%s'\n", srdyGpioCfg.gpio.value);
	memcpy(srdyGpioCfg.gpio.direction,
			gpioCfg->gpio.direction,
			strlen(gpioCfg->gpio.direction));
	debug_printf("srdyGpioCfg.gpio.direction = '%s'\n", srdyGpioCfg.gpio.direction);
	srdyGpioCfg.gpio.active_high_low = gpioCfg->gpio.active_high_low;
	memcpy(srdyGpioCfg.levelshifter.value,
			gpioCfg->levelshifter.value,
			strlen(gpioCfg->levelshifter.value));
	debug_printf("srdyGpioCfg.levelshifter.value = '%s'\n", srdyGpioCfg.levelshifter.value);
	memcpy(srdyGpioCfg.levelshifter.direction,
			gpioCfg->levelshifter.direction,
			strlen(gpioCfg->levelshifter.direction));
	srdyGpioCfg.levelshifter.active_high_low = gpioCfg->levelshifter.active_high_low;
	debug_printf("srdyGpioCfg.levelshifter.direction = '%s'\n", srdyGpioCfg.levelshifter.direction);

	//open the GPIO DIR file for the level shifter direction signal
	gpioSrdyFd = open(srdyGpioCfg.levelshifter.direction, O_RDWR);
	//  gpioSrdyFd = open(srdyGpioCfg.levelshifter.direction, O_RDWR);
	if(gpioSrdyFd == 0)
	{
		perror(srdyGpioCfg.levelshifter.direction);
		debug_printf("\n%s open failed\n",srdyGpioCfg.levelshifter.direction);
		return(-1);
	}

	//Set the direction of the GPIO to output
	if (ERROR == write(gpioSrdyFd, "out", 3))
	{
		perror(srdyGpioCfg.levelshifter.direction);
		debug_printf("\ncan't write in %s \n",srdyGpioCfg.levelshifter.direction);
		return(-1);
	}
	//close the DIR file
	close(gpioSrdyFd);

	//open the GPIO VALUE file for the level shifter direction signal
	gpioSrdyFd = open(srdyGpioCfg.levelshifter.value, O_RDWR);
	if(gpioSrdyFd == 0)
	{
		perror(srdyGpioCfg.levelshifter.value);
		debug_printf("%s open failed\n",srdyGpioCfg.levelshifter.value);
		return(-1);
	}

	//Set the value of the GPIO to 0 (level shifter direction from CC2531 to Host)

	if (ERROR == write(gpioSrdyFd, "0", 1))
	{
		perror(srdyGpioCfg.levelshifter.direction);
		debug_printf("\ncan't write in %s \n",srdyGpioCfg.levelshifter.value);
		return(-1);
	}
	//close the DIR file
	close(gpioSrdyFd);

	//TODO: Lock the shift register GPIO.

	//open the SRDY GPIO DIR file
	gpioSrdyFd = open(srdyGpioCfg.gpio.direction, O_RDWR);
	if(gpioSrdyFd == 0)
	{
		perror(srdyGpioCfg.gpio.direction);
		debug_printf("%s open failed\n",srdyGpioCfg.gpio.direction);
		exit(-1);
	}

	//Set SRDY GPIO as input
	if(ERROR == write(gpioSrdyFd, "in", 2))
	{
		perror(srdyGpioCfg.levelshifter.direction);
		debug_printf("\ncan't write in %s \n",srdyGpioCfg.gpio.direction);
		return(-1);
	}
	//close SRDY DIR file
	close(gpioSrdyFd);

	//open the SRDY GPIO VALUE file so it can be written to using the file handle later
	gpioSrdyFd = open(srdyGpioCfg.gpio.value, O_RDWR);
	if(gpioSrdyFd == 0)
	{
		perror(srdyGpioCfg.gpio.value);
		debug_printf("%s open failed\n",srdyGpioCfg.gpio.value);
		return(-1);
	}

	return(0);
}
/**************************************************************************************************
 * @fn      HalGpioMrdyInit
 *
 *
 * @brief   Initialise MRDY GPIO.
 *
 * @param   gpioCfg - MRDY pin configuration parameters
 *
 * @return  None
 **************************************************************************************************/
uint8 HalGpioMrdyInit(halGpioCfg_t *gpioCfg)
{
	memcpy(mrdyGpioCfg.gpio.value,
			gpioCfg->gpio.value,
			strlen(gpioCfg->gpio.value));
	debug_printf("resetGpioCfg.gpio.value = '%s'\n", mrdyGpioCfg.gpio.value);
	memcpy(mrdyGpioCfg.gpio.direction,
			gpioCfg->gpio.direction,
			strlen(gpioCfg->gpio.direction));
	debug_printf("mrdyGpioCfg.gpio.direction = '%s'\n", mrdyGpioCfg.gpio.direction);
	mrdyGpioCfg.gpio.active_high_low = gpioCfg->gpio.active_high_low;
	memcpy(mrdyGpioCfg.levelshifter.value,
			gpioCfg->levelshifter.value,
			strlen(gpioCfg->levelshifter.value));
	debug_printf("mrdyGpioCfg.levelshifter.value = '%s'\n", mrdyGpioCfg.levelshifter.value);
	memcpy(mrdyGpioCfg.levelshifter.direction,
			gpioCfg->levelshifter.direction,
			strlen(gpioCfg->levelshifter.direction));
	mrdyGpioCfg.levelshifter.active_high_low = gpioCfg->levelshifter.active_high_low;
	debug_printf("mrdyGpioCfg.levelshifter.direction = '%s'\n", mrdyGpioCfg.levelshifter.direction);

	//open the GPIO DIR file for the level shifter direction signal
	gpioMrdyFd = open(mrdyGpioCfg.levelshifter.direction, O_RDWR);
	gpioMrdyFd = open(mrdyGpioCfg.levelshifter.direction, O_RDWR);
	if(gpioMrdyFd == 0)
	{
		perror(mrdyGpioCfg.levelshifter.direction);
		debug_printf("%s open failed\n",mrdyGpioCfg.levelshifter.direction);
		return(-1);
	}

	//Set the direction of the GPIO to output
	if(ERROR == write(gpioMrdyFd, "out", 3))
	{
		perror(mrdyGpioCfg.levelshifter.direction);
		debug_printf("\ncan't write in %s \n",mrdyGpioCfg.levelshifter.direction);
		return(-1);
	}
	//close the DIR file
	close(gpioMrdyFd);

	//open the GPIO VALUE file for the level shifter direction signal
	gpioMrdyFd = open(mrdyGpioCfg.levelshifter.value, O_RDWR);
	if(gpioMrdyFd == 0)
	{
		perror(mrdyGpioCfg.levelshifter.value);
		debug_printf("%s open failed\n",mrdyGpioCfg.levelshifter.value);
		return(-1);
	}

	//Set the value of the GPIO to 0 (level shifter direction from Host to CC2531)
	if(ERROR == write(gpioMrdyFd, "1", 1))
	{
		perror(mrdyGpioCfg.levelshifter.value);
		debug_printf("\ncan't write in %s \n",mrdyGpioCfg.levelshifter.value);
		return(-1);
	}
	//close the DIR file
	close(gpioMrdyFd);

	//open the MRDY GPIO DIR file
	gpioMrdyFd = open(mrdyGpioCfg.gpio.direction, O_RDWR);
	if(gpioMrdyFd == 0)
	{
		perror(mrdyGpioCfg.gpio.direction);
		debug_printf("%s open failed\n",mrdyGpioCfg.gpio.direction);
		return(-1);
	}

	//Set MRDY GPIO as output
	if(ERROR == write(gpioMrdyFd, "out", 3))
	{
		perror(mrdyGpioCfg.gpio.direction);
		debug_printf("\ncan't write in %s \n",mrdyGpioCfg.gpio.direction);
		return(-1);
	}
	//close MRDY DIR file
	close(gpioMrdyFd);

	//open the MRDY GPIO VALUE file so it can be written to using the file handle later
	gpioMrdyFd = open(mrdyGpioCfg.gpio.value, O_RDWR);
	if(gpioMrdyFd == 0)
	{
		perror(mrdyGpioCfg.gpio.value);
		debug_printf("%s open failed\n",mrdyGpioCfg.gpio.value);
		return(-1);
	}

	//Set MRDY GPIO to 1 as default

	if (ERROR == write(gpioMrdyFd, "1", 3))
	{
		perror(mrdyGpioCfg.gpio.value);
		debug_printf("\ncan't write in %s \n",mrdyGpioCfg.gpio.value);
		return(-1);
	}

	return(0);
}

/**************************************************************************************************
 * @fn      HalGpioResetInit
 *
 *
 * @brief   Initialise RESET GPIO.
 *
 * @param   gpioCfg - Reset pin configuration parameters
 *
 * @return  None
 **************************************************************************************************/
uint8 HalGpioResetInit(halGpioCfg_t *gpioCfg)
{
	memcpy(resetGpioCfg.gpio.value,
			gpioCfg->gpio.value,
			strlen(gpioCfg->gpio.value));
	debug_printf("resetGpioCfg.gpio.value = '%s'\n", resetGpioCfg.gpio.value);
	memcpy(resetGpioCfg.gpio.direction,
			gpioCfg->gpio.direction,
			strlen(gpioCfg->gpio.direction));
	debug_printf("resetGpioCfg.gpio.direction = '%s'\n", resetGpioCfg.gpio.direction);
	resetGpioCfg.gpio.active_high_low = gpioCfg->gpio.active_high_low;
	memcpy(resetGpioCfg.levelshifter.value,
			gpioCfg->levelshifter.value,
			strlen(gpioCfg->levelshifter.value));
	debug_printf("resetGpioCfg.levelshifter.value = '%s'\n", resetGpioCfg.levelshifter.value);
	memcpy(resetGpioCfg.levelshifter.direction,
			gpioCfg->levelshifter.direction,
			strlen(gpioCfg->levelshifter.direction));
	resetGpioCfg.levelshifter.active_high_low = gpioCfg->levelshifter.active_high_low;
	debug_printf("resetGpioCfg.levelshifter.direction = '%s'\n", resetGpioCfg.levelshifter.direction);


	//open the GPIO DIR file for the level shifter direction signal
	gpioResetFd = open(resetGpioCfg.levelshifter.direction, O_RDWR);
	if(gpioResetFd == 0)
	{
		perror(resetGpioCfg.levelshifter.direction);
		debug_printf("%s open failed\n",resetGpioCfg.levelshifter.direction);
		return(-1);
	}

	//Set the direction of the GPIO to output
	if (ERROR == write(gpioResetFd, "out", 3))
	{
		perror(resetGpioCfg.levelshifter.direction);
		debug_printf("\ncan't write in %s \n",resetGpioCfg.levelshifter.direction);
		return(-1);
	}
	//close the DIR file
	close(gpioResetFd);

	//open the GPIO VALUE file for the level shifter direction signal
	gpioResetFd = open(resetGpioCfg.levelshifter.value, O_RDWR);
	if(gpioResetFd == 0)
	{
		perror(resetGpioCfg.levelshifter.value);
		debug_printf("%s open failed\n",resetGpioCfg.levelshifter.value);
		return(-1);
	}

	//Set the value of the GPIO to 0 (level shifter direction from Host to CC2531)
	if(ERROR == write(gpioResetFd, "1", 1))
	{
		perror(resetGpioCfg.levelshifter.value);
		debug_printf("\ncan't write in %s \n",resetGpioCfg.levelshifter.value);
		return(-1);
	}
	//close the DIR file
	close(gpioResetFd);

	//open the MRDY GPIO DIR file
	gpioResetFd = open(resetGpioCfg.gpio.direction, O_RDWR);
	if(gpioResetFd == 0)
	{
		perror(resetGpioCfg.gpio.direction);
		debug_printf("%s open failed\n",resetGpioCfg.gpio.direction);
		return(-1);
	}

	//Set MRDY GPIO as output
	if(ERROR == write(gpioResetFd, "out", 3))
	{
		perror(resetGpioCfg.gpio.direction);
		debug_printf("\ncan't write in %s \n",resetGpioCfg.gpio.direction);
		return(-1);
	}
	//close MRDY DIR file
	close(gpioResetFd);

	//open the MRDY GPIO VALUE file so it can be writen to using the file handle later
	gpioResetFd = open(resetGpioCfg.gpio.value, O_RDWR);
	if(gpioResetFd == 0)
	{
		perror(resetGpioCfg.gpio.value);
		debug_printf("%s open failed\n",resetGpioCfg.gpio.value);
		return(-1);
	}

	//Set MRDY GPIO to 1 as default
	if(ERROR == write(gpioResetFd, "1", 3))
	{
		perror(resetGpioCfg.gpio.value);
		debug_printf("\ncan't write in %s \n",resetGpioCfg.gpio.value);
		return(-1);
	}

	return(0);
}

/**************************************************************************************************
 * @fn      HalGpioMrdySet
 *
 *
 * @brief   Set MRDY.
 *
 * @param
 *
 * @return  None
 **************************************************************************************************/
void HalGpioMrdySet(uint8 state)
{
	if(state == 0)
	{
		debug_printf("MRDY set to low\n");
		if (ERROR == write(gpioMrdyFd, "0", 1))
		{
			perror(mrdyGpioCfg.gpio.value);
			debug_printf("\ncan't write in %s , is something already accessing it? abort everything for debug purpose...\n",mrdyGpioCfg.gpio.value);
			exit(-1);
		}
	}
	else
	{
		debug_printf("MRDY set to High\n");
    	if(ERROR == write(gpioMrdyFd, "1", 1))
		{
			perror(mrdyGpioCfg.gpio.value);
			debug_printf("\ncan't write in %s , is something already accessing it? abort everything for debug purpose...\n",mrdyGpioCfg.gpio.value);
			exit(-1);
		}
	}

}
/**************************************************************************************************
 * @fn      HalGpioReset
 *
 *
 * @brief   Set MRDY.
 *
 * @param   None
 *
 * @return  None
 **************************************************************************************************/
void HalGpioReset(void)
{
	debug_printf("Reset High\n");
	if(ERROR == write(gpioResetFd, "1", 1))
	{
		perror(resetGpioCfg.gpio.value);
		debug_printf("\ncan't write in %s , is something already accessing it? abort everything for debug purpose...\n",mrdyGpioCfg.gpio.value);
		exit(-1);
	}
	debug_printf("Reset low\n");
	if(ERROR == write(gpioResetFd, "0", 1))
	{
		perror(resetGpioCfg.gpio.value);
		debug_printf("\ncan't write in %s , is something already accessing it? abort everything for debug purpose...\n",mrdyGpioCfg.gpio.value);
		exit(-1);
	}
	//Add A Delay here:
	// Reset Should last at least 1us from datasheet, set it to 500us.
	usleep(500);

	debug_printf("Reset High\n");
	if(ERROR == write(gpioResetFd, "1", 1))
	{
		perror(resetGpioCfg.gpio.value);
		debug_printf("\ncan't write in %s , is something already accessing it? abort everything for debug purpose...\n",mrdyGpioCfg.gpio.value);
		exit(-1);
	}
}
/**************************************************************************************************
 * @fn      HalGpioSrdyCheck
 *
 *
 * @brief   Check SRDY Clear.
 *
 * @param   state	- Active  or  Inactive
 *
 * @return  None
 **************************************************************************************************/
uint8 HalGpioSrdyCheck(uint8 state)
{
	char srdy=2;
	lseek(gpioSrdyFd,0,SEEK_SET);
	if(ERROR == read(gpioSrdyFd,&srdy, 1))
	{
		perror(srdyGpioCfg.gpio.value);
		debug_printf("\ncan't read in %s , is something already accessing it? abort everything for debug purpose...\n",srdyGpioCfg.gpio.value);
		exit(-1);
	}

	debug_printf("===>check SRDY: %c  (%c) \n", srdy, srdy);

	return (state == ((srdy == '1') ? 1 : 0));
}

/**************************************************************************************************
 * @fn          HalGpioWaitSrdyClr
 *
 * @brief       Check that SRDY is low, if not, wait until it gets low. poll every ms.
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
void HalGpioWaitSrdyClr(void)
{
	char srdy= '1';

	debug_printf("Wait SRDY Low, \n");

	struct pollfd ufds[1];
	int pollRet, waitMs = 100;
	ufds[0].fd = gpioSrdyFd;
	ufds[0].events = POLLIN | POLLPRI;

	while(srdy == '1')
	{
		debug_printf("SRDY: 0x%.2X , %c(0x%.2X) TRUE \n", atoi(&srdy), srdy, srdy);
		pollRet = poll((struct pollfd*)&ufds, 1, waitMs);
		if (pollRet == -1)
		{
			// Error occured in poll()
			perror("poll");
		}
		else if (pollRet == 0)
		{
			// Timeout
			printf("[WARNING] Waiting for SRDY to go low timed out.\n");
			break;
		}
		else
		{
			if ( (ufds[0].revents & POLLIN) || (ufds[0].revents & POLLPRI) )
			{
				lseek(gpioSrdyFd,0,SEEK_SET);
				if(ERROR == read(gpioSrdyFd,&srdy, 1))
				{
					perror(srdyGpioCfg.gpio.value);
					debug_printf("\ncan't read in %s , is something already accessing it? abort everything for debug purpose...\n",srdyGpioCfg.gpio.value);
					exit(-1);
				}
			}
		}
	}
	debug_printf("SRDY: 0x%.2X , %c(0x%.2X) FALSE \n", atoi(&srdy), srdy, srdy);


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

  printf("[--> %.5ld.%.6ld (+%ld.%6ld)] SRDY Low \n",
		  curTime.tv_sec - startTime.tv_sec,
		  curTime.tv_usec,
		  curTime.tv_sec - prevTimeI2C.tv_sec - t,
		  diffPrev);
#endif //__STRESS_TEST__

  debug_printf("==>SRDY change to : %c  (%c) \n", srdy, srdy);
}

/**************************************************************************************************
 * @fn          HalGpioWaitSrdySet
 *
 * @brief       Check that SRDY is High, if not, wait until it gets high, or times out.
 * 				0xFFFF means never time out.
 *
 * input parameters
 *
 * @param       waitMs	- Maximum wait time in ms. Necessary to detect unexpected handshake.
 * 						  0xFFFF means never time out.
 *
 * output parameters
 *
 * None.
 *
 * @return      srdy	- If positive it indicates success
 **************************************************************************************************
 */
uint8 HalGpioWaitSrdySet(uint16 waitMs)
{
	char srdy= '0';

	debug_printf("Wait SRDY High, \n");

	struct pollfd ufds[1];
	int pollRet;
	ufds[0].fd = gpioSrdyFd;
	ufds[0].events = POLLIN | POLLPRI;

#ifdef __DEBUG_TIME__
	gettimeofday(&curTime, NULL);
	long int diffPrev;
	int t = 0;
	if (curTime.tv_usec >= prevTime.tv_usec)
	{
		diffPrev = curTime.tv_usec - prevTime.tv_usec;
	}
	else
	{
		diffPrev = (curTime.tv_usec + 1000000) - prevTime.tv_usec;
		t = 1;
	}

	prevTime = curTime;

	//	debug_
	printf("[%.5ld.%.6ld (+%ld.%6ld)] SRDY: %c  (%c), wait %d ms\n",
			curTime.tv_sec - startTime.tv_sec,
			curTime.tv_usec,
			curTime.tv_sec - prevTime.tv_sec - t,
			diffPrev,
			srdy,
			srdy,
			waitMs);
#endif //(defined __DEBUG_TIME__)

	while( (srdy == '0') )
	{
		pollRet = poll((struct pollfd*)&ufds, 1, waitMs);
		if (pollRet == -1)
		{
			// Error occured in poll()
			perror("poll");
		}
		else if (pollRet == 0)
		{
			// Timeout
			printf("[WARNING] Waiting for SRDY to go high timed out.\n");
			break;
		}
		else
		{
			if ( (ufds[0].revents & POLLIN) || (ufds[0].revents & POLLPRI) )
			{
				lseek(gpioSrdyFd,0,SEEK_SET);
				if(ERROR == read(gpioSrdyFd,&srdy, 1))
				{
					perror(srdyGpioCfg.gpio.value);
					debug_printf("\ncan't read in %s , is something already accessing it? abort everything for debug purpose...\n",srdyGpioCfg.gpio.value);
					exit(-1);
				}
			}
		}
	}

#ifdef __DEBUG_TIME__
	gettimeofday(&curTime, NULL);
	t = 0;
	if (curTime.tv_usec >= prevTime.tv_usec)
	{
		diffPrev = curTime.tv_usec - prevTime.tv_usec;
	}
	else
	{
		diffPrev = (curTime.tv_usec + 1000000) - prevTime.tv_usec;
		t = 1;
	}

	prevTime = curTime;

//	debug_
		printf("[%.5ld.%.6ld (+%ld.%6ld)] SRDY: %c  (%c), wait %d ms\n",
				curTime.tv_sec - startTime.tv_sec,
				curTime.tv_usec,
				curTime.tv_sec - prevTime.tv_sec - t,
				diffPrev,
				srdy,
				srdy,
				waitMs);
#endif //__DEBUG_TIME__

//	debug_printf("==>SRDY change to : %c  (%c) \n", srdy, srdy);

	return (srdy == '1') ? 1 : 0;
}

/**************************************************************************************************
 **************************************************************************************************/
