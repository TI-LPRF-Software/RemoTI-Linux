/**************************************************************************************************
  Filename:       hal_gpio.c
  Revised:        $Date: 2012-03-15 13:45:31 -0700 (Thu, 15 Mar 2012) $
  Revision:       $Revision: 237 $

  Description:    This file contains the interface to the HAL GPIO.


  Copyright (C) {2016} Texas Instruments Incorporated - http://www.ti.com/


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
#include "hal_types.h"
#include "hal_gpio.h"
#include "time_printf.h"

#include "npi_lnx_error.h"

#ifdef __STRESS_TEST__
#include <sys/time.h>
#elif defined __DEBUG_TIME__
#include <sys/time.h>
#endif //__DEBUG_TIME__

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
static int gpioSrdyFd;
static int gpioMrdyFd;
static int gpioResetFd;

static halGpioCfg_t srdyGpioCfg;
static halGpioCfg_t mrdyGpioCfg;
static halGpioCfg_t resetGpioCfg;

/**************************************************************************************************
 *                                          FUNCTIONS - API
 **************************************************************************************************/


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
 * @return  STATUS
 **************************************************************************************************/
int HalGpioSrdyInit(halGpioCfg_t *gpioCfg)
{
	char tmpStr[512];
	memcpy(srdyGpioCfg.gpio.value,
			gpioCfg->gpio.value,
			strlen(gpioCfg->gpio.value));
	if (__BIG_DEBUG_ACTIVE == TRUE)
	{
		snprintf(tmpStr, sizeof(tmpStr), "[%s] srdyGpioCfg.gpio.value = '%s'\n", __FUNCTION__, srdyGpioCfg.gpio.value);
		time_printf(tmpStr);
	}
	memcpy(srdyGpioCfg.gpio.direction,
			gpioCfg->gpio.direction,
			strlen(gpioCfg->gpio.direction));
	if (__BIG_DEBUG_ACTIVE == TRUE)
	{
		snprintf(tmpStr, sizeof(tmpStr), "[%s] srdyGpioCfg.gpio.direction = '%s'\n", __FUNCTION__, srdyGpioCfg.gpio.direction);
		time_printf(tmpStr);
	}

	srdyGpioCfg.gpio.active_high_low = gpioCfg->gpio.active_high_low;

	memcpy(srdyGpioCfg.gpio.edge,
			gpioCfg->gpio.edge,
			strlen(gpioCfg->gpio.edge));
	if (__BIG_DEBUG_ACTIVE == TRUE)
	{
		snprintf(tmpStr, sizeof(tmpStr), "[%s] srdyGpioCfg.gpio.edge = '%s'\n", __FUNCTION__, srdyGpioCfg.gpio.edge);
		time_printf(tmpStr);
	}

	if ( ( gpioCfg->levelshifter.value) &&
			( gpioCfg->levelshifter.active_high_low) &&
			( gpioCfg->levelshifter.direction))
	{
		memcpy(srdyGpioCfg.levelshifter.value,
				gpioCfg->levelshifter.value,
				strlen(gpioCfg->levelshifter.value));
		if (__BIG_DEBUG_ACTIVE == TRUE)
		{
			snprintf(tmpStr, sizeof(tmpStr), "[%s] srdyGpioCfg.levelshifter.value = '%s'\n", __FUNCTION__, srdyGpioCfg.levelshifter.value);
			time_printf(tmpStr);
		}
		memcpy(srdyGpioCfg.levelshifter.direction,
				gpioCfg->levelshifter.direction,
				strlen(gpioCfg->levelshifter.direction));
		srdyGpioCfg.levelshifter.active_high_low = gpioCfg->levelshifter.active_high_low;
		if (__BIG_DEBUG_ACTIVE == TRUE)
		{
			snprintf(tmpStr, sizeof(tmpStr), "[%s] srdyGpioCfg.levelshifter.direction = '%s'\n", __FUNCTION__, srdyGpioCfg.levelshifter.direction);
			time_printf(tmpStr);
		}

		//open the GPIO DIR file for the level shifter direction signal
		gpioSrdyFd = open(srdyGpioCfg.levelshifter.direction, O_RDWR);
		if(gpioSrdyFd == 0)
		{
			perror(srdyGpioCfg.levelshifter.direction);
			if (__BIG_DEBUG_ACTIVE == TRUE)
			{
				snprintf(tmpStr, sizeof(tmpStr), "[%s] %s open failed\n", __FUNCTION__, srdyGpioCfg.levelshifter.direction);
				time_printf(tmpStr);
			}
			npi_ipc_errno = NPI_LNX_ERROR_HAL_GPIO_SRDY_LVLSHFT_DIR_OPEN;
			return NPI_LNX_FAILURE;
		}

		//Set the direction of the GPIO to output
		if (ERROR == write(gpioSrdyFd, "out", 3))
		{
			perror(srdyGpioCfg.levelshifter.direction);
			if (__BIG_DEBUG_ACTIVE == TRUE)
			{
				snprintf(tmpStr, sizeof(tmpStr), "[%s] can't write in %s \n", __FUNCTION__, srdyGpioCfg.levelshifter.direction);
				time_printf(tmpStr);
			}
			npi_ipc_errno = NPI_LNX_ERROR_HAL_GPIO_SRDY_LVLSHFT_DIR_WRITE;
			return NPI_LNX_FAILURE;
		}
		//close the DIR file
		close(gpioSrdyFd);

		//open the GPIO VALUE file for the level shifter direction signal
		gpioSrdyFd = open(srdyGpioCfg.levelshifter.value, O_RDWR);
		if(gpioSrdyFd == 0)
		{
			perror(srdyGpioCfg.levelshifter.value);
			if (__BIG_DEBUG_ACTIVE == TRUE)
			{
				snprintf(tmpStr, sizeof(tmpStr), "[%s] %s open failed\n", __FUNCTION__, srdyGpioCfg.levelshifter.value);
				time_printf(tmpStr);
			}
			npi_ipc_errno = NPI_LNX_ERROR_HAL_GPIO_SRDY_LVLSHFT_VAL_OPEN;
			return NPI_LNX_FAILURE;
		}

		//Set the value of the GPIO to 0 (level shifter direction from CC2531 to Host)

		if (ERROR == write(gpioSrdyFd, "0", 1))
		{
			perror(srdyGpioCfg.levelshifter.value);
			if (__BIG_DEBUG_ACTIVE == TRUE)
			{
				snprintf(tmpStr, sizeof(tmpStr), "[%s] can't write in %s\n", __FUNCTION__, srdyGpioCfg.levelshifter.value);
				time_printf(tmpStr);
			}
			npi_ipc_errno = NPI_LNX_ERROR_HAL_GPIO_SRDY_LVLSHFT_VAL_WRITE;
			return NPI_LNX_FAILURE;
		}
		//close the DIR file
		close(gpioSrdyFd);
	}
	else
	{
		if (__BIG_DEBUG_ACTIVE == TRUE)
		{
			snprintf(tmpStr, sizeof(tmpStr), "[%s] Wrong Configuration File, one of the  following Key value are missing for SRDY.Level Shifter definition: '\n", __FUNCTION__);
			int strIndex = strlen(tmpStr);
			snprintf(tmpStr + strIndex, sizeof(tmpStr) - strIndex, "value: \t\t%s\n", srdyGpioCfg.gpio.value);
			strIndex = strlen(tmpStr);
			snprintf(tmpStr + strIndex, sizeof(tmpStr) - strIndex, "direction: \t%s\n", srdyGpioCfg.gpio.direction);
			strIndex = strlen(tmpStr);
			snprintf(tmpStr + strIndex, sizeof(tmpStr) - strIndex, "active_high_low: %d\n", srdyGpioCfg.gpio.active_high_low);
			strIndex = strlen(tmpStr);
			snprintf(tmpStr + strIndex, sizeof(tmpStr) - strIndex, "Level Shifter is optional, please check if you need it or not before continuing...\n");
		}
	}
	//TODO: Lock the shift register GPIO.

	//open the SRDY GPIO DIR file
	gpioSrdyFd = open(srdyGpioCfg.gpio.direction, O_RDWR);
	if(gpioSrdyFd == 0)
	{
		perror(srdyGpioCfg.gpio.direction);
		if (__BIG_DEBUG_ACTIVE == TRUE)
		{
			snprintf(tmpStr, sizeof(tmpStr), "[%s] %s open failed\n", __FUNCTION__, srdyGpioCfg.gpio.direction);
			time_printf(tmpStr);
		}
		npi_ipc_errno = NPI_LNX_ERROR_HAL_GPIO_SRDY_GPIO_DIR_OPEN;
		return NPI_LNX_FAILURE;
	}

	//Set SRDY GPIO as input
	if(ERROR == write(gpioSrdyFd, "in", 2))
	{
		perror(srdyGpioCfg.levelshifter.direction);
		if (__BIG_DEBUG_ACTIVE == TRUE)
		{
			snprintf(tmpStr, sizeof(tmpStr), "[%s] can't write in %s\n", __FUNCTION__, srdyGpioCfg.gpio.direction);
			time_printf(tmpStr);
		}
		npi_ipc_errno = NPI_LNX_ERROR_HAL_GPIO_SRDY_GPIO_DIR_WRITE;
		return NPI_LNX_FAILURE;
	}
	//close SRDY DIR file
	close(gpioSrdyFd);

	//open the SRDY GPIO Edge file
	gpioSrdyFd = open(srdyGpioCfg.gpio.edge, O_RDWR);
	if(gpioSrdyFd == 0)
	{
		perror(srdyGpioCfg.gpio.edge);
		if (__BIG_DEBUG_ACTIVE == TRUE)
		{
			snprintf(tmpStr, sizeof(tmpStr), "[%s] %s open failed\n", __FUNCTION__, srdyGpioCfg.gpio.edge);
			time_printf(tmpStr);
		}
		npi_ipc_errno = NPI_LNX_ERROR_HAL_GPIO_SRDY_GPIO_EDGE_OPEN;
		return NPI_LNX_FAILURE;
	}

	//Set SRDY GPIO edge detection for both rising and falling
	if(ERROR == write(gpioSrdyFd, "both", 4))
	{
		perror(srdyGpioCfg.levelshifter.edge);
		if (__BIG_DEBUG_ACTIVE == TRUE)
		{
			snprintf(tmpStr, sizeof(tmpStr), "[%s] can't write in %s\n", __FUNCTION__, srdyGpioCfg.gpio.edge);
			time_printf(tmpStr);
		}
		npi_ipc_errno = NPI_LNX_ERROR_HAL_GPIO_SRDY_GPIO_EDGE_WRITE;
		return NPI_LNX_FAILURE;
	}
	//close SRDY edge file
	close(gpioSrdyFd);

	//open the SRDY GPIO VALUE file so it can be written to using the file handle later
	gpioSrdyFd = open(srdyGpioCfg.gpio.value, O_RDWR| O_NONBLOCK);
	if(gpioSrdyFd == 0)
	{
		perror(srdyGpioCfg.gpio.value);
		if (__BIG_DEBUG_ACTIVE == TRUE)
		{
			snprintf(tmpStr, sizeof(tmpStr), "[%s] %s open failed\n", __FUNCTION__, srdyGpioCfg.gpio.value);
			time_printf(tmpStr);
		}
		npi_ipc_errno = NPI_LNX_ERROR_HAL_GPIO_SRDY_GPIO_VAL_OPEN;
		return NPI_LNX_FAILURE;
	}

	return(gpioSrdyFd);
}
/**************************************************************************************************
 * @fn      HalGpioMrdyInit
 *
 *
 * @brief   Initialise MRDY GPIO.
 *
 * @param   gpioCfg - MRDY pin configuration parameters
 *
 * @return  STATUS
 **************************************************************************************************/
int HalGpioMrdyInit(halGpioCfg_t *gpioCfg)
{
	char tmpStr[512];
	memcpy(mrdyGpioCfg.gpio.value,
			gpioCfg->gpio.value,
			strlen(gpioCfg->gpio.value));
	if (__BIG_DEBUG_ACTIVE == TRUE)
	{
		snprintf(tmpStr, sizeof(tmpStr), "[%s] mrdyGpioCfg.gpio.value = '%s'\n", __FUNCTION__, mrdyGpioCfg.gpio.value);
		time_printf(tmpStr);
	}
	memcpy(mrdyGpioCfg.gpio.direction,
			gpioCfg->gpio.direction,
			strlen(gpioCfg->gpio.direction));
	if (__BIG_DEBUG_ACTIVE == TRUE)
	{
		snprintf(tmpStr, sizeof(tmpStr), "[%s] mrdyGpioCfg.gpio.direction = '%s'\n", __FUNCTION__, mrdyGpioCfg.gpio.direction);
		time_printf(tmpStr);
	}
	mrdyGpioCfg.gpio.active_high_low = gpioCfg->gpio.active_high_low;

	if ( ( gpioCfg->levelshifter.value) &&
			( gpioCfg->levelshifter.active_high_low) &&
			( gpioCfg->levelshifter.direction))
	{
		memcpy(mrdyGpioCfg.levelshifter.value,
				gpioCfg->levelshifter.value,
				strlen(gpioCfg->levelshifter.value));
		if (__BIG_DEBUG_ACTIVE == TRUE)
		{
			snprintf(tmpStr, sizeof(tmpStr), "[%s] mrdyGpioCfg.levelshifter.value = '%s'\n", __FUNCTION__, mrdyGpioCfg.levelshifter.value);
			time_printf(tmpStr);
		}
		memcpy(mrdyGpioCfg.levelshifter.direction,
				gpioCfg->levelshifter.direction,
				strlen(gpioCfg->levelshifter.direction));
		mrdyGpioCfg.levelshifter.active_high_low = gpioCfg->levelshifter.active_high_low;
		if (__BIG_DEBUG_ACTIVE == TRUE)
		{
			snprintf(tmpStr, sizeof(tmpStr), "[%s] mrdyGpioCfg.levelshifter.direction = '%s'\n", __FUNCTION__, mrdyGpioCfg.levelshifter.direction);
			time_printf(tmpStr);
		}

		//open the GPIO DIR file for the level shifter direction signal
		gpioMrdyFd = open(mrdyGpioCfg.levelshifter.direction, O_RDWR);
		if(gpioMrdyFd == 0)
		{
			perror(mrdyGpioCfg.levelshifter.direction);
			if (__BIG_DEBUG_ACTIVE == TRUE)
			{
				snprintf(tmpStr, sizeof(tmpStr), "[%s] %s open failed\n", __FUNCTION__, mrdyGpioCfg.levelshifter.direction);
				time_printf(tmpStr);
			}
			npi_ipc_errno = NPI_LNX_ERROR_HAL_GPIO_MRDY_LVLSHFT_DIR_OPEN;
			return NPI_LNX_FAILURE;
		}

		//Set the direction of the GPIO to output
		if(ERROR == write(gpioMrdyFd, "out", 3))
		{
			perror(mrdyGpioCfg.levelshifter.direction);
			if (__BIG_DEBUG_ACTIVE == TRUE)
			{
				snprintf(tmpStr, sizeof(tmpStr), "[%s] can't write in %s \n", __FUNCTION__, mrdyGpioCfg.levelshifter.direction);
				time_printf(tmpStr);
			}
			npi_ipc_errno = NPI_LNX_ERROR_HAL_GPIO_MRDY_LVLSHFT_DIR_WRITE;
			return NPI_LNX_FAILURE;
		}
		//close the DIR file
		close(gpioMrdyFd);

		//open the GPIO VALUE file for the level shifter direction signal
		gpioMrdyFd = open(mrdyGpioCfg.levelshifter.value, O_RDWR);
		if(gpioMrdyFd == 0)
		{
			perror(mrdyGpioCfg.levelshifter.value);
			if (__BIG_DEBUG_ACTIVE == TRUE)
			{
				snprintf(tmpStr, sizeof(tmpStr), "[%s] %s open failed\n", __FUNCTION__, mrdyGpioCfg.levelshifter.value);
				time_printf(tmpStr);
			}
			npi_ipc_errno = NPI_LNX_ERROR_HAL_GPIO_MRDY_LVLSHFT_VAL_OPEN;
			return NPI_LNX_FAILURE;
		}

		//Set the value of the GPIO to 0 (level shifter direction from Host to CC2531)
		if(ERROR == write(gpioMrdyFd, "1", 1))
		{
			perror(mrdyGpioCfg.levelshifter.value);
			if (__BIG_DEBUG_ACTIVE == TRUE)
			{
				snprintf(tmpStr, sizeof(tmpStr), "[%s] can't write in %s\n", __FUNCTION__, mrdyGpioCfg.levelshifter.value);
				time_printf(tmpStr);
			}
			npi_ipc_errno = NPI_LNX_ERROR_HAL_GPIO_MRDY_LVLSHFT_VAL_WRITE;
			return NPI_LNX_FAILURE;
		}
		//close the DIR file
		close(gpioMrdyFd);
	}
	else
	{
		if (__BIG_DEBUG_ACTIVE == TRUE)
		{
			snprintf(tmpStr, sizeof(tmpStr), "[%s] Wrong Configuration File, one of the  following Key value are missing for MRDY.Level Shifter definition: '\n", __FUNCTION__);
			int strIndex = strlen(tmpStr);
			snprintf(tmpStr + strIndex, sizeof(tmpStr) - strIndex, "value: \t\t%s\n", mrdyGpioCfg.gpio.value);
			strIndex = strlen(tmpStr);
			snprintf(tmpStr + strIndex, sizeof(tmpStr) - strIndex, "direction: \t%s\n", mrdyGpioCfg.gpio.direction);
			strIndex = strlen(tmpStr);
			snprintf(tmpStr + strIndex, sizeof(tmpStr) - strIndex, "active_high_low: %d\n", mrdyGpioCfg.gpio.active_high_low);
			strIndex = strlen(tmpStr);
			snprintf(tmpStr + strIndex, sizeof(tmpStr) - strIndex, "Level Shifter is optional, please check if you need it or not before continuing...\n");
		}
	}

	//open the MRDY GPIO DIR file
	gpioMrdyFd = open(mrdyGpioCfg.gpio.direction, O_RDWR);
	if(gpioMrdyFd == 0)
	{
		perror(mrdyGpioCfg.gpio.direction);
		if (__BIG_DEBUG_ACTIVE == TRUE)
		{
			snprintf(tmpStr, sizeof(tmpStr), "[%s] %s open failed\n", __FUNCTION__, mrdyGpioCfg.gpio.direction);
			time_printf(tmpStr);
		}
		npi_ipc_errno = NPI_LNX_ERROR_HAL_GPIO_MRDY_GPIO_DIR_OPEN;
		return NPI_LNX_FAILURE;
	}

	//Set MRDY GPIO as output
	if(ERROR == write(gpioMrdyFd, "out", 3))
	{
		perror(mrdyGpioCfg.gpio.direction);
		if (__BIG_DEBUG_ACTIVE == TRUE)
		{
			snprintf(tmpStr, sizeof(tmpStr), "[%s] can't write in %s\n", __FUNCTION__, mrdyGpioCfg.gpio.direction);
			time_printf(tmpStr);
		}
		npi_ipc_errno = NPI_LNX_ERROR_HAL_GPIO_MRDY_GPIO_DIR_WRITE;
		return NPI_LNX_FAILURE;
	}
	//close MRDY DIR file
	close(gpioMrdyFd);

	//open the MRDY GPIO VALUE file so it can be written to using the file handle later
	gpioMrdyFd = open(mrdyGpioCfg.gpio.value, O_RDWR);
	if(gpioMrdyFd == 0)
	{
		perror(mrdyGpioCfg.gpio.value);
		if (__BIG_DEBUG_ACTIVE == TRUE)
		{
			snprintf(tmpStr, sizeof(tmpStr), "[%s] %s open failed\n", __FUNCTION__, mrdyGpioCfg.gpio.value);
			time_printf(tmpStr);
		}
		npi_ipc_errno = NPI_LNX_ERROR_HAL_GPIO_MRDY_GPIO_VAL_OPEN;
		return NPI_LNX_FAILURE;
	}

	//Set MRDY GPIO to 1 as default

	if (ERROR == write(gpioMrdyFd, "1", 3))
	{
		perror(mrdyGpioCfg.gpio.value);
		if (__BIG_DEBUG_ACTIVE == TRUE)
		{
			snprintf(tmpStr, sizeof(tmpStr), "[%s] can't write in %s\n", __FUNCTION__, mrdyGpioCfg.gpio.value);
			time_printf(tmpStr);
		}
		npi_ipc_errno = NPI_LNX_ERROR_HAL_GPIO_MRDY_GPIO_VAL_WRITE;
		return NPI_LNX_FAILURE;
	}

	return NPI_LNX_SUCCESS;
}

/**************************************************************************************************
 * @fn      HalGpioResetInit
 *
 *
 * @brief   Initialise RESET GPIO.
 *
 * @param   gpioCfg - Reset pin configuration parameters
 *
 * @return  STATUS
 **************************************************************************************************/
int HalGpioResetInit(halGpioCfg_t *gpioCfg)
{
	char tmpStr[512];
	memcpy(resetGpioCfg.gpio.value,
			gpioCfg->gpio.value,
			strlen(gpioCfg->gpio.value));
	if (__BIG_DEBUG_ACTIVE == TRUE)
	{
		snprintf(tmpStr, sizeof(tmpStr), "[%s] resetGpioCfg.gpio.value = '%s'\n", __FUNCTION__, resetGpioCfg.gpio.value);
		time_printf(tmpStr);
	}
	memcpy(resetGpioCfg.gpio.direction,
			gpioCfg->gpio.direction,
			strlen(gpioCfg->gpio.direction));
	if (__BIG_DEBUG_ACTIVE == TRUE)
	{
		snprintf(tmpStr, sizeof(tmpStr), "[%s] resetGpioCfg.gpio.direction = '%s'\n", __FUNCTION__, resetGpioCfg.gpio.direction);
		time_printf(tmpStr);
	}
	resetGpioCfg.gpio.active_high_low = gpioCfg->gpio.active_high_low;

	if ( ( gpioCfg->levelshifter.value) &&
			( gpioCfg->levelshifter.active_high_low) &&
			( gpioCfg->levelshifter.direction))
	{
		memcpy(resetGpioCfg.levelshifter.value,
				gpioCfg->levelshifter.value,
				strlen(gpioCfg->levelshifter.value));
		if (__BIG_DEBUG_ACTIVE == TRUE)
		{
			snprintf(tmpStr, sizeof(tmpStr), "[%s] resetGpioCfg.levelshifter.value = '%s'\n", __FUNCTION__, resetGpioCfg.levelshifter.value);
			time_printf(tmpStr);
		}
		memcpy(resetGpioCfg.levelshifter.direction,
				gpioCfg->levelshifter.direction,
				strlen(gpioCfg->levelshifter.direction));
		resetGpioCfg.levelshifter.active_high_low = gpioCfg->levelshifter.active_high_low;
		if (__BIG_DEBUG_ACTIVE == TRUE)
		{
			snprintf(tmpStr, sizeof(tmpStr), "[%s] resetGpioCfg.levelshifter.direction = '%s'\n", __FUNCTION__, resetGpioCfg.levelshifter.direction);
			time_printf(tmpStr);
		}

		//open the GPIO DIR file for the level shifter direction signal
		gpioResetFd = open(resetGpioCfg.levelshifter.direction, O_RDWR);
		if(gpioResetFd == 0)
		{
			perror(resetGpioCfg.levelshifter.direction);
			if (__BIG_DEBUG_ACTIVE == TRUE)
			{
				snprintf(tmpStr, sizeof(tmpStr), "[%s] %s open failed\n", __FUNCTION__, resetGpioCfg.levelshifter.direction);
				time_printf(tmpStr);
			}
			npi_ipc_errno = NPI_LNX_ERROR_HAL_GPIO_RESET_LVLSHFT_DIR_OPEN;
			return NPI_LNX_FAILURE;
		}

		//Set the direction of the GPIO to output
		if (ERROR == write(gpioResetFd, "out", 3))
		{
			perror(resetGpioCfg.levelshifter.direction);
			if (__BIG_DEBUG_ACTIVE == TRUE)
			{
				snprintf(tmpStr, sizeof(tmpStr), "[%s] can't write in %s \n", __FUNCTION__, resetGpioCfg.levelshifter.direction);
				time_printf(tmpStr);
			}
			npi_ipc_errno = NPI_LNX_ERROR_HAL_GPIO_RESET_LVLSHFT_DIR_WRITE;
			return NPI_LNX_FAILURE;
		}
		//close the DIR file
		close(gpioResetFd);

		//open the GPIO VALUE file for the level shifter direction signal
		gpioResetFd = open(resetGpioCfg.levelshifter.value, O_RDWR);
		if(gpioResetFd == 0)
		{
			perror(resetGpioCfg.levelshifter.value);
			if (__BIG_DEBUG_ACTIVE == TRUE)
			{
				snprintf(tmpStr, sizeof(tmpStr), "[%s] %s open failed\n", __FUNCTION__, resetGpioCfg.levelshifter.value);
				time_printf(tmpStr);
			}
			npi_ipc_errno = NPI_LNX_ERROR_HAL_GPIO_RESET_LVLSHFT_VAL_OPEN;
			return NPI_LNX_FAILURE;
		}

		//Set the value of the GPIO to 0 (level shifter direction from Host to CC2531)
		if(ERROR == write(gpioResetFd, "1", 1))
		{
			perror(resetGpioCfg.levelshifter.value);
			if (__BIG_DEBUG_ACTIVE == TRUE)
			{
				snprintf(tmpStr, sizeof(tmpStr), "[%s] can't write in %s\n", __FUNCTION__, resetGpioCfg.levelshifter.value);
				time_printf(tmpStr);
			}
			npi_ipc_errno = NPI_LNX_ERROR_HAL_GPIO_RESET_LVLSHFT_VAL_WRITE;
			return NPI_LNX_FAILURE;
		}
		//close the DIR file
		close(gpioResetFd);
	}
	else
	{
		if (__BIG_DEBUG_ACTIVE == TRUE)
		{
			snprintf(tmpStr, sizeof(tmpStr), "[%s] Wrong Configuration File, one of the  following Key value are missing for MRDY.Level Shifter definition: '\n", __FUNCTION__);
			int strIndex = strlen(tmpStr);
			snprintf(tmpStr + strIndex, sizeof(tmpStr) - strIndex, "value: \t\t%s\n", resetGpioCfg.gpio.value);
			strIndex = strlen(tmpStr);
			snprintf(tmpStr + strIndex, sizeof(tmpStr) - strIndex, "direction: \t%s\n", resetGpioCfg.gpio.direction);
			strIndex = strlen(tmpStr);
			snprintf(tmpStr + strIndex, sizeof(tmpStr) - strIndex, "active_high_low: %d\n", resetGpioCfg.gpio.active_high_low);
			strIndex = strlen(tmpStr);
			snprintf(tmpStr + strIndex, sizeof(tmpStr) - strIndex, "Level Shifter is optional, please check if you need it or not before continuing...\n");
		}
	}

	//open the RESET GPIO DIR file
	gpioResetFd = open(resetGpioCfg.gpio.direction, O_RDWR);
	if(gpioResetFd == 0)
	{
		perror(resetGpioCfg.gpio.direction);
		if (__BIG_DEBUG_ACTIVE == TRUE)
		{
			snprintf(tmpStr, sizeof(tmpStr), "[%s] %s open failed\n", __FUNCTION__, resetGpioCfg.gpio.direction);
			time_printf(tmpStr);
		}
		npi_ipc_errno = NPI_LNX_ERROR_HAL_GPIO_RESET_GPIO_DIR_OPEN;
		return NPI_LNX_FAILURE;
	}

	//Set RESET GPIO as output
	if(ERROR == write(gpioResetFd, "out", 3))
	{
		perror(resetGpioCfg.gpio.direction);
		if (__BIG_DEBUG_ACTIVE == TRUE)
		{
			snprintf(tmpStr, sizeof(tmpStr), "[%s] can't write in %s\n", __FUNCTION__, resetGpioCfg.gpio.direction);
			time_printf(tmpStr);
		}
		npi_ipc_errno = NPI_LNX_ERROR_HAL_GPIO_RESET_GPIO_DIR_WRITE;
		return NPI_LNX_FAILURE;
	}
	//close RESET DIR file
	close(gpioResetFd);

	//open the RESET GPIO VALUE file so it can be written to using the file handle later
	gpioResetFd = open(resetGpioCfg.gpio.value, O_RDWR);
	if(gpioResetFd == 0)
	{
		perror(resetGpioCfg.gpio.value);
		if (__BIG_DEBUG_ACTIVE == TRUE)
		{
			snprintf(tmpStr, sizeof(tmpStr), "[%s] %s open failed\n", __FUNCTION__, resetGpioCfg.gpio.value);
			time_printf(tmpStr);
		}
		npi_ipc_errno = NPI_LNX_ERROR_HAL_GPIO_RESET_GPIO_VAL_OPEN;
		return NPI_LNX_FAILURE;
	}

	//Set RESET GPIO to 1 as default
	if(ERROR == write(gpioResetFd, "1", 3))
	{
		perror(resetGpioCfg.gpio.value);
		if (__BIG_DEBUG_ACTIVE == TRUE)
		{
			snprintf(tmpStr, sizeof(tmpStr), "[%s] can't write in %s\n", __FUNCTION__, resetGpioCfg.gpio.value);
			time_printf(tmpStr);
		}
		npi_ipc_errno = NPI_LNX_ERROR_HAL_GPIO_RESET_GPIO_VAL_WRITE;
		return NPI_LNX_FAILURE;
	}

	return NPI_LNX_SUCCESS;
}

/**************************************************************************************************
 * @fn      HalGpioMrdySet
 *
 *
 * @brief   Set MRDY.
 *
 * @param
 *
 * @return  STATUS
 **************************************************************************************************/
int HalGpioMrdySet(uint8 state)
{
	char tmpStr[128];
	if(state == 0)
	{
		//		debug_printf("[%u][GPIO] MRDY set to low\n", (unsigned int) pthread_self());
		if (ERROR == write(gpioMrdyFd, "0", 1))
		{
			perror(mrdyGpioCfg.gpio.value);
			if (__BIG_DEBUG_ACTIVE == TRUE)
			{
				snprintf(tmpStr, sizeof(tmpStr), "[%s] can't write in %s, is something already accessing it? abort everything for debug purpose...\n",
						__FUNCTION__, mrdyGpioCfg.gpio.value);
				time_printf(tmpStr);
			}
			npi_ipc_errno = NPI_LNX_ERROR_HAL_GPIO_MRDY_GPIO_VAL_WRITE_SET_LOW;
			return NPI_LNX_FAILURE;
		}
	}
	else
	{
		//		debug_printf("[%u][GPIO] MRDY set to High\n", (unsigned int) pthread_self());
		if(ERROR == write(gpioMrdyFd, "1", 1))
		{
			perror(mrdyGpioCfg.gpio.value);
			if (__BIG_DEBUG_ACTIVE == TRUE)
			{
				snprintf(tmpStr, sizeof(tmpStr), "[%s] can't write in %s, is something already accessing it? abort everything for debug purpose...\n",
						__FUNCTION__, mrdyGpioCfg.gpio.value);
				time_printf(tmpStr);
			}
			npi_ipc_errno = NPI_LNX_ERROR_HAL_GPIO_MRDY_GPIO_VAL_WRITE_SET_HIGH;
			return NPI_LNX_FAILURE;
		}
	}

	return NPI_LNX_SUCCESS;

}
/**************************************************************************************************
 * @fn      HalGpioMrdyCheck
 *
 *
 * @brief   Check MRDY Clear.
 *
 * @param   state	- Active  or  Inactive
 *
 * @return  None
 **************************************************************************************************/
int HalGpioMrdyCheck(uint8 state)
{
	char mrdy=2, tmpStr[128];
	lseek(gpioMrdyFd,0,SEEK_SET);
	if(ERROR == read(gpioMrdyFd,&mrdy, 1))
	{
		perror(mrdyGpioCfg.gpio.value);
		if (__BIG_DEBUG_ACTIVE == TRUE)
		{
			snprintf(tmpStr, sizeof(tmpStr), "[%s] can't write in %s, is something already accessing it? abort everything for debug purpose...\n",
					__FUNCTION__, mrdyGpioCfg.gpio.value);
			time_printf(tmpStr);
		}
		npi_ipc_errno = NPI_LNX_ERROR_HAL_GPIO_MRDY_GPIO_VAL_READ;
		return NPI_LNX_FAILURE;
	}

	//	debug_printf("[%u][GPIO]===>check MRDY: %c  (%c) \n", (unsigned int) pthread_self(), mrdy, mrdy);

	return (state == ((mrdy == '1') ? 1 : 0));
}

/**************************************************************************************************
 * @fn      HalGpioResetSet
 *
 *
 * @brief   Set Reset.
 *
 * @param
 *
 * @return  STATUS
 **************************************************************************************************/
int HalGpioResetSet(uint8 state)
{
	char tmpStr[128];
	if(state == 0)
	{
		if (__BIG_DEBUG_ACTIVE == TRUE)
		{
			snprintf(tmpStr, sizeof(tmpStr), "[%s] Reset set low. [Thread %u]\n", __FUNCTION__, (unsigned int) pthread_self());
			time_printf(tmpStr);
		}
		if (ERROR == write(gpioResetFd, "0", 1))
		{
			perror(resetGpioCfg.gpio.value);
			if (__BIG_DEBUG_ACTIVE == TRUE)
			{
				snprintf(tmpStr, sizeof(tmpStr), "[%s] can't write in %s, is something already accessing it? abort everything for debug purpose...\n",
						__FUNCTION__, resetGpioCfg.gpio.value);
				time_printf(tmpStr);
			}
			npi_ipc_errno = NPI_LNX_ERROR_HAL_GPIO_RESET_GPIO_VAL_WRITE_SET_LOW;
			return NPI_LNX_FAILURE;
		}
	}
	else
	{
		if (__BIG_DEBUG_ACTIVE == TRUE)
		{
			snprintf(tmpStr, sizeof(tmpStr), "[%s] Reset set high. [Thread %u]\n", __FUNCTION__, (unsigned int) pthread_self());
			time_printf(tmpStr);
		}
		if(ERROR == write(gpioResetFd, "1", 1))
		{
			perror(resetGpioCfg.gpio.value);
			if (__BIG_DEBUG_ACTIVE == TRUE)
			{
				snprintf(tmpStr, sizeof(tmpStr), "[%s] can't write in %s, is something already accessing it? abort everything for debug purpose...\n",
						__FUNCTION__, resetGpioCfg.gpio.value);
				time_printf(tmpStr);
			}
			npi_ipc_errno = NPI_LNX_ERROR_HAL_GPIO_RESET_GPIO_VAL_WRITE_SET_HIGH;
			return NPI_LNX_FAILURE;
		}
	}
	if (__BIG_DEBUG_ACTIVE == TRUE)
	{
		snprintf(tmpStr, sizeof(tmpStr), "[%s] Reset called with state = %d\n", __FUNCTION__, state);
		time_printf(tmpStr);
	}

	return NPI_LNX_SUCCESS;

}
/**************************************************************************************************
 * @fn      HalGpioReset
 *
 *
 * @brief   Set MRDY.
 *
 * @param   None
 *
 * @return  STATUS
 **************************************************************************************************/
int HalGpioReset(void)
{
	char tmpStr[128];
	if (__BIG_DEBUG_ACTIVE == TRUE)
	{
		snprintf(tmpStr, sizeof(tmpStr), "[%s] Reset High\n", __FUNCTION__);
		time_printf(tmpStr);
	}
	if(ERROR == write(gpioResetFd, "1", 1))
	{
		perror(resetGpioCfg.gpio.value);
		if (__BIG_DEBUG_ACTIVE == TRUE)
		{
			snprintf(tmpStr, sizeof(tmpStr), "[%s] can't write in %s , is something already accessing it? abort everything for debug purpose...\n",
					__FUNCTION__, resetGpioCfg.gpio.value);
			time_printf(tmpStr);
		}
		npi_ipc_errno = NPI_LNX_ERROR_HAL_GPIO_RESET_GPIO_VAL_WRITE_SET_HIGH;
		return NPI_LNX_FAILURE;
	}
	if (__BIG_DEBUG_ACTIVE == TRUE)
	{
		snprintf(tmpStr, sizeof(tmpStr), "[%s] Reset Low, [Thread: %u]\n", __FUNCTION__, (unsigned int) pthread_self());
		time_printf(tmpStr);
	}
	if(ERROR == write(gpioResetFd, "0", 1))
	{
		perror(resetGpioCfg.gpio.value);
		if (__BIG_DEBUG_ACTIVE == TRUE)
		{
			snprintf(tmpStr, sizeof(tmpStr), "[%s] can't write in %s , is something already accessing it? abort everything for debug purpose...\n",
					__FUNCTION__, resetGpioCfg.gpio.value);
			time_printf(tmpStr);
		}
		npi_ipc_errno = NPI_LNX_ERROR_HAL_GPIO_RESET_GPIO_VAL_WRITE_SET_LOW;
		return NPI_LNX_FAILURE;
	}
	//Add A Delay here:
	// Reset Should last at least 1us from datasheet, set it to 500us.
	usleep(500);

	if (__BIG_DEBUG_ACTIVE == TRUE)
	{
		snprintf(tmpStr, sizeof(tmpStr), "[%s] Reset High, [Thread: %u]\n", __FUNCTION__, (unsigned int) pthread_self());
		time_printf(tmpStr);
	}
	if(ERROR == write(gpioResetFd, "1", 1))
	{
		perror(resetGpioCfg.gpio.value);
		if (__BIG_DEBUG_ACTIVE == TRUE)
		{
			snprintf(tmpStr, sizeof(tmpStr), "[%s] can't write in %s , is something already accessing it? abort everything for debug purpose...\n",
					__FUNCTION__, resetGpioCfg.gpio.value);
			time_printf(tmpStr);
		}
		npi_ipc_errno = NPI_LNX_ERROR_HAL_GPIO_RESET_GPIO_VAL_WRITE_SET_HIGH;
		return NPI_LNX_FAILURE;
	}

	return NPI_LNX_SUCCESS;
}
/**************************************************************************************************
 * @fn      HalGpioSrdyCheck
 *
 *
 * @brief   Check SRDY Clear.
 *
 * @param   state	- Active  or  Inactive
 *
 * @return  STATUS if error, otherwise boolean TRUE/FALSE if state is matching
 **************************************************************************************************/
int HalGpioSrdyCheck(uint8 state)
{
	char srdy=2, tmpStr[128];;
	lseek(gpioSrdyFd,0,SEEK_SET);
	if(ERROR == read(gpioSrdyFd,&srdy, 1))
	{
		perror(srdyGpioCfg.gpio.value);
		if (__BIG_DEBUG_ACTIVE == TRUE)
		{
			snprintf(tmpStr, sizeof(tmpStr), "[%s] can't write in %s , is something already accessing it? abort everything for debug purpose...\n",
					__FUNCTION__, srdyGpioCfg.gpio.value);
			time_printf(tmpStr);
		}
		npi_ipc_errno = NPI_LNX_ERROR_HAL_GPIO_SRDY_GPIO_VAL_READ_FAILED;
		return NPI_LNX_FAILURE;
	}

	//	debug_printf("[%u][GPIO]===>check SRDY: %c [%d]  (%c) \n", (unsigned int) pthread_self(), srdy, srdy, atoi(&srdy));

	return (state == ((srdy == '1') ? 1 : 0));
}

/**************************************************************************************************
 * @fn          HalGpioWaitSrdyClr
 *
 * @brief       Check that SRDY is low, if not, wait until it gets low.
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * None.
 *
 * @return      STATUS
 **************************************************************************************************
 */
int HalGpioWaitSrdyClr(void)
{
	char srdy= '1', tmpStr[128];
	int ret = NPI_LNX_SUCCESS, accTimeout = 0;

	if (__BIG_DEBUG_ACTIVE == TRUE)
	{
		snprintf(tmpStr, sizeof(tmpStr), "[%s] Wait for SRDY to go Low. [Thread: %u]\n", __FUNCTION__, (unsigned int) pthread_self());
		time_printf(tmpStr);
	}

	struct pollfd ufds[1];
	int pollRet;
	ufds[0].fd = gpioSrdyFd;
	ufds[0].events = POLLPRI;

	lseek(gpioSrdyFd,0,SEEK_SET);
	read(gpioSrdyFd,&srdy, 1);

	while(srdy == '1')
	{
		pollRet = poll((struct pollfd*)&ufds, 1, HAL_WAIT_SRDY_LOW_INTERMEDIATE_TIMEOUT);
		if (pollRet == -1)
		{
			// Error occured in poll()
			perror("poll");
		}
		else if (pollRet == 0)
		{
			accTimeout++;
			// Timeout
			lseek(gpioSrdyFd,0,SEEK_SET);
			read(gpioSrdyFd,&srdy, 1);

			if(srdy == '1')
			{
				if (accTimeout >= (HAL_WAIT_SRDY_LOW_TIMEOUT / HAL_WAIT_SRDY_LOW_INTERMEDIATE_TIMEOUT) )
				{
					if (__BIG_DEBUG_ACTIVE == TRUE)
					{
						snprintf(tmpStr, sizeof(tmpStr), "[%s] Waiting for SRDY to go low timed out. [Thread: %u]\n", __FUNCTION__, (unsigned int) pthread_self());
						time_printf(tmpStr);
					}
					npi_ipc_errno = NPI_LNX_ERROR_HAL_GPIO_WAIT_SRDY_CLEAR_POLL_TIMEDOUT;
					ret = NPI_LNX_FAILURE;
					break;
				}
				else
				{
					// This timeout is expected, and ok. Nothing to report, only for debug.
					if (__BIG_DEBUG_ACTIVE == TRUE)
					{
						snprintf(tmpStr, sizeof(tmpStr), "[%s] Waiting for SRDY to go low intermediate timed out, %d. [Thread: %u]\n",
								__FUNCTION__, accTimeout, (unsigned int) pthread_self());
						time_printf(tmpStr);
					}
				}
			}
			else
			{
				// Missed interrupt waiting for SRDY to go high
				// This timeout is expected, and ok. Nothing to report, only for debug.
				if (__BIG_DEBUG_ACTIVE == TRUE)
				{
					snprintf(tmpStr, sizeof(tmpStr), "[%s] Waiting for SRDY to go low intermediate timed out, %d, but SRDY is now low. [Thread: %u]\n",
							__FUNCTION__, accTimeout, (unsigned int) pthread_self());
					time_printf(tmpStr);
				}
				break;
			}
		}
		else
		{
			if (ufds[0].revents & POLLPRI)
			{
				lseek(gpioSrdyFd,0,SEEK_SET);
				if(ERROR == read(gpioSrdyFd,&srdy, 1))
				{
					perror(srdyGpioCfg.gpio.value);
					if (__BIG_DEBUG_ACTIVE == TRUE)
					{
						snprintf(tmpStr, sizeof(tmpStr), "[%s] can't write in %s , is something already accessing it? abort everything for debug purpose...\n",
								__FUNCTION__, srdyGpioCfg.gpio.value);
						time_printf(tmpStr);
					}
					npi_ipc_errno = NPI_LNX_ERROR_HAL_GPIO_WAIT_SRDY_CLEAR_READ_FAILED;
					ret = NPI_LNX_FAILURE;
					break;
				}
			}
			else
			{
				if (__BIG_DEBUG_ACTIVE == TRUE)
				{
					snprintf(tmpStr, sizeof(tmpStr), "[%s] ufds[0].revents = 0x%X\n", __FUNCTION__, ufds[0].revents);
					time_printf(tmpStr);
				}
			}
		}
	}
	if (__BIG_DEBUG_ACTIVE == TRUE)
	{
		snprintf(tmpStr, sizeof(tmpStr), "[%s] SRDY: %c (%d). [Thread: %u]\n", __FUNCTION__, srdy, (int)srdy, (unsigned int) pthread_self());
		time_printf(tmpStr);
	}

	return ret;
}

/**************************************************************************************************
 * @fn          HalGpioWaitSrdySet
 *
 * @brief       Check that SRDY is High, if not, wait until it gets high, or times out.
 * 				0xFFFF means never time out.
 *
 * input parameters
 *
 * None
 *
 * output parameters
 *
 * None.
 *
 * @return      STATUS
 **************************************************************************************************
 */
int HalGpioWaitSrdySet()
{
	char srdy= '0', first = 0, tmpStr[128];

	int ret = NPI_LNX_SUCCESS, accTimeout = 0;

	//	debug_printf("[%u][GPIO]Wait SRDY High, \n", (unsigned int) pthread_self());

	struct pollfd ufds[1];
	int pollRet;
	ufds[0].fd = gpioSrdyFd;
	ufds[0].events = POLLPRI;

	if (__BIG_DEBUG_ACTIVE == TRUE)
	{
		snprintf(tmpStr, sizeof(tmpStr), "[%s] Wait for SRDY to go High. [Thread: %u]\n", __FUNCTION__, (unsigned int) pthread_self());
		time_printf(tmpStr);
	}

	lseek(gpioSrdyFd,0,SEEK_SET);
	read(gpioSrdyFd,&srdy, 1);

	while( (srdy == '0') )
	{
		// There is still a chance we can miss an interrupt. So we need to split the
		// big HAL_WAIT_SRDY_HIGH_TIMEOUT timeout into smaller timeouts
		if (first == 0)
		{
			pollRet = poll((struct pollfd*)&ufds, 1, HAL_WAIT_SRDY_HIGH_FIRST_TIMEOUT);
		}
		else
		{
			pollRet = poll((struct pollfd*)&ufds, 1, HAL_WAIT_SRDY_HIGH_INTERMEDIATE_TIMEOUT);
		}
		if (pollRet == -1)
		{
			// Error occured in poll()
			perror("poll");
		}
		else if (pollRet == 0)
		{
			if (first == 0)
			{
				accTimeout += HAL_WAIT_SRDY_HIGH_FIRST_TIMEOUT;
				first++;
			}
			else
			{
				accTimeout += HAL_WAIT_SRDY_HIGH_INTERMEDIATE_TIMEOUT;
			}
			// Timeout
			lseek(gpioSrdyFd,0,SEEK_SET);
			read(gpioSrdyFd,&srdy, 1);

			if(srdy == '0')
			{
				if (accTimeout >= HAL_WAIT_SRDY_HIGH_TIMEOUT)
				{
					if (__BIG_DEBUG_ACTIVE == TRUE)
					{
						snprintf(tmpStr, sizeof(tmpStr), "[%s] Waiting for SRDY to go high timed out. [Thread: %u]\n", __FUNCTION__, (unsigned int) pthread_self());
						time_printf(tmpStr);
					}
					npi_ipc_errno = NPI_LNX_ERROR_HAL_GPIO_WAIT_SRDY_SET_POLL_TIMEDOUT;
					ret = NPI_LNX_FAILURE;
					break;
				}
				else
				{
					// This timeout is expected, and ok. Nothing to report, only for debug.
					if (__BIG_DEBUG_ACTIVE == TRUE)
					{
						snprintf(tmpStr, sizeof(tmpStr), "[%s] Waiting for SRDY to go high intermediate timed out, %d. [Thread: %u]\n",
								__FUNCTION__, accTimeout, (unsigned int) pthread_self());
						time_printf(tmpStr);
					}
				}
			}
			else
			{
				// Missed interrupt waiting for SRDY to go high
				// This timeout is expected, and ok. Nothing to report, only for debug.
				if (__BIG_DEBUG_ACTIVE == TRUE)
				{
					snprintf(tmpStr, sizeof(tmpStr), "[%s] Waiting for SRDY to go high intermediate timed out, %d, but SRDY is now high. [Thread: %u]\n",
							__FUNCTION__, accTimeout, (unsigned int) pthread_self());
					time_printf(tmpStr);
				}
				break;
			}
		}
		else
		{
			if (ufds[0].revents & POLLPRI)
			{
				lseek(gpioSrdyFd,0,SEEK_SET);
				if(ERROR == read(gpioSrdyFd,&srdy, 1))
				{
					perror(srdyGpioCfg.gpio.value);
					if (__BIG_DEBUG_ACTIVE == TRUE)
					{
						snprintf(tmpStr, sizeof(tmpStr), "[%s] can't write in %s , is something already accessing it? abort everything for debug purpose...\n",
								__FUNCTION__, srdyGpioCfg.gpio.value);
						time_printf(tmpStr);
					}
					npi_ipc_errno = NPI_LNX_ERROR_HAL_GPIO_WAIT_SRDY_SET_READ_FAILED;
					return NPI_LNX_FAILURE;
				}
			}
			if (__BIG_DEBUG_ACTIVE == TRUE)
			{
				snprintf(tmpStr, sizeof(tmpStr), "[%s] ufds[0].revents = 0x%X\n", __FUNCTION__, ufds[0].revents);
				time_printf(tmpStr);
			}
		}
	}
	if (__BIG_DEBUG_ACTIVE == TRUE)
	{
		snprintf(tmpStr, sizeof(tmpStr), "[%s] SRDY: %c (%d). [Thread: %u]\n", __FUNCTION__, srdy, (int)srdy, (unsigned int) pthread_self());
		time_printf(tmpStr);
	}

	return ret;
}

/**************************************************************************************************
 **************************************************************************************************/
