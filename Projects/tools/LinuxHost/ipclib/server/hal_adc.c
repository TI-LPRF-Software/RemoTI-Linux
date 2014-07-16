/**************************************************************************************************
  Filename:       hal_adc.c
  Revised:        $Date: 2012-03-15 13:45:31 -0700 (Thu, 15 Mar 2012) $
  Revision:       $Revision: 237 $

  Description:    This file contains the interface to the HAL I2C.


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
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>

//RemoTI Include
//#include  "hal_board.h"
#include  "hal_types.h"
#include  "hal_adc.h"

#include "npi_lnx_error.h"

#ifdef __BIG_DEBUG__
#define debug_printf(fmt, ...) printf( fmt, ##__VA_ARGS__)
#else
#define debug_printf(fmt, ...) st (if (__BIG_DEBUG_ACTIVE == TRUE) printf( fmt, ##__VA_ARGS__);)
#endif

#if (defined NPI_ADC_CONTROL) && (NPI_ADC_CONTROL == TRUE)
/**************************************************************************************************
 *                                            CONSTANTS
 **************************************************************************************************/
#define MAX_NUM_OF_ADC_PORTS					8
/**************************************************************************************************
 *                                              MACROS
 **************************************************************************************************/

/**************************************************************************************************
 *                                            TYPEDEFS
 **************************************************************************************************/

/**************************************************************************************************
 *                                         GLOBAL VARIABLES
 **************************************************************************************************/
static int adcDevFd[MAX_NUM_OF_ADC_PORTS] = {-1, -1, -1, -1, -1, -1, -1, -1};

/**************************************************************************************************
 *                                          FUNCTIONS - API
 **************************************************************************************************/

/**************************************************************************************************
 * @fn      HalADCInit
 *
 * @brief   Initialize ADC Service
 *
 * @param   devpaths			-	list of ADC devices
 * 			numberOfAdcDevices	-	number of ADC devices
 *
 * @return  STATUS
 **************************************************************************************************/
int HalADCInit(char **devpaths, char numberOfAdcDevices)
{
	// If the device is open, attempt to close it first.
	uint8 i;
	debug_printf("\n\nInitializing ADC\n");
	for (i = 0; i < MAX_NUM_OF_ADC_PORTS; i++)
	{
		if (adcDevFd[i] >= 0)
		{
			/* The device is open, attempt to close it first */
			debug_printf("Closing %d %s ...\n", adcDevFd[i], devpaths[i]);
			close(adcDevFd[i]);
		}
	}

	/* open the devices */
	for (i = 0; i < numberOfAdcDevices; i++)
	{
		debug_printf("Opening %s ...\n", devpaths[i]);

		adcDevFd[i] = open(devpaths[i], O_RDONLY | O_NONBLOCK);
		if(adcDevFd[i] < 0)
		{
			fprintf(stderr, "Error: Could not open file "
					"%s: %s, already used?\n", devpaths[i], strerror(errno));
			debug_printf("Error: Could not open file "
					"%s: %s, already used?\n", devpaths[i], strerror(errno));
			npi_ipc_errno = NPI_LNX_ERROR_HAL_ADC_INIT_FAILED_TO_OPEN_DEVICE;
			return NPI_LNX_FAILURE;
		}
	}

	return NPI_LNX_SUCCESS;
}
/**************************************************************************************************
 * @fn      HalADCClose
 *
 * @brief   Close ADC Service
 *
 * @param   None
 *
 * @return  STATUS
 **************************************************************************************************/
int HalADCClose(void)
{
	debug_printf("Closing ADC file descriptors\n");
	uint8 i;
	int ret = NPI_LNX_SUCCESS;
	for (i = 0; i < sizeof(adcDevFd); i++)
	{
		if (adcDevFd[i] >= 0)
		{
			if (-1 == close(adcDevFd[i]))
			{
				fprintf(stderr, "Error: Could not Close ADC device file %d (%d), reason:  %s,\n", adcDevFd[i], i, strerror(errno));

			    npi_ipc_errno = NPI_LNX_ERROR_HAL_ADC_CLOSE_GENERIC;
			    ret = NPI_LNX_FAILURE;
			}
		}
	}

	return ret;
}

/**************************************************************************************************
 * @fn      HalADCRead
 *
 * @brief   Read the ADC.
 *
 * @param   port - ADC port.
 *          pBuf - Pointer to the buffer that will be written.
 *
 * @return  STATUS
 **************************************************************************************************/
int HalADCRead(uint8 port, int *voltage)
{
  int ret = NPI_LNX_SUCCESS;
  char strBuf[32] = {"\0"};

  if (port < sizeof(adcDevFd))
  {
	  if (adcDevFd[port] >= 0)
	  {
		  // Rewind file before reading
		  lseek(adcDevFd[port], 0, SEEK_SET);
		  if (read(adcDevFd[port], strBuf, sizeof(int)) < 0)
		  {
			  npi_ipc_errno = NPI_LNX_ERROR_HAL_ADC_FAILED_TO_READ;
			  ret = NPI_LNX_FAILURE;
		  }
		  else
		  {
			  printf("[INFO] Voltage read: %s\n", strBuf);
		  }
	  }
	  else
	  {
		  npi_ipc_errno = NPI_LNX_ERROR_HAL_ADC_PORT_NOT_OPEN;
		  ret = NPI_LNX_FAILURE;
	  }
  }
  else
  {
	  npi_ipc_errno = NPI_LNX_ERROR_HAL_ADC_INVALID_PORT;
	  ret = NPI_LNX_FAILURE;
  }

  if (ret == NPI_LNX_SUCCESS)
  {
	  // Convert to actual value. 4096 is 1800mV
	  double actualVoltage = strtol(strBuf, NULL, 10);
	  *voltage = (int)((actualVoltage / 4096) * 1800);
	  printf("[INFO] Voltage read: %d (%f), from port %d\n", *voltage, actualVoltage, port);
  }
  else
  {
	  *voltage = -1;
  }

  return ret;
}

#endif

/**************************************************************************************************
**************************************************************************************************/
