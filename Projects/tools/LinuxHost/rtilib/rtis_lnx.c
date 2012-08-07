 /**************************************************************************************************
  Filename:       rtis_lnx.c
  Revised:        $Date: 2012-03-21 17:37:33 -0700 (Wed, 21 Mar 2012) $
  Revision:       $Revision: 246 $

  Description:    This file contains Linux platform specific RemoTI (RTI) API
                  Surrogate implementation

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
 *                                           Includes
 **************************************************************************************************/

#include <assert.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>

/* NPI includes */
#include "npi_lnx.h"

/* RTI includes */
#include "rti_lnx_constants.h"
#include "rti_lnx.h"
#include "rtis_lnx.h"


//#if (defined HAL_SPI) && (HAL_SPI == TRUE)
#include "npi_lnx_spi.h"
#include "hal_spi.h"
//#endif

//#if (defined HAL_I2C) && (HAL_I2C == TRUE)
#include "npi_lnx_i2c.h"
#include "hal_i2c.h"
//#endif


//#if (defined HAL_UART) && (HAL_UART == TRUE)
#include "npi_lnx_uart.h"
// The following is only necessary because we always read out GPIO configuration
#include "hal_gpio.h"
//#endif


#ifdef __BIG_DEBUG__
#define debug_printf(fmt, ...) printf( fmt, ##__VA_ARGS__)
#else
#define debug_printf(fmt, ...)
#endif
/**************************************************************************************************
 *                                        Externals
 **************************************************************************************************/

/**************************************************************************************************
 *                                           Constant
 **************************************************************************************************/

const char* sectionNamesArray[3][2]=
{
		{
				"GPIO_SRDY.GPIO",
				"GPIO_SRDY.LEVEL_SHIFTER",
		},
		{
				"GPIO_MRDY.GPIO",
				"GPIO_MRDY.LEVEL_SHIFTER",
		},
		{
				"GPIO_RESET.GPIO",
				"GPIO_RESET.LEVEL_SHIFTER",
		},
};

const pNPI_OpenDeviceFn NPI_OpenDeviceFnArr[] =
{
		NPI_UART_OpenDevice,
#if (defined HAL_SPI) && (HAL_SPI == TRUE)
		NPI_SPI_OpenDevice,
#endif
#if (defined HAL_I2C) && (HAL_I2C == TRUE)
		NPI_I2C_OpenDevice
#endif
};
const pNPI_CloseDeviceFn NPI_CloseDeviceFnArr[] =
{
		NPI_UART_CloseDevice,
#if (defined HAL_SPI) && (HAL_SPI == TRUE)
		NPI_SPI_CloseDevice,
#endif
#if (defined HAL_I2C) && (HAL_I2C == TRUE)
		NPI_I2C_CloseDevice
#endif
};
const pNPI_SendAsynchDataFn NPI_SendAsynchDataFnArr[] =
{
		NPI_UART_SendAsynchData,
#if (defined HAL_SPI) && (HAL_SPI == TRUE)
		NPI_SPI_SendAsynchData,
#endif
#if (defined HAL_I2C) && (HAL_I2C == TRUE)
		NPI_I2C_SendAsynchData
#endif
};
const pNPI_SendSynchDataFn NPI_SendSynchDataFnArr[] =
{
		NPI_UART_SendSynchData,
#if (defined HAL_SPI) && (HAL_SPI == TRUE)
		NPI_SPI_SendSynchData,
#endif
#if (defined HAL_I2C) && (HAL_I2C == TRUE)
		NPI_I2C_SendSynchData
#endif
};

const pNPI_ResetSlaveFn NPI_ResetSlaveFnArr[] =
{
		NULL,
#if (defined HAL_SPI) && (HAL_SPI == TRUE)
		NPI_SPI_ResetSlave,
#endif
#if (defined HAL_I2C) && (HAL_I2C == TRUE)
		NPI_I2C_ResetSlave,
#endif
};

const pNPI_SynchSlaveFn NPI_SynchSlaveFnArr[] =
{
		NULL,
#if (defined HAL_SPI) && (HAL_SPI == TRUE)
		NPI_SPI_SynchSlave,
#endif
#if (defined HAL_I2C) && (HAL_I2C == TRUE)
		NULL,
#endif
};

/**************************************************************************************************
 *                                        Type definitions
 **************************************************************************************************/

/**************************************************************************************************
 *                                        Global Variables
 **************************************************************************************************/

/**************************************************************************************************
 *                                        Local Variables
 **************************************************************************************************/

// current state
#if defined _WIN32 || defined unix
// windows does not require RTI_Init() call
static uint8 rtisState = RTIS_STATE_READY;
#else
static uint8 rtisState;
#endif // _WIN32
static uint8 rtisBE; // big endian machine flag

/**************************************************************************************************
 *                                     Local Function Prototypes
 **************************************************************************************************/

// RTI Task Related
void   RTI_Init( uint8 taskId );
uint16 RTI_ProcessEvent( uint8 taskId, uint16 events );

// NPI Callback Related
void NPI_AsynchMsgCback( npiMsgData_t *pMsg );

// Endianness determination
static uint8 rtisIsBE( void );

// Endianness conversion
static void rtisAttribEConv( uint8 attrib, uint8 len, uint8 *pValue );

// Endianness conversion macros
#define RTI_ECONV16(_value) ((((_value) & 0xff)<<8)|(((_value) & 0xff00)>>8))
#define RTI_ECONV32(_value) \
  ((((_value) & 0xff)<<24)|(((_value) & 0xff00)<<8)| \
   (((_value) & 0xff0000)>>8)|(((_value) & 0xff000000)>>24))
#define RTI_PAIRING_ENTRY_REQUIRED_LEN(_field) \
  ((uint16) (&((rcnNwkPairingEntry_t *) 0)->_field) + sizeof(((rcnNwkPairingEntry_t *) 0)->_field))

char* SerialConfigParser(FILE* serialCfgFd,
		const char* section,
		const char* key,
		char* resString);

static uint8 devIdx = 0;


void npiSynchSlave( void )
{
  // This function is specific to SPI
//#if (defined HAL_SPI) && (HAL_SPI == TRUE)
//  NPI_SynchSlave();
//#endif
}

void halResetSlave( void )
{
  // This function is specific to SPI and I2C
//#if ( (defined HAL_SPI) && (HAL_SPI == TRUE) ) || ( (defined HAL_I2C) && (HAL_I2C == TRUE))
//  NPI_ResetSlave();
//#endif
}

// memcpy routine
void *msg_memcpy( void *dst, const void *src, uint16 len )
{
  return memcpy(dst, src, (size_t) len);
}

// TODO: relocate this
void rtisFatalError( uint8 status )
{
  // Do nothing for now
  assert(0);
}

/**************************************************************************************************
 * @fn          halDelay
 *
 * @brief       Delay for milliseconds.
 *              Do not invoke with zero.
 *              Do not invoke with greater than 500 msecs.
 *              Invoking with very high frequency and/or with long delays will start to
 *              significantly impact the real time performance of TimerA tasks because this will
 *              invisibly overrun the period when the TimerA count remaining, when this function
 *              is invoked, is less than the delay requested.
 *
 * input parameters
 *
 * @param       msecs - Milliseconds to delay in low power mode.
 * @param       sleep - Enforces blocking delay in low power mode if set.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void halDelay(uint8 msecs, uint8 sleep)
{
  if (sleep)
  {
    usleep(msecs * 1000);
  }
}

/**************************************************************************************************
 *
 * @fn          RTIS_Init
 *
 * @brief       This function initializes RTI Surrogate
 *
 * input parameters
 *
 * @param       devpath - path to the serial port device connected to
 * 						  the network processor
 *
 * output parameters
 *
 * None.
 *
 * @return      TRUE if the surrogate module started off successfully.
 *              FALSE, otherwise. Most of the cases it will be due to serial
 *              port opening failure.
 *
 **************************************************************************************************/

int RTIS_Init(const char *devpath)
{
	(void) devpath;
	int res = FALSE;
//	char i;

	FILE *serialCfgFd;
	char* strBuf;
	char* pStrBufRoot;
	char* devPath;
	uint8 gpioIdx = 0;
	halGpioCfg_t** gpioCfg;

//	halGpioCfg_t gpioCfgArr[3];
//	gpioCfg = (halGpioCfg_t** )&gpioCfgArr;


	// Allocate memory for string buffer and configuration buffer
	strBuf = (char*)malloc(128);
	pStrBufRoot = strBuf;
	devPath = (char*)malloc(128);
	gpioCfg = (halGpioCfg_t**)malloc(3 * sizeof(halGpioCfg_t*));
	debug_printf("gpioCfg \t\t\t\t@0x%.8X\n",
			(unsigned int)&(gpioCfg));
	for (gpioIdx = 0; gpioIdx < 3; gpioIdx++)
	{
		gpioCfg[gpioIdx] = (halGpioCfg_t*)malloc(sizeof(halGpioCfg_t));
		debug_printf("gpioCfg[%d] \t\t\t\t@0x%.8X\n",
				gpioIdx,
				(unsigned int)&(gpioCfg[gpioIdx]));
		debug_printf("gpioCfg[%d].gpio \t\t\t@0x%.8X\n",
				gpioIdx,
				(unsigned int)&(gpioCfg[gpioIdx]->gpio));
		debug_printf("gpioCfg[%d].levelshifter \t\t@0x%.8X\n",
				gpioIdx,
				(unsigned int)&(gpioCfg[gpioIdx]->levelshifter));
	}

	// Open file for parsing
	const char* configFilePath = "./RemoTI_RNP.cfg";
	serialCfgFd = fopen(configFilePath, "r");
	if (serialCfgFd == NULL)
	{
//		debug_
		printf("Could not open file '%s'\n", configFilePath);
		return res;
	}

	// Get device type
	strBuf = SerialConfigParser(serialCfgFd, "DEVICE", "deviceKey", strBuf);

	// Copy from buffer to variable
	devIdx = strBuf[0] - '0';
//	debug_
	printf("deviceKey = %i\n", devIdx);

	// Get path to the device
	strBuf = pStrBufRoot;
	strBuf = SerialConfigParser(serialCfgFd, "DEVICE", "devPath", strBuf);
	// Copy from buffer to variable
	memcpy(devPath, strBuf, strlen(strBuf));
//	debug_
	printf("devPath = '%s'\n", devPath);

//	printf("devPath = ");
//	for (i = 0; i < strlen(strBuf); i++)
//	{
//		printf("_");
//	}
//	printf("<\n");

	// GPIO configuration
	if ( (devIdx == 1) || (devIdx == 2) )
	{
		for (gpioIdx = 0; gpioIdx < 3; gpioIdx++)
		{
			// Get SRDY, MRDY or RESET GPIO
			debug_printf("gpioCfg[gpioIdx]->gpio \t\t\t@ 0x%.8X\n", (unsigned int)&(gpioCfg[gpioIdx]->gpio));

			// Get SRDY, MRDY or RESET GPIO value
			strBuf = pStrBufRoot;
			strBuf = SerialConfigParser(serialCfgFd, sectionNamesArray[gpioIdx][0], "value", strBuf);
			// Copy from buffer to variable
			debug_printf("strBuf \t\t\t\t\t@ 0x%.8X\n", (unsigned int)&strBuf);
			debug_printf("gpioCfg[gpioIdx]->gpio.value \t\t@ 0x%.8X\n", (unsigned int)&(gpioCfg[gpioIdx]->gpio.value));
			memcpy(gpioCfg[gpioIdx]->gpio.value, strBuf, strlen(strBuf));
			debug_printf("gpioCfg[%i]->gpio.value = '%s'\n",
					gpioIdx,
					gpioCfg[gpioIdx]->gpio.value);
//					strlen(strBuf));

			// Get SRDY, MRDY or RESET GPIO direction
			strBuf = pStrBufRoot;
			strBuf = SerialConfigParser(serialCfgFd, sectionNamesArray[gpioIdx][0], "direction", strBuf);
			// Copy from buffer to variable
			debug_printf("strBuf \t\t\t\t\t@ 0x%.8X\n", (unsigned int)&strBuf);
			debug_printf("gpioCfg[gpioIdx]->gpio.direction \t@ 0x%.8X\n", (unsigned int)&(gpioCfg[gpioIdx]->gpio.direction));
			memcpy(gpioCfg[gpioIdx]->gpio.direction, strBuf, strlen(strBuf));
			debug_printf("gpioCfg[%i]->gpio.direction = '%s'\n",
					gpioIdx,
					gpioCfg[gpioIdx]->gpio.direction);

			// Get SRDY, MRDY or RESET GPIO Active High/Low
			strBuf = pStrBufRoot;
			strBuf = SerialConfigParser(serialCfgFd, sectionNamesArray[gpioIdx][0], "active_high_low", strBuf);
			// Copy from buffer to variable
			gpioCfg[gpioIdx]->gpio.active_high_low = strBuf[0] - '0';
			debug_printf("gpioCfg[%i]->gpio.active_high_low = %d\n",
					gpioIdx,
					gpioCfg[gpioIdx]->gpio.active_high_low);

			// Get SRDY, MRDY or RESET Level Shifter
			debug_printf("gpioCfg[gpioIdx]->levelshifter \t\t\t@ 0x%.8X\n", (unsigned int)&(gpioCfg[gpioIdx]->levelshifter));

			// Get SRDY, MRDY or RESET Level Shifter value
			strBuf = pStrBufRoot;
			strBuf = SerialConfigParser(serialCfgFd, sectionNamesArray[gpioIdx][1], "value", strBuf);
			// Copy from buffer to variable
			memcpy(gpioCfg[gpioIdx]->levelshifter.value, strBuf, strlen(strBuf));
			debug_printf("gpioCfg[%i]->levelshifter.value = '%s'\n",
					gpioIdx,
					gpioCfg[gpioIdx]->levelshifter.value);

			// Get SRDY, MRDY or RESET Level Shifter direction
			strBuf = pStrBufRoot;
			strBuf = SerialConfigParser(serialCfgFd, sectionNamesArray[gpioIdx][1], "direction", strBuf);
			// Copy from buffer to variable
			memcpy(gpioCfg[gpioIdx]->levelshifter.direction, strBuf, strlen(strBuf));
			debug_printf("gpioCfg[%i]->levelshifter.direction = '%s'\n",
					gpioIdx,
					gpioCfg[gpioIdx]->levelshifter.direction);

			// Get SRDY, MRDY or RESET Level Shifter Active High/Low
			strBuf = pStrBufRoot;
			strBuf = SerialConfigParser(serialCfgFd, sectionNamesArray[gpioIdx][1], "active_high_low", strBuf);
			// Copy from buffer to variable
			gpioCfg[gpioIdx]->levelshifter.active_high_low = strBuf[0] - '0';
			debug_printf("gpioCfg[%i]->levelshifter.active_high_low = %d\n",
					gpioIdx,
					gpioCfg[gpioIdx]->levelshifter.active_high_low);
		}
	}

	//#if (defined HAL_UART) && (HAL_UART == TRUE)

	if (devIdx == 0)
	{
		res = (NPI_OpenDeviceFnArr[devIdx])(devPath, NULL);
	}
	else if (devIdx == 1)
	{
		npiSpiCfg_t spiCfg;
		strBuf = pStrBufRoot;
		strBuf = SerialConfigParser(serialCfgFd, "SPI", "speed", strBuf);
		spiCfg.speed = atoi(strBuf);
		spiCfg.gpioCfg = gpioCfg;
		res = (NPI_OpenDeviceFnArr[devIdx])(devPath, (npiSpiCfg_t *)&spiCfg);

		// Perform Reset of the RNP
		(NPI_ResetSlaveFnArr[devIdx])();

		// Do the Hw Handshake
		(NPI_SynchSlaveFnArr[devIdx])();
	}
	if (devIdx == 2)
	{
		npiI2cCfg_t i2cCfg;
		i2cCfg.gpioCfg = gpioCfg;

		// Open the Device and perform a reset
		res = (NPI_OpenDeviceFnArr[devIdx])(devPath, (npiI2cCfg_t *)&i2cCfg);
	}

	// Close file for parsing
	fclose(serialCfgFd);

	// Free memory for configuration buffers
	free(pStrBufRoot);
	free(devPath);
	for (gpioIdx = 0; gpioIdx < 3; gpioIdx++)
	{
		free(gpioCfg[gpioIdx]);
	}
	free(gpioCfg);

	return res;
}


/**************************************************************************************************
 *
 * @fn          SerialConfigParser
 *
 * @brief       This function searches for a string a returns its value
 *
 * input parameters
 *
 * @param	configFilePath	- path to configuration file
 * @param	section			- section to search for
 * @param	key				- key to return value of within section
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 *
 **************************************************************************************************/
char* SerialConfigParser(FILE* serialCfgFd, const char* section, const char* key, char* resString)
{
	uint8 sectionFound = FALSE, invalidLineLen = FALSE;
	char* psStr;	// Processing string pointer
//	psStr = (char*)malloc(128);

	debug_printf("------------------------------------------------------\n");
	debug_printf("Serial Config Parsing:\n");
	debug_printf("- \tSection \t%s:\n", section);
	debug_printf("- \tKey \t\t%s:\n", key);

	// Do nothing if the file doesn't exist
	if (serialCfgFd != NULL)
	{
		// Make sure we start search from the beginning of the file
		fseek(serialCfgFd, 0, SEEK_SET);

		// Search through the configuration file for the wanted
		while ( (resString = fgets(resString, 128, serialCfgFd)) != NULL)
		{
			// Check if we have a valid line, i.e. begins with [.
			// Note! No valid line can span more than 128 bytes. Hence we
			// must hold off parsing until we hit a newline.
			if (strlen(resString) == 128)
			{
				invalidLineLen = TRUE;
				debug_printf("Found line > 128 bytes\r");
				fflush(stdout);
			}
			else
			{
				// First time we find a valid line length after having
				// found invalid line length may be the end of the
				// invalid line. Hence, do not process this string.
				// We set the invalidLineLen parameter to FALSE after
				// the processing logic.
				if (invalidLineLen == FALSE)
				{
					// Remove the newline character (ok even if line had length 128)
					resString[strlen(resString) - 1] = '\0';

					debug_printf("Found line < 128 bytes\r");
					fflush(stdout);
					if (resString[0] == '[')
					{
						debug_printf("Found section %s\n", resString);
						// Search for wanted section
						psStr = strstr(resString, section);
						if (psStr != NULL)
						{
							resString = psStr;
							// We found our wanted section. Now search for wanted key.
							sectionFound = TRUE;
							debug_printf("Found wanted section!\n");
						}
						else
						{
							// We found another section.
							sectionFound = FALSE;
						}
					}
					else if (sectionFound == TRUE)
					{
						debug_printf("Line to process %s (strlen=%d)\n", resString, strlen(resString));
						// We have found our section, now we search for wanted key
						// Check for commented lines, tagged with '#', and
						// lines > 0 in length
						if ( (resString[0] != '#') && (strlen(resString) > 0) )
						{
							// Search for wanted section
							psStr = strstr(resString, key);
							if (psStr != NULL)
							{
								debug_printf("Found key \t'%s' in \t'%s'\n", key, resString);
								// We found our key. The value is located after the '='
								// after the key.
//								printf("%s\n", psStr);
								psStr = strtok(psStr, "=");
//								printf("%s\n", psStr);
								psStr = strtok(NULL, "=;\"");
//								printf("%s\n", psStr);

								resString = psStr;
								debug_printf("Found value '%s'\n", resString);

								// We can return this string to the calling function
								break;
							}
						}
					}
				}
				else
				{
					debug_printf("Found end of line > 128 bytes\n");
					invalidLineLen = FALSE;
				}
			}
		}
	}

//	free(psStr);

//	int i;
//	for (i = 0; i < strlen(resString); i++)
//	{
//		printf("_");
//	}
//	printf("<\n");
	// Return status of parsing
	return resString;
}

/**************************************************************************************************
 *
 * @fn          RTIS_Close
 *
 * @brief       This function stops RTI surrogate module
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 *
 **************************************************************************************************/
void RTIS_Close(void)
{
  (NPI_CloseDeviceFnArr[devIdx])();
}


/**************************************************************************************************
 *
 * @fn          RTI_Init
 *
 * @brief       This is the RemoTI task initialization called by OSAL.
 *
 * input parameters
 *
 * @param       task_id - Task identifier assigned after RTI was added in the
 *                        OSAL task queue.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 *
 **************************************************************************************************/
void RTI_Init( uint8 task_id )
{
  npiMsgData_t pMsg;

  // set task Id if one is needed
  (void)task_id;

  // determine endianness
  rtisBE = rtisIsBE();

  // set state during initialization
  rtisState = RTIS_STATE_INIT;

  // initialize the AP Network Processor Interface (NPI)
//  NPI_Init();

  // reset the slave hardware
  halResetSlave();

  // synchronize with the slave
  npiSynchSlave();

  // ping NP; ping request will be discarded
  pMsg.subSys   = RPC_SYS_RCAF;
  pMsg.cmdId    = RTIS_CMD_ID_TEST_PING_REQ;
  pMsg.len      = 2;
  pMsg.pData[0] = 0xAA;
  pMsg.pData[1] = 0xCC;

  // send command to slave
  (NPI_SendAsynchDataFnArr[devIdx])( &pMsg );

  // the RTIS is ready to go
  rtisState = RTIS_STATE_READY;
}

/**************************************************************************************************
 *
 * @fn          RTI_ReadItemEx
 *
 * @brief       This API is used to read an item from a Profile's Configuration Interface.
 *
 * input parameters
 *
 * @param       profileId - The Profile identifier.
 * @param       itemId - The Configuration Interface item identifier.
 * @param       len - The length in bytes of the item identifier's data.
 *
 * output parameters
 *
 * @param       *pValue - Pointer to buffer where read data is placed.
 *
 * @return      RTI_SUCCESS, RTI_ERROR_NOT_PERMITTED, RTI_ERROR_INVALID_INDEX,
 *              RTI_ERROR_INVALID_PARAMETER, RTI_ERROR_UNKNOWN_PARAMETER,
 *              RTI_ERROR_UNSUPPORTED_ATTRIBUTE, RTI_ERROR_OSAL_NV_OPER_FAILED,
 *              RTI_ERROR_OSAL_NV_ITEM_UNINIT, RTI_ERROR_OSAL_NV_BAD_ITEM_LEN
 *
 **************************************************************************************************/
rStatus_t RTI_ReadItemEx( uint8 profileId, uint8 itemId, uint8 len, uint8 *pValue )
{
  npiMsgData_t pMsg;

  // prep Read Item request
  // Note: no need to send pValue over the NPI
  pMsg.subSys   = RPC_SYS_RCAF;
  pMsg.cmdId    = RTIS_CMD_ID_RTI_READ_ITEM_EX;
  pMsg.len      = 3;
  pMsg.pData[0] = profileId;
  pMsg.pData[1] = itemId;
  pMsg.pData[2] = len;

  // send Read Item request to NP RTIS synchronously
  (NPI_SendSynchDataFnArr[devIdx])( &pMsg );

  // DEBUG
  if ( pMsg.pData[0] == RTI_ERROR_SYNCHRONOUS_NPI_TIMEOUT )
  {
    rtisFatalError( RTI_ERROR_SYNCHRONOUS_NPI_TIMEOUT );
  }

  // copy the reply data to the client's buffer
  // Note: the first byte of the payload is reserved for the status
  msg_memcpy( pValue, &pMsg.pData[1], len );

  // perform endianness change
  rtisAttribEConv( itemId, len, pValue );

  // return the status, which is stored is the first byte of the payload
  return( (rStatus_t)pMsg.pData[0] );
}

/**************************************************************************************************
 *
 * @fn          RTI_ReadItem
 *
 * @brief       This API is used to read the RTI Configuration Interface item
 *              from the Configuration Parameters table, the State Attributes
 *              table, or the Constants table.
 *
 * input parameters
 *
 * @param       itemId - The Configuration Interface item identifier.
 * @param       len - The length in bytes of the item identifier's data.
 *
 * output parameters
 *
 * @param       *pValue - Pointer to buffer where read data is placed.
 *
 * @return      RTI_SUCCESS, RTI_ERROR_NOT_PERMITTED, RTI_ERROR_INVALID_INDEX,
 *              RTI_ERROR_INVALID_PARAMETER, RTI_ERROR_UNKNOWN_PARAMETER,
 *              RTI_ERROR_UNSUPPORTED_ATTRIBUTE, RTI_ERROR_OSAL_NV_OPER_FAILED,
 *              RTI_ERROR_OSAL_NV_ITEM_UNINIT, RTI_ERROR_OSAL_NV_BAD_ITEM_LEN
 *
 **************************************************************************************************/
rStatus_t RTI_ReadItem( uint8 itemId, uint8 len, uint8 *pValue )
{
  npiMsgData_t pMsg;

  // prep Read Item request
  // Note: no need to send pValue over the NPI
  pMsg.subSys   = RPC_SYS_RCAF;
  pMsg.cmdId    = RTIS_CMD_ID_RTI_READ_ITEM;
  pMsg.len      = 2;
  pMsg.pData[0] = itemId;
  pMsg.pData[1] = len;

  // send Read Item request to NP RTIS synchronously
  (NPI_SendSynchDataFnArr[devIdx])( &pMsg );

  // DEBUG
  if ( pMsg.pData[0] == RTI_ERROR_SYNCHRONOUS_NPI_TIMEOUT )
  {
    rtisFatalError( RTI_ERROR_SYNCHRONOUS_NPI_TIMEOUT );
  }

  // copy the reply data to the client's buffer
  // Note: the first byte of the payload is reserved for the status
  msg_memcpy( pValue, &pMsg.pData[1], len );

  // perform endianness change
  rtisAttribEConv( itemId, len, pValue );

  // return the status, which is stored is the first byte of the payload
  return( (rStatus_t)pMsg.pData[0] );
}

/**************************************************************************************************
 *
 * @fn          RTI_WriteItemEx
 *
 * @brief       This API is used to write an item to a Profile's Configuration Interface.
 *
 * input parameters
 *
 * @param       profileId - The Profile identifier.
 * @param       itemId - The Configuration Interface item identifier.
 * @param       len - The length in bytes of the item identifier's data.
 * @param       *pValue - Pointer to buffer where write data is stored.
 *
 * input parameters
 *
 * None.
 *
 * @return      RTI_SUCCESS, RTI_ERROR_NOT_PERMITTED, RTI_ERROR_INVALID_INDEX,
 *              RTI_ERROR_INVALID_PARAMETER, RTI_ERROR_UNKNOWN_PARAMETER,
 *              RTI_ERROR_UNSUPPORTED_ATTRIBUTE, RTI_ERROR_OSAL_NV_OPER_FAILED,
 *              RTI_ERROR_OSAL_NV_ITEM_UNINIT, RTI_ERROR_OSAL_NV_BAD_ITEM_LEN
 *
 **************************************************************************************************/
rStatus_t RTI_WriteItemEx( uint8 profileId, uint8 itemId, uint8 len, uint8 *pValue )
{
  npiMsgData_t pMsg;

  // prep Write Item request
  pMsg.subSys   = RPC_SYS_RCAF;
  pMsg.cmdId    = RTIS_CMD_ID_RTI_WRITE_ITEM_EX;
  pMsg.len      = 3+len;
  pMsg.pData[0] = profileId;
  pMsg.pData[1] = itemId;
  pMsg.pData[2] = len;

  // copy the client's data to be sent
  msg_memcpy( &pMsg.pData[3], pValue, len );

  // perform endianness change
  rtisAttribEConv( itemId, len, &pMsg.pData[3] );

  // send Write Item request to NP RTIS synchronously
  (NPI_SendSynchDataFnArr[devIdx])( &pMsg );

  // DEBUG
  if ( pMsg.pData[0] == RTI_ERROR_SYNCHRONOUS_NPI_TIMEOUT )
  {
    rtisFatalError( RTI_ERROR_SYNCHRONOUS_NPI_TIMEOUT );
  }

  // return the status, which is stored is the first byte of the payload
  return( (rStatus_t)pMsg.pData[0] );
}

/**************************************************************************************************
 *
 * @fn          RTI_WriteItem
 *
 * @brief       This API is used to write RTI Configuration Interface parameters
 *              to the Configuration Parameters table, and permitted attributes
 *              to the State Attributes table.
 *
 * input parameters
 *
 * @param       itemId  - The Configuration Interface item identifier.
 * @param       len - The length in bytes of the item identifier's data.
 * @param       *pValue - Pointer to buffer where write data is stored.
 *
 * input parameters
 *
 * None.
 *
 * @return      RTI_SUCCESS, RTI_ERROR_NOT_PERMITTED, RTI_ERROR_INVALID_INDEX,
 *              RTI_ERROR_INVALID_PARAMETER, RTI_ERROR_UNKNOWN_PARAMETER,
 *              RTI_ERROR_UNSUPPORTED_ATTRIBUTE, RTI_ERROR_OSAL_NV_OPER_FAILED,
 *              RTI_ERROR_OSAL_NV_ITEM_UNINIT, RTI_ERROR_OSAL_NV_BAD_ITEM_LEN
 *
 **************************************************************************************************/
rStatus_t RTI_WriteItem( uint8 itemId, uint8 len, uint8 *pValue )
{
  npiMsgData_t pMsg;

  // prep Write Item request
  pMsg.subSys   = RPC_SYS_RCAF;
  pMsg.cmdId    = RTIS_CMD_ID_RTI_WRITE_ITEM;
  pMsg.len      = 2+len;
  pMsg.pData[0] = itemId;
  pMsg.pData[1] = len;

  // copy the client's data to be sent
  msg_memcpy( &pMsg.pData[2], pValue, len );

  // perform endianness change
  rtisAttribEConv( itemId, len, &pMsg.pData[2] );

  // send Write Item request to NP RTIS synchronously
  (NPI_SendSynchDataFnArr[devIdx])( &pMsg );

  // DEBUG
  if ( pMsg.pData[0] == RTI_ERROR_SYNCHRONOUS_NPI_TIMEOUT )
  {
    rtisFatalError( RTI_ERROR_SYNCHRONOUS_NPI_TIMEOUT );
  }
  // DEBUG, test if RNP not lock in boot mode
  if ( pMsg.subSys == RPC_SYS_BOOT )
  {
    return( 1 );
  }

  // return the status, which is stored is the first byte of the payload
  return( (rStatus_t)pMsg.pData[0] );
}

/**************************************************************************************************
 *
 * @fn          RTI_InitReq
 *
 * @brief       This API is used to initialize the RemoTI stack and begin
 *              network operation. A RemoTI confirmation callback is generated
 *              and handled by the client.
 *
 *              The first thing this function does is take a snapshot of the
 *              Configuration Parameters (CP) table stored in NV memory, and
 *              only the snapshot will be used by RTI until another call is made
 *              to this function (presumably due to a reset). Therefore, any
 *              changes to the CP table must be made prior to calling this
 *              function. Once the RTI is started, subsequent changes by the
 *              client to the CP table can be made, but they will have no affect
 *              on RTI operation. The CP table is stored in NV memory and will
 *              persist across a device reset. The client can restore the
 *              the CP table to its default settings by setting the Startup
 *              Option parameter accordingly.
 *
 *              The client's confirm callback will provide a status, which can
 *              be one of the following:
 *
 *              RTI_SUCCESS
 *              RTI_ERROR_INVALID_PARAMTER
 *              RTI_ERROR_UNSUPPORTED_ATTRIBUTE
 *              RTI_ERROR_INVALID_INDEX
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 *
 **************************************************************************************************/
RTILIB_API void RTI_InitReq( void )
{
  npiMsgData_t pMsg;

  // prep Init request
  pMsg.subSys = RPC_SYS_RCAF;
  pMsg.cmdId  = RTIS_CMD_ID_RTI_INIT_REQ;
  pMsg.len    = 0;

  // send Init request to NP RTIS asynchronously as a confirm is due back
  (NPI_SendAsynchDataFnArr[devIdx])( &pMsg );
}

/**************************************************************************************************
 *
 * @fn          RTI_PairReq
 *
 * @brief       This API is used to initiate a pairing process. Note that this
 *              call actually consists of a discovery followed by pairing. That
 *              is a NLME-DISCOVERY.request followed by NLME-PAIR.request.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 *
 **************************************************************************************************/
RTILIB_API void RTI_PairReq( void )
{
  npiMsgData_t pMsg;

  // prep Pair request
  pMsg.subSys = RPC_SYS_RCAF;
  pMsg.cmdId  = RTIS_CMD_ID_RTI_PAIR_REQ;
  pMsg.len    = 0;

  // send Pair request to NP RTIS asynchronously as a confirm is due back
  (NPI_SendAsynchDataFnArr[devIdx])( &pMsg );
}

/**************************************************************************************************
 *
 * @fn          RTI_PairAbortReq
 *
 * @brief       This API is used to abort an on-going pairing process.
 *
 *              The client's confirm callback will provide a status, which can
 *              be one of the following:
 *
 *              RTI_SUCCESS
 *              RTI_ERROR_PAIR_COMPLETE
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 *
 **************************************************************************************************/
RTILIB_API void RTI_PairAbortReq( void )
{
  npiMsgData_t pMsg;

  // prep Pair request
  pMsg.subSys = RPC_SYS_RCAF;
  pMsg.cmdId  = RTIS_CMD_ID_RTI_PAIR_ABORT_REQ;
  pMsg.len    = 0;

  // send Pair request to NP RTIS asynchronously as a confirm is due back
  (NPI_SendAsynchDataFnArr[devIdx])( &pMsg );
}

/**************************************************************************************************
 *
 * @fn          RTI_AllowPairReq
 *
 * @brief       This function is used by the Target application to ready the
 *              node for a pairing request, and thereby allow this node to
 *              respond.
 *
 *              The client's confirm callback will provide a status, which can
 *              be one of the following:
 *
 *              RTI_SUCCESS
 *              RTI_ERROR_OSAL_NO_TIMER_AVAIL
 *              RTI_ERROR_ALLOW_PAIRING_TIMEOUT
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 *
 **************************************************************************************************/
RTILIB_API void RTI_AllowPairReq( void )
{
  npiMsgData_t pMsg;

  // prep Pair request
  pMsg.subSys = RPC_SYS_RCAF;
  pMsg.cmdId  = RTIS_CMD_ID_RTI_ALLOW_PAIR_REQ;
  pMsg.len    = 0;

  // send Pair request to NP RTIS asynchronously as a confirm is due back
  (NPI_SendAsynchDataFnArr[devIdx])( &pMsg );
}

/**************************************************************************************************
 *
 * @fn          RTI_AllowPairAbortReq
 *
 * @brief       This API is used to attempt to abort an on-going allow-pairing process.
 *
 *              It is possible that allow pair is at a state of no return (no aborting).
 *              There is no callback associated to this function call.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 *
 **************************************************************************************************/
RTILIB_API void RTI_AllowPairAbortReq( void )
{
  npiMsgData_t pMsg;

  // prep Pair request
  pMsg.subSys = RPC_SYS_RCAF;
  pMsg.cmdId  = RTIS_CMD_ID_RTI_ALLOW_PAIR_ABORT_REQ;
  pMsg.len    = 0;

  // send Pair request to NP RTIS asynchronously as a confirm is due back
  (NPI_SendAsynchDataFnArr[devIdx])( &pMsg );
}

/**************************************************************************************************
 *
 * @fn          RTI_UnpairReq
 *
 * @brief       This API is used to trigger un-pairing of a pair entry
 *
 * input parameters
 *
 * @param      dstIndex - destination index
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 *
 **************************************************************************************************/
RTILIB_API void RTI_UnpairReq( uint8 dstIndex )
{
  npiMsgData_t pMsg;

  // prep Pair request
  pMsg.subSys = RPC_SYS_RCAF;
  pMsg.cmdId  = RTIS_CMD_ID_RTI_UNPAIR_REQ;
  pMsg.len    = 1;
  pMsg.pData[0] = dstIndex;

  // send Pair request to NP RTIS asynchronously as a confirm is due back
  (NPI_SendAsynchDataFnArr[devIdx])( &pMsg );
}

/**************************************************************************************************
 *
 * @fn          RTI_SendDataReq
 *
 * @brief       This function sends data to the destination specified by the
 *              pairing table index.
 *
 * input parameters
 *
 * @param       dstIndex  - Pairing table index.
 * @param       profileId - Profile identifier.
 * @param       vendorId  - Vendor identifier.
 * @param       txOptions - Transmission options, as specified in Table 2 of the
 *                          RF4CE specification.
 * @param       len       - Number of bytes to send.
 * @param       *pData    - Pointer to buffer of data to be sent.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 *
 **************************************************************************************************/
RTILIB_API void RTI_SendDataReq( uint8 dstIndex, uint8 profileId, uint16 vendorId, uint8 txOptions, uint8 len, uint8 *pData )
{
  npiMsgData_t pMsg;

  // prep Send Data request
  pMsg.subSys   = RPC_SYS_RCAF;
  pMsg.cmdId    = RTIS_CMD_ID_RTI_SEND_DATA_REQ;
  pMsg.len      = 6+len;
  pMsg.pData[0] = dstIndex;
  pMsg.pData[1] = profileId;
  RTI_SET_ITEM_HALFWORD( &pMsg.pData[2], vendorId );
  pMsg.pData[4] = txOptions;
  pMsg.pData[5] = len;

  // copy the client's data to be sent
  msg_memcpy( &pMsg.pData[6], pData, len );

  // send Send Data request to NP RTIS synchronously
  (NPI_SendAsynchDataFnArr[devIdx])( &pMsg );
}

/**************************************************************************************************
 *
 * @fn          RTI_StandbyReq
 *
 * @brief       This API is used by the Target client to place this node into
 *              standby mode. Th properties of the standby consist of the active
 *              period and the duty cycle. These values are set in the
 *              Configuration Parameters table using the RTI_WriteItemReq API,
 *              and go into effect when standby is enabled for this node.
 *
 * input parameters
 *
 * @param       mode - RTI_STANDBY_ON, RTI_STANDBY_OFF
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 *
 **************************************************************************************************/
RTILIB_API void RTI_StandbyReq( uint8 mode )
{
  npiMsgData_t pMsg;

  // prep Standby request
  pMsg.subSys   = RPC_SYS_RCAF;
  pMsg.cmdId    = RTIS_CMD_ID_RTI_STANDBY_REQ;
  pMsg.len      = 1;
  pMsg.pData[0] = mode;

  // send Standby request to NP RTIS asynchronously as a confirm is due back
  (NPI_SendAsynchDataFnArr[devIdx])( &pMsg );
}

/**************************************************************************************************
 *
 * @fn          RTI_RxEnableReq
 *
 * @brief       This API is used to enable the radio receiver, enable the radio
 *              receiver for a specified amount of time, or disable the radio
 *              receiver.
 *
 * input parameters
 *
 * @param       duration - RTI_RX_ENABLE_ON, RTI_RX_ENABLE_OFF, 1..0xFFFE
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 *
 **************************************************************************************************/
RTILIB_API void RTI_RxEnableReq( uint16 duration )
{
  npiMsgData_t pMsg;

  // prep Rx Enable request
  pMsg.subSys = RPC_SYS_RCAF;
  pMsg.cmdId  = RTIS_CMD_ID_RTI_RX_ENABLE_REQ;
  pMsg.len    = 4;
  RTI_SET_ITEM_WORD( &pMsg.pData[0], (duration & 0x00FFFFFF) ); // max duration is 0x00FF_FFFF

  // send Rx Enable request to NP RTIS asynchronously as a confirm is due back
  (NPI_SendAsynchDataFnArr[devIdx])( &pMsg );
}

/**************************************************************************************************
 *
 * @fn          RTI_EnableSleepReq
 *
 * @brief       This API is used to enable sleep on the target.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 *
 **************************************************************************************************/
RTILIB_API void RTI_EnableSleepReq( void )
{
  npiMsgData_t pMsg;

  // prep Enable Sleep request
  pMsg.subSys = RPC_SYS_RCAF;
  pMsg.cmdId  = RTIS_CMD_ID_RTI_ENABLE_SLEEP_REQ;
  pMsg.len    = 0;

  // send Enable Sleep request to NP RTIS asynchronously
  (NPI_SendAsynchDataFnArr[devIdx])( &pMsg );
}

/**************************************************************************************************
 *
 * @fn          RTI_DisableSleepReq
 *
 * @brief       This API is used to disable sleep on the target.
 *
 *              Note: When used from the RTIS, no actual message is sent to the
 *                    RTI, but wakeup bytes are sent instead. The RTI will
 *                    disable sleep as a result.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 *
 **************************************************************************************************/
RTILIB_API void RTI_DisableSleepReq( void )
{
  npiMsgData_t pMsg;

  // ping NP; ping request will be discarded
  pMsg.subSys   = RPC_SYS_RCAF;
  pMsg.cmdId    = RTIS_CMD_ID_RTI_DISABLE_SLEEP_REQ; //RTIS_CMD_ID_TEST_PING_REQ;
  pMsg.len      = 2;
  pMsg.pData[0] = 0xAA;
  pMsg.pData[1] = 0xCC;

  // send command to slave
  (NPI_SendAsynchDataFnArr[devIdx])( &pMsg );
}

/**************************************************************************************************
 *
 * @fn          RTI_SwResetReq
 *
 * @brief       This function resets the radio processor CPU by way of software triggering.
 *              Implementation of this function is target (CPU) dependent.
 *              Note that in production platform, the reset could be done by chip reset signal
 *              (halResetSlave) and hence use of this function should be restricted to development
 *              phase.
 *
 * input parameters
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 *
 **************************************************************************************************/
RTILIB_API void RTI_SwResetReq( void )
{
  npiMsgData_t pMsg;

  pMsg.subSys   = RPC_SYS_RCAF;
  pMsg.cmdId    = RTIS_CMD_ID_RTI_SW_RESET_REQ;
  pMsg.len      = 0;

  // send command to slave
  (NPI_SendAsynchDataFnArr[devIdx])( &pMsg );

  // wait for 200ms.
  halDelay(200, 1);
}

/**************************************************************************************************
 *
 * @fn          RTI_TestModeReq
 *
 * @brief       This function is used to place the radio in test modes.
 *              Note that implementation is chip dependent. HAL is not used to reduce code
 *              size overhead.
 *
 * input parameters
 *
 * @param       mode - test mode: RTI_TEST_MODE_TX_RAW_CARRIER, RTI_TEST_MODE_TX_RANDOM_DATA
 *                     or RTI_TEST_MODE_RX_AT_FREQ
 * @param       txPower - transmit power as negative dBm value. That is, 20 implies -20dBm.
 * @param       channel - MAC channel number
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 *
 **************************************************************************************************/
RTILIB_API void RTI_TestModeReq( uint8 mode, int8 txPower, uint8 channel )
{
  npiMsgData_t pMsg;

  pMsg.subSys   = RPC_SYS_RCAF;
  pMsg.cmdId    = RTIS_CMD_ID_RTI_TEST_MODE_REQ;
  pMsg.len      = 3;
  pMsg.pData[0] = mode;
  pMsg.pData[1] = (uint8) txPower;
  pMsg.pData[2] = channel;

  // send command to slave
  (NPI_SendAsynchDataFnArr[devIdx])( &pMsg );
}

/**************************************************************************************************
 *
 * @fn          RTI_TestRxCounterGetReq
 *
 * @brief       This function is used to obtain received packet counter value.
 *
 * input parameters
 *
 * @param       resetFlag - whether or not to reset the counter after reading the value
 *
 * output parameters
 *
 * None.
 *
 * @return      counter value
 *
 **************************************************************************************************/
RTILIB_API uint16 RTI_TestRxCounterGetReq(uint8 resetFlag)
{
  npiMsgData_t pMsg;

  // serialize the request
  pMsg.subSys   = RPC_SYS_RCAF;
  pMsg.cmdId    = RTIS_CMD_ID_RTI_RX_COUNTER_GET_REQ;
  pMsg.len      = 1;
  pMsg.pData[0] = resetFlag;

  // send serialized request to NP RTIS synchronously
  (NPI_SendSynchDataFnArr[devIdx])( &pMsg );

  // return the status, which is stored is the first byte of the payload
  return (pMsg.pData[0] + ((uint16)pMsg.pData[1] << 8));
}

/**************************************************************************************************
 * @fn          NPI_AsynchMsgCback
 *
 * @brief       This function is a NPI callback to the client that inidcates an
 *              asynchronous message has been received. The client software is
 *              expected to complete this call.
 *
 *              Note: The client must copy this message if it requires it
 *                    beyond the context of this call.
 *
 * input parameters
 *
 * @param       *pMsg - A pointer to an asychronously received message.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void NPI_AsynchMsgCback( npiMsgData_t *pMsg )
{
  if (rtisState != RTIS_STATE_READY)
  {
    return;
  }

  if (pMsg->subSys == RPC_SYS_RCAF)
  {
    switch( pMsg->cmdId )
    {
    // confirmation to init request
    case RTIS_CMD_ID_RTI_INIT_CNF:
      RTI_InitCnf( (rStatus_t)pMsg->pData[0] );
      break;

    // confirmation to pair request
    case RTIS_CMD_ID_RTI_PAIR_CNF:
      // status, pairing ref table index, pairing table device type
      RTI_PairCnf( (rStatus_t)pMsg->pData[0], pMsg->pData[1], pMsg->pData[2] );
      break;

    // confirmation to pair abort request
    case RTIS_CMD_ID_RTI_PAIR_ABORT_CNF:
      RTI_PairAbortCnf( (rStatus_t) pMsg->pData[0] );
      break;

    // confirmation to allow pair request
    case RTIS_CMD_ID_RTI_ALLOW_PAIR_CNF:
      RTI_AllowPairCnf( (rStatus_t) pMsg->pData[0], pMsg->pData[1], pMsg->pData[2]);
      break;

    // confirmation to send data request
    case RTIS_CMD_ID_RTI_SEND_DATA_CNF:
      RTI_SendDataCnf( (rStatus_t)pMsg->pData[0] );
      break;

    // indication of received data
    case RTIS_CMD_ID_RTI_REC_DATA_IND:
      RTI_ReceiveDataInd( pMsg->pData[0], pMsg->pData[1],
        pMsg->pData[2] | (pMsg->pData[3] << 8), // vendor Id
        pMsg->pData[4],
        pMsg->pData[5],
        pMsg->pData[6],
        &pMsg->pData[7]);
      break;

    case RTIS_CMD_ID_RTI_STANDBY_CNF:
      RTI_StandbyCnf( (rStatus_t) pMsg->pData[0] );
      break;

    // confirmation to send data request
    case RTIS_CMD_ID_RTI_ENABLE_SLEEP_CNF:
      RTI_EnableSleepCnf( (rStatus_t)pMsg->pData[0] );
      break;

    // confirmation to send data request
    case RTIS_CMD_ID_RTI_DISABLE_SLEEP_CNF:
      RTI_DisableSleepCnf( (rStatus_t)pMsg->pData[0] );
      break;

    case RTIS_CMD_ID_RTI_RX_ENABLE_CNF:
      RTI_RxEnableCnf( (rStatus_t ) pMsg->pData[0] );
      break;

    case RTIS_CMD_ID_RTI_UNPAIR_CNF:
      RTI_UnpairCnf( (rStatus_t) pMsg->pData[0],
        pMsg->pData[1] ); // dstIndex
      break;

    case RTIS_CMD_ID_RTI_UNPAIR_IND:
      RTI_UnpairInd( pMsg->pData[0] ); // dstIndex
      break;

    default:
      // nothing can be done here!
      break;
    }
  }
  else if (pMsg->subSys == RPC_SYS_RCN_CLIENT)
  {
#ifdef _WIN32 // TODO: remove this compile flag once RCNS is ported to an app processor
    RCNS_AsynchMsgCback(pMsg);
#endif
  }
}

/**************************************************************************************************
 * @fn          rtisIsBE
 *
 * @brief       This function determines endianness.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      TRUE if the machine is a big-endian machine. FALSE, otherwise.
 **************************************************************************************************
 */
static uint8 rtisIsBE( void )
{
  static const uint16 test = 1;
  uint8 *pTest = (uint8 *) &test;

  return (*pTest == 0);
}

/**************************************************************************************************
 * @fn          rtisAttribEConv
 *
 * @brief       This function converts endianness of an attribute value if the current machine
 *              is a big endian machine.
 *              This function will not do anything if the machine is a little endian machine.
 *
 * input parameters
 *
 * @param       attrib - attribute identifier
 * @param       len    - length of the value buffer
 * @param       pValue - buffer where attribute value is stored
 *
 * output parameters
 *
 * @param       pValue - the buffer is rewritten with swapped endianness value.
 *
 * @return      None.
 **************************************************************************************************
 */
static void rtisAttribEConv( uint8 attrib, uint8 len, uint8 *pValue )
{
  if (rtisBE)
  {
    if (attrib == RTI_SA_ITEM_PT_CURRENT_ENTRY)
    {
      if (len >= sizeof(rcnNwkPairingEntry_t))
      {
        // Note that endianness conversion will not occur if the retrieved attribute length is
        // smaller.
        rcnNwkPairingEntry_t *pEntry = (rcnNwkPairingEntry_t *) pValue;

        pEntry->srcNwkAddress = RTI_ECONV16(pEntry->srcNwkAddress);
        // Note that IEEE address is not converted. It is always supposed to be little endian
        pEntry->panId = RTI_ECONV16(pEntry->panId);
        pEntry->nwkAddress = RTI_ECONV16(pEntry->nwkAddress);
        pEntry->recFrameCounter = RTI_ECONV32(pEntry->recFrameCounter);
        pEntry->vendorIdentifier = RTI_ECONV16(pEntry->vendorIdentifier);
      }
    }
    else
    {
      // all other attributes are natural number
      uint8 i, j, buf;

      for (i = 0, j = len - 1; i < j; i++, j--)
      {
        buf = pValue[i];
        pValue[i] = pValue[j];
        pValue[j] = buf;
      }
    }
  }
}

// -- utility porting --

// These utility functions are called from RTI surrogate module

// -- dummy functions --

// These dummies are called by RTIS module, but they are irrelevant in windows
// environment.
void NPI_Init(void)
{
  // NPI_Init() is a function supposed to be called in OSAL environment
  // Application processor RTIS module calls this function directly
  // to make NPI work for non-OSAL application processor but
  // in case of Windows port of NPI module, this function call is not
  // implemented.
}


/**************************************************************************************************
 **************************************************************************************************/
