/**************************************************************************************************
 Filename:       sbl.c

 Description:    Linux Host serial boot application

   Copyright (C) {YEAR} Texas Instruments Incorporated - http://www.ti.com/


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

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <getopt.h>
#include <pthread.h>
#include <time.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <stddef.h>
#include <unistd.h>

#include "npi_lnx.h"
#include "npi_boot.h"


// Linux surrogate interface, including both RTIS and NPI
#include "npi_ipc_client.h"
#include "sbl.h"
#include "timer.h"

#include "sb_load.h"

#include "hal_rpc.h"
#include "npi_lnx_ipc_rpc.h"
#include "npi_lnx_error.h"

#include "lprfLogging.h"

// macros
#define PAUSE() { fflush(stdout); while (getchar()!='\n'); }

#define SBL_FLASH_PAGE_SIZE				2048
#define SBL_RNP_VERSION_OFFSET			0x94
#define SBL_RNP_CC2531_VERSION_OFFSET	0x98
#define SBL_RNP_EXTENDED_VERSION_LEN	   8

// Application state variable
uint8 sblState;
struct timeval curTime, startTime, prevTimeSend, prevTimeRec;
struct timeval curTime, prevTime;

// Static variables
uint8 *sblImageBuf = NULL;
static int sblImageLen = 0;
static uint8 isUSBdevice = FALSE;

// Symbols to access binary blob
extern unsigned char _binary_bin_RNP_bin_start;
extern unsigned char _binary_bin_RNP_bin_size;

static char tmpStrForTimePrint[1024];

//THREAD and MUTEX 

void SoftwareVersionToString(char *retStr, int maxStrLen, swVerExtended_t* swVerExtended);
int sbExec(uint8 *pBuf, int length);

#define SB_DST_ADDR_DIV                    4

#define NAME_ELEMENT(element) [element] = #element
const char * const sblState_list[SBL_STATE_READY + 1] = { //18 Application States
		[0 ... SBL_STATE_READY] = NULL,
		NAME_ELEMENT(SBL_STATE_INIT),
		NAME_ELEMENT(SBL_STATE_INIT_SERIAL_BOOT),
		NAME_ELEMENT(SBL_STATE_SERIAL_BOOT),
		NAME_ELEMENT(SBL_STATE_RESET),
		NAME_ELEMENT(SBL_STATE_READY)
};

const char * const serialInterface[8] =
{
		[0 ... 7] = NULL,
		[0] = "UART",
		[1] = "SPI",
		[2] = "I2C",
		[3] = "USB-CDC",
		[4] = "UART w/WakeUp"
};

int SBL_Init(const char *imagePath)
{
	int retVal = NPI_LNX_SUCCESS;
	LOG_INFO("[SBL] Provided path:%s (%d)\n", imagePath, (int)strlen(imagePath));

	if (strlen(imagePath) > 0)
	{
		struct stat fileStat;
		if (stat(imagePath, &fileStat) == 0)
		{
			// Get size of file
			sblImageLen = fileStat.st_size;
			if (sblImageLen > 0)
			{
				// Free existing memory if already allocated
				if (sblImageBuf != NULL)
				{
					free(sblImageBuf);
					sblImageBuf = NULL;
				}
				LOG_INFO("[SBL] Allocating %d bytes for image %s\n", sblImageLen, imagePath);

				// Then allocate memory for the new file
				sblImageBuf = (uint8 *)malloc(fileStat.st_size);
				if (sblImageBuf == NULL)
				{
					LOG_ERROR("[SBL] Allocation of memory for file failed\n");

					// File is empty skip processing it
					retVal = NPI_LNX_FAILURE;
				}
			}
			else
			{
				LOG_ERROR("[SBL] File is empty\n");

				// File is empty skip processing it
				retVal = NPI_LNX_FAILURE;
			}
		}
		else
		{
			LOG_ERROR("[SBL] File doesn't exist\n");

			retVal = NPI_LNX_FAILURE;
		}
		FILE *imageFd = fopen(imagePath, "rb");
		fread(sblImageBuf, 1, sblImageLen, imageFd);
		fclose(imageFd);
	}
	else if ((int)(&_binary_bin_RNP_bin_size) > 0)
	{
		sblImageLen = (int)(&_binary_bin_RNP_bin_size);
		sblImageBuf = (uint8 *)&_binary_bin_RNP_bin_start;
		LOG_INFO("[SBL] RNP firmware image linked in: size %d, location %p\n", sblImageLen, sblImageBuf);
	}
	else
	{
		LOG_WARN("[SBL] No path provided, and no image linked in\n");
		retVal = NPI_LNX_FAILURE;
	}

	return retVal;
}

int SBL_CheckForUpdate(swVerExtended_t *RNPversion)
{
	int retVal = NPI_LNX_FAILURE;
	swVerExtended_t newRNPversion;

	if ((NULL == sblImageBuf) || (sblImageLen <= 0))
	{
		LOG_WARN("[SBL] Skipping update check because no new firmware image has been specified.\n");
	}
	else
	{
		uint8 versionHigher = FALSE;
		uint8 versionLower = FALSE;

		// Find version number in image we want to update to
		if (sblImageLen < (96 * 1024))
		{
			isUSBdevice = FALSE;
			// CC253xF64 or CC253xF96
			memcpy((uint8 *)&newRNPversion, &sblImageBuf[SBL_RNP_VERSION_OFFSET], SBL_RNP_EXTENDED_VERSION_LEN);
		}
		else
		{
			isUSBdevice = TRUE;
			// CC253xF128 or CC253xF256
			memcpy((uint8 *)&newRNPversion, &sblImageBuf[SBL_RNP_CC2531_VERSION_OFFSET], SBL_RNP_EXTENDED_VERSION_LEN);
		}

		LOG_INFO("[SBL] Checking for update (%p, %d)\n", sblImageBuf, sblImageLen);

		uint8 hardwareOkay = TRUE;
#ifndef SKIP_SBL_HARDWARE_VERSION_CHECK
		// First check hardware version for exact match
		if (RNPversion->serial.applies && newRNPversion.serial.applies)
		{
			LOG_INFO("[SBL] [Update Check] Current Serial Interface:\t%s on Port %d\n",
					serialInterface[RNPversion->serial.interface], RNPversion->serial.port);
			LOG_INFO("[SBL] [Update Check] New Serial Interface:\t%s on Port %d\n",
					serialInterface[newRNPversion.serial.interface], newRNPversion.serial.port);

			if (!((RNPversion->serial.interface == newRNPversion.serial.interface) &&
					(RNPversion->serial.port == newRNPversion.serial.port) &&
					(RNPversion->serial.alternative == newRNPversion.serial.alternative)))
			{
				hardwareOkay = FALSE;
				LOG_INFO("[SBL] [Update Check] Serial interface in new image does not match current interface. Cannot update.\n");
				retVal = NPI_LNX_SUCCESS;
			}
		}
		else
		{
			LOG_WARN("[SBL] [Update Check] Cannot check serial interface.\n");
		}
#endif //SKIP_SBL_HARDWARE_VERSION_CHECK
		if (hardwareOkay == TRUE)
		{
			if (!RNPversion)
			{
				LOG_WARN("[SBL] Not comparing extended version info because it's NULL for current firmware, and thus presuming current firmware is quite old.\n");
				versionHigher = TRUE;
			}
			else
			{
				LOG_INFO("[SBL] [Update Check] Current Extended Software Version = %d.%d.%d svn %d\n",
						RNPversion->major, RNPversion->minor, RNPversion->patch, RNPversion->svnRev);

				// Only log all of the extended info if debug logging is enabled.
				SoftwareVersionToString(tmpStrForTimePrint, sizeof(tmpStrForTimePrint), &newRNPversion);
				LOG_DEBUG("[SBL] [Update Check] - FULL Current %s", tmpStrForTimePrint);

				// Check extended version information:
				if (newRNPversion.major > RNPversion->major)
					versionHigher = TRUE;
				else if (newRNPversion.major < RNPversion->major)
					versionLower = TRUE;
				else if (newRNPversion.minor > RNPversion->minor)
					versionHigher = TRUE;
				else if (newRNPversion.minor < RNPversion->minor)
					versionLower = TRUE;
				else if (newRNPversion.patch > RNPversion->patch)
					versionHigher = TRUE;
				else if (newRNPversion.patch < RNPversion->patch)
					versionLower = TRUE;
				else if (newRNPversion.svnRev > RNPversion->svnRev)
					versionHigher = TRUE;
				else if (newRNPversion.svnRev < RNPversion->svnRev)
					versionLower = TRUE;
			}

			if (!versionHigher && !versionLower)
			{
				LOG_INFO("[SBL] [Update Check] Specified version matches current version.  Nothing to update.\n");
				retVal = NPI_LNX_SUCCESS;
			}
			else
			{
				LOG_INFO("[SBL] [Update Check] New Extended Software Version = %d.%d.%d svn %d\n",
						newRNPversion.major, newRNPversion.minor, newRNPversion.patch, newRNPversion.svnRev);

				// Only log all of the extended info if debug logging is enabled.
				SoftwareVersionToString(tmpStrForTimePrint, sizeof(tmpStrForTimePrint), &newRNPversion);
				LOG_DEBUG("[SBL] [Update Check] - FULL New %s", tmpStrForTimePrint);

#ifdef ALLOW_ONLY_HIGHER_VERSIONS
				if (!versionHigher)
				{
					LOG_WARN("[SBL] [Update Check] Specified version is less than current!  Will not update!\n");
				}
				else
#endif
				{
#if 0 // If running old firmware, RNPversion may be legitimately null
					// Then Compare RNP. Here, only interested in hardware
					if (RNPversion == NULL)
					{
						// Hardware must match
						LOG_ERROR("[SBL] [Update Check] - NULL RNPversion\n");
					}
					else
#endif
					{
						// We can download new firmware!
						LOG_INFO("[SBL] Updating firmware...\n");

						retVal = SBL_Execute();
						if (retVal != NPI_LNX_SUCCESS)
						{
							LOG_ERROR("[SBL] Firmware update failed!\n");
						}
						else
						{
							LOG_INFO("[SBL] Firmware update complete.\n");
						}
					}
				}
			}
		}
	}

	return retVal;
}

int SBL_Execute()
{
	int retVal = NPI_LNX_SUCCESS;

	LOG_INFO("[SBL] Executing Serial Bootloader\n");

	if (!sblImageBuf || (sblImageLen <= 0))
	{
		LOG_ERROR("[SBL] No binary file found\n");
		retVal = NPI_LNX_FAILURE;
	}
	else
	{
		int sbResult = 0;
		sblState = SBL_STATE_SERIAL_BOOT;
		sbResult = sbExec(sblImageBuf, sblImageLen);

		if (sbResult != 0)
		{
			npiMsgData_t pMsg;

			LOG_WARN("[SBL] Serial boot loader failed. Attempting hard reset\n");
			// Trying again after a hard reset

			pMsg.len = 0;
			pMsg.subSys = RPC_SYS_SRV_CTRL | RPC_CMD_AREQ;
			pMsg.cmdId = NPI_LNX_CMD_ID_RESET_DEVICE;
			// Send command
			NPI_SendAsynchData( &pMsg );

			// After a very short delay attempt again
			LOG_INFO("[SBL] Send Handshake command\n");

			retVal = BOOT_HandshakeReq();

			if (retVal != NPI_LNX_SUCCESS)
			{
				LOG_FATAL("[SBL] Serial boot loader failed. Please restart application.\n");
			}
			else
			{
				// Try again
				retVal = sbExec(sblImageBuf, sblImageLen);
			}
		}
	}
	return retVal;
}

int SBL_IsDeviceInSBLMode()
{
	int retVal = NPI_LNX_SUCCESS;

	LOG_INFO("[SBL] Checking if the device is in Serial Bootloader\n");

	retVal = BOOT_HandshakeReq();

	if (retVal == SB_NO_RESPOSNE)
	{
		LOG_INFO("[SBL] Device not in SBL mode\n");
	}
	else if (retVal == NPI_LNX_SUCCESS)
	{
		LOG_INFO("[SBL] Device is in SBL mode, status 0x%.2X\n", retVal);
	}

	return retVal;
}

/**************************************************************************************************
 * @fn          sbExec
 *
 * @brief       This function executes the serial boot loading of a binary image.
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
 **************************************************************************************************
 */
int sbExec(uint8 *pBuf, int length)
{
	uint8 returnVal = 0;
	// Create a "blank page" that we can compare with later to find if source is an empty page
	uint8 blankPage[SBL_FLASH_PAGE_SIZE];
	memset(blankPage, 0xFF, sizeof(blankPage));

	uint8* srcAddr = pBuf;

	// The destination address is shifted down by two, so address & length do not exceed uint16.
	uint16 dstAddr = 0; // The embedded boot loader on RNP adds any necessary offset.
	uint16 blkCnt;
	uint32 blkTotal;

	blkCnt = (length / SB_RW_BUF_LEN); //Arbitrary 960 *64 = 61440
	if ((length % SB_RW_BUF_LEN) != 0)
		blkCnt += 1;

	blkTotal = blkCnt;

	LOG_INFO("[SBL] SB exec start., length : %d (%u blocks)\n", length, blkTotal);


	returnVal = BOOT_HandshakeReq();

	if (returnVal != SB_SUCCESS) // Time to process request may require a second handshake attempt.
	{
		LOG_INFO("[SBL] Send Sys reboot command\n");

		// Send command to erase code validation and force boot loader request to NP asynchronously.
		BOOT_BootloadReq();

		if (isUSBdevice == FALSE)
		{
			usleep(50000); // Allow 50ms to reset.
		}
		else
		{
			usleep(1200000); // Allow 1.2 second to reset.
		}

		LOG_INFO("[SBL] Send Handshake command\n");

		returnVal = BOOT_HandshakeReq();
	}

	if (returnVal != SB_SUCCESS) // Time to process request may require a second handshake attempt.
	{
		LOG_INFO("[SBL] Handshake command KO, try again\n");

		returnVal = BOOT_HandshakeReq();
	}

	if (returnVal != SB_SUCCESS)
	{
		LOG_WARN("[SBL] SB start error.\n");

		return NPI_LNX_FAILURE;
	}
	else
	{
		LOG_INFO("[SBL] SB loading...\n");

	}

	int consecutiveFailedReadAttempts = 0, consecutiveFailedWriteAttempts = 0;

	LOG_INFO("[SBL] \n");
	while (blkCnt != 0)
	{
		uint8 bufw[SB_RW_BUF_LEN];
		uint8 bufr[SB_RW_BUF_LEN];
		if (blkCnt % (SBL_FLASH_PAGE_SIZE / SB_RW_BUF_LEN) == 0)
		{
			// LOG_INFO("[SBL] block %d over %d (%d)\n", blkCnt, length / SB_RW_BUF_LEN, (length / SB_RW_BUF_LEN) - blkCnt);
			LOG_INFO("[SBL] Block %u of %u (%u%%)\n", blkTotal - blkCnt, blkTotal, ((blkTotal-blkCnt) * 100) / blkTotal);
		}
		memcpy(bufw, srcAddr, SB_RW_BUF_LEN);

		//This Overwrite is done to force the CRC value to 0xFF.
		// However, the CRC is now calculated without this memory range.
		// Keep it for compatibility reason.
		if (dstAddr == 0x24 /*(0x90 / SB_DST_ADDR_DIV)*/)
		{
			// Setting the enabled word to invalid (which is the erased flash value).
			(void) memset(bufw, 0xFF, 4);
		}

#ifdef __DEBUG_SERIAL_BOOT_LOADER__
		uint8 i;
		char tmpStr[SB_RW_BUF_LEN * 3 + (SB_RW_BUF_LEN/16) + 32];
		size_t strLen = 0;
		for (i = 0; i < SB_RW_BUF_LEN; i++)
		{
			if ( ( (i % 16) == 0 ) && (i > 0) )
			{
				snprintf(tmpStr+strLen, sizeof(tmpStr)-strLen, "\n");
				strLen += 1;
			}
			snprintf(tmpStr+strLen, sizeof(tmpStr)-strLen, "%.2X ", bufw[i]);
			strLen += 3;
		}
		LOG_INFO("[SBL] Writing %d bytes to @ %.4Xd\n%s",
				SB_RW_BUF_LEN,
				(uint16)dstAddr,
				tmpStr);
#endif //__DEBUG_SERIAL_BOOT_LOADER__

		returnVal = BOOT_WriteReq(bufw, SB_RW_BUF_LEN, dstAddr);


		if (returnVal == SB_SUCCESS)
		{
			returnVal = BOOT_ReadReq(bufr, SB_RW_BUF_LEN, &dstAddr);

#ifdef __DEBUG_SERIAL_BOOT_LOADER__
			size_t strLen = 0;
			for (i = 0; i < SB_RW_BUF_LEN; i++)
			{
				if ( ( (i % 16) == 0 ) && (i > 0) )
				{
					snprintf(tmpStr+strLen, sizeof(tmpStr)-strLen, "\n");
					strLen += 1;
				}
				snprintf(tmpStr+strLen, sizeof(tmpStr)-strLen, "%.2X ", bufr[i]);
					strLen += 3;
			}
			LOG_INFO("[SBL] Reading %d bytes from @ %.4Xd\n%s",
					SB_RW_BUF_LEN,
					(uint16)dstAddr,
					tmpStr);
#endif //__DEBUG_SERIAL_BOOT_LOADER__

			if (returnVal == SB_SUCCESS)
			{
				if (memcmp(bufw, bufr , SB_RW_BUF_LEN) == 0)
				{
					if ( (dstAddr % (SBL_FLASH_PAGE_SIZE / SB_DST_ADDR_DIV)) == 0)
					{
						/* Check what the next address is. If it's on the border of a page then check
						 * what the rest of the page is. If the rest of the page is blank, i.e. all 0xFF,
						 * then we don't have to send the rest of the page as it is already erased to all 0xFF.
						 */
						if (memcmp(srcAddr, blankPage , SBL_FLASH_PAGE_SIZE) == 0)
						{
							// This is an empty page. Skip it
							dstAddr += (SBL_FLASH_PAGE_SIZE / SB_DST_ADDR_DIV);
							srcAddr += SBL_FLASH_PAGE_SIZE;
							blkCnt -= (SBL_FLASH_PAGE_SIZE / SB_RW_BUF_LEN);
						}
						else
						{
							dstAddr += (SB_RW_BUF_LEN / SB_DST_ADDR_DIV);
							srcAddr += SB_RW_BUF_LEN;
							blkCnt--;
						}
					}
					else
					{
						dstAddr += (SB_RW_BUF_LEN / SB_DST_ADDR_DIV);
						srcAddr += SB_RW_BUF_LEN;
						blkCnt--;
					}
					consecutiveFailedReadAttempts = 0;
				}
				else
				{
					LOG_WARN("[SBL] KO, compare failed (%d)\n", consecutiveFailedReadAttempts++);

					if (consecutiveFailedReadAttempts > 10)
					{
						break;
					}
				}
			}
			else
			{
				LOG_WARN("[SBL] KO, read failed (%d)\n", consecutiveFailedReadAttempts++);

				if (consecutiveFailedReadAttempts > 10)
				{
					break;
				}
			}
			consecutiveFailedWriteAttempts = 0;
		}
		else
		{
			LOG_ERROR("[SBL] KO, write failed (%d)\n", consecutiveFailedWriteAttempts++);

			if (consecutiveFailedWriteAttempts > 10)
			{
				break;
			}
		}
	}

	if ( (blkCnt == 0) && (returnVal == SB_SUCCESS))
	{
		LOG_INFO("[SBL] Block %u of %u (%u%%)\n", blkTotal - blkCnt, blkTotal, ((blkTotal-blkCnt) * 100) / blkTotal);

		returnVal = BOOT_EnableReq();

		if (returnVal == SB_SUCCESS)
		{
			LOG_INFO("[SBL] SB success!, restart in 1.5 seconds\n");
			usleep(1500000);
			returnVal = 0;
		}
		else
		{
			LOG_ERROR("[SBL] SB fail enable.\n");
			returnVal = -1;
		}
	}
	else
	{
		LOG_ERROR("[SBL] SB failed load.\n");
		returnVal = -1;
	}

	return returnVal;
}

void SoftwareVersionToString(char *retStr, int maxStrLen, swVerExtended_t* swVerExtended)
{
	static char tmpStr[1024];
	sprintf(tmpStr, "Extended Software Version:\n");
	sprintf(tmpStr, "%s\tMajor:\t%d\n", tmpStr, swVerExtended->major);
	sprintf(tmpStr, "%s\tMinor:\t%d\n", tmpStr, swVerExtended->minor);
	sprintf(tmpStr, "%s\tPatch:\t%d\n", tmpStr, swVerExtended->patch);
	sprintf(tmpStr, "%s\tOptional:\t%d\n", tmpStr, swVerExtended->svnRev);
	if (swVerExtended->stack.applies)
	{
		sprintf(tmpStr, "%s\tStack:\n", tmpStr);
		sprintf(tmpStr, "%s\t\tInterface:\t%d\n", tmpStr, swVerExtended->stack.interface);
		sprintf(tmpStr, "%s\t\tNode:\t\t%d\n", tmpStr, swVerExtended->stack.node);
	}
	else
	{
		sprintf(tmpStr, "%s\tStack field doesn't apply (0x%2X)\n", tmpStr,
				((uint8 *) swVerExtended)[offsetof(swVerExtended_t, stack)]);
	}
	if (swVerExtended->profiles.applies)
	{
		sprintf(tmpStr, "%s\tProfiles:\n", tmpStr);
		if (swVerExtended->profiles.zrc11)
		{
			sprintf(tmpStr, "%s\t\t%s\n", tmpStr, "ZRC 1.1");
		}
		if (swVerExtended->profiles.mso)
		{
			sprintf(tmpStr, "%s\t\t%s\n", tmpStr, "MSO");
		}
		if (swVerExtended->profiles.zrc20)
		{
			sprintf(tmpStr, "%s\t\t%s\n", tmpStr, "ZRC 2.0");
		}
	}
	else
	{
		sprintf(tmpStr, "%s\tProfiles field doesn't apply (0x%2X)\n", tmpStr,
				((uint8 *) swVerExtended)[offsetof(swVerExtended_t, profiles)]);
	}
	if (swVerExtended->serial.applies)
	{
		sprintf(tmpStr, "%s\tSerial:\n", tmpStr);
		sprintf(tmpStr, "%s\t\tInterface:\t%d\n", tmpStr, swVerExtended->serial.interface);
		sprintf(tmpStr, "%s\t\tPort:\t\t%d\n", tmpStr, swVerExtended->serial.port);
		sprintf(tmpStr, "%s\t\tAlternative:\t%d\n", tmpStr, swVerExtended->serial.alternative);
	}
	else
	{
		sprintf(tmpStr, "%s\tSerial Interface field doesn't apply (0x%2X)\n",
				tmpStr, ((uint8 *) swVerExtended)[offsetof(swVerExtended_t, serial)]);
	}

	strncpy(retStr, tmpStr, maxStrLen);
}
