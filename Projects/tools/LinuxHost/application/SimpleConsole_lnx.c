/**************************************************************************************************
  Filename:       SimpleConsole_lnx.c

  Description:    Linux SimpleConsole application

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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "rti_lnx.h"

// Linux surrogate interface
#include "rtis_lnx.h"

//#include "hal_rpc.h"
#include "npi_lnx.h"

#ifdef __BIG_DEBUG__
#define debug_printf(fmt, ...) printf( fmt, ##__VA_ARGS__)
#else
uint8 __BIG_DEBUG_ACTIVE = FALSE;
#define debug_printf(fmt, ...) st (if (__BIG_DEBUG_ACTIVE == TRUE) printf( fmt, ##__VA_ARGS__);)
#endif

// Pairing reference
uint8 destIdx;

// Character read from console
uint8 ch;

static void ConfigParam( void );
void appClearPairingTable( void );

int main(int argc, char **argv)
{
	// All configuration items are read from CFG file

	uint8 value[1];


	if (argc > 1)
	{
		if (!RTIS_Init(argv[1]))
		{
			fprintf(stderr, "Failed to start RTI library module\n");
			exit(-1);
		}
	}
	else
	{
		printf("\n-------------------- Please input IP Address and port-------------------\n");
		exit(-1);
	}

	printf("\n-------------------- START Node Configuration-------------------\n");
	ConfigParam();
	printf("-------------------- END Node Configuration-------------------\n");

	printf("\n-------------------- START SOFTWARE VERSION READING-------------------\n");

	if (RTI_SUCCESS != RTI_ReadItem(RTI_CONST_ITEM_SW_VERSION, 1, value))
	{

		fprintf(stderr, "Failed to read Software Version. Please check the connection\n");
		exit(-1);
	}

	printf("- Software Version = 0x%x\n", value[0]);
	printf("-------------------- END SOFTWARE VERSION READING-------------------\n");

	printf("\n-------------------- START Init Req-------------------\n");
	// Initialize node and RF4CE stack
	printf("- Calling RTI_InitReq...\n");
	RTI_InitReq();
	printf("- ...Waiting for RTI_InitCnf...\n");

	// Accept CERC commands until 'q' is pressed
	while ( (ch=getchar() ) != 'q')
	{
		if (ch == 'p')
		{
			// Allow pairing
			printf("Calling RTI_AllowPairReq\n");
			RTI_AllowPairReq();
		}
		else if (ch == 'c')
		{
			// Clear pairing table
			appClearPairingTable();
		}
	}

	RTIS_Close();

	return 0;
}


/**************************************************************************************************
 *
 * @fn      appClearPairingTable
 *
 * @brief   Function to clear pairing table
 *
 * @param   status - Result of RTI_InitReq API call.
 *
 * @return  void
 */
void appClearPairingTable()
{
	rcnNwkPairingEntry_t *pEntry;
	// Allocate memory for one pairing entry
	pEntry = (rcnNwkPairingEntry_t *) malloc(sizeof(rcnNwkPairingEntry_t));

	uint8 i, result, pairingTableSize;

	(void)RTI_ReadItemEx(RTI_PROFILE_RTI, RTI_CONST_ITEM_MAX_PAIRING_TABLE_ENTRIES, 1, (uint8*)&pairingTableSize);

	for (i = 0; i < pairingTableSize; i++)
	{
		// Set current pairing entry
		RTI_WriteItemEx(RTI_PROFILE_RTI, RTI_SA_ITEM_PT_CURRENT_ENTRY_INDEX, 1,
				(uint8 *) &i);
		// Try to read out this entry
		if ((result = RTI_ReadItemEx(RTI_PROFILE_RTI,
				RTI_SA_ITEM_PT_CURRENT_ENTRY,
				sizeof(rcnNwkPairingEntry_t),
				(uint8 *) pEntry)) == RTI_SUCCESS)
		{
			// Invalidate item
			pEntry->pairingRef = RTI_INVALID_PAIRING_REF;
			RTI_WriteItemEx(RTI_PROFILE_RTI,
				RTI_SA_ITEM_PT_CURRENT_ENTRY,
				sizeof(rcnNwkPairingEntry_t),
				(uint8 *) pEntry);
		}
	}

	printf("*************************************\n");
	printf("* Pairing Table Is Empty\n");
	printf("*************************************\n");

	// Free pairing entry buffer
	free(pEntry);
}

/**************************************************************************************************
 *
 * @fn      RTI_InitCnf
 *
 * @brief   RTI confirmation callback initiated by client's RTI_InitReq API
 *          call. The client is expected to complete this function.
 *
 *          NOTE: It is possible that this call can be made to the RTI client
 *                before the call to RTI_InitReq has returned.
 *
 * @param   status - Result of RTI_InitReq API call.
 *
 * @return  void
 */
void RTI_InitCnf(rStatus_t status)
{
	if ( status == RTI_SUCCESS )
	{
		uint8 startupFlg;
		startupFlg = RESTORE_STATE;
		RTI_WriteItem(RTI_CP_ITEM_STARTUP_CTRL, 1, &startupFlg );
		printf("-------------------- END Init Req-------------------\n");
		printf("\n- SUCCESS: RTI_InitCnf called with status %u\n", (unsigned) status);
		printf("\nPress key 'p' then <enter> to allow pairing\n");
		printf("Press key 'c' then <enter> to clear pairing table\n");
	}
	else
	{
		printf("ERROR: RTI_InitCnf called with status %u\n", (unsigned) status);
		exit(-1);
	}
}

/**************************************************************************************************
 *
 * @fn      RTI_PairCnf
 *
 * @brief   RTI confirmation callback initiated by client's RTI_PairReq API
 *          call. The client is expected to complete this function.
 *
 *          NOTE: It is possible that this call can be made to the RTI client
 *                before the call to RTI_PairReq has returned.
 *
 * @param   status - Result of RTI_PairReq API call.
 * @param   dstIndex - Pairing table index of paired device, or invalid.
 * @param   devType  - Pairing table index device type, or invalid.
 * @return  void
 */
void RTI_PairCnf( rStatus_t status, uint8 dstIndex, uint8 devType )
{
}

/**************************************************************************************************
 *
 * @fn      RTI_AllowPairCnf
 *
 * @brief   RTI confirmation callback initiated by client's RTI_AllowPairReq API
 *          call. The client is expected to complete this function.
 *
 *          NOTE: It is possible that this call can be made to the RTI client
 *                before the call to RTI_AllowPairReq has returned.
 * @param   status   - Result of RTI_PairReq API call.
 * @param   dstIndex - Pairing table entry of paired device, or invalid
 * @param   devType  - Pairing table index device type, or invalid
 *
 * @return  void
 */
void RTI_AllowPairCnf( rStatus_t status, uint8 dstIndex, uint8 devType )
{
	// set paring reference (destination index)
	destIdx = dstIndex;

	printf("RTI_AllowpairCnf called with status 0x%.2X\n", (unsigned) status);
	if (0 == status)
		printf("Paired!! Waiting for data from RC, press the <q> key followd by <enter> at any time to quit\n");

}

/**************************************************************************************************
 *
 * @fn      RTI_SendDataCnf
 *
 * @brief   RTI confirmation callback initiated by client's RTI_SendDataReq API
 *          call. The client is expected to complete this function.
 *
 *          NOTE: It is possible that this call can be made to the RTI client
 *                before the call to RTI_SendDataReq has returned.
 *
 * @param   status - Result of RTI_SendDataReq API call.
 *
 * @return  void
 */
void RTI_SendDataCnf( rStatus_t status )
{
}

/**************************************************************************************************
 *
 * @fn      RTI_StandbyCnf
 *
 * @brief   RTI confirmation callback initiated by client's RTI_StandbyReq API
 *          call. The client is expected to complete this function.
 *
 *          NOTE: It is possible that this call can be made to the RTI client
 *                before the call to RTI_RxEnableReq has returned.
 *
 * input parameters
 *
 * @param   status - RTI_SUCCESS
 *                   RTI_ERROR_INVALID_PARAMETER
 *                   RTI_ERROR_UNSUPPORTED_ATTRIBUTE
 *                   RTI_ERROR_INVALID_INDEX
 *                   RTI_ERROR_UNKNOWN_STATUS_RETURNED
 *
 * output parameters
 *
 * None.
 *
 * @return  None.
 */
void RTI_StandbyCnf( rStatus_t status )
{
}

/**************************************************************************************************
 *
 * @fn      RTI_ReceiveDataInd
 *
 * @brief   RTI receive data indication callback asynchronously initiated by
 *          another node. The client is expected to complete this function.
 *
 * input parameters
 *
 * @param   srcIndex:  Pairing table index.
 * @param   profileId: Profile identifier.
 * @param   vendorId:  Vendor identifier.
 * @param   rxLQI:     Link Quality Indication.
 * @param   rxFlags:   Receive flags.
 * @param   len:       Number of bytes to send.
 * @param   *pData:    Pointer to data to be sent.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 */
void RTI_ReceiveDataInd( uint8 srcIndex, uint8 profileId, uint16 vendorId, uint8 rxLQI, uint8 rxFlags, uint8 len, uint8 *pData )
{
	int i = FALSE;

	printf("*************************************************\n");
	printf("RTI_ReceiveDataInd @ \n");
	printf("Source Idx: %d, profileId %d , vendorId: %d , rxLQI %d \n",
			srcIndex,
			profileId,
			vendorId,
			rxLQI);

	printf("Raw Data: ");
	for (i=0; i<len; i++)
		printf("%d ", pData[i]);
	printf("\n");

	printf("*************************************************\n");
}

/**************************************************************************************************
 *
 * @fn      RTI_RxEnableCnf
 *
 * @brief   RTI confirmation callback initiated by client's RTI_RxEnableReq API
 *          call. The client is expected to complete this function.
 *
 * @param   status - Result of RTI_EnableRxReqReq API call.
 *
 * @return  void
 */
void RTI_RxEnableCnf( rStatus_t status )
{
}

/**************************************************************************************************
 *
 * @fn      RTI_EnableSleepCnf
 *
 * @brief   RTI confirmation callback initiated by client's RTI_EnableSleepReq API
 *          call. The client is expected to complete this function.
 *
 * @param   status - Result of RTI_EnableSleepReq API call.
 *
 * @return  void
 *
 */
void RTI_EnableSleepCnf( rStatus_t status )
{
}

/**************************************************************************************************
 *
 * @fn      RTI_DisableSleepCnf
 *
 * @brief   RTI confirmation callback initiated by client's RTI_DisableSleepReq API
 *          call. The client is expected to complete this function.
 *
 * @param   status - Result of RTI_EnableSleepReq API call.
 *
 * @return  void
 *
 */
void RTI_DisableSleepCnf( rStatus_t status )
{
}

/**************************************************************************************************
 *
 * @fn      RTI_UnpairInd
 *
 * @brief   RTI indication callback initiated by receiving unpair request command.
 *
 * @param   dstIndex - Pairing table index of paired device.
 *
 * @return  void
 */
void RTI_UnpairInd( uint8 dstIndex )
{
	// This function is introduced in RemoTI 1.
}

/**************************************************************************************************
 *
 * @fn      RTI_PairAbortCnf
 *
 * @brief   RTI confirmation callback initiated by client's RTI_PairAbortReq API
 *          call. The client is expected to complete this function.
 *
 *          NOTE: It is possible that this call can be made to the RTI client
 *                before the call to RTI_PairAbortReq has returned.
 *
 * @param   status - Result of RTI_PairAbortReq API call.
 * @return  void
 */
void RTI_PairAbortCnf( rStatus_t status )
{
	// This function is introduced in RemoTI 1.
}

/**************************************************************************************************
 *
 * @fn      RTI_UnpairCnf
 *
 * @brief   RTI confirmation callback initiated by client's RTI_UnpairReq API
 *          call. The client is expected to complete this function.
 *
 *          NOTE: It is possible that this call can be made to the RTI client
 *                before the call to RTI_UnpairReq has returned.
 *
 * @param   status   - Result of RTI_PairReq API call.
 * @param   dstIndex - Pairing table index of paired device, or invalid.
 *
 * @return  void
 */
void RTI_UnpairCnf( rStatus_t status, uint8 dstIndex )
{
	// This function is introduced in RemoTI 1.1
}

/**************************************************************************************************
 *
 * @fn      RTI_ResetInd
 *
 * @brief   RTI indication that is used to notify AP that the NP has been reset.
 *
 * @param   void
 *
 * @return  void
 */
void RTI_ResetInd( void )
{
	printf("\nReset detected\n");
	printf("\n-------------------- START Init Req-------------------\n");
	// Initialize node and RF4CE stack
	printf("- Calling RTI_InitReq...\n");
	RTI_InitReq();
}


// List of supported target device types: maximum up to 6 device types.
static const uint8 tgtList[RTI_MAX_NUM_SUPPORTED_TGT_TYPES] =
{
		RTI_DEVICE_REMOTE_CONTROL,
		RTI_DEVICE_RESERVED_INVALID,
		RTI_DEVICE_RESERVED_INVALID,
		RTI_DEVICE_RESERVED_INVALID,
		RTI_DEVICE_RESERVED_INVALID,
		RTI_DEVICE_RESERVED_INVALID
};

// List of implemented device types: maximum up to 3 device types.
static const uint8 devList[RTI_MAX_NUM_DEV_TYPES] =
{
		RTI_DEVICE_TELEVISION,
		RTI_DEVICE_RESERVED_INVALID,
		RTI_DEVICE_RESERVED_INVALID
};

// List of implemented device types: maximum up to 3 device types.
static const uint8 profileList[RTI_MAX_NUM_PROFILE_IDS] =
{
		RTI_PROFILE_ZRC, RTI_PROFILE_ZID, 0, 0, 0, 0, 0
};

static const uint8 vendorName[RTI_VENDOR_STRING_LENGTH] = "TI-LPRF";

void ConfigParam( void )
{
	//  uint8 pValue[MAX_AVAIL_DEVICE_TYPES]; // space for largest number of bytes, not counting strings
	uint8 i;
	//////////////////////////////
	// Configuration Parameters //
	//////////////////////////////

	union { // Space for largest number of bytes, not counting strings.
		uint8 TgtTypes[RTI_MAX_NUM_SUPPORTED_TGT_TYPES];
		uint8 DevList[RTI_MAX_NUM_DEV_TYPES];
		uint8 VendorName[RTI_VENDOR_STRING_LENGTH];
		uint8 ProfileList[RTI_MAX_NUM_PROFILE_IDS];
		uint8 Buf[SADDR_EXT_LEN];
	} u;

	// Target Type; A/C Pwr; Security capable; Channel Normalization capable.
	printf("- Target Configuration\n");
	u.Buf[0] = RTI_BUILD_NODE_CAPABILITIES(1, 1, 1, 1);
	// Be careful, if security deactivated, pairing failed for some Zid device.
	//  u.Buf[0] = RTI_BUILD_NODE_CAPABILITIES(1, 1, 0, 1);
	if (RTI_WriteItemEx(RTI_PROFILE_RTI, RTI_CP_ITEM_NODE_CAPABILITIES, 1, u.Buf) != RTI_SUCCESS)
	{
		//   AP_FATAL_ERROR();
	}

	// Set up configuration parameters that are different from default values
	for (i = 0; i < sizeof(tgtList); i++)
	{
		u.TgtTypes[0] = tgtList[0];
	}
	if (RTI_WriteItemEx(RTI_PROFILE_RTI, RTI_CP_ITEM_NODE_SUPPORTED_TGT_TYPES,
			RTI_MAX_NUM_SUPPORTED_TGT_TYPES, u.TgtTypes) != RTI_SUCCESS)
	{
		///AP_FATAL_ERROR();
	}

	printf("- ZID and ZRC profile activated\n");
	// No User String pairing; 1 Device (Television); 2 Profiles (ZRC & ZID)
	u.Buf[0] = RTI_BUILD_APP_CAPABILITIES(0, 1, 2);

	if (RTI_WriteItemEx(RTI_PROFILE_RTI, RTI_CP_ITEM_APPL_CAPABILITIES, 1, u.Buf) != RTI_SUCCESS)
	{
		// AP_FATAL_ERROR();
	}

	for (i = 0; i < sizeof(devList); i++)
	{
		u.DevList[0] = devList[0];
	}
	if (RTI_WriteItemEx(RTI_PROFILE_RTI, RTI_CP_ITEM_APPL_DEV_TYPE_LIST,
			RTI_MAX_NUM_DEV_TYPES, u.DevList) != RTI_SUCCESS)
	{
		//   AP_FATAL_ERROR();
	}

	for (i = 0; i < sizeof(profileList); i++)
	{
		u.ProfileList[i] = profileList[i];
	}
	if (RTI_WriteItemEx(RTI_PROFILE_RTI, RTI_CP_ITEM_APPL_PROFILE_ID_LIST,
			RTI_MAX_NUM_PROFILE_IDS, u.ProfileList) != RTI_SUCCESS)
	{
		//    AP_FATAL_ERROR();
	}


	*(uint16 *)u.Buf = RTI_VENDOR_TEXAS_INSTRUMENTS;
	if (RTI_WriteItemEx(RTI_PROFILE_RTI, RTI_CP_ITEM_VENDOR_ID, 2, u.Buf) != RTI_SUCCESS)
	{
		//  AP_FATAL_ERROR();
	}

	for (i = 0; i < sizeof(vendorName); i++)
	{
		u.VendorName[i] = vendorName[i];
	}
	if (RTI_WriteItemEx(RTI_PROFILE_RTI, RTI_CP_ITEM_VENDOR_NAME,
			sizeof(vendorName), u.VendorName) != RTI_SUCCESS)
	{
		//  AP_FATAL_ERROR();
	}

}
