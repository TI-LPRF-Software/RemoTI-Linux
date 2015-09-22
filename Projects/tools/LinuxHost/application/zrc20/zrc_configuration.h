   /**************************************************************************************************
  Filename:       zrc_configuration.h

  Description:    This file declares all the configuration APIs that the ZRC
  	  	  	  	  host may use.


  Copyright (C) 2015 Texas Instruments Incorporated - http://www.ti.com/


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

#ifndef ZRC_CFG_H
#define ZRC_CFG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "hal_types.h"

#if !defined PACK_1
#define PACK_1
#endif


#if defined(_MSC_VER) || defined(unix) || (defined(__ICC430__) && (__ICC430__==1))
#pragma pack(1)
#endif

#ifndef ATTR_PACKED
# ifdef __GNUC__
#  define ATTR_PACKED __attribute__ ((__packed__))
# else
#  define ATTR_PACKED
# endif
#endif

#pragma pack(1)

/**********************************************************************************
 * Common defines
 */
#define MAX_NUM_OF_PAIRING_ENTRIES_LINUX_SIDE			10
#define ZRC_MAX_BOUND_REMOTES							 8

#define ZRC_CFG_STORE_NV							 	 0
#define ZRC_CFG_STORE_NV_FORCED							 1

typedef struct ATTR_PACKED ZRC_BoundRemote_s
{
    unsigned char           MacAddress[8];
    unsigned short          ShortAddress;
    unsigned int            CommandCount;
    unsigned char           LastCommand;
    unsigned long           TimestampLastCommand;
    char                    SignalStrength;
    unsigned char           LinkQuality;
    unsigned char           BatteryLevelLoaded;
    unsigned char           BatteryLevelUnloaded;
    unsigned long           TimestampBatteryUpdate;
} ZRC_BoundRemote_t;

typedef struct ATTR_PACKED GDP_ConfigParam_s
{
	uint8 primaryClassDescriptor;
	uint8 secondaryClassDescriptor;
	uint8 tertiaryClassDescriptor;
	uint8 bindingCapabilities;
	uint32 gdpCapabilities;
	uint16 extendedValidationWaitTime;
} GDP_ConfigParam_t;

typedef struct ATTR_PACKED ZRC_ConfigParam_s
{
	uint32 zrcCapabilities;
	uint8 actionBanksSupported[ZRC_ATTR_LEN_ACTION_BANKS_SUPPORTED];

} ZRC_ConfigParam_t;

typedef struct ATTR_PACKED RTI_ConfigParam_s
{
	appDevInfo_t devInfo;
	GDP_ConfigParam_t gdpInfo;
	ZRC_ConfigParam_t zrcInfo;
} RTI_ConfigParam_t;

/////////////////////////////////////////////////////////////////////////////
// Function declarations

void zrcCfgSetCFGParamOnRNP(void);
void zrcCfgInitConfigParam(void);
void zrcCfgPostInitializationConfiguration(void);
void zrcCfgSetTarget(uint8 isTarget);
uint8 zrcCfgIsTarget(void);

uint8 zrcCfgGetIdentificationClientCapabilities();

void zrcCfgUpdateLastKeypress(uint8 index, uint8 lastCommand, unsigned long timeStamp, uint8 rxLQI);
uint8 zrcCfgGetLastKeypressCommand(uint8 index);
unsigned long zrcCfgGetLastKeypressTimeStamp(uint8 index);

void zrcCfgStoreToNV(uint8 condition);
void zrcCfgRestoreFromNV(void);

void zrcCfgPostPairingProcessing(uint8 index);
void zrcCfgClearPairingTable(void);
void zrcCfgDisplayPairingTable(void);
void zrcCfgAddDiscoveredZRCUserstringToLinkedList( uint8 *ieeeAddr, uint8* zrcUserString );
uint8* zrcCfgGetDiscoveredZRCUserstringFromLinkedList( uint8 *ieeeAddr);
void zrcCfgClearDiscoveredZRCUserstringLinkedList();

void zrcCfgUpdateBackupPairingTable(void);
void zrcCfgEnterCLIPDiscovery(void);
void zrcCfgExitCLIPDiscovery(void);

#define NAME_ELEMENT(element) [element] = #element

#ifdef __cplusplus
}
#endif

#endif // MAC_APP_H
