/**************************************************************************************************
 Filename:       zrc_configuration.c

 Description:    Linux Host application for ZRC profile target node - Configuration part

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

#include <stdio.h>
#include <stddef.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <getopt.h>
#include <pthread.h>
#include <time.h>
#include <sys/time.h>
#include <unistd.h>


#include "npi_lnx.h"
#include "npi_rti.h"


// Linux surrogate interface, including both RTIS and NPI
#include "npi_ipc_client.h"
#include "zrc_app_main.h"
#include "zrc_app.h"
#include "zrc_configuration.h"
#include "timer.h"
#include "configParser.h"
#include "tiLogging.h"

#include "zrc_app_log_constants.h"

// Profile includes
#include "gdp_profile.h"
#include "zrc.h"
#include "zrc_profile.h"
#include "gdp.h"

#include "hal_rpc.h"
#include "npi_lnx_ipc_rpc.h"
#include "npi_lnx_error.h"

// macros
#define PAUSE() { fflush(stdout); while (getchar()!='\n'); }

/**********************************************************************************
 * Local type defs and other defines
 */
// Pairing reference
uint8 stbDstIndex;

// ch from console
char ch;
// string from console
char str[128];

uint8 zrcCfgMaxNumPairingEntries = 0;

char inIRValidationMode = 0;

// List of supported target device types: maximum up to 6 device types.
static uint8 tgtListCTL[RTI_MAX_NUM_SUPPORTED_TGT_TYPES] = {
        RTI_DEVICE_SET_TOP_BOX, RTI_DEVICE_TELEVISION,
        RTI_DEVICE_MEDIA_CENTER_PC, RTI_DEVICE_RESERVED_INVALID,
        RTI_DEVICE_RESERVED_INVALID, RTI_DEVICE_RESERVED_INVALID };
// List of supported target device types: maximum up to 6 device types.
static uint8 tgtListTGT[RTI_MAX_NUM_SUPPORTED_TGT_TYPES] = {
        RTI_DEVICE_REMOTE_CONTROL, RTI_DEVICE_RESERVED_INVALID,
        RTI_DEVICE_RESERVED_INVALID, RTI_DEVICE_RESERVED_INVALID,
        RTI_DEVICE_RESERVED_INVALID, RTI_DEVICE_RESERVED_INVALID };

// List of implemented device types: maximum up to 3 device types.
static uint8 devListCTL[RTI_MAX_NUM_DEV_TYPES] = { RTI_DEVICE_REMOTE_CONTROL,
        RTI_DEVICE_RESERVED_INVALID, RTI_DEVICE_RESERVED_INVALID };
// List of implemented device types: maximum up to 3 device types.
static uint8 devListTGT[RTI_MAX_NUM_DEV_TYPES] = { RTI_DEVICE_SET_TOP_BOX,
        RTI_DEVICE_TELEVISION, RTI_DEVICE_RESERVED_INVALID };

// List of implemented device types: maximum up to 3 device types.
static uint8 profileList[RTI_MAX_NUM_PROFILE_IDS] = { RTI_PROFILE_ZRC,
        RTI_PROFILE_ZRC20, 0, 0, 0, 0, 0 };
static uint8 vendorName[RTI_VENDOR_STRING_LENGTH] = "TI-LPRF";

static uint8 zrcCfgActionBanksSupported[ZRC_ATTR_LEN_ACTION_BANKS_SUPPORTED] =
{
  0x01, 0x00, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00, // HDMI-CEC, HID Keyboard, HID Consumer
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, // since HA Actions Recipient, indicate all instances supported
  0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, // Vendor specific HID action bank
};

static uint8 zrcCfgCercActionCodesSupported[] =
{
  0x1f, 0x22, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00,
  0x06, 0x00, 0x00, 0x00, 0x00, 0x38, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static uint8 zrcCfgHidKeyboardActionCodesSupported[] =
{
  0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static uint8 zrcCfgHidConsumerPageSectionAActionCodesSupported[] =
{
  0x00, 0xEE, 0xDD, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

//static uint8 zrcCfgHaDirtyFlags[] =
//{
//  0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
//  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
//};

static uint8 zrcCfgKeyMappingDefault[] =
{
  // mapping flags
  ZRC_FLAGS_USE_DEFAULT_MASK
};

static uint8 zrcCfgKeyMappingIRTest[] =
{
  // mapping flags
  ZRC_FLAGS_IR_SPECIFIED_MASK,

  // IR Descriptor
  0x00, // IR config
  0x34, // IR code length

  0x03, 0x00,
  0x06, // # of Timing Definitions
  0x22, // # of symbols in pressed frame
  0x02, // # of symbols in repeat frame
  0x00, // # of symbols in release frame
  0x00, // # of symbols in toggle sequences
  0x00, // # of toggle sequences in toggle definitions
  0xD3, 0x00, // Carrier period
  0xDC, 0x08, 0x5E, 0x04,
  0x91, 0x00, 0x91, 0x00,
  0x91, 0x00, 0x9D, 0x01,
  0x91, 0x00, 0xC4, 0x27,
  0xDC, 0x08, 0x2B, 0x02,
  0x91, 0x00, 0x5E, 0x5E,
  0x01, 0x12, 0x11, 0x11, 0x12, 0x21, 0x22, 0x22, 0x21, 0x11, 0x21, 0x11, 0x12, 0x22, 0x12, 0x22, 0x23, // Pressed frame
  0x45 // Repeat frame
};

static uint8 zrcCfgKeyMappingPower[] =
{
  // mapping flags
  ZRC_FLAGS_IR_SPECIFIED_MASK,

  // IR Descriptor
  0x00, // IR config
  0x34, // IR code length

  0x03, 0x00,
  0x06, // # of Timing Definitions
  0x22, // # of symbols in pressed frame
  0x02, // # of symbols in repeat frame
  0x00, // # of symbols in release frame
  0x00, // # of symbols in toggle sequences
  0x00, // # of toggle sequences in toggle definitions
  0xD3, 0x00, // Carrier period
  0xDC, 0x08, 0x5E, 0x04,
  0x91, 0x00, 0x91, 0x00,
  0x91, 0x00, 0x9D, 0x01,
  0x91, 0x00, 0xC4, 0x27,
  0xDC, 0x08, 0x2B, 0x02,
  0x91, 0x00, 0x5E, 0x5E,
  0x01, 0x12, 0x11, 0x11, 0x12, 0x21, 0x22, 0x22, 0x21, 0x11, 0x21, 0x11, 0x12, 0x22, 0x12, 0x22, 0x23, // Pressed frame
  0x45 // Repeat frame
};

//static uint8 zrcCfgKeyMappingSelect[] =
//{
//  // mapping flags
//  ZRC_FLAGS_RF_SPECIFIED_MASK,
//
//  // RF descriptor
//  0x41, // Atomic Action with 1 transmission
//  RTI_TX_OPTION_ACKNOWLEDGED | RTI_TX_OPTION_SECURITY,
//  3, // action data length
//  0x00,
//  0x00,
//  0x42
//};
//
//static uint8 zrcCfgKeyMappingUp[] =
//{
//  // mapping flags
//  ZRC_FLAGS_RF_SPECIFIED_MASK | ZRC_FLAGS_IR_SPECIFIED_MASK | ZRC_FLAGS_RF_DESCRIPTOR_FIRST_MASK,
//
//  // RF descriptor
//  0x41, // Atomic Action with 1 transmission
//  RTI_TX_OPTION_ACKNOWLEDGED | RTI_TX_OPTION_SECURITY,
//  3, // action data length
//  0x00,
//  0x00,
//  0x40,
//
//  // IR Descriptor
//  0x00, // IR config
//  0x0a, // IR code length
//  0xa0,
//  0xa1,
//  0xa2,
//  0xa3,
//  0xa4,
//  0xa5,
//  0xa6,
//  0xa7,
//  0xa8,
//  0xa9
//};
//
//static uint8 zrcCfgKeyMappingDown[] =
//{
//  // mapping flags
//  ZRC_FLAGS_RF_SPECIFIED_MASK,
//
//  // RF descriptor
//  0x41, // Atomic Action with 1 transmission
//  RTI_TX_OPTION_ACKNOWLEDGED | RTI_TX_OPTION_SECURITY,
//  3, // action data length
//  0x00,
//  0x00,
//  0x41
//};
//
//static uint8 zrcCfgKeyMappingLeft[] =
//{
//  // mapping flags
//  ZRC_FLAGS_IR_SPECIFIED_MASK,
//
//  // IR Descriptor
//  0x00, // IR config
//  0x0a, // IR code length
//  0xb0,
//  0xb1,
//  0xb2,
//  0xb3,
//  0xb4,
//  0xb5,
//  0xb6,
//  0xb7,
//  0xb8,
//  0xb9
//};

//static uint8 *zrcCfgActionMappingTables[] =
//{
//  zrcCfgKeyMappingUp,
//  zrcCfgKeyMappingDown,
//  zrcCfgKeyMappingLeft
//};

/* Table of mapped functions */
typedef struct zrcCfgActionMappingIndextoCmdTableEntry {
    uint8  dataLength;
    uint8 *data;
} ZrcCfgActionMappingIndextoCmdTableEntry;

static ZrcCfgActionMappingIndextoCmdTableEntry zrcCfgActionMappingIndexToCmdTable[256] = {{0}};

/* ------------------------------------------------------------------------------------------------
 *                                           Global Variables
 * ------------------------------------------------------------------------------------------------
 */
static uint8 zrcCfgIdentificationClientCapabilities;
static gdpPollConfigAttr_t zrcCfgPollConfiguration;
static uint8 zrcCfgOnOff[] =
{
  0x01
};
static uint16 zrcCfgPollingTimeInterval = 2000;

// Define structure to hold diagnostics for the bound originators
ZRC_BoundRemote_t boundRemotes[RCN_CAP_PAIR_TABLE_SIZE];

/**********************************************************************************
 * Common variables
 */
response_t responseBuf;    // Used for Set/Get Attributes Response

/***********************************************************************************
 * Configuration parameters
 */
RTI_ConfigParam_t zrcCfgCFGParam;
static uint32 zrcCfgMinPeerGdpCapabilities = 0;
static uint32 zrcCfgMinPeerZrcCapabilities = 0;

uint8 zrcCfgTarget = TRUE;

/***************************************************************
 * Local function declarations
 */
static void zrcCfgSetupUserString(void);
static void zrcCfgGetCFGParamFromRNP(void);
uint8 zrcCfgGetOldestEntryIndex(void);
void zrcCfgDisplayPairingTable(void);
void DisplayPairingTable(rcnNwkPairingEntry_t *pEntry, ZRC_BoundRemote_t *rfStat);
void DispCFGCurrentCfg(RTI_ConfigParam_t *appCfg, uint16 shortAddr, uint16 panID);
void zrcRIBInit( void );

uint8 i;
void zrcCfgSetTarget(uint8 isTarget)
{
    zrcCfgTarget = isTarget;
}

uint8 zrcCfgIsTarget()
{
    return zrcCfgTarget;
}

void zrcCfgPostInitializationConfiguration()
{
    // Get configuration parameters from RNP, to make sure we display the correct settings
    zrcCfgGetCFGParamFromRNP();

    // Get max number of pairing entries from constant field on RNP
    RTI_ReadItemEx(RTI_PROFILE_RTI, RTI_CONST_ITEM_MAX_PAIRING_TABLE_ENTRIES, 1,&zrcCfgMaxNumPairingEntries);

    // User string must be setup after configuration
    zrcCfgSetupUserString();
}

static void zrcCfgSetupUserString()
{
//    // Setup User String
//    uint8 i;
//    memset(zrcCfgCFGParam.devInfo.devInfo.userString, 0, sizeof(zrcCfgCFGParam.devInfo.devInfo.userString));
//    for (i = 0; i < sizeof(zrcUserString); i++)
//    {
//        zrcCfgCFGParam.devInfo.devInfo.userString[i] = zrcUserString[i];
//    }
//    if (RTI_WriteItemEx(RTI_PROFILE_RTI, RTI_SA_ITEM_USER_STRING,
//            sizeof(zrcCfgCFGParam.devInfo.userString), zrcCfgCFGParam.devInfo.userString) != RTI_SUCCESS)
//    {
//        //  AP_FATAL_ERROR();
//        LOG_WARN("[Configuration] Could not write RTI_SA_ITEM_USER_STRING\n");
//    }
//    else
//    {
//        LOG_DEBUG("[Configuration] Successfully wrote RTI_SA_ITEM_USER_STRING\n");
//    }
}

uint8 zrcCfgGetIdentificationClientCapabilities()
{
    return zrcCfgIdentificationClientCapabilities;
}

void zrcCfgUpdateLastKeypress(uint8 index, uint8 lastCommand, unsigned long timeStamp, uint8 rxLQI)
{
    boundRemotes[index].CommandCount++;
    boundRemotes[index].LastCommand = lastCommand;
    boundRemotes[index].TimestampLastCommand = timeStamp;
    boundRemotes[index].LinkQuality = rxLQI;
    boundRemotes[index].SignalStrength = ZRC_APP_LQI_TO_DBM_CONVERSION(rxLQI);
}

uint8 zrcCfgGetLastKeypressCommand(uint8 index)
{
    return boundRemotes[index].LastCommand;
}

unsigned long zrcCfgGetLastKeypressTimeStamp(uint8 index)
{
    return boundRemotes[index].TimestampLastCommand;
}

/**************************************************************************************************
 *
 * @fn      zrcCfgInitConfigParam
 *
 * @brief   Function to initialize the configuration structure zrcCfgCFGParam
 *
 * @param   void
 *
 *
 * @return  void
 * */
void zrcCfgInitConfigParam()
{
    int i;

    LOG_DEBUG("[Configuration] ZRC profile activated\n");
    // No User String pairing; 1 Device (Remote Controller or STB); 2 Profiles (ZRC1.1 and ZRC2.0)
    zrcCfgCFGParam.devInfo.appCapabilities = RTI_BUILD_APP_CAPABILITIES(1, 1, 2);

    if (FALSE == zrcCfgIsTarget()) /* Controller */
    {
        // Controller Type; Battery Powered; Security capable; Channel Normalization capable.
        LOG_DEBUG("[Configuration] Controller Configuration\n");
        zrcCfgCFGParam.devInfo.nodeCapabilities = RTI_BUILD_NODE_CAPABILITIES(0, 0, 1, 1);

        // Set up configuration parameters that are different from default values
        for (i = 0; i < sizeof(tgtListCTL); i++) {
            zrcCfgCFGParam.devInfo.tgtTypeList[i] = tgtListCTL[i];
        }

        for (i = 0; i < sizeof(devListCTL); i++) {
            zrcCfgCFGParam.devInfo.devTypeList[i] = devListCTL[i];
        }

        for (i = 0; i < sizeof(profileList); i++) {
            zrcCfgCFGParam.devInfo.profileIdList[i] = profileList[i];
        }

        zrcCfgCFGParam.devInfo.vendorId = RTI_VENDOR_TEXAS_INSTRUMENTS;

        for (i = 0; i < sizeof(vendorName); i++) {
            zrcCfgCFGParam.devInfo.vendorString[i] = vendorName[i];
        }
    }
    else /* Target */
    {
        // Target Type; A/C Pwr; Security capable; Channel Normalization capable.
//        debug_
        LOG_INFO("[Configuration] Target Configuration\n");
        zrcCfgCFGParam.devInfo.nodeCapabilities = RTI_BUILD_NODE_CAPABILITIES(1, 1, 1, 1);

        // Set up configuration parameters that are different from default values
        for (i = 0; i < sizeof(tgtListTGT); i++) {
            zrcCfgCFGParam.devInfo.tgtTypeList[i] = tgtListTGT[i];
        }

        for (i = 0; i < sizeof(devListTGT); i++) {
            zrcCfgCFGParam.devInfo.devTypeList[i] = devListTGT[i];
        }

        for (i = 0; i < sizeof(profileList); i++) {
            zrcCfgCFGParam.devInfo.profileIdList[i] = profileList[i];
        }

        zrcCfgCFGParam.devInfo.vendorId = RTI_VENDOR_TEXAS_INSTRUMENTS;

        for (i = 0; i < sizeof(vendorName); i++) {
            zrcCfgCFGParam.devInfo.vendorString[i] = vendorName[i];
        }

        // ZRC20 Parameters
        memcpy(zrcCfgCFGParam.zrcInfo.actionBanksSupported, zrcCfgActionBanksSupported, ZRC_ATTR_LEN_ACTION_BANKS_SUPPORTED);

        // ZRC Capabilities
        zrcCfgCFGParam.zrcInfo.zrcCapabilities = ZRC_CAP_SUPPORT_ACTIONS_RECIPIENT_MASK |
                                                 ZRC_CAP_INFORM_ABOUT_SUPPORTED_ACTIONS_MASK |
                                                 ZRC_CAP_SUPPORT_HA_ACTIONS_RECIPIENT_MASK |
                                                 ZRC_CAP_SUPPORT_ACTION_MAPPING_SERVER_MASK;
        zrcCfgMinPeerZrcCapabilities =  GDP_CAP_SUPPORT_POLL_CLIENT_MASK |
                                        GDP_CAP_SUPPORT_IDENTIFICATION_CLIENT_MASK |
                                        GDP_CAP_SUPPORT_ENHANCED_SECURITY_MASK;

        // GDP Parameters
        zrcCfgCFGParam.gdpInfo.primaryClassDescriptor = GDP_CLASS_DESC_BUILD_DESCRIPTOR( 4, GDP_DUP_CLASS_HDL_USE_AS_IS );

        zrcCfgCFGParam.gdpInfo.secondaryClassDescriptor = GDP_CLASS_DESC_BUILD_DESCRIPTOR( 5, GDP_DUP_CLASS_HDL_ABORT );

        zrcCfgCFGParam.gdpInfo.tertiaryClassDescriptor = GDP_CLASS_DESC_BUILD_DESCRIPTOR( 6, GDP_DUP_CLASS_HDL_ABORT );

        // recipient binding type used
        zrcCfgCFGParam.gdpInfo.bindingCapabilities = GDP_BINDING_CAP_TYPE_PUSH_BUTTON_AND_VALIDATION;
//        zrcCfgCFGParam.gdpInfo.bindingCapabilities = GDP_BINDING_CAP_TYPE_PUSH_BUTTON;

        // GDP Capabilities
        zrcCfgCFGParam.gdpInfo.gdpCapabilities = GDP_CAP_SUPPORT_POLL_SERVER_MASK |
                GDP_CAP_SUPPORT_IDENTIFICATION_SERVER_MASK |
                GDP_CAP_SUPPORT_ENHANCED_SECURITY_MASK;
        zrcCfgMinPeerGdpCapabilities = GDP_CAP_SUPPORT_POLL_CLIENT_MASK |
                GDP_CAP_SUPPORT_IDENTIFICATION_CLIENT_MASK |
                GDP_CAP_SUPPORT_ENHANCED_SECURITY_MASK;

        zrcCfgCFGParam.gdpInfo.extendedValidationWaitTime = 64000;
    }
}

///**************************************************************************************************
// *
// * @fn      zrcCfgUpdateRFstatistics
// *
// * @brief   Function called upon initialization to align pairing table on RNP and structure stored
// *             in SOC NV.
// *
// * @param   void
// *
// *
// * @return  void
// * */
//static void zrcCfgUpdateRFstatistics(void)
//{
//    // Look for the structure stored in SOC NV. Then align this with the actual pairing table on
//    // the RNP
//    int debugState = __DEBUG_APP_ACTIVE;
//    __DEBUG_APP_ACTIVE = FALSE;
//    // First verify that path exists
//    if (persistentPath && shadowPath)
//    {
//        // Then verify that we can open the file
//        FILE *nvFileFd, *shadowFileFd;
//
//        nvFileFd = fopen(persistentPath, "r");
//        shadowFileFd = fopen(shadowPath, "w");
//
//        if (nvFileFd && shadowFileFd)
//        {
//            // The file exists so we begin operation on it after we close the file
//            fclose(nvFileFd);
//            fclose(shadowFileFd);
//            // Then get all info
//            char *strBuf, *pStrBufRoot;
//            strBuf = (char*) malloc(1024);
//            memset(strBuf, 0, 1024);
//            pStrBufRoot = strBuf;
//            LOG_DEBUG"[ZRC NV][INFO] Writing RF statistics\n");
//            {
//                LOG_DEBUG("[ZRC NV][INFO] Section %s\n", i, searchSectionStrList[i]);
//                {
//                    // Copy value from rfStatus structure
//                    zrcCfgFromByteArrayToAsciiArray(strBuf, (uint8*)&rfStatus.lowUptime, searchKeysRFStatsTgtStrList[LowUpTime].byteCount);
//                    // Add NULL character to terminate string
//                    strBuf[searchKeysRFStatsTgtStrList[LowUpTime].byteCount << 1] = '\0';
//
//                    // Update NV file
//                    ConfigParserSet(persistentPath, shadowPath, ZRC_APP_SEARCH_SECTION_RF_STATIS_TGT, searchKeysRFStatsTgtStrList[LowUpTime].str, strBuf);
//
//                    LOG_DEBUG"[ZRC NV][INFO]\t Writing key %s \t value = %s\n",
//                            searchKeysRFStatsTgtStrList[LowUpTime].str,
//                            strBuf);
//                }
//            }
//            free(pStrBufRoot);
//
//        }
//        else
//        {
//            LOG_ERROR("[ZRC NV][ERROR] Could not open NV file\n");
//        }
//    }
//
//    // Take software version number from RAM, as it was already captured during initialization
//    memcpy(rfStatus.versionInfo, (uint8 *)&ZRC_AppRNP_ver.ver, sizeof(rfStatus.versionInfo));
//
//    // Get currently active channel from RNP
//    if (RTI_SUCCESS != RTI_ReadItemEx(RTI_PROFILE_RTI, RTI_SA_ITEM_CURRENT_CHANNEL, 1, (uint8 *) &rfStatus.activeChannel.Number))
//    {
//        LOG_WARN("[Configuration] Failed to read active channel\n");
//    }
//    else
//    {
//        if (rfStatus.activeChannel.Number == 15)
//        {
//            rfStatus.backupChannels[0].Number = 20;
//            rfStatus.backupChannels[1].Number = 25;
//        }
//        else if (rfStatus.activeChannel.Number == 20)
//        {
//            rfStatus.backupChannels[0].Number = 15;
//            rfStatus.backupChannels[1].Number = 25;
//        }
//        else
//        {
//            rfStatus.backupChannels[0].Number = 15;
//            rfStatus.backupChannels[1].Number = 20;
//        }
//    }
//
//    // Get IEEE address from RNP
//    if (RTI_SUCCESS != RTI_ReadItemEx(RTI_PROFILE_RTI, RTI_SA_ITEM_IEEE_ADDRESS, sizeof(rfStatus.macAddress), (uint8 *) rfStatus.macAddress))
//    {
//        LOG_WARN("[Configuration] Failed to read IEEE address\n");
//    }
//
//    // Get Short Address from RNP
//    if (RTI_SUCCESS != RTI_ReadItemEx(RTI_PROFILE_RTI, RTI_SA_ITEM_SHORT_ADDRESS, sizeof(rfStatus.shortAddress), (uint8 *) &rfStatus.shortAddress))
//    {
//        LOG_WARN("[Configuration] Failed to read short address\n");
//    }
//
//    // Get Pan ID from RNP
//    if (RTI_SUCCESS != RTI_ReadItemEx(RTI_PROFILE_RTI, RTI_SA_ITEM_PAN_ID, sizeof(rfStatus.panId), (uint8 *) &rfStatus.panId))
//    {
//        LOG_WARN("[Configuration] Failed to read Pan ID\n");
//    }
//
//    // Get number of active entries from RNP
//    if (RTI_SUCCESS != RTI_ReadItemEx(RTI_PROFILE_RTI, RTI_SA_ITEM_PT_NUMBER_OF_ACTIVE_ENTRIES, 1, (uint8 *) &rfStatus.nbrBindRemotes))
//    {
//        LOG_WARN("[Configuration] Failed to read number of active pairing entries\n");
//    }
//
//    __DEBUG_APP_ACTIVE = debugState;
//}

void zrcCfgSetCFGParamOnRNP(void)
{
    ////////////////////////////////////
    // Apply Configuration Parameters //
    ////////////////////////////////////

    LOG_INFO("[Configuration] -------------------- SET RNP CONFIGURATION PARAMETERS-------------------\n");
    if (RTI_WriteItemEx(RTI_PROFILE_RTI, RTI_CP_ITEM_NODE_CAPABILITIES, 1,
            (uint8*) &(zrcCfgCFGParam.devInfo.nodeCapabilities)) != RTI_SUCCESS) {
        //   AP_FATAL_ERROR();
        LOG_WARN("[Configuration] Could not write RTI_CP_ITEM_NODE_CAPABILITIES\n");
    } else
        LOG_DEBUG("[Configuration] Successfully wrote RTI_CP_ITEM_NODE_CAPABILITIES\n");

    if (RTI_WriteItemEx(RTI_PROFILE_RTI, RTI_CP_ITEM_NODE_SUPPORTED_TGT_TYPES,
            RTI_MAX_NUM_SUPPORTED_TGT_TYPES,
            zrcCfgCFGParam.devInfo.tgtTypeList) != RTI_SUCCESS) {
        ///AP_FATAL_ERROR();
        LOG_WARN("[Configuration] Could not write RTI_CP_ITEM_NODE_SUPPORTED_TGT_TYPES\n");
    } else
        LOG_DEBUG("[Configuration] Successfully wrote RTI_CP_ITEM_NODE_SUPPORTED_TGT_TYPES\n");

    if (RTI_WriteItemEx(RTI_PROFILE_RTI, RTI_CP_ITEM_APPL_CAPABILITIES, 1,
            (uint8*) &(zrcCfgCFGParam.devInfo.appCapabilities)) != RTI_SUCCESS) {
        // AP_FATAL_ERROR();
        LOG_WARN("[Configuration] Could not write RTI_CP_ITEM_APPL_CAPABILITIES\n");
    } else
        LOG_DEBUG("[Configuration] Successfully wrote RTI_CP_ITEM_APPL_CAPABILITIES\n");

    if (RTI_WriteItemEx(RTI_PROFILE_RTI, RTI_CP_ITEM_APPL_DEV_TYPE_LIST,
            RTI_MAX_NUM_DEV_TYPES, zrcCfgCFGParam.devInfo.devTypeList) != RTI_SUCCESS) {
        //   AP_FATAL_ERROR();
        LOG_WARN("[Configuration] Could not write RTI_CP_ITEM_APPL_DEV_TYPE_LIST\n");
    } else
        LOG_DEBUG("[Configuration] Successfully wrote RTI_CP_ITEM_APPL_DEV_TYPE_LIST\n");

    if (RTI_WriteItemEx(RTI_PROFILE_RTI, RTI_CP_ITEM_APPL_PROFILE_ID_LIST,
            RTI_MAX_NUM_PROFILE_IDS, zrcCfgCFGParam.devInfo.profileIdList) != RTI_SUCCESS) {
        //    AP_FATAL_ERROR();
        LOG_WARN("[Configuration] Could not write RTI_CP_ITEM_APPL_PROFILE_ID_LIST\n");
    } else
        LOG_DEBUG("[Configuration] Successfully wrote RTI_CP_ITEM_APPL_PROFILE_ID_LIST\n");

    if (RTI_WriteItemEx(RTI_PROFILE_RTI, RTI_CP_ITEM_VENDOR_ID, 2,
            (uint8*) &(zrcCfgCFGParam.devInfo.vendorId)) != RTI_SUCCESS) {
        //  AP_FATAL_ERROR();
        LOG_WARN("[Configuration] Could not write RTI_CP_ITEM_VENDOR_ID\n");
    } else
        LOG_DEBUG("[Configuration] Successfully wrote RTI_CP_ITEM_VENDOR_ID\n");

    if (RTI_WriteItemEx(RTI_PROFILE_RTI, RTI_CP_ITEM_VENDOR_NAME,
            sizeof(vendorName), zrcCfgCFGParam.devInfo.vendorString) != RTI_SUCCESS) {
        //  AP_FATAL_ERROR();
        LOG_WARN("[Configuration] Could not write RTI_CP_ITEM_VENDOR_NAME\n");
    } else
        LOG_DEBUG("[Configuration] Successfully wrote RTI_CP_ITEM_VENDOR_NAME\n");

    if (RTI_WriteItemEx( RTI_PROFILE_ZRC20,
                       ZRC_ITEM_ZRC_PROFILE_CAPABILITIES,
                       sizeof( zrcCfgCFGParam.zrcInfo.zrcCapabilities ),
                       (uint8 *)&zrcCfgCFGParam.zrcInfo.zrcCapabilities ) != RTI_SUCCESS)
    {
        //  AP_FATAL_ERROR();
        LOG_WARN("[Configuration] Could not write ZRC_ITEM_ZRC_PROFILE_CAPABILITIES\n");
    }
    else
    {
        LOG_DEBUG("[Configuration] Successfully wrote ZRC_ITEM_ZRC_PROFILE_CAPABILITIES\n");
    }

    // Action Banks supported
    if (RTI_WriteItemEx( RTI_PROFILE_ZRC20,
            ZRC_ITEM_ACTION_BANKS_SUPPORTED_RX,
            ZRC_ATTR_LEN_ACTION_BANKS_SUPPORTED,
            zrcCfgCFGParam.zrcInfo.actionBanksSupported ) != RTI_SUCCESS)
    {
        //  AP_FATAL_ERROR();
        LOG_WARN("[Configuration] Could not write ZRC_ITEM_ACTION_BANKS_SUPPORTED_RX\n");
    }
    else
    {
        LOG_DEBUG("[Configuration] Successfully wrote ZRC_ITEM_ACTION_BANKS_SUPPORTED_RX\n");
    }

    if (RTI_WriteItemEx( RTI_PROFILE_GDP,
            GDP_ITEM_PRIMARY_CLASS_DESCRIPTOR,
            sizeof(zrcCfgCFGParam.gdpInfo.primaryClassDescriptor),
            (uint8 *)&zrcCfgCFGParam.gdpInfo.primaryClassDescriptor ) != RTI_SUCCESS)
    {
        //  AP_FATAL_ERROR();
        LOG_WARN("[Configuration] Could not write GDP_ITEM_PRIMARY_CLASS_DESCRIPTOR\n");
    }
    else
    {
        LOG_DEBUG("[Configuration] Successfully wrote GDP_ITEM_PRIMARY_CLASS_DESCRIPTOR\n");
    }

    if (RTI_WriteItemEx( RTI_PROFILE_GDP,
            GDP_ITEM_SECONDARY_CLASS_DESCRIPTOR,
            sizeof(zrcCfgCFGParam.gdpInfo.secondaryClassDescriptor),
            (uint8 *)&zrcCfgCFGParam.gdpInfo.secondaryClassDescriptor ) != RTI_SUCCESS)
    {
        //  AP_FATAL_ERROR();
        LOG_WARN("[Configuration] Could not write GDP_ITEM_SECONDARY_CLASS_DESCRIPTOR\n");
    }
    else
    {
        LOG_DEBUG("[Configuration] Successfully wrote GDP_ITEM_SECONDARY_CLASS_DESCRIPTOR\n");
    }

    if (RTI_WriteItemEx( RTI_PROFILE_GDP,
            GDP_ITEM_TERTIARY_CLASS_DESCRIPTOR,
            sizeof(zrcCfgCFGParam.gdpInfo.tertiaryClassDescriptor),
            (uint8 *)&zrcCfgCFGParam.gdpInfo.tertiaryClassDescriptor ) != RTI_SUCCESS)
    {
        //  AP_FATAL_ERROR();
        LOG_WARN("[Configuration] Could not write GDP_ITEM_TERTIARY_CLASS_DESCRIPTOR\n");
    }
    else
    {
        LOG_DEBUG("[Configuration] Successfully wrote GDP_ITEM_TERTIARY_CLASS_DESCRIPTOR\n");
    }

    // recipient binding type used
    if (RTI_WriteItemEx( RTI_PROFILE_GDP,
            GDP_ITEM_BINDING_CAP,
            sizeof(zrcCfgCFGParam.gdpInfo.bindingCapabilities),
            (uint8 *)&zrcCfgCFGParam.gdpInfo.bindingCapabilities ) != RTI_SUCCESS)
    {
        //  AP_FATAL_ERROR();
        LOG_WARN("[Configuration] Could not write GDP_ITEM_BINDING_CAP\n");
    }
    else
    {
        LOG_DEBUG("[Configuration] Successfully wrote GDP_ITEM_BINDING_CAP\n");
    }

    // Extended validation wait time
    if (RTI_WriteItemEx( RTI_PROFILE_GDP,
            GDP_ITEM_BINDING_RECIPIENT_EXTENDED_VALIDATION_WAIT_TIME,
            sizeof( zrcCfgCFGParam.gdpInfo.extendedValidationWaitTime ),
            (uint8 *)&zrcCfgCFGParam.gdpInfo.extendedValidationWaitTime ) != RTI_SUCCESS)
    {
        //  AP_FATAL_ERROR();
        LOG_WARN("[Configuration] Could not write GDP_ITEM_BINDING_RECIPIENT_EXTENDED_VALIDATION_WAIT_TIME\n");
    }
    else
    {
        LOG_DEBUG("[Configuration] Successfully wrote GDP_ITEM_BINDING_RECIPIENT_EXTENDED_VALIDATION_WAIT_TIME\n");
    }

    zrcRIBInit();
}

void zrcRIBInit( void )
{
    // Initialize all attribute to their default value.
}

void zrcCfgGetCFGParamFromRNP(void)
{
    ///////////////////////////////////////////
    // Read Configuration Parameters From RNP//
    ///////////////////////////////////////////

    if (RTI_ReadItemEx(RTI_PROFILE_RTI, RTI_CP_ITEM_NODE_CAPABILITIES, 1,
            (uint8*) &(zrcCfgCFGParam.devInfo.nodeCapabilities)) != RTI_SUCCESS) {
        //   AP_FATAL_ERROR();
        LOG_WARN("[Configuration] Failed to read Node Capabilities\n");
    }
    else
    {
        zrcCfgSetTarget((zrcCfgCFGParam.devInfo.nodeCapabilities & RCN_NODE_CAP_TARGET) ? TRUE : FALSE);
    }

    if (RTI_ReadItemEx(RTI_PROFILE_RTI, RTI_CP_ITEM_NODE_SUPPORTED_TGT_TYPES,
            RTI_MAX_NUM_SUPPORTED_TGT_TYPES,
            zrcCfgCFGParam.devInfo.tgtTypeList) != RTI_SUCCESS) {
        ///AP_FATAL_ERROR();
        LOG_WARN("[Configuration] Failed to read Target Types\n");
    }

    if (RTI_ReadItemEx(RTI_PROFILE_RTI, RTI_CP_ITEM_APPL_CAPABILITIES, 1,
            (uint8*) &(zrcCfgCFGParam.devInfo.appCapabilities)) != RTI_SUCCESS) {
        // AP_FATAL_ERROR();
        LOG_WARN("[Configuration] Failed to read Application Capabilities\n");
    }

    if (RTI_ReadItemEx(RTI_PROFILE_RTI, RTI_CP_ITEM_APPL_DEV_TYPE_LIST,
            RTI_MAX_NUM_DEV_TYPES, zrcCfgCFGParam.devInfo.devTypeList) != RTI_SUCCESS) {
        //   AP_FATAL_ERROR();
        LOG_WARN("[Configuration] Failed to read Device Types\n");
    }

    if (RTI_ReadItemEx(RTI_PROFILE_RTI, RTI_CP_ITEM_APPL_PROFILE_ID_LIST,
            RTI_MAX_NUM_PROFILE_IDS, zrcCfgCFGParam.devInfo.profileIdList) != RTI_SUCCESS) {
        //    AP_FATAL_ERROR();
        LOG_WARN("[Configuration] Failed to read Profile IDs\n");
    }

    if (RTI_ReadItemEx(RTI_PROFILE_RTI, RTI_CP_ITEM_VENDOR_ID, 2,
            (uint8*) &(zrcCfgCFGParam.devInfo.vendorId)) != RTI_SUCCESS) {
        //  AP_FATAL_ERROR();
        LOG_WARN("[Configuration] Failed to read Vendor ID\n");
    }

    if (RTI_ReadItemEx(RTI_PROFILE_RTI, RTI_CP_ITEM_VENDOR_NAME,
            sizeof(vendorName), zrcCfgCFGParam.devInfo.vendorString) != RTI_SUCCESS) {
        //  AP_FATAL_ERROR();
        LOG_WARN("[Configuration] Failed to read Vendor String\n");
    }
    uint16 shortAddr = 0xFFFF;
    if (RTI_ReadItemEx(RTI_PROFILE_RTI, RTI_SA_ITEM_SHORT_ADDRESS,
            sizeof(shortAddr), (uint8*)&shortAddr) != RTI_SUCCESS) {
        //  AP_FATAL_ERROR();
        LOG_WARN("[Configuration] Failed to read shortAddr\n");
    }
    uint16 panID = 0xFFFF;
    if (RTI_ReadItemEx(RTI_PROFILE_RTI, RTI_SA_ITEM_PAN_ID,
            sizeof(panID), (uint8*)&panID) != RTI_SUCCESS) {
        //  AP_FATAL_ERROR();
        LOG_WARN("[Configuration] Failed to read panID\n");
    }
    DispCFGCurrentCfg(&zrcCfgCFGParam, shortAddr, panID);
}

/**************************************************************************************************
 *
 * @fn      zrcCfgClearPairingTableEntryPrivate
 *
 * @brief   INTERNAL (common-code) function to clear ONE entry from the pairing table
 *
 * @param   index - Index (0..pairingTableSize) within the pairing table to clear.
 * @param   tempEntry - Caller-allocated working buf (so if called repeatedly, this func doesn't have to alloc/free).
 * @return  void
 */
static void zrcCfgClearPairingTableEntryPrivate(uint8 index, rcnNwkPairingEntry_t *tempEntry)
{
    // WARNING: Presumes caller pre-determined index is within range or that the read
    //.itself range checks the index and will fail if out of range.

    if (!tempEntry)
        LOG_ERROR("[Configuration] NULL tempEntry!\n");
    else
    {
        if ((RTI_ReadIndexedItem(RTI_PROFILE_RTI,
                RTI_SA_ITEM_PAIRING_TABLE_ENTRY, index, sizeof(*tempEntry),
                (uint8 *) tempEntry)) != RTI_SUCCESS)
        {
            LOG_ERROR("[Configuration] Pairing Table index %u read failed.  CANNOT CLEAR.\n", (unsigned int)index);
        }
        else
        {
            // Invalidate item
            tempEntry->pairingRef = RTI_INVALID_PAIRING_REF;
            RTI_WriteIndexedItem(RTI_PROFILE_RTI, RTI_SA_ITEM_PAIRING_TABLE_ENTRY, index,
                    sizeof(*tempEntry), (uint8 *) tempEntry);

            LOG_INFO("[Configuration] Pairing Table index %u cleared.\n", (unsigned int)index);
        }


    }
}

/**************************************************************************************************
 *
 * @fn      zrcCfgClearPairingTableEntry
 *
 * @brief   Function to clear ONE entry from the pairing table
 *
 * @param   index - Index (0..pairingTableSize) within the pairing table to clear.
 *
 * @return  void
 */
void zrcCfgClearPairingTableEntry(uint8 index)
{
    uint8 pairingTableSize = 0;

    RTI_ReadItemEx(RTI_PROFILE_RTI,
            RTI_CONST_ITEM_MAX_PAIRING_TABLE_ENTRIES, 1,
            (uint8*) &pairingTableSize);

    if (index >= pairingTableSize)
        LOG_WARN("[Configuration] Doing nothing! Index %u is out of range (0..%u).\n", (unsigned int)index, (unsigned int)pairingTableSize);
    else
    {
        // Allocate memory for one pairing entry
        rcnNwkPairingEntry_t *pEntry = (rcnNwkPairingEntry_t *)malloc(sizeof(*pEntry));

        zrcCfgClearPairingTableEntryPrivate(index, pEntry);
        // Free pairing entry buffer
        free(pEntry);
    }
}


/**************************************************************************************************
 *
 * @fn      zrcCfgClearPairingTable
 *
 * @brief   Function to clear ENTIRE pairing table
 *
 * @return  void
 */
void zrcCfgClearPairingTable()
{
    uint8                index;
    uint8                pairingTableSize = 0;
    rcnNwkPairingEntry_t *pEntry = (rcnNwkPairingEntry_t *)malloc(sizeof(*pEntry));

    RTI_ReadItemEx(RTI_PROFILE_RTI,
            RTI_CONST_ITEM_MAX_PAIRING_TABLE_ENTRIES, 1,
            (uint8*) &pairingTableSize);

    for (index = 0; index < pairingTableSize; index++)
        zrcCfgClearPairingTableEntryPrivate(index, pEntry);

    // Free pairing entry buffer
    free(pEntry);

    LOG_INFO("[Configuration] Pairing Table has been cleared.\n");
}

/**************************************************************************************************
 *
 * @fn      zrcCfgGetOldestEntryIndex
 *
 * @brief   Function to find the oldest pairing entry in the table
 *
 * @param   none.
 *
 * @return  uint8 index of oldest entry
 */
uint8 zrcCfgGetOldestEntryIndex()
{
    unsigned long oldestIndexTime = boundRemotes[0].TimestampLastCommand;
    uint8 oldestIndex = 0, i;
    for (i = 1; i < (ZRC_MAX_BOUND_REMOTES-1); i++)
    {
        if (boundRemotes[i].TimestampLastCommand < oldestIndexTime)
        {
            oldestIndexTime = boundRemotes[i].TimestampLastCommand;
            oldestIndex = i;
        }
    }
    return oldestIndex;
}

void RTI_GetAttributeReq( uint8 pairIndex, uint8 attrId, uint16 entryId )
{
      switch (attrId)
      {
        case aplActionCodesSupportedRx:
        {
          if (entryId == 0)
          {
            RTI_GetAttributeCnf( pairIndex, RTI_SUCCESS,
                sizeof( zrcCfgCercActionCodesSupported ),
                zrcCfgCercActionCodesSupported );
          }
          else if (entryId == 32)
          {
            RTI_GetAttributeCnf( pairIndex, RTI_SUCCESS,
                sizeof( zrcCfgHidKeyboardActionCodesSupported ),
                zrcCfgHidKeyboardActionCodesSupported );
          }
          else if (entryId == 34)
          {
            RTI_GetAttributeCnf( pairIndex, RTI_SUCCESS,
                sizeof( zrcCfgHidConsumerPageSectionAActionCodesSupported ),
                zrcCfgHidConsumerPageSectionAActionCodesSupported );
          }
          else if (entryId == 224)
          {
            RTI_GetAttributeCnf( pairIndex, RTI_SUCCESS,
                sizeof( zrcCfgHidKeyboardActionCodesSupported ),
                zrcCfgHidKeyboardActionCodesSupported );
          }
          else
          {
            RTI_GetAttributeCnf( pairIndex, RTI_ERROR_INVALID_PARAMETER,
                0, NULL );
          }
          LOG_INFO("[Configuration][%d] aplActionCodesSupportedRx requested, index %d\n", pairIndex, entryId);
          break;
        }

        case aplActionMappings:
        {
          if (inIRValidationMode)
          {
              LOG_INFO("IR validation for entryID %d\n", entryId);

              RTI_GetAttributeCnf( pairIndex,
                  (entryId == 0xFF) ? 0xCC : RTI_SUCCESS,
                  sizeof(zrcCfgKeyMappingIRTest),
                  zrcCfgKeyMappingIRTest);

              if (entryId == 0xFF)
              {
                  inIRValidationMode = 0;
              }
          }
          else
          {
              if (zrcCfgActionMappingIndexToCmdTable[entryId & 0xFF].data)
              {
                    RTI_GetAttributeCnf(pairIndex, RTI_SUCCESS,
                        zrcCfgActionMappingIndexToCmdTable[entryId & 0xFF].dataLength,
                        zrcCfgActionMappingIndexToCmdTable[entryId & 0xFF].data);
              }
              else
              {
                    RTI_GetAttributeCnf( pairIndex, RTI_SUCCESS,
                        sizeof(zrcCfgKeyMappingDefault),
                        zrcCfgKeyMappingDefault);
              }
          }
          LOG_INFO("[Configuration][%d] aplActionMappings requested\n", pairIndex);
          break;
        }

        case aplHomeAutomation:
        {
          if (entryId == 0x2000)
          {
            RTI_GetAttributeCnf( pairIndex, RTI_SUCCESS, sizeof( zrcCfgOnOff ), zrcCfgOnOff );
          }
          else
          {
            RTI_GetAttributeCnf( pairIndex, RTI_ERROR_INVALID_PARAMETER, 0, NULL );
          }

          LOG_INFO("[Configuration][%d] HA attribute requested; instance = %2.2x, HA attrId = %2.2x\n", pairIndex, entryId & 0xff, (entryId & 0xff00) >> 8);
          break;
        }

        case aplPollConfiguration:
        {
          RTI_GetAttributeCnf( pairIndex, RTI_SUCCESS, sizeof( zrcCfgPollConfiguration ), (uint8 *)&zrcCfgPollConfiguration );
          LOG_INFO("[Configuration][%d] Poll configuration requested (interval %d, timeout %d)\n", pairIndex,
                  zrcCfgPollConfiguration.pollingTimeInterval,
                  zrcCfgPollConfiguration.pollingTimeout);
          break;
        }

        default:
        {
          LOG_INFO("[Configuration][%d] RTI_GetAttributeReq received, attrId = %2.2x, Entry Id = %d\n", pairIndex, attrId, entryId);
          break;
        }
      }
}

/**************************************************************************************************
 *
 * @fn      RTI_SetAttributeReq
 *
 * @brief   Local function to handle received Set Attributes Requests from node
 *
 * @param   pairIndex            - pairing table index of node sending the command
 *             gdpAttrHeader_t *    - request header
 *             pAttrData            - pointer to attribute data
 *
 * @return  void
 */
void RTI_SetAttributeReq( uint8 pairIndex, gdpAttrHeader_t *pAttrHdr, uint8 *pAttrData )
{
      uint8 i;
      uint16 vendor;
      uint8 attrStatus = GDP_ATTR_RSP_SUCCESS;

      switch (pAttrHdr->attrId)
      {
        case aplMappableActions:
        {
          // answer
          RTI_SetAttributeCnf( pairIndex, attrStatus );

          LOG_INFO("[Configuration] aplMappableActions received\n");
          if ( (pAttrData[1] == (ZRC_ACTION_BANK_CLASS_HDMI_CEC | ZRC_ACTION_BANK_HDMI_CEC)) )
          {
            /* HDMI CEC Action code */
            switch(pAttrData[2])
            {
                case RTI_CERC_ROOT_MENU:
                    /* Assign the new function to this entry */
                    zrcCfgActionMappingIndexToCmdTable[pAttrHdr->entryIdLsb].dataLength = sizeof(zrcCfgKeyMappingPower);
                    zrcCfgActionMappingIndexToCmdTable[pAttrHdr->entryIdLsb].data = zrcCfgKeyMappingPower;
                    break;
                case RTI_CERC_UP:
                    zrcCfgActionMappingIndexToCmdTable[pAttrHdr->entryIdLsb].dataLength = sizeof(zrcCfgKeyMappingPower);
                    zrcCfgActionMappingIndexToCmdTable[pAttrHdr->entryIdLsb].data = zrcCfgKeyMappingPower;
                    break;

                default:
                    /*
                     * We don't want to remap this action code, so we want to
                     * let the action mapping client know that it should use its
                     * default behavior
                     */
                    zrcCfgActionMappingIndexToCmdTable[pAttrHdr->entryIdLsb].dataLength = sizeof(zrcCfgKeyMappingDefault);
                    zrcCfgActionMappingIndexToCmdTable[pAttrHdr->entryIdLsb].data = zrcCfgKeyMappingDefault;
                    break;
            }
          }
          else if ( (pAttrData[1] == (ZRC_ACTION_BANK_CLASS_HDMI_CEC |
                                      ZRC_ACTION_BANK_CLASS_VENDOR_SPECIFIC_EXPLICIT)) &&
                    (pAttrData[2] == 0xFF) )
          {
               LOG_INFO ("In IR Validation now\n");
               inIRValidationMode = 1;
          }
          break;
        }

        case aplIrdbVendorSupport:
        {
          // answer
          RTI_SetAttributeCnf( pairIndex, attrStatus );

          LOG_INFO("[Configuration] aplIrdbVendorSupport received\n");
          LOG_INFO("[Configuration] Vendor ID list =\n");
          for (i = 0; i < pAttrHdr->attrLen; i += 2)
          {
            vendor = pAttrData[i] | (pAttrData[i+1]) << 8;
            LOG_INFO("[Configuration] %4.4X\n", vendor);
          }
          break;
        }

        case aplHomeAutomationSupported:
        {
          // answer
          RTI_SetAttributeCnf( pairIndex, attrStatus );

          LOG_INFO("[Configuration] aplHomeAutomationSupported received\n");
          break;
        }

        case aplPollConstraints:
        {
          gdpPollConstraintsAttr_t *pPollConstraintsAttr;
          gdpPollConstraintRec_t *pPollConstraintRec;

          pPollConstraintsAttr = (gdpPollConstraintsAttr_t *)pAttrData;
          LOG_INFO("[Configuration] aplPollConstraints received:\n");
          LOG_INFO("\t\t\t\t %s: %d\n", "pPollConstraintsAttr->numPollMethodsSupported", pPollConstraintsAttr->numPollMethodsSupported);
          if (pPollConstraintsAttr->numPollMethodsSupported == 1)
          {
            pPollConstraintRec = (gdpPollConstraintRec_t *)&pPollConstraintsAttr->records[0];
            LOG_INFO("\t\t\t\t %s: %d\n", "pPollConstraintRec->pollingMethodId", pPollConstraintRec->pollingMethodId);
            if (pPollConstraintRec->pollingMethodId == GDP_POLL_METHOD_HEARTBEAT)
            {
              // got a valid configuration, so set up poll configuration
              zrcCfgPollConfiguration.pollingMethodId = GDP_POLL_METHOD_HEARTBEAT;
              zrcCfgPollConfiguration.pollingTriggerConfig = pPollConstraintRec->pollingTriggerCapabilities;
              if (pPollConstraintRec->pollingTriggerCapabilities & GDP_POLL_KEY_PRESS_MASK)
              {
                zrcCfgPollConfiguration.pollingKeyPressCounter =
                  (pPollConstraintRec->minPollingKeyPressCounter + pPollConstraintRec->maxPollingKeyPressCounter) / 2;
              }
              else
              {
                zrcCfgPollConfiguration.pollingKeyPressCounter = 0;
              }

              LOG_INFO("\t\t\t\t %s: %d\n", "pPollConstraintRec->pollingTriggerCapabilities", pPollConstraintRec->pollingTriggerCapabilities);
              if (pPollConstraintRec->pollingTriggerCapabilities & GDP_POLL_TIME_BASED_MASK)
              {
                if ( (pPollConstraintRec->minPollingTimeInterval >= GDP_POLL_MIN_TIME_INTERVAL_MIN_VALUE) &&
                     (pPollConstraintRec->minPollingTimeInterval <= GDP_POLL_MIN_TIME_INTERVAL_MAX_VALUE) &&
                     (pPollConstraintRec->maxPollingTimeInterval >= GDP_POLL_MAX_TIME_INTERVAL_MIN_VALUE) &&
                     (pPollConstraintRec->maxPollingTimeInterval <= GDP_POLL_MAX_TIME_INTERVAL_MAX_VALUE) )
                {
                  zrcCfgPollConfiguration.pollingTimeInterval = zrcCfgPollingTimeInterval;
                  zrcCfgPollingTimeInterval = 5000; // update for next time
                }
                else
                {
                  // bad value, so reject
                  attrStatus = GDP_ATTR_RSP_INVALID_ENTRY;
                }
              }
              else
              {
                zrcCfgPollConfiguration.pollingTimeInterval = 0;
              }

              zrcCfgPollConfiguration.pollingTimeout = aplcMaxPollingTimeout;
            }
          }

          // answer
          RTI_SetAttributeCnf( pairIndex, attrStatus );

          break;
        }

        case aplIdentificationCapabilities:
        {
          zrcCfgIdentificationClientCapabilities = *pAttrData;

          // success, so move on to next state
          attrStatus = GDP_GENERIC_RSP_SUCCESS;

          // answer
          RTI_SetAttributeCnf( pairIndex, attrStatus );

          LOG_INFO("[Configuration] aplIdentificationCapabilities received\n");
         break;
        }

        default:
        {
          // answer
//          RTI_SetAttributeCnf( pairIndex, attrStatus );

          LOG_INFO("[Configuration] RTI_SetAttributeReq received, pairIndex = %d, attrId = %2.2x, Entry Id = %d\n",
            pairIndex, pAttrHdr->attrId, pAttrHdr->entryIdLsb | (pAttrHdr->entryIdMsb << 8));
          break;
        }
      }
}

uint8 zrcCfgProcessGDPPushAttributesRequest(uint8 srcIndex, uint8 len, uint8 *pData)
{
    gdpPushAttrCmd_t *pReq = (gdpPushAttrCmd_t *)pData;
    uint8 retStatus = GDP_ATTR_RSP_UNSUPPORTED;
    uint8 *pRec = pReq->attrRecordList;
    uint8 *pRecEnd = pData + len;
    uint8 attrId, attrLen;
//    uint16 attrIndex;

    // Walk through the attribute list
    while (pRec < pRecEnd)
    {
        attrId = *pRec++;
        if (GDP_ATTR_IS_ATTR_ARRAYED(attrId))
        {
            // Get index for indexed attributes
//            attrIndex = BUILD_UINT16(pRec[0], pRec[1]);
            pRec += 2;
        }
        attrLen = *pRec++;
        // Process attribute
        switch (attrId)
        {
        case aplPowerStatus:
            LOG_INFO("[%d] Power Status received: %d\n", srcIndex, *pRec & GDP_POWER_STATUS_POWER_METER_MASK);
            if (*pRec & GDP_POWER_STATUS_CRITICAL_MASK)
            {
                LOG_WARN("[%d] Power level is critical!\n", srcIndex);
            }
            if (*pRec & GDP_POWER_STATUS_IMDENDING_DOOM_MASK)
            {
                LOG_ERROR("[%d] Device pending doom!\n", srcIndex);
            }
            if (*pRec & GDP_POWER_STATUS_CHARGING_MASK)
            {
                LOG_INFO("[%d] Device is currently charging!\n", srcIndex);
            }
            retStatus = GDP_ATTR_RSP_SUCCESS;
            break;
        default:
            break;
        }
        pRec += attrLen;
    }

    return retStatus;
}

void zrcCfgPostPairingProcessing(uint8 srcIndex)
{
    // Read out entry to populate local RF Diagnostics entry

    rcnNwkPairingEntry_t *pEntry;
    uint8 result;
    // Allocate memory for one pairing entry
    pEntry = (rcnNwkPairingEntry_t *) malloc(sizeof(rcnNwkPairingEntry_t));
    // Try to read out this entry
    if ((result = RTI_ReadIndexedItem(RTI_PROFILE_RTI,
            RTI_SA_ITEM_PAIRING_TABLE_ENTRY, srcIndex, sizeof(rcnNwkPairingEntry_t),
            (uint8 *) pEntry)) != RTI_SUCCESS) {
        LOG_WARN("[Validation] Failed to read out pairing entry after validation\n");
    }
    else
    {
        /********************************************************************************************
         * Make sure we only keep the 8 most recent pairing entries
         * To make logic easier later we perform a little bit of pairing table manipulation here
         */
        RTI_ReadItemEx(RTI_PROFILE_RTI, RTI_SA_ITEM_PT_NUMBER_OF_ACTIVE_ENTRIES, 1, (uint8 *) &result);
        if (result > ZRC_MAX_BOUND_REMOTES)
        {
            // Pairing table is full, replace oldest entry
            uint8 indexToReplace = zrcCfgGetOldestEntryIndex();
            LOG_INFO("[Validation] Pairing table full, replacing entry %d\n", indexToReplace);

            rcnNwkPairingEntry_t *pEntryCleared;
            uint8 result;
            // Allocate memory for one pairing entry
            pEntryCleared = (rcnNwkPairingEntry_t *) malloc(sizeof(rcnNwkPairingEntry_t));
            // Clear the original entry, simply by setting the pairing reference to an invalid value
            pEntryCleared->pairingRef = RTI_INVALID_PAIRING_REF;
            if ((result = RTI_WriteIndexedItem(RTI_PROFILE_RTI,
                    RTI_SA_ITEM_PAIRING_TABLE_ENTRY, srcIndex, sizeof(rcnNwkPairingEntry_t),
                    (uint8 *) pEntryCleared)) != RTI_SUCCESS)
            {
                LOG_WARN("[Validation] Failed to invalidate pairing entry\n");
            }
            // Free this entry, we no longer need it
            free(pEntryCleared);

            // Update reference in pairing entry
            pEntry->pairingRef = indexToReplace;
            // Overwrite oldest entry
            if ((result = RTI_WriteIndexedItem(RTI_PROFILE_RTI,
                    RTI_SA_ITEM_PAIRING_TABLE_ENTRY, indexToReplace, sizeof(rcnNwkPairingEntry_t),
                    (uint8 *) pEntry)) != RTI_SUCCESS)
            {
                LOG_WARN("[Validation] Failed to overwrite oldest pairing entry\n");
            }

            // For the rest of the logic we use srcIndex, so set srcIndex to the indexToReplace
            srcIndex = indexToReplace;
        }

        // Populate fields in RF Diagnostics entry with information from the pairing entry
        // TODO: Verify endianess
        memcpy(boundRemotes[srcIndex].MacAddress, (uint8 *)pEntry->ieeeAddress, sizeof(boundRemotes[srcIndex].MacAddress));
        boundRemotes[srcIndex].ShortAddress = pEntry->nwkAddress;
    }
    // Free pairing entry buffer
    free(pEntry);

    boundRemotes[srcIndex].CommandCount = 0;
}

/**************************************************************************************************
 *
 * @fn      zrcCfgDisplayPairingTable
 *
 * @brief   Function to read out, and display, pairing table
 *
 * @param   status - Result of RTI_InitReq API call.
 *
 * @return  void
 */
void zrcCfgDisplayPairingTable() {
    rcnNwkPairingEntry_t *pEntry;
    // Allocate memory for one pairing entry
    pEntry = (rcnNwkPairingEntry_t *) malloc(sizeof(rcnNwkPairingEntry_t));

    uint8 i, result, atLeastOneEntryFound = 0;

    for (i = 0; i < MIN(zrcCfgMaxNumPairingEntries, MAX_NUM_OF_PAIRING_ENTRIES_LINUX_SIDE); i++)
    {
        // Try to read out this entry
        if ((result = RTI_ReadIndexedItem(RTI_PROFILE_RTI,
                RTI_SA_ITEM_PAIRING_TABLE_ENTRY, i, sizeof(rcnNwkPairingEntry_t),
                (uint8 *) pEntry)) == RTI_SUCCESS)
        {
            // Found pairing entry; display this.
            DisplayPairingTable(pEntry, NULL);
            atLeastOneEntryFound = 1;
        }
    }

    if (atLeastOneEntryFound != 0) {
        LOG_INFO("*************************************\n");
    } else {
        LOG_INFO("*************************************\n");
        LOG_INFO("* Pairing Table Is Empty\n");
        LOG_INFO("*************************************\n");
    }

    // Free pairing entry buffer
    free(pEntry);
}
void DisplayPairingTable(rcnNwkPairingEntry_t *pEntry, ZRC_BoundRemote_t *rfStat)
{
    int8 j;
    char tmpStr[512];
    size_t strLen = 0;
    LOG_INFO("*************************************\n");
    LOG_INFO("* Pairing Index: \t \t 0x%.2X\n", pEntry->pairingRef);
    LOG_INFO("* SRC NWK Address: \t \t 0x%.4hX\n", pEntry->srcNwkAddress);
    LOG_INFO("* Logical Channel: \t \t 0x%.2hX\n", pEntry->logicalChannel); //((pEntry->logicalChannel >> 8) & 0x00FF));
    strLen = 0;
    for (j = (SADDR_EXT_LEN - 2); j >= 0; j--)
    {
        snprintf(tmpStr+strLen, sizeof(tmpStr)-strLen, ":%.2X", (pEntry->ieeeAddress[j] & 0x00FF));
        strLen += 3;
    }
    LOG_INFO("* IEEE Address: \t \t %.2hX%s\n", (pEntry->ieeeAddress[SADDR_EXT_LEN - 1] & 0x00FF), tmpStr);
    LOG_INFO("* PAN Id: \t \t \t 0x%.4X\n", pEntry->panId);
    LOG_INFO("* NWK Address: \t \t 0x%.4hX\n", pEntry->nwkAddress);
    LOG_INFO("* Rec Capabilities: \t \t 0x%.2hX\n", (pEntry->recCapabilities & 0x00FF));
    LOG_INFO("* Security Key Valid: \t %s\n", (pEntry->securityKeyValid & 0x00FF) ? "Yes" : "No");
    LOG_INFO("* Vendor Identifier: \t \t 0x%.4hX\n", pEntry->vendorIdentifier);
    LOG_INFO("* Device Type List: \t \t [0x%.2hX, 0x%.2hX, 0x%.2hX]\n",
            (pEntry->devTypeList[0] & 0x00FF),
            (pEntry->devTypeList[1] & 0x00FF),
            (pEntry->devTypeList[2] & 0x00FF));
    LOG_INFO("* Received Frame Counter: \t 0x%.8X (%u)\n", pEntry->recFrameCounter, pEntry->recFrameCounter);
    LOG_INFO("* Profiles Discovered: \t 0x%.4X\n", pEntry->profileDiscs[0]);
}

void DispCFGCurrentCfg(RTI_ConfigParam_t *appCfg, uint16 shortAddr, uint16 panID)
{
    int16 i;
    char* tmpStrTbl[4];
    char tmpStr[512];
    size_t strLen = 0;
    LOG_INFO("------------------------------------------------------\n");
    LOG_INFO("- Current Configuration:\n");
    LOG("- \tNode Capabilities: \t \t \t 0x%.2X\n", appCfg->devInfo.nodeCapabilities);
    if (appCfg->devInfo.nodeCapabilities & RTI_NODE_CAP_NODE_TYPE_BM)
        tmpStrTbl[0] = "Target";
    else
        tmpStrTbl[0] = "Controller";

    if (appCfg->devInfo.nodeCapabilities & RTI_NODE_CAP_MAINS_PWR_BM)
        tmpStrTbl[1] = "Mains Powered";
    else
        tmpStrTbl[1] = "Battery Powered";

    if (appCfg->devInfo.nodeCapabilities & RTI_NODE_CAP_SEC_CAP_BM)
        tmpStrTbl[2] = "Security Enabled";
    else
        tmpStrTbl[2] = "-";

    if (appCfg->devInfo.nodeCapabilities & RTI_NODE_CAP_CHAN_NORM_BM)
        tmpStrTbl[3] = "Channel Normalization Supported";
    else
        tmpStrTbl[3] = "-";

    for (i = 0; i < 4; i++)
        LOG("- \t \t%s\n", tmpStrTbl[i]);


    LOG_INFO("- \tApplication Capabilities: \t \t 0x%.2X\n", appCfg->devInfo.appCapabilities);
    LOG_INFO("- \t \t Number of supported profiles:\t 0x%.2X\n",
            RCN_APP_CAPA_GET_NUM_PROFILES(appCfg->devInfo.appCapabilities));
    LOG_INFO("- \t \t Number of supported devices:\t 0x%.2X\n",
            RCN_APP_CAPA_GET_NUM_DEV_TYPES(appCfg->devInfo.appCapabilities));
    if (RCN_APP_CAPA_GET_USER_STRING(appCfg->devInfo.appCapabilities))
        LOG_INFO("- \t \t User String supported\n");
    else
    {
        LOG_INFO("- \t \t User String not supported\n");
        LOG_INFO("- \t \t USER STRING NEEDS TO BE ENABLE FOR ZRC PROFILE\n");
    }

    LOG_INFO("- \tSupported Profiles:\n");
    for (i = 0; i < RCN_APP_CAPA_GET_NUM_PROFILES(appCfg->devInfo.appCapabilities); i++)
    {
        LOG_INFO("- \t \t \t \t %s\n", rtiProfileId_list[appCfg->devInfo.profileIdList[i]]);
    }
    if (appCfg->devInfo.vendorId <= RTI_VENDOR_TEXAS_INSTRUMENTS)
    {
        LOG_INFO("- \tVendor ID: \t \t 0x%.2X (%s)\n", appCfg->devInfo.vendorId, rtiVendorId_list[appCfg->devInfo.vendorId]);
    }
    else
    {
        LOG_INFO("- \tVendor ID: \t \t 0x%.2X (Unknown)\n", appCfg->devInfo.vendorId);
    }

    LOG_INFO("- \tDevice Types:\n");
    for (i = 0; i < RTI_MAX_NUM_DEV_TYPES; i++)
    {
        if (appCfg->devInfo.devTypeList[i] <= RTI_DEVICE_TARGET_TYPE_END)
        {
            LOG_INFO("- \t \t \t \t 0x%.2X (%s)\n", appCfg->devInfo.devTypeList[i], rtiDevType_list[appCfg->devInfo.devTypeList[i]]);
        }
        else
        {
            LOG_INFO("- \t \t \t \t 0x%.2X (Unknown)\n", appCfg->devInfo.devTypeList[i]);
        }
    }

    LOG_INFO("- \tTarget Device Types:\n");
    for (i = 0; i < RTI_MAX_NUM_SUPPORTED_TGT_TYPES; i++)
    {
        if (appCfg->devInfo.tgtTypeList[i] <= RTI_DEVICE_TARGET_TYPE_END)
        {
            LOG_INFO("- \t \t \t \t 0x%.2X (%s)\n", appCfg->devInfo.tgtTypeList[i], rtiDevType_list[appCfg->devInfo.tgtTypeList[i]]);
        }
        else
        {
            LOG_INFO("- \t \t \t \t 0x%.2X (Unknown)\n", appCfg->devInfo.tgtTypeList[i]);
        }
    }

    LOG_INFO("- \tShort Address: \t \t \t \t 0x%.4X\n", shortAddr);

    LOG_INFO("- \tPAN ID: \t \t \t \t 0x%.4X\n", panID);

    LOG_INFO("----   GDP                                        ----\n");
    LOG_INFO("- \tBinding Capabilities: \t \t \t 0x%.2X\n", appCfg->gdpInfo.bindingCapabilities);
    LOG_INFO("- \tGDP Capabilities: \t \t \t 0x%.2X\n", appCfg->gdpInfo.gdpCapabilities);
    LOG_INFO("- \tPrimary Class Descriptor: \t \t 0x%.2X\n", appCfg->gdpInfo.primaryClassDescriptor);
    LOG_INFO("- \tSecondary Class Descriptor: \t \t 0x%.2X\n", appCfg->gdpInfo.secondaryClassDescriptor);
    LOG_INFO("- \tTertiary Class Descriptor: \t \t 0x%.2X\n", appCfg->gdpInfo.tertiaryClassDescriptor);
    LOG_INFO("- \tExtended Validation Wait Time: \t \t %d\n", appCfg->gdpInfo.extendedValidationWaitTime);

    LOG_INFO("----   ZRC 2.0                                    ----\n");
    LOG_INFO("- \tZRC Capabilities: \t \t \t 0x%.2X\n", appCfg->zrcInfo.zrcCapabilities);
    strLen = 0;
    for (i = 1; i < ZRC_ATTR_LEN_ACTION_BANKS_SUPPORTED; i++)
    {
        snprintf(tmpStr+strLen, sizeof(tmpStr)-strLen, ":%.2X", appCfg->zrcInfo.actionBanksSupported[i]);
        strLen += 3;
    }
    LOG_INFO("- \tAction Banks Supported: \n- \t \t %.2X%s\n", appCfg->zrcInfo.actionBanksSupported[0], tmpStr);

    LOG_INFO("------------------------------------------------------\n");
}
