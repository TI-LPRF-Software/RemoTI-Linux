/**************************************************************************************************
  Filename:       npi_ipc_client.h
  Revised:        $Date: 2011-11-23 12:02:49 -0800 (Wed, 23 Nov 2011) $
  Revision:       $Revision: 108 $

  Description:    This file contains special interface for RTI linux surrogate

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
#ifndef NPI_IPC_CLIENT_H
#define NPI_IPC_CLIENT_H


// -- Exported function --
#ifdef __cplusplus
extern "C"
{
#endif

  /**************************************************************************************************
   * INCLUDES
   **************************************************************************************************/

/* NPI includes */
#include "npi_lnx.h"
#include "rtis_lnx.h"

/* RTI_LNX includes */
 /* Initialize RTI Surrogate
   * devpath - serial interface device path, or IPaddress:port
   * returns TRUE when initialized successfully. Otherwise, FALSE */
  int NPI_ClientInit(const char *devpath);

  /* Close RTI Surrogate */
  void NPI_ClientClose(void);

  /* The following two functions comes from NPI. They are used in the RTIS client module.*/
  void NPI_SendAsynchData( npiMsgData_t *pMsg );
  void NPI_SendSynchData( npiMsgData_t *pMsg );

  /* The following two functions allow client to control Server */
  void NPI_ConnectReq( uint8 *pStatus, uint8 length, uint8 *devPath );
  void NPI_DisconnectReq( uint8 *pStatus );

  void NPI_SetWorkaroundReq( uint8 workaroundID, uint8 *pStatus );

  /**************************************************************************************************
   * TYPEDEFS
   **************************************************************************************************/

  /***************************************************************************************************
   * SYS COMMANDS
   ***************************************************************************************************/

  /* AREQ from host */
  #define MT_SYS_RESET_REQ                     0x00
  #define MT_SYS_SWITCH_MAC_REQ				   0x10

  /* SREQ/SRSP */
  #define MT_SYS_PING                          0x01
  #define MT_SYS_VERSION                       0x02
  #define MT_SYS_SET_EXTADDR                   0x03
  #define MT_SYS_GET_EXTADDR                   0x04
  #define MT_SYS_RAM_READ                      0x05
  #define MT_SYS_RAM_WRITE                     0x06
  #define MT_SYS_OSAL_NV_ITEM_INIT             0x07
  #define MT_SYS_OSAL_NV_READ                  0x08
  #define MT_SYS_OSAL_NV_WRITE                 0x09
  #define MT_SYS_OSAL_START_TIMER              0x0A
  #define MT_SYS_OSAL_STOP_TIMER               0x0B
  #define MT_SYS_RANDOM                        0x0C
  #define MT_SYS_ADC_READ                      0x0D
  #define MT_SYS_GPIO                          0x0E
  #define MT_SYS_STACK_TUNE                    0x0F
  #define MT_SYS_SET_TX_POWER                  0x14

  /* AREQ to host */
  #define MT_SYS_RESET_IND                     0x80
  #define MT_SYS_OSAL_TIMER_EXPIRED            0x81
  #define NPI_SYS_MAC_SWITCH                   0x82

  /***************************************************************************************************
   * MAC COMMANDS
   ***************************************************************************************************/
  /* SREQ/SRSP */
  #define MT_MAC_RESET_REQ                     0x01
  #define MT_MAC_INIT                          0x02
  #define MT_MAC_START_REQ                     0x03
  #define MT_MAC_SYNC_REQ                      0x04
  #define MT_MAC_DATA_REQ                      0x05
  #define MT_MAC_ASSOCIATE_REQ                 0x06
  #define MT_MAC_DISASSOCIATE_REQ              0x07
  #define MT_MAC_GET_REQ                       0x08
  #define MT_MAC_SET_REQ                       0x09
  #define MT_MAC_GTS_REQ                       0x0a
  #define MT_MAC_RX_ENABLE_REQ                 0x0b
  #define MT_MAC_SCAN_REQ                      0x0c
  #define MT_MAC_POLL_REQ                      0x0d
  #define MT_MAC_PURGE_REQ                     0x0e
  #define MT_MAC_SET_RX_GAIN_REQ               0x0f

  /* Security PIB SREQ */
  #define MT_MAC_SECURITY_GET_REQ              0x10
  #define MT_MAC_SECURITY_SET_REQ              0x11
  #define MT_MAC_DELETE_DEVICE_REQ             0x12
  #define MT_MAC_READ_KEY_WITH_ID_REQ          0x13
  #define MT_MAC_WRITE_KEY_WITH_ID_REQ         0x14
  #define MT_MAC_ADD_DEVICE_REQ                0x15
  #define MT_MAC_DELETE_ALL_DEVICES_REQ        0x16

  /* Enhanced Active Scan request */
  #define MT_MAC_ENHANCED_ACTIVE_SCAN_REQ      0x17
  #define MT_MAC_ENHANCED_ACTIVE_SCAN_RSP      0x18

  /* Update Device Table Entry with new PAN ID */
  #define MT_MAC_UPDATE_PAN_ID                 0x19

  /* Source Matching SREQ */
  #define MT_MAC_SRC_MATCH_ENABLE_REQ          0x20
  #define MT_MAC_SRC_MATCH_ADD_ENTRY_REQ       0x21
  #define MT_MAC_SRC_MATCH_DELETE_ENTRY_REQ    0x22
  #define MT_MAC_SRC_MATCH_ACK_ALL_PENDING_REQ 0x23
  #define MT_MAC_SRC_MATCH_CHECK_ALL_PENDING_REQ 0x24

  /* Extension Interface */
  #define MT_MAC_PWR_OFF_REQ			       0x25
  #define MT_MAC_PWR_ON_REQ			           0x26
  #define MT_MAC_PWR_MODE_REQ			       0x27
  #define MT_MAC_PWR_NEXT_TIMEOUT_REQ 	       0x28


  /* AREQ from Host */
  #define MT_MAC_ASSOCIATE_RSP                 0x50
  #define MT_MAC_ORPHAN_RSP                    0x51

  /* AREQ to host */
  #define MT_MAC_SYNC_LOSS_IND                 0x80
  #define MT_MAC_ASSOCIATE_IND                 0x81
  #define MT_MAC_ASSOCIATE_CNF                 0x82
  #define MT_MAC_BEACON_NOTIFY_IND             0x83
  #define MT_MAC_DATA_CNF                      0x84
  #define MT_MAC_DATA_IND                      0x85
  #define MT_MAC_DISASSOCIATE_IND              0x86
  #define MT_MAC_DISASSOCIATE_CNF              0x87
  #define MT_MAC_GTS_CNF                       0x88
  #define MT_MAC_GTS_IND                       0x89
  #define MT_MAC_ORPHAN_IND                    0x8a
  #define MT_MAC_POLL_CNF                      0x8b
  #define MT_MAC_SCAN_CNF                      0x8c
  #define MT_MAC_COMM_STATUS_IND               0x8d
  #define MT_MAC_START_CNF                     0x8e
  #define MT_MAC_RX_ENABLE_CNF                 0x8f
  #define MT_MAC_PURGE_CNF                     0x90
  #define MT_MAC_POLL_IND                      0x91

  /***************************************************************************************************
   * RTI COMMANDS (RCAF)
   ***************************************************************************************************/

  // RTIS Command Ids for NPI Callback
  #define RTIS_CMD_ID_RTI_READ_ITEM              0x01  // Deprecated for 0x21
  #define RTIS_CMD_ID_RTI_WRITE_ITEM             0x02  // Deprecated for 0x22
  #define RTIS_CMD_ID_RTI_INIT_REQ               0x03
  #define RTIS_CMD_ID_RTI_PAIR_REQ               0x04
  #define RTIS_CMD_ID_RTI_SEND_DATA_REQ          0x05
  #define RTIS_CMD_ID_RTI_ALLOW_PAIR_REQ         0x06
  #define RTIS_CMD_ID_RTI_STANDBY_REQ            0x07
  #define RTIS_CMD_ID_RTI_RX_ENABLE_REQ          0x08
  #define RTIS_CMD_ID_RTI_ENABLE_SLEEP_REQ       0x09
  #define RTIS_CMD_ID_RTI_DISABLE_SLEEP_REQ      0x0A
  #define RTIS_CMD_ID_RTI_UNPAIR_REQ             0x0B
  #define RTIS_CMD_ID_RTI_PAIR_ABORT_REQ         0x0C
  #define RTIS_CMD_ID_RTI_ALLOW_PAIR_ABORT_REQ   0x0D
  //
  #define RTIS_CMD_ID_TEST_PING_REQ              0x10
  #define RTIS_CMD_ID_RTI_TEST_MODE_REQ          0x11
  #define RTIS_CMD_ID_RTI_RX_COUNTER_GET_REQ     0x12
  #define RTIS_CMD_ID_RTI_SW_RESET_REQ           0x13
  //
  #define RTIS_CMD_ID_RTI_READ_ITEM_EX           0x21
  #define RTIS_CMD_ID_RTI_WRITE_ITEM_EX          0x22

#ifdef ZRC20_PROFILE
  #define RTIS_CMD_ID_RTI_BIND_REQ                     0x30
  #define RTIS_CMD_ID_RTI_ALLOW_BIND_REQ               0x31
  #define RTIS_CMD_ID_RTI_GET_VALIDATION_STATUS_RSP    0x32
  #define RTIS_CMD_ID_RTI_POLL_CONFIG_REQ              0x33
  #define RTIS_CMD_ID_RTI_IDENTIFICATION_CONFIG_REQ    0x34
  #define RTIS_CMD_ID_RTI_POLL_REQ                     0x35
  #define RTIS_CMD_ID_RTI_KEY_EXCHANGE_REQ             0x36
  #define RTIS_CMD_ID_RTI_GET_ATTRIBUTE_CNF            0x37
  #define RTIS_CMD_ID_RTI_SET_ATTRIBUTE_CNF            0x38
  #define RTIS_CMD_ID_RTI_SEND_IRDB_VENDOR_SUPPORT_REQ 0x39
  #define RTIS_CMD_ID_RTI_SEND_MAPPABLE_ACTIONS_REQ    0x3a
  #define RTIS_CMD_ID_RTI_GET_ACTION_MAPPINGS_REQ      0x3b
  #define RTIS_CMD_ID_RTI_HA_SUPPORTED_ANNOUNCE_REQ    0x3c
  #define RTIS_CMD_ID_RTI_PULL_HA_ATTRIBUTES_REQ       0x3d
  #define RTIS_CMD_ID_RTI_POLL_RSP                     0x3e
  #define RTIS_CMD_ID_RTI_UNBIND_REQ                   0x3f
  #define RTIS_CMD_ID_RTI_BIND_ABORT_REQ               0x40
  #define RTIS_CMD_ID_RTI_ALLOW_BIND_ABORT_REQ         0x41
#endif //ZRC20_PROFILE

#if (defined FEATURE_MSO) && (FEATURE_MSO == TRUE)
  #define RTIS_CMD_ID_RTI_BIND_PARAMS_REQ        0x0E
  #define RTIS_CMD_ID_RTI_SET_BIND_INDICATOR_REQ 0x0F
  #define RTIS_CMD_ID_RTI_SEND_MSO_COMMAND_REQ   0x30
  #define RTIS_CMD_ID_RTI_BUILD_MSO_CMD_SET_REQ  0x31
  #define RTIS_CMD_ID_RTI_BUILD_MSO_CMD_GET_REQ  0x32
#endif //(defined FEATURE_MSO) && (FEATURE_MSO == TRUE)

  // RTIS Confirm Ids
  #define RTIS_CMD_ID_RTI_INIT_CNF               0x01
  #define RTIS_CMD_ID_RTI_PAIR_CNF               0x02
  #define RTIS_CMD_ID_RTI_SEND_DATA_CNF          0x03
  #define RTIS_CMD_ID_RTI_ALLOW_PAIR_CNF         0x04
  #define RTIS_CMD_ID_RTI_REC_DATA_IND           0x05
  #define RTIS_CMD_ID_RTI_STANDBY_CNF            0x06
  #define RTIS_CMD_ID_RTI_RX_ENABLE_CNF          0x07
  #define RTIS_CMD_ID_RTI_ENABLE_SLEEP_CNF       0x08
  #define RTIS_CMD_ID_RTI_DISABLE_SLEEP_CNF      0x09
  #define RTIS_CMD_ID_RTI_UNPAIR_CNF             0x0A
  #define RTIS_CMD_ID_RTI_UNPAIR_IND             0x0B
  #define RTIS_CMD_ID_RTI_PAIR_ABORT_CNF         0x0C
  #define RTIS_CMD_ID_RTI_RESET_IND              0x0D
  #define RTIS_CMD_ID_RTI_IR_IND				 0xA0

#ifdef ZRC20_PROFILE
  #define RTIS_CMD_ID_RTI_BIND_CNF                     0x30
  #define RTIS_CMD_ID_RTI_SEND_PROFILE_CMD_CNF         0x31
  #define RTIS_CMD_ID_RTI_BIND_IND                     0x32
  #define RTIS_CMD_ID_RTI_START_VALIDATION_IND         0x33
  #define RTIS_CMD_ID_RTI_GET_VALIDATION_STATUS_IND    0x34
  #define RTIS_CMD_ID_RTI_POLL_CONFIG_CNF              0x35
  #define RTIS_CMD_ID_RTI_IDENTIFICATION_CONFIG_CNF    0x36
  #define RTIS_CMD_ID_RTI_POLL_CNF                     0x37
  #define RTIS_CMD_ID_RTI_KEY_EXCHANGE_CNF             0x38
  #define RTIS_CMD_ID_RTI_GET_ATTRIBUTE_REQ            0x39
  #define RTIS_CMD_ID_RTI_SET_ATTRIBUTE_REQ            0x3a
  #define RTIS_CMD_ID_RTI_SEND_IRDB_VENDOR_SUPPORT_CNF 0x3b
  #define RTIS_CMD_ID_RTI_SEND_MAPPABLE_ACTIONS_CNF    0x3c
  #define RTIS_CMD_ID_RTI_GET_ACTION_MAPPINGS_CNF      0x3d
  #define RTIS_CMD_ID_RTI_HA_SUPPORTED_ANNOUNCE_CNF    0x3e
  #define RTIS_CMD_ID_RTI_PULL_HA_ATTRIBUTES_CNF       0x3f
  #define RTIS_CMD_ID_RTI_POLL_IND                     0x40
  #define RTIS_CMD_ID_RTI_UNBIND_CNF                   0x41
  #define RTIS_CMD_ID_RTI_UNBIND_IND                   0x42
  #define RTIS_CMD_ID_RTI_BIND_ABORT_CNF               0x43
#endif //ZRC20_PROFILE

#if (defined FEATURE_MSO) && (FEATURE_MSO == TRUE)
  //MSO Profile Command
  #define RTIS_CMD_ID_RTI_SEND_MSO_COMMAND_CNF      0x0F

  #define RTIS_CMD_ID_RTI_PAIR_IND                  0x30
  #define RTIS_CMD_ID_RTI_START_VALIDATION_IND      0x31
  #define RTIS_CMD_ID_RTI_GET_VALIDATION_STATUS_IND 0x32
  #define RTIS_CMD_ID_RTI_GET_VALIDATION_STATUS_RSP 0x33
  #define RTIS_CMD_ID_RTI_BIND_PARAMS_IND           0x34
  #define RTIS_CMD_ID_RTI_BIND_PARAMS_RSP           0x35
  #define RTIS_CMD_ID_RTI_UPDATE_BACKUP_P_ENTRY     0x36
#endif //(defined FEATURE_MSO) && (FEATURE_MSO == TRUE)

  // RTI States
  enum
  {
    NPI_STATE_INIT,
    NPI_STATE_READY,
    NPI_STATE_NETWORK_LAYER_BRIDGE
  };

  /**************************************************************************************************
   * DEFINES
   **************************************************************************************************/

#define NPI_SERVER_CONNECT_RETRY_SECONDS  2
#define NPI_SERVER_CONNECT_ATTEMPTS_MAX    ((300 + (NPI_SERVER_CONNECT_RETRY_SECONDS-1)) / NPI_SERVER_CONNECT_RETRY_SECONDS) // Give up after 5 minutes (macro rounds up, so minimum of 5 minutes)

#define NPI_IPC_CLIENT_SYNCH_TIMEOUT					2	// 1 second timeout on synchronous request

#ifdef __cplusplus
}
#endif

#endif // NPI_IPC_CLIENT_H
