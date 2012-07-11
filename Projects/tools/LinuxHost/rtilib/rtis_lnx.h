/**************************************************************************************************
  Filename:       rtis_lnx.h
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
#ifndef RTIS_LNX_H
#define RTIS_LNX_H


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

  /* RTI_LNX includes */
  #include "rti_lnx.h"
  #include "rti_lnx_constants.h"

  /* Initialize RTI Surrogate
   * devpath - serial interface device path, or IPaddress:port
   * returns TRUE when initialized successfully. Otherwise, FALSE */
  int RTIS_Init(const char *devpath);

  /* Close RTI Surrogate */
  void RTIS_Close(void);


  /**************************************************************************************************
   * TYPEDEFS
   **************************************************************************************************/

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

  // RTI States
  enum
  {
    RTIS_STATE_INIT,
    RTIS_STATE_READY,
    RTIS_STATE_NETWORK_LAYER_BRIDGE
  };

  /**************************************************************************************************
   * DEFINES
   **************************************************************************************************/

  // Macro to generate little endian 32 bit word to be used as NPI interface
  #define RTI_SET_ITEM_WORD( pVal, attrib )                  \
    (pVal)[0] = (uint8)( ((uint32)(attrib) >>  0) & 0xFF );  \
    (pVal)[1] = (uint8)( ((uint32)(attrib) >>  8) & 0xFF );  \
    (pVal)[2] = (uint8)( ((uint32)(attrib) >> 16) & 0xFF );  \
    (pVal)[3] = (uint8)( ((uint32)(attrib) >> 24) & 0xFF );

  // Macro to generate little endian 16 bit value to be used as NPI interface
  #define RTI_SET_ITEM_HALFWORD( pVal, attrib )              \
    (pVal)[0] = (uint8)( ((uint16)(attrib) >>  0) & 0xFF );  \
    (pVal)[1] = (uint8)( ((uint16)(attrib) >>  8) & 0xFF );

#ifdef __cplusplus
}
#endif

#endif // RTIS_LNX_H
