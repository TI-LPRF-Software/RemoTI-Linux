/**************************************************************************************************
  Filename:       npi_lnx_serial_configuration.h
  Revised:        $Date: 2011-11-23 12:02:49 -0800 (Wed, 23 Nov 2011) $
  Revision:       $Revision: 108 $

  Description:    This file defines linux specific interface to Network Processor Interface
                  module.


  Copyright (C) {2015} Texas Instruments Incorporated - http://www.ti.com/


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
#ifndef NPI_SER_CFG_LNX_H
#define NPI_SER_CFG_LNX_H

#ifdef __cplusplus
extern "C" {
#endif

#include "npi_lnx_spi.h"
#include "npi_lnx_i2c.h"
#include "npi_lnx_uart.h"

#if !defined PACK_1
#define PACK_1
#endif

#define SERIAL_CFG_MAX_NUM_OF_GPIOS			5

// To be compatible with MS and unix native target
// declare pragma for structure packing
#if defined(_MSC_VER) || defined(unix) || (defined(__ICC430__) && (__ICC430__==1))
#pragma pack(1)
#endif
  /////////////////////////////////////////////////////////////////////////////
  // Typedefs

  PACK_1 typedef struct ATTR_PACKED
  {
	  char port[128];
	  char devPath[128];
	  char logPath[128];
	  halGpioCfg_t gpioCfg[SERIAL_CFG_MAX_NUM_OF_GPIOS];
	  uint8 devIdx;
	  uint8 debugSupported;
	  union {
		  npiSpiCfg_t npiSpiCfg;
		  npiI2cCfg_t npiI2cCfg;
		  npiUartCfg_t npiUartCfg;
	  } serial;
  } npiSerialCfg_t;


  /////////////////////////////////////////////////////////////////////////////
  // Interface function prototypes

  /******************************************************************************
   * @fn        getSerialConfiguration
   *
   * @brief     This function populates the serial configuration parameters as
   * 			read from the configuration file.
   *
   * input parameters
   *
   * @param		configFilePath	-- configuration file path
   * @param     serialCfg 		-- pointer to structure holding all parameters
   *
   * output parameters
   *
   * @param     serialCfg 		-- pointer to structure holding all parameters
   *
   * @return     TRUE if the configuration parameters were read successfully.
   *             FALSE, otherwise.
   ******************************************************************************
   */
  extern int getSerialConfiguration(const char *configFilePath, npiSerialCfg_t *serialCfg);

  /**************************************************************************************************
   *
   * @fn          SerialConfigParser
   *
   * @brief       This function searches for a string a returns its value
   *
   * input parameters
   *
   * @param          configFilePath   - path to configuration file
   * @param          section          - section to search for
   * @param          key                                                         - key to return value of within section
   *
   * output parameters
   *
   * None.
   *
   * @return      None.
   *
   **************************************************************************************************/
  extern int SerialConfigParser(FILE* serialCfgFd, const char* section, const char* key, char* resultString);

#ifdef __cplusplus
}
#endif

#endif // NPI_SER_CFG_LNX_H
