/**************************************************************************************************
  Filename:       RTI_main_linux.h
  Revised:        $Date: 2012-03-21 17:37:33 -0700 (Wed, 21 Mar 2012) $
  Revision:       $Revision: 246 $

  Description:    This file defines linux specific interface to Network Processor Interface
                  module.


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
#ifndef RTI_MAIN_LNX_H
#define RTI_MAIN_LNX_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include "hal_types.h"
#include "hal_defs.h"
#include "simple_app.h"

  /////////////////////////////////////////////////////////////////////////////
  // Typedefs


extern void DispMenuInit( void );
extern void DispCFGProfileIDmenu(void);
extern void DispCFGCurrentCfg(appDevInfo_t appCfg, uint16 nwkAddr, uint16 panId, uint8 *ieeAddr);
extern void DispCFGNodeCapMenu(void);
extern void DispCFGProfileIDMenu(void);
extern void DispCFGdevTypesMenu(void);
extern void DispCFGtgtTypesMenu(void);
extern void DispCFGVendorIDMenu(void);

extern void DispMenuReady( void );
extern void DispStringList( char** strList, uint16 len );
extern void DispNumberedStringList( char** strList, uint16 len );

extern void DispControlAttenuatorMenu(void);

extern void DispPhyTestModeMenu(void);
extern void DispPhyTestModeActiveMenu(void);
extern void DispPhyTestModeRxMenu(void);
extern void DispPhyTestModeTxRawMenu(void);
extern void DispPhyTestModeTxModulatedMenu(void);

extern void DispSendDataDestIndexMenu();
extern void DispSendDataPayloadMenu();
extern void DispSendDataTxOptionsMenu();
extern void DispSendDataProfileIDMenu();
extern void DispSendDataCurrentCfg(appSendData_t appSendData_s);
extern void DispSendDataMenu();

extern void DisplayConfigurationParameters ( void );

extern void DisplayPairingTable(rcnNwkPairingEntry_t *pEntry);

// Global variables

#define RTI_MAIN_INPUT_RELEASED		0
#define RTI_MAIN_INPUT_READY		1

struct consoleInput_s 
{
	char latestCh;			// latest character from stdin
	char latestStr[128];	// latest string of characters from stdin
	char handle;			// handle to allow application to say when the character is read;
							//    set to 1 by main thread when character is ready
							//    set to 0 by application thread when character is processed
};

struct consoleInput_s consoleInput;

// Constants defined in RTI_main_linux.c that are globally available
extern const char * const rtiStatus_list[];
extern const char * const rtiProfileId_list[];
extern const char * const rtiVendorId_list[];
extern const char * const rtiDevType_list[];
extern const char * const testMode_startCondition_list[];

#ifdef __cplusplus
}
#endif

#endif // RTI_MAIN_LNX_H
