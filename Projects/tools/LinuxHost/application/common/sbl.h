   /**************************************************************************************************
  Filename:       sbl.h
  Revised:        $Date: 2012-03-21 17:37:33 -0700 (Wed, 21 Mar 2012) $
  Revision:       $Revision: 246 $

  Description:    This file defines Serial Bootloader application interface.


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

#ifndef SBL_H
#define SBL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "hal_types.h"
#include "rti_lnx.h"
#include "common_app.h"

/////////////////////////////////////////////////////////////////////////////
  // Defines

// Application States
enum {
	SBL_STATE_INIT, // Initial state - prior to initialization completion
	SBL_STATE_INIT_SERIAL_BOOT,
	SBL_STATE_SERIAL_BOOT,
	SBL_STATE_RESET, // Initial state - However, RNP is in Serial Boot mode.
	SBL_STATE_READY, // Ready state - ready to pair, ready to send data
};

/////////////////////////////////////////////////////////////////////////////
// Function declarations

int SBL_Init(const char *imagePath);
int SBL_Execute(void);
int SBL_CheckForUpdate(swVerExtended_t *RNPversion);
int SBL_IsDeviceInSBLMode(void);

#ifdef __cplusplus
}
#endif

#endif // SBL_H
