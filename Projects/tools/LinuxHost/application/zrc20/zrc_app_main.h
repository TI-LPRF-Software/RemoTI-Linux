/**************************************************************************************************
  Filename:       zrc_app_main.h

  Description:    This file defines linux specific interface to Network Processor Interface
                  module.


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
#ifndef ZRC_MAIN_LNX_H
#define ZRC_MAIN_LNX_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include "hal_types.h"
#include "hal_defs.h"
#include "zrc_app.h"


  /////////////////////////////////////////////////////////////////////////////
  // Typedefs
  
extern void DispMenuReady( void );
extern void DispMenuInit( void );

// Global variables

#define MAIN_INPUT_RELEASED		0
#define MAIN_INPUT_READY		1

extern char *imagePath;

struct consoleInput_s 
{
	char latestCh;			// latest character from stdin
	char latestStr[128];	// latest string of characters from stdin
	char handle;			// handle to allow application to say when the character is read;
							//    set to 1 by main thread when character is ready
							//    set to 0 by application thread when character is processed
};

struct consoleInput_s consoleInput;

uint8  main_set_event	(uint8 threadId, uint16 event);
uint8  main_clear_event	(uint8 threadId, uint16 event);
uint16 main_get_event	(uint8 threadId);

#ifdef __cplusplus
}
#endif

#endif // ZRC_MAIN_LNX_H
