/**************************************************************************************************
 Filename:       configParser.h

 Description:    Linux Host application tool

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
#include <stdlib.h>
#include <string.h>
#include "hal_types.h"
#include "hal_defs.h"

typedef struct {
	uint8 big;		// Debug setting BIG on NPI_Server
	uint8 timer;	// Debug setting TIMER on NPI_Server
	uint8 client;		// Debug setting on NPI Client
	uint8 app;		// Debug setting on application
} debugSetting_s;

typedef struct {
	char *ip_addr;
	char *port;
	debugSetting_s debug;
	uint8 freqAgilityEnabled;
	uint8 macChannel;
} appBaseSetting_s;


int ConfigParserInit(const char *configFilePath, const char *shadowPath);
void ConfigParserClose( void );
int ConfigParserGetBaseSettings(appBaseSetting_s *baseSettings);
int ConfigParserFromPath(const char *configFilePath, const char* section, const char* key, char* resultString);
int ConfigParserFromFd(FILE* cfgFd, const char* section, const char* key, char* resultString);
int ConfigParserSetGetFromFd(FILE* cfgFd, const char* section, const char* key, char* resultString, char* newValue);

int ConfigParserSet(const char *configFilePath, const char *shadowPath, const char* section, const char* key, char* newValue);
int ConfigParser(const char* section, const char* key, char* resultString);
