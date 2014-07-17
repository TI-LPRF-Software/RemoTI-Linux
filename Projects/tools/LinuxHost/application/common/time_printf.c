/*
 * time_printf.c
 *
 *  Created on: Feb 5, 2014
 *      Author: root

  Copyright (C) {2014} Texas Instruments Incorporated - http://www.ti.com/


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


#ifdef __DEBUG_TIME__

#include <sys/time.h>
#include <stdio.h>
#include "time_printf.h"

struct timeval curTime, prevTime, startTime;

int __DEBUG_TIME_ACTIVE = 0;

void time_printf_start()
{
	gettimeofday(&startTime, NULL);
}

void time_printf(char *str)
{
	if (__DEBUG_TIME_ACTIVE == 1)
	{
		//            debug_
		gettimeofday(&curTime, NULL);
		long int diffPrev;
		int t = 0;
		if (curTime.tv_usec >= prevTime.tv_usec)
		{
			diffPrev = curTime.tv_usec - prevTime.tv_usec;
		}
		else
		{
			diffPrev = (curTime.tv_usec + 1000000) - prevTime.tv_usec;
			t = 1;
		}

		int hours = ((curTime.tv_sec - startTime.tv_sec) - ((curTime.tv_sec - startTime.tv_sec) % 3600))/3600;
		int minutes = ((curTime.tv_sec - startTime.tv_sec) - ((curTime.tv_sec - startTime.tv_sec) % 60))/60;
		//            debug_
		printf("[%.3d:%.2d:%.2d.%.6ld (+%ld.%6ld)] %s ",
				hours,											// hours
				minutes,										// minutes
				(int)(curTime.tv_sec - startTime.tv_sec) % 60,		// seconds
				(long int)curTime.tv_usec,
				curTime.tv_sec - prevTime.tv_sec - t,
				diffPrev,
				str);
		prevTime = curTime;
	}
}
#endif //__DEBUG_TIME__
