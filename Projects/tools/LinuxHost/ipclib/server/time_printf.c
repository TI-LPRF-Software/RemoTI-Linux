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


#include <sys/time.h>
#include <stdio.h>
#include "time_printf.h"

static struct timespec gStartTimePrint = {0,0}, gPrevTimePrint = {0,0};

#ifdef __DEBUG_TIME__

int __DEBUG_TIME_ACTIVE = 0;

/**************************************************************************************************
 * @fn      time_printf_start
 * @brief   Forces global stsartTime variable used by time_printf and time_printf_always to current time
 *				Only avaialbe if __DEBUG__TIME__ is defined.
 *
 * input parameters
 *
 * @param *str - String to print.  If pointer is null, prints "<null>".
 *
 * @return  NONE
 **************************************************************************************************
 */
void time_printf_start(void)
{
	clock_gettime(CLOCK_MONOTONIC, &gStartTimePrint);
}


/**************************************************************************************************
 * @fn      time_printf
 * @brief   Prints string prefixed by relative time and delta from previous call
 *				Only avaialbe if __DEBUG__TIME__ is defined and only prints of __DEBUG_TIME_ACTIVE == 1.
 * 
 * input parameters
 *
 * @param *str - String to print.  If pointer is null, prints "<null>".
 *
 * @return  NONE
 **************************************************************************************************
 */
void time_printf(char const *str)
{
	if (__DEBUG_TIME_ACTIVE == 1)
		time_printf_always(str);
}

#endif //__DEBUG_TIME__

/**************************************************************************************************
 * @fn      time_printf_always
 * @brief   Prints string prefixed by relative time and delta from previous call
 *          Same as time_printf, but always available (not dependant on #defines or settings.
 *
 * input parameters
 *
 * @param *str - String to print.  If pointer is null, prints "<null>".
 *
 * @return  NONE
 **************************************************************************************************
 */
void time_printf_always(char const *str)
{
	struct timespec curTimePrint;
	clock_gettime(CLOCK_MONOTONIC, &curTimePrint);

	if (gStartTimePrint.tv_sec == 0 && gStartTimePrint.tv_nsec == 0)
		gStartTimePrint = curTimePrint;

	time_printf_always_localized(str, &gStartTimePrint, &curTimePrint, &gPrevTimePrint);
}

/**************************************************************************************************
 * @fn      time_printf_always_localized
 * @brief   Prints string prefixed by relative time and delta from previous call
 *          Similar to time_printf_always, but allows caller to pass in its own times.  
 *          If any of the passed in times are null, the relevant global one is used.
 *          NOTE: This sets *callersPrevTime to *callersStartTime before returning.
 *
 * input parameters
 *
 * @param *str                - String to print.  If pointer is null, prints "<null>".
 * @param *callersStartTime   - Start time set by caller.  If null, uses global time print start time.
 * @param *callersCurrentTime - Current time set by caller.  If null, gets current time using global time print current time
 * @param *callersPrevTime    - Previous time to diff, set by caller. If null, uses global time print previous time.
 *
 * output parameters
 *
 * @param *callersPrevTime    - Gets set to *callersCurrentTime before returning
 *
 * @return NONE
 **************************************************************************************************
 */
void time_printf_always_localized(char const *str, struct timespec const *callersStartTime, struct timespec const *callersCurrentTime, struct timespec *callersPrevTime)
{
	long diffPrevFraction;
	long diffPrevSecs;
	long elapsedSeconds;
	int  elapsedMinutes;
	int  elapsedHours;
	int  t;
	struct timespec curTimePrint;

	if (!callersCurrentTime)
	{
		clock_gettime(CLOCK_MONOTONIC, &curTimePrint);
		callersCurrentTime = &curTimePrint;
	}
	if (!callersStartTime)
	{
		if (gStartTimePrint.tv_sec == 0 && gStartTimePrint.tv_nsec == 0)
			gStartTimePrint = *callersCurrentTime;
		callersStartTime = &gStartTimePrint;
	}

	if (!callersPrevTime)
		callersPrevTime = &gPrevTimePrint;

	if (callersCurrentTime->tv_nsec >= callersPrevTime->tv_nsec)
	{
		diffPrevFraction = callersCurrentTime->tv_nsec - callersPrevTime->tv_nsec;
		t = 0;
	}
	else
	{
		diffPrevFraction = (callersCurrentTime->tv_nsec + 1000000000) - callersPrevTime->tv_nsec;
		t = 1;
	}
	diffPrevSecs = (callersCurrentTime->tv_sec - callersPrevTime->tv_sec - t),

	elapsedSeconds = (callersCurrentTime->tv_sec - callersStartTime->tv_sec);
	elapsedHours = elapsedSeconds/3600;
	elapsedSeconds -= (elapsedHours * 3600);
	elapsedMinutes = elapsedSeconds/60;
	elapsedSeconds -= (elapsedMinutes * 60);

	//            debug_
	fprintf(stderr, "[%.3d:%02d:%02ld.%06ld (+%ld.%06ld)] %s",
			elapsedHours,
			elapsedMinutes,
			elapsedSeconds,
			(callersCurrentTime->tv_nsec + 499 )/1000, // Convert to microseconds, rounding up where >500ns = 1us
			diffPrevSecs,
			(diffPrevFraction + 499 )/1000, // Convert to microseconds, rounding up where >500ns = 1us
			str ? str : "<null>");

	*callersPrevTime = *callersCurrentTime;
}
