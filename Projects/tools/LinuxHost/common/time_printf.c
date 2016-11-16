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
#include <pthread.h>
#include "tiLogging.h"

#define NSECS_PER_SEC         1000000000L
#define NSECS_PER_USEC        1000L
#define NSECS_PER_HALF_USEC   (NSECS_PER_USEC/2)
#define SECS_PER_MINUTE       60
#define SECS_PER_HOUR         (60*SECS_PER_MINUTE)

static struct timespec  gStartTimePrint = {0,0};
static struct timespec  gPrevTimePrint = {0,0};
static pthread_mutex_t  timePrintMutex = PTHREAD_MUTEX_INITIALIZER; // WARNING: Non-recursive b/c there is no portable recursive initializer.
static pthread_mutex_t  timePrintOutputMutex = PTHREAD_MUTEX_INITIALIZER; // WARNING: Non-recursive b/c there is no portable recursive initializer.

static void vtime_printf_always_localized(struct timespec const *callersStartTime, struct timespec const *callersCurrentTime, struct timespec *callersPrevTime, char const *fmt, va_list args);

#ifdef __DEBUG_TIME__

int __DEBUG_TIME_ACTIVE = 0;

/**************************************************************************************************
 * @fn      time_printf_start
 * @brief   Forces global start time variable used by all other functions to the current time and
 *          prints a message indicating that it's being done.
 *				Only available if __DEBUG__TIME__ is defined.
 *
 * @return  NONE
 **************************************************************************************************
 */
void time_printf_start(void)
{
	pthread_mutex_lock(&timePrintMutex);
	// Just set to zero.  It will get (re)initialized when the log message is printed.
	gStartTimePrint.tv_sec = gStartTimePrint.tv_nsec = 0;
	// Unfortunately, statically initialized mutex isn't recursive, so we
	// must release it before printing so print function can acquire it.
	pthread_mutex_unlock(&timePrintMutex);

	time_printf_always_localized(NULL, NULL, NULL, "%s", "(Re-)Initialized time_printf start time.\n");
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
void time_printf(char const *fmt, ...)
{
	if (__DEBUG_TIME_ACTIVE)
	{
		va_list args;
		va_start(args, fmt);
  		vtime_printf_always_localized(NULL, NULL, NULL, fmt, args);
		va_end(args);
	}
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
#define TIMING_PREFIX_CONST_FMT  "[TIMING] [%.3d:%02d:%02ld.%06ld (+%ld.%06ld)] "

void time_printf_always(char const *fmt, ...)
{
	va_list args;
	va_start(args, fmt);
	vtime_printf_always_localized(NULL, NULL, NULL, fmt, args);
	va_end(args);
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
 * @param *callersStartTime   - Start time set by caller.  If null, uses global time print start time.
 * @param *callersCurrentTime - Current time set by caller.  If null, gets current time using global time print current time
 * @param *callersPrevTime    - Previous time to diff, set by caller. If null, uses global time print previous time.
 * @param *str                - String to print.  If pointer is null, prints "<null>".
 *
 * output parameters
 *
 * @param *callersPrevTime    - Gets set to *callersCurrentTime before returning
 *
 * @return NONE
 **************************************************************************************************
 */
void time_printf_always_localized(struct timespec const *callersStartTime, struct timespec const *callersCurrentTime, struct timespec *callersPrevTime, char const *fmt, ...)
{
	va_list args;
	va_start(args, fmt);
	vtime_printf_always_localized(callersStartTime, callersCurrentTime, callersPrevTime, fmt, args);
	va_end(args);
}

static void vtime_printf_always_localized(struct timespec const *callersStartTime, struct timespec const *callersCurrentTime, struct timespec *callersPrevTime, char const *fmt, va_list args)
{
	long diffPrevNanos;
	long diffPrevSecs;
	long elapsedNanos;
	long elapsedSeconds;
	int  elapsedMinutes;
	int  elapsedHours;
	struct timespec curTimePrint;
#ifdef __DEBUG_TIME__ // If enabled, then include the realtime clock info, too, just like normal lpfrLogging.
	struct timespec realTime;
	struct tm real_tm;
	long   real_usecs;
	clock_gettime(CLOCK_REALTIME, &realTime);
#endif

	if (!callersCurrentTime)
	{
		clock_gettime(CLOCK_MONOTONIC, &curTimePrint);
		callersCurrentTime = &curTimePrint;
	}

	if (!callersPrevTime)
		callersPrevTime = &gPrevTimePrint;



	// Acquire mutex before working with VALUES of any global variables.
	pthread_mutex_lock(&timePrintMutex);

	if (!callersStartTime) // If passed in, believe it - only set/initialize it if using our static one.
	{
		if (gStartTimePrint.tv_sec == 0 && gStartTimePrint.tv_nsec == 0)
			gStartTimePrint = *callersCurrentTime;

		callersStartTime = &gStartTimePrint;
	}

	diffPrevSecs = (callersCurrentTime->tv_sec - callersPrevTime->tv_sec);
	diffPrevNanos = (callersCurrentTime->tv_nsec - callersPrevTime->tv_nsec);
	if (diffPrevNanos < 0)
	{
		diffPrevNanos += NSECS_PER_SEC;
		--diffPrevSecs;
	}

	elapsedSeconds = (callersCurrentTime->tv_sec - callersStartTime->tv_sec);
	elapsedNanos = (callersCurrentTime->tv_nsec - callersStartTime->tv_nsec);
	// Done using callersPrevTime, so okay to update it now
	*callersPrevTime = *callersCurrentTime;

	// Everything is now local, so okay to unlock mutex so we're blocking for less time
	pthread_mutex_unlock(&timePrintMutex);

	// Do final calculations with local variables, then print

	if (elapsedNanos < 0)
	{
		elapsedNanos += NSECS_PER_SEC;
		--elapsedSeconds;
	}
	elapsedHours = elapsedSeconds/SECS_PER_HOUR;
	elapsedSeconds -= (elapsedHours * SECS_PER_HOUR);
	elapsedMinutes = elapsedSeconds/SECS_PER_MINUTE;
	elapsedSeconds -= (elapsedMinutes * SECS_PER_MINUTE);

#ifdef __DEBUG_TIME__
	real_tm = *gmtime(&realTime.tv_sec);
	real_usecs = realTime.tv_nsec/1000;
#endif

	if (!fmt)
		fmt="<null>\n"; // In case

	// Don't let this output get interrupted/intermixed with another, so use another mutex.
	// We could use the same mutex, but let's not block prep during output.  Just block
	// other output during our output.  IDEALLY we'd make one call, but there is no
	// portable way to muck around with a va_list to insert additional arguments, and
	// we really don't want to waste time always create a new format that has our data
	// already in the string.  Better to use the mutex and we'll likely not even block
	// another thread anyway.
	pthread_mutex_lock(&timePrintOutputMutex);

	// Make header match format of other logging from tiLogging.h,
	// substituting "TIMING" for where other logging shows the logging level.
#ifdef __DEBUG_TIME__
	fprintf(LOG_DESTINATION_FP, _PREPEND_TO_LOG(TIMING_PREFIX_CONST_FMT, real_tm, real_usecs),
#else
	fprintf(LOG_DESTINATION_FP, _PREPEND_TO_LOG(TIMING_PREFIX_CONST_FMT),
#endif
			elapsedHours,
			elapsedMinutes,
			elapsedSeconds,
			(elapsedNanos + NSECS_PER_HALF_USEC - 1) / NSECS_PER_USEC, // Convert to microseconds, rounding up where >500ns = 1us
			diffPrevSecs,
			(diffPrevNanos + NSECS_PER_HALF_USEC - 1) / NSECS_PER_USEC); // Convert to microseconds, rounding up where >500ns = 1us

	vfprintf(LOG_DESTINATION_FP, fmt, args);
	// And release our output mutex.
	pthread_mutex_unlock(&timePrintOutputMutex);
}

