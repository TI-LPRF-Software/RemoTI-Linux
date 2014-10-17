/**************************************************************************************************
  Filename:       timer.c

  Description:    Linux timer module, modeled after OSAL_Timer.c

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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <errno.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>
#include <math.h>

#include "timer.h"
#include "hal_defs.h"

#ifdef __DEBUG_TIME__
#include "time_printf.h"
#endif //__DEBUG_TIME__


#ifdef TIMER_DEBUG
#define TIMER_TICKS				100
#define debug_printf(fmt, ...) printf( fmt, ##__VA_ARGS__)
#else
#define debug_printf(fmt, ...)
#endif

#define TIMER_MAX			(0xFFFF * 1000 + 1)

static void timerInitSyncRes(void);
static void *timerThreadFunc(void *ptr);

//THREAD and MUTEX
static pthread_mutex_t  timerThreadMutex;
static pthread_mutex_t  timerInitMutex;
static pthread_mutex_t  timerMutex;
static pthread_mutex_t  timerEventMutex;

// conditional variable to notify that a TIMER is set
static pthread_cond_t timerSetCond;

static pthread_t        timerThreadId;
static int timerThreadTerminate;

static timer_thread_s *timerThreadTbl;
static uint16 timerNumOfThreads;

int timer_init(uint16 numOfThreads)
{
	timerThreadTbl = (timer_thread_s *) malloc(sizeof(timer_thread_s) * numOfThreads);
	memset(timerThreadTbl, 0, sizeof(timer_thread_s) * numOfThreads);

	timerNumOfThreads = numOfThreads;

	timerThreadTerminate = 0;
	timerInitSyncRes();

	if(pthread_create(&timerThreadId, NULL, timerThreadFunc, NULL))
	{
		// thread creation failed
		printf("Failed to create timer thread\n");
		return -1;
	}

	return 0;
}

static void *timerThreadFunc(void *ptr)
{
	/* lock mutex in order not to lose signal */
	pthread_mutex_lock(&timerThreadMutex);
	int i, j, tryLockFirstTimeOnly = 0, waitRes = TRUE;
	long int diffPrev, decrementValue = 0, minimumTimeout = 0, waitMargin = 2000, active = FALSE;
	int t = 0, res;

	struct timeval curTime, startTime, prevTime;
	struct timespec waitToTime, remWaitTime;

	gettimeofday(&startTime, NULL);
	prevTime = startTime;

#ifdef TIMER_DEBUG
	printf("Timer Thread Started \n");
	printf("\t %d threads supported \n", timerNumOfThreads);
#endif //TIMER_DEBUG

	while(!timerThreadTerminate)
	{
		if (tryLockFirstTimeOnly == 0)
		{
			// Lock mutex
			debug_printf("[MUTEX] Lock TIMER Mutex (Handle)\n");
			int mutexRet = 0, writeOnce = 0;
			while ( (mutexRet = pthread_mutex_trylock(&timerMutex)) == EBUSY)
			{
				if (writeOnce == 0)
				{
					debug_printf("[MUTEX] TIMER Mutex (Handle) busy");
					fflush(stdout);
					writeOnce++;
				}
				else
				{
					writeOnce++;
					if ( (writeOnce % 1000) == 0)
					{
						debug_printf(".");
					}
					if (writeOnce > 0xEFFFFFF0)
						writeOnce = 1;
					fflush(stdout);
				}
			}
			debug_printf("\n[MUTEX] TIMER Lock (Handle) status: %d\n", (int)mutexRet);
			tryLockFirstTimeOnly = 1;

			debug_printf("Timer Table\n");
			int i, j;
			for (i = 0; i < timerNumOfThreads; i++)
			{
				debug_printf("Thread %d\n", i);
				debug_printf("\t Events:\t0x%.8X\n", timerThreadTbl[i].eventFlag);
				debug_printf("\t Enabled:\t0x%.8X\n", timerThreadTbl[i].timerEnabled);
				debug_printf("\t Timeouts:\n");
				for (j = 0; j < 32; j++)
				{
					debug_printf("\t\t %d:\t%ld\n", j, timerThreadTbl[i].timeoutValue[j]);
				}
			}
			debug_printf("\n");
		}

		// Get current time to compensate for processing.
		gettimeofday(&curTime, NULL);

		if (curTime.tv_usec >= prevTime.tv_usec)
		{
			diffPrev = curTime.tv_usec - prevTime.tv_usec;
			t = 0;
		}
		else
		{
			diffPrev = (curTime.tv_usec + 1000000) - prevTime.tv_usec;
			t = 1;
		}

		decrementValue = diffPrev  + 1000000 * (curTime.tv_sec - prevTime.tv_sec - t);

		// minimumTimeout is the value we expected to wait. If the timer indicates active
		// it means that decrementValue should equal minimumTimeout. The difference is the
		// extra time it took to get back running. We can compensate for this by waking up
		// earlier and then use less CPU friendly wait mechanism for the remaining wait time.
		// The adjustment time is also updated if we're still waking up too late.
		if (active == TRUE)
		{
			debug_printf("Timer Table\n");
			int i, j;
			for (i = 0; i < timerNumOfThreads; i++)
			{
				debug_printf("Thread %d\n", i);
				debug_printf("\t Events:\t0x%.8X\n", timerThreadTbl[i].eventFlag);
				debug_printf("\t Enabled:\t0x%.8X\n", timerThreadTbl[i].timerEnabled);
				debug_printf("\t Timeouts:\n");
				for (j = 0; j < 32; j++)
				{
					debug_printf("\t\t %d:\t%ld\n", j, timerThreadTbl[i].timeoutValue[j]);
				}
			}
			debug_printf("\n");
			long int timeWaitedDifference = (decrementValue - minimumTimeout);
			debug_printf("[TIMER] Decrementing value %dus, diff %dus\n", (int)decrementValue, (int)timeWaitedDifference);
			if ( timeWaitedDifference > 250)
			{
				// Waited too long, must increase waitMargin, add the difference
				waitMargin += (decrementValue - minimumTimeout);
				debug_printf("[TIMER] Adjusting waitMargin %dus, because decrementValue = %dus and minimumTimeout = %dus\n",
						(int)waitMargin, (int)decrementValue, (int)minimumTimeout);
			}
			else if ( (timeWaitedDifference < -200) && (timeWaitedDifference > (-waitMargin)) )
			{
				// Wait the remainder with more accurate means if it's less than the margin.
				// Use nanosleep to wait the remainder
				waitToTime.tv_sec = 0;
				waitToTime.tv_nsec = 1000;

				while (timeWaitedDifference < 0)
				{
					waitRes = nanosleep(&waitToTime, &remWaitTime);
					while (waitRes != 0)
					{
						if (errno == EINTR)
						{
							// Restart timer with the remainder
							waitRes = nanosleep(&remWaitTime, &remWaitTime);
						}
						else
						{
							perror("Nanosleep failed:");
							break;
						}
					}
					// Update decrement value

					// Get current time to compensate for processing.
					gettimeofday(&curTime, NULL);

					if (curTime.tv_usec >= prevTime.tv_usec)
					{
						diffPrev = curTime.tv_usec - prevTime.tv_usec;
						t = 0;
					}
					else
					{
						diffPrev = (curTime.tv_usec + 1000000) - prevTime.tv_usec;
						t = 1;
					}

					decrementValue = diffPrev  + 1000000 * (curTime.tv_sec - prevTime.tv_sec - t);
					timeWaitedDifference = (decrementValue - minimumTimeout);
				}
				debug_printf("[TIMER] Updated decrementing value %dus\n", (int)decrementValue);
			}
		}

		active = FALSE;
		minimumTimeout = TIMER_MAX;

		// Do all the timer calculation for all threads
		for( i = 0 ; i < timerNumOfThreads; i++)
		{
			// Decrement all enabled counters, and check if they have timed out
			for (j = 0; j < 32; j++)
			{
				// Check if event j for timer (for thread) i is enabled
				if (timerThreadTbl[i].timerEnabled & BV(j))
				{
					debug_printf("[TIMER] Decrementing %.4ldus for ThreadID 0x%.2X, event 0x%.8X. Time left before decrementing: %d\n",
								decrementValue,
								i,
								j,
								(int)timerThreadTbl[i].timeoutValue[j]);

					if (timerThreadTbl[i].justKicked & BV(j))
					{
						timerThreadTbl[i].justKicked &= ~(BV(j));
					}
					else
					{
						// Decrement active timer
						timerThreadTbl[i].timeoutValue[j] -= decrementValue;
					}
					// !!! Possible race condition !!!
					if (timerThreadTbl[i].timeoutValue[j] <= 0)
					{
						debug_printf("[TIMER] Timer Expired. \t Thread ID: %.2d \t Event Mask: 0x%.8X\n",
								i, BV(j));
						// timerEnabled can only be set to 0 here
						timerThreadTbl[i].timerEnabled &= ~(BV(j));
						timer_set_event(i, BV(j));
					}
					else if ( timerThreadTbl[i].timeoutValue[j] < minimumTimeout )
					{
						minimumTimeout = timerThreadTbl[i].timeoutValue[j];
						active = TRUE;
					}
				}
			}
		}

		gettimeofday(&prevTime, NULL);

		waitToTime.tv_sec = prevTime.tv_sec;
		waitToTime.tv_nsec = prevTime.tv_usec * 1000;

		// We need to adjust for the inaccuracy of pthread_cond_timedwait
		debug_printf("[TIMER] wait margin %dus, minimumTimeout %dus\n", (int)waitMargin, (int)minimumTimeout);
		if ( (minimumTimeout > (waitMargin + 1)) || (minimumTimeout == 0) )
		{
			if (active == TRUE)
			{
				minimumTimeout -= waitMargin;
			}

			if (minimumTimeout > 1000000)
			{
				// Adjust seconds
				waitToTime.tv_sec += (minimumTimeout / 1000000);
			}
			// Adjust nanoseconds to comply with the timespec struct
			waitToTime.tv_nsec += ((minimumTimeout % 1000000) * 1000);
			if (waitToTime.tv_nsec >= 1000000000 )
			{
				// Fix overflow
				waitToTime.tv_nsec -= 1000000000;
				waitToTime.tv_sec += 1;
			}

			// Conditional wait for a call to TimerSet()
			if ((minimumTimeout <= 0) || (active == FALSE))
			{
				debug_printf("[TIMER][MUTEX] TIMER Wait forever\n");
				res = pthread_cond_wait(&timerSetCond, &timerMutex);

				// Since we have waited an unknown period we set previous time here.
				gettimeofday(&prevTime, NULL);
			}
			else
			{
				debug_printf("[TIMER][MUTEX] Wait %dus (%ds:%dns) for TIMER Set Cond (Handle) signal... effectively releasing lock\n",
						(int)minimumTimeout, (int)(minimumTimeout / 1000000), (int)((minimumTimeout % 1000000) * 1000));
				res = pthread_cond_timedwait(&timerSetCond, &timerMutex, &waitToTime);
				if (res != 0)
				{
					debug_printf("[TIMER][MUTEX] TIMER conditional wait returned with %d\n", res);
					if (res == EINVAL)
					{
						debug_printf("[TIMER][MUTEX] Wait until %lds:%ldns\n",
							waitToTime.tv_sec, waitToTime.tv_nsec);
						// Terminate thread
						timerThreadTerminate = 1;
					}
				}
			}

			// Add the value back so calculations are based on correct numbers.
			if (active == TRUE)
			{
				minimumTimeout += waitMargin;
			}
			debug_printf("[TIMER][MUTEX] TIMER (Handle) has lock\n");
		}
		else
		{
			// Wait time is lower than the margin, so move on in loop, skip sleep
		}
	}

	pthread_mutex_unlock(&timerThreadMutex);

	return 0;
}

/**************************************************************************************************
 *
 * @fn      timer_isActive
 *
 * @brief   Check to see if timer is running
 *
 * @param   threadId - Id of the thread to call at the event.
 * @param	event - event bitmask
 *
 * @return  TRUE is timer is running
 */
uint8 timer_isActive(uint8 threadId, uint32 event)
{
	uint8 isActive = FALSE;

	if (timerThreadTbl[threadId].timerEnabled & event)
	{
		isActive = TRUE;
	}

	return isActive;
}

/**************************************************************************************************
 *
 * @fn      timer_start_timerEx
 *
 * @brief   Modeled after OSAL_start_timerEx.
 * 			To stop timer call with same threadId, and event, and
 * 			set timeout to 0.
 *
 * @param   threadId - Id of the thread to call at the event.
 * @param	event - event bitmask
 * @param	timeout - number of milliseconds to count down
 *
 * @return  void
 */
uint8 timer_start_timerEx(uint8 threadId, uint32 event, uint32 timeout)
{
	uint8 i;
	for (i = 0; i < 32; i++)
	{
		if (event & BV(i))
			break;
	}

	// To avoid race conditions we cannot update timerThreadTbl without mutex lock
	pthread_mutex_lock(&timerMutex);

	// Value is stored in us for better precision
	timerThreadTbl[threadId].timeoutValue[i] = timeout * 1000;
	// Use a 0 value of timeout to disable timer
	if (timeout)
	{
		// Check if timer is already enabled for this event
		if (timerThreadTbl[threadId].timerEnabled & event)
		{
			// It is, so we need to make sure the timer update does not decrement now.
			timerThreadTbl[threadId].justKicked |= event;
		}
		else
		{
			timerThreadTbl[threadId].timerEnabled |= event;
		}
	}
	else
	{
		timerThreadTbl[threadId].timerEnabled &= ~event;
		// Clear event in case it just fired.
		timer_clear_event(threadId, event);
	}

#if (defined __DEBUG_TIME__) && (defined TIMER_DEBUG)
	char str[128];
	snprintf(str, sizeof(str), "[TIMER] %dus timer started for event 0x%.8X and thread %d\n",
			(int)timerThreadTbl[threadId].timeoutValue[i], event, threadId);
	time_printf(str);
#elif defined TIMER_DEBUG
	printf("Timer started for %dus\n", (int)timerThreadTbl[threadId].timeoutValue[i]);
#endif //TIMER_DEBUG

	// Unlock mutex before notifying timer thread
	pthread_mutex_unlock(&timerMutex);

	// Notify timer thread that timer has been set
	pthread_cond_signal(&timerSetCond);

	return 1;
}

uint8 timer_set_event(uint8 threadId, uint32 event)
{
#ifdef TIMER_DEBUG
	printf("[TIMER] Setting event 0x%.2X\n", event);
#endif //TIMER_DEBUG

	//Mutex needs to be used in order to provide access to set/clear event to several thread...
	// Set event in table

	pthread_mutex_lock(&timerEventMutex);
#if (defined __DEBUG_TIME__) && (defined TIMER_DEBUG)
	char str[128];
	snprintf(str, sizeof(str), "[TIMER] Event 0x%.8X set for thread %d\n", event, threadId);
	time_printf(str);
#endif //__DEBUG_TIME__
	timerThreadTbl[threadId].eventFlag |= event;

	// Release resources waiting for this event
	if (sem_post(&event_mutex) < 0)
	{
//		perror("Failed to post event");
	}

	pthread_mutex_unlock(&timerEventMutex);


	return TRUE;
}

uint8 timer_clear_event(uint8 threadId, uint32 event)
{
#ifdef TIMER_DEBUG
	printf("clearing event 0x%.8X\n", event);
#endif //TIMER_DEBUG
	pthread_mutex_lock(&timerEventMutex);
#if (defined __DEBUG_TIME__) && (defined TIMER_DEBUG)
	char str[128];
	snprintf(str, sizeof(str), "[TIMER] Event 0x%.8X cleared for thread %d\n", event, threadId);
	time_printf(str);
#endif //__DEBUG_TIME__
	timerThreadTbl[threadId].eventFlag &= ~event;
	pthread_mutex_unlock(&timerEventMutex);
	return TRUE;
}

uint32 timer_get_event(uint8 threadId)
{
	pthread_mutex_lock(&timerEventMutex);
	pthread_mutex_unlock(&timerEventMutex);
#if (defined __DEBUG_TIME__) && (defined TIMER_DEBUG)
	char str[128];
	snprintf(str, sizeof(str), "[TIMER] Event 0x%.8X read for thread %d\n", timerThreadTbl[threadId].eventFlag, threadId);
	time_printf(str);
#endif //__DEBUG_TIME__
	return timerThreadTbl[threadId].eventFlag;
}


/**************************************************************************************************
 *
 * @fn          timerInitSyncRes
 *
 * @brief
 *
 * input parameters
 *
 * @param       none
 *
 * output parameters
 *
 * None.
 *
 * @return      none
 *
 **************************************************************************************************/
static void timerInitSyncRes(void)
{
	// initialize all mutexes
	if (pthread_mutex_init(&timerThreadMutex, NULL))
	{
		printf("Fail To Initialize Mutex appThreadMutex\n");
		exit(-1);
	}

	if(pthread_mutex_init(&timerInitMutex, NULL))
	{
		printf("Fail To Initialize Mutex timerInitMutex\n");
		exit(-1);
	}

	if(pthread_mutex_init(&timerMutex, NULL))
	{
		printf("Fail To Initialize Mutex timerMutex\n");
		exit(-1);
	}

	if(pthread_mutex_init(&timerEventMutex, NULL))
	{
		printf("Fail To Initialize Mutex timerEventMutex\n");
		exit(-1);
	}

	// initialize conditions
	if (pthread_cond_init(&timerSetCond, NULL))
	{
		printf("Fail To Initialize Cond timerSetCond\n");
		exit(-1);
	}
}
