#ifndef _TI_LOGGING_
#define _TI_LOGGING_
#include <string.h>
#include <stdio.h>
#include <time.h>
#include <stdarg.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define LOG_DESTINATION_FP             stderr              // Using stderr is preferred as it's unbuffered so you get realtime logging.
#define DEFAULT_LOG_CFG_SUFFIX         "_log.conf"         // See notes for tiLogging_Init() below.
#define LOG_CONFIG_FILE_OVERRIDE_PATH1 "/opt/"	           // See notes for tiLogging_Init() below.
#define LOG_CONFIG_FILE_OVERRIDE_PATH2 "/mnt/flash/logs/"  // See notes for tiLogging_Init() below.

#if __BIG_DEBUG__
	#define  MAX_LOG_LEVEL_TO_STDIO  LOG_LEVEL_DEBUG
#else
	#define  MAX_LOG_LEVEL_TO_STDIO  LOG_LEVEL_INFO
#endif

// __APP_LOG_LEVEL is declared in file containing main().
// Normally main will set this to MAX_LOG_LEVEL_TO_STDIO. Optionally,
// (if main()provides for it) another means of setting the max log
// level can be use to set this to a positive value matching one
// of the defined log levels below.
extern int __APP_LOG_LEVEL;

// __BIG_DEBUG_ACTIVE is also declared in the source file containing main().
//    This variable is something that TI allows to be changed in the server
//    while running, via a command from the client.  Pretty handy!
extern int __BIG_DEBUG_ACTIVE;

extern char const *processLogPrefix; // Warning: NOT user modifiable.  Set within tiLogging.c

/**************************************************************************************************
 * @fn      tiLogging_Init
 * @brief   Initializes logging code.  If never called, logging will still
 *          work, but the process name won't be shown and only the default
 *				log level will be possible.
 * 
 * input parameters
 *
 * @param *configFileName: multi-purpose...
 *    If 'configFileName' as an unsigned integer is >0 and <= LOG_LEVEL_MAX 
 *       it will use that as the logging level (Limitation: Using this method you cannot 
 *       disable LOG_FATAL logging, but that's probably not anything you would want off.)
 *    Else if 'configFileName' is non-NULL, 
 *       it will try to read the logging level from 'configFileName'.
 *    Else if 'configFileName' is NULL, then first looks for a file in the same base name as 
 *       the process but with DEFAULT_LOG_CFG_SUFFIX, and will look for it in 
 *       LOG_CONFIG_FILE_OVERRIDE_PATH1 and if not found then in LOG_CONFIG_FILE_OVERRIDE_PATH2.
 *       If it doesn't exist there, it will look in the path of the process. If it's not there 
 *       it gives up and defaults to MAX_LOG_LEVEL_TO_STDIO (defined above).
 *    If it finds a config file and the value is out of range, it ignores it and defaults to
 *    MAX_LOG_LEVEL_TO_STDIO.
 *
 * @return  FALSE if previously initialized OR no config was found, otherwise TRUE.
 **************************************************************************************************
 */
extern bool tiLogging_Init(char const *configFileName);


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
extern void time_printf_always(char const *fmt, ...);

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
extern void time_printf_always_localized(struct timespec const *callersStartTime, struct timespec const *callersCurrentTime, struct timespec *callersPrevTime, char const *fmt, ...);

#ifdef __DEBUG_TIME__
   /**************************************************************************************************
    * @fn      time_printf_start
    * @brief   Forces global start time variable used by all other functions to the current time and 
    *          prints a message indicating that it's being done.
    *				Only available if __DEBUG__TIME__ is defined.
    *
    * @return  NONE
    **************************************************************************************************
    */
   extern void time_printf_start(void);

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
   extern void time_printf(char const *fmt, ...);
#else
   #define   time_printf_start()      (void)0
   #define   time_printf(__FMT, ...)  (void)0
#endif

// ------------------------------------------------------------------------
// ------------------------------------------------------------------------
// _PREPEND_TO_LOG is intended for use within this header, tiLogging.c and
// time_printf.c ONLY

#ifdef __DEBUG_TIME__
	#define  _PREPEND_TO_LOG(__CONST_FMT, __TM, __USECS)                  \
					"[%04d-%02d-%02d %02d:%02d:%02d.%06ld] %s" __CONST_FMT,   \
					__TM.tm_year+1900, __TM.tm_mon+1, __TM.tm_mday,           \
					__TM.tm_hour, __TM.tm_min, __TM.tm_sec, __USECS, processLogPrefix
#else
	#define  _PREPEND_TO_LOG(__CONST_FMT) \
					"%s" __CONST_FMT, processLogPrefix
#endif
// ------------------------------------------------------------------------
// ------------------------------------------------------------------------

#define LOG_LEVEL_ALWAYS  0 // Not necessarily an error, but ALWAYS log these (should be used sparingly and within some normally-disabled #ifdef)
#define LOG_LEVEL_FATAL   1
#define LOG_LEVEL_ERROR   2
#define LOG_LEVEL_WARN    3
#define LOG_LEVEL_INFO    4
#define LOG_LEVEL_DEBUG   5
#define LOG_LEVEL_TRACE1  6
#define LOG_LEVEL_TRACE2  7
#define LOG_LEVEL_MAX     LOG_LEVEL_TRACE2 // IMPORTANT! If this ever gets extended, you MUST extend the kLevelNames[] strings in tiLogging.c

#ifdef __DEBUG_TIME__
	#define INT_LOG(__LVL, __FMT, ...) do                                       \
	{                                                                           \
		struct timespec _currTime;                                               \
		struct tm _currTm;                                                       \
		long   _uSecs;                                                           \
		if ((__LVL <= __APP_LOG_LEVEL) ||                                        \
		    ((__LVL <= LOG_LEVEL_DEBUG) && __BIG_DEBUG_ACTIVE))                  \
		{                                                                        \
			clock_gettime(CLOCK_REALTIME, &_currTime);                            \
			_currTm = *gmtime(&_currTime.tv_sec);                                 \
			_uSecs = _currTime.tv_nsec/1000;                                      \
			fprintf(LOG_DESTINATION_FP, _PREPEND_TO_LOG(__FMT, _currTm, _uSecs), ##__VA_ARGS__);\
		}                                                                        \
} while(0)
#else
	#define INT_LOG(__LVL, __FMT, ...) do                                       \
	{                                                                           \
		if ((__LVL <= __APP_LOG_LEVEL) ||                                        \
		    ((__LVL <= LOG_LEVEL_DEBUG) && __BIG_DEBUG_ACTIVE))                  \
		{                                                                        \
			fprintf(LOG_DESTINATION_FP, _PREPEND_TO_LOG(__FMT), ##__VA_ARGS__);   \
		}                                                                        \
} while(0)
#endif

#define LOG_TRACE2(__FMT, ...)   INT_LOG(LOG_LEVEL_TRACE2, "[TRACE2] " __FMT, ##__VA_ARGS__)
#define LOG_TRACE(__FMT, ...)    INT_LOG(LOG_LEVEL_TRACE1, "[TRACE1] " __FMT, ##__VA_ARGS__)
#define LOG_DEBUG(__FMT, ...)    INT_LOG(LOG_LEVEL_DEBUG,  "[DEBUG]  " __FMT, ##__VA_ARGS__)
#define LOG_INFO(__FMT, ...)     INT_LOG(LOG_LEVEL_INFO,   "[INFO]   " __FMT, ##__VA_ARGS__)
#define LOG_WARN(__FMT, ...)     INT_LOG(LOG_LEVEL_WARN,   "[WARN]   " __FMT, ##__VA_ARGS__)
#define LOG_ERROR(__FMT, ...)    INT_LOG(LOG_LEVEL_ERROR,  "[ERROR]  " __FMT, ##__VA_ARGS__)
#define LOG_FATAL(__FMT, ...)    INT_LOG(LOG_LEVEL_FATAL,  "[FATAL]  " __FMT, ##__VA_ARGS__)
#define LOG_ALWAYS(__FMT, ...)   INT_LOG(LOG_LEVEL_ALWAYS, "[NOTICE] " __FMT, ##__VA_ARGS__)

#define LOG(__FMT, ...)    LOG_DEBUG(__FMT, ##__VA_ARGS__)

#ifdef __cplusplus
}
#endif

#endif // _LPRF_LOGGING_

