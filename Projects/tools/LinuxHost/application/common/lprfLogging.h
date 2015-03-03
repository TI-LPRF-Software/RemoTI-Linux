#ifndef _LPRF_LOGGING_
#define _LPRF_LOGGING_
#include <string.h>
#include <time.h>
#include "common_app.h"

#define  MAX_LOG_LEVEL_TO_STDIO __APP_LOG_LEVEL

// ------------------------------------------------------------------------
// ------------------------------------------------------------------------
// _PREPEND_LOG_TIMESTAMPS is intended for use within this header only.  
//It presumes the existance of _currTm and uSecs variables.
#define  _PREPEND_LOG_TIMESTAMPS(__CONST_FMT) \
               "[%04d-%02d-%02d %02d:%02d:%02d.%06ld] " __CONST_FMT ,\
               _currTm.tm_year+1900, _currTm.tm_mon+1, _currTm.tm_mday, \
               _currTm.tm_hour, _currTm.tm_min, _currTm.tm_sec, uSecs
// ------------------------------------------------------------------------
// ------------------------------------------------------------------------

#define LOG_LEVEL_FATAL   1
#define LOG_LEVEL_ERROR   2
#define LOG_LEVEL_WARN    3
#define LOG_LEVEL_INFO    4
#define LOG_LEVEL_DEBUG   5
#define LOG_LEVEL_TRACE1  6
#define LOG_LEVEL_TRACE2  7
#ifdef __DEBUG_TIME__
#define INT_LOG(__LVL, __FMT, __REST...) do {\
		struct timespec _currTime;                           \
		struct tm _currTm;                                   \
		long   uSecs;                                      \
		if (__LVL <= MAX_LOG_LEVEL_TO_STDIO) {             \
			clock_gettime(CLOCK_REALTIME, &_currTime);        \
			_currTm = *gmtime(&_currTime.tv_sec);               \
			uSecs = _currTime.tv_nsec/1000;                      \
			fprintf(stderr, _PREPEND_LOG_TIMESTAMPS(__FMT), ##__REST);\
		}\
} while(0)
#else
#define INT_LOG(__LVL, __FMT, __REST...) do {\
		if (__LVL <= MAX_LOG_LEVEL_TO_STDIO) {             \
			fprintf(stderr,  __FMT, ##__REST);\
		}\
} while(0)
#endif

#define LOG_TRACE2(__FMT, __REST...)	INT_LOG(LOG_LEVEL_TRACE2, "[APP]-[TRACE2] " __FMT, ##__REST)
#define LOG_TRACE(__FMT, __REST...) 	INT_LOG(LOG_LEVEL_TRACE1, "[APP]-[TRACE1] " __FMT, ##__REST)
#define LOG_DEBUG(__FMT, __REST...) 	INT_LOG(LOG_LEVEL_DEBUG,  "[APP]-[DEBUG] " __FMT, ##__REST)
#define LOG_INFO(__FMT, __REST...)  	INT_LOG(LOG_LEVEL_INFO,   "[APP]-[INFO] " __FMT, ##__REST)
#define LOG_WARN(__FMT, __REST...)  	INT_LOG(LOG_LEVEL_WARN,   "[APP]-[WARN] " __FMT, ##__REST)
#define LOG_ERROR(__FMT, __REST...) 	INT_LOG(LOG_LEVEL_ERROR,  "[APP]-[ERROR] " __FMT, ##__REST)
#define LOG_FATAL(__FMT, __REST...) 	INT_LOG(LOG_LEVEL_FATAL,  "[APP]-[FATAL] " __FMT, ##__REST)

#define LOG(__FMT, __REST...)    LOG_DEBUG(__FMT, ##__REST)

#endif // _LPRF_LOGGING_

