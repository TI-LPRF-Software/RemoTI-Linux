/*
 * time_printf.h
 *
 *  Created on: Feb 5, 2014
 *      Author: root
 *
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

#ifndef TIME_PRINT_H_
#define TIME_PRINT_H_

#include <time.h>
#include "npi_lnx_error.h"

#ifdef __cplusplus
extern "C"
{
#endif

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
extern void time_printf_always(char const *str);

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
extern void time_printf_always_localized(char const *str, struct timespec const *callersStartTime, struct timespec const *callersCurrentTime, struct timespec *callersPrevTime);

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
   extern void time_printf(char const *str);
#else
   #define   time_printf_start()        (void)0
   #define   time_printf(__conststr__)  (void)0
#endif

#ifdef __cplusplus
}
#endif


#endif /* TIME_PRINT_H_ */
