/**************************************************************************************************
  Filename:       saddr.h
  Revised:        $Date: 2011-11-23 12:02:49 -0800 (Wed, 23 Nov 2011) $
  Revision:       $Revision: 108 $

  Description:    Zigbee and 802.15.4 device address utility functions.


  Copyright (C) {2012} Texas Instruments Incorporated - http://www.ti.com/

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

#ifndef SADDR_H
#define SADDR_H


#ifdef __cplusplus
extern "C" {
#endif

/****************************************************************************
 * MACROS
 */

/* Extended address length */
#define SADDR_EXT_LEN   8

/* Address modes */
#define SADDR_MODE_NONE       0       /* Address not present */
#define SADDR_MODE_SHORT      2       /* Short address */
#define SADDR_MODE_EXT        3       /* Extended address */

/****************************************************************************
 * TYPEDEFS
 */

/* Extended address */
typedef uint8 sAddrExt_t[SADDR_EXT_LEN];

/* Combined short/extended device address */
typedef struct
{
  union
  {
    uint16      shortAddr;    /* Short address */
    sAddrExt_t  extAddr;      /* Extended address */
  } addr;
  uint8         addrMode;     /* Address mode */
} sAddr_t;

#ifdef __cplusplus
}
#endif

#endif /* SADDR_H */
