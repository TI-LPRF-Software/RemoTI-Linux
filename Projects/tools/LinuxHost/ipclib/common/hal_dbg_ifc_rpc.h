/**************************************************************************************************
  Filename:       hal_flash_programmer_rpc.h
  Revised:        $Date: 2012-03-15 13:45:31 -0700 (Thu, 15 Mar 2012) $
  Revision:       $Revision: 237 $

  Description:     This file contains the interface to the CC2533 Flash Programmer.


  Copyright (C) {2016} Texas Instruments Incorporated - http://www.ti.com/


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
#ifndef HAL_FLASH_PROGRAMMER_RPC_H
#define HAL_FLASH_PROGRAMMER_RPC_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */

//#include "hal_board.h"
#include "hal_types.h"
#include "npi_lnx.h"

/*********************************************************************
 * MACROS
 */

#ifndef ATTR_PACKED
# ifdef __GNUC__
#  define ATTR_PACKED __attribute__ ((__packed__))
# else
#  define ATTR_PACKED
# endif
#endif

#define ERROR -1
/*******************************************/

// Synchronous Command IDs
#define DEBUG_CMD_ID_GET_CHIP_ID					0x01
#define DEBUG_CMD_ID_CONFIG_BUFFER					0x02
#define DEBUG_CMD_ID_WRITE_BUFFER					0x03
#define DEBUG_CMD_ID_READ_BUFFER					0x04
#define DEBUG_CMD_ID_WRITE_XDATA_BLOCK				0x05
#define DEBUG_CMD_ID_READ_XDATA_BLOCK				0x06
#define DEBUG_CMD_ID_GET_FLASH_SIZE					0x07

// Asynchronous Command IDs
#define DEBUG_CMD_ID_ENTER_DEBUG_MODE_REQ			0x01
#define DEBUG_CMD_ID_PROGRAM_BUFFER_REQ				0x02
#define DEBUG_CMD_ID_READ_FROM_CHIP_TO_BUFFER_REQ	0x03

// Asynchronous Command IDs
#define DEBUG_CMD_ID_PROGRAM_BUFFER_CNF				0x01
#define DEBUG_CMD_ID_READ_FROM_CHIP_TO_BUFFER_CNF	0x02

// Status
#define DEBUG_FLASH_PROGRAM_SUCCESS					0x00
#define DEBUG_FLASH_FAILED_TO_WRITE_BUFFER			0x01
#define DEBUG_FLASH_FAILED_TO_READ_BUFFER			0x02
#define DEBUG_FLASH_FAILED_TO_START_PROGRAM_BUFFER	0x03
#define DEBUG_FLASH_FAILED_TO_CONFIGURE_BUFFER		0x04
#define DEBUG_FLASH_FAILED_TO_READ_FLASH_SIZE		0x05
#define DEBUG_FLASH_NO_BUFFER_CONFIGURED			0x06
#define DEBUG_FLASH_FAILED_TO_WAIT_FOR_RESPONSE		0x07

// Flash write block sizes
#define DEBUG_FLASH_PROGRAM_NPI_BLOCK_SIZE			64
#define DEBUG_FLASH_PROGRAM_BUFFER_BLOCK_SIZE		1024
/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */
#if !defined PACK_1
#define PACK_1
#endif

// To be compatible with MS and unix native target
// declare pragma for structure packing
#if defined(_MSC_VER) || defined(unix) || (defined(__ICC430__) && (__ICC430__==1))
#pragma pack(1)
#endif


/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * FUNCTIONS - API
 */

extern int Hal_write_buffer_memory_block(uint32 address, uint32 *values, uint16 num_bytes);
extern int Hal_read_buffer_memory_block(uint32 address, uint32 *values, uint16 num_bytes);
extern int Hal_write_xdata_memory_block(uint16 address, uint8 *values, uint16 num_bytes);
extern int Hal_read_xdata_memory_block(uint16 address, uint8 *values, uint16 num_bytes);
extern int Hal_read_chip_id(uint8 *chipId);
extern int Hal_read_flash_size(uint8 *chipId, uint32 *returnVal);
extern uint8 Hal_configure_buffer(uint32 bufferSize, uint32 startAddress);

extern int Hal_program_bufferReq(void);
extern int Hal_read_from_chip_to_bufferReq();
extern int Hal_EnterDebugModeReq(void);

extern void Hal_program_bufferCnf(uint8 status);
extern void Hal_read_from_chip_to_bufferCnf(uint8 status);

#ifdef __cplusplus
}
#endif

#endif
/******************************************************************************
******************************************************************************/
