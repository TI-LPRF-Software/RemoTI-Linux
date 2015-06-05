/**************************************************************************************************
  Filename:       sb_load.h
  Revised:        $Date: 2012-02-21 18:12:56 -0800 (ti, 21 feb 2012) $
  Revision:       $Revision: 205 $

  Description:    This file contrains a common declarations and definitions
                  for the Serial Bootloader


  Copyright 2008-2011 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED �AS IS� WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/

#ifndef SB_LOAD_H
#define SB_LOAD_H

#ifdef _MSC_VER
// MS C compiler does not understand __near_func keyword
# define __near_func
#endif

#if defined _WIN32 || defined unix
#define NEAR_FUNC
#define XDATA
#elif defined __ICC430__
#define NEAR_FUNC
#define XDATA
#else
#define NEAR_FUNC __near_func
#define XDATA __xdata
#endif

// Serial RX configuration
#define SB_BUF_SIZE                 128
#define SB_SOF                      0xFE
#define SB_HANDSHAKE                0xFE

// Serial RX State Machine Control Structure
typedef struct
{
  uint8 state;
  uint8 cmd1;
  uint8 cmd;
  uint8 len;
  uint8 fcs;
  uint8 pos;
  uint8 data[SB_BUF_SIZE];
} SB_RxInfo_t;

// Void function data type
typedef NEAR_FUNC void sb_func_t(void);


// Image Preamble
typedef struct
{
  uint32 enabled;
  sb_func_t *pfnStart;
} sb_preamble_t;

#define SB_ADDER_LEN                2
#define SB_RW_BUF_LEN               64
#define SB_WRITE_CMD_LEN            (SB_RW_BUF_LEN + SB_ADDER_LEN)

// Commands to Bootloader
#define SB_WRITE_CMD                0x01
#define SB_READ_CMD                 0x02
#define SB_ENABLE_CMD               0x03
#define SB_HANDSHAKE_CMD            0x04

// Responses from Bootloader - for historical consistency, SBL has OR'ed the MSBit of all commands
// when responding - this is probably not necessary for smooth functionality.
#define SB_RSP_MASK                	0x80

// Commands to Target Application
#define SB_TGT_BOOTLOAD             0x10

// Mailbox values
#define SB_MB_BOOT_APP              1
#define SB_MB_WAIT_HS               2

// Status codes
#define SB_SUCCESS                  0
#define SB_FAILURE                  1
#define SB_INVALID_FCS              2
#define SB_INVALID_FILE             3
#define SB_FILESYSTEM_ERROR         4
#define SB_ALREADY_STARTED          5
#define SB_NO_RESPOSNE              6
#define SB_VALIDATE_FAILED          7
#define SB_CANCELED                 8
#define SB_OUT_OF_SEQUENCE          9

// Serial RX States
#define SB_SOF_STATE                0
#define SB_LEN_STATE                1
#define SB_CMD1_STATE               2
#define SB_CMD2_STATE               3
#define SB_DATA_STATE               4
#define SB_FCS_STATE                5

// External Function declarations
extern void SB_HandleMsg(uint8* pMsg);

#endif // SB_LOAD_H
