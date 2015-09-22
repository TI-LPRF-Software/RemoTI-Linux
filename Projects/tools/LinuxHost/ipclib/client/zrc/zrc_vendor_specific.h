/**************************************************************************************************
  Filename:       zrc_vendor_specific.h

  Description:    This file contains definitions for TI Vendor Specific ZRC 2.0
                  Action/Command Bank implementations.

  Copyright (C) 2013-2015 Texas Instruments Incorporated. All rights reserved.

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

#ifndef ZRC_VENDOR_SPECIFIC_H
#define ZRC_VENDOR_SPECIFIC_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "hal_types.h"
#include "zrc_profile.h"

/*********************************************************************
 * CONSTANTS
 */

// Number of paired HID devices allowed
#define ZRC_MAX_NUM_PROXIED_DEVICES                1

// Number of HID descriptors supported - only keyboard and mouse
#define ZRC_NUM_HID_DESC_SUPPORTED 2

// Action bank "bank" definitions - these define bits 0-4 of the action bank
#define ZRC_ACTION_BANK_TI_HID    (ZRC_ACTION_BANK_CLASS_VENDOR_SPECIFIC_EXPLICIT | 0)

// Action codes for TI HID Action Bank
#define ZRC_TI_ACTION_CODE_HID_REPORT_DATA 0

// ZRC HID report type field
#define ZRC_REPORT_TYPE_IN                 0x01
#define ZRC_REPORT_TYPE_OUT                0x02

// Mouse and keyboard report IDs
enum
{
  ZRC_STD_REPORT_NONE,
  ZRC_STD_REPORT_MOUSE,
  ZRC_STD_REPORT_KEYBOARD,
  ZRC_STD_REPORT_TOTAL_NUM = ZRC_STD_REPORT_KEYBOARD
};

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * TYPEDEFS
 */

// A packed structure of the following items that constitute a HID proxy table entry
typedef struct
{
  uint16 HIDParserVersion;
  uint16 HIDDeviceReleaseNumber;
  uint16 HIDVendorId;
  uint16 HIDProductId;
  uint8  HIDDeviceSubclass;
  uint8  HIDProtocolCode;
  uint8  HIDCountryCode;
  uint8  HIDNumEndpoints;
  uint8  HIDPollInterval;
  uint8  HIDNumStdDescComps;
  uint8  HIDStdDescCompsList[ZRC_NUM_HID_DESC_SUPPORTED];
  uint8  DeviceIdleRate;
  uint8  CurrentProtocol;
} zrc_proxy_entry_t;

// A packed structure that constitutes a Report data command frame.
typedef struct
{
  uint8 len;     // Report size.
  uint8 type;    // Report type.
  uint8 id;      // Report Id.
  uint8 data[];  // Report data.
} zrc_report_record_t;

// A packed structure that constitutes the data of the ZRC_STD_REPORT_MOUSE report.
typedef struct
{
  uint8 btns;  // Bits 0-2 : Left/Right/Center click; bits 3-7 : pad.
  uint8 x;     // -127 to +127 relative X movement.
  uint8 y;     // -127 to +127 relative Y movement.
} zrc_mouse_data_t;
#define ZRC_MOUSE_DATA_LEN                (sizeof(zrc_mouse_data_t))
#define ZRC_MOUSE_DATA_MAX                 127

// A packed structure that constitutes the data of the ZRC_STD_REPORT_KEYBOARD report IN.
typedef struct {
  uint8 mods;     // Bits 0-4 : modifier bits; bits 5-7 : LEDs.
  uint8 reserved;
  uint8 keys[6];  // Key arrays.
} zrc_keyboard_data_t;
#define ZRC_KEYBOARD_DATA_LEN             (sizeof(zrc_keyboard_data_t))
#define ZRC_KEYBOARD_MOD_MASK              0x1F
#define ZRC_KEYBOARD_LED_MASK              0xE0

// A packed structure that constitutes the data of the ZRC_STD_REPORT_KEYBOARD report OUT.
typedef struct {
  uint8 numLock:1;      // Num Lock
  uint8 capsLock:1;     // Caps Lock
  uint8 scrollLock:1;   // Scroll Lock
  uint8 compose:1;      // Compose
  uint8 kana:1;         // Kana
  uint8 reserved:3;     // Reserved
} zrc_keyboard_data_out_t;
#define ZRC_KEYBOARD_DATA_OUT_LEN          (sizeof(zrc_keyboard_data_out_t))
#define ZRC_KEYBOARD_LED_OUT_MASK           0x1F

/*********************************************************************
 * GLOBALS
 */

/*********************************************************************
 * FUNCTIONS
 */

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif
