/**************************************************************************************************
  Filename:       MSO_Voice_lib.h

  Description:    RemoTI Audio Subsystem Library


  Copyright 2013 Texas Instruments Incorporated. All rights reserved.

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
  PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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


#ifndef RSA_LIB_H
#define RSA_LIB_H

#ifdef __cplusplus
extern "C" {
#endif


#if !defined PACK_1
#define PACK_1
#endif


#if defined(_MSC_VER) || defined(unix) || (defined(__ICC430__) && (__ICC430__==1))
#pragma pack(1)
#endif


/////////////////////////////////////////////////////////////////////////////
// Defines

#define MAX_INPUT_BUF_SIZE 				128

#define MSO_VOICE_PACKET_LOST 			0
#define MSO_VOICE_DECODE_TI_TYPE1		0x0001

#define MSO_VOICE_NO_PEC		   		0
#define MSO_VOICE_PEC_MODE1   			1

//MSO_Voice Software Version: v1.0
#define MSO_VOICE_SOFTWARE_VERSION		0x0100
/////////////////////////////////////////////////////////////////////////////
// Typedefs
#ifndef int8
typedef signed   char   int8;
#endif

#ifndef uint8
typedef unsigned char   uint8;
#endif

#ifndef int16
typedef signed   short  int16;
#endif

#ifndef uint16
typedef unsigned short  uint16;
#endif

/////////////////////////////////////////////////////////////////////////////
// Global variable


/////////////////////////////////////////////////////////////////////////////
// Function declarations
/**************************************************************************************************
 *
 * @fn      MSO_Voice_GetVersion
 *
 * @brief   RemoTI Audio Subsystem, retrieve software version
 *
 * input parameters
 *
 * none
 *
 * output parameters
 *
 * None.
 *
 * @return      .
 * Software Version.	MSB	 Major revision number
 *						LSB: Minor revision number
 */
uint16 MSO_Voice_GetVersion( void );

/**************************************************************************************************
 *
 * @fn      MSO_Voice_Init
 *
 * @brief   RemoTI Audio Subsystem, initialization function
 *
 * input parameters
 *
 * @param   pec_mode:    	Packet Error concealment algorithm to apply:
 * 							MSO_VOICE_NO_PEC(0): 		None (default)
 * 							MSO_VOICE_PEC_MODE1(1): 	Replace lost packets by last valid.
 *
 * output parameters
 *
 * None.
 *
 * @return      .
 * status.	0				SUCCESS
 * 			-1				ERROR: INVALID PARAMETER
 */
uint8 MSO_Voice_Init( uint8 pec_mode );


/**************************************************************************************************
 *
 * @fn      MSO_Voice_Decode
 *
 * @brief   RemoTI Audio Subsystem, decoding function. decode encoded audioframe to PCM samples.
 *
 * input parameters
 *
 * @param   option:    		decoding option. can be pure decoding, or packet lot concealment algorithm:
 * 							MSO_VOICE_PACKET_LOST(0)
 * 							MSO_VOICE_DECODE(1)
 * @param   input: 			address of the buffer to decode, this buffer must include the 3 bytes header..
 *
 * @param   inputLength:  	length of the buffer to decode, excluding the 3 bytes header.
 * 							cannot be greater than 128 (MAX_INPUT_BUF_SIZE);
 *
 * output parameters
 *
 * @param   output:     	buffer where the decoded PCM will be written. This buffer must be allocated by the caller.
 * 							it must have a length of 4 times the inputLength variable
 *
 * @param   outputLength:  	length of the decoded buffer.
 * 							max possible value 512 (4*MAX_INPUT_BUF_SIZE);
 *
 *
 * @return      .
 * status.	0				SUCCESS
 * 			-1				ERROR: INVALID PARAMETER
 *
 */
uint8 MSO_Voice_Decode( uint8 option, uint8* input, uint16 inputLength, int16* output,uint16 *outputLength );

#ifdef __cplusplus
}
#endif

#endif // RSA_LIB_H
