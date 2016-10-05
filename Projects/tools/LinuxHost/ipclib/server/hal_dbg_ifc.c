/**************************************************************************************************
  Filename:       hal_dbg_ifc.c
  Revised:        $Date: 2012-03-15 13:45:31 -0700 (Thu, 15 Mar 2012) $
  Revision:       $Revision: 237 $

  Description:    This file contains the interface to the CC2533 Debug Interface.


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

/**************************************************************************************************
 *                                           INCLUDES
 **************************************************************************************************/
#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <sys/poll.h>

//#include  "hal_board.h"
#include "hal_types.h"
#include "hal_rpc.h"
#include "hal_dbg_ifc.h"
#include "hal_dbg_ifc_rpc.h"
#include "time_printf.h"

#include "npi_lnx_error.h"

#ifdef __STRESS_TEST__
#include <sys/time.h>
#elif defined __DEBUG_TIME__
#include <sys/time.h>
#endif //__DEBUG_TIME__

/**************************************************************************************************
 *                                            CONSTANTS
 **************************************************************************************************/

/**************************************************************************************************
 *                                              MACROS
 **************************************************************************************************/

#ifdef __BIG_DEBUG__
#define debug_printf(fmt, ...) printf( fmt, ##__VA_ARGS__)
#else
#define debug_printf(fmt, ...) st (if (__BIG_DEBUG_ACTIVE == TRUE) printf( fmt, ##__VA_ARGS__);)
#endif

#define HAL_GPIO_DC_CFG(x) st (x = HalGpioDCInit(0);)
#define HAL_GPIO_DC_CLR(x) st (x = halGpioDCSet(0);)
#define HAL_GPIO_DC_SET(x) st (x = halGpioDCSet(1);)

#define HAL_GPIO_DD_CFG(x) st (x = HalGpioDDInit(0);)
#define HAL_GPIO_DD_CLR(x) st (x = halGpioDDSet(0);)
#define HAL_GPIO_DD_SET(x) st (x = halGpioDDSet(1);)
#define HAL_GPIO_DD_IS_CLR(x, v) st (x = HalGpioDDCheck(0, v);)
#define HAL_GPIO_DD_IS_SET(x, v) st (x = HalGpioDDCheck(1, v);)

#define HAL_GPIO_DD_RX(x) HAL_GPIO_DD_SET(x)
#define HAL_GPIO_DD_TX(x) HAL_GPIO_DD_CLR(x)

#define	MIN(a,b)		((a < b) ? a : b)
/******************************************************************************
* DEFINES
*/
// Programmer data line bitmasks (programmer I/O port 0)
#define DD                          0x01 // P0.0
#define DC                          0x02 // P0.1
#define RESET_N                     0x80 // P0.7

// Start addresses on DUP (Increased buffer size improves performance)
#define ADDR_BUF0                   0x0000 // Buffer (2048 bytes)
#define ADDR_DMA_DESC_0             0x0800 // DMA descriptors (8 bytes)
#define ADDR_DMA_DESC_1             (ADDR_DMA_DESC_0 + 8)

// DMA channels used on DUP
#define CH_DBG_TO_BUF0              0x01   // Channel 0
#define CH_BUF0_TO_FLASH            0x02   // Channel 1

// Debug commands
#define CMD_CHIP_ERASE              0x10
#define CMD_WR_CONFIG               0x19
#define CMD_RD_CONFIG               0x24
#define CMD_READ_STATUS             0x30
#define CMD_RESUME                  0x4C
#define CMD_DEBUG_INSTR_1B          (0x54|1)
#define CMD_DEBUG_INSTR_2B          (0x54|2)
#define CMD_DEBUG_INSTR_3B          (0x54|3)
#define CMD_BURST_WRITE             0x80
#define CMD_GET_CHIP_ID             0x68

// Debug status bitmasks
#define STATUS_CHIP_ERASE_BUSY_BM   0x80 // New debug interface
#define STATUS_PCON_IDLE_BM         0x40
#define STATUS_CPU_HALTED_BM        0x20
#define STATUS_PM_ACTIVE_BM         0x10
#define STATUS_HALT_STATUS_BM       0x08
#define STATUS_DEBUG_LOCKED_BM      0x04
#define STATUS_OSC_STABLE_BM        0x02
#define STATUS_STACK_OVERFLOW_BM    0x01

// DUP registers (XDATA space address)
#define DUP_DBGDATA                 0x6260  // Debug interface data buffer
#define DUP_FCTL                    0x6270  // Flash controller
#define DUP_FADDRL                  0x6271  // Flash controller addr
#define DUP_FADDRH                  0x6272  // Flash controller addr
#define DUP_FWDATA                  0x6273  // Clash controller data buffer
#define DUP_CLKCONSTA               0x709E  // Sys clock status
#define DUP_CLKCONCMD               0x70C6  // Sys clock configuration
#define DUP_MEMCTR                  0x70C7  // Flash bank xdata mapping
#define DUP_DMA1CFGL                0x70D2  // Low byte, DMA config ch. 1
#define DUP_DMA1CFGH                0x70D3  // Hi byte , DMA config ch. 1
#define DUP_DMA0CFGL                0x70D4  // Low byte, DMA config ch. 0
#define DUP_DMA0CFGH                0x70D5  // Low byte, DMA config ch. 0
#define DUP_DMAARM                  0x70D6  // DMA arming register


#define HAL_GPIO_INPUT		0
#define HAL_GPIO_OUTPUT		1
// Utility macros
//! Set programmer DD line as input
#define SET_DD_INPUT(x)      st (x = halGpioDDSetDirection(HAL_GPIO_INPUT);)
//! Set programmer DD line as output
#define SET_DD_OUTPUT(x)     st (x = halGpioDDSetDirection(HAL_GPIO_OUTPUT);)
//! Low nibble of 16bit variable
#define LOBYTE(w)           ((uint8)(w))
//! High nibble of 16bit variable
#define HIBYTE(w)           ((uint8)(((uint16)(w) >> 8) & 0xFF))
//! Convert XREG register declaration to an XDATA integer address
#define XREG_TO_INT(a)      ((uint16)(&(a)))


/**************************************************************************************************
 *                                            TYPEDEFS
 **************************************************************************************************/

/**************************************************************************************************
 *                                         GLOBAL VARIABLES
 **************************************************************************************************/
static int gpioDDFd_val;
static int gpioDDFd_gpio_dir;
static int gpioDDFd_level_val;
static int gpioDDFd_level_val_exists = FALSE;
static int gpioDCFd;

static halGpioCfg_t ddGpioCfg;
static halGpioCfg_t dcGpioCfg;

#if defined __DEBUG_TIME__
struct timespec currentTime={0,0}, previousTime={0,0};
#endif //__DEBUG_TIME__

/******************************************************************************
* VARIABLES
*/
//! DUP DMA descriptor
static uint8 dma_desc_0[8] =
{
    // Debug Interface -> Buffer
    HIBYTE(DUP_DBGDATA),            // src[15:8]
    LOBYTE(DUP_DBGDATA),            // src[7:0]
    HIBYTE(ADDR_BUF0),              // dest[15:8]
    LOBYTE(ADDR_BUF0),              // dest[7:0]
    0,                              // len[12:8] - filled in later
    0,                              // len[7:0]
    31,                             // trigger: DBG_BW
    0x11                            // increment destination
};
//! DUP DMA descriptor
static uint8 dma_desc_1[8] =
{
    // Buffer -> Flash controller
    HIBYTE(ADDR_BUF0),              // src[15:8]
    LOBYTE(ADDR_BUF0),              // src[7:0]
    HIBYTE(DUP_FWDATA),             // dest[15:8]
    LOBYTE(DUP_FWDATA),             // dest[7:0]
    0,                              // len[12:8] - filled in later
    0,                              // len[7:0]
    18,                             // trigger: FLASH
    0x42,                           // increment source
};

//! Configuration struct
static struct
{
	uint32 size;
	uint32 startAddress;		// 18 bit address. 16 MSB is used (word addressing)
	uint32 numOfBytesWritten;
} bufferCfg;

static uint8 *flashBuffer = NULL;

/**************************************************************************************************
 *                                          FUNCTIONS - API
 **************************************************************************************************/

int halGpioDCSet(uint8 state);
int HalGpioDCCheck(uint8 state, uint8 *match);
int halGpioDDSet(uint8 state);
int HalGpioDDCheck(uint8 state, uint8 *match);
int HalGpioWaitDDClr(void);
int HalGpioWaitDDSet(void);

// APIs accessible through RPC
int Hal_write_buffer_memory_block(uint32 address, uint32 *values, uint16 num_words);
int Hal_read_buffer_memory_block(uint32 address, uint32 *values, uint16 num_words);
int Hal_write_xdata_memory_block(uint16 address, uint8 *values, uint16 num_bytes);
int Hal_read_xdata_memory_block(uint16 address, uint8 *values, uint16 num_bytes);
int Hal_read_chip_id(uint8 *chipId);
int Hal_read_flash_size(uint8 *chipId, uint32 *returnVal);
uint8 Hal_configure_buffer(uint32 bufferSize, uint32 startAddress);

int Hal_program_bufferReq(void);
int Hal_read_from_chip_to_bufferReq(void);

int Hal_chip_erase(void);
int Hal_write_flash_memory_block(uint8 *src, uint32 start_addr, uint16 num_bytes);
int Hal_read_flash_memory_block(uint8 bank,uint16 flash_addr, uint16 num_bytes, uint8 *values);
int Hal_read_xdata_memory(uint16 address, uint8 *byte);
int Hal_write_xdata_memory(uint16 address, uint8);

/******************************************************************************
* FUNCTIONS
*/


/**************************************************************************************************
 * @fn          Hal_DebugInterface_SynchMsgCback
 *
 * @brief       This function is a NPI callback to the Flash Programmer that indicates
 * 				an asynchronous message has been received. The Flash Programmer software
 * 				is expected to complete this call.
 *
 *              Note: The Flash Programmer must copy this message if it requires it
 *                    beyond the context of this call.
 *
 * input parameters
 *
 * @param       *pMsg - A pointer to an asychronously received message.
 *
 * output parameters
 *
 * None.
 *
 * @return      STATUS
 **************************************************************************************************
 */
int Hal_DebugInterface_SynchMsgCback( npiMsgData_t *pMsg )
{
	int ret = NPI_LNX_SUCCESS;
	if ((pMsg->subSys & RPC_SUBSYSTEM_MASK) == RPC_SYS_DEBUG)
	{
		switch( pMsg->cmdId )
		{
		case DEBUG_CMD_ID_GET_CHIP_ID:
			pMsg->len = 1;
			ret = Hal_read_chip_id(pMsg->pData);
			break;
		case DEBUG_CMD_ID_GET_FLASH_SIZE:
			pMsg->len = sizeof(uint32) + 1;
			ret = Hal_read_flash_size(pMsg->pData, (uint32 *)&pMsg->pData[1]);
			break;
		case DEBUG_CMD_ID_CONFIG_BUFFER:
			pMsg->len = 1;
			// Function returns status
			pMsg->pData[0] = Hal_configure_buffer(((uint32 *)pMsg->pData)[0],
					((uint32 *)pMsg->pData)[1]);
			break;
		case DEBUG_CMD_ID_WRITE_BUFFER:
			pMsg->len = 1;
			// Function returns status
			pMsg->pData[0] = Hal_write_buffer_memory_block(((uint32 *)pMsg->pData)[0],
					(uint32 *)&pMsg->pData[6],
					((uint16 *)pMsg->pData)[2]);
			break;
		case DEBUG_CMD_ID_READ_BUFFER:
			// Function returns status
			pMsg->pData[0] = Hal_read_buffer_memory_block(((uint32 *)pMsg->pData)[0],
					(uint32 *)&pMsg->pData[6],
					((uint16 *)pMsg->pData)[2]);
			pMsg->len = 6 + ((uint16 *)pMsg->pData)[2];
			break;
		case DEBUG_CMD_ID_WRITE_XDATA_BLOCK:
			// Function returns status
			pMsg->pData[0] = Hal_write_xdata_memory_block(((uint16 *)pMsg->pData)[0],
					&pMsg->pData[4],
					((uint16 *)pMsg->pData)[1]);
			pMsg->len = 1;
			break;
		case DEBUG_CMD_ID_READ_XDATA_BLOCK:
			// Function returns status
			pMsg->pData[0] = Hal_read_xdata_memory_block(((uint16 *)pMsg->pData)[0],
					&pMsg->pData[4],
					((uint16 *)pMsg->pData)[1]);
			pMsg->len = 4 + ((uint16 *)pMsg->pData)[1];
			break;
		default:
			// nothing can be done here!
			break;
		}
	}
	if ( (ret != NPI_LNX_SUCCESS) && (npi_ipc_errno == NPI_LNX_ERROR_HAL_DBG_IFC_WAIT_DUP_READY) )
	{
		pMsg->pData[0] = 0xFF;
	}
	return ret;
}

/**************************************************************************************************
 * @fn          Hal_DebugInterface_AsynchMsgCback
 *
 * @brief       This function is a NPI callback to the Flash Programmer that indicates
 * 				an asynchronous message has been received. The Flash Programmer software
 * 				is expected to complete this call.
 *
 *              Note: The Flash Programmer must copy this message if it requires it
 *                    beyond the context of this call.
 *
 * input parameters
 *
 * @param       *pMsg - A pointer to an asynchronously received message.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
int Hal_DebugInterface_AsynchMsgCback( npiMsgData_t *pMsg )
{
	int ret = NPI_LNX_SUCCESS;
	if ((pMsg->subSys & RPC_SUBSYSTEM_MASK) == RPC_SYS_DEBUG)
	{
		switch( pMsg->cmdId )
		{
		case DEBUG_CMD_ID_ENTER_DEBUG_MODE_REQ:
			ret = Hal_EnterDebugModeReq();
			break;
		case DEBUG_CMD_ID_PROGRAM_BUFFER_REQ:
			ret = Hal_program_bufferReq();
			break;
		case DEBUG_CMD_ID_READ_FROM_CHIP_TO_BUFFER_REQ:
			ret = Hal_read_from_chip_to_bufferReq();
			break;
		default:
			npi_ipc_errno = NPI_LNX_ERROR_HAL_DBG_IFC_ASYNCH_INVALID_CMDID;
			ret = NPI_LNX_FAILURE;
			break;
		}
	}
	return ret;
}

/**************************************************************************//**
* @brief    Resets the DUP into debug mode. Function assumes that
*           the programmer I/O has already been configured using e.g.
*           Hal_programmer_init().
*
* @return   STATUS
******************************************************************************/
int Hal_debug_init(void)
{
	return Hal_EnterDebugModeReq();
}

/****************************************************************************
* @brief    Toggles the debug clock line twice while holding reset low.
* 			This makes the chip enter debug mode.
*
* @param    None.
*
* @return   STATUS
******************************************************************************/
int Hal_EnterDebugModeReq(void)
{
    uint8 i;
    int ret = NPI_LNX_SUCCESS;

    printf("[DEBUG INTERFACE] Entering Debug Mode");
    fflush(stdout);

    // Send two flanks on DC while keeping RESET_N low
//    P0 = 0x00;                  // All low (incl. RESET_N)
	if ( ret == NPI_LNX_SUCCESS )
	{
		HAL_GPIO_DC_CLR(ret);
	}
    printf(" .");
    fflush(stdout);
	if ( ret == NPI_LNX_SUCCESS )
	{
		ret = HalGpioResetSet(!RESET_N);
	}
	for (i = 0; i < 0xFF; i++);   // Wait
	//    P0 = DC;                    // DC high
    printf(" .");
    fflush(stdout);
	if ( ret == NPI_LNX_SUCCESS )
	{
		HAL_GPIO_DC_SET(ret);
	}
	//    P0 = 0x00;                  // DC low
	if ( ret == NPI_LNX_SUCCESS )
	{
		HAL_GPIO_DC_CLR(ret);
	}
    printf(" .");
    fflush(stdout);
	//    P0 = DC;                    // DC high
	if ( ret == NPI_LNX_SUCCESS )
	{
		HAL_GPIO_DC_SET(ret);
	}
	//    P0 = 0x00;                  // DC low
	if ( ret == NPI_LNX_SUCCESS )
	{
		HAL_GPIO_DC_CLR(ret);
	}
    printf(" .");
    fflush(stdout);
	for (i = 0; i < 0xFF; i++);   // Wait
	//    P0 = RESET_N;               // Release RESET_N
    printf(" .");
    fflush(stdout);
	if ( ret == NPI_LNX_SUCCESS )
	{
		ret = HalGpioResetSet(RESET_N);
	}
	printf(" Entered Debug Mode (ret = %d)\n", ret);

	return ret;
}

/**************************************************************************//**
* @brief    Writes a byte on the debug interface. Requires DD to be
*           output when function is called.
*
* @param    data    Byte to write
*
* @return   STATUS
******************************************************************************/
int Hal_write_debug_byte(uint8 data)
{
    uint8 i;
    int ret = NPI_LNX_SUCCESS;
    for (i = 0; i < 8; i++)
    {
    	// Set clock high and put data on DD line
    	//        P0 = (data & 0x80) ? (RESET_N|DC|DD) : (RESET_N|DC);
    	HAL_GPIO_DC_SET(ret);
    	if (data & 0x80)
    	{
    		if (ret != NPI_LNX_FAILURE)
    		{
    			HAL_GPIO_DD_SET(ret);
    		}
    	}
    	else
    	{
    		if (ret != NPI_LNX_FAILURE)
    		{
    			HAL_GPIO_DD_CLR(ret);
    		}
    	}
    	data <<= 1;
    	if (ret != NPI_LNX_FAILURE)
    	{
    		HAL_GPIO_DC_CLR(ret);
    	}
    	//        P0 &= ~DC; // set clock low (DUP capture flank)
    }

    return ret;
}


/**************************************************************************//**
* @brief    Reads a byte from the debug interface. Requires DD to be
*           input when function is called.
*
* @param	data	- pointer to the byte read
*
* @return   STATUS.
******************************************************************************/
int Hal_read_debug_byte(uint8 *byte)
{
    uint8 i, res;
    int ret = NPI_LNX_SUCCESS;
    for (i = 0; i < 8; i++)
    {
//        P0 = (RESET_N|DC);  // DC high
        HAL_GPIO_DC_SET(ret);
        *byte <<= 1;
//        data |= (P0 & DD);  // Read DD line
        if (ret != NPI_LNX_FAILURE)
        {
        	ret = HalGpioDDCheck(DD, &res);  // Check if DD line is high
            if (ret != NPI_LNX_FAILURE)
            {
            	*byte |= res;
            }
        }
        else
        {
        	break;
        }
//        P0 = (RESET_N);     // DC low
        if (ret != NPI_LNX_FAILURE)
        {
        	HAL_GPIO_DC_CLR(ret);
        }
        else
        {
        	break;
        }
    }
    return ret;
}


/**************************************************************************//**
* @brief    Function waits for DUP to indicate that it is ready. The DUP will
*           pulls DD line low when it is ready. Requires DD to be input when
*           function is called.
*
* @param	status - 0 if function timed out waiting for DD line to go low
* 					 1 when DUP has indicated it is ready.
* @return   STATUS
******************************************************************************/
int Hal_wait_dup_ready(void)
{
	int ret = NPI_LNX_SUCCESS;
    // DUP pulls DD low when ready
    uint8 count = 0, byte, res;
//    while (P0 & DD && count < 16)
	debug_printf("[DEBUG INTERFACE] Wait for DUP begin\n");
    ret = HalGpioDDCheck(DD, &res);
	while ( (count < 16) && (ret == NPI_LNX_SUCCESS) && (res == 1))
    {
        // Clock out 8 bits before checking if DD is low again
		if (ret == NPI_LNX_SUCCESS)
		{
			ret = Hal_read_debug_byte(&byte);
		}
		else
		{
			return ret;
		}
        count++;
        // wait
        usleep(5);
        ret = HalGpioDDCheck(DD, &res);
    }

	if (count == 16)
	{
		debug_printf("Failed response from DUP\n");
		npi_ipc_errno = NPI_LNX_ERROR_HAL_DBG_IFC_WAIT_DUP_READY;
		return NPI_LNX_FAILURE;
	}
	debug_printf("[DEBUG INTERFACE] Wait for DUP done\n");

	return ret;
}


/**************************************************************************//**
* @brief    Issues a command on the debug interface. Only commands that return
*           one output byte are supported.
*
* @param    cmd             Command byte
* @param    cmd_bytes       Pointer to the array of data bytes following the
*                           command byte [0-3]
* @param    num_cmd_bytes   The number of data bytes (input to DUP) [0-3]
*
* Output
*
* @param	cmd_bytes       Pointer to resulting byte
*
* @return   STATUS
******************************************************************************/
int Hal_debug_command(uint8 cmd, uint8 *cmd_bytes,
                            uint16 num_cmd_bytes)
{
    uint16 i;
    int ret = NPI_LNX_SUCCESS;

    // Make sure DD is output
    SET_DD_OUTPUT(ret);

    // Send command

	if (ret == NPI_LNX_SUCCESS)
	{
		ret = Hal_write_debug_byte(cmd);
	}

    // Send bytes
    for (i = 0; i < num_cmd_bytes; i++)
    {
    	if (ret == NPI_LNX_SUCCESS)
    	{
    		ret = Hal_write_debug_byte(cmd_bytes[i]);
    	}
    	else
    	{
    		break;
    	}
    }

    // Only read back if an output is expected
    if (cmd_bytes)
    {
    	// Set DD as input
    	if (ret == NPI_LNX_SUCCESS)
    	{
    		SET_DD_INPUT(ret);
    	}

    	// Wait for data to be ready
    	if (ret == NPI_LNX_SUCCESS)
    	{
    		ret = Hal_wait_dup_ready();
    	}

    	// Read returned byte
    	if (ret == NPI_LNX_SUCCESS)
    	{
    		ret = Hal_read_debug_byte(cmd_bytes);
    	}

    	// Set DD as output
    	if (ret == NPI_LNX_SUCCESS)
    	{
    		SET_DD_OUTPUT(ret);
    	}
    }

    return ret;
}

/**************************************************************************//**
* @brief    Reads the chip ID over the debug interface using the
*           GET_CHIP_ID command.
*
* @param	chipId		- pointer to chipId
*
* @return   Returns the chip id returned by the DUP
******************************************************************************/
int Hal_read_chip_id(uint8 *chipId)
{
    uint8 revision = 0;
    int ret = NPI_LNX_SUCCESS;

    // Make sure DD is output
    SET_DD_OUTPUT(ret);
    // Make sure DD is high
    if ( ret == NPI_LNX_SUCCESS )
    {
    	HAL_GPIO_DD_SET(ret);
    }

    // Send command
    if ( ret == NPI_LNX_SUCCESS )
    {
    	ret = Hal_write_debug_byte(CMD_GET_CHIP_ID);
    }

    // Set DD as input
    if ( ret == NPI_LNX_SUCCESS )
    {
    	SET_DD_INPUT(ret);
    }

    // Wait for data to be ready
    if ( ret == NPI_LNX_SUCCESS )
    {
    	ret = Hal_wait_dup_ready();
    }

    // Read ID and revision
    if ( ret == NPI_LNX_SUCCESS )
    {
    	ret = Hal_read_debug_byte(chipId); // ID
    }
    if ( ret == NPI_LNX_SUCCESS )
    {
    	ret = Hal_read_debug_byte(&revision);      // Revision (discard)
    }

    // Set DD as output
    if ( ret == NPI_LNX_SUCCESS )
    {
    	SET_DD_OUTPUT(ret);
    }

    debug_printf("[DEBUG INTERFACE] Chip ID read 0x%.2X (ret = %d)\n", *chipId, ret);
    return ret;
}

/**************************************************************************//**
* @brief    Reads the flash size over the debug interface.
*
* Input
* @param	chipId		- requested chipId
*
* Output
* @param	chipId		- actual chipId of device
* @param	returnVal	- found flash size
*
* @return   STATUS
******************************************************************************/
int Hal_read_flash_size(uint8 *chipId, uint32 *returnVal)
{
	int ret = NPI_LNX_SUCCESS;
	uint8 flashSizeReg = 0;
	ret = Hal_read_xdata_memory(0x6276, &flashSizeReg);

	// Get chip ID, if the passed value is NULL
	if (*chipId == 0x00)
	{
		Hal_read_chip_id(chipId);
		printf("Chip ID was not provided. Read from chip: 0x%.2X\n", *chipId);
	}

	debug_printf("Flash REG: 0x%.2X\n", flashSizeReg);

	if ((flashSizeReg & 0x70) == 0x10)
		returnVal[0] = (32*1024);
	else if ((flashSizeReg & 0x70) == 0x20)
		returnVal[0] = (64*1024);
	else if ((flashSizeReg & 0x70) == 0x30)
		if (*chipId == 0x95)
			returnVal[0] = (96*1024);
		else
			returnVal[0] = (128*1024);
	else if ((flashSizeReg & 0x70) == 0x40)
		returnVal[0] = (256*1024);
	else
		returnVal[0] = 0;

	if (returnVal[0] == 0)
	{

		ret = (uint8)NPI_LNX_FAILURE;
	}
	return ret;
}

/**************************************************************************//**
* @brief    Configures the flash programmer buffer.
* 			This includes buffer size and start address. The function will
* 			allocate the appropriate buffer.
*
* @param    bufferSize		buffer size in bytes
* @param	startAddress	start address
*
* @return   Status.
******************************************************************************/
uint8 Hal_configure_buffer(uint32 bufferSize, uint32 startAddress)
{
	bufferCfg.size = bufferSize;
	bufferCfg.startAddress = startAddress;
	bufferCfg.numOfBytesWritten = 0;

	if (flashBuffer != NULL)
		free(flashBuffer);

	// Only accept sizes as multiples of 1024 (flash page size on CC2533)
	if (bufferSize % 1024)
		return DEBUG_FLASH_FAILED_TO_CONFIGURE_BUFFER;
	// Only accept addresses on flash page boundaries,
	// i.e. multiples of 1024 (flash page size on CC2533)
	if (startAddress % 1024)
		return DEBUG_FLASH_FAILED_TO_CONFIGURE_BUFFER;

	flashBuffer = (uint8 *)malloc(bufferSize);

	return DEBUG_FLASH_PROGRAM_SUCCESS;
}

/****************************************************************************
* @brief    Programs the flash with the content of the buffer.
*
* @param    None.
*
* @return   STATUS.
******************************************************************************/
int Hal_program_bufferReq()
{
	int ret = NPI_LNX_SUCCESS;

	npiMsgData_t pMsg;
	pMsg.len = 1;
	pMsg.subSys = RPC_SYS_DEBUG | RPC_CMD_AREQ;
	pMsg.cmdId = DEBUG_CMD_ID_PROGRAM_BUFFER_CNF;
	pMsg.pData[0] = DEBUG_FLASH_PROGRAM_SUCCESS;

	uint8 chipId = 0;
	ret = Hal_read_chip_id(&chipId);

	if (flashBuffer == NULL)
	{
		pMsg.pData[0] = DEBUG_FLASH_FAILED_TO_START_PROGRAM_BUFFER;
		ret = NPI_LNX_FAILURE;
	}
	else if (bufferCfg.numOfBytesWritten != bufferCfg.size)
	{
		// The buffer has not been filled. Return error.
		pMsg.pData[0] = DEBUG_FLASH_FAILED_TO_START_PROGRAM_BUFFER;
		ret = NPI_LNX_FAILURE;
	}
	if (chipId != 0x95)
	{
		// We only support CC2533. Return error.
		pMsg.pData[0] = DEBUG_FLASH_FAILED_TO_START_PROGRAM_BUFFER;
		ret = NPI_LNX_FAILURE;
	}

	if ( ret == NPI_LNX_SUCCESS )
	{
	    debug_printf("[DEBUG INTERFACE] Begin writing the image to the chip, @0x%.6X\n",
	    		bufferCfg.startAddress);
		// Program the buffer. This may take a while...
		uint32 address = bufferCfg.startAddress;
		uint32 flashSize = 0;
		ret = Hal_read_flash_size(&chipId, &flashSize);

		// Start with Mass Erase if start address is 0, and buffer size is equal to flash size
		if ( (address == 0) && (bufferCfg.size == flashSize) && (ret == NPI_LNX_SUCCESS) )
		{
//			debug_
			printf("[DEBUG INTERFACE] Erase chip\n");
			ret = Hal_chip_erase();
			if (ret == NPI_LNX_SUCCESS )
			{
				debug_printf("[DEBUG INTERFACE] Chip erase successful\n");
			}
			else
			{
//				debug_
				printf("[DEBUG INTERFACE] Chip erase reported error, chip may still be erased\n");
				pMsg.pData[0] = DEBUG_FLASH_FAILED_TO_WAIT_FOR_RESPONSE;
			}

		    // Enable DMA (Disable DMA_PAUSE bit in debug configuration)
		    uint8 debug_config = 0x22;
			if ( ret == NPI_LNX_SUCCESS )
			{
				Hal_debug_command(CMD_WR_CONFIG, &debug_config, 1);

				for (; address < bufferCfg.size; address += DEBUG_FLASH_PROGRAM_BUFFER_BLOCK_SIZE)
				{
//					debug_
					printf("[DEBUG INTERFACE] Write page %u: \t", address/DEBUG_FLASH_PROGRAM_BUFFER_BLOCK_SIZE);
					uint32 tmpAdd = address;
					for (; tmpAdd < (address + 32); tmpAdd++)
					{
						debug_printf("%.2X", flashBuffer[tmpAdd]);
					}
					debug_printf("\n");

					if ( ret == NPI_LNX_SUCCESS )
					{
						// Write one page at a time
						ret = Hal_write_flash_memory_block((uint8 *)&flashBuffer[address], address, DEBUG_FLASH_PROGRAM_BUFFER_BLOCK_SIZE);
					}
					else
					{
						break;
					}

				}
			}
		}
		else
		{
			for (; address < (bufferCfg.size + bufferCfg.startAddress); address += DEBUG_FLASH_PROGRAM_BUFFER_BLOCK_SIZE)
			{
				// Erase page before writing it.
				debug_printf("[DEBUG INTERFACE] Erase page %u\n", address/DEBUG_FLASH_PROGRAM_BUFFER_BLOCK_SIZE);

				if ( ret == NPI_LNX_SUCCESS )
				{
					// 1. Set Flash controller address of page we want to erase
					ret = Hal_write_xdata_memory(DUP_FADDRH, HIBYTE( (address>>2) ));
				}
				else
				{
					break;
				}

				if ( ret == NPI_LNX_SUCCESS )
				{
				    // 2. Start programming: buffer to flash
				    ret = Hal_write_xdata_memory(DUP_FCTL, 0x01);
				}
				else
				{
					break;
				}


			    // 3. Wait until flash controller is done
			    uint8 byte = 0;
			    do {
			    	if ( ret == NPI_LNX_SUCCESS )
			    	{
			    		ret = Hal_read_xdata_memory(DUP_FCTL, &byte);
			    	}
			    	else
			    	{
			    		break;
			    	}
			    }
			    while (byte & 0x80);

			    uint8 debug_config = 0x22;
		    	if ( ret == NPI_LNX_SUCCESS )
		    	{
				    // Enable DMA (Disable DMA_PAUSE bit in debug configuration)
				    ret = Hal_debug_command(CMD_WR_CONFIG, &debug_config, 1);
		    	}
		    	else
		    	{
		    		break;
		    	}

			    debug_printf("[DEBUG INTERFACE] Write page %u: \t", address/DEBUG_FLASH_PROGRAM_BUFFER_BLOCK_SIZE);
				uint32 tmpAdd = address;
				for (; tmpAdd < (address + 32); tmpAdd++)
				{
					debug_printf("%.2X", flashBuffer[tmpAdd]);
				}
				debug_printf("\n");

		    	if ( ret == NPI_LNX_SUCCESS )
		    	{
					// Write one page at a time
					ret = Hal_write_flash_memory_block((uint8 *)&flashBuffer[address], address, DEBUG_FLASH_PROGRAM_BUFFER_BLOCK_SIZE);
		    	}
		    	else
		    	{
		    		break;
		    	}

		    	if ( ret == NPI_LNX_SUCCESS )
		    	{
				    // Disable DMA (Enable DMA_PAUSE bit in debug configuration)
				    debug_config = 0x26;
				    ret = Hal_debug_command(CMD_WR_CONFIG, &debug_config, 1);
		    	}
		    	else
		    	{
		    		break;
		    	}
			}
		}
	}

	// Send the asynchronous response back
	NPI_AsynchMsgCback(&pMsg);

	return ret;
}

/****************************************************************************
* @brief    Reads the flash into the buffer.
*
* @param    None.
*
* @return   STATUS.
******************************************************************************/
int Hal_read_from_chip_to_bufferReq()
{
	int ret = NPI_LNX_SUCCESS;

	npiMsgData_t pMsg;
	pMsg.len = 1;
	pMsg.subSys = RPC_SYS_DEBUG | RPC_CMD_AREQ;
	pMsg.cmdId = DEBUG_CMD_ID_READ_FROM_CHIP_TO_BUFFER_CNF;
	pMsg.pData[0] = DEBUG_FLASH_PROGRAM_SUCCESS;

	uint8 chipId = 0;
	ret = Hal_read_chip_id(&chipId);

	if (flashBuffer == NULL)
	{
		pMsg.pData[0] = DEBUG_FLASH_NO_BUFFER_CONFIGURED;
		ret = NPI_LNX_FAILURE;
	}
	if ((chipId != 0x95) && (chipId != 0x34))
	{
		// We only support CC2533/34. Return error.
		pMsg.pData[0] = DEBUG_FLASH_FAILED_TO_START_PROGRAM_BUFFER;
		ret = NPI_LNX_FAILURE;
	}

	if (ret == NPI_LNX_SUCCESS)
	{
	    debug_printf("[DEBUG INTERFACE] Begin reading from chip to buffer\n");
		// Program the buffer. This may take a while...
		uint32 address = bufferCfg.startAddress;
		uint32 flashSize = 0;
		ret = Hal_read_flash_size(&chipId, &flashSize);

		// Perform conversion to XDATA address
		uint16 xdataAddress = (uint16)bufferCfg.startAddress;
		// Get Bank number from upper bits
		uint8 bank = (bufferCfg.startAddress >> 15);
		if ( (bufferCfg.startAddress & 0x0000FFFF ) < 0x8000)
		{
			// Flash banks are always in the upper 32K, so add 0x8000 to even numbered banks
			xdataAddress += 0x8000;
		}

		// In case the wanted area crosses Bank boundaries we need to read in two steps.
		uint16 readSize = MIN (bufferCfg.size, (0x10000 - xdataAddress - 1));
		int bytesLeftToRead = bufferCfg.size - readSize;
		bufferCfg.numOfBytesWritten = 0;
		do
		{
		    debug_printf("[DEBUG INTERFACE] Reading %d bytes from @0x%.4X, bank %d\n",
		    		readSize,
		    		xdataAddress,
		    		bank);

			if ( ret == NPI_LNX_SUCCESS )
			{
				ret = Hal_read_flash_memory_block(bank, xdataAddress, readSize, &flashBuffer[address+bufferCfg.numOfBytesWritten]);
			}
			else
			{
				break;
			}

			bufferCfg.numOfBytesWritten += readSize;
			// Check if we initially split and hence have to change bank
			if (bytesLeftToRead > 0)
			{
				xdataAddress = 0x8000;
			}
			else
			{
				break;
			}
			readSize = MIN (bytesLeftToRead, (0x10000 - xdataAddress));
			bytesLeftToRead = bytesLeftToRead - readSize;
		} while (readSize > 0);

	    debug_printf("[DEBUG INTERFACE] Now in buffer, from @0x%.6X - @0x%.6X\n",
	    		bufferCfg.startAddress,
	    		bufferCfg.startAddress + bufferCfg.size);
	    uint16 k, j;
	    // Write 16 bytes per row
	    for (k = 0; k < (bufferCfg.size>>4); k++)
	    {
	    	// Tabulate line
	    	debug_printf("@0x%.6X \t", address+(k*16));
	    	for (j = 0; j < 16; j+=2)
	    	{
	    		// Write 16 bit at a time
		    	debug_printf("%.2X%.2X ",
		    			flashBuffer[address+(k*16)+j],
		    			flashBuffer[address+(k*16)+j+1]);
	    	}
	    	// New line
	    	debug_printf("\n");
	    }
	}

	// Send the asynchronous response back
	NPI_AsynchMsgCback(&pMsg);

	return ret;
}

/****************************************************************************
* @brief    Writes a block of data to the flash programmer buffer.
*
* @param    address     Buffer Address
* @param    values      Pointer to the array of bytes to write
* @param    num_bytes   Number of bytes to write
*
* @return   Status
******************************************************************************/
int Hal_write_buffer_memory_block(uint32 address, uint32 *values, uint16 num_bytes)
{
	memcpy((uint8 *)&flashBuffer[address], (uint8 *)values, num_bytes);
	bufferCfg.numOfBytesWritten += num_bytes;

	debug_printf("[DEBUG INTERFACE] Write buffer block %u: \t", address/num_bytes);
	uint32 tmpAdd = 0;
	for (; tmpAdd < num_bytes; tmpAdd++)
	{
		debug_printf("%.2X", flashBuffer[tmpAdd + address]);
	}
	debug_printf("\n");

	return NPI_LNX_SUCCESS;
}

/****************************************************************************
* @brief    Reads a block of data from the flash programmer buffer.
*
* @param    address     Buffer Address
* @param    values      Pointer to the array of bytes to read
* @param    num_bytes   Number of bytes to read
*
* @return   Status
******************************************************************************/
int Hal_read_buffer_memory_block(uint32 address, uint32 *values, uint16 num_bytes)
{
	printf("Reading %d bytes from buffer @0x%.6X\n", num_bytes, address);
	memcpy((uint8 *)values, &flashBuffer[address], num_bytes);
	return NPI_LNX_SUCCESS;
}

/**************************************************************************//**
* @brief    Sends a block of data over the debug interface using the
*           BURST_WRITE command.
*
* @param    src         Pointer to the array of input bytes
* @param    num_bytes   The number of input bytes
*
* @return   None.
******************************************************************************/
int Hal_burst_write_block(uint8 *src, uint16 num_bytes)
{
    uint16 i;
    int ret = NPI_LNX_SUCCESS;

    // Make sure DD is output
    SET_DD_OUTPUT(ret);


	if (ret == NPI_LNX_SUCCESS)
	{
		ret = Hal_write_debug_byte(CMD_BURST_WRITE | HIBYTE(num_bytes));
	}

	if (ret == NPI_LNX_SUCCESS)
	{
		ret = Hal_write_debug_byte(LOBYTE(num_bytes));
	}
    for (i = 0; i < num_bytes; i++)
    {
    	if (ret == NPI_LNX_SUCCESS)
    	{
    		ret = Hal_write_debug_byte(src[i]);
    	}
    }

    // Set DD as input

	if (ret == NPI_LNX_SUCCESS)
	{
		SET_DD_INPUT(ret);
	}

    // Wait for DUP to be ready

	if (ret == NPI_LNX_SUCCESS)
	{
		ret = Hal_wait_dup_ready();
	}


	if (ret == NPI_LNX_SUCCESS)
	{
		ret = Hal_read_debug_byte((uint8 *)&i); // ignore output
	}

    // Set DD as output

	if (ret == NPI_LNX_SUCCESS)
	{
		SET_DD_OUTPUT(ret);
	}
	return ret;
}


/**************************************************************************//**
* @brief    Issues a CHIP_ERASE command on the debug interface and waits for it
*           to complete.
*
* @return   STATUS
******************************************************************************/
int Hal_chip_erase(void)
{
	uint8 status = STATUS_CHIP_ERASE_BUSY_BM;
	int ret = NPI_LNX_SUCCESS;
	// Send command
	ret = Hal_debug_command(CMD_CHIP_ERASE, 0, 0);

	if (ret == NPI_LNX_SUCCESS)
	{
		// Wait for status bit 7 to go low
		do
		{
			ret = Hal_debug_command(CMD_READ_STATUS, &status, 1);
			if (ret != NPI_LNX_SUCCESS)
			{
				debug_printf("[DEBUG INTERFACE] Erase chip; couldn't read status\n");
			}
		} while((status & STATUS_CHIP_ERASE_BUSY_BM));
	}
	else
	{
		debug_printf("[DEBUG INTERFACE] Failed to send chip erase command\n");
	}

	if ((!(status & STATUS_CHIP_ERASE_BUSY_BM)) && (ret == NPI_LNX_SUCCESS) )
	{
		debug_printf("[DEBUG INTERFACE] Erase chip; chip erase no longer busy\n");
	}

	return ret;
}

/**************************************************************************//**
* @brief    Writes a block of data to the DUP's XDATA space.
*
* @param    address     XDATA start address
* @param    values      Pointer to the array of bytes to write
* @param    num_bytes   Number of bytes to write
*
* @return   STATUS
******************************************************************************/
int Hal_write_xdata_memory_block(uint16 address, uint8 *values, uint16 num_bytes)
{
    uint8 instr[3];
    uint16 i;
    int ret = NPI_LNX_SUCCESS;

    // MOV DPTR, address
    instr[0] = 0x90;
    instr[1] = HIBYTE(address);
    instr[2] = LOBYTE(address);
    ret = Hal_debug_command(CMD_DEBUG_INSTR_3B, instr, 3);
	debug_printf("Write XDATA address @%.2X%.2X", instr[1], instr[2]);

	debug_printf("Write ");
    for (i = 0; i < num_bytes; i++)
    {
    	debug_printf(" %.2X", values[i]);
    	// MOV A, values[i]
    	instr[0] = 0x74;
    	instr[1] = values[i];
    	if (ret == NPI_LNX_SUCCESS)
    	{
    		ret = Hal_debug_command(CMD_DEBUG_INSTR_2B, instr, 2);
    	}

    	// MOV @DPTR, A
    	instr[0] = 0xF0;
    	if (ret == NPI_LNX_SUCCESS)
    	{
    		ret = Hal_debug_command(CMD_DEBUG_INSTR_1B, instr, 1);
    	}

    	// INC DPTR
    	instr[0] = 0xA3;
    	if (ret == NPI_LNX_SUCCESS)
    	{
    		ret = Hal_debug_command(CMD_DEBUG_INSTR_1B, instr, 1);
    	}
    }
    debug_printf("\n");
	debug_printf("to @0x.%4X\n", address);

    return ret;
}


/**************************************************************************//**
* @brief    Writes a byte to a specific address in the DUP's XDATA space.
*
* @param    address     XDATA address
* @param    value       Value to write
*
* @return   STATUS
******************************************************************************/
int Hal_write_xdata_memory(uint16 address, uint8 value)
{
    uint8 instr[3];
    int ret = NPI_LNX_SUCCESS;

    // MOV DPTR, address
    instr[0] = 0x90;
    instr[1] = HIBYTE(address);
    instr[2] = LOBYTE(address);
    ret = Hal_debug_command(CMD_DEBUG_INSTR_3B, instr, 3);

    // MOV A, values[i]
    instr[0] = 0x74;
    instr[1] = value;
	if (ret == NPI_LNX_SUCCESS)
	{
		ret = Hal_debug_command(CMD_DEBUG_INSTR_2B, instr, 2);
	}

    // MOV @DPTR, A
    instr[0] = 0xF0;
    if (ret == NPI_LNX_SUCCESS)
    {
    	ret = Hal_debug_command(CMD_DEBUG_INSTR_1B, instr, 1);
    }

    return ret;
}

/**************************************************************************//**
* @brief    Read a block from a specific address in the DUP's XDATA space.
*
* @param    address     XDATA address
*
* Output:
* @param	values		- pointer to read block
* @param	num_bytes	- size of block, in number of bytes
*
* @return   STATUS
******************************************************************************/
int Hal_read_xdata_memory_block(uint16 address,
        uint8 *values,
        uint16 num_bytes)
{
    uint16 i;
    int ret = NPI_LNX_SUCCESS;
    for (i = 0; i < num_bytes; i++)
    {
    	if (ret == NPI_LNX_SUCCESS)
    	{
    		ret = Hal_read_xdata_memory(address+i, &values[i]);
    	}
    	else
    	{
    		break;
    	}
    }

    return ret;
}

/**************************************************************************//**
* @brief    Read a byte from a specific address in the DUP's XDATA space.
*
* @param    address     XDATA address
*
* Output
* @param	byte		- Read byte from XDATA
*
* @return   STATUS
******************************************************************************/
int Hal_read_xdata_memory(uint16 address, uint8 *byte)
{
    uint8 instr[3];
    int ret = NPI_LNX_SUCCESS;

    // MOV DPTR, address
    instr[0] = 0x90;
    instr[1] = HIBYTE(address);
    instr[2] = LOBYTE(address);
    ret = Hal_debug_command(CMD_DEBUG_INSTR_3B, instr, 3);

    // MOVX A, @DPTR
    instr[0] = 0xE0;
    if (ret == NPI_LNX_SUCCESS)
    {
    	ret = Hal_debug_command(CMD_DEBUG_INSTR_1B, instr, 1);
    	if (ret == NPI_LNX_SUCCESS)
    	{
    		*byte = instr[0];
    		debug_printf("[DEBUG INTERFACE] Read XDATA memory, SUCCESS\n");
    	}
    	else
		{
			debug_printf("[DEBUG INTERFACE] Read XDATA memory, failed CMD_DEBUG_INSTR_1B\n");
		}
    }
	else
	{
		debug_printf("[DEBUG INTERFACE] Read XDATA memory, failed CMD_DEBUG_INSTR_3B\n");
	}

    return ret;
}


/**************************************************************************//**
* @brief    Reads 1-32767 bytes from DUP's flash to a given buffer on the
*           programmer.
*
* @param    bank        Flash bank to read from [0-7]
* @param    address     Flash memory start address [0x0000 - 0x7FFF]
* @param    values      Pointer to destination buffer.
*
* @return   STATUS
******************************************************************************/
int Hal_read_flash_memory_block(uint8 bank,uint16 flash_addr,
                             uint16 num_bytes, uint8 *values)
{
    uint8 instr[3];
    uint16 i;
    int ret = NPI_LNX_SUCCESS;
//    uint16 xdata_addr = (0x8000 + flash_addr);
    uint16 xdata_addr = flash_addr;

    // 1. Map flash memory bank to XDATA address 0x8000-0xFFFF
    ret = Hal_write_xdata_memory(DUP_MEMCTR, bank);

    // 2. Move data pointer to XDATA address (MOV DPTR, xdata_addr)
    instr[0] = 0x90;
    instr[1] = HIBYTE(xdata_addr);
    instr[2] = LOBYTE(xdata_addr);
    if (ret == NPI_LNX_SUCCESS)
    {
    	ret = Hal_debug_command(CMD_DEBUG_INSTR_3B, instr, 3);
    }

    for (i = 0; i < num_bytes; i++)
    {
        // 3. Move value pointed to by DPTR to accumulator (MOVX A, @DPTR)
        instr[0] = 0xE0;
        if (ret == NPI_LNX_SUCCESS)
        {
        	ret = Hal_debug_command(CMD_DEBUG_INSTR_1B, instr, 1);
            if (ret == NPI_LNX_SUCCESS)
            {
            	values[i] = instr[0];
            }
        }

        // 4. Increment data pointer (INC DPTR)
        instr[0] = 0xA3;
        if (ret == NPI_LNX_SUCCESS)
        {
        	ret = Hal_debug_command(CMD_DEBUG_INSTR_1B, instr, 1);
        }
        else
        {
        	break;
        }
    }

    return ret;
}


/**************************************************************************//**
* @brief    Writes 4-2048 bytes to DUP's flash memory. Parameter \c num_bytes
*           must be a multiple of 4.
*
* @param    src         Pointer to programmer's source buffer (in XDATA space)
* @param    start_addr  FLASH memory start address [0x0000 - 0x7FFF]
* @param    num_bytes   Number of bytes to transfer [4-1024]
*
* @return   STATUS
******************************************************************************/
int Hal_write_flash_memory_block(uint8 *src, uint32 start_addr,
                              uint16 num_bytes)
{
	int ret = NPI_LNX_SUCCESS;
    // 1. Update LEN value in DUP's DMA descriptors
	dma_desc_0[4] = HIBYTE(num_bytes);  // LEN, DBG => ram
	dma_desc_0[5] = LOBYTE(num_bytes);  // LEN, DBG => ram
	dma_desc_1[4] = HIBYTE(num_bytes);  // LEN, ram => flash
	dma_desc_1[5] = LOBYTE(num_bytes);  // LEN, ram => flash

    // 2. Write the 2 DMA descriptors to RAM
    ret = Hal_write_xdata_memory_block(ADDR_DMA_DESC_0, dma_desc_0, 8);
    if (ret == NPI_LNX_SUCCESS)
    {
    	ret = Hal_write_xdata_memory_block(ADDR_DMA_DESC_1, dma_desc_1, 8);
    }

    // 3. Set DMA controller pointer to the DMA descriptors
    if (ret == NPI_LNX_SUCCESS)
    {
    	ret = Hal_write_xdata_memory(DUP_DMA0CFGH, HIBYTE(ADDR_DMA_DESC_0));
    }
    if (ret == NPI_LNX_SUCCESS)
    {
    	ret = Hal_write_xdata_memory(DUP_DMA0CFGL, LOBYTE(ADDR_DMA_DESC_0));
    }
    if (ret == NPI_LNX_SUCCESS)
    {
    	ret = Hal_write_xdata_memory(DUP_DMA1CFGH, HIBYTE(ADDR_DMA_DESC_1));
    }
    if (ret == NPI_LNX_SUCCESS)
    {
    	ret = Hal_write_xdata_memory(DUP_DMA1CFGL, LOBYTE(ADDR_DMA_DESC_1));
    }

    // 4. Set Flash controller start address (wants 16MSb of 18 bit address)
    if (ret == NPI_LNX_SUCCESS)
    {
    	ret = Hal_write_xdata_memory(DUP_FADDRH, HIBYTE( (start_addr>>2) ));
    }
    if (ret == NPI_LNX_SUCCESS)
    {
    	ret = Hal_write_xdata_memory(DUP_FADDRL, LOBYTE( (start_addr>>2) ));
    }

    // 5. Arm DBG=>buffer DMA channel and start burst write
    if (ret == NPI_LNX_SUCCESS)
    {
    	ret = Hal_write_xdata_memory(DUP_DMAARM, CH_DBG_TO_BUF0);
    }

    debug_printf("[DEBUG INTERFACE] Burst write from @%p to @0x%.4X\t", src, start_addr>>2);
    uint32 tmpAdd;
    for (tmpAdd = 0; tmpAdd < 32; tmpAdd++)
    {
    	debug_printf("%.2X", src[tmpAdd]);
    }
    debug_printf("\n");
    if (ret == NPI_LNX_SUCCESS)
    {
    	ret = Hal_burst_write_block(src, num_bytes);
    }

    // 6. Start programming: buffer to flash
    if (ret == NPI_LNX_SUCCESS)
    {
    	ret = Hal_write_xdata_memory(DUP_DMAARM, CH_BUF0_TO_FLASH);
    }
    if (ret == NPI_LNX_SUCCESS)
    {
    	ret = Hal_write_xdata_memory(DUP_FCTL, 0x06);
    }

    // 7. Wait until flash controller is done
    uint8 byte = 0;
    do {
        if (ret == NPI_LNX_SUCCESS)
        {
        	ret = Hal_read_xdata_memory(DUP_FCTL, &byte);
        }
        else
        {
        	break;
        }
    }
    while (byte & 0x80);

    return ret;
}

/**************************************************************************************************
 * @fn      HalGpioDDClose
 *
 * @brief   Close DD IO
 *
 * @param   None
 *
 * @return  None
 **************************************************************************************************/
void HalGpioDDClose(void)
{
	printf("dd close\n");
	close(gpioDDFd_val);
	if (gpioDDFd_level_val_exists == TRUE)
	{
		close(gpioDDFd_level_val);
	}
	close(gpioDDFd_gpio_dir);
}

/**************************************************************************************************
 * @fn      HalGpioDCClose
 *
 * @brief   Close DD IO
 *
 * @param   None
 *
 * @return  None
 **************************************************************************************************/
void HalGpioDCClose(void)
{
	printf("dc close\n");
	close(gpioDCFd);
}

/**************************************************************************************************
 * @fn      HalGpioDDInit
 *
 *
 * @brief   Initialise DD GPIO.
 *
 * @param   gpioCfg - DD pin configuration parameters
 *
 * @return  STATUS
 **************************************************************************************************/
int HalGpioDDInit(halGpioCfg_t *gpioCfg)
{
	memcpy(ddGpioCfg.gpio.value,
			gpioCfg->gpio.value,
			strlen(gpioCfg->gpio.value));
	debug_printf("[GPIO]ddGpioCfg.gpio.value = '%s'\n", ddGpioCfg.gpio.value);
	memcpy(ddGpioCfg.gpio.direction,
			gpioCfg->gpio.direction,
			strlen(gpioCfg->gpio.direction));
	debug_printf("[GPIO]ddGpioCfg.gpio.direction = '%s'\n", ddGpioCfg.gpio.direction);

	ddGpioCfg.gpio.active_high_low = gpioCfg->gpio.active_high_low;

#ifdef DD_INTERRUPT
	memcpy(ddGpioCfg.gpio.edge,
			gpioCfg->gpio.edge,
			strlen(gpioCfg->gpio.edge));
	debug_printf("[GPIO]ddGpioCfg.gpio.edge = '%s'\n", ddGpioCfg.gpio.edge);
#endif

	if ( ( gpioCfg->levelshifter.value) &&
		 ( gpioCfg->levelshifter.active_high_low) &&
		 ( gpioCfg->levelshifter.direction))
	{

		memcpy(ddGpioCfg.levelshifter.value,
				gpioCfg->levelshifter.value,
				strlen(gpioCfg->levelshifter.value));
		debug_printf("[GPIO]ddGpioCfg.levelshifter.value = '%s'\n", ddGpioCfg.levelshifter.value);
		memcpy(ddGpioCfg.levelshifter.direction,
				gpioCfg->levelshifter.direction,
				strlen(gpioCfg->levelshifter.direction));
		ddGpioCfg.levelshifter.active_high_low = gpioCfg->levelshifter.active_high_low;
		debug_printf("[GPIO]ddGpioCfg.levelshifter.direction = '%s'\n", ddGpioCfg.levelshifter.direction);

		//open the GPIO DIR file for the level shifter direction signal
		gpioDDFd_level_val = open(ddGpioCfg.levelshifter.direction, O_RDWR);
		if(gpioDDFd_level_val == 0)
		{
			perror(ddGpioCfg.levelshifter.direction);
			debug_printf("\n[GPIO]%s open failed\n",ddGpioCfg.levelshifter.direction);
			npi_ipc_errno = NPI_LNX_ERROR_HAL_DBG_IFC_DD_LVLSHFT_DIR_OPEN;
			return NPI_LNX_FAILURE;
		}

		//Set the direction of the GPIO to output
		if (ERROR == write(gpioDDFd_level_val, "out", 3))
		{
			perror(ddGpioCfg.levelshifter.direction);
			debug_printf("\n[GPIO]can't write in %s \n",ddGpioCfg.levelshifter.direction);
			npi_ipc_errno = NPI_LNX_ERROR_HAL_DBG_IFC_DD_LVLSHFT_DIR_WRITE;
			return NPI_LNX_FAILURE;
		}
		//close the DIR file
		close(gpioDDFd_level_val);

		//open the GPIO VALUE file for the level shifter direction signal
		gpioDDFd_level_val = open(ddGpioCfg.levelshifter.value, O_RDWR);
		if(gpioDDFd_level_val == 0)
		{
			perror(ddGpioCfg.levelshifter.value);
			debug_printf("[GPIO]%s open failed\n",ddGpioCfg.levelshifter.value);
			npi_ipc_errno = NPI_LNX_ERROR_HAL_DBG_IFC_DD_LVLSHFT_VAL_OPEN;
			return NPI_LNX_FAILURE;
		}

		//Set the value of the GPIO to 0 (level shifter direction from Host to CC2533)

		if (ERROR == write(gpioDDFd_val, "1", 1))
		{
			perror(ddGpioCfg.levelshifter.value);
			debug_printf("\n[GPIO]can't write in %s \n",ddGpioCfg.levelshifter.value);
			npi_ipc_errno = NPI_LNX_ERROR_HAL_DBG_IFC_DD_LVLSHFT_VAL_WRITE;
			return NPI_LNX_FAILURE;
		}
		//keep the level val file open
		gpioDDFd_level_val_exists = TRUE;
	}
	else
	{
		debug_printf("WARNING: Wrong Configuration File, one of the  following Key value are missing for DD.Level Shifter definition: '\n");
		debug_printf("value: %s\n", ddGpioCfg.gpio.value);
		debug_printf("direction: %s\n", ddGpioCfg.gpio.direction);
		debug_printf("active_high_low: %d\n", ddGpioCfg.gpio.active_high_low);
		debug_printf("Level Shifter is optional, please check if you need it or not before continuing...\n");
		gpioDDFd_level_val_exists = FALSE;
	}
	//TODO: Lock the shift register GPIO.

	//open the DD GPIO DIR file
	gpioDDFd_gpio_dir = open(ddGpioCfg.gpio.direction, O_RDWR);
	if(gpioDDFd_gpio_dir == 0)
	{
		perror(ddGpioCfg.gpio.direction);
		debug_printf("[GPIO]%s open failed\n",ddGpioCfg.gpio.direction);
		npi_ipc_errno = NPI_LNX_ERROR_HAL_DBG_IFC_DD_GPIO_DIR_OPEN;
	    return NPI_LNX_FAILURE;
	}

	//Set DD GPIO as output
	if(ERROR == write(gpioDDFd_gpio_dir, "out", 3))
	{
		perror(ddGpioCfg.gpio.direction);
		debug_printf("\n[GPIO]can't write in %s \n",ddGpioCfg.gpio.direction);
		npi_ipc_errno = NPI_LNX_ERROR_HAL_DBG_IFC_DD_GPIO_DIR_WRITE;
	    return NPI_LNX_FAILURE;
	}
	//keep DD DIR file open

	//open the DD GPIO VALUE file so it can be written to using the file handle later
	gpioDDFd_val = open(ddGpioCfg.gpio.value, O_RDWR| O_NONBLOCK);
	if(gpioDDFd_val == 0)
	{
		perror(ddGpioCfg.gpio.value);
		debug_printf("%s open failed\n",ddGpioCfg.gpio.value);
		npi_ipc_errno = NPI_LNX_ERROR_HAL_DBG_IFC_DD_GPIO_VAL_OPEN;
	    return NPI_LNX_FAILURE;
	}

	//Set DC GPIO to 0 as default

	if (ERROR == write(gpioDDFd_val, "0", 1))
	{
		perror(ddGpioCfg.gpio.value);
		debug_printf("\n[GPIO]can't write in %s \n",ddGpioCfg.gpio.value);
		npi_ipc_errno = NPI_LNX_ERROR_HAL_DBG_IFC_DD_GPIO_VAL_WRITE;
	    return NPI_LNX_FAILURE;
	}

	return NPI_LNX_SUCCESS;
}
/**************************************************************************************************
 * @fn      halGpioDDSetDirection
 *
 *
 * @brief   Initialise DD GPIO.
 *
 * @param   direction - direction of pin
 *
 * @return  STATUS
 **************************************************************************************************/
int halGpioDDSetDirection(uint8 direction)
{
	if(gpioDDFd_gpio_dir == 0)
	{
		perror(ddGpioCfg.gpio.direction);
		debug_printf("[GPIO]%s not open\n",ddGpioCfg.gpio.direction);
		npi_ipc_errno = NPI_LNX_ERROR_HAL_DBG_IFC_DD_GPIO_DIR_OPEN;
	    return NPI_LNX_FAILURE;
	}
	if( (gpioDDFd_level_val == 0) && (gpioDDFd_level_val_exists == TRUE))
	{
		perror(ddGpioCfg.levelshifter.value);
		debug_printf("[GPIO]%s not open\n",ddGpioCfg.levelshifter.value);
		npi_ipc_errno = NPI_LNX_ERROR_HAL_DBG_IFC_DD_GPIO_DIR_OPEN;
	    return NPI_LNX_FAILURE;
	}

	if (direction == HAL_GPIO_INPUT)
	{
		//Set DD GPIO as input
		if(ERROR == write(gpioDDFd_gpio_dir, "in", 2))
		{
			perror(ddGpioCfg.gpio.direction);
			debug_printf("\n[GPIO]can't write in %s \n",ddGpioCfg.gpio.direction);
			npi_ipc_errno = NPI_LNX_ERROR_HAL_DBG_IFC_DD_GPIO_DIR_WRITE;
			return NPI_LNX_FAILURE;
		}
		if  (gpioDDFd_level_val_exists == TRUE)
		{
			if(ERROR == write(gpioDDFd_level_val, "0", 1))
			{
				perror(ddGpioCfg.levelshifter.value);
				debug_printf("\n[GPIO]can't write in %s \n",ddGpioCfg.levelshifter.value);
				npi_ipc_errno = NPI_LNX_ERROR_HAL_DBG_IFC_DD_GPIO_DIR_WRITE;
				return NPI_LNX_FAILURE;
			}
		}

		debug_printf("Direction set successfully for DD to IN\n");
	}
	else
	{
		//Set DD GPIO as output
		if(ERROR == write(gpioDDFd_gpio_dir, "out", 3))
		{
			perror(ddGpioCfg.gpio.direction);
			debug_printf("\n[GPIO]can't write in %s \n",ddGpioCfg.gpio.direction);
			npi_ipc_errno = NPI_LNX_ERROR_HAL_DBG_IFC_DD_GPIO_DIR_WRITE;
			return NPI_LNX_FAILURE;
		}
		if (gpioDDFd_level_val_exists == TRUE)
		{
			if(ERROR == write(gpioDDFd_level_val, "1", 1))
			{
				perror(ddGpioCfg.levelshifter.value);
				debug_printf("\n[GPIO]can't write in %s \n",ddGpioCfg.levelshifter.value);
				npi_ipc_errno = NPI_LNX_ERROR_HAL_DBG_IFC_DD_GPIO_DIR_WRITE;
				return NPI_LNX_FAILURE;
			}
		}

		debug_printf("Direction set successfully for DD to OUT\n");
	}

	return NPI_LNX_SUCCESS;
}

/**************************************************************************************************
 * @fn      HalGpioDCInit
 *
 *
 * @brief   Initialise DC GPIO.
 *
 * @param   gpioCfg - DC pin configuration parameters
 *
 * @return  STATUS
 **************************************************************************************************/
int HalGpioDCInit(halGpioCfg_t *gpioCfg)
{
	memcpy(dcGpioCfg.gpio.value,
			gpioCfg->gpio.value,
			strlen(gpioCfg->gpio.value));
	debug_printf("[GPIO]dcGpioCfg.gpio.value = '%s'\n", dcGpioCfg.gpio.value);
	memcpy(dcGpioCfg.gpio.direction,
			gpioCfg->gpio.direction,
			strlen(gpioCfg->gpio.direction));
	debug_printf("[GPIO]dcGpioCfg.gpio.direction = '%s'\n", dcGpioCfg.gpio.direction);
	dcGpioCfg.gpio.active_high_low = gpioCfg->gpio.active_high_low;

	if ( ( gpioCfg->levelshifter.value) &&
		 ( gpioCfg->levelshifter.active_high_low) &&
		 ( gpioCfg->levelshifter.direction))
	{
		memcpy(dcGpioCfg.levelshifter.value,
				gpioCfg->levelshifter.value,
				strlen(gpioCfg->levelshifter.value));
		debug_printf("[GPIO]dcGpioCfg.levelshifter.value = '%s'\n", dcGpioCfg.levelshifter.value);
		memcpy(dcGpioCfg.levelshifter.direction,
				gpioCfg->levelshifter.direction,
				strlen(gpioCfg->levelshifter.direction));
		dcGpioCfg.levelshifter.active_high_low = gpioCfg->levelshifter.active_high_low;
		debug_printf("[GPIO]dcGpioCfg.levelshifter.direction = '%s'\n", dcGpioCfg.levelshifter.direction);

		//open the GPIO DIR file for the level shifter direction signal
		gpioDCFd = open(dcGpioCfg.levelshifter.direction, O_RDWR);
		if(gpioDCFd == 0)
		{
			perror(dcGpioCfg.levelshifter.direction);
			debug_printf("[GPIO]%s open failed\n",dcGpioCfg.levelshifter.direction);
			npi_ipc_errno = NPI_LNX_ERROR_HAL_DBG_IFC_DC_LVLSHFT_DIR_OPEN;
			return NPI_LNX_FAILURE;
		}

		//Set the direction of the GPIO to output
		if(ERROR == write(gpioDCFd, "out", 3))
		{
			perror(dcGpioCfg.levelshifter.direction);
			debug_printf("\n[GPIO]can't write in %s \n",dcGpioCfg.levelshifter.direction);
			npi_ipc_errno = NPI_LNX_ERROR_HAL_DBG_IFC_DC_LVLSHFT_DIR_WRITE;
			return NPI_LNX_FAILURE;
		}
		//close the DIR file
		close(gpioDCFd);

		//open the GPIO VALUE file for the level shifter direction signal
		gpioDCFd = open(dcGpioCfg.levelshifter.value, O_RDWR);
		if(gpioDCFd == 0)
		{
			perror(dcGpioCfg.levelshifter.value);
			debug_printf("[GPIO]%s open failed\n",dcGpioCfg.levelshifter.value);
			npi_ipc_errno = NPI_LNX_ERROR_HAL_DBG_IFC_DC_LVLSHFT_VAL_OPEN;
			return NPI_LNX_FAILURE;
		}

		//Set the value of the GPIO to 0 (level shifter direction from Host to CC2531)
		if(ERROR == write(gpioDCFd, "1", 1))
		{
			perror(dcGpioCfg.levelshifter.value);
			debug_printf("\n[GPIO]can't write in %s \n",dcGpioCfg.levelshifter.value);
			npi_ipc_errno = NPI_LNX_ERROR_HAL_DBG_IFC_DC_LVLSHFT_VAL_WRITE;
			return NPI_LNX_FAILURE;
		}
		//close the DIR file
		close(gpioDCFd);
	}
	else
	{
		debug_printf("WARNING: Wrong Configuration File, one of the  following Key value are missing for DC.Level Shifter definition: '\n");
		debug_printf("value: %s\n", dcGpioCfg.gpio.value);
		debug_printf("direction: %s\n", dcGpioCfg.gpio.direction);
		debug_printf("active_high_low: %d\n", dcGpioCfg.gpio.active_high_low);
		debug_printf("Level Shifter is optional, please check if you need it or not before continuing...\n");
	}

	//open the DC GPIO DIR file
	gpioDCFd = open(dcGpioCfg.gpio.direction, O_RDWR);
	if(gpioDCFd == 0)
	{
		perror(dcGpioCfg.gpio.direction);
		debug_printf("[GPIO]%s open failed\n",dcGpioCfg.gpio.direction);
		npi_ipc_errno = NPI_LNX_ERROR_HAL_DBG_IFC_DC_GPIO_DIR_OPEN;
	    return NPI_LNX_FAILURE;
	}

	//Set DC GPIO as output
	if(ERROR == write(gpioDCFd, "out", 3))
	{
		perror(dcGpioCfg.gpio.direction);
		debug_printf("\n[GPIO]can't write in %s \n",dcGpioCfg.gpio.direction);
		npi_ipc_errno = NPI_LNX_ERROR_HAL_DBG_IFC_DC_GPIO_DIR_WRITE;
	    return NPI_LNX_FAILURE;
	}
	//close DC DIR file
	close(gpioDCFd);

	//open the DC GPIO VALUE file so it can be written to using the file handle later
	gpioDCFd = open(dcGpioCfg.gpio.value, O_RDWR);
	if(gpioDCFd == 0)
	{
		perror(dcGpioCfg.gpio.value);
		debug_printf("[GPIO]%s open failed\n",dcGpioCfg.gpio.value);
		npi_ipc_errno = NPI_LNX_ERROR_HAL_DBG_IFC_DC_GPIO_VAL_OPEN;
	    return NPI_LNX_FAILURE;
	}

	//Set DC GPIO to 0 as default

	if (ERROR == write(gpioDCFd, "0", 1))
	{
		perror(dcGpioCfg.gpio.value);
		debug_printf("\n[GPIO]can't write in %s \n",dcGpioCfg.gpio.value);
		npi_ipc_errno = NPI_LNX_ERROR_HAL_DBG_IFC_DC_GPIO_VAL_WRITE;
	    return NPI_LNX_FAILURE;
	}

	return NPI_LNX_SUCCESS;
}

/**************************************************************************************************
 * @fn      halGpioDCSet
 *
 *
 * @brief   Set debug clock.
 *
 * @param
 *
 * @return  STATUS
 **************************************************************************************************/
int halGpioDCSet(uint8 state)
{
	if(state == 0)
	{
		debug_printf("[GPIO]DC set to low\n");
		if (ERROR == write(gpioDCFd, "0", 1))
		{
			perror(dcGpioCfg.gpio.value);
			debug_printf("\n[GPIO]can't write in %s , is something already accessing it? abort everything for debug purpose...\n",dcGpioCfg.gpio.value);
			npi_ipc_errno = NPI_LNX_ERROR_HAL_DBG_IFC_DC_GPIO_VAL_WRITE_SET_LOW;
		    return NPI_LNX_FAILURE;
		}
	}
	else
	{
		debug_printf("[GPIO]DC set to High\n");
    	if(ERROR == write(gpioDCFd, "1", 1))
		{
			perror(dcGpioCfg.gpio.value);
			debug_printf("\n[GPIO]can't write in %s , is something already accessing it? abort everything for debug purpose...\n",dcGpioCfg.gpio.value);
			npi_ipc_errno = NPI_LNX_ERROR_HAL_DBG_IFC_DC_GPIO_VAL_WRITE_SET_HIGH;
			return NPI_LNX_FAILURE;
		}
	}

	return NPI_LNX_SUCCESS;
}

/**************************************************************************************************
 * @fn      HalGpioDCCheck
 *
 *
 * @brief   Check debug data (DD) Clear.
 *
 * @param   state	- Active  or  Inactive
 * @param	match	- pointer to result of match
 *
 * @return  None
 **************************************************************************************************/
int HalGpioDCCheck(uint8 state, uint8 *match)
{
	char dc=2;
	lseek(gpioDCFd,0,SEEK_SET);
	if(ERROR == read(gpioDCFd,&dc, 1))
	{
		perror(dcGpioCfg.gpio.value);
		debug_printf("\ncan't read in %s , is something already accessing it? abort everything for debug purpose...\n",dcGpioCfg.gpio.value);
		npi_ipc_errno = NPI_LNX_ERROR_HAL_DBG_IFC_DC_GPIO_VAL_READ;
		return NPI_LNX_FAILURE;
	}

	debug_printf("[GPIO]===>check DC: %c  (%d) \n", dc, state);

	*match = (state == ((dc == '1') ? 1 : 0));

	debug_printf("[GPIO] match %d\n", *match);

	return NPI_LNX_SUCCESS;
}

/**************************************************************************************************
 * @fn      halGpioDDSet
 *
 *
 * @brief   Set DD.
 *
 * @param
 *
 * @return  STATUS
 **************************************************************************************************/
int halGpioDDSet(uint8 state)
{
	if(state == 0)
	{
		debug_printf("[GPIO]DD set to low\n");
		if (ERROR == write(gpioDDFd_val, "0", 1))
		{
			perror(ddGpioCfg.gpio.value);
			debug_printf("\n[GPIO]can't write in %s , is something already accessing it? abort everything for debug purpose...\n",ddGpioCfg.gpio.value);
			npi_ipc_errno = NPI_LNX_ERROR_HAL_DBG_IFC_DD_GPIO_VAL_WRITE_SET_LOW;
		    return NPI_LNX_FAILURE;
		}
	}
	else
	{
		debug_printf("[GPIO]DD set to High\n");
    	if(ERROR == write(gpioDDFd_val, "1", 1))
		{
			perror(ddGpioCfg.gpio.value);
			debug_printf("\n[GPIO]can't write in %s , is something already accessing it? abort everything for debug purpose...\n",ddGpioCfg.gpio.value);
			npi_ipc_errno = NPI_LNX_ERROR_HAL_DBG_IFC_DD_GPIO_VAL_WRITE_SET_HIGH;
			return NPI_LNX_FAILURE;
		}
	}

	return NPI_LNX_SUCCESS;

}

/**************************************************************************************************
 * @fn      HalGpioDDCheck
 *
 *
 * @brief   Check DD Clear.
 *
 * @param   state	- Active  or  Inactive
 * @param	match	- result. Match or not
 *
 * @return  STATUS
 **************************************************************************************************/
int HalGpioDDCheck(uint8 state, uint8 *match)
{
	char dd=2;
	lseek(gpioDDFd_val,0,SEEK_SET);
	if(ERROR == read(gpioDDFd_val,&dd, 1))
	{
		perror(ddGpioCfg.gpio.value);
		debug_printf("\n[GPIO]can't read in %s , is something already accessing it? abort everything for debug purpose...\n",ddGpioCfg.gpio.value);
		npi_ipc_errno = NPI_LNX_ERROR_HAL_DBG_IFC_DD_GPIO_VAL_READ_FAILED;
		return NPI_LNX_FAILURE;
	}

	debug_printf("[GPIO]===>check dd: %c  (%d) \n", dd, state);

	*match = (state == ((dd == '1') ? 1 : 0));

	debug_printf("[GPIO] match %d\n", *match);

	return NPI_LNX_SUCCESS;
}

/**************************************************************************************************
 * @fn          HalGpioWaitDDClr
 *
 * @brief       Check that DD is low, if not, wait until it gets low.
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * None.
 *
 * @return      STATUS
 **************************************************************************************************
 */
int HalGpioWaitDDClr(void)
{
	char dd= '1';
	int ret = NPI_LNX_SUCCESS;

	debug_printf("[GPIO]Wait DD Low, \n");

	struct pollfd ufds[1];
	int pollRet;
	ufds[0].fd = gpioDDFd_val;
	ufds[0].events = POLLIN | POLLPRI;
//	ufds[0].events = POLLPRI;

#ifdef __DEBUG_TIME__
	time_printf("DD: wait to go Low\n");
#endif //(defined __DEBUG_TIME__)

	while(dd == '1')
	{
		pollRet = poll((struct pollfd*)&ufds, 1, 100);
		if (pollRet == -1)
		{
			// Error occured in poll()
			perror("poll");
		}
		else if (pollRet == 0)
		{
			// Timeout
			printf("[GPIO][WARNING] Waiting for DD to go low timed out.\n");
			npi_ipc_errno = NPI_LNX_ERROR_HAL_DBG_IFC_WAIT_DD_CLEAR_POLL_TIMEDOUT;
			ret = NPI_LNX_FAILURE;
			break;
		}
		else
		{
			if ( (ufds[0].revents & POLLIN) || (ufds[0].revents & POLLPRI) )
			//if ( ufds[0].revents & POLLPRI)
			{
				lseek(gpioDDFd_val,0,SEEK_SET);
				if(ERROR == read(gpioDDFd_val,&dd, 1))
				{
					perror(ddGpioCfg.gpio.value);
					debug_printf("\n[GPIO]can't read in %s , is something already accessing it? abort everything for debug purpose...\n",ddGpioCfg.gpio.value);
					npi_ipc_errno = NPI_LNX_ERROR_HAL_DBG_IFC_WAIT_DD_CLEAR_READ_FAILED;
					return NPI_LNX_FAILURE;
				}
				debug_printf("[0x%.2X , %c(0x%.2X)]", atoi(&dd), dd, (unsigned int)dd);
			}
			else
			{
				printf("[GPIO](%d)", ufds[0].revents);
			}
		}
	}
#ifdef __DEBUG_TIME__
	char tmpStr[100];
	snprintf(tmpStr, sizeof(tmpStr), "dd: %c  (0x%02X)\n", dd, (unsigned int)dd);
	time_printf(tmpStr);
#endif //(defined __DEBUG_TIME__)


#ifdef __STRESS_TEST__
	time_printf("DD Low\n");
#endif //__STRESS_TEST__

  debug_printf("[GPIO]==>dd change to : %c  (0x%02X)\n", dd, (unsigned int)dd);

  return ret;
}

/**************************************************************************************************
 * @fn          HalGpioWaitDDSet
 *
 * @brief       Check that DD is High, if not, wait until it gets high, or times out.
 * 				0xFFFF means never time out.
 *
 * input parameters
 *
 * None
 *
 * output parameters
 *
 * None.
 *
 * @return      STATUS
 **************************************************************************************************
 */
int HalGpioWaitDDSet()
{
	char dd= '0';

	int ret = NPI_LNX_SUCCESS;

	debug_printf("[GPIO]Wait DD High, \n");

	struct pollfd ufds[1];
	int pollRet;
	ufds[0].fd = gpioDDFd_val;
	ufds[0].events = POLLIN | POLLPRI;

#ifdef __DEBUG_TIME__
	time_printf("DD: wait to go High\n");
#endif //(defined __DEBUG_TIME__)

	while( (dd == '0') )
	{
		pollRet = poll((struct pollfd*)&ufds, 1, 400);
		if (pollRet == -1)
		{
			// Error occured in poll()
			perror("poll");
		}
		else if (pollRet == 0)
		{
			// Timeout
			printf("[GPIO][WARNING] Waiting for DD to go high timed out.\n");
			npi_ipc_errno = NPI_LNX_ERROR_HAL_DBG_IFC_WAIT_DD_SET_POLL_TIMEDOUT;
			ret = NPI_LNX_FAILURE;
			break;
		}
		else
		{
			if ( (ufds[0].revents & POLLIN) || (ufds[0].revents & POLLPRI) )
			{
				lseek(gpioDDFd_val,0,SEEK_SET);
				if(ERROR == read(gpioDDFd_val,&dd, 1))
				{
					perror(ddGpioCfg.gpio.value);
					debug_printf("\n[GPIO]can't read in %s , is something already accessing it? abort everything for debug purpose...\n",ddGpioCfg.gpio.value);
					npi_ipc_errno = NPI_LNX_ERROR_HAL_DBG_IFC_WAIT_DD_SET_READ_FAILED;
					return NPI_LNX_FAILURE;
				}
			}
			else
			{
				printf("[GPIO](%d)", ufds[0].revents);
			}
		}
	}

#ifdef __DEBUG_TIME__
	char tmpStr[100];
	snprintf(tmpStr, sizeof(tmpStr), "DD: %c  (0x%02X)\n", dd, (unsigned int)dd);
	time_printf(tmpStr);
#endif //__DEBUG_TIME__

	debug_printf("[GPIO]==>DD change to : %c  (0x%02X) \n", dd, (unsigned int)dd);

	return ret;
}

/**************************************************************************************************
 **************************************************************************************************/
