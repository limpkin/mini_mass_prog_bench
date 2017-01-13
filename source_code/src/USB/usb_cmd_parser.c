/* CDDL HEADER START
 *
 * The contents of this file are subject to the terms of the
 * Common Development and Distribution License (the "License").
 * You may not use this file except in compliance with the License.
 *
 * You can obtain a copy of the license at src/license_cddl-1.0.txt
 * or http://www.opensolaris.org/os/licensing.
 * See the License for the specific language governing permissions
 * and limitations under the License.
 *
 * When distributing Covered Code, include this CDDL HEADER in each
 * file and include the License file at src/license_cddl-1.0.txt
 * If applicable, add the following below this CDDL HEADER, with the
 * fields enclosed by brackets "[]" replaced with your own identifying
 * information: Portions Copyright [yyyy] [name of copyright owner]
 *
 * CDDL HEADER END
 */
/*!  \file     usb_cmd_parser.c
*    \brief    USB communication communication parser
*    Created:  09/6/2014
*    Author:   Mathieu Stephan
*/
#include "smart_card_higher_level_functions.h"
#include "gui_smartcard_functions.h"
#include "logic_fwflash_storage.h"
#include "gui_screen_functions.h"
#include "gui_basic_functions.h"
#include "logic_aes_and_comms.h"
#include "gui_pin_functions.h"
#include "eeprom_addresses.h"
#include "watchdog_driver.h"
#include "logic_smartcard.h"
#include "usb_cmd_parser.h"
#include "timer_manager.h"
#include "oled_wrapper.h"
#include "logic_eeprom.h"
#include "hid_defines.h"
#include "mini_inputs.h"
#include <avr/eeprom.h>
#include "mooltipass.h"
#include "mini_leds.h"
#include "node_mgmt.h"
#include "flash_mem.h"
#include <string.h>
#include <ctype.h>
#include <stdio.h>
#include "delays.h"
#include "utils.h"
#include "stack.h"
#include "usb.h"
#include "rng.h"

// Bool to know if we can import in the media part of flash
uint8_t mediaFlashImportApproved = FALSE;
// Media flash import temp offset
uint16_t mediaFlashImportOffset;
// Media flash import temp page
uint16_t mediaFlashImportPage;
/* External var, addr of bottom of stack (usually located at end of RAM)*/
extern uint8_t __stack;
/* External var, end of known static RAM (to be filled by linker) */
extern uint8_t _end;


/*! \fn     leaveMemoryManagementMode(void)
*   \brief  Leave memory management mode
*/
void leaveMemoryManagementMode(void)
{
}

/*! \fn     usbCancelRequestReceived(void)
*   \brief  Check if a cancel request packet was received
*   \return RETURN_OK if packet received, RETURN_NOK otherwise
*/
RET_TYPE usbCancelRequestReceived(void)
{
    return RETURN_NOK;
}

/*! \fn     usbProcessIncoming(uint8_t caller_id)
*   \brief  Process a possible incoming USB packet
*   \param  caller_id   UID of the calling function
*/
void usbProcessIncoming(uint8_t caller_id)
{
    (void)caller_id;
    
    // Our USB data buffer
    uint8_t incomingData[RAWHID_TX_SIZE];
    
    // Try to read data from USB, return if we didn't receive anything
    if(usbRawHidRecv(incomingData) != RETURN_COM_TRANSF_OK)
    {
        return;
    }
    
    // Temp plugin return value, error by default
    uint8_t plugin_return_value = PLUGIN_BYTE_ERROR;

    // Use message structure
    usbMsg_t* msg = (usbMsg_t*)incomingData;

    // Get data len
    uint8_t datalen = msg->len;

    // Get data cmd
    uint8_t datacmd = msg->cmd;

    // Otherwise, process command
    switch(datacmd)
    {
        // ping command
        case CMD_PING :
        {
            usbHidSend(0, msg, 6);
            return;
        }

        // version command
        case CMD_VERSION :
        {            
            // Our Mooltipass version that will be returned to our application
            #ifndef MINI_VERSION
                const char mooltipass_version[] = FLASH_CHIP_STR "" MOOLTIPASS_VERSION;
            #else
                const char mooltipass_version[] = FLASH_CHIP_STR "" MOOLTIPASS_VERSION "" "_mini";
            #endif
            usbSendMessage(CMD_VERSION, sizeof(mooltipass_version), mooltipass_version);
            return;
        }
        
        // status command
        case CMD_MOOLTIPASS_STATUS :
        {
            uint8_t mp_status = 0x00;
            
            // Inform the plugin to inform the user to unlock his card
            usbSendMessage(CMD_MOOLTIPASS_STATUS, 1, &mp_status);
            return;            
        }

        // import media flash contents
        case CMD_IMPORT_MEDIA_START :
        {            
            // Set default addresses
            mediaFlashImportPage = GRAPHIC_ZONE_PAGE_START;
            plugin_return_value = PLUGIN_BYTE_OK;
            mediaFlashImportApproved = TRUE;
            mediaFlashImportOffset = 0;
            break;
        }

        // import media flash contents
        case CMD_IMPORT_MEDIA :
        {
            // Check if we actually approved the import, haven't gone over the flash boundaries, if we're correctly aligned page size wise
            if ((mediaFlashImportApproved == FALSE) || (mediaFlashImportPage >= GRAPHIC_ZONE_PAGE_END) || (mediaFlashImportOffset + datalen > BYTES_PER_PAGE))
            {
                plugin_return_value = PLUGIN_BYTE_ERROR;
                mediaFlashImportApproved = FALSE;
            }
            else
            {
                flashWriteBuffer(msg->body.data, mediaFlashImportOffset, datalen);
                mediaFlashImportOffset+= datalen;

                // If we just filled a page, flush it to the page
                if (mediaFlashImportOffset == BYTES_PER_PAGE)
                {
                    flashWriteBufferToPage(mediaFlashImportPage);
                    mediaFlashImportOffset = 0;
                    mediaFlashImportPage++;
                }
                plugin_return_value = PLUGIN_BYTE_OK;
            }
            break;
        }

        // end media flash import
        case CMD_IMPORT_MEDIA_END :
        {
            if ((mediaFlashImportApproved == TRUE) && (mediaFlashImportOffset != 0))
            {
                flashWriteBufferToPage(mediaFlashImportPage);
            }
            plugin_return_value = PLUGIN_BYTE_OK;
            mediaFlashImportApproved = FALSE;
            break;
        }
        
        // Get button pressed array
        case CMD_BUTTON_PRESSED :
        {
            uint8_t temp_array[9];
            get_and_clear_button_pressed_return(temp_array);
            usbSendMessage(CMD_BUTTON_PRESSED, sizeof(temp_array), temp_array);
            return;
        }
        
        // Get 32 random bytes
        case CMD_GET_RANDOM_NUMBER :
        {
            uint8_t randomBytes[32];
            fillArrayWithRandomBytes(randomBytes, 32);
            usbSendMessage(CMD_GET_RANDOM_NUMBER, 32, randomBytes);
            return;
        }  
        
        // Get number of available random bytes
        case CMD_GET_NB_RNG_B:
        {
            uint8_t available_bytes = rngGetBufferCount()*4;
            usbSendMessage(CMD_GET_NB_RNG_B, 1, &available_bytes);
            return;
        }
        
        // Programming done!
        case CMD_PROG_DONE:
        {
            programming_success(msg->body.data[0]);
            plugin_return_value = PLUGIN_BYTE_OK;
            break;
        }
        
        // Programming done!
        case CMD_PROG_FAILURE:
        {
            programming_failure(msg->body.data[0]);
            plugin_return_value = PLUGIN_BYTE_OK;
            break;
        }
        
        // Display lines
        case CMD_DISPLAY_LINE1:
        {
            plugin_return_value = PLUGIN_BYTE_OK;
            miniOledDrawRectangle(0, 0, SSD1305_OLED_WIDTH, 10, FALSE);
            miniOledPutstrXY(0, THREE_LINE_TEXT_FIRST_POS, OLED_LEFT, (char*)msg->body.data);
            break;
        }
        
        // Display lines
        case CMD_DISPLAY_LINE2:
        {
            plugin_return_value = PLUGIN_BYTE_OK;
            miniOledDrawRectangle(0, 10, SSD1305_OLED_WIDTH, 11, FALSE);
            miniOledPutstrXY(0, THREE_LINE_TEXT_SECOND_POS, OLED_LEFT, (char*)msg->body.data);
            break;
        }
        
        // Display lines
        case CMD_DISPLAY_LINE3:
        {
            plugin_return_value = PLUGIN_BYTE_OK;
            miniOledDrawRectangle(0, 21, SSD1305_OLED_WIDTH, 11, FALSE);
            miniOledPutstrXY(0, THREE_LINE_TEXT_THIRD_POS, OLED_LEFT, (char*)msg->body.data);
            break;
        }

        default :   return;
    }
    
    // Return an answer that was defined before calling break
    usbSendMessage(datacmd, 1, &plugin_return_value);
}

