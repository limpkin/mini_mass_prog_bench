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
/*! \file   mooltipass.c
 *  \brief  main file
 *  Copyright [2014] [Mathieu Stephan]
 */
#include <util/atomic.h>
#include <avr/eeprom.h>
#include <avr/boot.h>
#include <stdlib.h>
#include <avr/io.h>
#include <string.h>
#include <stdio.h>
#include "smart_card_higher_level_functions.h"
#include "gui_smartcard_functions.h"
#include "gui_screen_functions.h"
#include "gui_basic_functions.h"
#include "logic_aes_and_comms.h"
#include "functional_testing.h"
#include "eeprom_addresses.h"
#include "define_printouts.h"
#include "watchdog_driver.h"
#include "logic_smartcard.h"
#include "usb_cmd_parser.h"
#include "timer_manager.h"
#include "bitstreammini.h"
#include "oled_wrapper.h"
#include "logic_eeprom.h"
#include "hid_defines.h"
#include "mini_inputs.h"
#include "mooltipass.h"
#include "interrupts.h"
#include "smartcard.h"
#include "mini_leds.h"
#include "flash_mem.h"
#include "defines.h"
#include "delays.h"
#include "utils.h"
#include "anim.h"
#include "spi.h"
#include "pwm.h"
#include "usb.h"
#include "rng.h"
#include "i2c.h"

/* Boolean to know state of lock/unlock feature */
uint8_t mp_lock_unlock_shortcuts = FALSE;
/* Boolean to know if user timeout is enabled */
uint8_t mp_timeout_enabled = FALSE;
/* Flag set by anything to signal activity */
uint8_t act_detected_flag = FALSE;

/* Defines for IOs connected on the PCA9554 */
#define PCA_PSU_EN_MASK         0x40
#define PCA_COLOR_MASK          0x07
/* Arrays for interrupt signals */
volatile uint8_t* int_ddr_array[] = {&DDRF, &DDRC, &DDRC, &DDRB, &DDRB, &DDRB, &DDRB, &DDRB};
volatile uint8_t* int_port_array[] = {&PORTF, &PORTC, &PORTC, &PORTB, &PORTB, &PORTB, &PORTB, &PORTB};
volatile uint8_t* int_pin_array[] = {&PINF, &PINC, &PINC, &PINB, &PINB, &PINB, &PINB, &PINB};
uint8_t int_pin_id_array[] = {1 << 7, 1 << 7, 1 << 6, 1 << 2, 1 << 3, 1 << 7, 1 << 6, 1 << 5};
/* Current states on all programming rigs */
uint8_t programming_states[9];
/* RED led state for blinking */
uint8_t red_led_blinking_state[9];
/* 5v powered states for programming sockets */
uint8_t prog_socket_powered_state[9];
/* Colors currently displayed */
uint8_t prog_socket_displayed_colors[9];
/* Button pressed buffer to return to the computer */
uint8_t button_pressed_states_return[9];
/* enum for colors */
enum color_t    {RED = 0x01, ORANGE = 0x02, GREEN = 0x04, BLACK = 0x00};
/* enum for programming states */
enum prog_state {PROG_IDLE = 0, PROG_ERROR_SHORTED, PROG_PROGRAMMING, PROG_ERROR};

void get_and_clear_button_pressed_return(uint8_t* buffer)
{
    memcpy(buffer, button_pressed_states_return, sizeof(button_pressed_states_return));
    memset(button_pressed_states_return, 0x00, sizeof(button_pressed_states_return));
}

void set_prog_rig_led_color(uint8_t id, uint8_t color)
{
    prog_socket_displayed_colors[id] = (~color) & PCA_COLOR_MASK;
    writeDataToI2C(0x70 | (id << 1), 1, prog_socket_displayed_colors[id] | prog_socket_powered_state[id]);  
}

void disable_prog_rig_5v(uint8_t id)
{
    prog_socket_powered_state[id] = PCA_PSU_EN_MASK;
    writeDataToI2C(0x70 | (id << 1), 1, prog_socket_displayed_colors[id] | prog_socket_powered_state[id]);    
}

void enable_prog_rig_5v(uint8_t id)
{
    prog_socket_powered_state[id] = 0;
    writeDataToI2C(0x70 | (id << 1), 1, prog_socket_displayed_colors[id] | prog_socket_powered_state[id]); 
}

uint8_t read_prog_rig_inputs(uint8_t id)
{
    uint8_t current_inputs;
    readDataFromI2C(0x70 | (id << 1), 0, &current_inputs);
    return current_inputs;    
}

void platform_io_init(void)
{
    memset(button_pressed_states_return, 0x00, sizeof(button_pressed_states_return));
    memset(prog_socket_powered_state, PCA_PSU_EN_MASK, sizeof(prog_socket_powered_state));
    memset(prog_socket_displayed_colors, PCA_COLOR_MASK, sizeof(prog_socket_displayed_colors));
    
    /* prog rig directly connected to the MCU */
    DDRE &= ~0x40;
    PORTE |= 0x40;
    DDRF &= ~0x40;
    PORTF |= 0x40;
    DDRF |= 0x33;
    PORTF |= 0x33;
    
    /* int signals: input with pull ups */
    for (uint8_t i = 0; i < 8; i++)
    {
        *int_ddr_array[i] &= ~(int_pin_id_array[i]);
        *int_port_array[i] |= (int_pin_id_array[i]);
    }
    
    /* init gpio extenders */
    for (uint8_t id = 0; id < 8; id++)
    {
        /* pin mapping: 0 LED1, 1 LED2, 2 LED3, 3 switch, 6 psu_en, 7 psu_fault */
        writeDataToI2C(0x70 | (id << 1), 3, 0x88);
        /* reset outputs in case it is a quick reboot */
        writeDataToI2C(0x70 | (id << 1), 1, 0x07);
        /* read to clear interrupts */
        read_prog_rig_inputs(id);
        /* disable 5v supplied to this prog rig */
        disable_prog_rig_5v(id);
        /* set led to green to signal ready */        
        set_prog_rig_led_color(id, GREEN);    
    }   
    PORTF &= ~0x01; 
}

void programming_success(uint8_t socket_id)
{
    if (socket_id != 8)
    {
        disable_prog_rig_5v(socket_id);
        set_prog_rig_led_color(socket_id, GREEN);
    } 
    else
    {
        PORTF |= 0x13;
        PORTF &= ~0x01;
    }
    programming_states[socket_id] = PROG_IDLE;    
}

void programming_failure(uint8_t socket_id)
{
    if (socket_id != 8)
    {
        disable_prog_rig_5v(socket_id);
        set_prog_rig_led_color(socket_id, RED);
    }
    else
    {
        PORTF |= 0x13;
        PORTF &= ~0x10;
    }
    red_led_blinking_state[socket_id] = 0xFF;
    programming_states[socket_id] = PROG_ERROR;    
}

int main(void)
{
    RET_TYPE flash_init_result; 
    DDRB |= 0x01;
    
    initIRQ();                                  // Initialize interrupts
    powerSettlingDelay();                       // Let the power settle before enabling USB controller
    initUsb();                                  // Initialize USB controller
    powerSettlingDelay();                       // Let the USB 3.3V LDO rise
    initI2cPort();                              // Initialize I2C interface
    rngInit();                                  // Initialize avrentropy library
    oledInitIOs();                              // Initialize OLED inputs/outputs
    initFlashIOs();                             // Initialize Flash inputs/outputs
    spiUsartBegin();                            // Start USART SPI at 8MHz (standard) or 4MHz (mini)
    platform_io_init();                         // Init platform IOs
    while(!isUsbConfigured());                  // Wait for host to set configuration
    
    /** FLASH INITIALIZATION **/
    flash_init_result = checkFlashID();         // Check for flash presence
    while(flash_init_result != RETURN_OK);
    
    /** OLED INITIALIZATION **/
    oledBegin(FONT_DEFAULT);                    // Only do it now as we're enumerated

    /* Go to startup screen */
    miniOledBitmapDrawFlash(0, 0, BITMAP_MOOLTIPASS, OLED_SCROLL_UP);
    miniOledFlushWrittenTextToDisplay();

    /* Error timer */
    activateTimer(TIMER_CAPS, 500);
    while (1)
    {
        /* Process possible incoming USB packets */
        usbProcessIncoming(USB_CALLER_MAIN);
        
        /* Error blinking */
        if (hasTimerExpired(TIMER_CAPS, TRUE) == TIMER_EXPIRED)
        {
            activateTimer(TIMER_CAPS, 500);
            for (uint8_t i = 0; i < 9; i++)
            {
                if ((programming_states[i] == PROG_ERROR_SHORTED) || (programming_states[i] == PROG_ERROR))
                {
                    if (i == 8)
                    {
                        if (red_led_blinking_state[i])
                        {
                            PORTF |= 0x13;
                            red_led_blinking_state[i] = 0;
                        }
                        else
                        {
                            PORTF |= 0x13;
                            PORTF &= ~0x10;
                            red_led_blinking_state[i] = 0xFF;
                        }
                    } 
                    else
                    {
                        if (red_led_blinking_state[i])
                        {
                            red_led_blinking_state[i] = 0;
                            set_prog_rig_led_color(i, BLACK);
                        } 
                        else
                        {
                            red_led_blinking_state[i] = 0xFF;
                            set_prog_rig_led_color(i, RED);
                        }
                    }
                }
            }
        }
        
        /* Scan timers */
        for (uint8_t i = 0; i < 9; i++)
        {            
            if (hasTimerExpired(TIMER_PROG_RIG_0+i, TRUE) == TIMER_EXPIRED)
            {
                if (programming_states[i] == PROG_PROGRAMMING)
                {
                    /* If we've started to enter programming mode, inform the computer that he can program the MCU */
                    button_pressed_states_return[i] = TRUE;
                }
            }
        }
        
        /* Scan interrupts */
        for (uint8_t i = 0; i < 8; i++)
        {
            /* Do we have an interrupt ? */
            if ((*int_pin_array[i] & int_pin_id_array[i]) == 0)
            {
                //PORTB = (PORTB & 0xFE) | (~PORTB & 0x01);                
                /* Read the GPIO extender value */
                uint8_t io_val = read_prog_rig_inputs(i);
                
                if ((programming_states[i] == PROG_IDLE) || (programming_states[i] == PROG_ERROR) || (programming_states[i] == PROG_ERROR_SHORTED))
                {
                    /* button pressed? */
                    if ((io_val & 0x08) == 0)
                    {
                        /* enter programming mode */
                        enable_prog_rig_5v(i);
                        set_prog_rig_led_color(i, ORANGE);
                        programming_states[i] = PROG_PROGRAMMING;
                        
                        /* arm timer to signal success */
                        activateTimer(TIMER_PROG_RIG_0+i, 200);
                    }
                } 
                else if (programming_states[i] == PROG_PROGRAMMING)
                {
                    /* 5v shorted? */
                    if ((io_val & 0x80) == 0)
                    {
                        /* disable 5v, signal error */
                        disable_prog_rig_5v(i);
                        set_prog_rig_led_color(i, RED);
                        red_led_blinking_state[i] = 0xFF;
                        programming_states[i] = PROG_ERROR_SHORTED;
                        
                        /* clear timer */
                        activateTimer(TIMER_PROG_RIG_0+i, 0);
                        hasTimerExpired(TIMER_PROG_RIG_0+i, TRUE);
                    }
                }                               
            }
        }
        
        /* prog rig #9 is directly wired to MCU */        
        if ((programming_states[8] == PROG_IDLE) || (programming_states[8] == PROG_ERROR) || (programming_states[8] == PROG_ERROR_SHORTED))
        {
            if ((PINE & 0x40) == 0)
            {
                /* enter programming mode */
                PORTF &= ~0x20;
                PORTF |= 0x13;
                PORTF &= ~0x02;
                programming_states[8] = PROG_PROGRAMMING;
                
                /* arm timer to signal success */
                activateTimer(TIMER_PROG_RIG_0+8, 200);
            }
        }
        else if (programming_states[8] == PROG_PROGRAMMING)
        {
            if ((PINF & 0x40) == 0)
            {
                /* disable 5v, signal error */
                PORTF |= 0x20;
                PORTF |= 0x13;
                PORTF &= ~0x10;
                red_led_blinking_state[8] = 0xFF;
                programming_states[8] = PROG_ERROR_SHORTED;
                
                /* clear timer */
                activateTimer(TIMER_PROG_RIG_0+8, 0);
                hasTimerExpired(TIMER_PROG_RIG_0+8, TRUE);
            }
        }        
    }
}
