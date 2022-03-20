/**
 * Copyright (c) 2014 - 2021, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
 *
 * @defgroup simpleye_main main.c
 * @{
 * @ingroup simpleye
 * @brief simpleye main file.
 *
 * This file contains the source code for simpleye.
 *
 */

// Standard libraries
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

// Nordic "nrf"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_uart.h"
#include "nrf_uarte.h"

// Nordic "app"
#include "app_uart.h"
#include "app_error.h"

// This board
#include "board.h"

// UART defines
#define UART_TX_BUF_SIZE 256      /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 256      /**< UART RX buffer size. */

static void show_error(void);
void uart_error_handle(app_uart_evt_t *p_event);
static void simpleye_init();

void uart_error_handle(app_uart_evt_t *p_event)
{
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        // APP_ERROR_HANDLER(p_event->data.error_communication);
        show_error();
    }
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        // APP_ERROR_HANDLER(p_event->data.error_code);
        show_error();
    }
}
/** @brief Function for setting the @ref ERROR_PIN high, and then enter an infinite loop.
 */
static void show_error(void)
{
    while (true)
    {
        nrf_gpio_pin_toggle(LED_R);
        nrf_delay_ms(200);
    }
}

static void simpleye_init()
{
    // Ensure REGOUT0 is set to 3.0V to make sure we can interface with SWD
    // if UICR (???) is ever erased
    // See https://devzone.nordicsemi.com/guides/short-range-guides/b/getting-started/posts/nrf52840-dongle-programming-tutorial

    if ((NRF_UICR->REGOUT0 & UICR_REGOUT0_VOUT_Msk) == (UICR_REGOUT0_VOUT_DEFAULT << UICR_REGOUT0_VOUT_Pos))
    {
        NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen;
        while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
        {
        }

        NRF_UICR->REGOUT0 = (NRF_UICR->REGOUT0 & ~((uint32_t)UICR_REGOUT0_VOUT_Msk)) |
                            (UICR_REGOUT0_VOUT_3V0 << UICR_REGOUT0_VOUT_Pos);

        NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren;
        while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
        {
        }

        // System reset is needed to update UICR registers.
        NVIC_SystemReset();
    }

    // Configure LED GPIOs
    nrf_gpio_cfg_output(LED_R);
    nrf_gpio_cfg_output(LED_G);
    nrf_gpio_cfg_output(LED_B);

    nrf_gpio_pin_set(LED_R);
    nrf_gpio_pin_set(LED_G);
    nrf_gpio_pin_set(LED_B);

    // Configure UART for debug output
    uint32_t err_code;

    const app_uart_comm_params_t comm_params =
        {
            RX_PIN_NUMBER,
            TX_PIN_NUMBER,
            RTS_PIN_NUMBER,
            CTS_PIN_NUMBER,
            UART_HWFC,
            false,
            NRF_UARTE_BAUDRATE_115200};

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_error_handle,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);

    APP_ERROR_CHECK(err_code);
}

/**
 * @brief Function for application main entry.
 */
int main(void)
{
    simpleye_init();

    printf("\r\nUART example started.\r\n");

    while (true)
    {
        uint8_t cr;
        while (app_uart_get(&cr) != NRF_SUCCESS) {

        }
        while (app_uart_put(cr) != NRF_SUCCESS) {

        }

        // if (cr == 'q' || cr == 'Q')
        // {
        //     printf(" \r\nExit!\r\n");

        //     while (true)
        //     {
        //         // Do nothing.
        //     }
        // }
    }

}

/**
 *@}
 **/
