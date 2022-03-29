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
#include "nrf_ppi.h"

// Nordic "nrfx"
#include "nrfx_gpiote.h"
#include "nrfx_ppi.h"
#include "nrfx_timer.h"

// Nordic "nrf_drv"
#include "nrf_drv_twi.h"
#include "nrf_drv_clock.h"
#include "nrf_drv_ppi.h"

// Nordic "app" libraries
#include "app_uart.h"
#include "app_error.h"

// This board
#include "board.h"

// Camera
#include "OV7675.h"

// UART defines
#define UART_TX_BUF_SIZE 256      /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 256      /**< UART RX buffer size. */

// TWI defines
#define TWI_INSTANCE_ID 0
#define PWM_INSTANCE_ID 0
#define MAX_PENDING_TRANSACTIONS    5

static const nrf_drv_twi_t simpleye_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

// Timer instance used to generate camera's XCLK
static const nrfx_timer_t xclk_timer = NRFX_TIMER_INSTANCE(0);

// PPI channel used to attach timer and GPIOTE together
static nrf_ppi_channel_t xclk_ppi_channel;

uint8_t prod_reg0_data = 32;
uint8_t prod_reg1_data = 42;

static void show_error(void);
void uart_error_handle(app_uart_evt_t *p_event);
static void simpleye_init(void);
static void twi_init(void);
static void camera_clk_init(void);

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

static void uart_init(void) {

    
}

static void twi_init(void) {
    uint32_t err_code;

    const nrf_drv_twi_config_t  simpleye_twi_config = {
        .scl = CAMERA_SCL_PIN,
        .sda = CAMERA_SDA_PIN,
        .frequency = NRF_DRV_TWI_FREQ_100K,
        .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
        .clear_bus_init = false
    };

    err_code = nrf_drv_twi_init(&simpleye_twi, &simpleye_twi_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&simpleye_twi);
}

static void camera_clk_init(void) {
    uint32_t err_code;

    // Configure a toggling GPIOTE out task
    static const nrfx_gpiote_out_config_t xclk_gpiote_config = NRFX_GPIOTE_CONFIG_OUT_TASK_TOGGLE(1);

    // Initialize the GPIOTE if it is not already
    if (!nrfx_gpiote_is_init()) {
        err_code = nrfx_gpiote_init();
        APP_ERROR_CHECK(err_code);
    }

    // Configure the XCLK camera pin as GPIOTE OUT
    err_code = nrfx_gpiote_out_init(CAMERA_XCLK_PIN,&xclk_gpiote_config);

    // Enable the GPIOTE output pin task (is this needed?)
    nrfx_gpiote_out_task_enable(CAMERA_XCLK_PIN);

    // Configure the timer
    nrfx_timer_config_t xclk_timer_config = NRFX_TIMER_DEFAULT_CONFIG;
    xclk_timer_config.frequency = NRF_TIMER_FREQ_16MHz;

    // Initialize the timer without an event handler
    err_code = nrfx_timer_init(&xclk_timer,&xclk_timer_config,NULL);

    // Initialize PPI ignoring error if already initialized
    nrf_drv_ppi_init();

    // Allocate a free PPI channel
    nrfx_ppi_channel_alloc(&xclk_ppi_channel);

    // Assign the event and task to the PPI
    nrfx_ppi_channel_assign(xclk_ppi_channel,
                            nrfx_timer_event_address_get(&xclk_timer,NRF_TIMER_EVENT_COMPARE0),
                            nrfx_gpiote_out_task_addr_get(CAMERA_XCLK_PIN));
    
    nrfx_timer_enable(&xclk_timer);

}

static void simpleye_init(void)
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
            NRF_UARTE_BAUDRATE_115200
        };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_error_handle,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);

    APP_ERROR_CHECK(err_code);

    // Initialize camera pins
    nrf_gpio_cfg_output(CAMERA_PWDN_PIN);
    // nrf_gpio_cfg_output(CAMERA_XCLK_PIN);
    nrf_gpio_cfg_input(CAMERA_PCLK_PIN, NRF_GPIO_PIN_NOPULL);
    nrf_gpio_pin_clear(CAMERA_PWDN_PIN);

    // Enable XCLK
    camera_clk_init();

    // Initialize I2C peripheral
    twi_init();

    const uint8_t init_read_addresses[] = { OV7675_REG_PROD_ID_1, OV7675_REG_PROD_ID_2 };
    uint8_t sample_data = 0;

    printf("Starting I2C address search...\n\r");
    for (uint8_t address = 0x41; address < 0x45; address++) {
        err_code = nrf_drv_twi_rx(&simpleye_twi, address, &sample_data, sizeof(sample_data));
        printf("Address = 0x%x, err_code = 0x%x\r\n",address,err_code);
        nrf_delay_ms(100);
        if (err_code == NRF_SUCCESS) {
            printf("I2C found at address 0x%x\n\r",address);
        }
    }

    // err_code = nrf_twi_mngr_perform(&simpleye_twi_mngr, &simpleye_twi_config, camera_read, sizeof(camera_read) / sizeof(camera_read[0]), twi_mngr_handler );
    // printf("\r\n\r\n");
    // printf("OV7675 id reg 0 = %x\r\n",prod_reg0_data);
    // printf("OV7675 id reg 1 = %x\r\n",prod_reg1_data);

    // err_code = NRF_TWI_MNGR_TRANSFER()
    // Check camera product IDs
    // nrf_twi_mngr_perform
}

// void twi_mngr_handler(void) {
//     printf("handler called\r\n");
//     printf("OV7675 id reg 0 = %x\r\n",prod_reg0_data);
//     printf("OV7675 id reg 1 = %x\r\n",prod_reg1_data);
// }

/**
 * @brief Function for application main entry.
 */
int main(void)
{
    simpleye_init();

    printf("\r\nIn main().\r\n");

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
