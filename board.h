/**
 * Copyright (c) 2016 - 2021, Nordic Semiconductor ASA
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
#ifndef BOARD_H
#define BOARD_H

#ifdef __cplusplus
extern "C" {
#endif

#define LED_R          NRF_GPIO_PIN_MAP(0,24)
#define LED_G          NRF_GPIO_PIN_MAP(0,16)
#define LED_B          NRF_GPIO_PIN_MAP(0,6)
#define LED_GS         NRF_GPIO_PIN_MAP(1,9)

// UART pin definitions
#define RX_PIN_NUMBER 28    // Using shield "A6"
#define TX_PIN_NUMBER 3     // Using shield "A7"
#define RTS_PIN_NUMBER UART_PIN_DISCONNECTED
#define CTS_PIN_NUMBER UART_PIN_DISCONNECTED
// Disable HW flow control
#define UART_HWFC APP_UART_FLOW_CONTROL_DISABLED

// OV7675 breakout-board to nRF52840 pinmap
#define CAMERA_SCL_PIN    NRF_GPIO_PIN_MAP(0,2)
#define CAMERA_SDA_PIN    NRF_GPIO_PIN_MAP(0,31)

#define CAMERA_VS_PIN     NRF_GPIO_PIN_MAP(0,21)
#define CAMERA_HS_PIN     NRF_GPIO_PIN_MAP(0,5)

#define CAMERA_PCLK_PIN   NRF_GPIO_PIN_MAP(0,4)
#define CAMERA_XCLK_PIN   NRF_GPIO_PIN_MAP(0,27)

// D0 & D1 might be swapped on the schematic & pcb
#define CAMERA_D0_PIN     NRF_GPIO_PIN_MAP(1,2)
#define CAMERA_D1_PIN     NRF_GPIO_PIN_MAP(1,3)
#define CAMERA_D2_PIN     NRF_GPIO_PIN_MAP(1,10)
#define CAMERA_D3_PIN     NRF_GPIO_PIN_MAP(1,11)
#define CAMERA_D4_PIN     NRF_GPIO_PIN_MAP(1,12)
#define CAMERA_D5_PIN     NRF_GPIO_PIN_MAP(1,13)
#define CAMERA_D6_PIN     NRF_GPIO_PIN_MAP(0,14)
#define CAMERA_D7_PIN     NRF_GPIO_PIN_MAP(0,15)

#define CAMERA_PWDN_PIN   NRF_GPIO_PIN_MAP(1,1)
#define CAMERA_PEN_PIN    NRF_GPIO_PIN_MAP(1,8)

// TODO Uncomment this line to use I2C1, otherwise it can be used
// as SWO
// #define I2C1_PULLUP     NRF_GPIO_PIN_MAP(1,0)

// #define LEDS_ACTIVE_STATE 0

// #define LEDS_LIST { LED_R, LED_G, LED_B, LED_GS }

// // TODO what is function of BSP_LED_0?
// #define BSP_LED_0      LED_GS

// // This seems to be a bitmask for the BSP_LEDs, unclear what they do
// #define LEDS_INV_MASK  0

// // The only button on the nano 33 BLE is a hardware reset button
// #define BUTTONS_NUMBER 0

// // There is a button on the shield, but it's connected to a "SCK"
// // GPIO that is also an orange LED?
// // TODO comment if conflicting with LED function
// #define BUTTON_SCK    NRF_GPIO_PIN_MAP(0,13)
// #define BUTTONS_ACTIVE_STATE 0

// #define BUTTONS_LIST { BUTTON_SCK }

// // TODO what is function of BSP_BUTTON_0?
// #define BSP_BUTTON_0   BUTTON_SCK

// // There is no "schematic defined" UART (no pins assigned a UART function on
// // the scmatic). However, the shield exposes "analog" pins A6 and A7 which 
// // are not used by any on-board peripheral and can be connected to the UART 
// // peripheral for debugging!

// // TODO these two pins might not want to be "gpio pin maps"
// #define RX_PIN_NUMBER  NRF_GPIO_PIN_MAP(0,28) // pin GPIO17_A net AIN4_A6
// #define TX_PIN_NUMBER  NRF_GPIO_PIN_MAP(0,03) // pin GPIO16_A net AIN1_A7
// #define CTS_PIN_NUMBER UART_PIN_DISCONNECTED
// #define RTS_PIN_NUMBER UART_PIN_DISCONNECTED
// #define HWFC           false


#ifdef __cplusplus
}
#endif

#endif // ARDUINO_PRIMO_H
