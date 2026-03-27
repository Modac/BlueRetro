/*
 * Copyright (c) 2024, BlueRetro contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * WS2812 port-indicator LED driver.
 *
 * Drives a chain of 4 WS2812 LEDs (one per controller port) via the
 * espressif/led_strip component using its SPI backend.  The SPI backend
 * avoids any conflict with the RMT peripheral used by the N64/GC wired
 * driver (nsi.c).
 *
 * LED colours
 *   OFF           – no Bluetooth device assigned to that port
 *   Green (dim)   – Bluetooth controller connected and ready
 *   Blue blink    – Bluetooth inquiry active, waiting for a controller
 *                   (blink cadence driven by ws2812_update() call rate ~320 ms)
 *
 * SPI backend details (espressif/led_strip ≥ 3.0.0)
 *   Each WS2812 bit is encoded as 3 SPI bits: 100 = '0', 110 = '1'.
 *   Required SPI clock: 2.5 MHz  →  3 × 2.5 MHz = 7.5 Mbps effective.
 *   The component selects the clock automatically.
 */

#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"
#include "led_strip.h"
#include "ws2812.h"

#define WS2812_NUM_LEDS 4

/* Pixel colours (passed as RGB to led_strip_set_pixel; the component handles
 * the WS2812 GRB wire encoding automatically).                              */
#define CLR_CONNECTED_R   0
#define CLR_CONNECTED_G  32
#define CLR_CONNECTED_B   0

#define CLR_SEARCHING_R   0
#define CLR_SEARCHING_G   0
#define CLR_SEARCHING_B  32

static led_strip_handle_t strip;
static uint8_t port_state[WS2812_NUM_LEDS];
static uint8_t blink_toggle;
static uint8_t needs_refresh;

void ws2812_init(uint32_t gpio_pin)
{
    led_strip_config_t strip_config = {
        .strip_gpio_num = (int)gpio_pin,
        .max_leds       = WS2812_NUM_LEDS,
        .led_model      = LED_MODEL_WS2812,
    };

    led_strip_spi_config_t spi_config = {
        .clk_src = SPI_CLK_SRC_DEFAULT,
        .spi_bus = CONFIG_BLUERETRO_WS2812_SPI_HOST,
        .flags   = { .with_dma = true },
    };

    esp_err_t ret = led_strip_new_spi_device(&strip_config, &spi_config, &strip);
    if (ret != ESP_OK) {
        printf("# %s: led_strip init failed (%d)\n", __FUNCTION__, ret);
        strip = NULL;
        return;
    }

    memset(port_state, WS2812_STATE_OFF, sizeof(port_state));
    blink_toggle  = 0;
    needs_refresh = 0;

    led_strip_clear(strip);
}

void ws2812_set_port(uint32_t port, uint8_t state)
{
    if (port >= WS2812_NUM_LEDS || port_state[port] == state) {
        return;
    }
    port_state[port] = state;
    needs_refresh = 1;
}

void ws2812_update(void)
{
    if (!strip) {
        return;
    }

    blink_toggle ^= 1;

    for (uint32_t i = 0; i < WS2812_NUM_LEDS; i++) {
        switch (port_state[i]) {
            case WS2812_STATE_CONNECTED:
                led_strip_set_pixel(strip, i,
                    CLR_CONNECTED_R, CLR_CONNECTED_G, CLR_CONNECTED_B);
                break;

            case WS2812_STATE_SEARCHING:
                /* Blink: alternate blue and off on each update tick. */
                if (blink_toggle) {
                    led_strip_set_pixel(strip, i,
                        CLR_SEARCHING_R, CLR_SEARCHING_G, CLR_SEARCHING_B);
                } else {
                    led_strip_set_pixel(strip, i, 0, 0, 0);
                }
                needs_refresh = 1;
                break;

            default: /* WS2812_STATE_OFF */
                led_strip_set_pixel(strip, i, 0, 0, 0);
                break;
        }
    }

    if (needs_refresh) {
        led_strip_refresh(strip);
        needs_refresh = 0;
    }
}
