/*
 * Copyright (c) 2024, BlueRetro contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * WS2812 LED strip driver – bit-banging via direct GPIO register access.
 *
 * A chain of 4 WS2812 LEDs is driven from a single configurable GPIO.
 * Each LED maps to one GameCube/N64 controller port:
 *   LED 0 → port 1, LED 1 → port 2, …
 *
 * Colours
 *   OFF       – all off
 *   CONNECTED – solid green (dim)
 *   SEARCHING – blue, blinks at ~1.5 Hz (driven by ws2812_update() cadence)
 *
 * Timing (WS2812B spec, symmetric 350 ns / 900 ns):
 *   T0H = 350 ns,  T0L = 900 ns
 *   T1H = 900 ns,  T1L = 350 ns
 * All values are well within the ±150 ns tolerance window.
 *
 * The bit-banging loop is placed in IRAM and runs inside a FreeRTOS
 * critical section (~120 µs, occurring only on connect / disconnect events).
 */

#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "sdkconfig.h"
#include "driver/gpio.h"
#include "soc/gpio_struct.h"
#include "system/delay.h"
#include "ws2812.h"

/* ── LED count ──────────────────────────────────────────────────────────── */
#define WS2812_NUM_LEDS 4

/* ── Timing constants (CPU cycles) ─────────────────────────────────────── */
/* Uses CONFIG_ESP32_DEFAULT_CPU_FREQ_MHZ defined by sdkconfig (240 MHz).   */
#define WS2812_T0H_CYC  (CONFIG_ESP32_DEFAULT_CPU_FREQ_MHZ * 350 / 1000)
#define WS2812_T0L_CYC  (CONFIG_ESP32_DEFAULT_CPU_FREQ_MHZ * 900 / 1000)
#define WS2812_T1H_CYC  (WS2812_T0L_CYC)
#define WS2812_T1L_CYC  (WS2812_T0H_CYC)

/* ── Pixel colours (GRB byte order) ────────────────────────────────────── */
#define CLR_CONNECTED_G  32
#define CLR_CONNECTED_R   0
#define CLR_CONNECTED_B   0

#define CLR_SEARCHING_G   0
#define CLR_SEARCHING_R   0
#define CLR_SEARCHING_B  32

/* ── Internal types ─────────────────────────────────────────────────────── */
typedef struct {
    uint8_t g, r, b;   /* WS2812 expects GRB order on the wire */
} ws2812_pixel_t;

/* ── Module state ───────────────────────────────────────────────────────── */
static uint8_t          ws2812_gpio_pin;
static uint8_t          port_state[WS2812_NUM_LEDS];
static ws2812_pixel_t   pixels[WS2812_NUM_LEDS];
static ws2812_pixel_t   prev_pixels[WS2812_NUM_LEDS];
static uint8_t          blink_toggle;

static portMUX_TYPE ws2812_mux = portMUX_INITIALIZER_UNLOCKED;

/* ── Helpers ────────────────────────────────────────────────────────────── */

/* Busy-wait for exactly `cycles` CPU cycles (inlined into IRAM function). */
static inline IRAM_ATTR void ws2812_wait(uint32_t cycles)
{
    uint32_t start = xthal_get_ccount();
    while ((uint32_t)(xthal_get_ccount() - start) < cycles) {}
}

/* Send one byte (MSB first) to the strip. Runs from IRAM inside critical
 * section – no flash accesses allowed here.                                */
static inline IRAM_ATTR void ws2812_send_byte(uint8_t byte, uint32_t gpio_bit)
{
    for (int bit = 7; bit >= 0; bit--) {
        if ((byte >> bit) & 1) {
            GPIO.out_w1ts = gpio_bit;
            ws2812_wait(WS2812_T1H_CYC);
            GPIO.out_w1tc = gpio_bit;
            ws2812_wait(WS2812_T1L_CYC);
        } else {
            GPIO.out_w1ts = gpio_bit;
            ws2812_wait(WS2812_T0H_CYC);
            GPIO.out_w1tc = gpio_bit;
            ws2812_wait(WS2812_T0L_CYC);
        }
    }
}

/* Transmit the complete pixel buffer to the strip. */
static void IRAM_ATTR ws2812_write_strip(void)
{
    if (ws2812_gpio_pin >= 32) {
        /* Pins 32-39 need GPIO.out1_w1ts / out1_w1tc – not used by default. */
        return;
    }

    uint32_t gpio_bit = 1UL << ws2812_gpio_pin;

    /* Disable FreeRTOS scheduler + interrupts up to SYSCALL level (~120 µs). */
    portENTER_CRITICAL(&ws2812_mux);

    for (int i = 0; i < WS2812_NUM_LEDS; i++) {
        ws2812_send_byte(pixels[i].g, gpio_bit);
        ws2812_send_byte(pixels[i].r, gpio_bit);
        ws2812_send_byte(pixels[i].b, gpio_bit);
    }

    portEXIT_CRITICAL(&ws2812_mux);

    /* Reset / latch pulse: keep DIN low for ≥50 µs. */
    delay_us(60);
}

/* ── Public API ─────────────────────────────────────────────────────────── */

void ws2812_init(uint32_t gpio_pin)
{
    ws2812_gpio_pin = (uint8_t)gpio_pin;

    gpio_config_t io_conf = {
        .intr_type    = GPIO_INTR_DISABLE,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pin_bit_mask = 1ULL << gpio_pin,
    };
    gpio_config(&io_conf);
    gpio_set_level(gpio_pin, 0);

    memset(port_state,   0, sizeof(port_state));
    memset(pixels,       0, sizeof(pixels));
    memset(prev_pixels, 0, sizeof(prev_pixels));
    blink_toggle = 0;

    /* Push all-off to clear any previous LED state. */
    ws2812_write_strip();
}

void ws2812_set_port(uint32_t port, uint8_t state)
{
    if (port >= WS2812_NUM_LEDS) {
        return;
    }
    port_state[port] = state;
}

void ws2812_update(void)
{
    blink_toggle ^= 1;

    for (int i = 0; i < WS2812_NUM_LEDS; i++) {
        switch (port_state[i]) {
            case WS2812_STATE_CONNECTED:
                pixels[i] = (ws2812_pixel_t){CLR_CONNECTED_G,
                                             CLR_CONNECTED_R,
                                             CLR_CONNECTED_B};
                break;

            case WS2812_STATE_SEARCHING:
                if (blink_toggle) {
                    pixels[i] = (ws2812_pixel_t){CLR_SEARCHING_G,
                                                 CLR_SEARCHING_R,
                                                 CLR_SEARCHING_B};
                } else {
                    pixels[i] = (ws2812_pixel_t){0, 0, 0};
                }
                break;

            default: /* WS2812_STATE_OFF */
                pixels[i] = (ws2812_pixel_t){0, 0, 0};
                break;
        }
    }

    /* Only write to the strip when the pixel data actually changed. */
    if (memcmp(pixels, prev_pixels, sizeof(pixels)) != 0) {
        ws2812_write_strip();
        memcpy(prev_pixels, pixels, sizeof(pixels));
    }
}
