/*
 * Copyright (c) 2024, BlueRetro contributors
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _WS2812_H_
#define _WS2812_H_

#include <stdint.h>

/* Port LED states */
enum {
    WS2812_STATE_OFF = 0,
    WS2812_STATE_CONNECTED,
    WS2812_STATE_SEARCHING,
};

/**
 * @brief Initialize the WS2812 LED strip driver.
 *
 * Configures the given GPIO pin as output and clears all 4 LEDs.
 *
 * @param gpio_pin  GPIO number connected to the WS2812 DIN line.
 */
void ws2812_init(uint32_t gpio_pin);

/**
 * @brief Set the desired state for one controller-port LED.
 *
 * The change is only written to the strip on the next ws2812_update() call.
 *
 * @param port   Port index (0–3).
 * @param state  WS2812_STATE_OFF / _CONNECTED / _SEARCHING.
 */
void ws2812_set_port(uint32_t port, uint8_t state);

/**
 * @brief Recompute pixel colours and transmit to the LED strip if changed.
 *
 * Should be called at regular intervals (e.g. every ~320 ms) so that the
 * SEARCHING blink animation is driven.
 */
void ws2812_update(void);

#endif /* _WS2812_H_ */
