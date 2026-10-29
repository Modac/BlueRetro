/*
 * Copyright (c) 2026, BlueRetro contributors
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _PORT_LED_H_
#define _PORT_LED_H_

#include <stdint.h>
#include "adapter/adapter.h"

#define PORT_LED_CFG_VER 1
#define PORT_LED_MAX 4

/* Port states, also index in port_led_cfg.color[] */
enum {
    PORT_LED_WIRED = 0,
    PORT_LED_BT,
    PORT_LED_PAIRING,
    PORT_LED_BTN,
    PORT_LED_COLOR_MAX,
    PORT_LED_OFF = 0xFF,
};

struct port_led_color {
    uint8_t r;
    uint8_t g;
    uint8_t b;
    uint8_t brightness; /* 0 - 100 % */
} __packed;

struct port_led_cfg {
    uint8_t version;
    uint8_t enable;
    uint8_t gpio;
    uint8_t chain_pos[PORT_LED_MAX]; /* Port index -> position in LED chain */
    struct port_led_color color[PORT_LED_COLOR_MAX];
} __packed;

void port_led_init(void);
void port_led_set_port(uint32_t port, uint8_t state);
void port_led_set_btn(uint32_t active, uint32_t flash_hz);
void port_led_identify(void);
void port_led_get_cfg(struct port_led_cfg *cfg);
int32_t port_led_set_cfg(const struct port_led_cfg *cfg);
int32_t port_led_save_cfg(void);

#endif /* _PORT_LED_H_ */
