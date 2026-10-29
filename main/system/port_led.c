/*
 * Copyright (c) 2026, BlueRetro contributors
 * SPDX-License-Identifier: Apache-2.0
 */

/* WS2812 port status LEDs, one per controller port, chained on a single GPIO.
 * RMT is fully used by the N64/GC wired driver so the LED chain is driven
 * by SPI3 MOSI instead: each WS2812 bit is sent as 3 SPI bits at 2.5 MHz
 * (0 = 100, 1 = 110), giving 400/800 ns pulses.
 */

#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <esp_heap_caps.h>
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "adapter/adapter.h"
#include "system/fs.h"
#include "tools/util.h"
#include "port_led.h"

#define PORT_LED_SPI_HOST SPI3_HOST
#define PORT_LED_SPI_HZ 2500000
#define PORT_LED_TICK_MS 20
#define PORT_LED_BREATH_MS 2000
#define PORT_LED_IDENTIFY_MS 10000
#define PORT_LED_IDENTIFY_BRIGHTNESS 50

/* 1 leading reset byte, 3 SPI bits per WS2812 bit, >280 us reset (100 bytes = 320 us) */
#define PORT_LED_DATA_LEN (PORT_LED_MAX * 24 * 3 / 8)
#define PORT_LED_TX_LEN (1 + PORT_LED_DATA_LEN + 100)

/* Pins not used by BlueRetro HW2 on N64/GC */
static const uint8_t valid_gpio[] = {18, 21, 22, 23, 25};

static const struct port_led_color identify_colors[PORT_LED_MAX] = {
    {0xFF, 0x00, 0x00, PORT_LED_IDENTIFY_BRIGHTNESS}, /* Red */
    {0x00, 0xFF, 0x00, PORT_LED_IDENTIFY_BRIGHTNESS}, /* Green */
    {0x00, 0x00, 0xFF, PORT_LED_IDENTIFY_BRIGHTNESS}, /* Blue */
    {0xFF, 0xFF, 0x00, PORT_LED_IDENTIFY_BRIGHTNESS}, /* Yellow */
};

static struct port_led_cfg port_led_cfg = {
    .version = PORT_LED_CFG_VER,
    .enable = 0,
    .gpio = 21,
    .chain_pos = {0, 1, 2, 3},
    .color = {
        [PORT_LED_WIRED] = {0xFF, 0xFF, 0xFF, 50}, /* White */
        [PORT_LED_BT] = {0x00, 0x00, 0xFF, 50}, /* Blue */
        [PORT_LED_PAIRING] = {0x00, 0x00, 0xFF, 50}, /* Blue */
        [PORT_LED_BTN] = {0xFF, 0x90, 0x00, 50}, /* Amber */
    },
};

static SemaphoreHandle_t cfg_lock;
static uint8_t port_state[PORT_LED_MAX] = {PORT_LED_OFF, PORT_LED_OFF, PORT_LED_OFF, PORT_LED_OFF};
static volatile uint32_t btn_active;
static volatile uint32_t btn_flash_hz;
static volatile uint32_t identify_end_ms;
static spi_device_handle_t spi_hdl;
static int32_t hw_gpio = -1;
static int32_t hw_fail_gpio = -1;
static uint8_t *tx_buf;
static uint8_t last_grb[PORT_LED_MAX * 3];

/* BLE cfg cmds may arrive before port_led_init(), no task to race with then */
static void cfg_take(void) {
    if (cfg_lock) {
        xSemaphoreTake(cfg_lock, portMAX_DELAY);
    }
}

static void cfg_give(void) {
    if (cfg_lock) {
        xSemaphoreGive(cfg_lock);
    }
}

static uint32_t now_ms(void) {
    return pdTICKS_TO_MS(xTaskGetTickCount());
}

static bool gpio_is_valid(uint8_t gpio) {
    for (uint32_t i = 0; i < ARRAY_SIZE(valid_gpio); i++) {
        if (valid_gpio[i] == gpio) {
            return true;
        }
    }
    return false;
}

static bool cfg_is_valid(const struct port_led_cfg *cfg) {
    if (cfg->version != PORT_LED_CFG_VER || cfg->enable > 1 || !gpio_is_valid(cfg->gpio)) {
        return false;
    }
    for (uint32_t i = 0; i < PORT_LED_MAX; i++) {
        if (cfg->chain_pos[i] >= PORT_LED_MAX) {
            return false;
        }
    }
    for (uint32_t i = 0; i < PORT_LED_COLOR_MAX; i++) {
        if (cfg->color[i].brightness > 100) {
            return false;
        }
    }
    return true;
}

static void cfg_load(void) {
    struct port_led_cfg tmp;
    struct stat st;

    if (stat(PORT_LED_FILE, &st) != 0) {
        printf("# %s: No cfg, using default\n", __FUNCTION__);
        return;
    }

    FILE *file = fopen(PORT_LED_FILE, "rb");
    if (file == NULL) {
        printf("# %s: failed to open file\n", __FUNCTION__);
        return;
    }
    uint32_t count = fread(&tmp, sizeof(tmp), 1, file);
    fclose(file);

    if (count == 1 && cfg_is_valid(&tmp)) {
        port_led_cfg = tmp;
    }
    else {
        printf("# %s: invalid cfg, using default\n", __FUNCTION__);
    }
}

static void hw_deinit(void) {
    if (hw_gpio < 0) {
        return;
    }
    spi_bus_remove_device(spi_hdl);
    spi_bus_free(PORT_LED_SPI_HOST);
    gpio_reset_pin(hw_gpio);
    printf("# %s: GPIO%ld released\n", __FUNCTION__, hw_gpio);
    hw_gpio = -1;
}

static void hw_init(uint8_t gpio) {
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = gpio,
        .miso_io_num = -1,
        .sclk_io_num = -1,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = PORT_LED_TX_LEN,
    };
    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = PORT_LED_SPI_HZ,
        .mode = 0,
        .spics_io_num = -1,
        .queue_size = 1,
    };

    /* Don't retry a failed init until cfg change */
    hw_fail_gpio = gpio;
    if (spi_bus_initialize(PORT_LED_SPI_HOST, &bus_cfg, SPI_DMA_CH_AUTO) != ESP_OK) {
        printf("# %s: SPI bus init fail\n", __FUNCTION__);
        return;
    }
    if (spi_bus_add_device(PORT_LED_SPI_HOST, &dev_cfg, &spi_hdl) != ESP_OK) {
        printf("# %s: SPI device add fail\n", __FUNCTION__);
        spi_bus_free(PORT_LED_SPI_HOST);
        return;
    }
    hw_fail_gpio = -1;
    hw_gpio = gpio;
    /* Force a refresh */
    memset(last_grb, 0xFF, sizeof(last_grb));
    printf("# %s: WS2812 on GPIO%d\n", __FUNCTION__, gpio);
}

static void hw_write(const uint8_t *grb) {
    spi_transaction_t trans = {
        .length = PORT_LED_TX_LEN * 8,
        .tx_buffer = tx_buf,
    };
    uint32_t bit = 8;

    if (hw_gpio < 0 || memcmp(grb, last_grb, sizeof(last_grb)) == 0) {
        return;
    }
    memcpy(last_grb, grb, sizeof(last_grb));

    memset(tx_buf, 0, PORT_LED_TX_LEN);
    for (uint32_t i = 0; i < sizeof(last_grb); i++) {
        for (uint32_t mask = 0x80; mask; mask >>= 1) {
            uint32_t pattern = (grb[i] & mask) ? 0x6 : 0x4;
            for (int32_t j = 2; j >= 0; j--, bit++) {
                if (pattern & BIT(j)) {
                    tx_buf[bit / 8] |= 0x80 >> (bit % 8);
                }
            }
        }
    }
    spi_device_polling_transmit(spi_hdl, &trans);
}

static void set_pixel(uint8_t *grb, uint32_t pos, const struct port_led_color *color, uint32_t scale) {
    /* scale is 0 - 255 on top of the color brightness */
    uint32_t div = 100 * 255;

    grb[pos * 3 + 0] = color->g * color->brightness * scale / div;
    grb[pos * 3 + 1] = color->r * color->brightness * scale / div;
    grb[pos * 3 + 2] = color->b * color->brightness * scale / div;
}

static uint32_t breath_scale(uint32_t ms) {
    uint32_t phase = ms % PORT_LED_BREATH_MS;
    uint32_t tri = (phase < PORT_LED_BREATH_MS / 2) ? phase : PORT_LED_BREATH_MS - phase;

    tri = tri * 255 / (PORT_LED_BREATH_MS / 2);
    /* Square for a more natural looking fade */
    return tri * tri / 255;
}

static void render(uint8_t *grb, uint32_t ms) {
    memset(grb, 0, PORT_LED_MAX * 3);

    if (identify_end_ms && (int32_t)(identify_end_ms - ms) > 0) {
        /* Color by chain position, ignore port mapping */
        for (uint32_t i = 0; i < PORT_LED_MAX; i++) {
            set_pixel(grb, i, &identify_colors[i], 255);
        }
        return;
    }
    identify_end_ms = 0;

    if (btn_active) {
        uint32_t on = 1;

        if (btn_flash_hz) {
            on = !(((ms % 1000) * btn_flash_hz * 2 / 1000) & 1);
        }
        if (on) {
            for (uint32_t i = 0; i < PORT_LED_MAX; i++) {
                set_pixel(grb, i, &port_led_cfg.color[PORT_LED_BTN], 255);
            }
        }
        return;
    }

    for (uint32_t i = 0; i < PORT_LED_MAX; i++) {
        uint8_t state = port_state[i];
        uint32_t pos = port_led_cfg.chain_pos[i];

        switch (state) {
            case PORT_LED_WIRED:
            case PORT_LED_BT:
                set_pixel(grb, pos, &port_led_cfg.color[state], 255);
                break;
            case PORT_LED_PAIRING:
                set_pixel(grb, pos, &port_led_cfg.color[state], breath_scale(ms));
                break;
            default:
                break;
        }
    }
}

static void port_led_task(void *arg) {
    uint8_t grb[PORT_LED_MAX * 3];

    while (1) {
        cfg_take();
        if (port_led_cfg.enable) {
            if (hw_gpio != port_led_cfg.gpio && hw_fail_gpio != port_led_cfg.gpio) {
                hw_deinit();
                hw_init(port_led_cfg.gpio);
            }
            render(grb, now_ms());
            hw_write(grb);
        }
        else if (hw_gpio >= 0) {
            memset(grb, 0, sizeof(grb));
            hw_write(grb);
            hw_deinit();
        }
        cfg_give();
        vTaskDelay(PORT_LED_TICK_MS / portTICK_PERIOD_MS);
    }
}

void port_led_init(void) {
    cfg_lock = xSemaphoreCreateMutex();
    cfg_load();

    /* Other systems use SPI or the free GPIO for their own wired interface */
    if (wired_adapter.system_id != N64 && wired_adapter.system_id != GC) {
        printf("# %s: not supported on this system\n", __FUNCTION__);
        return;
    }

    tx_buf = heap_caps_malloc(PORT_LED_TX_LEN, MALLOC_CAP_DMA);
    if (tx_buf == NULL) {
        printf("# %s: tx_buf alloc fail\n", __FUNCTION__);
        return;
    }

    xTaskCreatePinnedToCore(port_led_task, "port_led_task", 2048, NULL, 3, NULL, 0);
}

void port_led_set_port(uint32_t port, uint8_t state) {
    if (port < PORT_LED_MAX) {
        port_state[port] = state;
    }
}

void port_led_set_btn(uint32_t active, uint32_t flash_hz) {
    btn_flash_hz = flash_hz;
    btn_active = active;
}

void port_led_identify(void) {
    printf("# %s: chain LED 1 Red, 2 Green, 3 Blue, 4 Yellow\n", __FUNCTION__);
    /* 0 mean inactive */
    identify_end_ms = (now_ms() + PORT_LED_IDENTIFY_MS) | 1;
}

void port_led_get_cfg(struct port_led_cfg *cfg) {
    cfg_take();
    *cfg = port_led_cfg;
    cfg_give();
}

int32_t port_led_set_cfg(const struct port_led_cfg *cfg) {
    if (!cfg_is_valid(cfg)) {
        printf("# %s: invalid cfg\n", __FUNCTION__);
        return -1;
    }
    cfg_take();
    port_led_cfg = *cfg;
    hw_fail_gpio = -1;
    cfg_give();
    return 0;
}

int32_t port_led_save_cfg(void) {
    struct port_led_cfg tmp;

    port_led_get_cfg(&tmp);

    FILE *file = fopen(PORT_LED_FILE, "wb");
    if (file == NULL) {
        printf("# %s: failed to open file\n", __FUNCTION__);
        return -1;
    }
    fwrite(&tmp, sizeof(tmp), 1, file);
    fclose(file);
    printf("# %s: cfg saved\n", __FUNCTION__);
    return 0;
}
