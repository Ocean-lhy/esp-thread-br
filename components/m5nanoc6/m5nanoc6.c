/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "esp_err.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "led_indicator.h"
#include "iot_button.h"
#include "bsp/m5nanoc6.h"
#include "bsp_err_check.h"

static const char *TAG = "M5NanoC6";

static const blink_step_t bsp_led_on[] = {
    {LED_BLINK_HOLD, LED_STATE_ON, 0},
    {LED_BLINK_STOP, 0, 0},
};

static const blink_step_t bsp_led_off[] = {
    {LED_BLINK_HOLD, LED_STATE_OFF, 0},
    {LED_BLINK_STOP, 0, 0},
};

static const blink_step_t bsp_led_blink_fast[] = {
    {LED_BLINK_HOLD, LED_STATE_ON, 100},
    {LED_BLINK_HOLD, LED_STATE_OFF, 100},
    {LED_BLINK_LOOP, 0, 0},
};

static const blink_step_t bsp_led_blink_slow[] = {
    {LED_BLINK_HOLD, LED_STATE_ON, 500},
    {LED_BLINK_HOLD, LED_STATE_OFF, 500},
    {LED_BLINK_LOOP, 0, 0},
};

static const blink_step_t bsp_led_breathe_fast[] = {
    {LED_BLINK_BREATHE, LED_STATE_ON, 1000},
    {LED_BLINK_LOOP, 0, 0},
};

static const blink_step_t bsp_led_breathe_slow[] = {
    {LED_BLINK_BREATHE, LED_STATE_ON, 3000},
    {LED_BLINK_LOOP, 0, 0},
};

// 定义LED闪烁模式列表
const blink_step_t *bsp_led_blink_defaults_lists[] = {
    bsp_led_on,
    bsp_led_off,
    bsp_led_blink_fast,
    bsp_led_blink_slow,
    bsp_led_breathe_fast,
    bsp_led_breathe_slow,
};

static const button_config_t bsp_button_config[] = {
    {
        .type = BUTTON_TYPE_GPIO,
        .gpio_button_config = {
            .gpio_num = BSP_BUTTON_MAIN_IO,
            .active_level = 0,
        },
    },
};

static led_indicator_gpio_config_t bsp_leds_gpio_config[] = {
    {
        .is_active_level_high = 1,
        .gpio_num = BSP_LED_BLUE_IO,
    },
};

static const led_indicator_config_t bsp_leds_config[BSP_LED_NUM] = {
    {
        .mode = LED_GPIO_MODE,
        .led_indicator_gpio_config = &bsp_leds_gpio_config[0],
        .blink_lists = bsp_led_blink_defaults_lists,
        .blink_list_num = BSP_LED_MAX,
    },
};

esp_err_t bsp_iot_button_create(button_handle_t btn_array[], int *btn_cnt, int btn_array_size)
{
    esp_err_t ret = ESP_OK;
    if ((btn_array_size < BSP_BUTTON_NUM) ||
            (btn_array == NULL)) {
        return ESP_ERR_INVALID_ARG;
    }

    if (btn_cnt) {
        *btn_cnt = 0;
    }
    for (int i = 0; i < BSP_BUTTON_NUM; i++) {
        btn_array[i] = iot_button_create(&bsp_button_config[i]);
        if (btn_array[i] == NULL) {
            ret = ESP_FAIL;
            break;
        }
        if (btn_cnt) {
            (*btn_cnt)++;
        }
    }
    return ret;
}

esp_err_t bsp_led_indicator_create(led_indicator_handle_t led_array[], int *led_cnt, int led_array_size)
{
    esp_err_t ret = ESP_OK;
    if ((led_array_size < BSP_LED_NUM) ||
            (led_array == NULL)) {
        return ESP_ERR_INVALID_ARG;
    }

    if (led_cnt) {
        *led_cnt = 0;
    }
    for (int i = 0; i < BSP_LED_NUM; i++) {
        led_array[i] = led_indicator_create(&bsp_leds_config[i]);
        if (led_array[i] == NULL) {
            ret = ESP_FAIL;
            break;
        }
        if (led_cnt) {
            (*led_cnt)++;
        }
    }
    return ret;
}

esp_err_t bsp_led_set(led_indicator_handle_t handle, const bool on)
{
    if (on) {
        led_indicator_start(handle, BSP_LED_ON);
    } else {
        led_indicator_start(handle, BSP_LED_OFF);
    }

    return ESP_OK;
}

esp_err_t bsp_ir_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BSP_IR_IO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    return gpio_config(&io_conf);
}

esp_err_t bsp_ws2812_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BSP_WS2812_IO) | (1ULL << BSP_WS2812_EN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    BSP_ERROR_CHECK_RETURN_ERR(gpio_config(&io_conf));
    
    // 默认禁用 WS2812
    return gpio_set_level(BSP_WS2812_EN, 0);
}
