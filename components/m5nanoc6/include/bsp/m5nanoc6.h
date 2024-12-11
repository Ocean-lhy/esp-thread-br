/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief ESP BSP: M5NanoC6
 */

#pragma once

#include "sdkconfig.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "iot_button.h"
#include "led_indicator.h"

/**************************************************************************************************
 *  BSP Capabilities
 **************************************************************************************************/
#define BSP_CAPS_DISPLAY        0  
#define BSP_CAPS_TOUCH          0
#define BSP_CAPS_BUTTONS        1
#define BSP_CAPS_KNOB           0
#define BSP_CAPS_AUDIO          0
#define BSP_CAPS_AUDIO_SPEAKER  0
#define BSP_CAPS_AUDIO_MIC      0
#define BSP_CAPS_SDCARD         0
#define BSP_CAPS_IMU            0

/**************************************************************************************************
 *  M5NanoC6 pinout
 **************************************************************************************************/
/* LED */
#define BSP_LED_BLUE_IO        (GPIO_NUM_7)

/* Button */
#define BSP_BUTTON_MAIN_IO     (GPIO_NUM_9)

/* IR */
#define BSP_IR_IO              (GPIO_NUM_3)

/* WS2812 */
#define BSP_WS2812_IO          (GPIO_NUM_20)
#define BSP_WS2812_EN          (GPIO_NUM_19)

/* Buttons */
typedef enum {
    BSP_BUTTON_MAIN,
    BSP_BUTTON_NUM
} bsp_button_t;

/* LEDs */
typedef enum {
    BSP_LED_BLUE,
    BSP_LED_NUM
} bsp_led_t;

/* Default LED effects */
enum {
    BSP_LED_ON,
    BSP_LED_OFF,
    BSP_LED_BLINK_FAST,
    BSP_LED_BLINK_SLOW,
    BSP_LED_BREATHE_FAST,
    BSP_LED_BREATHE_SLOW,
    BSP_LED_MAX,
};

#ifdef __cplusplus
extern "C" {
#endif

/**************************************************************************************************
 * 
 * Button API
 * 
 **************************************************************************************************/
/**
 * @brief Initialize all buttons
 *
 * Returned button handlers must be used with espressif/button component API
 *
 * @param[out] btn_array      Output button array
 * @param[out] btn_cnt        Number of button handlers saved to btn_array, can be NULL
 * @param[in]  btn_array_size Size of output button array. Must be at least BSP_BUTTON_NUM
 * @return
 *     - ESP_OK               All buttons initialized
 *     - ESP_ERR_INVALID_ARG  btn_array is too small or NULL
 *     - ESP_FAIL             Underlaying iot_button_create failed
 */
esp_err_t bsp_iot_button_create(button_handle_t btn_array[], int *btn_cnt, int btn_array_size);

/**************************************************************************************************
 * 
 * LED API
 * 
 **************************************************************************************************/
/**
 * @brief Initialize all LEDs
 *
 * @param[out] led_array      Output LED array
 * @param[out] led_cnt        Number of LED handlers saved to led_array, can be NULL
 * @param[in]  led_array_size Size of output LED array. Must be at least BSP_LED_NUM
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t bsp_led_indicator_create(led_indicator_handle_t led_array[], int *led_cnt, int led_array_size);

/**
 * @brief Turn LED on/off
 *
 * @param handle led handle
 * @param on Switch LED on/off
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t bsp_led_set(led_indicator_handle_t handle, const bool on);

/**************************************************************************************************
 * 
 * IR API
 * 
 **************************************************************************************************/
/**
 * @brief Initialize IR receiver
 * 
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t bsp_ir_init(void);

/**************************************************************************************************
 * 
 * WS2812 API
 * 
 **************************************************************************************************/
/**
 * @brief Initialize WS2812 LED strip
 * 
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t bsp_ws2812_init(void);

#ifdef __cplusplus
}
#endif
