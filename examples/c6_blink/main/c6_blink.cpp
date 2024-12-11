#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "led_strip.h"
#include "iot_button.h"
#include "bsp/m5nanoc6.h"

static const char *TAG = "c6_blink";

// LED CONFIG
#define LED_STRIP_LED_NUMBERS 1
#define LED_STRIP_RMT_RES_HZ  (10 * 1000 * 1000)  // 10MHz resolution

// Define several colors
typedef struct {
    uint8_t r;
    uint8_t g; 
    uint8_t b;
} rgb_color_t;

static const rgb_color_t colors[] = {
    {255, 0, 0},    // RED
    {0, 255, 0},    // GREEN
    {0, 0, 255},    // BLUE
    {255, 255, 0},  // YELLOW
    {0, 255, 255},  // CYAN
    {255, 0, 255},  // PURPLE
};
#define COLOR_COUNT (sizeof(colors) / sizeof(colors[0]))

static int current_color = 0;
static led_strip_handle_t led_strip;
static button_handle_t button_handles[BSP_BUTTON_NUM];
static led_indicator_handle_t led_handles[BSP_LED_NUM];

// Button callback function
static void button_click_cb(void *arg, void *data)
{
    ESP_LOGI(TAG, "Button pressed!");
    
    // Switch to the next color
    current_color = (current_color + 1) % COLOR_COUNT;
    
    // Set the new color
    ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, 0, colors[current_color].r, 
                       colors[current_color].g, colors[current_color].b));
    ESP_ERROR_CHECK(led_strip_refresh(led_strip));
    
    // Switch the blue LED state
    ESP_ERROR_CHECK(bsp_led_set(led_handles[BSP_LED_BLUE], current_color % 2));
    
    ESP_LOGI(TAG, "Changed color to: R:%d G:%d B:%d", 
             colors[current_color].r, colors[current_color].g, colors[current_color].b);
}

extern "C" void app_main(void)
{
    int btn_cnt = 0;
    int led_cnt = 0;
    
    // Initialize buttons
    ESP_ERROR_CHECK(bsp_iot_button_create(button_handles, &btn_cnt, BSP_BUTTON_NUM));
    ESP_ERROR_CHECK(iot_button_register_cb(button_handles[BSP_BUTTON_MAIN], 
                                         BUTTON_PRESS_DOWN, button_click_cb, NULL));
    
    // Initialize LED indicators
    ESP_ERROR_CHECK(bsp_led_indicator_create(led_handles, &led_cnt, BSP_LED_NUM));
    ESP_ERROR_CHECK(bsp_led_set(led_handles[BSP_LED_BLUE], true));  // Default blue LED on
    
    // Initialize WS2812
    ESP_ERROR_CHECK(bsp_ws2812_init());
    ESP_ERROR_CHECK(gpio_set_level(BSP_WS2812_EN, 1));  // Enable WS2812
    
    // 配置LED strip
    led_strip_config_t strip_config = {
        BSP_WS2812_IO,              // strip_gpio_num
        LED_STRIP_LED_NUMBERS,      // max_leds
        LED_PIXEL_FORMAT_GRB,       // led_pixel_format
        LED_MODEL_WS2812,           // led_model
        {false}                     // flags
    };

    led_strip_rmt_config_t rmt_config = {
        RMT_CLK_SRC_DEFAULT,        // clk_src
        LED_STRIP_RMT_RES_HZ,       // resolution_hz
        {false}                     // flags
    };

    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));

    // Set the initial color
    ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, 0, colors[0].r, colors[0].g, colors[0].b));
    ESP_ERROR_CHECK(led_strip_refresh(led_strip));

    ESP_LOGI(TAG, "RGB LED START");
    ESP_LOGI(TAG, "Press the button to change color");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
