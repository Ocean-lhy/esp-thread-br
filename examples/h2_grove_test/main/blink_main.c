/* LED Running Light Example for ESP32-H2 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"

#define H2_TXD_GPIO         24
#define H2_RXD_GPIO         23
#define H2_BOOT_GPIO        9


static void gpio_init(void)
{
    gpio_config_t io_conf1 = 
    {
        .mode = GPIO_MODE_OUTPUT,          
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,    
    };
    io_conf1.pin_bit_mask = (1ULL << H2_TXD_GPIO);
    gpio_config(&io_conf1);
    gpio_set_level(H2_TXD_GPIO, 0);
    io_conf1.pin_bit_mask = (1ULL << H2_RXD_GPIO);
    gpio_config(&io_conf1);
    gpio_set_level(H2_RXD_GPIO, 0);

    gpio_config_t io_conf2 = 
    {
        .mode = GPIO_MODE_INPUT,          
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,    
    };
    io_conf2.pin_bit_mask = (1ULL << H2_BOOT_GPIO);
    gpio_config(&io_conf2);

    vTaskDelay(pdMS_TO_TICKS(10));
}

void app_main(void)
{
    
    gpio_init();
    
    int boot_level = 0;

    while (1) 
    {
        boot_level = gpio_get_level(H2_BOOT_GPIO);
        if (boot_level == 0)
        {
            // ESP_LOGI("BOOT", "BOOT_GPIO: %d", boot_level);
            gpio_set_level(H2_TXD_GPIO, 1);
            gpio_set_level(H2_RXD_GPIO, 0);
        }
        else
        {
            // ESP_LOGI("BOOT", "BOOT_GPIO: %d", boot_level);
            gpio_set_level(H2_TXD_GPIO, 0);
            gpio_set_level(H2_RXD_GPIO, 1);
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}
