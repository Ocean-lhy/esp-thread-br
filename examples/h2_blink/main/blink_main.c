/* LED Running Light Example for ESP32-H2 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"

// map core    G9-G36 G4-G12 G23-G16 G0-G18 G1-G19 G3-G23 G2-G15 G12-G0 G24-G17 G5-G25
// map cores3  G0-G8  G4-G6  G23-G18 G0-G36 G1-G35 G3-G37 G2-G13 G12-G0 G24-G17 G5-G7
static const int valid_gpios[] =
{
    9, 4, 23, 0, 1, 3, 2, 12, 24, 5
};

#define GPIO_COUNT (sizeof(valid_gpios) / sizeof(valid_gpios[0]))
#define FLOW_ROUNDS 2       // 流水次数
#define FLOW_SPEED 200      // 流水速度
#define ALL_ON_TIME 5000    // 全亮时间

static void set_all_leds(bool on) 
{
    for (int i = 0; i < GPIO_COUNT; i++) 
    {
        gpio_set_level(valid_gpios[i], on);
    }
}

static void gpio_init(void)
{
    gpio_config_t io_conf = 
    {
        .mode = GPIO_MODE_OUTPUT,          
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,    
    };

    for (int i = 0; i < GPIO_COUNT; i++) 
    {
        io_conf.pin_bit_mask = (1ULL << valid_gpios[i]);
        gpio_config(&io_conf);
        gpio_set_level(valid_gpios[i], 0);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void app_main(void)
{
    gpio_init();
    
    int current_led = 0;
    bool led_on = true;
    int flow_count = 0;  // count rounds

    while (1) 
    {
        // flow stage
        if (flow_count < FLOW_ROUNDS) 
        {
            if (led_on)
            {
                gpio_set_level(valid_gpios[current_led], 1);
            }
            else
            {
                gpio_set_level(valid_gpios[current_led], 0);
                current_led = (current_led + 1) % GPIO_COUNT;
                
                // calculate full flow rounds
                if (current_led == 0)
                {
                    flow_count++;  // direct count, no longer use led_cycles
                }
            }

            led_on = !led_on;
            if (led_on)
            {
                vTaskDelay(pdMS_TO_TICKS(10));
            }
            else
            {
                vTaskDelay(pdMS_TO_TICKS(FLOW_SPEED));
            }
        }
        // all on stage
        else
        {  
            set_all_leds(true);
            vTaskDelay(pdMS_TO_TICKS(ALL_ON_TIME));
            
            set_all_leds(false);
            
            // recovery and start new flow rounds
            flow_count = 0;
            current_led = 0;
            led_on = true;
        }
    }
}
