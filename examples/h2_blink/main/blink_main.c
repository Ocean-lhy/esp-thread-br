/* LED Running Light Example for ESP32-H2 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"

// Define LED sequence
typedef enum 
{
    BASE_G12 = 1, 
    BASE_G02 = 2,
    BASE_G21 = 3,
    BASE_G16 = 4,
    BASE_G03 = 5,
    BASE_G18 = 6, 
    BASE_G19 = 7, 
    BASE_G23 = 8, 
    BASE_G15 = 9, 
    BASE_G00 = 10,
    BASE_G13 = 11,
    BASE_G05 = 12,
    BASE_G22 = 13,
    BASE_G17 = 14, 
    BASE_G01 = 15, 
    BASE_G26 = 16,
    BASE_G25 = 17,
    BASE_G35 = 18, 
    BASE_G36 = 19, 
    LED_BASE_MAX
} LED_BASE_GPIO;

// Structure to store GPIO mapping relationship
typedef struct {
    int h2_gpio;           // H2 GPIO number
    LED_BASE_GPIO led_id;  // Corresponding LED in sequence
} gpio_map_t;

// Define the mapping relationship between H2 GPIO and LED sequence
static const gpio_map_t gpio_mapping[] = {
    {4,  BASE_G12},    // G4-G12
    {9,  BASE_G16},    // G9-G16
    {0,  BASE_G18},    // G0-G18
    {1,  BASE_G19},    // G1-G19
    {3,  BASE_G23},    // G3-G23
    {2,  BASE_G15},    // G2-G15
    {12, BASE_G00},    // G12-G0
    {23, BASE_G17},    // G23-G17
    {5,  BASE_G25},    // G5-G25
    {24, BASE_G35},    // G24-G35
};

static LED_BASE_GPIO get_led_index(int h2_gpio) {
    for (int i = 0; i < sizeof(gpio_mapping)/sizeof(gpio_mapping[0]); i++) {
        if (gpio_mapping[i].h2_gpio == h2_gpio) {
            return gpio_mapping[i].led_id;
        }
    }
    return LED_BASE_MAX;
}

static int compare_gpios(const void *a, const void *b) {
    int gpio_a = *(const int*)a;
    int gpio_b = *(const int*)b;
    
    LED_BASE_GPIO led_a = get_led_index(gpio_a);
    LED_BASE_GPIO led_b = get_led_index(gpio_b);
    
    if (led_a == LED_BASE_MAX) return 1;
    if (led_b == LED_BASE_MAX) return -1;
    
    return led_a - led_b;
}

#define H2_MISO_GPIO        1
#define H2_CLK_GPIO         0
#define H2_MOSI_GPIO        3
#define H2_CS_GPIO          2
#define H2_BT_PRIO_GPIO     5
#define H2_BT_ACTIVE_GPIO   4
#define H2_WL_ACTIVE_GPIO   12
#define H2_TXD_GPIO         24
#define H2_RXD_GPIO         23
#define H2_BOOT_GPIO        9
// #define H2_EN_GPIO       NULL

static int valid_gpios[] = {
    H2_BOOT_GPIO, 
    H2_BT_ACTIVE_GPIO, 
    H2_CLK_GPIO, 
    H2_MISO_GPIO, 
    H2_MOSI_GPIO, 
    H2_CS_GPIO, 
    H2_WL_ACTIVE_GPIO, 
    H2_RXD_GPIO, 
    H2_BT_PRIO_GPIO, 
    H2_TXD_GPIO
};

// Function to sort valid_gpios according to LED base sequence
static void sort_valid_gpios(void) {
    qsort(valid_gpios, sizeof(valid_gpios)/sizeof(valid_gpios[0]), 
          sizeof(valid_gpios[0]), compare_gpios);
}

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
    sort_valid_gpios();
    
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
