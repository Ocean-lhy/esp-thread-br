/*
 * SPDX-FileCopyrightText: 2015-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_check.h"
#include "bsp/esp-bsp.h"
#include "lvgl.h"
#include "ui_boot_animate.h"
#include "esp_event.h"
#include "openthread/border_agent.h"
#include "ui_for_br.h"
#include "esp_openthread.h"
#include "esp_openthread_netif_glue.h"
#include "esp_openthread_lock.h"
#include "esp_random.h"
#include "nvs.h"

#define TAG "esp_ot_br_ui"

// static const char *TAG = "ui_for";
bool flag_ui_ready = false;
lv_obj_t *epskc_generate_btn = NULL;
lv_obj_t *epskc_clear_btn = NULL;
lv_obj_t *epskc_label = NULL;
lv_obj_t *main_page = NULL;
lv_obj_t *epskc_page = NULL;
static char epskc_str[10];
static bool is_espkc_page_created = false;
char s_br_web[40] = "";
char s_wifi_ipv4_address[16];
static void ui_after_boot(void);
static void add_main_page(void);
static void br_meshcop_e_publish_handler(void *args, esp_event_base_t base, int32_t event_id, void *data);
static void br_meshcop_e_remove_handler(void *args, esp_event_base_t base, int32_t event_id, void *data);
static void epskc_generate_cb(lv_event_t *e);
static void factoryreset_cb(lv_event_t *e);
static void erase_all_settings(void);
static void add_epskc_page(void);
static void display_epskc_page(void);
static char *eps_openthread_generate_epskc(void);
static void erase_all_settings(void);

lv_obj_t *border_router = NULL;
lv_obj_t *booting = NULL;
static void ui_after_boot(void)
{
    // ESP logo
    LV_IMG_DECLARE(esp_logo_tiny);
    lv_obj_t *img_logo = lv_img_create(lv_scr_act());
    lv_img_set_src(img_logo, &esp_logo_tiny);
    lv_img_set_zoom(img_logo, 300);
    lv_obj_align(img_logo, LV_ALIGN_CENTER, -64, -50);
    lv_obj_t *img_text = lv_label_create(lv_scr_act());         
    lv_label_set_text(img_text, "Espressif"); 
    lv_obj_set_style_text_color(img_text, lv_color_black(), LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(img_text, &lv_font_montserrat_28, LV_PART_MAIN);
    lv_obj_align_to(img_text, img_logo, LV_ALIGN_OUT_RIGHT_MID, 5, 0);

    border_router = lv_label_create(lv_scr_act());         /*Add a label to the button*/
    lv_label_set_text(border_router, "Thread Border Router"); /*Set the labels text*/
    lv_obj_set_style_text_color(border_router, lv_color_black(), LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(border_router, &lv_font_montserrat_26, LV_PART_MAIN);
    lv_obj_align(border_router, LV_ALIGN_CENTER, 0, 0);


    booting = lv_label_create(lv_scr_act());         /*Add a label to the button*/
    lv_label_set_text(booting, "starting up..."); /*Set the labels text*/
    lv_obj_set_style_text_color(booting, lv_color_black(), LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(booting, &lv_font_montserrat_16, LV_PART_MAIN);
    lv_obj_align_to(booting, border_router, LV_ALIGN_OUT_BOTTOM_MID, 0, 20);
    flag_ui_ready = true;
}

void ui_after_boot_but_wifi_fail(void)
{
    if (border_router && booting) {
        lv_label_set_text(booting, "Invaild wifi"); /*Set the labels text*/
        lv_obj_set_style_text_color(booting, lv_color_make(255, 0, 0), LV_STATE_DEFAULT);
        lv_obj_set_style_text_font(booting, &lv_font_montserrat_12, LV_PART_MAIN);
        lv_obj_align_to(booting, border_router, LV_ALIGN_OUT_BOTTOM_MID, 0, 5);

        lv_obj_t *wifi_warning = lv_label_create(lv_scr_act());
        lv_label_set_text(wifi_warning, "open console, set a new one with:"); /*Set the labels text*/
        lv_obj_set_style_text_color(wifi_warning, lv_color_make(255, 0, 0), LV_STATE_DEFAULT);
        lv_obj_set_style_text_font(wifi_warning, &lv_font_montserrat_12, LV_PART_MAIN);
        lv_obj_align_to(wifi_warning, booting, LV_ALIGN_OUT_BOTTOM_MID, 0, 5);
         

        lv_obj_t *wifi_config_command = lv_label_create(lv_scr_act());
        lv_label_set_text(wifi_config_command, "wifi connect -s <ssid> -p <password>"); /*Set the labels text*/
        lv_obj_set_style_text_color(wifi_config_command, lv_color_make(255, 0, 0), LV_STATE_DEFAULT);
        lv_obj_set_style_text_font(wifi_config_command, &lv_font_montserrat_16, LV_PART_MAIN);
        lv_obj_align_to(wifi_config_command, wifi_warning, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);

        lv_obj_t *restart_text = lv_label_create(lv_scr_act());
        lv_label_set_text(restart_text, "and then restart the device"); /*Set the labels text*/
        lv_obj_set_style_text_color(restart_text, lv_color_make(255, 0, 0), LV_STATE_DEFAULT);
        lv_obj_set_style_text_font(restart_text, &lv_font_montserrat_12, LV_PART_MAIN);
        lv_obj_align_to(restart_text, wifi_config_command, LV_ALIGN_OUT_BOTTOM_MID, 0, 5);
    }
}


esp_err_t ui_for_br_start(void)
{
    bsp_i2c_init();
    bsp_display_start();
    bsp_display_lock(0);
    bsp_display_backlight_on();
    lv_obj_set_style_bg_color(lv_scr_act(), lv_color_make(237, 238, 239), LV_STATE_DEFAULT);
    boot_animate_start(ui_after_boot);

    bsp_display_unlock();
    

    return ESP_OK;
}

void config_box(void)
{
    // start ui
    epskc_str[0] = '\0';
    bsp_display_lock(0);
    lv_obj_clean(lv_scr_act());
    add_main_page();
    bsp_display_unlock();
}

static void add_main_page(void)
{   
    esp_openthread_register_meshcop_e_handler(br_meshcop_e_publish_handler, true);
    esp_openthread_register_meshcop_e_handler(br_meshcop_e_remove_handler, false);
    main_page = lv_obj_create(lv_scr_act());
    lv_obj_set_size(main_page, lv_obj_get_width(lv_scr_act()), lv_obj_get_height(lv_scr_act()));
    lv_obj_set_style_bg_color(main_page, lv_obj_get_style_bg_color(lv_scr_act(), LV_STATE_DEFAULT), LV_STATE_DEFAULT);
    lv_obj_center(main_page);

    // ESP logo
    LV_IMG_DECLARE(esp_logo_tiny);
    lv_obj_t *img_logo = lv_img_create(main_page);
    lv_img_set_src(img_logo, &esp_logo_tiny);
    lv_obj_align(img_logo, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_obj_t *img_text = lv_label_create(main_page);         
    lv_label_set_text(img_text, "Espressif"); 
    lv_obj_set_style_text_color(img_text, lv_color_black(), LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(img_text, &lv_font_montserrat_20, LV_PART_MAIN);
    lv_obj_align_to(img_text, img_logo, LV_ALIGN_OUT_RIGHT_MID, 5, 0);

    // epskc generation button
    epskc_generate_btn = lv_btn_create(main_page);                                
    lv_obj_set_size(epskc_generate_btn, 200, 50);                                    
    lv_obj_set_style_bg_color(epskc_generate_btn, lv_color_make(220, 220, 220), LV_STATE_DEFAULT);
    lv_obj_align(epskc_generate_btn, LV_ALIGN_CENTER, 0, -10);                              
    lv_obj_add_event_cb(epskc_generate_btn, epskc_generate_cb, LV_EVENT_CLICKED, NULL); 
    lv_obj_t *epskc_generate_label = lv_label_create(epskc_generate_btn);         
    lv_label_set_text(epskc_generate_label, "generate epskc"); 
    lv_obj_set_style_text_color(epskc_generate_label, lv_color_black(), LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(epskc_generate_label, &lv_font_montserrat_24, LV_PART_MAIN);
    lv_obj_center(epskc_generate_label);

    // br web
    lv_obj_t *br_web_label = lv_label_create(main_page);
    lv_label_set_text(br_web_label, s_br_web); 
    lv_obj_set_style_text_color(br_web_label, lv_color_black(), LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(br_web_label, &lv_font_montserrat_16, LV_PART_MAIN);
    lv_obj_align_to(br_web_label, epskc_generate_btn, LV_ALIGN_OUT_BOTTOM_MID, 0, 30);

    // factoryreset button
    lv_obj_t *factoryreset_btn = lv_btn_create(main_page);                                
    lv_obj_set_size(factoryreset_btn, 80, 20);                                  
    lv_obj_set_style_bg_color(factoryreset_btn, lv_color_make(220, 220, 220), LV_STATE_DEFAULT);
    lv_obj_align(factoryreset_btn, LV_ALIGN_TOP_RIGHT, 0, 0);
    lv_obj_add_event_cb(factoryreset_btn, factoryreset_cb, LV_EVENT_CLICKED, NULL); 
    lv_obj_t *factoryreset_label = lv_label_create(factoryreset_btn);         
    lv_label_set_text(factoryreset_label, "factoryreset"); 
    lv_obj_set_style_text_color(factoryreset_label, lv_color_make(255, 0, 0), LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(factoryreset_label, &lv_font_montserrat_12, LV_PART_MAIN);
    lv_obj_center(factoryreset_label);
}

static void br_meshcop_e_publish_handler(void *args, esp_event_base_t base, int32_t event_id, void *data)
{
}

static void br_meshcop_e_remove_handler(void *args, esp_event_base_t base, int32_t event_id, void *data)
{
    bsp_display_lock(0);
    lv_obj_add_flag(epskc_page, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(main_page, LV_OBJ_FLAG_HIDDEN);
    bsp_display_unlock();
}

static void epskc_generate_cb(lv_event_t *e)
{
    bsp_display_lock(0);
    lv_obj_add_flag(main_page, LV_OBJ_FLAG_HIDDEN);
    bsp_display_unlock();
    eps_openthread_generate_epskc();
    if (!is_espkc_page_created) {
        is_espkc_page_created = true;
        add_epskc_page();
    } else {
        display_epskc_page();
    }
}

static void factoryreset_cb(lv_event_t *e)
{
    erase_all_settings();
    esp_restart();
}

static void add_epskc_page(void)
{
    bsp_display_lock(0);
    epskc_page = lv_obj_create(lv_scr_act());
    lv_obj_set_size(epskc_page, lv_obj_get_width(lv_scr_act()), lv_obj_get_height(lv_scr_act()));
    lv_obj_set_style_bg_color(epskc_page, lv_obj_get_style_bg_color(lv_scr_act(), LV_STATE_DEFAULT), LV_STATE_DEFAULT);
    lv_obj_center(epskc_page);

    // ESP_LOGO
    LV_IMG_DECLARE(esp_logo_tiny);
    lv_obj_t *img_logo = lv_img_create(epskc_page);
    lv_img_set_src(img_logo, &esp_logo_tiny);
    lv_obj_align(img_logo, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_obj_t *img_text = lv_label_create(epskc_page);         
    lv_label_set_text(img_text, "Espressif"); 
    lv_obj_set_style_text_color(img_text, lv_color_black(), LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(img_text, &lv_font_montserrat_20, LV_PART_MAIN);
    lv_obj_align_to(img_text, img_logo, LV_ALIGN_OUT_RIGHT_MID, 5, 0);

    
    epskc_label = lv_label_create(epskc_page);      
    lv_label_set_text(epskc_label, epskc_str);
    lv_obj_set_style_text_color(epskc_label, lv_color_black(), LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(epskc_label, &lv_font_montserrat_48, LV_PART_MAIN);
    lv_obj_center(epskc_label);

    lv_obj_clear_flag(epskc_page, LV_OBJ_FLAG_HIDDEN);
    bsp_display_unlock();
}

static void display_epskc_page(void)
{
    bsp_display_lock(0);
    lv_label_set_text(epskc_label, epskc_str); 
    lv_obj_clear_flag(epskc_page, LV_OBJ_FLAG_HIDDEN);
    bsp_display_unlock();
}

static char *eps_openthread_generate_epskc(void)
{
    uint32_t radom_epskc = esp_random() % 1000000000;

    for (int i = 0; i < 9; i++) {
        epskc_str[8 - i] = radom_epskc % 10 + '0';
        radom_epskc = radom_epskc / 10;
    }
    epskc_str[9] = '\0';
    esp_openthread_lock_acquire(portMAX_DELAY);

    if (otBorderAgentSetEphemeralKey(esp_openthread_get_instance(), epskc_str, CONFIG_OPENTHREAD_EPHEMERALKEY_LIFE_TIME * 1000, 49160) != OT_ERROR_NONE) {
        ESP_LOGE(TAG, "Fail to generate ephemeral key");
    }
    esp_openthread_lock_release();
    return epskc_str;
}

static void erase_all_settings(void)
{
    nvs_handle_t ot_nvs_handle;
    esp_err_t err = nvs_open("openthread", NVS_READWRITE, &ot_nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS namespace (0x%x)", err);
        assert(0);
    }
    nvs_erase_all(ot_nvs_handle);
    nvs_close(ot_nvs_handle);
}