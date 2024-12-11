/*
 * SPDX-FileCopyrightText: 2015-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#pragma once

#include <stdbool.h>
#include "esp_err.h"
#include "lvgl.h"

extern bool flag_ui_ready;
extern char s_br_web[40];
extern char s_wifi_ipv4_address[16];

esp_err_t ui_for_br_start(void);
void ui_after_boot_but_wifi_fail(void);

void config_box(void);