/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "esp_ot_cli_extension.h"
#include "esp_openthread.h"
#include "esp_ot_iperf.h"
#include "esp_ot_ota_commands.h"
#include "esp_ot_tcp_socket.h"
#include "esp_ot_udp_socket.h"
#include "esp_ot_wifi_cmd.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"
#include "openthread/cli.h"

static const otCliCommand kCommands[] = {
#if CONFIG_OPENTHREAD_CLI_SOCKET
    {"tcpsockserver", esp_ot_process_tcp_server},
    {"tcpsockclient", esp_ot_process_tcp_client},
    {"udpsockserver", esp_ot_process_udp_server},
    {"udpsockclient", esp_ot_process_udp_client},
    {"mcast", esp_ot_process_mcast_group},
#endif // CONFIG_OPENTHREAD_CLI_SOCKET
#if CONFIG_OPENTHREAD_CLI_IPERF
    {"iperf", esp_ot_process_iperf},
#endif // CONFIG_OPENTHREAD_CLI_IPERF
#if CONFIG_OPENTHREAD_CLI_WIFI
    {"wifi", esp_ot_process_wifi_cmd},
#endif // CONFIG_OPENTHREAD_CLI_WIFI
#if CONFIG_OPENTHREAD_CLI_OTA
    {"ota", esp_openthread_process_ota_command},
#endif // CONFIG_OPENTHREAD_CLI_RCP_UPDATE
};

void esp_cli_custom_command_init()
{
    otInstance *instance = esp_openthread_get_instance();
    otCliSetUserCommands(kCommands, (sizeof(kCommands) / sizeof(kCommands[0])), instance);
}
