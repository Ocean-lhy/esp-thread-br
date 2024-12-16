#include "ui.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include <inttypes.h>

static const char *TAG = "UI";

static lv_obj_t *label_status;
static lv_obj_t *label_rssi; 
static lv_obj_t *label_tx_bytes;
static lv_obj_t *label_rx_bytes;
static lv_obj_t *btn_start;
static lv_obj_t *btn_stop;
static lv_obj_t *label_snr;
static lv_obj_t *label_packet_loss;

static uint64_t total_tx_packets = 0;
static uint64_t total_rx_packets = 0;
static uint64_t total_tx_bytes = 0;
static uint64_t total_rx_bytes = 0;
static bool is_sending = false;

static void btn_start_cb(lv_event_t *e)
{
    is_sending = true;
    // 按下开始按钮时重置所有计数
    total_tx_bytes = 0;
    total_rx_bytes = 0;
    total_tx_packets = 0;
    total_rx_packets = 0;
    if (bsp_display_lock(100)) {
        lv_label_set_text(label_tx_bytes, "TX: 0 B");
        lv_label_set_text(label_rx_bytes, "RX: 0 B");
        lv_label_set_text(label_packet_loss, "Loss: 0.00%");
        bsp_display_unlock();
    }
    ESP_LOGI(TAG, "Start sending data");
}

static void btn_stop_cb(lv_event_t *e) 
{
    is_sending = false;
    ESP_LOGI(TAG, "Stop sending data");
}

void ui_init(lv_display_t *disp)
{
    // 创建状态标签
    label_status = lv_label_create(lv_scr_act());
    lv_label_set_text(label_status, "BLE: Disconnected");
    lv_obj_align(label_status, LV_ALIGN_TOP_LEFT, 10, 10);

    // 创建RSSI标签
    label_rssi = lv_label_create(lv_scr_act());
    lv_label_set_text(label_rssi, "RSSI: N/A");
    lv_obj_align(label_rssi, LV_ALIGN_TOP_LEFT, 10, 40);

    // 创建发送字节标签
    label_tx_bytes = lv_label_create(lv_scr_act());
    lv_label_set_text(label_tx_bytes, "TX: 0 bytes");
    lv_obj_align(label_tx_bytes, LV_ALIGN_TOP_LEFT, 10, 100);

    // 创建接收字节标签
    label_rx_bytes = lv_label_create(lv_scr_act());
    lv_label_set_text(label_rx_bytes, "RX: 0 bytes");
    lv_obj_align(label_rx_bytes, LV_ALIGN_TOP_LEFT, 10, 130);

    // 创建SNR标签
    label_snr = lv_label_create(lv_scr_act());
    lv_label_set_text(label_snr, "SNR: N/A");
    lv_obj_align(label_snr, LV_ALIGN_TOP_LEFT, 10, 70);

    // 创建丢包率标签
    label_packet_loss = lv_label_create(lv_scr_act());
    lv_label_set_text(label_packet_loss, "Loss: 0.00%");
    lv_obj_align(label_packet_loss, LV_ALIGN_TOP_LEFT, 10, 160);

    // 创建开始按钮
    btn_start = lv_btn_create(lv_scr_act());
    lv_obj_add_event_cb(btn_start, btn_start_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_align(btn_start, LV_ALIGN_BOTTOM_LEFT, 10, -10);
    lv_obj_t *label_start = lv_label_create(btn_start);
    lv_label_set_text(label_start, "Start");
    lv_obj_center(label_start);

    // 创建停止按钮
    btn_stop = lv_btn_create(lv_scr_act());
    lv_obj_add_event_cb(btn_stop, btn_stop_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_align(btn_stop, LV_ALIGN_BOTTOM_RIGHT, -10, -10);
    lv_obj_t *label_stop = lv_label_create(btn_stop);
    lv_label_set_text(label_stop, "Stop");
    lv_obj_center(label_stop);
}

void ui_update_ble_status(bool connected)
{
    if(connected) {
        lv_label_set_text(label_status, "BLE: Connected");
    } else {
        lv_label_set_text(label_status, "BLE: Disconnected");
    }
}

void ui_update_rssi(int8_t rssi)
{
    char buf[32];
    snprintf(buf, sizeof(buf), "RSSI: %d dBm", rssi);
    lv_label_set_text(label_rssi, buf);
}

void ui_update_snr(int8_t snr)
{
    char buf[32];
    snprintf(buf, sizeof(buf), "SNR: %d dB", snr);
    if (bsp_display_lock(100)) {
        lv_label_set_text(label_snr, buf);
        bsp_display_unlock();
    }
}

void ui_update_bytes(uint32_t tx_bytes, uint32_t rx_bytes)
{
    char buf[64];
    
    // 更新总字节数
    total_tx_bytes += tx_bytes;
    total_rx_bytes += rx_bytes;
    
    // 更新包计数
    if (tx_bytes > 0) {
        total_tx_packets++;
    }
    if (rx_bytes > 0) {
        total_rx_packets++;
    }
    
    // 计算丢包率
    float loss_rate = 0.0f;
    if (total_tx_packets > 0) {
        loss_rate = 100.0f * (total_tx_packets - total_rx_packets) / total_tx_packets;
    }
    
    if (bsp_display_lock(100)) {
        // 根据字节数大小选择合适的单位显示
        if (total_tx_bytes < 1024) {
            snprintf(buf, sizeof(buf), "TX: %llu B", total_tx_bytes);
        } else if (total_tx_bytes < 1024*1024) {
            snprintf(buf, sizeof(buf), "TX: %.2f KB", total_tx_bytes/1024.0);
        } else if (total_tx_bytes < 1024*1024*1024) {
            snprintf(buf, sizeof(buf), "TX: %.2f MB", total_tx_bytes/(1024.0*1024.0));
        } else {
            snprintf(buf, sizeof(buf), "TX: %.2f GB", total_tx_bytes/(1024.0*1024.0*1024.0));
        }
        lv_label_set_text(label_tx_bytes, buf);
        
        if (total_rx_bytes < 1024) {
            snprintf(buf, sizeof(buf), "RX: %llu B", total_rx_bytes);
        } else if (total_rx_bytes < 1024*1024) {
            snprintf(buf, sizeof(buf), "RX: %.2f KB", total_rx_bytes/1024.0);
        } else if (total_rx_bytes < 1024*1024*1024) {
            snprintf(buf, sizeof(buf), "RX: %.2f MB", total_rx_bytes/(1024.0*1024.0));
        } else {
            snprintf(buf, sizeof(buf), "RX: %.2f GB", total_rx_bytes/(1024.0*1024.0*1024.0));
        }
        lv_label_set_text(label_rx_bytes, buf);
        
        bsp_display_unlock();
    }
}

bool ui_get_sending_status(void)
{
    return is_sending;
} 

void ui_update_packet_loss(float loss_rate)
{
    char buf[32];
    snprintf(buf, sizeof(buf), "Loss: %.2f%%", loss_rate);
    if (bsp_display_lock(100)) {
        lv_label_set_text(label_packet_loss, buf);
        bsp_display_unlock();
    }
}