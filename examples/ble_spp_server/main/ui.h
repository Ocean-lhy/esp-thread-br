#pragma once

#include "bsp/m5stack_core_2.h"
#include <stdbool.h>
#include <stdint.h>

/**
 * @brief 初始化UI界面
 * 
 * @param disp LVGL显示设备句柄
 */
void ui_init(lv_display_t *disp);

/**
 * @brief 更新蓝牙连接状态
 * 
 * @param connected true表示已连接,false表示未连接
 */
void ui_update_ble_status(bool connected);

/**
 * @brief 更新RSSI信号强度
 * 
 * @param rssi RSSI值(dBm)
 */
void ui_update_rssi(int8_t rssi);

/**
 * @brief 更新SNR信号强度
 * 
 * @param snr SNR值(dB)
 */
void ui_update_snr(int8_t snr);

/**
 * @brief 更新收发字节数
 * 
 * @param tx_bytes 发送的字节数
 * @param rx_bytes 接收的字节数
 */
void ui_update_bytes(uint32_t tx_bytes, uint32_t rx_bytes);

/**
 * @brief 获取当前发送状态
 * 
 * @return true 正在发送
 * @return false 已停止发送
 */
bool ui_get_sending_status(void); 

/**
 * @brief 更新丢包率
 * 
 * @param loss_rate 丢包率(%)
 */
void ui_update_packet_loss(float loss_rate);