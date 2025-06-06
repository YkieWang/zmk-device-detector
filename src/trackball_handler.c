/*
 * Copyright (c) 2024 YkieWang
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zmk_driver_device_detector/device_detector.h>
#include "zmk_driver_device_detector/trackball_handler.h"

// Placeholder for API from the actual badjeff/zmk-pmw3610-driver
// These would be declared in a header file from that driver, e.g., <zmk_pmw3610.h>
// extern int zmk_pmw3610_start_reporting(const struct device *pmw_dev);
// extern int zmk_pmw3610_stop_reporting(const struct device *pmw_dev);

LOG_MODULE_REGISTER(trackball_handler, CONFIG_LOG_DEFAULT_LEVEL);

// 添加全局变量来跟踪轨迹球的启用状态
static bool trackball_enabled = false;

// 获取配置结构体定义（与device_detector.c中的定义一致）
struct device_detector_config {
    const struct device *adc_dev; // ADC device pointer
    uint8_t adc_channel_id;
    uint16_t poll_interval_ms;
    // Voltage thresholds
    uint16_t voltage_none_max_mv;
    uint16_t voltage_joystick_min_mv;
    uint16_t voltage_joystick_max_mv;
    uint16_t voltage_trackball_min_mv;
    uint16_t voltage_trackball_max_mv;
    uint16_t voltage_encoder_min_mv;
    uint16_t voltage_encoder_max_mv;
    // ADC specific configuration that might be needed for adc_raw_to_millivolts
    int32_t adc_vref_mv; // ADC reference voltage in millivolts
    uint8_t adc_resolution; // ADC resolution in bits
    const struct device *controlled_encoder_dev; // Added for controlled encoder
    const struct device *controlled_joystick_dev;  // Added for joystick
    const struct device *controlled_trackball_dev; // Added for trackball
    // 各种驱动指针
    const struct device *joystick_dev;
    const struct device *trackball_dev;
    const struct device *encoder_dev;
};

/**
 * @brief Initializes the trackball device.
 *
 * This function is called by the device_detector when a trackball is detected.
 * It ensures the trackball device (e.g., PMW3610) is ready to use.
 *
 * @param dev_detector_dev Pointer to the device_detector device instance.
 * @return int 0 on success, or an error code on failure.
 */
int trackball_handler_init(const struct device *dev_detector_device) {
    LOG_INF("Initializing trackball device");
    
    // 获取设备检测器配置
    const struct device_detector_config *config = dev_detector_device->config;
    if (config == NULL) {
        LOG_ERR("Device detector config not found");
        return -EINVAL;
    }

    if (config->trackball_dev == NULL) {
        LOG_WRN("Trackball device not defined in device detector config");
        // 不是错误，只是警告
    }
    
    // 设置全局启用标志
    trackball_enabled = true;
    
    LOG_INF("Trackball device initialized successfully");
    return 0;
}

/**
 * @brief Deinitializes the trackball device.
 *
 * This function is called by the device_detector when a trackball is disconnected
 * or a different device is detected.
 *
 * @param dev_detector_dev Pointer to the device_detector device instance.
 * @return int 0 on success, or an error code on failure.
 */
int trackball_handler_deinit(const struct device *dev_detector_device) {
    LOG_INF("De-initializing trackball device");
    
    // 清除全局启用标志
    trackball_enabled = false;
    
    LOG_INF("Trackball device de-initialized successfully");
    return 0;
}

/**
 * @brief 检查轨迹球设备是否处于活动状态
 * 
 * 该函数供ZMK核心代码调用，用于决定是否处理轨迹球事件
 * 
 * @return bool 如果轨迹球已启用返回true，否则返回false
 */
bool zmk_trackball_is_active(void) {
    // 简单地返回全局状态变量
    return trackball_enabled;
} 