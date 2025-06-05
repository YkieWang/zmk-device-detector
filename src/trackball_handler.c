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
    if (!dev_detector_device) {
        LOG_ERR("Device detector device is NULL");
        return -EINVAL;
    }
    
    // 从设备配置中直接获取轨迹球设备指针
    const struct device_detector_config *config = dev_detector_device->config;
    const struct device *trackball_dev = config->controlled_trackball_dev;
    
    if (!trackball_dev) {
        LOG_ERR("No trackball device found in device config");
        return -ENODEV;
    }
    
    if (!device_is_ready(trackball_dev)) {
        LOG_ERR("Trackball device is not ready");
        return -ENODEV;
    }
    
    // 简化设备初始化，不使用电源管理API
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
    if (!dev_detector_device) {
        LOG_ERR("Device detector device is NULL");
        return -EINVAL;
    }
    
    // 从设备配置中直接获取轨迹球设备指针
    const struct device_detector_config *config = dev_detector_device->config;
    const struct device *trackball_dev = config->controlled_trackball_dev;
    
    if (!trackball_dev) {
        LOG_ERR("No trackball device found in device config");
        return -ENODEV;
    }
    
    // 简化设备关闭，不使用电源管理API
    LOG_INF("Trackball device de-initialized successfully");
    return 0;
} 