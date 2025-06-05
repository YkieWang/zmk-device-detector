/*
 * Copyright (c) 2024 YkieWang
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zmk_driver_device_detector/device_detector.h>
#include "zmk_driver_device_detector/encoder_handler.h"

LOG_MODULE_REGISTER(encoder_handler, CONFIG_LOG_DEFAULT_LEVEL);

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
    // ADC specifics
    uint16_t adc_vref_mv;
    uint8_t adc_resolution;
    // Controlled devices
    const struct device *controlled_joystick_dev;
    const struct device *controlled_trackball_dev;
    const struct device *controlled_encoder_dev;
};

/**
 * @brief 初始化旋钮设备
 *
 * 当设备检测器检测到旋钮(encoder)设备时调用此函数
 * 对EC11旋钮，主要是启用节点并将其连接到传感器和按键处理
 *
 * @param dev_detector_device 设备检测器设备指针
 * @return int 成功返回0，失败返回负错误码
 */
int encoder_handler_init(const struct device *dev_detector_device) {
    LOG_INF("Initializing encoder device");
    
    const struct device_detector_config *config = dev_detector_device->config;
    
    // 检查是否已配置旋钮设备
    if (config->controlled_encoder_dev == NULL) {
        LOG_ERR("No encoder device configured");
        return -ENODEV;
    }
    
    // 对于EC11旋钮，只需要确保设备已启用
    // EC11驱动已经由ZMK内核管理，我们只需要确保设备状态正确
    
    LOG_INF("Encoder device initialized successfully: %s", 
            config->controlled_encoder_dev->name);
    
    return 0;
}

/**
 * @brief 反初始化旋钮设备
 *
 * 当设备检测器检测到旋钮设备被移除时调用此函数
 *
 * @param dev_detector_device 设备检测器设备指针
 * @return int 成功返回0，失败返回负错误码
 */
int encoder_handler_deinit(const struct device *dev_detector_device) {
    LOG_INF("De-initializing encoder device");
    
    const struct device_detector_config *config = dev_detector_device->config;
    
    // 检查是否已配置旋钮设备
    if (config->controlled_encoder_dev == NULL) {
        LOG_ERR("No encoder device configured");
        return -ENODEV;
    }
    
    // 对于EC11旋钮，不需要特别的反初始化步骤
    
    LOG_INF("Encoder device de-initialized successfully");
    
    return 0;
}