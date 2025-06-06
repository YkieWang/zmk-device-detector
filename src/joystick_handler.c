/*
 * Copyright (c) 2024 YkieWang
 * SPDX-License-Identifier: MIT
 */
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zmk_driver_device_detector/device_detector.h>
#include <zmk_driver_device_detector/joystick_handler.h>

LOG_MODULE_REGISTER(joystick_handler, CONFIG_LOG_DEFAULT_LEVEL);

// 添加全局变量来跟踪摇杆的启用状态
static bool joystick_enabled = false;

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

    // 各种驱动指针
    const struct device *joystick_dev;
    const struct device *trackball_dev;
    const struct device *encoder_dev;
};

int joystick_handler_init(const struct device *dev_detector_device) {
    LOG_INF("Initializing joystick device");
    
    // 获取设备检测器配置
    const struct device_detector_config *config = dev_detector_device->config;
    if (config == NULL) {
        LOG_ERR("Device detector config not found");
        return -EINVAL;
    }

    if (config->joystick_dev == NULL) {
        LOG_WRN("Joystick device not defined in device detector config");
        // 不是错误，只是警告
    }
    
    // 设置全局启用标志
    joystick_enabled = true;
    
    LOG_INF("Joystick device initialized successfully");
    return 0;
}

int joystick_handler_deinit(const struct device *dev_detector_device) {
    LOG_INF("De-initializing joystick device");
    
    // 清除全局启用标志
    joystick_enabled = false;
    
    LOG_INF("Joystick device de-initialized successfully");
    return 0;
}

// 提供给ZMK核心的公共函数，用于检查摇杆是否活跃
bool zmk_joystick_is_active(void) {
    // 简单地返回全局状态变量
    return joystick_enabled;
}