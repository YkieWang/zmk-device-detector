/*
 * Copyright (c) 2024 YkieWang
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <zephyr/device.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化旋钮设备
 *
 * 当设备检测器检测到旋钮(encoder)设备时调用此函数
 *
 * @param dev_detector_device 设备检测器设备指针
 * @return int 成功返回0，失败返回负错误码
 */
int encoder_handler_init(const struct device *dev_detector_device);

/**
 * @brief 反初始化旋钮设备
 *
 * 当设备检测器检测到旋钮设备被移除时调用此函数
 *
 * @param dev_detector_device 设备检测器设备指针
 * @return int 成功返回0，失败返回负错误码
 */
int encoder_handler_deinit(const struct device *dev_detector_device);

/**
 * @brief 检查旋钮设备是否处于活动状态
 * 
 * 该函数供ZMK核心代码调用，用于决定是否处理旋钮事件
 * 
 * @return bool 如果旋钮已启用返回true，否则返回false
 */
bool zmk_encoder_is_active(void);

#ifdef __cplusplus
}
#endif 