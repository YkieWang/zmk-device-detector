/*
 * Copyright (c) 2024 YkieWang
 * SPDX-License-Identifier: MIT
 */

#ifndef ZMK_DRIVER_DEVICE_DETECTOR_TRACKBALL_HANDLER_H
#define ZMK_DRIVER_DEVICE_DETECTOR_TRACKBALL_HANDLER_H

#include <zephyr/device.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initializes the trackball handler.
 *
 * @param dev_detector_device Pointer to the main device_detector device instance.
 * @return 0 on success, negative error code otherwise.
 */
int trackball_handler_init(const struct device *dev_detector_device);

/**
 * @brief De-initializes the trackball handler.
 *
 * @param dev_detector_device Pointer to the main device_detector device instance.
 * @return 0 on success, negative error code otherwise.
 */
int trackball_handler_deinit(const struct device *dev_detector_device);

/**
 * @brief 检查轨迹球设备是否处于活动状态
 * 
 * 该函数供ZMK核心代码调用，用于决定是否处理轨迹球事件
 * 
 * @return bool 如果轨迹球已启用返回true，否则返回false
 */
bool zmk_trackball_is_active(void);

#ifdef __cplusplus
}
#endif

#endif /* ZMK_DRIVER_DEVICE_DETECTOR_TRACKBALL_HANDLER_H */ 