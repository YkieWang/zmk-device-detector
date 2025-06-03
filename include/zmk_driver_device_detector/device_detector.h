/*
 * Copyright (c) 2024 YkieWang
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Represents the type of device detected.
 */
enum detected_device_type {
    DEVICE_TYPE_NONE = 0,
    DEVICE_TYPE_JOYSTICK,
    DEVICE_TYPE_TRACKBALL,
    DEVICE_TYPE_ENCODER,
    DEVICE_TYPE_UNKNOWN, // If voltage doesn't match any known range
};

/**
 * @brief Get the currently detected device type.
 *
 * @param dev Pointer to the device_detector device instance.
 * @return enum detected_device_type The type of device currently active.
 *         Returns DEVICE_TYPE_NONE if no device is detected or active.
 */
enum detected_device_type zmk_device_detector_get_active_device(const struct device *dev);

#ifdef __cplusplus
}
#endif 