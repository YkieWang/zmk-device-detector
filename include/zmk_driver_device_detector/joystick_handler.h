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
 * @brief Initializes the joystick handler.
 *
 * @param dev_detector_dev Pointer to the main device_detector device instance.
 * @param joystick_to_manage Pointer to the specific joystick device to manage (can be NULL).
 * @return 0 on success, negative error code otherwise.
 */
int zmk_device_detector_joystick_handler_init(const struct device *dev_detector_dev, const struct device *joystick_to_manage);

/**
 * @brief De-initializes the joystick handler.
 *
 * @param dev_detector_dev Pointer to the main device_detector device instance.
 * @param joystick_to_manage Pointer to the specific joystick device that was managed (can be NULL).
 * @return 0 on success, negative error code otherwise.
 */
int zmk_device_detector_joystick_handler_deinit(const struct device *dev_detector_dev, const struct device *joystick_to_manage);

#ifdef __cplusplus
}
#endif 