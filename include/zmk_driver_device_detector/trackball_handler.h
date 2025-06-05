/*
 * Copyright (c) 2024 YkieWang
 * SPDX-License-Identifier: MIT
 */

#ifndef ZMK_DRIVER_DEVICE_DETECTOR_TRACKBALL_HANDLER_H
#define ZMK_DRIVER_DEVICE_DETECTOR_TRACKBALL_HANDLER_H

#include <zephyr/device.h>

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

#ifdef __cplusplus
}
#endif

#endif /* ZMK_DRIVER_DEVICE_DETECTOR_TRACKBALL_HANDLER_H */ 