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
 * @brief Initializes the encoder handler.
 *
 * @param dev_detector_dev Pointer to the main device_detector device instance.
 * @param encoder_to_manage Pointer to the specific EC11 device to manage (can be NULL).
 * @return 0 on success, negative error code otherwise.
 */
int zmk_device_detector_encoder_handler_init(const struct device *dev_detector_dev, const struct device *encoder_to_manage);

/**
 * @brief De-initializes the encoder handler.
 *
 * @param dev_detector_dev Pointer to the main device_detector device instance.
 * @param encoder_to_manage Pointer to the specific EC11 device that was managed (can be NULL).
 * @return 0 on success, negative error code otherwise.
 */
int zmk_device_detector_encoder_handler_deinit(const struct device *dev_detector_dev, const struct device *encoder_to_manage);

#ifdef __cplusplus
}
#endif 