/*
 * Copyright (c) 2024 YkieWang
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/sensor.h> // Required for sensor_attr_set and sensor_value
#include <zmk_driver_device_detector/joystick_handler.h>

// Include the header for the integrated analog input sub-driver
#include <zmk/drivers/analog_input.h>

// Placeholder for API from the actual YkieWang/zmk-analog-input-driver
// These would be declared in a header file from that driver, e.g., <zmk_analog_input.h>
// extern int zmk_analog_input_start(const struct device *analog_input_dev);
// extern int zmk_analog_input_stop(const struct device *analog_input_dev);

// Reuse the main device_detector log module for now, or define a new one.
LOG_MODULE_DECLARE(zmk_device_detector, CONFIG_ZMK_DEVICE_DETECTOR_LOG_LEVEL);

int zmk_device_detector_joystick_handler_init(const struct device *dev_detector_dev, const struct device *joystick_to_manage) {
    LOG_INF("Joystick Handler: Initializing for detector %s...", dev_detector_dev->name);
    int ret = 0;

    if (joystick_to_manage) {
        if (device_is_ready(joystick_to_manage)) {
            LOG_INF("Joystick Handler: Activating joystick device: %s (detector %s)",
                        joystick_to_manage->name, dev_detector_dev->name);

            struct sensor_value attr_val;

            // Enable the analog input sensor
            attr_val.val1 = 1; // true
            attr_val.val2 = 0;
            ret = sensor_attr_set(joystick_to_manage, SENSOR_CHAN_ALL,
                                  (enum sensor_attribute)ANALOG_INPUT_ATTR_ENABLE, &attr_val);
            if (ret != 0) {
                LOG_ERR("Failed to enable joystick device %s: %d", joystick_to_manage->name, ret);
                return ret;
            }
            LOG_DBG("Joystick %s enabled.", joystick_to_manage->name);

            // Activate reporting for the analog input sensor
            attr_val.val1 = 1; // true
            attr_val.val2 = 0;
            ret = sensor_attr_set(joystick_to_manage, SENSOR_CHAN_ALL,
                                  (enum sensor_attribute)ANALOG_INPUT_ATTR_ACTIVE, &attr_val);
            if (ret != 0) {
                LOG_ERR("Failed to activate joystick device %s: %d", joystick_to_manage->name, ret);
                // Try to disable it back if activation failed
                attr_val.val1 = 0;
                sensor_attr_set(joystick_to_manage, SENSOR_CHAN_ALL, (enum sensor_attribute)ANALOG_INPUT_ATTR_ENABLE, &attr_val);
                return ret;
            }
            LOG_DBG("Joystick %s activated.", joystick_to_manage->name);

            LOG_INF("Joystick Handler: Successfully activated joystick device %s.", joystick_to_manage->name);
        } else {
            LOG_WRN("Joystick Handler: Passed joystick device %s for detector %s is not ready.",
                        joystick_to_manage->name, dev_detector_dev->name);
            // Not returning an error, as the device might become ready later.
            // The detector will continue to poll.
            return -ENODEV; // Or return 0 and let polling handle it? For now, signal an issue.
        }
    } else {
        LOG_INF("Joystick Handler: No specific joystick device instance provided by detector %s.", dev_detector_dev->name);
        // If ANALOG_INPUT is enabled but no specific device, this implies a configuration for a
        // system-wide joystick that isn't managed by a phandle from the detector.
        // This scenario is less likely with the current detector design but could be a future enhancement.
        if (IS_ENABLED(CONFIG_ANALOG_INPUT)) {
            LOG_WRN("Joystick Handler: Generic joystick activation (CONFIG_ANALOG_INPUT enabled but no specific instance) is not yet fully supported by this handler's logic.");
            // Here, one might try to find a default "zmk,analog-input" device if the driver supports it,
            // or rely on the analog_input driver to self-initialize if it's designed to do so globally.
            // For now, we assume joystick_to_manage is required.
        } else {
            LOG_WRN("Joystick Handler: CONFIG_ANALOG_INPUT is not enabled, cannot manage generic joystick.");
        }
    }

    LOG_INF("Joystick Handler: Initialized for %s.", dev_detector_dev->name);
    return ret;
}

int zmk_device_detector_joystick_handler_deinit(const struct device *dev_detector_dev, const struct device *joystick_to_manage) {
    LOG_INF("Joystick Handler: De-initializing for detector %s...", dev_detector_dev->name);
    int ret = 0;

    if (joystick_to_manage) {
        // We assume if we have joystick_to_manage, it was the one we tried to init.
        // Check readiness? Maybe not critical for deinit.
        LOG_INF("Joystick Handler: De-activating joystick device: %s (detector %s)",
                    joystick_to_manage->name, dev_detector_dev->name);

        struct sensor_value attr_val;

        // Deactivate reporting for the analog input sensor
        attr_val.val1 = 0; // false
        attr_val.val2 = 0;
        ret = sensor_attr_set(joystick_to_manage, SENSOR_CHAN_ALL,
                              (enum sensor_attribute)ANALOG_INPUT_ATTR_ACTIVE, &attr_val);
        if (ret != 0) {
            LOG_ERR("Failed to deactivate joystick device %s: %d. Proceeding with disable.", joystick_to_manage->name, ret);
            // Don't return immediately, try to disable anyway.
        } else {
            LOG_DBG("Joystick %s deactivated.", joystick_to_manage->name);
        }

        // Disable the analog input sensor
        attr_val.val1 = 0; // false
        attr_val.val2 = 0;
        int disable_ret = sensor_attr_set(joystick_to_manage, SENSOR_CHAN_ALL,
                                     (enum sensor_attribute)ANALOG_INPUT_ATTR_ENABLE, &attr_val);
        if (disable_ret != 0) {
            LOG_ERR("Failed to disable joystick device %s: %d", joystick_to_manage->name, disable_ret);
            if (ret == 0) ret = disable_ret; // Preserve original error if any
        } else {
            LOG_DBG("Joystick %s disabled.", joystick_to_manage->name);
        }
        LOG_INF("Joystick Handler: De-activation process completed for joystick device %s.", joystick_to_manage->name);

    } else {
        LOG_INF("Joystick Handler: No specific joystick device instance was managed by detector %s for de-init.", dev_detector_dev->name);
        if (IS_ENABLED(CONFIG_ANALOG_INPUT)) {
             LOG_WRN("Joystick Handler: Generic joystick de-activation TBD.");
        }
    }

    LOG_INF("Joystick Handler: De-initialized for %s.", dev_detector_dev->name);
    return ret;
} 