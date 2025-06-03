/*
 * Copyright (c) 2024 YkieWang
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zmk_driver_device_detector/trackball_handler.h>

// Placeholder for API from the actual badjeff/zmk-pmw3610-driver
// These would be declared in a header file from that driver, e.g., <zmk_pmw3610.h>
// extern int zmk_pmw3610_start_reporting(const struct device *pmw_dev);
// extern int zmk_pmw3610_stop_reporting(const struct device *pmw_dev);

LOG_MODULE_DECLARE(zmk_device_detector, CONFIG_ZMK_DEVICE_DETECTOR_LOG_LEVEL);

int zmk_device_detector_trackball_handler_init(const struct device *dev_detector_dev, const struct device *trackball_to_manage) {
    LOG_INF("Trackball Handler: Initializing for detector %s...", dev_detector_dev->name);
    int ret = 0;

    if (trackball_to_manage) {
        if (device_is_ready(trackball_to_manage)) {
            LOG_INF("Trackball Handler: Activating trackball device: %s (detector %s)", 
                        trackball_to_manage->name, dev_detector_dev->name);
            // ret = zmk_pmw3610_start_reporting(trackball_to_manage); // Placeholder call
            // if (ret != 0) {
            //     LOG_ERR("Failed to start trackball device %s: %d", trackball_to_manage->name, ret);
            //     return ret;
            // }
            LOG_WRN("Trackball Handler: Real 'zmk_pmw3610_start_reporting' for %s needs to be implemented/called.", trackball_to_manage->name);
        } else {
            LOG_WRN("Trackball Handler: Passed trackball device %s for detector %s is not ready.", 
                        trackball_to_manage->name, dev_detector_dev->name);
        }
    } else {
        LOG_INF("Trackball Handler: No specific trackball device instance provided by detector %s.", dev_detector_dev->name);
        // TODO: Handle generic trackball initialization if applicable.
        // This might involve Kconfig checks like IS_ENABLED(CONFIG_ZMK_PMW3610_DRIVER_ENABLE).
        LOG_WRN("Trackball Handler: Generic trackball activation (no specific instance) TBD.");
    }
    
    LOG_INF("Trackball Handler: Initialized for %s.", dev_detector_dev->name);
    return ret;
}

int zmk_device_detector_trackball_handler_deinit(const struct device *dev_detector_dev, const struct device *trackball_to_manage) {
    LOG_INF("Trackball Handler: De-initializing for detector %s...", dev_detector_dev->name);
    int ret = 0;

    if (trackball_to_manage) {
        LOG_INF("Trackball Handler: De-activating trackball device: %s (detector %s)", 
                    trackball_to_manage->name, dev_detector_dev->name);
        // ret = zmk_pmw3610_stop_reporting(trackball_to_manage); // Placeholder call
        // if (ret != 0) {
        //     LOG_ERR("Failed to stop trackball device %s: %d", trackball_to_manage->name, ret);
        // }
        LOG_WRN("Trackball Handler: Real 'zmk_pmw3610_stop_reporting' for %s needs to be implemented/called.", trackball_to_manage->name);
    } else {
        LOG_INF("Trackball Handler: No specific trackball device instance was managed by detector %s for de-init.", dev_detector_dev->name);
        // TODO: Handle generic trackball de-initialization if applicable.
        LOG_WRN("Trackball Handler: Generic trackball de-activation (no specific instance) TBD.");
    }
        
    LOG_INF("Trackball Handler: De-initialized for %s.", dev_detector_dev->name);
    return ret;
} 