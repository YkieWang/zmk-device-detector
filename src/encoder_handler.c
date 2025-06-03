/*
 * Copyright (c) 2024 YkieWang
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/devicetree.h> // Required for DT_HAS_PROP, DT_PROP, DT_PHANDLE, etc.
#include <zmk_driver_device_detector/encoder_handler.h>
// May need: #include <zephyr/drivers/sensor.h> if interacting with EC11 API directly

// Define DT_DRV_COMPAT if we need to get properties from the zmk,device-detector node itself.
// However, the 'dev_detector_dev' passed to these functions is the zmk,device-detector instance.
#define DEVICE_DETECTOR_NODE DT_DRV_COMPAT // This will be zmk_device_detector

LOG_MODULE_DECLARE(zmk_device_detector, CONFIG_ZMK_DEVICE_DETECTOR_LOG_LEVEL);

// No longer need a file-static variable for controlled_ec11_dev
// static const struct device *controlled_ec11_dev = NULL;

int zmk_device_detector_encoder_handler_init(const struct device *dev_detector_dev, const struct device *encoder_to_manage) {
    LOG_INF("Encoder Handler: Initializing for detector %s...", dev_detector_dev->name);

    if (encoder_to_manage) {
        if (device_is_ready(encoder_to_manage)) {
            LOG_INF("Encoder Handler: Managing EC11 sensor: %s (detector %s)", 
                        encoder_to_manage->name, dev_detector_dev->name);
            // TODO: Add any specific ZMK EC11 activation logic if needed for encoder_to_manage.
        } else {
            // This case should ideally be handled before calling, or a non-ready device passed explicitly.
            LOG_WRN("Encoder Handler: Passed EC11 sensor %s for detector %s is not ready.", 
                        encoder_to_manage->name, dev_detector_dev->name); 
             // Continue, but specific control won't be possible.
        }
    } else {
        LOG_INF("Encoder Handler: No specific EC11 sensor passed to manage for detector %s.", dev_detector_dev->name);
    }

    if (IS_ENABLED(CONFIG_EC11)) {
        LOG_DBG("Encoder Handler: EC11 Kconfig is enabled.");
    } else {
        LOG_WRN("Encoder Handler: EC11 Kconfig is NOT enabled. Encoder functionality will be limited for %s.", dev_detector_dev->name);
        return -ENODEV; 
    }

    LOG_INF("Encoder Handler: Initialized for %s.", dev_detector_dev->name);
    return 0;
}

int zmk_device_detector_encoder_handler_deinit(const struct device *dev_detector_dev, const struct device *encoder_to_manage) {
    LOG_INF("Encoder Handler: De-initializing for detector %s...", dev_detector_dev->name);
    
    if (encoder_to_manage) {
        LOG_INF("Encoder Handler: Releasing management of EC11 sensor: %s (detector %s)", 
                    encoder_to_manage->name, dev_detector_dev->name);
        // TODO: Add any specific ZMK EC11 deactivation logic if needed for encoder_to_manage.
    }
    // No other specific deinit for the global EC11 Kconfig status needed here.
    
    LOG_INF("Encoder Handler: De-initialized for %s.", dev_detector_dev->name);
    return 0;
} 