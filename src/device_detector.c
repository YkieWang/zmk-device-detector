/*
 * Copyright (c) 2024 YkieWang
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_device_detector

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/init.h>

// Include the module's header file (path relative to this C file or via include paths)
// Assuming an include path like "-Imodules/zmk-device-detector/include" is set by CMake
// or using the full path from the Zephyr base perspective.
// For ZMK modules, it's common to use the module name in the include path.
#include <zmk_driver_device_detector/device_detector.h>
#include <zmk_driver_device_detector/joystick_handler.h>
#include <zmk_driver_device_detector/trackball_handler.h>
#include <zmk_driver_device_detector/encoder_handler.h>

LOG_MODULE_REGISTER(zmk_device_detector, CONFIG_LOG_DEFAULT_LEVEL);

// Forward declaration for the enum-to-string helper function
const char *zd_device_type_to_str(enum detected_device_type type);

// Forward declarations for internal static handlers
static void internal_no_device_handler_init(const struct device *dev_detector_dev);
static void internal_no_device_handler_deinit(const struct device *dev_detector_dev);
static void internal_unknown_device_handler_init(const struct device *dev_detector_dev);
static void internal_unknown_device_handler_deinit(const struct device *dev_detector_dev);

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
    // ADC specific configuration that might be needed for adc_raw_to_millivolts
    int32_t adc_vref_mv; // ADC reference voltage in millivolts
    uint8_t adc_resolution; // ADC resolution in bits
    const struct device *controlled_encoder_dev; // Added for controlled encoder
    const struct device *controlled_joystick_dev;  // Added for joystick
    const struct device *controlled_trackball_dev; // Added for trackball
};

struct device_detector_data {
    struct k_work_delayable poll_work; // For periodic polling
    struct adc_sequence adc_seq;
    struct adc_channel_cfg adc_channel_cfg;
    uint8_t adc_buffer[2]; // Buffer for ADC result (e.g., 10-bit or 12-bit ADC)
    enum detected_device_type current_device;
    enum detected_device_type previous_device;
    int32_t last_voltage_mv; // Store the last read voltage for debugging or advanced logic
    const struct device *dev_parent_device; // Added for storing the device pointer
};

static enum detected_device_type determine_device_from_voltage(const struct device *dev, int32_t voltage_mv) {
    const struct device_detector_config *config = dev->config;

    if (voltage_mv <= config->voltage_none_max_mv) {
        return DEVICE_TYPE_NONE;
    } else if (voltage_mv >= config->voltage_joystick_min_mv && voltage_mv <= config->voltage_joystick_max_mv) {
        return DEVICE_TYPE_JOYSTICK;
    } else if (voltage_mv >= config->voltage_trackball_min_mv && voltage_mv <= config->voltage_trackball_max_mv) {
        return DEVICE_TYPE_TRACKBALL;
    } else if (voltage_mv >= config->voltage_encoder_min_mv && voltage_mv <= config->voltage_encoder_max_mv) {
        return DEVICE_TYPE_ENCODER;
    }
    return DEVICE_TYPE_UNKNOWN;
}

static void call_device_handler(const struct device *dev, enum detected_device_type device_type, bool is_init) {
    LOG_DBG("%s handler for device type %s (%d) for detector %s",
            is_init ? "Initializing" : "De-initializing",
            zd_device_type_to_str(device_type), device_type, dev->name);
    int ret = 0;
    const struct device_detector_config *config = dev->config;

    switch (device_type) {
    case DEVICE_TYPE_NONE:
        if (is_init) internal_no_device_handler_init(dev); else internal_no_device_handler_deinit(dev);
        break;
    case DEVICE_TYPE_JOYSTICK:
        if (is_init) ret = joystick_handler_init(dev); 
        else ret = joystick_handler_deinit(dev);
        break;
    case DEVICE_TYPE_TRACKBALL:
        if (is_init) ret = trackball_handler_init(dev); 
        else ret = trackball_handler_deinit(dev);
        break;
    case DEVICE_TYPE_ENCODER:
        if (is_init) ret = encoder_handler_init(dev); 
        else ret = encoder_handler_deinit(dev);
        break;
    case DEVICE_TYPE_UNKNOWN:
    default:
        if (is_init) internal_unknown_device_handler_init(dev); else internal_unknown_device_handler_deinit(dev);
        break;
    }
    if (ret != 0) {
        LOG_ERR("Handler for device type %s (%s) on detector %s failed with error %d",
                zd_device_type_to_str(device_type), is_init ? "init" : "deinit", dev->name, ret);
    }
}

static void device_detector_poll_work_handler(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct device_detector_data *data = CONTAINER_OF(dwork, struct device_detector_data, poll_work);
    // Correctly getting the device instance using the stored parent device pointer
    const struct device *dev = data->dev_parent_device;

    const struct device_detector_config *config = dev->config;
    int ret;

    // 1. Read ADC value
    ret = adc_read(config->adc_dev, &data->adc_seq);
    if (ret < 0) {
        LOG_ERR("ADC read failed on %s (channel %d): %d", config->adc_dev->name, config->adc_channel_id, ret);
        // Reschedule anyway, maybe it's a transient issue
        k_work_reschedule(&data->poll_work, K_MSEC(config->poll_interval_ms));
        return;
    }

    uint16_t raw_adc_value = sys_get_le16(data->adc_buffer); // Use Zephyr utility for endian safety

    // 2. Convert raw ADC value to millivolts
    int32_t voltage_mv = 0; // Initialize
    int32_t temp_raw_adc_value = (int32_t)raw_adc_value;
    
    // Reverting to standard 4-parameter adc_raw_to_millivolts.
    // This function returns the voltage in millivolts, or a negative error code.
    voltage_mv = adc_raw_to_millivolts(config->adc_vref_mv, ADC_GAIN_1, config->adc_resolution, &temp_raw_adc_value);

    if (voltage_mv < 0) { 
        LOG_ERR("adc_raw_to_millivolts failed (error: %d) for raw value %u", voltage_mv, raw_adc_value);
        // Error is already in voltage_mv, so it will be handled downstream.
    }
    data->last_voltage_mv = voltage_mv; // Store the read/converted voltage (or error code)

    LOG_DBG("ADC raw: %u, Voltage: %d mV", raw_adc_value, voltage_mv);

    // 3. Determine device type based on voltage
    enum detected_device_type new_device = DEVICE_TYPE_UNKNOWN;
    if (voltage_mv >= 0) { // Only if conversion was successful
        new_device = determine_device_from_voltage(dev, voltage_mv);
    }

    // 4. Update current_device
    if (new_device != data->current_device) {
        data->previous_device = data->current_device;
        data->current_device = new_device;
        LOG_INF("Device changed from %d (%s) to %d (%s) (Voltage: %d mV)", 
                data->previous_device, zd_device_type_to_str(data->previous_device),
                data->current_device, zd_device_type_to_str(data->current_device), 
                voltage_mv);

        // 5. Handle device switching
        // De-initialize previous device (if any and not NONE)
        call_device_handler(dev, data->previous_device, false); // deinit old

        // Initialize new device (if any and not NONE)
        call_device_handler(dev, data->current_device, true); // init new
    }

    // Reschedule the work item
    k_work_reschedule(&data->poll_work, K_MSEC(config->poll_interval_ms));
}

// Helper function to convert enum to string for logging (optional but useful)
const char *zd_device_type_to_str(enum detected_device_type type) {
    switch (type) {
    case DEVICE_TYPE_NONE: return "NONE";
    case DEVICE_TYPE_JOYSTICK: return "JOYSTICK";
    case DEVICE_TYPE_TRACKBALL: return "TRACKBALL";
    case DEVICE_TYPE_ENCODER: return "ENCODER";
    case DEVICE_TYPE_UNKNOWN: return "UNKNOWN";
    default: return "INVALID_TYPE";
    }
}

enum detected_device_type zmk_device_detector_get_active_device(const struct device *dev) {
    if (!dev) {
        LOG_WRN("Device pointer is NULL");
        return DEVICE_TYPE_UNKNOWN;
    }
    struct device_detector_data *data = dev->data;
    return data->current_device;
}

// Placeholder implementations for device specific handlers (these will now call the modularized functions)

// Internal handlers for "NONE" and "UNKNOWN" can remain static if not complex
static void internal_no_device_handler_init(const struct device *dev_detector_dev) { LOG_INF("No device connected or active."); }
static void internal_no_device_handler_deinit(const struct device *dev_detector_dev) { LOG_DBG("De-init no_device handler (nop)"); }
static void internal_unknown_device_handler_init(const struct device *dev_detector_dev) { LOG_WRN("Unknown device detected."); }
static void internal_unknown_device_handler_deinit(const struct device *dev_detector_dev) { LOG_WRN("Unknown device de-init."); }

static int device_detector_init(const struct device *dev) {
    const struct device_detector_config *config = dev->config;
    struct device_detector_data *data = dev->data;

    // Store the device pointer in the data structure
    data->dev_parent_device = dev; // Initialize the new field

    LOG_INF("Initializing ZMK Device Detector: %s", dev->name);

    data->current_device = DEVICE_TYPE_NONE;
    data->previous_device = DEVICE_TYPE_NONE;
    data->last_voltage_mv = -1; // Initialize last voltage

    if (!device_is_ready(config->adc_dev)) {
        LOG_ERR("ADC device %s not ready", config->adc_dev->name);
        return -ENODEV;
    }

    // 手动获取受控制的设备
    struct device_detector_config *mutable_config = (struct device_detector_config *)config;
    
    // 尝试获取轨迹球设备（假设我们知道设备名称）
    mutable_config->controlled_trackball_dev = device_get_binding("PMW3610_TRACKBALL");
    if (mutable_config->controlled_trackball_dev) {
        LOG_INF("Found trackball device: %s", mutable_config->controlled_trackball_dev->name);
    } else {
        LOG_WRN("Trackball device not found, will be unavailable");
    }
    
    // 尝试获取摇杆设备
    mutable_config->controlled_joystick_dev = device_get_binding("ANALOG_JOYSTICK");
    if (mutable_config->controlled_joystick_dev) {
        LOG_INF("Found joystick device: %s", mutable_config->controlled_joystick_dev->name);
    } else {
        LOG_WRN("Joystick device not found, will be unavailable");
    }
    
    // 尝试获取编码器设备
    mutable_config->controlled_encoder_dev = device_get_binding("ENCODER_1");
    if (mutable_config->controlled_encoder_dev) {
        LOG_INF("Found encoder device: %s", mutable_config->controlled_encoder_dev->name);
    } else {
        LOG_WRN("Encoder device not found, will be unavailable");
    }

    // Configure ADC channel
    data->adc_channel_cfg.gain = ADC_GAIN_1;
    data->adc_channel_cfg.reference = ADC_REF_INTERNAL;
    data->adc_channel_cfg.acquisition_time = ADC_ACQ_TIME_DEFAULT;
    data->adc_channel_cfg.channel_id = config->adc_channel_id;
    // adc_channel_setup returns 0 on success, negative on error.
    int err = adc_channel_setup(config->adc_dev, &data->adc_channel_cfg);
    if (err != 0) {
        LOG_ERR("Failed to setup ADC channel %d on %s (error: %d)", 
                  config->adc_channel_id, config->adc_dev->name, err);
        return err;
    }

    // Configure ADC sequence
    data->adc_seq.channels = BIT(config->adc_channel_id);
    data->adc_seq.buffer = data->adc_buffer;
    data->adc_seq.buffer_size = sizeof(data->adc_buffer);
    data->adc_seq.resolution = config->adc_resolution; // Use configured resolution
    // data->adc_seq.options = ...; // any specific options if needed
    data->adc_seq.calibrate = false; // No calibration for now

    // Initialize and start the polling work item
    k_work_init_delayable(&data->poll_work, device_detector_poll_work_handler);
    k_work_reschedule(&data->poll_work, K_MSEC(config->poll_interval_ms)); // Start polling

    LOG_INF("Device Detector %s initialized. Polling every %d ms on ADC channel %d of %s",
              dev->name, config->poll_interval_ms, config->adc_channel_id, config->adc_dev->name);

    return 0;
}

// Instantiate the driver
// This uses the new DEVICE_DT_INST_DEFINE macro
// We need to make sure Kconfig options properly set the number of instances, or handle single instance.
// For now, let's assume a single instance defined in DTS.

#define ZMK_DEVICE_DETECTOR_NODE DT_NODELABEL(zmk_device_detector_node)

#define ZMK_DEVICE_DETECTOR_INIT(n)\
    static struct device_detector_data data_##n = {\
        /* Initialized in device_detector_init */ \
    };\
    static const struct device_detector_config config_##n = {\
        /* 硬编码ADC设备为"ADC_0"，这在nice_nano上是正确的 */\
        .adc_dev = DEVICE_DT_GET_ONE(nordic_nrf_saadc),\
        /* 硬编码所有的属性值，这些值来自于设备树中的定义 */\
        .adc_channel_id = 0,\
        .poll_interval_ms = 500,\
        .voltage_none_max_mv = 50,\
        .voltage_joystick_min_mv = 51,\
        .voltage_joystick_max_mv = 800,\
        .voltage_trackball_min_mv = 801,\
        .voltage_trackball_max_mv = 1600,\
        .voltage_encoder_min_mv = 1601,\
        .voltage_encoder_max_mv = 2400,\
        .adc_vref_mv = 3300,\
        .adc_resolution = 10,\
        /* 简化受控设备指针的初始化 */\
        .controlled_encoder_dev = NULL,\
        .controlled_joystick_dev = NULL,\
        .controlled_trackball_dev = NULL\
    };\
    DEVICE_DT_INST_DEFINE(n, device_detector_init, NULL, &data_##n, &config_##n, POST_KERNEL,\
                          CONFIG_SENSOR_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(ZMK_DEVICE_DETECTOR_INIT) 