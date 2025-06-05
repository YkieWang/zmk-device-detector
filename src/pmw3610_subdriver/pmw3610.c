/*
 * Copyright (c) 2022 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT pixart_pmw3610

#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/input/input.h>
#include <zephyr/pm/device.h>
#include <zmk/keymap.h>
#include <zmk/events/activity_state_changed.h>
#include "pmw3610.h" // This will correctly refer to pmw3610.h in the same directory

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(pmw3610, CONFIG_PMW3610_LOG_LEVEL);

//////// Sensor initialization steps definition //////////
// init is done in non-blocking manner (i.e., async), a //
// delayable work is defined for this purpose           //
enum pmw3610_init_step {
    ASYNC_INIT_STEP_POWER_UP,  // reset cs line and assert power-up reset
    ASYNC_INIT_STEP_CLEAR_OB1, // clear observation1 register for self-test check
    ASYNC_INIT_STEP_CHECK_OB1, // check the value of observation1 register after self-test check
    ASYNC_INIT_STEP_CONFIGURE, // set other registes like cpi and donwshift time (run, rest1, rest2)
                               // and clear motion registers

    ASYNC_INIT_STEP_COUNT // end flag
};

/* Timings (in ms) needed in between steps to allow each step finishes succussfully. */
// - Since MCU is not involved in the sensor init process, i is allowed to do other tasks.
//   Thus, k_sleep or delayed schedule can be used.
static const int32_t async_init_delay[ASYNC_INIT_STEP_COUNT] = {
    [ASYNC_INIT_STEP_POWER_UP] = 10 + CONFIG_PMW3610_INIT_POWER_UP_EXTRA_DELAY_MS, // >10ms needed
    [ASYNC_INIT_STEP_CLEAR_OB1] = 200, // 150 us required, test shows too short,
                                       // also power-up reset is added in this step, thus using 50 ms
    [ASYNC_INIT_STEP_CHECK_OB1] = 50,  // 10 ms required in spec,
                                       // test shows too short,
                                       // especially when integrated with display,
                                       // > 50ms is needed
    [ASYNC_INIT_STEP_CONFIGURE] = 0,
};

static int pmw3610_async_init_power_up(const struct device *dev);
static int pmw3610_async_init_clear_ob1(const struct device *dev);
static int pmw3610_async_init_check_ob1(const struct device *dev);
static int pmw3610_async_init_configure(const struct device *dev);

static int (*const async_init_fn[ASYNC_INIT_STEP_COUNT])(const struct device *dev) = {
    [ASYNC_INIT_STEP_POWER_UP] = pmw3610_async_init_power_up,
    [ASYNC_INIT_STEP_CLEAR_OB1] = pmw3610_async_init_clear_ob1,
    [ASYNC_INIT_STEP_CHECK_OB1] = pmw3610_async_init_check_ob1,
    [ASYNC_INIT_STEP_CONFIGURE] = pmw3610_async_init_configure,
};

//////// Function definitions //////////

static int pmw3610_read(const struct device *dev, uint8_t addr, uint8_t *value, uint8_t len) {
	const struct pixart_config *cfg = dev->config;
	const struct spi_buf tx_buf = { .buf = &addr, .len = sizeof(addr) };
	const struct spi_buf_set tx = { .buffers = &tx_buf, .count = 1 };
	struct spi_buf rx_buf[] = {
		{ .buf = NULL, .len = sizeof(addr), },
		{ .buf = value, .len = len, },
	};
	const struct spi_buf_set rx = { .buffers = rx_buf, .count = ARRAY_SIZE(rx_buf) };
	return spi_transceive_dt(&cfg->spi, &tx, &rx);
}

static int pmw3610_read_reg(const struct device *dev, uint8_t addr, uint8_t *value) {
	return pmw3610_read(dev, addr, value, 1);
}

static int pmw3610_write_reg(const struct device *dev, uint8_t addr, uint8_t value) {
	const struct pixart_config *cfg = dev->config;
	uint8_t write_buf[] = {addr | SPI_WRITE_BIT, value};
	const struct spi_buf tx_buf = { .buf = write_buf, .len = sizeof(write_buf), };
	const struct spi_buf_set tx = { .buffers = &tx_buf, .count = 1, };
	return spi_write_dt(&cfg->spi, &tx);
}

static int pmw3610_write(const struct device *dev, uint8_t reg, uint8_t val) {
	pmw3610_write_reg(dev, PMW3610_REG_SPI_CLK_ON_REQ, PMW3610_SPI_CLOCK_CMD_ENABLE);
	k_sleep(K_USEC(T_CLOCK_ON_DELAY_US));

    int err = pmw3610_write_reg(dev, reg, val);
    if (unlikely(err != 0)) {
        return err;
    }
    
    pmw3610_write_reg(dev, PMW3610_REG_SPI_CLK_ON_REQ, PMW3610_SPI_CLOCK_CMD_DISABLE);
    return 0;
}

static int pmw3610_set_cpi(const struct device *dev, uint32_t cpi) {
    /* Set resolution with CPI step of 200 cpi
     * 0x1: 200 cpi (minimum cpi)
     * 0x2: 400 cpi
     * 0x3: 600 cpi
     * :
     */

    if ((cpi > PMW3610_MAX_CPI) || (cpi < PMW3610_MIN_CPI)) {
        LOG_ERR("CPI value %u out of range", cpi);
        return -EINVAL;
    }

    // Convert CPI to register value
    uint8_t value = (cpi / 200);
    LOG_INF("Setting CPI to %u (reg value 0x%x)", cpi, value);

    /* set the cpi */
    uint8_t addr[] = {0x7F, PMW3610_REG_RES_STEP, 0x7F};
    uint8_t data[] = {0xFF, value, 0x00};

	pmw3610_write_reg(dev, PMW3610_REG_SPI_CLK_ON_REQ, PMW3610_SPI_CLOCK_CMD_ENABLE);
	k_sleep(K_USEC(T_CLOCK_ON_DELAY_US));

    /* Write data */
    int err;
    for (size_t i = 0; i < sizeof(data); i++) {
        err = pmw3610_write_reg(dev, addr[i], data[i]);
        if (err) {
            LOG_ERR("Burst write failed on SPI write (data)");
            break;
        }
    }
    pmw3610_write_reg(dev, PMW3610_REG_SPI_CLK_ON_REQ, PMW3610_SPI_CLOCK_CMD_DISABLE);

    if (err) {
        LOG_ERR("Failed to set CPI");
        return err;
    }

    return 0;
}

/* Set sampling rate in each mode (in ms) */
static int pmw3610_set_sample_time(const struct device *dev, uint8_t reg_addr, uint32_t sample_time) {
    uint32_t maxtime = 2550;
    uint32_t mintime = 10;
    if ((sample_time > maxtime) || (sample_time < mintime)) {
        LOG_WRN("Sample time %u out of range [%u, %u]", sample_time, mintime, maxtime);
        return -EINVAL;
    }

    uint8_t value = sample_time / mintime;
    LOG_INF("Set sample time to %u ms (reg value: 0x%x)", sample_time, value);

    /* The sample time is (reg_value * mintime ) ms. 0x00 is rounded to 0x1 */
    int err = pmw3610_write(dev, reg_addr, value);
    if (err) {
        LOG_ERR("Failed to change sample time");
    }

    return err;
}

/* Set downshift time in ms. */
// NOTE: The unit of run-mode downshift is related to pos mode rate, which is hard coded to be 4 ms
// The pos-mode rate is configured in pmw3610_async_init_configure
static int pmw3610_set_downshift_time(const struct device *dev, uint8_t reg_addr, uint32_t time) {
    uint32_t maxtime;
    uint32_t mintime;

    switch (reg_addr) {
    case PMW3610_REG_RUN_DOWNSHIFT:
        /*
         * Run downshift time = PMW3610_REG_RUN_DOWNSHIFT
         *                      * 8 * pos-rate (fixed to 4ms)
         */
        maxtime = 8160; // 32 * 255;
        mintime = 32; // hard-coded in pmw3610_async_init_configure
        break;

    case PMW3610_REG_REST1_DOWNSHIFT:
        /*
         * Rest1 downshift time = PMW3610_REG_RUN_DOWNSHIFT
         *                        * 16 * Rest1_sample_period (default 40 ms)
         */
        maxtime = 255 * 16 * CONFIG_PMW3610_REST1_SAMPLE_TIME_MS;
        mintime = 16 * CONFIG_PMW3610_REST1_SAMPLE_TIME_MS;
        break;

    case PMW3610_REG_REST2_DOWNSHIFT:
        /*
         * Rest2 downshift time = PMW3610_REG_REST2_DOWNSHIFT
         *                        * 128 * Rest2 rate (default 100 ms)
         */
        maxtime = 255 * 128 * CONFIG_PMW3610_REST2_SAMPLE_TIME_MS;
        mintime = 128 * CONFIG_PMW3610_REST2_SAMPLE_TIME_MS;
        break;

    default:
        LOG_ERR("Not supported");
        return -ENOTSUP;
    }

    // Check if the given time is in the range.
    if ((time > maxtime) || (time < mintime)) {
        LOG_WRN("Time %u is out of range [%u, %u]", time, mintime, maxtime);
        return -EINVAL;
    }

    // Calculate the register value
    uint32_t unit_time;
    if (reg_addr == PMW3610_REG_RUN_DOWNSHIFT) {
        unit_time = mintime;
    } else if (reg_addr == PMW3610_REG_REST1_DOWNSHIFT) {
        unit_time = 16 * CONFIG_PMW3610_REST1_SAMPLE_TIME_MS;
    } else {
        // reg_addr == PMW3610_REG_REST2_DOWNSHIFT
        unit_time = 128 * CONFIG_PMW3610_REST2_SAMPLE_TIME_MS;
    }

    // convert from time to reg_value
    uint8_t value = time / unit_time;
    LOG_INF("Set time to %u (reg_value 0x%x)", time, value);

    int err = pmw3610_write(dev, reg_addr, value);
    if (err) {
        LOG_ERR("Failed to change downshift time");
    }

    return err;
}

static int pmw3610_set_performance(const struct device *dev, bool enabled) {
    const struct pixart_config *config = dev->config;
    if (config->force_awake) {
        return 0;
    }

    uint8_t val;

    if (enabled) {
        val = 0x00; // Normal mode
    } else {
        val = 0xA5; // Shutdown mode (low power)
    }

    LOG_DBG("Set performance to %s", enabled ? "normal" : "shutdown");

    // writing 0xA5 to Shutdown register causes sensor to enter shutdown mode
    // writing anything else to Shutdown register causes sensor to enter normal mode
    return pmw3610_write(dev, PMW3610_REG_SHUTDOWN, val);
}

static int pmw3610_set_interrupt(const struct device *dev, const bool en) {
    const struct pixart_config *config = dev->config;
    // enable motion interrupt by writing 0x01 to register 0x11 (Performance)
    // the register is kept 0x00 by default
    uint8_t val = en ? 0x01 : 0x00;
    return pmw3610_write(dev, PMW3610_REG_PERFORMANCE, val);
}

static int pmw3610_async_init_power_up(const struct device *dev) {
    const struct pixart_config *config = dev->config;
    int err = 0;

    // Reset CS line
    err = gpio_pin_set_dt(&config->spi.cs.gpio, 0);
    if (err) {
        LOG_ERR("Failed to set CS pin (%d)", err);
        return err;
    }
    k_sleep(K_MSEC(1)); // Keep CS low for >100ns before asserting reset

    // Assert hardware reset
    err = pmw3610_write_reg(dev, PMW3610_REG_POWER_UP_RESET, PMW3610_POWERUP_CMD_RESET);
    if (err) {
        LOG_ERR("Failed to reset the sensor (%d)", err);
    }
    return err;
}

static int pmw3610_async_init_clear_ob1(const struct device *dev) {
    // Clear Observation1 register (0x2D) for self-test check
    return pmw3610_write(dev, PMW3610_REG_OBSERVATION, 0x00);
}

static int pmw3610_async_init_check_ob1(const struct device *dev) {
    uint8_t value;
    int err;

    // Read Observation1 register (0x2D)
    err = pmw3610_read_reg(dev, PMW3610_REG_OBSERVATION, &value);
    if (err) {
        LOG_ERR("Failed to read observation register (%d)", err);
        return err;
    }

    // Check self-test result (expect 0x0F)
    if (value == 0x0F) {
        LOG_INF("Observation register check OK (0x%02X)", value);
    } else {
        LOG_ERR("Observation register check FAILED (0x%02X), expected 0x0F", value);
        return -ENODEV; // Indicate device error
    }
    return 0;
}

static int pmw3610_async_init_configure(const struct device *dev) {
    uint8_t product_id;
    int err = 0;
    const struct pixart_config *config = dev->config;

    // Verify Product ID and Revision ID
    err = pmw3610_read_reg(dev, PMW3610_REG_PRODUCT_ID, &product_id);
    if (err) {
        LOG_ERR("Failed to read product ID");
        return err;
    }
    if (product_id != PMW3610_PRODUCT_ID) {
        LOG_ERR("Wrong product ID 0x%x", product_id);
        return -ENODEV;
    }
    LOG_INF("PMW3610 Product ID: 0x%x", product_id);

    // Set default CPI
    err = pmw3610_set_cpi(dev, config->cpi);
    if (err) {
        return err;
    }

    // Set other registers for operation, like downshift time and sample rate for different modes.
    // Run mode setting (Fixed to 4ms sample rate, downshift time is configured by user)
    err = pmw3610_write(dev, 0x1A, 0x04); // set pos mode rate to 4ms (fixed)
    if (err) return err;
    err = pmw3610_set_downshift_time(dev, PMW3610_REG_RUN_DOWNSHIFT, CONFIG_PMW3610_RUN_DOWNSHIFT_TIME_MS);
    if (err) return err;

    // Rest1 mode setting
    err = pmw3610_set_sample_time(dev, PMW3610_REG_REST1_RATE, CONFIG_PMW3610_REST1_SAMPLE_TIME_MS);
    if (err) return err;
    err = pmw3610_set_downshift_time(dev, PMW3610_REG_REST1_DOWNSHIFT, CONFIG_PMW3610_REST1_DOWNSHIFT_TIME_MS);
    if (err) return err;

    // Rest2 mode setting
    err = pmw3610_set_sample_time(dev, PMW3610_REG_REST2_RATE, CONFIG_PMW3610_REST2_SAMPLE_TIME_MS);
    if (err) return err;
    err = pmw3610_set_downshift_time(dev, PMW3610_REG_REST2_DOWNSHIFT, CONFIG_PMW3610_REST2_DOWNSHIFT_TIME_MS);
    if (err) return err;

    // Rest3 mode setting (Sample rate is fixed to 500ms)
    err = pmw3610_set_sample_time(dev, PMW3610_REG_REST3_RATE, CONFIG_PMW3610_REST3_SAMPLE_TIME_MS);
    if (err) return err;

    // Clear motion registers (MOTION, DELTA_X_L, DELTA_Y_L, DELTA_XY_H) by reading them
    uint8_t motion_data[4];
    err = pmw3610_read(dev, PMW3610_REG_MOTION, motion_data, sizeof(motion_data));
    if (err) {
        LOG_ERR("Failed to clear motion registers");
        return err;
    }

    // Enable motion interrupt (MOT bit in PERFORMANCE register)
    err = pmw3610_set_interrupt(dev, true);
    if (err) {
        LOG_ERR("Failed to enable motion interrupt");
        return err;
    }

    // enable smart algorithm for better surface coverage (optional)
    if (IS_ENABLED(CONFIG_PMW3610_SMART_ALGORITHM)) {
        // refer to the datasheet for details.
        pmw3610_write(dev, 0x25, 0x14);
        pmw3610_write(dev, 0x26, 0x14);
    }

    return 0;
}

static void pmw3610_async_init(struct k_work *work) {
    struct k_work_delayable *work2 = (struct k_work_delayable *)work;
    struct pixart_data *data = CONTAINER_OF(work2, struct pixart_data, init_work);
    const struct device *dev = data->dev;

    if (data->async_init_step >= ASYNC_INIT_STEP_COUNT) {
        LOG_ERR("PMW3610 init error: step out of bound.");
        data->ready = false; // Should not happen
        return;
    }

    LOG_DBG("PMW3610 async init step %d", data->async_init_step);
    data->err = async_init_fn[data->async_init_step](dev);

    if (data->err) {
        LOG_ERR("PMW3610 async init step %d failed with error %d", data->async_init_step, data->err);
        data->ready = false;
        // TODO: consider retry logic or permanent failure handling
        return;
    }

    data->async_init_step++;

    if (data->async_init_step < ASYNC_INIT_STEP_COUNT) {
        // Schedule next step
        k_work_schedule(&data->init_work, K_MSEC(async_init_delay[data->async_init_step]));
    } else {
        LOG_INF("PMW3610 initialized successfully");
        data->ready = true;
        // After successful initialization, set performance based on current activity state
        // This ensures the sensor is in the correct power mode (e.g., shutdown if system is idle)
        pmw3610_set_performance(dev, zmk_activity_get_state() == ZMK_ACTIVITY_ACTIVE);
    }
}

static int pmw3610_report_data(const struct device *dev) {
    struct pixart_data *data = dev->data;
    const struct pixart_config *config = dev->config;
    int err = 0;
    uint8_t motion_burst[PMW3610_BURST_SIZE]; // MOT, DX_L, DY_L, DX_H_DY_H, SQUAL, SHUTTER_H, SHUTTER_L
    int16_t dx = 0, dy = 0;

    if (!data->ready) {
        LOG_WRN("Sensor not ready for reporting data");
        return -EBUSY;
    }

    // Read motion burst data
    err = pmw3610_read_reg(dev, PMW3610_REG_MOTION_BURST, motion_burst);
    if (err) {
        LOG_ERR("Failed to read motion burst data: %d", err);
        return err;
    }

    // Check MOT bit (bit 7 of MOTION register)
    if (!(motion_burst[0] & BIT(7))) {
        // No motion detected
        return 0;
    }

    // Extract dx and dy (12-bit 2's complement)
    // DX = DX_H_DY_H<3:0> : DX_L<7:0>
    // DY = DX_H_DY_H<7:4> : DY_L<7:0>
    dx = (int16_t)sys_le16_to_cpu( ( ( (motion_burst[PMW3610_XY_H_POS] & 0x0F) << 8) | motion_burst[PMW3610_X_L_POS] ) );
    dy = (int16_t)sys_le16_to_cpu( ( ( (motion_burst[PMW3610_XY_H_POS] & 0xF0) << 4) | motion_burst[PMW3610_Y_L_POS] ) );

    // Sign extend from 12-bit to 16-bit
    if (dx & 0x0800) dx |= 0xF000;
    if (dy & 0x0800) dy |= 0xF000;

    if (IS_ENABLED(CONFIG_PMW3610_SWAP_XY)) {
        int16_t temp = dx;
        dx = dy;
        dy = temp;
    }

    if (IS_ENABLED(CONFIG_PMW3610_INVERT_X)) {
        dx = -dx;
    }

    if (IS_ENABLED(CONFIG_PMW3610_INVERT_Y)) {
        dy = -dy;
    }

    LOG_DBG("Raw dx=%d, dy=%d", dx, dy);

    // Report X movement
    if (dx != 0) {
        input_report_rel(dev, config->x_input_code, dx, false, K_NO_WAIT);
    }

    // Report Y movement
    if (dy != 0) {
        input_report_rel(dev, config->y_input_code, dy, false, K_NO_WAIT);
    }
    
    // Always report sync after X and Y if either moved, or if sync reporting is unconditional.
    // For PMW3610, it's common to always send SYNC if any motion was processed.
    if (dx != 0 || dy != 0) {
       input_report_sync(dev);
    }


#if IS_ENABLED(CONFIG_PMW3610_LOG_LEVEL_DBG)
    uint8_t squal = motion_burst[4]; // SQUAL is at index 4 (0-indexed) in a 7-byte burst
    uint16_t shutter = ((uint16_t)motion_burst[PMW3610_SHUTTER_H_POS] << 8) | motion_burst[PMW3610_SHUTTER_L_POS];
    LOG_DBG("SQUAL: 0x%02X, Shutter: 0x%04X", squal, shutter);
#endif

    return 0;
}

static void pmw3610_gpio_callback(const struct device *gpiob, struct gpio_callback *cb,
                                  uint32_t pins) {
    struct pixart_data *data = CONTAINER_OF(cb, struct pixart_data, irq_gpio_cb);
    const struct device *dev = data->dev;

    // Submit work to handle interrupt data (typically reading motion registers)
    k_work_submit(&data->trigger_work);
}

static void pmw3610_work_callback(struct k_work *work) {
    struct pixart_data *data = CONTAINER_OF(work, struct pixart_data, trigger_work);
    const struct device *dev = data->dev;

    pmw3610_report_data(dev);
}

static int pmw3610_init_irq(const struct device *dev) {
    int err;
    struct pixart_data *data = dev->data;
    const struct pixart_config *config = dev->config;

    if (!device_is_ready(config->irq_gpio.port)) {
        LOG_ERR("IRQ GPIO port not ready");
        return -ENODEV;
    }

    err = gpio_pin_configure_dt(&config->irq_gpio, GPIO_INPUT | GPIO_PULL_UP);
    if (err) {
        LOG_ERR("Failed to configure IRQ GPIO: %d", err);
        return err;
    }

    gpio_init_callback(&data->irq_gpio_cb, pmw3610_gpio_callback, BIT(config->irq_gpio.pin));
    err = gpio_add_callback(config->irq_gpio.port, &data->irq_gpio_cb);
    if (err) {
        LOG_ERR("Failed to add IRQ GPIO callback: %d", err);
        return err;
    }

    err = gpio_pin_interrupt_configure_dt(&config->irq_gpio, GPIO_INT_EDGE_FALLING);
    if (err) {
        LOG_ERR("Failed to configure IRQ interrupt: %d", err);
        return err;
    }

    return 0;
}

static int pmw3610_init(const struct device *dev) {
    struct pixart_data *data = dev->data;
    const struct pixart_config *config = dev->config;
    int err;

    LOG_INF("Initializing PMW3610");

    if (!spi_is_ready_dt(&config->spi)) {
        LOG_ERR("SPI bus is not ready");
        return -ENODEV;
    }

    data->dev = dev;
    k_work_init(&data->trigger_work, pmw3610_work_callback);
    k_work_init_delayable(&data->init_work, pmw3610_async_init);

    // Initialize and start the asynchronous initialization process
    data->async_init_step = ASYNC_INIT_STEP_POWER_UP;
    data->ready = false; // Mark as not ready until async init completes
    k_work_schedule(&data->init_work, K_MSEC(async_init_delay[data->async_init_step]));
    
    // Initialize IRQ GPIO
    err = pmw3610_init_irq(dev);
    if (err) {
        LOG_ERR("Failed to initialize IRQ: %d", err);
        // Even if IRQ fails, the sensor might still work in polled mode or if status is okay
        // but it's a significant issue. For now, let init proceed but log error.
    }
    
    // if force_awake is true, keep the sensor in normal mode always.
    if (config->force_awake) {
        LOG_INF("Sensor configured to be always awake.");
        // The actual set_performance will be called at the end of successful async_init.
    }


    LOG_INF("PMW3610 initialization sequence started.");
    return 0; // Async init will determine final readiness.
}


#if CONFIG_PM_DEVICE
static int pmw3610_pm_action(const struct device *dev, enum pm_device_action action) {
    int err = 0;
    const struct pixart_config *config = dev->config;

    if (config->force_awake) {
        return 0; // Do nothing if forced awake
    }

    switch (action) {
    case PM_DEVICE_ACTION_RESUME:
        LOG_INF("PMW3610 resume");
        err = pmw3610_set_performance(dev, true); // Wake up
        if (!err) {
            // After waking up, re-enable interrupts if they were configured
            err = pmw3610_set_interrupt(dev, true);
        }
        break;
    case PM_DEVICE_ACTION_SUSPEND:
        LOG_INF("PMW3610 suspend");
        err = pmw3610_set_performance(dev, false); // Shutdown
        break;
    case PM_DEVICE_ACTION_TURN_OFF:
         LOG_INF("PMW3610 turn off");
         err = pmw3610_write(dev, PMW3610_REG_SHUTDOWN, 0xB6); // Shutdown command for PMW3610
        break;
    default:
        return -ENOTSUP;
    }

    return err;
}
#endif

static int pmw3610_attr_set(const struct device *dev, enum sensor_channel chan,
                            enum sensor_attribute attr, const struct sensor_value *val) {
    // This driver uses custom attributes, not standard sensor_channel
    if (chan != SENSOR_CHAN_ALL) {
        return -ENOTSUP;
    }

    struct pixart_data *data = dev->data;
    if (!data->ready && attr != PMW3610_ATTR_CPI) { // CPI can be set before full init
        LOG_WRN("Sensor not ready for setting attributes (except CPI)");
       // return -EBUSY; // Allow CPI setting for initial configuration
    }


    switch ((enum pmw3610_attribute)attr) {
    case PMW3610_ATTR_CPI:
        return pmw3610_set_cpi(dev, PMW3610_SVALUE_TO_CPI(*val));
    case PMW3610_ATTR_RUN_DOWNSHIFT_TIME:
        return pmw3610_set_downshift_time(dev, PMW3610_REG_RUN_DOWNSHIFT, PMW3610_SVALUE_TO_TIME(*val));
    case PMW3610_ATTR_REST1_DOWNSHIFT_TIME:
        return pmw3610_set_downshift_time(dev, PMW3610_REG_REST1_DOWNSHIFT, PMW3610_SVALUE_TO_TIME(*val));
    case PMW3610_ATTR_REST2_DOWNSHIFT_TIME:
        return pmw3610_set_downshift_time(dev, PMW3610_REG_REST2_DOWNSHIFT, PMW3610_SVALUE_TO_TIME(*val));
    case PMW3610_ATTR_REST1_SAMPLE_TIME:
        return pmw3610_set_sample_time(dev, PMW3610_REG_REST1_RATE, PMW3610_SVALUE_TO_TIME(*val));
    case PMW3610_ATTR_REST2_SAMPLE_TIME:
        return pmw3610_set_sample_time(dev, PMW3610_REG_REST2_RATE, PMW3610_SVALUE_TO_TIME(*val));
    case PMW3610_ATTR_REST3_SAMPLE_TIME:
        return pmw3610_set_sample_time(dev, PMW3610_REG_REST3_RATE, PMW3610_SVALUE_TO_TIME(*val));
    default:
        LOG_ERR("Unknown attribute %d", attr);
        return -ENOTSUP;
    }

    return 0;
}

static const struct sensor_driver_api pmw3610_driver_api = {
    .attr_set = pmw3610_attr_set,
    // No sample_fetch or channel_get for this type of sensor directly, data is event-driven via input subsystem
};


#define PMW3610_INIT(n)                                                                                \
    static struct pixart_data pmw3610_data_##n;                                                        \
    static const struct pixart_config pmw3610_config_##n = {                                           \
        .spi = SPI_DT_SPEC_INST_GET(                                                                   \
            n, SPI_OP_MODE_MASTER | SPI_MODE_CPOL | SPI_MODE_CPHA | SPI_WORD_SET(8), 0),               \
        .irq_gpio = GPIO_DT_SPEC_INST_GET(n, irq_gpios),                                               \
        .cpi = DT_INST_PROP_OR(n, cpi, 600),                                                       \
        .evt_type = DT_INST_PROP(n, evt_type),                                                         \
        .x_input_code = DT_INST_PROP(n, x_input_code),                                                 \
        .y_input_code = DT_INST_PROP(n, y_input_code),                                                 \
        .force_awake = DT_INST_PROP(n, force_awake),                                                   \
    };                                                                                                 \
                                                                                                       \
    PM_DEVICE_DT_INST_DEFINE(n, pmw3610_pm_action);                                                    \
                                                                                                       \
    DEVICE_DT_INST_DEFINE(n, pmw3610_init, PM_DEVICE_DT_INST_GET(n),                                   \
                          &pmw3610_data_##n, &pmw3610_config_##n,                                      \
                          POST_KERNEL, CONFIG_INPUT_PMW3610_INIT_PRIORITY,                             \
                          &pmw3610_driver_api);

DT_INST_FOREACH_STATUS_OKAY(PMW3610_INIT)


#if IS_ENABLED(CONFIG_ZMK_ACTIVITY_MONITOR)
// Handle activity state changes to control sensor power state
static const struct device *pmw3610_devs[] = {
    DT_INST_FOREACH_STATUS_OKAY_SEP(PMW3610_INIT_DEV_GET, (,))
};

static int on_activity_state(const zmk_event_t *eh) {
    struct zmk_activity_state_changed *state_ev = as_zmk_activity_state_changed(eh);
    bool active = (state_ev->state == ZMK_ACTIVITY_ACTIVE);

    for (int i = 0; i < ARRAY_SIZE(pmw3610_devs); i++) {
        const struct device *dev = pmw3610_devs[i];
        if (device_is_ready(dev)) {
             // pm_device_action_run(dev, active ? PM_DEVICE_ACTION_RESUME : PM_DEVICE_ACTION_SUSPEND);
             // The pm_device_action_run seems to be handled by the Zephyr PM subsystem automatically
             // based on system idle. However, we can explicitly call set_performance.
             const struct pixart_config *config = dev->config;
             if (!config->force_awake) {
                pmw3610_set_performance(dev, active);
             }
        }
    }
    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(pmw3610_activity_listener, on_activity_state)
ZMK_SUBSCRIPTION(pmw3610_activity_listener, zmk_activity_state_changed);
#endif

