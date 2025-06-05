# CMakeLists.txt for ZMK Device Detector Module
#
# Copyright (c) 2024 YkieWang
# SPDX-License-Identifier: MIT

cmake_minimum_required(VERSION 3.20)

# Register the module with Zephyr. This defines a library target.
# The name of the target is specified in zephyr/module.yml (e.g., zmk-driver-device-detector)
zephyr_library()

# Add the module's own 'include' directory to its public interface.
# This allows sources within this module (and other modules that depend on it)
# to use <zmk_driver_device_detector/header.h> and for analog_input.c to find
# <zmk/drivers/analog_input.h> (assuming 'include' is the root for that path).
zephyr_library_include_directories(include src)

# Add source files for the library
zephyr_library_sources(
    src/device_detector.c
    src/joystick_handler.c
    src/trackball_handler.c
    src/encoder_handler.c
)

# Add source file for the integrated analog input (joystick) sub-driver
# This file's compilation is controlled by CONFIG_ANALOG_INPUT,
# which is enabled when CONFIG_ZMK_DEVICE_DETECTOR_HAS_JOYSTICK is true.
zephyr_library_sources_ifdef(CONFIG_ANALOG_INPUT src/analog_input_subdriver/analog_input.c)
zephyr_library_sources_ifdef(CONFIG_ZMK_DEVICE_DETECTOR_HAS_TRACKBALL src/pmw3610_subdriver/pmw3610.c)

# Add include directories for the library
# Public headers for this module are in include/zmk_driver_device_detector
# zephyr_library() by default adds "include" to the interface include directories.
# We also need to make sure that the analog_input_subdriver headers are found.
# The include path for analog_input.c (src/analog_input_subdriver/analog_input.c) itself
# would be something like #include <zmk_driver_device_detector/analog_input_subdriver/analog_input.h>
# The zephyr_library() already adds "include" to the target's private include dirs.
# If analog_input.c uses #include <drivers/analog_input.h> or similar, we might need more.
# For now, assume the structure will be handled by include paths in C files.
# The main library's public includes are in "include", which becomes zmk_driver_device_detector after install.
# The sub-driver's header will be in "include/zmk_driver_device_detector/analog_input_subdriver".
# Let's add the "include/zmk_driver_device_detector" for the sub-driver as well.
# However, zephyr_library() adds the "include" dir to the library's PUBLIC include path.
# So, #include "zmk_driver_device_detector/analog_input_subdriver/analog_input.h" should work from joystick_handler.c
# and #include "analog_input.h" from within analog_input_subdriver/analog_input.c if its own include path is set.
# The original analog_input CMakeLists used zephyr_include_directories(include), which is relative.
# For our structure, analog_input.c is in src/analog_input_subdriver
# and its header is in include/zmk_driver_device_detector/analog_input_subdriver.
# The `zephyr_library_sources()` paths are relative to the CMakeLists.txt file.
# The include paths for `analog_input.c` should be adjusted in the C file itself if it was using relative paths
# like `#include "../include/zmk/drivers/analog_input.h"`.
# The current setup should allow `analog_input.c` to use `#include <zmk_driver_device_detector/analog_input_subdriver/analog_input.h>`
# and `joystick_handler.c` to use the same.

# For Kconfig generated headers (if any)
# zephyr_library_compile_definitions(CONFIG_MY_MODULE_GENERATED_HEADER)
