/*
 * Vive Libre
 *
 * Copyright (C) 2016 Emmanuel Gil Peyrot <emmanuel.peyrot@collabora.co.uk>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the
 * Free Software Foundation, Inc., 51 Franklin Street, Suite 500,
 * Boston, MA 02110-1335, USA.
 */

#pragma once

#include <cstdint>

enum class vl_vendor : uint16_t {
    HTC = 0x0bb4,
    VALVE = 0x28de,
};

enum class vl_product : uint16_t {
    HMD = 0x2c87,
    WATCHMAN_DONGLE = 0x2101,
    LIGHTHOUSE_FPGA_RX = 0x2000,
    CONTROLLER = 0x2012,
};

enum class vl_report_id : uint8_t {
    HMD_MAINBOARD_STATUS = 0x03,
    HMD_POWER = 0x04,
    HMD_MAINBOARD_DEVICE_INFO = 0x04,
    HMD_FIRMWARE_VERSION = 0x05,

    HMD_IMU = 0x20,
    HMD_LIGHTHOUSE_PULSE1 = 0x21,

    CONTROLLER1 = 0x23,
    CONTROLLER2 = 0x24,
    HMD_LIGHTHOUSE_PULSE2 = 0x25,
    CONTROLLER_DISCONNECT = 0x26,

    CONTROLLER_COMMAND = 0xff,
};

enum class vl_report_type : uint16_t {
    HMD_POWER = 0x2978,
    HMD_MAINBOARD_DEVICE_INFO = 0x2987,
};

enum class vl_controller_button : uint8_t {
    TRIGGER = 0x01,
    TOUCH = 0x02,
    THUMB = 0x04,
    SYSTEM = 0x08,
    GRIP = 0x10,
    MENU = 0x20,
};

enum class vl_controller_command : uint8_t {
    POWEROFF = 0x9f,
};
