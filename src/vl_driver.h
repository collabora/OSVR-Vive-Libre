/*
 * OpenHMD
 *
 * Copyright (C) 2013 Fredrik Hultin
 * Copyright (C) 2013 Jakob Bornecrantz
 *
 * Vive Libre
 *
 * Copyright (C) 2016 Lubosz Sarnecki <lubosz.sarnecki@collabora.co.uk>
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

#include <cassert>
#include <cstdint>
#include <map>
#include <unistd.h>
#include <vector>

#include <libusb.h>

#include "vl_magic.h"
#include "vl_fusion.h"
#include "vl_messages.h"
#include "vl_light.h"
#include "vl_log.h"

#define FEATURE_BUFFER_SIZE 64

#define FREQ_48MHZ 1.0f / 48000000.0f

struct vl_device {
    libusb_device_handle* handle = nullptr;
    std::vector<int> interfaces;
    std::map<int, libusb_transfer*> transfers;
    std::map<int, std::array<uint8_t, FEATURE_BUFFER_SIZE>> transfer_buffers;
};

class vl_driver {
private:
    libusb_context* context;

public:
    vl_device hmd_device;
    vl_device hmd_lighthouse_device;
    vl_device watchman_dongle_device;
    uint32_t previous_ticks;
    std::unique_ptr<vl_fusion> sensor_fusion;
    vl_lighthouse_samples raw_light_samples = {};


    vl_driver();
    ~vl_driver();
    bool init_devices(unsigned index);
    bool open_devices(int idx);
    void add_fd(int fd, short events);
    void remove_fd(int fd);
    bool poll();
    void update_pose();

    void _update_pose(const vive_headset_imu_report &pkt);
};

static inline int hid_send_feature_report(libusb_device_handle* dev, uint16_t interface, std::vector<uint8_t> data) {
    // Currently not implemented and unused, see hidapi’s implementation.
    assert(data[0] != 0);

    uint16_t report_id = (3/*HID feature*/ << 8) | data[0];

    int ret = libusb_control_transfer(dev,
                                      LIBUSB_REQUEST_TYPE_CLASS | LIBUSB_RECIPIENT_INTERFACE | LIBUSB_ENDPOINT_OUT,
                                      0x09/*HID set_report*/,
                                      report_id,
                                      interface,
                                      data.data(), data.size(),
                                      1000/*timeout millis*/);
    if (ret < 0)
        return -1;

    return data.size();
}

typedef void(*capture_callback)(uint8_t*, int, vl_driver*);
struct vl_callback {
    vl_driver* driver;
    capture_callback func;
};

void vl_driver_log_hmd_mainboard(uint8_t* buffer, int size, vl_driver* driver);
void vl_driver_log_hmd_imu(uint8_t* buffer, int size, vl_driver* driver);
void vl_driver_log_watchman(uint8_t* buffer, int size, vl_driver* driver);
void vl_driver_log_hmd_light(uint8_t* buffer, int size, vl_driver* driver);
void vl_driver_update_pose(uint8_t* buffer, int size, vl_driver* driver);

bool vl_driver_start_hmd_mainboard_capture(vl_driver*, capture_callback);
bool vl_driver_stop_hmd_mainboard_capture(vl_driver*);
bool vl_driver_start_hmd_imu_capture(vl_driver*, capture_callback);
bool vl_driver_stop_hmd_imu_capture(vl_driver*);
bool vl_driver_start_watchman_capture(vl_driver*, capture_callback);
bool vl_driver_stop_watchman_capture(vl_driver*);
bool vl_driver_start_hmd_light_capture(vl_driver*, capture_callback);
bool vl_driver_stop_hmd_light_capture(vl_driver*);
