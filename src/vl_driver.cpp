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

#include <vector>
#include <map>

#include "vl_driver.h"
#include "vl_math.h"
#include "vl_log.h"

vl_driver::vl_driver() {
    hid_init();
    previous_ticks = 0;
}

vl_driver::~vl_driver() {
    delete(sensor_fusion);
    hid_close(hmd_device.handle);
    hid_close(hmd_imu_device.handle);
    hid_close(watchman_dongle_device.handle);
    hid_close(hmd_light_sensor_device.handle);
    hid_exit();
}

bool vl_driver::init_devices(unsigned index) {
    bool success = open_devices(index);
    if (!success)
        vl_error("No connected Vive found (index %d).", index);
    return success;
}

static void print_info_string(int (*fun)(hid_device*, wchar_t*, size_t), const char* what, hid_device* device)
{
    wchar_t wbuffer[512] = {0};
    char buffer[1024] = {0};

    int hret = fun(device, wbuffer, 511);

    if(hret == 0){
        wcstombs(buffer, wbuffer, sizeof(buffer));
        vl_info("%s: '%s'\n", what, buffer);
    }
}

void print_device_info(hid_device* dev) {
    print_info_string(hid_get_manufacturer_string, "Manufacturer", dev);
    print_info_string(hid_get_product_string , "Product", dev);
    print_info_string(hid_get_serial_number_string, "Serial Number", dev);
}


static char* _hid_to_unix_path(char* path)
{
       const int len = 4;
       char bus [len];
       char dev [len];
       char *result = new char[20 + 1];

       sprintf (bus, "%.*s\n", len, path);
       sprintf (dev, "%.*s\n", len, path + 5);

       sprintf (result, "/dev/bus/usb/%03d/%03d",
               (int)strtol(bus, NULL, 16),
               (int)strtol(dev, NULL, 16));
       return result;
}


static bool open_device_idx(vl_device& device, int manufacturer, int product, int iface, int iface_tot, int device_index)
{
    struct hid_device_info* devs = hid_enumerate(manufacturer, product);
    struct hid_device_info* cur_dev = devs;

    int idx = 0;
    int iface_cur = 0;
    hid_device* ret = NULL;

    if (!devs) {
        vl_error("No hid devices found.");
        return false;
    }

    // vl_debug("Opening %04x:%04x %d/%d", manufacturer, product, iface+1, iface_tot);

    while (cur_dev) {
        if(idx == device_index && iface == iface_cur) {
            ret = hid_open_path(cur_dev->path);

            if (ret == NULL) {
                char* path = _hid_to_unix_path(cur_dev->path);
                vl_warn("Opening failed. Is another driver running? Do you have the correct udev rules in place?");
                vl_warn("Try: sudo chmod 666 %s", path);
                free(path);
                hid_free_enumeration(devs);
                return false;
            }

        }
        cur_dev = cur_dev->next;
        iface_cur++;
        if(iface_cur >= iface_tot){
            idx++;
            iface_cur = 0;
        }
    }
    hid_free_enumeration(devs);

    if (!ret) {
        vl_error("Couldn’t find device %04d:%04d interface %d, check that it is plugged in.", manufacturer, product, iface);
        return false;
    }

    if(hid_set_nonblocking(ret, 1) == -1){
        vl_error("failed to set non-blocking on device.");
        return false;
    }

    // print_device_info(ret);

    device.handle = ret;
    return true;
}

bool vl_driver::open_devices(int idx)
{
    // Open the HMD device
    bool success = open_device_idx(hmd_device, HTC_ID, VIVE_HMD, 0, 1, idx);
    if (!success)
        return false;

    // Open the lighthouse device
    success = open_device_idx(hmd_imu_device, VALVE_ID, VIVE_LIGHTHOUSE_FPGA_RX, 0, 2, idx);
    if (!success)
        return false;

    success = open_device_idx(hmd_light_sensor_device, VALVE_ID, VIVE_LIGHTHOUSE_FPGA_RX, 1, 2, idx);
    if (!success)
        return false;

    success = open_device_idx(watchman_dongle_device, VALVE_ID, VIVE_WATCHMAN_DONGLE, 1, 2, idx);
    if (!success)
        return false;

    //hret = hid_send_feature_report(drv->hmd_device, vive_magic_enable_lighthouse, sizeof(vive_magic_enable_lighthouse));
    //vl_debug("enable lighthouse magic: %d\n", hret);

    sensor_fusion = new vl_fusion();

    return true;
}

#define VL_GRAVITY_EARTH 9.81
#define VL_POW_2_M13 4.0/32768.0 // pow(2, -13)
#define VL_POW_2_M12 8.0/32768.0 // pow(2, -12)
#define VL_ACCEL_FACTOR VL_GRAVITY_EARTH * VL_POW_2_M13

static Eigen::Vector3d vec3_from_accel(const __s16* smp)
{
    Eigen::Vector3d sample(smp[0], smp[1], smp[2]);
    return sample * VL_ACCEL_FACTOR;
}

static Eigen::Vector3d vec3_from_gyro(const __s16* smp)
{
    Eigen::Vector3d sample(smp[0], smp[1], smp[2]);
    return sample * VL_POW_2_M12; // 8/32768 = 2^-12
}


void _log_watchman(unsigned char *buffer, int size) {
    if (buffer[0] == VL_MSG_WATCHMAN) {
        vive_controller_report1 pkt = vive_controller_report1();
        vl_msg_decode_watchman(&pkt, buffer, size);
        vl_msg_print_watchman(&pkt);
    }
}

void _log_hmd_imu(unsigned char *buffer, int size) {
    if (buffer[0] == VL_MSG_HMD_IMU) {
        vive_headset_imu_report pkt;
        vl_msg_decode_hmd_imu(&pkt, buffer, size);
        vl_msg_print_hmd_imu(&pkt);
    }
}

void _log_hmd_light(unsigned char *buffer, int size) {
    if (buffer[0] == VL_MSG_HMD_LIGHT) {
        vive_headset_lighthouse_pulse_report2 pkt;
        vl_msg_decode_hmd_light(&pkt, buffer, size);
        //vl_msg_print_hmd_light(&pkt);
        vl_msg_print_hmd_light_csv(&pkt);
    } else if (buffer[0] == VL_MSG_CONTROLLER_LIGHT) {
        vive_headset_lighthouse_pulse_report1 pkt = vive_headset_lighthouse_pulse_report1();
        vl_msg_decode_controller_light(&pkt, buffer, size);
        vl_msg_print_controller_light(&pkt);
    }
}

void vl_driver_log_watchman(vl_device& dev) {
    hid_query(dev.handle, &_log_watchman);
}

void vl_driver_log_hmd_imu(vl_device& dev) {
    hid_query(dev.handle, &_log_hmd_imu);
}

void vl_driver_log_hmd_light(vl_device& dev) {
    hid_query(dev.handle, &_log_hmd_light);
}

static bool is_timestamp_valid(uint32_t t1, uint32_t t2) {
    return t1 != t2 && (
        (t1 < t2 && t2 - t1 > UINT32_MAX >> 2) ||
        (t1 > t2 && t1 - t2 < UINT32_MAX >> 2)
    );
}

static int get_lowest_index(uint8_t s0, uint8_t s1, uint8_t s2) {
    return (s0 == (uint8_t)(s1 + 2) ) ? 1
         : (s1 == (uint8_t)(s2 + 2) ) ? 2
         :                              0;
}

void vl_driver::_update_pose(const vive_headset_imu_report &pkt) {
    int li = get_lowest_index(
                pkt.samples[0].seq,
                pkt.samples[1].seq,
                pkt.samples[2].seq);

    for (int offset = 0; offset < 3; offset++) {
        int index = (li + offset) % 3;

        vive_headset_imu_sample sample = pkt.samples[index];

        if (previous_ticks == 0) {
            previous_ticks = sample.time_ticks;
            continue;
        }

        if (is_timestamp_valid(sample.time_ticks, previous_ticks)) {
            float dt = FREQ_48MHZ * (sample.time_ticks - previous_ticks);
            Eigen::Vector3d vec3_gyro = vec3_from_gyro(sample.rot);
            Eigen::Vector3d vec3_accel = vec3_from_accel(sample.acc);
            sensor_fusion->update(dt, vec3_gyro, vec3_accel);
            previous_ticks = pkt.samples[index].time_ticks;
        }
    }
}



void vl_driver::update_pose() {
    query_fun update_pose_fun = [this](unsigned char *buffer, int size) {
        if (buffer[0] == VL_MSG_HMD_IMU) {
            vive_headset_imu_report pkt;
            vl_msg_decode_hmd_imu(&pkt, buffer, size);
            this->_update_pose(pkt);
        }
    };
    hid_query(hmd_imu_device.handle, update_pose_fun);
}
