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

#include <libusb.h>

#include "vl_driver.h"
#include "vl_math.h"
#include "vl_log.h"

vl_driver::vl_driver() {
    libusb_init(&context);
    previous_ticks = 0;
}

vl_driver::~vl_driver() {
    libusb_close(hmd_device.handle);
    libusb_close(hmd_lighthouse_device.handle);
    libusb_close(watchman_dongle_device.handle);
    libusb_exit(context);
}

bool vl_driver::init_devices(unsigned index) {
    bool success = open_devices(index);
    if (!success)
        vl_error("No connected Vive found (index %d).", index);
    return success;
}

static void print_device_info(libusb_device_handle* dev, const libusb_device_descriptor& desc) {
    uint8_t string[255];
    struct {
        uint8_t index;
        const char* descriptor;
    } info[] = {
        {desc.iManufacturer, "Manufacturer"},
        {desc.iProduct, "Product"},
        {desc.iSerialNumber, "Serial number"},
    };
    for (int i : {0, 1, 2}) {
        if (libusb_get_string_descriptor_ascii(dev, info[i].index, string, sizeof(string)) >= 0)
            vl_info("%s: %s", info[i].descriptor, string);
    }
}

static std::string _hid_to_unix_path(libusb_device* dev)
{
   // TODO: find a better way to obtain a constexpr length from a constant string.
   // strlen("/dev/bus/usb/000/000") == 20
   constexpr size_t path_len = 20;
   char path[path_len + 1];

   uint8_t bus = libusb_get_bus_number(dev);
   uint8_t addr = libusb_get_device_address(dev);

   sprintf(path, "/dev/bus/usb/%03d/%03d", bus, addr);
   return std::string(path);
}

static bool open_device_idx(libusb_device** devs, vl_device& device, uint16_t manufacturer, uint16_t product, int device_index)
{
    libusb_device* dev;
    int idx = 0, index = -1;
    while ((dev = devs[idx++])) {
        libusb_device_descriptor desc;
        libusb_device_handle* handle;

        int ret = libusb_get_device_descriptor(dev, &desc);
        if (ret < 0) {
            vl_error("Failed to get device descriptor for device %d.", idx);
            continue;
        }

        if (desc.idVendor != manufacturer || desc.idProduct != product)
            continue;

        ++index;
        vl_debug("Found device %04X:%04X %d/%d", manufacturer, product, index + 1,
                 device_index + 1);

        if (index != device_index) {
            vl_debug("Requested Vive %d but currently at %d, skipping.",
                     device_index, index);
            continue;
        }

        // The Vive only exposes a single configuration for each of the devices.
        assert(desc.bNumConfigurations == 1);

        struct libusb_config_descriptor *conf_desc;
        ret = libusb_get_config_descriptor(dev, 0, &conf_desc);
        if (ret != 0)
            continue;

        ret = libusb_open(dev, &handle);
        if (ret != LIBUSB_SUCCESS) {
            std::string path = _hid_to_unix_path(dev);
            vl_warn("Failed to open device %04X:%04X.\nIs another driver "
                    "running?  Do you have the correct udev rules in "
                    "place?", manufacturer, product);
            vl_warn("Try: sudo chmod 666 %s", path.c_str());
            continue;
        }

        print_device_info(handle, desc);

        uint8_t nb_interfaces = conf_desc->bNumInterfaces;
        std::vector<int> interfaces = {};
        vl_debug("nb_interfaces: %d", nb_interfaces);

        // Currently use all interfaces, even the ones we don’t use yet.
        for (int i = 0; i < nb_interfaces; ++i) {
            const struct libusb_interface *interface = &conf_desc->interface[i];

            // All Vive devices have only a single altsetting per interface.
            assert(interface->num_altsetting == 1);
            const struct libusb_interface_descriptor *altsetting = &interface->altsetting[0];
            int iface = altsetting->bInterfaceNumber;

            uint8_t string[128];
            vl_debug("  interface: %d", iface);
            if (libusb_get_string_descriptor_ascii(handle, altsetting->iInterface, string, 128) >= 0)
                vl_debug("    name: %s", string);

            for (int k = 0; k < altsetting->bNumEndpoints; ++k) {
                const struct libusb_endpoint_descriptor *endpoint = &altsetting->endpoint[k];
                vl_debug("    endpoint: 0x%x, interval: %d", endpoint->bEndpointAddress, endpoint->bInterval);
            }

            // In order to send and receive things on an interface we need to claim
            // it, and to unclaim it from the kernel first.
            if (libusb_kernel_driver_active(handle, iface) == 1) {
                ret = libusb_detach_kernel_driver(handle, iface);
                if (ret != LIBUSB_SUCCESS) {
                    vl_warn("Failed to unclaim interface %d for device %04X:%04X "
                            "from the kernel.", iface, manufacturer, product);
                    libusb_free_config_descriptor(conf_desc);
                    libusb_close(handle);
                    continue;
                }
            }

            ret = libusb_claim_interface(handle, iface);
            if (ret != LIBUSB_SUCCESS) {
                vl_warn("Failed to claim interface %d for device %04X:%04X.",
                        iface, manufacturer, product);
                libusb_free_config_descriptor(conf_desc);
                libusb_close(handle);
                continue;
            }

            interfaces.push_back(iface);
        }

        libusb_free_config_descriptor(conf_desc);

        device.handle = handle;
        device.interfaces = std::move(interfaces);

        return true;
    }

    return false;
}

bool vl_driver::open_devices(int idx)
{
    libusb_device** devs;

    int ret = libusb_get_device_list(context, &devs);
    if (ret < 0) {
        vl_error("Failed to enumerate USB devices, check your permissions.");
        return false;
    }

    // Open the HMD device
    bool success = open_device_idx(devs, hmd_device, HTC_ID, VIVE_HMD, idx);
    if (!success) {
        vl_error("No connected VIVE found.");
        return false;
    }

    // Open the lighthouse device
    success = open_device_idx(devs, hmd_lighthouse_device, VALVE_ID, VIVE_LIGHTHOUSE_FPGA_RX, idx);
    if (!success)
        return false;

    success = open_device_idx(devs, watchman_dongle_device, VALVE_ID, VIVE_WATCHMAN_DONGLE, idx);
    if (!success)
        return false;

    libusb_free_device_list(devs, 1);

    //hret = hid_send_feature_report(drv->hmd_device, vive_magic_enable_lighthouse, sizeof(vive_magic_enable_lighthouse));
    //vl_debug("enable lighthouse magic: %d\n", hret);

    sensor_fusion = std::make_unique<vl_fusion>();

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


void vl_driver_log_watchman(uint8_t* buffer, int size, vl_driver* driver) {
    (void)driver;

    if (buffer[0] != VL_MSG_WATCHMAN) {
        vl_warn("Called %s with a wrong buffer type (0x%02x).", __func__, buffer[0]);
        return;
    }

    vive_controller_report1 pkt = vive_controller_report1();
    vl_msg_decode_watchman(&pkt, buffer, size);
    vl_msg_print_watchman(&pkt);
}

void vl_driver_log_hmd_mainboard(unsigned char *buffer, int size, vl_driver* driver) {
    (void)driver;

    if (size != 64) {
        vl_warn("Called %s with a wrong buffer length (%d expected, %d got).", __func__, 64, size);
        return;
    }

    if (buffer[0] != VIVE_MAINBOARD_STATUS_REPORT_ID) {
        vl_warn("Called %s with a wrong buffer type (0x%02x).", __func__, buffer[0]);
        return;
    }

    vive_mainboard_status_report pkt;

    // TODO: use a proper decode function.
    memcpy(&pkt, buffer, size);
    //vl_msg_decode_hmd_mainboard(&pkt, buffer, size);
    //vl_msg_print_hmd_mainboard(&pkt);

    assert(pkt.len == 60);

    switch (pkt.proximity_change) {
    case NO_CHANGE:
        break;
    case DECREASE:
        vl_info("proximity decreased: %d", pkt.proximity);
        break;
    case INCREASE:
        vl_info("proximity increased: %d", pkt.proximity);
        break;
    }

    // TODO: add some state for those, and likely provide callbacks.
    vl_info("lens_separation: %d", pkt.lens_separation);
    vl_info("button: %d", pkt.button);
    vl_info("IPD: %4.1fmm", 1e-2 * pkt.ipd);
}

void vl_driver_log_hmd_imu(unsigned char *buffer, int size, vl_driver* driver) {
    (void)driver;

    if (buffer[0] != VL_MSG_HMD_IMU) {
        vl_warn("Called %s with a wrong buffer type (0x%02x).", __func__, buffer[0]);
        return;
    }

    vive_headset_imu_report pkt;
    vl_msg_decode_hmd_imu(&pkt, buffer, size);
    vl_msg_print_hmd_imu(&pkt);
}

void vl_driver_log_hmd_light(uint8_t* buffer, int size, vl_driver* driver) {
    (void)driver;

    if (buffer[0] == VL_MSG_HMD_LIGHT) {
        vive_headset_lighthouse_pulse_report2 pkt;
        vl_msg_decode_hmd_light(&pkt, buffer, size);
        //vl_msg_print_hmd_light(&pkt);
        vl_msg_print_hmd_light_csv(&pkt);
    } else if (buffer[0] == VL_MSG_CONTROLLER_LIGHT) {
        vive_headset_lighthouse_pulse_report1 pkt = vive_headset_lighthouse_pulse_report1();
        vl_msg_decode_controller_light(&pkt, buffer, size);
        vl_msg_print_controller_light(&pkt);
    } else {
        vl_warn("Called %s with a wrong buffer type (0x%02x).", __func__, buffer[0]);
    }
}

static void handle_transfer(libusb_transfer* transfer) {
    vl_callback* callback = reinterpret_cast<vl_callback*>(transfer->user_data);

    if (transfer->status == LIBUSB_TRANSFER_CANCELLED) {
        vl_debug("Transfer cancelled.");
        delete callback;
        libusb_free_transfer(transfer);
        return;
    }

    if (transfer->status != LIBUSB_TRANSFER_COMPLETED) {
        vl_error("Transfer had an issue: %d", transfer->status);
        delete callback;
        libusb_free_transfer(transfer);
        return;
    }

    vl_debug("Transfer complete of %d bytes!", transfer->actual_length);

    callback->func(transfer->buffer, transfer->actual_length, callback->driver);

    libusb_error ret = static_cast<libusb_error>(libusb_submit_transfer(transfer));
    if (ret != LIBUSB_SUCCESS) {
        vl_error("Failed to submit transfer: %s", libusb_strerror(ret));
        // TODO: notice the user of the error.
    }
}

static bool vl_driver_start_capture(vl_driver* driver, vl_device& dev, int endpoint, capture_callback func) {
    // 0 for no isochronous packet.
    libusb_transfer* transfer = libusb_alloc_transfer(0);
    if (!transfer) {
        vl_error("Failed to allocate memory for USB transfer");
        return false;
    }

    dev.transfer_buffers[endpoint] = {};
    uint8_t* buffer = dev.transfer_buffers[endpoint].data();
    int length = dev.transfer_buffers[endpoint].size();

    vl_callback* callback = new vl_callback();
    callback->driver = driver;
    callback->func = func;

    libusb_fill_interrupt_transfer(transfer, dev.handle, endpoint, buffer, length, handle_transfer, reinterpret_cast<void*>(callback), 0);

    libusb_error ret = static_cast<libusb_error>(libusb_submit_transfer(transfer));
    if (ret) {
        vl_error("Failed to submit transfer: %s", libusb_strerror(ret));
        dev.transfer_buffers.erase(endpoint);
        return false;
    }

    dev.transfers[endpoint] = transfer;

    return true;
}

// XXX: use that.
#if 0
static bool vl_driver_actually_stop_capture(vl_device& dev, int endpoint) {
    dev.transfer_buffers.erase(endpoint);
    dev.transfers.erase(endpoint);
}
#endif

static bool vl_driver_stop_capture(vl_device& dev, int endpoint) {
    libusb_transfer* transfer = dev.transfers[endpoint];
    libusb_cancel_transfer(transfer);
    return true;
}

bool vl_driver_start_hmd_mainboard_capture(vl_driver* driver, capture_callback fun) {
    return vl_driver_start_capture(driver, driver->hmd_device, 0x81, fun);
}

bool vl_driver_stop_hmd_mainboard_capture(vl_driver* driver) {
    return vl_driver_stop_capture(driver->hmd_device, 0x81);
}

bool vl_driver_start_hmd_imu_capture(vl_driver* driver, capture_callback fun) {
    return vl_driver_start_capture(driver, driver->hmd_lighthouse_device, 0x81, fun);
}

bool vl_driver_stop_hmd_imu_capture(vl_driver* driver) {
    return vl_driver_stop_capture(driver->hmd_lighthouse_device, 0x81);
}

bool vl_driver_start_watchman_capture(vl_driver* driver, capture_callback fun) {
    return vl_driver_start_capture(driver, driver->watchman_dongle_device, 0x81, fun);
}

bool vl_driver_stop_watchman_capture(vl_driver* driver) {
    return vl_driver_stop_capture(driver->watchman_dongle_device, 0x81);
}

bool vl_driver_start_hmd_light_capture(vl_driver* driver, capture_callback fun) {
    return vl_driver_start_capture(driver, driver->hmd_lighthouse_device, 0x82, fun);
}

bool vl_driver_stop_hmd_light_capture(vl_driver* driver) {
    return vl_driver_stop_capture(driver->hmd_lighthouse_device, 0x82);
}

bool vl_driver::poll() {
    libusb_error ret = static_cast<libusb_error>(libusb_handle_events(context));
    if (ret != LIBUSB_SUCCESS) {
        vl_debug("Failed to poll: %s", libusb_strerror(ret));
        return false;
    }

    return true;
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
    hid_query(hmd_lighthouse_device.handle, update_pose_fun);
}
