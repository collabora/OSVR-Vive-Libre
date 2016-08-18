#pragma once

#include <stdbool.h>
#include <hidapi.h>

#include <vector>
#include <Eigen/Geometry>

#include "vl_magic.h"
#include "vl_fusion.h"
#include "vl_messages.h"

#define FEATURE_BUFFER_SIZE 256

#define HTC_ID                   0x0bb4
#define VIVE_HMD                 0x2c87

#define VALVE_ID                 0x28de
#define VIVE_WATCHMAN_DONGLE     0x2101
#define VIVE_LIGHTHOUSE_FPGA_RX  0x2000

#define TICK_LEN 1000.0f // 1000 Hz ticks

#define FREQ_48KHZ 1.0 / 48000.0f

typedef struct {
    hid_device* hmd_device;
    hid_device* hmd_imu_device;
    hid_device* watchman_dongle_device;
    hid_device* hmd_light_sensor_device;
    uint32_t previous_ticks;
    vl_fusion sensor_fusion;
} vl_driver;

vl_driver* vl_driver_init();
void vl_driver_close(vl_driver* priv);
std::vector<int> vl_driver_get_device_paths(int vendor_id, int device_id);
vl_driver* vl_driver_open_device(int idx);

void vl_driver_log_watchman(hid_device *dev);
void vl_driver_log_hmd_light(hid_device *dev);
void vl_driver_log_hmd_imu(hid_device* dev);


Eigen::Quaternionf imu_to_pose(vl_driver* priv);
