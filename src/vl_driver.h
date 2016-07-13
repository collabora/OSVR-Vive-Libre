#ifndef VIVE_H
#define VIVE_H

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

#define TICK_LEN (1.0f / 1000.0f) // 1000 Hz ticks

#define FREQ_48KHZ 48000.0f

typedef struct {
    hid_device* hmd_handle;
    hid_device* imu_handle;
    hid_device* watchman_dongle_handle;
    hid_device* lighthouse_sensor_handle;
    fusion sensor_fusion;
    uint32_t previous_ticks;
} vive_priv;


bool vive_decode_imu_packet(vive_imu_packet* pkt, const unsigned char* buffer, int size);


void ohmd_sleep(double seconds);
bool vive_decode_imu_packet(vive_imu_packet* pkt, const unsigned char* buffer, int size);
bool vive_decode_watchman_packet(vive_watchman_packet* pkt, const unsigned char* buffer, int size);
bool vive_decode_lighthouse_packet(vive_lighthouse_packet* pkt, const unsigned char* buffer, int size);
bool vive_decode_controller_lighthouse_packet(vive_controller_lighthouse_packet* pkt, const unsigned char* buffer, int size);

void print_imu_packet(vive_imu_packet* pkt);
void print_watchman_packet(vive_watchman_packet * pkt);
void print_controller_lighthouse_packet(vive_controller_lighthouse_packet* pkt);
void print_lighthouse_packet(vive_lighthouse_packet* pkt);

void print_watchman_sensors(vive_priv* priv);
void print_hmd_light_sensors(vive_priv *priv);
void print_imu_sensors(vive_priv* priv);

Eigen::Quaternionf imu_to_pose(vive_priv* priv);


std::vector<int> vive_get_device_paths(int vendor_id, int device_id);
vive_priv* vive_open_device(int idx);

vive_priv* vive_init();
void vive_free(vive_priv* priv);

#endif
