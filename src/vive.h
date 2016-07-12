#ifndef VIVE_H
#define VIVE_H

#include <stdint.h>
#include <stdbool.h>
#include <hidapi.h>

#include "magic.h"
#include "fusion.h"

#include <vector>

#include <Eigen/Geometry>


#define FEATURE_BUFFER_SIZE 256

#define HTC_ID                   0x0bb4
#define VIVE_HMD                 0x2c87

#define VALVE_ID                 0x28de
#define VIVE_WATCHMAN_DONGLE     0x2101
#define VIVE_LIGHTHOUSE_FPGA_RX  0x2000

#define TICK_LEN (1.0f / 1000.0f) // 1000 Hz ticks

#define FREQ_48KHZ 48000.0f

typedef enum
{
	VIVE_IRQ_SENSORS = 32,
} vive_irq_cmd;

typedef struct 
{
	int16_t acc[3];
	int16_t rot[3];
    uint32_t	time_ticks;
	uint8_t seq;
} vive_sensor_sample;

typedef struct
{
    uint8_t report_id;
    vive_sensor_sample samples[3];
} vive_imu_packet;


typedef struct
{
    uint8_t sensor_id;
    uint16_t length;
    uint32_t time;
} vive_lighthouse_sample;

typedef struct
{
    uint8_t report_id;
    vive_lighthouse_sample samples[9];
} vive_lighthouse_packet;

typedef struct
{
    uint8_t report_id;
    vive_lighthouse_sample samples[7];
    uint8_t unknown;
} vive_controller_lighthouse_packet;

typedef struct
{
    uint8_t report_id;
    uint8_t time1;
    uint8_t type1;
    uint8_t time2;
    uint8_t type2;
    uint8_t pressed_buttons;
} vive_watchman_packet;

typedef struct
{
    uint8_t report_id;
    uint8_t command;
    uint8_t length;
} vive_controller_command_packet;

typedef struct {
    hid_device* hmd_handle;
    hid_device* imu_handle;
    hid_device* watchman_dongle_handle;
    hid_device* lighthouse_sensor_handle;
    fusion sensor_fusion;
    vec3f raw_accel, raw_gyro;
    uint32_t previous_ticks;
} vive_priv;


inline static uint8_t read8(const unsigned char** buffer)
{
    uint8_t ret = **buffer;
    *buffer += 1;
    return ret;

}

inline static int16_t read16(const unsigned char** buffer)
{
    int16_t ret = **buffer | (*(*buffer + 1) << 8);
    *buffer += 2;
    return ret;
}

inline static int32_t read32(const unsigned char** buffer)
{
    int32_t ret = **buffer | (*(*buffer + 1) << 8) | (*(*buffer + 1) << 16) | (*(*buffer + 1) << 24);
    *buffer += 4;
    return ret;
}

inline static uint16_t uread16(const unsigned char** buffer)
{
    const uint8_t *val = *buffer;
    uint16_t ret = val[0] | (val[1] << 8);
    *buffer += 2;
    return ret;
}

inline static uint32_t uread32(const unsigned char** buffer)
{
    const uint8_t *val = *buffer;
    uint32_t ret = val[0] | (val[1] << 8) | (val[2] << 16) | (val[3] << 24);

    *buffer += 4;
    return ret;
}

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
