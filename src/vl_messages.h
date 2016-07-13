#pragma once

#include <stdint.h>

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
