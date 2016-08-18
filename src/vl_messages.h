#pragma once

#include <stdint.h>

typedef enum
{
    VL_MSG_HMD_IMU = 32,
    VL_MSG_CONTROLLER_LIGHT = 33,
    VL_MSG_WATCHMAN = 35,
    VL_MSG_36 = 36,
    VL_MSG_HMD_LIGHT = 37,
} vl_message;

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

typedef struct
{
    int16_t acc[3];
    int16_t rot[3];
    uint32_t time_ticks;
    uint8_t seq;
} vl_imu_sample;

typedef struct
{
    uint8_t report_id;
    vl_imu_sample samples[3];
} vl_msg_hmd_imu;

inline static bool vl_msg_decode_hmd_imu(vl_msg_hmd_imu* pkt, const unsigned char* buffer, int size)
{
    if(size != 52){
        printf("invalid vive sensor packet size (expected 52 but got %d)\n", size);
        return false;
    }

    pkt->report_id = read8(&buffer);

    for(int j = 0; j < 3; j++){
        for(int i = 0; i < 3; i++)
            pkt->samples[j].acc[i] = read16(&buffer);

        for(int i = 0; i < 3; i++)
            pkt->samples[j].rot[i] = read16(&buffer);

        pkt->samples[j].time_ticks = uread32(&buffer);
        pkt->samples[j].seq = read8(&buffer);
    }

    return true;
}

inline static void vl_msg_print_hmd_imu(vl_msg_hmd_imu* pkt) {
    printf("== imu sample ==\n");
    printf("  report_id: %u\n", pkt->report_id);
    for(int i = 0; i < 3; i++){
        printf("    sample[%d]:\n", i);

        for(int j = 0; j < 3; j++){
            printf("      acc[%d]: %d\n", j, pkt->samples[i].acc[j]);
        }

        for(int j = 0; j < 3; j++){
            printf("      rot[%d]: %d\n", j, pkt->samples[i].rot[j]);
        }

        printf("time_ticks: %zd\n", pkt->samples[i].time_ticks);
        printf("seq: %u\n", pkt->samples[i].seq);
        printf("\n");
    }
}

typedef struct
{
    uint8_t sensor_id;
    uint16_t length;
    uint32_t time;
} vl_light_sample;

typedef struct
{
    uint8_t report_id;
    vl_light_sample samples[9];
} vl_msg_hmd_light;

inline static bool vl_msg_decode_hmd_light(vl_msg_hmd_light* pkt, const unsigned char* buffer, int size)
{
    if(size != 64){
        printf("invalid vive sensor packet size (expected 64 but got %d)\n", size);
        return false;
    }

    pkt->report_id = read8(&buffer);

    for(int j = 0; j < 9; j++){
        pkt->samples[j].sensor_id = read8(&buffer);
        pkt->samples[j].length = uread16(&buffer);
        pkt->samples[j].time = uread32(&buffer);
    }

    return true;
}

inline static void vl_msg_print_hmd_light(vl_msg_hmd_light* pkt) {
    printf("== hmd light sample ==\n");
    printf("  report_id: %u\n", pkt->report_id);
    for(int i = 0; i < 9; i++){
        if (pkt->samples[i].time == UINT32_MAX)
            continue;
        printf("     sensor_id[%d]: %u\n", i, pkt->samples[i].sensor_id);
        printf("      length[%d]: %d\n", i, pkt->samples[i].length);
        printf("      time[%d]: %zd\n", i, pkt->samples[i].time);
        printf("\n");
    }
}

inline static void vl_msg_print_hmd_light_csv(vl_msg_hmd_light* pkt) {
    for(int i = 0; i < 9; i++){
        printf("%zd, %u, %d\n", pkt->samples[i].time, pkt->samples[i].sensor_id, pkt->samples[i].length);
    }
}


typedef struct
{
    uint8_t report_id;
    vl_light_sample samples[7];
    uint8_t unknown;
} vl_msg_controller_light;

inline static bool vl_msg_decode_controller_light(vl_msg_controller_light* pkt, const unsigned char* buffer, int size)
{
    if(size != 58){
        printf("invalid vive sensor packet size (expected 58 but got %d)\n", size);
        return false;
    }

    pkt->report_id = read8(&buffer);

    for(int j = 0; j < 7; j++){
        pkt->samples[j].sensor_id = read8(&buffer);
        pkt->samples[j].length = read16(&buffer);
        pkt->samples[j].time = uread32(&buffer);
    }
    pkt->unknown = read8(&buffer);

    return true;
}

inline static void vl_msg_print_controller_light(vl_msg_controller_light* pkt) {
    printf("== controller light sample ==\n");
    printf("  report_id: %u\n", pkt->report_id);
    for(int i = 0; i < 7; i++){

        if (pkt->samples[i].time == UINT32_MAX)
            continue;

        printf("     sensor_id[%d]: %u\n", i, pkt->samples[i].sensor_id);
        printf("      length[%d]: %d\n", i, pkt->samples[i].length);
        printf("      time[%d]: %zd\n", i, pkt->samples[i].time);
        printf("\n");
    }
    printf("unknown: %u\n", pkt->unknown);
}

typedef struct
{
    uint8_t report_id;
    uint8_t time1;
    uint8_t type1;
    uint8_t time2;
    uint8_t type2;
    uint8_t pressed_buttons;
} vl_msg_watchman;

inline static bool vl_msg_decode_watchman(vl_msg_watchman* pkt, const unsigned char* buffer, int size)
{
    if(size != 30){
        printf("invalid vive sensor packet size (expected 30 but got %d)\n", size);
        return false;
    }

    pkt->report_id = buffer[0];
    pkt->time1 = buffer[1];
    pkt->type1 = buffer[2];
    pkt->time2 = buffer[3];
    pkt->type2 = buffer[4];
    pkt->pressed_buttons = buffer[5];

    return true;
}

inline static void vl_msg_print_watchman(vl_msg_watchman * pkt) {
    /*
    printf("vive watchman sample:\n");
    printf("  report_id: %u\n", pkt->report_id);
    printf("  time1: %u\n", pkt->time1);
    printf("  type1: %u\n", pkt->type1);
    printf("  time2: %u\n", pkt->time2);
    printf("  type2: %d\n", pkt->type2);
    */

    printf("type %d %d buttons: %d\n", pkt->type1, pkt->type2, pkt->pressed_buttons);
}

typedef struct
{
    uint8_t report_id;
    uint8_t command;
    uint8_t length;
} vl_msg_controller_command;


/*
vive_controller_command_packet controller_command;
controller_command.report_id = 255;
controller_command.command = 0x8f;
controller_command.length = 7;
printf("vive_controller_haptic_pulse: %d\n", hret);
*/

