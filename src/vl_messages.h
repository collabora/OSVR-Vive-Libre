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

#include <stdint.h>
#include "hid-reports.h"

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

inline static bool vl_msg_decode_hmd_imu(vive_headset_imu_report* pkt, const unsigned char* buffer, int size)
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

inline static void vl_msg_print_hmd_imu(vive_headset_imu_report* pkt) {
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

        printf("time_ticks: %u\n", pkt->samples[i].time_ticks);
        printf("seq: %u\n", pkt->samples[i].seq);
        printf("\n");
    }
}

inline static bool vl_msg_decode_hmd_light(vive_headset_lighthouse_pulse_report2* pkt, const unsigned char* buffer, int size)
{
    if(size != 64){
        printf("invalid vive sensor packet size (expected 64 but got %d)\n", size);
        return false;
    }

    pkt->report_id = read8(&buffer);

    for(int j = 0; j < 9; j++){
        pkt->samples[j].sensor_id = read8(&buffer);
        pkt->samples[j].length = uread16(&buffer);
        pkt->samples[j].timestamp = uread32(&buffer);
    }

    return true;
}

inline static void vl_msg_print_hmd_light(vive_headset_lighthouse_pulse_report2* pkt) {
    printf("== controller light sample ==\n");
    printf("  report_id: %u\n", pkt->report_id);
    for(int i = 0; i < 9; i++){
        if (pkt->samples[i].timestamp == UINT32_MAX)
            continue;
        printf("     sensor_id[%d]: %u\n", i, pkt->samples[i].sensor_id);
        printf("      length[%d]: %d\n", i, pkt->samples[i].length);
        printf("      time[%d]: %u\n", i, pkt->samples[i].timestamp);
        printf("\n");
    }
}

inline static void vl_msg_print_hmd_light_csv(vive_headset_lighthouse_pulse_report2* pkt) {
    for(int i = 0; i < 9; i++){
        printf("%u, %u, %d\n", pkt->samples[i].timestamp, pkt->samples[i].sensor_id, pkt->samples[i].length);
    }
}

inline static bool vl_msg_decode_controller_light(vive_headset_lighthouse_pulse_report1* pkt, const unsigned char* buffer, int size)
{
    if(size != 58){
        printf("invalid vive sensor packet size (expected 58 but got %d)\n", size);
        return false;
    }

    pkt->report_id = read8(&buffer);

    for(int j = 0; j < 7; j++){
        pkt->samples[j].sensor_id = read8(&buffer);
        pkt->samples[j].type = read8(&buffer);
        pkt->samples[j].length = read16(&buffer);
        pkt->samples[j].time = uread32(&buffer);
    }
    pkt->padding = read8(&buffer);

    if (pkt->padding != 0) {
        fprintf(stderr, "Wrong padding data (expected 0 but got %d)\n", pkt->padding);
        return false;
    }

    return true;
}

inline static void vl_msg_print_controller_light(vive_headset_lighthouse_pulse_report1* pkt) {
    printf("== hmd light sample ==\n");
    printf("  report_id: %u\n", pkt->report_id);
    printf("        num | id | type | length | time\n");
    for(int i = 0; i < 7; i++){

        if (pkt->samples[i].type == 0xff)
            // This is not a continue because there is never any sample after that.
            break;

        // TODO: identify what these two types mean, and why they are different.
        if (pkt->samples[i].type != 0x00 && pkt->samples[i].type != 0xfe) {
            fprintf(stderr, "Unknown sensor type %d\n", pkt->samples[i].type);
            continue;
        }

        printf("          %d | %02hhu |  %02hhu  | % 6d | %u\n",
               i,
               pkt->samples[i].sensor_id,
               pkt->samples[i].type,
               pkt->samples[i].length,
               pkt->samples[i].time);
    }
    printf("padding: %u\n", pkt->padding);
}

inline static bool vl_msg_decode_watchman(vive_controller_report1* pkt, const unsigned char* buffer, int size)
{
    if(size != 30){
        printf("invalid vive sensor packet size (expected 30 but got %d)\n", size);
        return false;
    }

    pkt->report_id = buffer[0];
    pkt->message.time1 = buffer[1];
    pkt->message.type1 = buffer[2];
    pkt->message.time2 = buffer[3];
    pkt->message.type2 = buffer[4];
    memcpy(&pkt->message.unknown, &buffer[5], sizeof(pkt->message.unknown));

    return true;
}

inline static void vl_msg_print_watchman(vive_controller_report1 * pkt) {
    /*
    printf("vive watchman sample:\n");
    printf("  report_id: %u\n", pkt->report_id);
    printf("  time1: %u\n", pkt->time1);
    printf("  type1: %u\n", pkt->type1);
    printf("  time2: %u\n", pkt->time2);
    printf("  type2: %d\n", pkt->type2);
    */

    vive_controller_message* msg = &pkt->message;
    printf("type %d %d buttons\n", msg->type1, msg->type2);
}


/*
vive_controller_command_packet controller_command;
controller_command.report_id = 255;
controller_command.command = 0x8f;
controller_command.length = 7;
printf("vive_controller_haptic_pulse: %d\n", hret);
*/

