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
#include "vl_hid_reports.h"
#include "vl_log.h"

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
        vl_error("invalid vive sensor packet size (expected 52 but got %d)", size);
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
    vl_info("== imu sample ==");
    vl_info("  report_id: %u", pkt->report_id);
    for(int i = 0; i < 3; i++){
        vl_info("    sample[%d]:", i);

        for(int j = 0; j < 3; j++){
            vl_info("      acc[%d]: %d", j, pkt->samples[i].acc[j]);
        }

        for(int j = 0; j < 3; j++){
            vl_info("      rot[%d]: %d", j, pkt->samples[i].rot[j]);
        }

        vl_info("time_ticks: %u", pkt->samples[i].time_ticks);
        vl_info("seq: %u\n", pkt->samples[i].seq);
    }
}

inline static bool vl_msg_decode_hmd_light(vive_headset_lighthouse_pulse_report2* pkt, const unsigned char* buffer, int size)
{
    if(size != 64){
        vl_error("invalid vive sensor packet size (expected 64 but got %d)", size);
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
    vl_info("== controller light sample ==");
    vl_info("  report_id: %u", pkt->report_id);
    for(int i = 0; i < 9; i++){
        if (pkt->samples[i].timestamp == UINT32_MAX)
            continue;
        vl_info("     sensor_id[%d]: %u", i, pkt->samples[i].sensor_id);
        vl_info("      length[%d]: %d", i, pkt->samples[i].length);
        vl_info("      time[%d]: %u\n", i, pkt->samples[i].timestamp);
    }
}

inline static void vl_msg_print_hmd_light_csv(vive_headset_lighthouse_pulse_report2* pkt) {
    for(int i = 0; i < 9; i++){
        vl_info("%u, %u, %d", pkt->samples[i].timestamp, pkt->samples[i].sensor_id, pkt->samples[i].length);
    }
}

inline static bool vl_msg_decode_controller_light(vive_headset_lighthouse_pulse_report1* pkt, const unsigned char* buffer, int size)
{
    if(size != 58){
        vl_error("invalid vive sensor packet size (expected 58 but got %d)", size);
        return false;
    }

    pkt->report_id = read8(&buffer);

    for(int j = 0; j < 7; j++){
        pkt->samples[j].sensor_id = read8(&buffer);
        pkt->samples[j].type = read8(&buffer);
        pkt->samples[j].length = read16(&buffer);
        pkt->samples[j].timestamp = uread32(&buffer);
    }
    pkt->padding = read8(&buffer);

    if (pkt->padding != 0) {
        vl_error("Wrong padding data (expected 0 but got %d)", pkt->padding);
        return false;
    }

    return true;
}

inline static void vl_msg_print_controller_light(vive_headset_lighthouse_pulse_report1* pkt) {
    vl_info("== hmd light sample ==");
    vl_info("  report_id: %u", pkt->report_id);
    vl_info("        num | id | type | length | time");
    for(int i = 0; i < 7; i++){

        if (pkt->samples[i].type == 0xff)
            // This is not a continue because there is never any sample after that.
            break;

        // TODO: identify what these two types mean, and why they are different.
        if (pkt->samples[i].type != 0x00 && pkt->samples[i].type != 0xfe) {
            vl_info("Unknown sensor type %d", pkt->samples[i].type);
            continue;
        }

        vl_info("          %d | %02hhu |  %02hhu  | % 6d | %u",
               i,
               pkt->samples[i].sensor_id,
               pkt->samples[i].type,
               pkt->samples[i].length,
               pkt->samples[i].timestamp);
    }
    vl_info("padding: %u", pkt->padding);
}

inline static bool vl_msg_decode_watchman(vive_controller_report1* pkt, const unsigned char* buffer, int size)
{
    if(size != 30){
        vl_error("invalid vive sensor packet size (expected 30 but got %d)", size);
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
    vl_info("vive watchman sample:");
    vl_info("  report_id: %u", pkt->report_id);
    vl_info("  time1: %u", pkt->time1);
    vl_info("  type1: %u", pkt->type1);
    vl_info("  time2: %u", pkt->time2);
    vl_info("  type2: %d", pkt->type2);
    */

    vive_controller_message* msg = &pkt->message;
    vl_info("type %d %d buttons", msg->type1, msg->type2);
}


/*
vive_controller_command_packet controller_command;
controller_command.report_id = 255;
controller_command.command = 0x8f;
controller_command.length = 7;
vl_info("vive_controller_haptic_pulse: %d", hret);
*/

