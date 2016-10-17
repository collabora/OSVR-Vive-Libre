/*
 * HTC Vive USB HID reports
 * Copyright 2016 Philipp Zabel
 * SPDX-License-Identifier:	LGPL-2.0+
 */

#pragma once

#include <stdint.h>

#include <asm/byteorder.h>

enum vive_proximity_change : __u8 {
    NO_CHANGE = 0,
    DECREASE = 1,
    INCREASE = 2,
};

struct vive_mainboard_status_report {
	__u8 id;
	__le16 unknown;
	__u8 len;
	__le16 lens_separation;
	__le16 reserved1;
	__u8 button;
	__u8 reserved2[3];
	vive_proximity_change proximity_change;
	__u8 reserved3;
	__le16 proximity;
	__le16 ipd;
	__u8 reserved4[46];
} __attribute__((packed));

struct vive_headset_power_report {
	__u8 id;
	__le16 type;
	__u8 len;
	__u8 unknown1[9];
	__u8 reserved1[32];
	__u8 unknown2;
	__u8 reserved2[18];
} __attribute__((packed));

struct vive_headset_mainboard_device_info_report {
	__u8 id;
	__le16 type;
	__u8 len;
	__be16 edid_vid;
	__le16 edid_pid;
	__u8 unknown1[4];
	__le32 display_firmware_version;
	__u8 unknown2[48];
} __attribute__((packed));

struct vive_firmware_version_report {
	__u8 id;
	__le32 firmware_version;
	__le32 unknown1;
	__u8 string1[16];
	__u8 string2[16];
	__u8 hardware_version_micro;
	__u8 hardware_version_minor;
	__u8 hardware_version_major;
	__u8 hardware_revision;
	__le32 unknown2;
	__u8 fpga_version_minor;
	__u8 fpga_version_major;
	__u8 reserved[13];
} __attribute__((packed));

struct vive_headset_imu_sample {
        __s16 acc[3];
        __s16 rot[3];
	__le32 time_ticks;
	__u8 seq;
} __attribute__((packed));

struct vive_headset_imu_report {
	__u8 report_id;
	struct vive_headset_imu_sample samples[3];
} __attribute__((packed));

struct vive_headset_lighthouse_pulse1 {
	__u8 sensor_id;
	__u8 type;
	__le16 length;
	__le32 timestamp;
} __attribute__((packed));

struct vive_headset_lighthouse_pulse_report1 {
	__u8 report_id;
	struct vive_headset_lighthouse_pulse1 samples[7];
	__u8 padding;
} __attribute__((packed));

struct vive_controller_analog_trigger_message {
	__u8 squeeze;
	__u8 unknown[4];
} __attribute__((packed));

struct vive_controller_button_message {
	__u8 buttons;
} __attribute__((packed));

struct vive_controller_touch_move_message {
	__le16 pos[2];
	__u8 unknown[4];
} __attribute__((packed));

struct vive_controller_touch_press_message {
	__u8 buttons;
	__le16 pos[2];
	__u8 unknown[4];
} __attribute__((packed));

struct vive_controller_imu_message {
	__u8 time3;
	__le16 accel[3];
	__le16 gyro[3];
	__u8 unknown[4];
} __attribute__((packed));

struct vive_controller_ping_message {
	__u8 charge;
	__u8 unknown1[2];
	__le16 accel[3];
	__le16 gyro[3];
	__u8 unknown2[5];
} __attribute__((packed));

struct vive_controller_message {
	__u8 time1;
	__u8 sensor_id;
	__u8 time2;
	__u8 type;
	union {
		struct vive_controller_analog_trigger_message analog_trigger;
		struct vive_controller_button_message button;
		struct vive_controller_touch_move_message touch_move;
		struct vive_controller_touch_press_message touch_press;
		struct vive_controller_imu_message imu;
		struct vive_controller_ping_message ping;
		__u8 unknown[25];
	};
} __attribute__((packed));

struct vive_controller_report1 {
	__u8 report_id;
	struct vive_controller_message message;
} __attribute__((packed));

struct vive_controller_report2 {
	__u8 report_id;
	struct vive_controller_message message[2];
} __attribute__((packed));

struct vive_headset_lighthouse_pulse2 {
        uint8_t sensor_id;
        uint16_t length;
        uint32_t timestamp;
} __attribute__((packed));

struct vive_headset_lighthouse_pulse_report2 {
	__u8 report_id;
	struct vive_headset_lighthouse_pulse2 samples[9];
} __attribute__((packed));

struct vive_controller_poweroff_report {
	__u8 id;
	__u8 command;
	__u8 len;
	__u8 magic[4];
} __attribute__((packed));
