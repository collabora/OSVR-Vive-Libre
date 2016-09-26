/*
 * HTC Vive USB HID reports
 * Copyright 2016 Philipp Zabel
 * SPDX-License-Identifier:	LGPL-2.0+
 */

#pragma once

#include <linux/hidraw.h>
#include <sys/ioctl.h>
#include <time.h>
#include <hidapi.h>

/*
 * Repeatedly tries to receive a feature report from the HID device
 * every millisecond until the timeout in milliseconds expires.
 */
static inline int hid_get_feature_report_timeout(hid_device *device, unsigned char *buf, size_t len,
						 unsigned int timeout)
{
    timespec ts = { /*.tv_sec=*/ 0, /*.tv_nsec=*/ 1000000 };

	int ret;

    for (unsigned i = 0; i < timeout; i++) {
        ret = hid_get_feature_report(device, buf, len);
		if (ret != -1 || errno != EPIPE)
			break;

		ts.tv_sec = 0;
		ts.tv_nsec = 1000000;
		nanosleep(&ts, NULL);
	}

	return ret;
}

