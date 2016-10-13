/*
 * HTC Vive USB HID reports
 * Copyright 2016 Philipp Zabel
 * SPDX-License-Identifier:	LGPL-2.0+
 */

#pragma once

#include <time.h>
#include <libusb.h>

static inline int hid_get_feature_report(libusb_device_handle* dev, uint16_t interface, uint8_t* buffer, size_t length) {
    // Currently not implemented and unused, see hidapi’s implementation.
    assert(buffer[0] != 0);

    uint16_t report_id = (3/*HID feature*/ << 8) | buffer[0];

    int ret = libusb_control_transfer(dev,
                                      LIBUSB_REQUEST_TYPE_CLASS | LIBUSB_RECIPIENT_INTERFACE | LIBUSB_ENDPOINT_IN,
                                      0x01/*HID get_report*/,
                                      report_id,
                                      interface,
                                      buffer, length,
                                      1000/*timeout millis*/);
    if (ret < 0)
        return -1;

    return ret;
}

/*
 * Repeatedly tries to receive a feature report from the HID device
 * every millisecond until the timeout in milliseconds expires.
 */
static inline int hid_get_feature_report_timeout(libusb_device_handle* device, uint16_t interface, unsigned char *buf, size_t len,
						 unsigned int timeout)
{
    timespec ts = { /*.tv_sec=*/ 0, /*.tv_nsec=*/ 1000000 };

	int ret;

    for (unsigned i = 0; i < timeout; i++) {
        ret = hid_get_feature_report(device, interface, buf, len);
		if (ret != -1 || errno != EPIPE)
			break;

		ts.tv_sec = 0;
		ts.tv_nsec = 1000000;
		nanosleep(&ts, NULL);
	}

	return ret;
}
