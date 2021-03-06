/*
 * HTC Vive USB HID reports
 * Copyright 2016 Philipp Zabel
 * Copyright 2016 Lubosz Sarnecki <lubosz.sarnecki@collabora.co.uk>
 * SPDX-License-Identifier:    LGPL-2.0+
 */

#include <errno.h>
#include <string.h>
#include <zlib.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <cstdint>

#include <libusb.h>

#include "vl_config.h"
#include "vl_synchronous.h"
#include "vl_log.h"

/*
 * Downloads configuration data stored in the Vive headset and controller.
 */
char *vl_get_config(vl_device& device, uint16_t interface)
{
    libusb_device_handle* dev = device.handle;
    unsigned char buf[64];
    unsigned char *config_json;
    void *config_z;
    z_stream strm;
    int count = 0;
    int ret;

    buf[0] = 0x10;
    ret = hid_get_feature_report_timeout(dev, interface, buf, sizeof(buf), 100);
    if (ret < 0) {
        vl_error("%s: Read error 0x10: %d", "devname", errno);
        return NULL;
    }

    config_z = malloc(4096);

    buf[0] = 0x11;
    do {
        ret = hid_get_feature_report_timeout(dev, interface, buf, sizeof(buf), 100);
        if (ret < 0) {
            vl_error("%s: Read error after %d bytes: %d",
                     "devname", count, errno);
            free(config_z);
            return NULL;
        }

        if (buf[1] > 62) {
            vl_error("%s: Invalid configuration data at %d",
                     "devname", count);
            free(config_z);
            return NULL;
        }

        if (count + buf[1] > 4096) {
            vl_error("%s: Configuration data too large",
                     "devname");
            free(config_z);
            return NULL;
        }

        memcpy((uint8_t*)config_z + count, buf + 2, buf[1]);
        count += buf[1];
    } while (buf[1]);

    vl_debug("%s: Read configuration data: %d bytes", "devname", count);

    strm.zalloc = Z_NULL;
    strm.zfree = Z_NULL;
    strm.opaque = Z_NULL;
    strm.avail_in = 0;
    strm.next_in = Z_NULL;
    ret = inflateInit(&strm);
    if (ret != Z_OK) {
        vl_error("inflate_init failed: %d\n", ret);
        free(config_z);
        return NULL;
    }

    config_json = (unsigned char*) malloc(32768);

    strm.avail_in = count;
    strm.next_in = (z_const Bytef *) config_z;
    strm.avail_out = 32768;
    strm.next_out = config_json;

    ret = inflate(&strm, Z_FINISH);
    free(config_z);
    if (ret != Z_STREAM_END) {
        vl_error("%s: Failed to inflate configuration data: %d",
                 "devname", ret);
        free(config_json);
        return NULL;
    }

    vl_error("%s: Inflated configuration data: %lu bytes",
             "devname", strm.total_out);

    config_json[strm.total_out] = '\0';

    return (char *) realloc(config_json, strm.total_out + 1);
}
