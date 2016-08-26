/*
 * HTC Vive configuration data readout
 * Copyright 2016 Philipp Zabel
 * SPDX-License-Identifier:	LGPL-2.0+
 */
#ifndef __VIVE_CONFIG_H__
#define __VIVE_CONFIG_H__

#include <hidapi.h>
//#include "device.h"

char *vl_get_config(hid_device *dev);

#endif /* __VIVE_LIGHTHOUSE_CONFIG_H__ */
