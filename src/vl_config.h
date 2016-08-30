/*
 * HTC Vive configuration data readout
 * Copyright 2016 Philipp Zabel
 * SPDX-License-Identifier:	LGPL-2.0+
 */
#pragma once

#include <hidapi.h>

char *vl_get_config(hid_device *dev);
