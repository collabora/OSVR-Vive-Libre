/*
 * HTC Vive USB HID reports
 * Copyright 2016 Philipp Zabel
 * SPDX-License-Identifier:	LGPL-2.0+
 */

#pragma once

#include <cstdint>

#include "vl_driver.h"

char *vl_get_config(vl_device& device, uint16_t interface);
