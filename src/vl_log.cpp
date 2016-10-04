/*
 * Vive Libre
 *
 * Copyright (C) 2016 Emmanuel Gil Peyrot <emmanuel.peyrot@collabora.co.uk>
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

#include <cstdarg>
#include <cstdio>

#include "vl_log.h"

static Level log_level = Level::Debug;

void vl_set_log_level(Level level) {
    log_level = level;
}

void vl_log(Level level, const char* format, ...) {
    if (level < log_level)
        return;

    va_list args;
    va_start(args, format);
    vprintf(format, args);
    va_end(args);
    printf("\n");
    fflush(stdout);
}
