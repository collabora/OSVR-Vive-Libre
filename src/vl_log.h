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

#pragma once

enum class Level : int {
    DEBUG,
    INFO,
    WARNING,
    ERROR,
};

void vl_set_log_level(Level level);

#ifdef __GNUC__
__attribute__((format(printf, 2, 3)))
#endif
void vl_log(Level level, const char* format, ...);

#define vl_debug(...) vl_log(Level::DEBUG, __VA_ARGS__)
#define vl_info(...) vl_log(Level::INFO, __VA_ARGS__)
#define vl_warn(...) vl_log(Level::WARNING, __VA_ARGS__)
#define vl_error(...) vl_log(Level::ERROR, __VA_ARGS__)
