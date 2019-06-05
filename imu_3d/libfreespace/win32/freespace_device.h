/*
 * This file is part of libfreespace.
 *
 * Copyright (c) 2009-2012 Hillcrest Laboratories, Inc.
 *
 * libfreespace is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#ifndef FREESPACE_DEVICE_H_
#define FREESPACE_DEVICE_H_

#include "freespace_win32.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Service all async operations on a device.
 *
 * @param device which device
 * @return FREESPACE_SUCCESS if ok
 */
int freespace_private_devicePerform(struct FreespaceDeviceStruct* device);

/**
 * Force a device to close.  Attempt to recover from errors.
 * @param device which device.
 */
void freespace_private_forceCloseDevice(struct FreespaceDeviceStruct* device);

/**
 * Remove the device from the libfreespace API.
 * @param device which device.
 */
void freespace_private_removeDevice(struct FreespaceDeviceStruct* device);

/**
 * Add the device to the libfreespace API.
 * @param device which device.
 */
void freespace_private_insertDevice(struct FreespaceDeviceStruct* device);

#ifdef __cplusplus
}
#endif

#endif /* FREESPACE_DEVICE_H_ */
