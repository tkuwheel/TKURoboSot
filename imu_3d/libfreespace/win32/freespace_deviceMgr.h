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

#ifndef FREESPACE_DEVICE_MGR_H_
#define FREESPACE_DEVICE_MGR_H_

#include "freespace_win32.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Private device manager API.
 */

/**
 * Create a new Freespace device.
 * @return The new device.
 */
struct FreespaceDeviceStruct* freespace_private_createDevice();

/**
 * Add an initialized device to the master list.
 *
 * @param device the device
 * @return FREESPACE_SUCCESS if ok
 */
int freespace_private_addDevice(struct FreespaceDeviceStruct* device);

/**
 * Free all resources associated with a device.
 * @param device The Freespace device.
 * @return FREESPACE_SUCCESS on success, or Freespace error code.
 */
int freespace_private_freeDevice(struct FreespaceDeviceStruct *device);

/**
 * Get an existing Freespace device using the primary handle index 0.
 * @param ref The reference compatible with discovery associated with the device.
 * @return The Freespace device
 */
struct FreespaceDeviceStruct* freespace_private_getDeviceByRef(FreespaceDeviceRef ref);

/**
 * Get an existing Freespace device.
 * @param id a device ID
 * @return The Freespace device
 */
struct FreespaceDeviceStruct* freespace_private_getDeviceById(FreespaceDeviceId id);

/**
 * Device filter for finding matching devices.
 * @param device The full device structure.
 * @return true on match, false otherwise.
 */
typedef BOOL (*freespace_deviceFilter)(struct FreespaceDeviceStruct* device);

/**
 * Get the list of devices with the mark set to the specified value.
 * @param list The list to populate with the available devices.
 * @param listSize The maximum size of the list.
 * @param listSizeOut The number of devices populated in the list.
 * @param filter The filter to use to determine matching devices.
 * @return 0 on success, error code on failure.
 */
int freespace_private_filterDevices(struct FreespaceDeviceStruct** list, int listSize, int *listSizeOut, freespace_deviceFilter filter);

#ifdef __cplusplus
}
#endif

#endif /* FREESPACE_DEVICE_MGR_H_ */
