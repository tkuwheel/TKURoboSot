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

#ifndef FREESPACE_DISCOVERY_H_
#define FREESPACE_DISCOVERY_H_

#include "freespace_win32.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize the discovery process.
 * @return 0 on success, or Freespace error code.
 */
int freespace_private_discoveryThreadInit();

/**
 * Exit the discovery process.
 * @return 0 on success, or Freespace error code.
 */
int freespace_private_discoveryThreadExit();

/*
 * Determine if the status of the connected devices has changed.
 * @return true if rescan is requested, false otherwise.
 */
BOOL freespace_private_discoveryStatusChanged();

/*
 * Get the handle used for device discovery.
 * @return The handle to be used for polling.
 */
HANDLE freespace_private_discoveryEventObject();

/*
 * Get the status of the independend discovery thread.
 * @return 0 on thread running normally, or Freespace error code.
 */
int freespace_private_discoveryGetThreadStatus();

/**
 * Request a rescan of the devices.  
 * Can be used to help recover from device errors.
 */
void freespace_private_requestDeviceRescan();

#ifdef __cplusplus
}
#endif

#endif /* FREESPACE_DISCOVERY_H_ */
