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

#include "freespace_deviceMgr.h"
#include "freespace_device.h"
#include "freespace_discovery.h"
#include "freespace_discoveryDetail.h"
#include <strsafe.h>
#include <malloc.h>
#include "freespace_config.h"
#include <cfgmgr32.h>

struct LibfreespaceData* freespace_instance_ = NULL;

static int checkDiscovery();


BOOL APIENTRY DllMain( HMODULE hModule,
                       DWORD  ul_reason_for_call,
                       LPVOID lpReserved
                       )
{
    switch (ul_reason_for_call)
	{
	case DLL_PROCESS_ATTACH:
	case DLL_THREAD_ATTACH:
	case DLL_THREAD_DETACH:
	case DLL_PROCESS_DETACH:
            break;
	}
    return TRUE;
}

LIBFREESPACE_API const char* freespace_version() {
	return LIBFREESPACE_VERSION;
}

LIBFREESPACE_API int freespace_init() {
    int rc;

    if (freespace_instance_ != NULL) {
        return FREESPACE_ERROR_BUSY;
    }

    freespace_instance_ = (struct LibfreespaceData*) malloc(sizeof(struct LibfreespaceData));
    if (freespace_instance_ == NULL) {
        return FREESPACE_ERROR_OUT_OF_MEMORY;
    }
    memset(freespace_instance_, 0, sizeof(struct LibfreespaceData));

    rc = freespace_private_discoveryThreadInit();
    if (rc != FREESPACE_SUCCESS) {
        return rc;
    }

    freespace_instance_->performEvent_ = CreateEvent(NULL, TRUE, FALSE, NULL);
    if (freespace_instance_->performEvent_ == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }

    return FREESPACE_SUCCESS;
}

LIBFREESPACE_API void freespace_exit() {
    int i;

    if (freespace_instance_ == NULL) {
        return;
    }

    // Shut down the discovery code.
    if (freespace_instance_->fdRemovedCallback_) {
        freespace_instance_->fdRemovedCallback_(freespace_instance_->performEvent_);
        freespace_instance_->fdRemovedCallback_(freespace_private_discoveryEventObject());
    }
    freespace_private_discoveryThreadExit();

    // Free all devices.
    for (i = 0; i < freespace_instance_->deviceCount_; i++) {
        freespace_private_freeDevice(freespace_instance_->devices_[i]);
    }

    CloseHandle(freespace_instance_->performEvent_);
    freespace_instance_->performEvent_ = NULL;

    free(freespace_instance_);
    freespace_instance_ = NULL;
}

LIBFREESPACE_API int freespace_setDeviceHotplugCallback(freespace_hotplugCallback callback,
                                                        void* cookie) {
    if (freespace_instance_ == NULL) {
        return FREESPACE_ERROR_NOT_FOUND;
    }

    freespace_instance_->hotplugCallback_ = callback;
    freespace_instance_->hotplugCookie_ = cookie;
    return FREESPACE_SUCCESS;
}

static BOOL performHelper(struct FreespaceDeviceStruct* device) {
    freespace_private_devicePerform(device);
    return FALSE;
}

LIBFREESPACE_API int freespace_perform() {
    int rc;
    // Reset the perform event 
    ResetEvent(freespace_instance_->performEvent_);

    // Check if the device discovery thread has detected any changes
    // and rescan if so.
    rc = checkDiscovery();

    // Service all of the devices.
    // NOTE: Servicing includes initiating
    freespace_private_filterDevices(NULL, 0, NULL, performHelper);

    return rc;
}

LIBFREESPACE_API void freespace_setFileDescriptorCallbacks(freespace_pollfdAddedCallback addedCallback,
                                                           freespace_pollfdRemovedCallback removedCallback) {
    freespace_instance_->fdAddedCallback_ = addedCallback;
    freespace_instance_->fdRemovedCallback_ = removedCallback;
}

LIBFREESPACE_API int freespace_syncFileDescriptors() {
    if (freespace_instance_->fdAddedCallback_ == NULL) {
        return FREESPACE_SUCCESS;
    }

    freespace_instance_->fdAddedCallback_(freespace_private_discoveryEventObject(), 1);
    freespace_instance_->fdAddedCallback_(freespace_instance_->performEvent_, 1);

    return FREESPACE_SUCCESS;
}

LIBFREESPACE_API int freespace_getNextTimeout(int* timeoutMsOut) {
    // TODO
    *timeoutMsOut = 0xffffffff;
    return FREESPACE_SUCCESS;
}

struct FreespaceDeviceStruct* freespace_private_getDeviceByRef(FreespaceDeviceRef ref) {
    int i;
    WCHAR* uniqueRef;

    // Determine the unique identifier
    uniqueRef = freespace_private_generateUniqueId(ref);
    if (uniqueRef == NULL) {
        return NULL;
    }

    for (i = 0; i < freespace_instance_->deviceCount_; i++) {
        if (lstrcmp(uniqueRef, freespace_instance_->devices_[i]->uniqueId_) == 0) {
            free(uniqueRef);
            return freespace_instance_->devices_[i];
        }
    }
    free(uniqueRef);
    return NULL;
}

struct FreespaceDeviceStruct* freespace_private_getDeviceById(FreespaceDeviceId id) {
    int i;
    for (i = 0; i < freespace_instance_->deviceCount_; i++) {
        if (id == freespace_instance_->devices_[i]->id_) {
            return freespace_instance_->devices_[i];
        }
    }
    return NULL;
}

int freespace_private_filterDevices(struct FreespaceDeviceStruct** list, int listSize, int *listSizeOut, freespace_deviceFilter filter) {
    int i;
    int itemsAdded = 0;

    if (filter == NULL) {
        return FREESPACE_SUCCESS;
    }

    for (i = 0; i < freespace_instance_->deviceCount_; i++) {
        struct FreespaceDeviceStruct* device = freespace_instance_->devices_[i];
        if (filter(device)) {
            if (itemsAdded < listSize && list != NULL) {
                list[itemsAdded] = device;
                itemsAdded++;
            }
        }
    }
    if (listSizeOut != NULL) {
        *listSizeOut = itemsAdded;
    }
    return FREESPACE_SUCCESS;
}

static BOOL filterApiDeviceList(struct FreespaceDeviceStruct* device) {
    return device->isAvailable_;
}


LIBFREESPACE_API int freespace_getDeviceList(FreespaceDeviceId* list, int listSize, int *listSizeOut) {
    int i;
    int rc;

    // Check if the device list has changed.
    rc = checkDiscovery();
    if (rc != FREESPACE_SUCCESS) {
        return rc;
    }

    for (*listSizeOut = 0, i = 0; *listSizeOut < listSize && i < freespace_instance_->deviceCount_; i++) {
        struct FreespaceDeviceStruct* device = freespace_instance_->devices_[i];
        if (device->isAvailable_) {
            list[*listSizeOut] = device->id_;
            (*listSizeOut)++;
        }
    }

    return FREESPACE_SUCCESS;
}

int freespace_private_addDevice(struct FreespaceDeviceStruct* device) {
    // Add the device to list.
    freespace_instance_->devices_[freespace_instance_->deviceCount_] = device;
    freespace_instance_->deviceCount_++;

    return FREESPACE_SUCCESS;
}

static BOOL filterInitialize(struct FreespaceDeviceStruct* device) {
    int i;
    for (i = 0; i < device->handleCount_; i++) {
        device->handle_[i].enumerationFlag_ = FALSE;
    }

    device->status_ = FREESPACE_DISCOVERY_STATUS_UNKNOWN;

    return FALSE;
}

static BOOL filterSweep(struct FreespaceDeviceStruct* device) {
    // Device has been removed if it existed before and
    // wasn't marked as existing now.
    return (device->status_ == FREESPACE_DISCOVERY_STATUS_UNKNOWN);
}

static BOOL filterPartiallyRemoved(struct FreespaceDeviceStruct* device) {
    /* Device is partially existing
     *    1. Status is existing or added
     *    2. Not all of usages were enumerated
     */
    int i;
    if (device->status_ != FREESPACE_DISCOVERY_STATUS_EXISTING &&
        device->status_ != FREESPACE_DISCOVERY_STATUS_ADDED) {
        return FALSE;
    }

    if (device->isAvailable_ == FALSE) {
        return FALSE;
    }

    for (i = 0; i < device->handleCount_; i++) {
        if (!device->handle_[i].enumerationFlag_) {
            return TRUE;
        }
    }

    return FALSE;
}

static BOOL filterReady(struct FreespaceDeviceStruct* device) {
    /* Device is added if
     *    1. Status is existing or added
     *    2. All of the required handles are valid
     */
    int i;
    if (device->status_ != FREESPACE_DISCOVERY_STATUS_EXISTING &&
        device->status_ != FREESPACE_DISCOVERY_STATUS_ADDED) {
        return FALSE;
    }

    if (device->isAvailable_ == TRUE) {
        return FALSE;
    }

    for (i = 0; i < device->handleCount_; i++) {
        if (!device->handle_[i].enumerationFlag_) {
            return FALSE;
        }
    }

    return TRUE;
}

/*
 * Remove the devices that are no longer present in the system. 
 */
int checkDiscoveryRemoveDevices() {
    int i;
    struct FreespaceDeviceStruct* list[FREESPACE_MAXIMUM_DEVICE_COUNT];
    int listLength = 0;

    // Collect the removed devices.
    freespace_private_filterDevices(list, FREESPACE_MAXIMUM_DEVICE_COUNT, &listLength, filterSweep);

    // Remove them from the device list so that future API calls fail to this device
    // fail. See callbacks after this loop.
    for (i = 0; i < listLength; i++) {
        int idx;
        DEBUG_WPRINTF(L"device %d removed\n", list[i]->id_);
        for (idx = 0; idx < freespace_instance_->deviceCount_; idx++) {
            if (list[i] == freespace_instance_->devices_[idx]) {
                memmove(&freespace_instance_->devices_[idx], &freespace_instance_->devices_[idx + 1], (freespace_instance_->deviceCount_ - idx - 1) * sizeof(struct FreespaceDeviceStruct*));
                freespace_instance_->deviceCount_--;
            }
        }
    }

    // Call the removal callbacks.
    for (i = 0; i < listLength; i++) {
        freespace_private_removeDevice(list[i]);
    }

    // Free the device structure.
    for (i = 0; i < listLength; i++) {
        freespace_private_freeDevice(list[i]);
    }

    return listLength;
}

/*
 * Process devices with multiple handles that are not fully complete.
 */
int checkDiscoveryPartiallyRemovedDevices() {
    struct FreespaceDeviceStruct* list[FREESPACE_MAXIMUM_DEVICE_COUNT];
    int listLength = 0;
    int i;

    // Collect the removed devices and call the removed callbacks.
    freespace_private_filterDevices(list, FREESPACE_MAXIMUM_DEVICE_COUNT, &listLength, filterPartiallyRemoved);
    for (i = 0; i < listLength; i++) {
        DEBUG_WPRINTF(L"device %d partially removed\n", list[i]->id_);
    }
    for (i = 0; i < listLength; i++) {
        freespace_private_removeDevice(list[i]);
    }

    // Close file handles of devices that are open.
    for (i = 0; i < listLength; i++) {
        freespace_closeDevice(list[i]->id_);
    }

    return listLength;
}

/*
 * Process devices with multiple handles that are not fully complete.
 */
int checkDiscoveryAddedDevices() {
    struct FreespaceDeviceStruct* list[FREESPACE_MAXIMUM_DEVICE_COUNT];
    int listLength = 0;
    int i;

    // Collect the added devices and call the insertion callbacks
    freespace_private_filterDevices(list, FREESPACE_MAXIMUM_DEVICE_COUNT, &listLength, filterReady);
    for (i = 0; i < listLength; i++) {
        DEBUG_WPRINTF(L"device %d added\n", list[i]->id_);
    }
    for (i = 0; i < listLength; i++) {
        freespace_private_insertDevice(list[i]);
    }
    return listLength;
}


int checkDiscovery() {
    if (freespace_private_discoveryStatusChanged()) {
        int rc;
        int totalChanges = 0;

        // Wait for system to stabilize before scanning.
        if (CMP_WaitNoPendingInstallEvents(0) == WAIT_TIMEOUT) {
            DEBUG_PRINTF("Pending install events.  Wait for resolution.\n");
            freespace_private_requestDeviceRescan();
            return FREESPACE_ERROR_BUSY;
        }

        DEBUG_WPRINTF(L"Scanning devices\n");

        // Mark and sweep the device list.
        freespace_private_filterDevices(NULL, 0, NULL, filterInitialize);

        // Mark everything that still exists and add new devices
        rc = freespace_private_scanAndAddDevices();
        if (rc != FREESPACE_SUCCESS) {
            // Unexpected error.  Schedule a rescan.
            DEBUG_WPRINTF(L"Error %d while scanning devices.  Request a rescan.\n", rc);
            freespace_private_requestDeviceRescan();
            return rc;
        }

        // Handle all changes.
        totalChanges += checkDiscoveryRemoveDevices();
        totalChanges += checkDiscoveryPartiallyRemovedDevices();
        totalChanges += checkDiscoveryAddedDevices();

        /*
         * Continue to schedule a rescan until no changes are detected.
         * Although this should not be necessary, rescanning until a 
         * stable state is reached should increase robustness.
         */
        if (totalChanges != 0) {
            DEBUG_WPRINTF(L"Detected %d changes.  Schedule rescan\n", totalChanges);
            freespace_private_requestDeviceRescan();
        }
    }

    return freespace_private_discoveryGetThreadStatus();
}
