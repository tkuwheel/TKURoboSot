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

#include "freespace/freespace.h"
#include "freespace/freespace_deviceTable.h"
#include "hotplug.h"
#include "freespace_config.h"

#include <libusb-1.0/libusb.h>
#include <stdlib.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>

#define FREESPACE_RECEIVE_QUEUE_SIZE 8 // Could be tuned better. 3-4 might be good enough

/**
 * The device state is primarily used to keep track of FreespaceDevice allocations.
 * The state machine looks like the following:
 *
 *     o-->CONNECTED
 *          | ^   |
 *          v |   |
 *        OPENED  |
 *           |    |
 *           v    v
 *         DISCONNECTED
 *
 *  A FreespaceDevice* is allocated when a device is connected.  On the CONNECTED->DISCONNECTED
 *  transition, the hotplug routine frees the FreespaceDevice*.  On the OPENED->DISCONNECTED
 *  transition, the user may have a reference to a FreespaceDevice*, so it can't be freed then. We
 *  free it on the call to close().
 */
enum FreespaceDeviceState {
    FREESPACE_CONNECTED,
    FREESPACE_OPENED,
    FREESPACE_DISCONNECTED
};

/**
 * FreespaceDevice internal data.
 */
struct FreespaceReceiveTransfer {
    // Convenience backpointer to the device data structure.
    struct FreespaceDevice* device_;

    // Transfer information
    struct libusb_transfer* transfer_;
    uint8_t buffer_[FREESPACE_MAX_INPUT_MESSAGE_SIZE];

    // Synchronous interface usage for the state of the
    // queue.
    int submitted_;
};

struct FreespaceDevice {
    FreespaceDeviceId id_;
    enum FreespaceDeviceState state_;

    // Timestamp for checking connected state.
    uint32_t ts_;

    struct libusb_device* dev_;
    struct libusb_device_handle* handle_;
    uint16_t idVendor_;
    uint16_t idProduct_;
    int kernelDriverDetached_;

    struct FreespaceDeviceAPI const * api_;
    int writeEndpointAddress_;
    int readEndpointAddress_;
    int maxWriteSize_;
    int maxReadSize_;

    freespace_receiveCallback receiveCallback_;
    freespace_receiveMessageCallback receiveMessageCallback_;
    void* receiveCookie_;
    void* receiveMessageCookie_;

    int receiveQueueHead_;
    struct FreespaceReceiveTransfer receiveQueue_[FREESPACE_RECEIVE_QUEUE_SIZE];
};

static struct FreespaceDevice* devices[FREESPACE_MAXIMUM_DEVICE_COUNT];
static int numDevices = 0;
static FreespaceDeviceId nextFreeIndex = 0;
static uint32_t ts = 0;

static struct libusb_context* freespace_libusb_context = NULL;
static freespace_pollfdAddedCallback userAddedCallback = NULL;
static freespace_pollfdRemovedCallback userRemovedCallback = NULL;
static freespace_hotplugCallback hotplugCallback = NULL;
static void* hotplugCookie;

static int libusb_to_freespace_error(int libusberror) {
    // libusb returns values greater than 0 for success for some functions.
    if (libusberror >= 0) {
        return FREESPACE_SUCCESS;
    }

    switch (libusberror) {
    case LIBUSB_ERROR_IO: return FREESPACE_ERROR_IO;
    case LIBUSB_ERROR_INVALID_PARAM: return FREESPACE_ERROR_UNEXPECTED;
    case LIBUSB_ERROR_ACCESS: return FREESPACE_ERROR_ACCESS;
    case LIBUSB_ERROR_NO_DEVICE: return FREESPACE_ERROR_NO_DEVICE;
    case LIBUSB_ERROR_NOT_FOUND: return FREESPACE_ERROR_NOT_FOUND;
    case LIBUSB_ERROR_BUSY: return FREESPACE_ERROR_BUSY;
    case LIBUSB_ERROR_TIMEOUT: return FREESPACE_ERROR_TIMEOUT;
    case LIBUSB_ERROR_OVERFLOW: return FREESPACE_ERROR_UNEXPECTED;
    case LIBUSB_ERROR_PIPE: return FREESPACE_ERROR_PIPE;
    case LIBUSB_ERROR_INTERRUPTED: return FREESPACE_ERROR_INTERRUPTED;
    case LIBUSB_ERROR_NO_MEM: return FREESPACE_ERROR_OUT_OF_MEMORY;
    case LIBUSB_ERROR_NOT_SUPPORTED: return FREESPACE_ERROR_UNEXPECTED;
    case LIBUSB_ERROR_OTHER: return FREESPACE_ERROR_UNEXPECTED;
    default: return FREESPACE_ERROR_UNEXPECTED;
    }
}

static int libusb_transfer_status_to_freespace_error(enum libusb_transfer_status status) {
    switch (status) {
    case LIBUSB_TRANSFER_COMPLETED: return FREESPACE_SUCCESS;
    case LIBUSB_TRANSFER_ERROR: return FREESPACE_ERROR_IO;
    case LIBUSB_TRANSFER_TIMED_OUT: return FREESPACE_ERROR_TIMEOUT;
    case LIBUSB_TRANSFER_CANCELLED: return FREESPACE_ERROR_INTERRUPTED;
    case LIBUSB_TRANSFER_STALL: return FREESPACE_ERROR_IO;
    case LIBUSB_TRANSFER_NO_DEVICE: return FREESPACE_ERROR_NO_DEVICE;
    case LIBUSB_TRANSFER_OVERFLOW: return FREESPACE_ERROR_UNEXPECTED; // libfreespace should protect against this.
    default: return FREESPACE_ERROR_UNEXPECTED;
    }
}

const char* freespace_version() {
    return LIBFREESPACE_VERSION;
}

int freespace_init() {
    int rc;

    rc = freespace_hotplug_init();
    if (rc != FREESPACE_SUCCESS) {
        return rc;
    }

    rc = libusb_init(&freespace_libusb_context);
    return libusb_to_freespace_error(rc);
}

void freespace_exit() {
    struct FreespaceDevice* device;
    int i;
    for (i = 0; i < FREESPACE_MAXIMUM_DEVICE_COUNT; i++) {
        if (devices[i] != NULL) {
            device = devices[i];
            if (nextFreeIndex == -1) {
                nextFreeIndex = i;
            }
            libusb_unref_device(device->dev_);
            free(device);
            devices[i] = NULL;
            numDevices--;
        }
    }
    libusb_exit(freespace_libusb_context);
    freespace_hotplug_exit();
}

static struct FreespaceDeviceAPI const * lookupDevice(struct libusb_device_descriptor* desc) {
    int i;
    for (i = 0; i < freespace_deviceAPITableNum; i++) {
        struct FreespaceDeviceAPI const * api = &freespace_deviceAPITable[i];
        if (desc->idVendor == api->idVendor_ &&
            desc->idProduct == api->idProduct_) {
            return api;
        }
    }

    return NULL;
}

static struct FreespaceDevice* findDeviceById(FreespaceDeviceId id) {
    int i;
    for (i = 0; i < FREESPACE_MAXIMUM_DEVICE_COUNT; i++) {
        if (devices[i] != NULL && devices[i]->id_ == id) {
            return devices[i];
        }
    }

    return NULL;
}

static int addFreespaceDevice(struct FreespaceDevice* device) {
    if (nextFreeIndex == -1) {
        return FREESPACE_ERROR_INVALID_DEVICE;
    }
    devices[nextFreeIndex] = device;
    numDevices++;
    if (numDevices < FREESPACE_MAXIMUM_DEVICE_COUNT) {
        while (devices[nextFreeIndex] != NULL) {
            nextFreeIndex++;
            if (nextFreeIndex == FREESPACE_MAXIMUM_DEVICE_COUNT) {
                nextFreeIndex = 0;
            }
        }
    } else {
        nextFreeIndex = -1;
    }
    return FREESPACE_SUCCESS;
}

static void removeFreespaceDevice(struct FreespaceDevice* device) {
    int i;
    for (i = 0; i < FREESPACE_MAXIMUM_DEVICE_COUNT; i++) {
        if (devices[i] != NULL && devices[i]->id_ == device->id_) {
            if (nextFreeIndex == -1) {
                nextFreeIndex = i;
            }
            libusb_unref_device(device->dev_);
            free(device);
            devices[i] = NULL;
            numDevices--;
            return;
        }
    }
}

static int scanDevices() {
    struct libusb_device** devs;
    ssize_t count;
    ssize_t i;
    int rc;
    int needToRescan;

    // Check if the devices need to be rescanned.
    rc = freespace_hotplug_perform(&needToRescan);
    if (rc != FREESPACE_SUCCESS || !needToRescan) {
        return rc;
    }

    count = libusb_get_device_list(freespace_libusb_context, &devs);
    if (count < 0) {
        return libusb_to_freespace_error(count);
    }

    ts++;
    for (i = 0; i < count; i++) {
        struct libusb_device_descriptor desc;
        struct libusb_device* dev = devs[i];
        struct FreespaceDeviceAPI const * api;

        rc = libusb_get_device_descriptor(dev, &desc);
        if (rc < 0) {
            // Can't get the device descriptor, so try the next one.
            continue;
        }

        // Find if this device is in the known list.
        api = lookupDevice(&desc);
        if (api != NULL) {
            uint8_t deviceAddress = libusb_get_device_address(dev);
            struct FreespaceDevice* device;
            device = findDeviceById(deviceAddress);
            if (device == NULL) {
                device = (struct FreespaceDevice*) malloc(sizeof(struct FreespaceDevice));
                if (device == NULL) {
                    // Out of memory.
                    libusb_free_device_list(devs, 1);
                    return FREESPACE_ERROR_OUT_OF_MEMORY;
                }
                memset(device, 0, sizeof(struct FreespaceDevice));

                libusb_ref_device(dev);
                device->dev_ = dev;
                device->idProduct_ = desc.idProduct;
                device->idVendor_ = desc.idVendor;
                device->api_ = api;
                device->id_ = libusb_get_device_address(dev);
                device->state_ = FREESPACE_CONNECTED;
                device->ts_ = ts;
                addFreespaceDevice(device);
                if (hotplugCallback) {
                    hotplugCallback(FREESPACE_HOTPLUG_INSERTION, device->id_, hotplugCookie);
                }
            }
            device->ts_ = ts;
        }
    }

    for (i = 0; i < FREESPACE_MAXIMUM_DEVICE_COUNT; i++) {
        struct FreespaceDevice* d = devices[i];
        if (d != NULL && d->ts_ != ts) {
            if (hotplugCallback) {
                hotplugCallback(FREESPACE_HOTPLUG_REMOVAL, d->id_, hotplugCookie);
            }
            if (d->state_ == FREESPACE_OPENED) {
                d->state_ = FREESPACE_DISCONNECTED;
            } else {
                removeFreespaceDevice(d);
            }
        }
    }

    libusb_free_device_list(devs, 1);
    return FREESPACE_SUCCESS;
}

int freespace_setDeviceHotplugCallback(freespace_hotplugCallback callback,
                                       void* cookie) {
    hotplugCallback = callback;
    hotplugCookie = cookie;
    return FREESPACE_SUCCESS;
}

int freespace_getDeviceList(FreespaceDeviceId* idList,
                            int maxIds,
                            int* numIds) {
    int i;
    int rc;
    *numIds = 0;

    rc = scanDevices();
    if (rc != FREESPACE_SUCCESS) {
        return rc;
    }

    for (i = 0; i < FREESPACE_MAXIMUM_DEVICE_COUNT && *numIds < maxIds; i++) {
        if (devices[i] != NULL) {
            idList[*numIds] = devices[i]->id_;
            *numIds = *numIds + 1;
        }
    }

    return FREESPACE_SUCCESS;
}

int freespace_getDeviceInfo(FreespaceDeviceId id,
                            struct FreespaceDeviceInfo* info) {
    struct FreespaceDevice* device = findDeviceById(id);

    if (device != NULL) {
        info->vendor = device->idVendor_;
        info->product = device->idProduct_;
        info->name = device->api_->name_;
        info->hVer = device->api_->hVer_;
        return FREESPACE_SUCCESS;
    } else {
        return FREESPACE_ERROR_NOT_FOUND;
    }
}

int freespace_isNewDevice(FreespaceDeviceId id) {
    struct FreespaceDeviceInfo info;
    const struct FreespaceDeviceInfo*   pDeviceInfo = NULL;
    int rc;
    int idx;

    rc = freespace_getDeviceInfo(id, &info);
    if (rc != FREESPACE_SUCCESS) {
        return rc;
    }

    // Determine if the product ID represent a new device
    for (idx = 0; idx < freespace_newDeviceAPITableNum; ++idx)
    {
        pDeviceInfo = &freespace_newDeviceAPITable[idx];
        if ( (pDeviceInfo->vendor == info.vendor) &&
             (pDeviceInfo->product == info.product) )
        {
            return FREESPACE_SUCCESS;
        }
    }
    return FREESPACE_ERROR_NO_DEVICE;
}

static void receiveCallback(struct libusb_transfer* transfer) {
    struct FreespaceReceiveTransfer* rt = (struct FreespaceReceiveTransfer*) transfer->user_data;
    struct FreespaceDevice* device = rt->device_;

    if (transfer->status == LIBUSB_TRANSFER_CANCELLED) {
        // Canceled. This only happens on cleanup. Don't report errors or resubmit.
        rt->submitted_ = 0;
        return;
    }

    if (device->receiveCallback_ != NULL || device->receiveMessageCallback_ != NULL) {
        // Using async interface, so call user back immediately.
        int rc = libusb_transfer_status_to_freespace_error(transfer->status);
        if (device->receiveCallback_ != NULL) {
            device->receiveCallback_(device->id_, (const uint8_t*) transfer->buffer, transfer->actual_length, device->receiveCookie_, rc);
        }
        if (device->receiveMessageCallback_ != NULL) {
            struct freespace_message m;
            
            rc = freespace_decode_message((const uint8_t*) transfer->buffer, transfer->actual_length, &m, device->api_->hVer_);
            if (rc == FREESPACE_SUCCESS) {
                device->receiveMessageCallback_(device->id_, &m, device->receiveMessageCookie_, FREESPACE_SUCCESS);
            } else {
                device->receiveMessageCallback_(device->id_, NULL, device->receiveMessageCookie_, rc);
            }
        }

        // Re-submit the transfer for the to get the next receive going.
        // NOTE: Can't handle any error returns here.
        libusb_submit_transfer(transfer);
    } else {
        // Using sync interface, so queue.
        rt->submitted_ = 0;
    }
}

int freespace_terminateReceiveTransfers(struct FreespaceDevice* device) {
    int rc = LIBUSB_SUCCESS;
    int i;
    int canceledCount = 0;
    int retries;

    // Cancel all submitted transfers.
    for (i = 0; i < FREESPACE_RECEIVE_QUEUE_SIZE; i++) {
        struct FreespaceReceiveTransfer* rt = &device->receiveQueue_[i];
        if (rt->transfer_ != NULL) {
            if (rt->submitted_) {
                rc = libusb_cancel_transfer(rt->transfer_);

                // Do not free the transfer here. Transfer cancels are
                // asynchronous. The callback will know what to do.

                if (rc == LIBUSB_SUCCESS) {
                    canceledCount++;
                } else {
                    // Force free on error.
                    libusb_free_transfer(rt->transfer_);
                    rt->transfer_ = NULL;
                    rt->submitted_ = 0;
                }
            } else {
                // Not submitted to libusb, so this can be freed immediatedly.
                libusb_free_transfer(rt->transfer_);
                rt->transfer_ = NULL;
            }
        }
    }

    // Wait for the cancellation to finish up. libusb seems to cancel
    // one transfer per call to libusb_handle_events_timeout, so make the
    // retries take that into account and add slack.
    retries = canceledCount * 3;
    while (canceledCount > 0 && retries > 0) {
        struct timeval tv;

        tv.tv_sec = 0;
        tv.tv_usec = 100000;

        rc = libusb_handle_events_timeout(freespace_libusb_context, &tv);
        if (rc != LIBUSB_SUCCESS) {
            break;
        }

        for (i = 0; i < FREESPACE_RECEIVE_QUEUE_SIZE; i++) {
            struct FreespaceReceiveTransfer* rt = &device->receiveQueue_[i];
            if (rt->transfer_ != NULL && rt->submitted_ == 0) {
                // Cancel completed.
                libusb_free_transfer(rt->transfer_);
                rt->transfer_ = NULL;
                canceledCount--;
            }
        }

        retries--;
    }

    // Force clean any left.
    for (i = 0; i < FREESPACE_RECEIVE_QUEUE_SIZE; i++) {
        struct FreespaceReceiveTransfer* rt = &device->receiveQueue_[i];
        if (rt->transfer_ != NULL) {
            libusb_free_transfer(rt->transfer_);
            rt->transfer_ = NULL;
        }
    }

    return libusb_to_freespace_error(rc);
}

int freespace_initiateReceiveTransfers(struct FreespaceDevice* device) {
    int rc = LIBUSB_SUCCESS;
    int i;

    device->receiveQueueHead_ = 0;
    for (i = 0; i < FREESPACE_RECEIVE_QUEUE_SIZE; i++) {
        struct FreespaceReceiveTransfer* rt = &device->receiveQueue_[i];
        rt->device_ = device;
        rt->transfer_ = libusb_alloc_transfer(0);
        libusb_fill_interrupt_transfer(rt->transfer_,
                                       device->handle_,
                                       device->readEndpointAddress_,
                                       rt->buffer_,
                                       device->maxReadSize_,
                                       receiveCallback,
                                       rt,
                                       0);
        rc = libusb_submit_transfer(rt->transfer_);
        if (rc != LIBUSB_SUCCESS) {
            freespace_terminateReceiveTransfers(device);
            break;
        }

        rt->submitted_ = 1;
    }

    return libusb_to_freespace_error(rc);
}

int freespace_openDevice(FreespaceDeviceId id) {
    struct FreespaceDevice* device = findDeviceById(id);
    struct libusb_config_descriptor *config;
    const struct libusb_interface_descriptor* intd;
    int controlInterfaceNumber;
    int i;
    int rc;

    if (device == NULL) {
        return FREESPACE_ERROR_NOT_FOUND;
    }

    rc = libusb_open(device->dev_, &device->handle_);
    if (rc != LIBUSB_SUCCESS) {
        return libusb_to_freespace_error(rc);
    }

    controlInterfaceNumber = device->api_->controlInterfaceNumber_;
    rc = libusb_kernel_driver_active(device->handle_, controlInterfaceNumber);
    if (rc == 1) {
    	// Kernel driver attached.  Disconnect it.
    	rc = libusb_detach_kernel_driver(device->handle_, controlInterfaceNumber);
    	if (rc == LIBUSB_SUCCESS) {
            device->kernelDriverDetached_ = 1;
    	}
    }

    rc = libusb_claim_interface(device->handle_, controlInterfaceNumber);
    if (rc != LIBUSB_SUCCESS) {
        return libusb_to_freespace_error(rc);
    }

    rc = libusb_get_active_config_descriptor(device->dev_, &config);
    if (rc != LIBUSB_SUCCESS) {
        return libusb_to_freespace_error(rc);
    }

    intd = config->interface[controlInterfaceNumber].altsetting;

    for (i = 0; i < intd[0].bNumEndpoints; ++i) {
        const struct libusb_endpoint_descriptor* endpoint = &intd[0].endpoint[i];

        if ((endpoint->bEndpointAddress & LIBUSB_ENDPOINT_DIR_MASK) == LIBUSB_ENDPOINT_IN) {
            device->readEndpointAddress_ = endpoint->bEndpointAddress;
            device->maxReadSize_ = endpoint->wMaxPacketSize;
        } else {
            device->writeEndpointAddress_ = endpoint->bEndpointAddress;
            device->maxWriteSize_ = endpoint->wMaxPacketSize;
        }
    }
    if (device->maxReadSize_ == 0 || device->maxWriteSize_ == 0) {
        // Weird.  The device didn't have a read and write endpoint.
        return FREESPACE_ERROR_UNEXPECTED;
    }

    device->state_ = FREESPACE_OPENED;

    // Start the receive queue working.
    rc = freespace_initiateReceiveTransfers(device);
    return rc;
}

void freespace_closeDevice(FreespaceDeviceId id) {
    struct FreespaceDevice* device;
    device = findDeviceById(id);
    if (device != NULL && device->handle_ != NULL) {
        // Stop receives.
        freespace_terminateReceiveTransfers(device);

        // Should we wait until everything terminates cleanly?

        // Release our lock on the interface.
        libusb_release_interface(device->handle_, device->api_->controlInterfaceNumber_);

        // Re-attach the kernel driver if we detached it before.
        if (device->kernelDriverDetached_) {
            // This currently fails, and there doesn't seem to be anything that we
            // can do.
            libusb_attach_kernel_driver(device->handle_, device->api_->controlInterfaceNumber_);
        }
        libusb_close(device->handle_);
        device->handle_ = NULL;

        if (device->state_ == FREESPACE_DISCONNECTED) {
            removeFreespaceDevice(device);
        } else {
            device->state_ = FREESPACE_CONNECTED;
        }
    }
}

int freespace_private_send(FreespaceDeviceId id,
                           const uint8_t* message,
                           int length) {
    int rc;
    int count;
    struct FreespaceDevice* device;
    device = findDeviceById(id);

    if (device == NULL || device->state_ != FREESPACE_OPENED) {
        return FREESPACE_ERROR_NOT_FOUND;
    }

    if (length > device->maxWriteSize_) {
        // Can't write more than the max allowed size, so fail rather than send a partial packet.
        return FREESPACE_ERROR_SEND_TOO_LARGE;
    }

    rc = libusb_interrupt_transfer(device->handle_, device->writeEndpointAddress_, (unsigned char*) message, length, &count, 0);
    if (rc != LIBUSB_SUCCESS) {
        return libusb_to_freespace_error(rc);
    }
    if (length != count) {
        // libusb should never fragment the message.
        return FREESPACE_ERROR_UNEXPECTED;
    }

    return FREESPACE_SUCCESS;
}

int freespace_sendMessage(FreespaceDeviceId id,
                          struct freespace_message* message) {
    int rc;
    uint8_t msgBuf[FREESPACE_MAX_OUTPUT_MESSAGE_SIZE];
    struct FreespaceDeviceInfo info;
    
    // Address is reserved for now and must be set to 0 by the caller.
    if (message->dest == 0) {
        message->dest = FREESPACE_RESERVED_ADDRESS;
    }

    rc = freespace_getDeviceInfo(id, &info);
    if (rc != FREESPACE_SUCCESS) {
        return rc;
    }
    
    message->ver = info.hVer;
    rc = freespace_encode_message(message, msgBuf, FREESPACE_MAX_OUTPUT_MESSAGE_SIZE);
    if (rc <= FREESPACE_SUCCESS) {
        return rc;
    }
    
    return freespace_private_send(id, msgBuf, rc);
}

int freespace_private_read(FreespaceDeviceId id,
                           uint8_t* message,
                           int maxLength,
                           unsigned int timeoutMs,
                           int* actualLength) {
    struct FreespaceDevice* device = findDeviceById(id);
    struct FreespaceReceiveTransfer* rt;
    int rc;

    if (device == NULL || device->state_ != FREESPACE_OPENED) {
        return FREESPACE_ERROR_NOT_FOUND;
    }

    if (maxLength < device->maxReadSize_) {
        // Don't risk causing an overflow due to too small
        // a receive buffer.
        return FREESPACE_ERROR_RECEIVE_BUFFER_TOO_SMALL;
    }

    rt = &device->receiveQueue_[device->receiveQueueHead_];

    // Check if we need to wait.
    if (rt->submitted_ != 0) {
        struct timeval tv;

        tv.tv_sec = timeoutMs / 1000;
        tv.tv_usec = (timeoutMs % 1000) * 1000;

        // Wait.
        do {
            rc = libusb_handle_events_timeout(freespace_libusb_context, &tv);
            if (rc != LIBUSB_SUCCESS) {
                return libusb_to_freespace_error(rc);
            }

            // Keep trying until something has been received.
            // Note that libusb_handle_events_timeout could return
            // without a receive if it ends up doing some other
            // processing such as an async send completion or
            // something on another device.

            // TODO: update tv with time left.
            timeoutMs = 0;
        } while (rt->submitted_ != 0 && timeoutMs > 0);

        if (rt->submitted_ != 0) {
            return LIBUSB_ERROR_TIMEOUT;
        }
    }

    // Copy the message out.
    *actualLength = rt->transfer_->actual_length;
    memcpy(message, rt->buffer_, *actualLength);
    rc = libusb_transfer_status_to_freespace_error(rt->transfer_->status);

    // Resubmit the transfer
    rt->submitted_ = 1;
    libusb_submit_transfer(rt->transfer_);
    device->receiveQueueHead_++;
    if (device->receiveQueueHead_ >= FREESPACE_RECEIVE_QUEUE_SIZE) {
        device->receiveQueueHead_ = 0;
    }

    return rc;
}

int freespace_readMessage(FreespaceDeviceId id,
                          struct freespace_message* message,
                          unsigned int timeoutMs) {
    int rc;
    uint8_t buffer[FREESPACE_MAX_INPUT_MESSAGE_SIZE];
    int actLen;
    struct FreespaceDeviceInfo info;
    
    rc = freespace_getDeviceInfo(id, &info);
    if (rc != FREESPACE_SUCCESS) {
        return rc;
    }
    
    rc = freespace_private_read(id, buffer, sizeof(buffer), timeoutMs, &actLen);
    
    if (rc == FREESPACE_SUCCESS) {
        return freespace_decode_message(buffer, actLen, message, info.hVer);
    } else {
        return rc;
    }
}

int freespace_flush(FreespaceDeviceId id) {
    struct FreespaceDevice* device = findDeviceById(id);
    struct FreespaceReceiveTransfer* rt;
    struct timeval tv;
    int repeat;
    int maxRepeats = FREESPACE_RECEIVE_QUEUE_SIZE * 2;

    if (device == NULL || device->state_ != FREESPACE_OPENED) {
        return FREESPACE_ERROR_NOT_FOUND;
    }


    // As long as there's work, try again.
    do {
        // Poll libusb to give it a chance to unload as many
        // events as possible.
        tv.tv_sec = 0;
        tv.tv_usec = 0;
        libusb_handle_events_timeout(freespace_libusb_context, &tv);

        repeat = 0;

        // Clear out our queue.
        rt = &device->receiveQueue_[device->receiveQueueHead_];
        while (rt->submitted_ == 0) {
            rt->submitted_ = 1;
            libusb_submit_transfer(rt->transfer_);
            device->receiveQueueHead_++;
            if (device->receiveQueueHead_ >= FREESPACE_RECEIVE_QUEUE_SIZE) {
                device->receiveQueueHead_ = 0;
            }

            rt = &device->receiveQueue_[device->receiveQueueHead_];
            repeat = 1;
        }

        maxRepeats--;
    } while (repeat > 0 && maxRepeats > 0);

    return FREESPACE_SUCCESS;
}

struct SendTransferInfo {
    FreespaceDeviceId id;
    freespace_sendCallback callback;
    void* cookie;
};

static void sendCallback(struct libusb_transfer* transfer) {
    struct SendTransferInfo* info = (struct SendTransferInfo*) transfer->user_data;
    int rc = libusb_transfer_status_to_freespace_error(transfer->status);
    info->callback(info->id, info->cookie, rc);

    free(info);
}

int freespace_private_sendAsync(FreespaceDeviceId id,
                                const uint8_t* message,
                                int length,
                                unsigned int timeoutMs,
                                freespace_sendCallback callback,
                                void* cookie) {
#ifdef __APPLE__
    // @TODO: Figure out why libusb on darwin doesn't seem to work with asynchronous messages
    int rc;

    rc = freespace_private_send(id, message, length);
    if (callback != NULL) {
        callback(id, cookie, rc);
    }

    return libusb_to_freespace_error(rc);
#else
    struct FreespaceDevice* device;
    device = findDeviceById(id);
    struct libusb_transfer* transfer;
    int rc;

    if (device == NULL || device->state_ != FREESPACE_OPENED) {
        return FREESPACE_ERROR_NOT_FOUND;
    }

    if (length > device->maxWriteSize_) {
        return FREESPACE_ERROR_SEND_TOO_LARGE;
    }

    transfer = libusb_alloc_transfer(0);
    if (transfer == NULL) {
        return FREESPACE_ERROR_OUT_OF_MEMORY;
    }

    transfer->dev_handle = device->handle_;
    transfer->endpoint = device->writeEndpointAddress_;
    transfer->type = LIBUSB_TRANSFER_TYPE_INTERRUPT;
    transfer->timeout = timeoutMs;
    transfer->buffer = (unsigned char*) message;
    transfer->length = length;
    transfer->flags = LIBUSB_TRANSFER_FREE_TRANSFER;

    if (callback != NULL) {
        struct SendTransferInfo* info = (struct SendTransferInfo*) malloc(sizeof(struct SendTransferInfo));
    	if (info == NULL) {
    	    libusb_free_transfer(transfer);
            return FREESPACE_ERROR_OUT_OF_MEMORY;
    	}
    	info->id = id;
        info->callback = callback;
        info->cookie = cookie;
    	transfer->callback = sendCallback;
    	transfer->user_data = info;
    } else {
    	transfer->callback = NULL;
    	transfer->user_data = NULL;
    }

    rc = libusb_submit_transfer(transfer);

    return libusb_to_freespace_error(rc);
#endif
}

int freespace_sendMessageAsync(FreespaceDeviceId id,
                               struct freespace_message* message,
                               unsigned int timeoutMs,
                               freespace_sendCallback callback,
                               void* cookie) {

    int rc;
    uint8_t msgBuf[FREESPACE_MAX_OUTPUT_MESSAGE_SIZE];
    struct FreespaceDeviceInfo info;
    
    // Address is reserved for now and must be set to 0 by the caller.
    if (message->dest == 0) {
        message->dest = FREESPACE_RESERVED_ADDRESS;
    }
    
    rc = freespace_getDeviceInfo(id, &info);
    if (rc != FREESPACE_SUCCESS) {
        return rc;
    }
    
    message->ver = info.hVer;
    rc = freespace_encode_message(message, msgBuf, FREESPACE_MAX_OUTPUT_MESSAGE_SIZE);
    if (rc <= FREESPACE_SUCCESS) {
        return rc;
    }

    return freespace_private_sendAsync(id, msgBuf, rc, timeoutMs, callback, cookie);
}

int freespace_getNextTimeout(int* timeoutMsOut) {
    struct timeval tv;
    int hotplugTimeout = freespace_hotplug_timeout();
    int timeoutMs;

    int rc = libusb_get_next_timeout(freespace_libusb_context, &tv);
    if (rc == 1) {
        // libusb has a timeout
        timeoutMs = tv.tv_sec * 1000 + tv.tv_usec / 1000;

        if (hotplugTimeout > 0 && hotplugTimeout < timeoutMs) {
            timeoutMs = hotplugTimeout;
        }
    } else if (rc == 0 && hotplugTimeout > 0) {
        // The hotplug code has a timeout.
        timeoutMs = hotplugTimeout;
    } else {
        // No one has a timeout.
        timeoutMs = -1;
    }
    *timeoutMsOut = timeoutMs;
    return libusb_to_freespace_error(rc);
}

int freespace_perform() {
    struct timeval tv = {0, 0};
    int rc;

    scanDevices();

    rc = libusb_handle_events_timeout(freespace_libusb_context, &tv);
    return libusb_to_freespace_error(rc);
}

static void pollfd_added_cb(int fd, short events, void* user_data) {
    if (userAddedCallback != NULL) {
        userAddedCallback(fd, events);
    }
}
static void pollfd_removed_cb(int fd, void* user_data) {
    if (userRemovedCallback != NULL) {
        userRemovedCallback(fd);
    }
}

void freespace_setFileDescriptorCallbacks(freespace_pollfdAddedCallback addedCallback,
                                          freespace_pollfdRemovedCallback removedCallback) {
    userAddedCallback = addedCallback;
    userRemovedCallback = removedCallback;

    libusb_set_pollfd_notifiers(freespace_libusb_context, pollfd_added_cb, pollfd_removed_cb, NULL);
}

int freespace_syncFileDescriptors() {
    const struct libusb_pollfd** usbfds;
    int i;

    if (userAddedCallback == NULL) {
        return FREESPACE_SUCCESS;
    }

    // Add the hotplug code's fd
    userAddedCallback(freespace_hotplug_getFD(), POLLIN);

    // Add all of libusb's handles
    usbfds = libusb_get_pollfds(freespace_libusb_context);
    for (i = 0; usbfds[i] != NULL; i++) {
        userAddedCallback(usbfds[i]->fd, usbfds[i]->events);
    }
    free(usbfds);

    return FREESPACE_SUCCESS;
}

int freespace_private_setReceiveCallback(FreespaceDeviceId id,
                                         freespace_receiveCallback callback,
                                         void* cookie) {
    struct FreespaceDevice* device = findDeviceById(id);
    int wereInSyncMode;

    if (device == NULL) {
        return FREESPACE_ERROR_NOT_FOUND;
    }

    wereInSyncMode = (device->receiveCallback_ == NULL && device->receiveMessageCallback_ == NULL);
    device->receiveCallback_ = callback;
    device->receiveCookie_ = cookie;

    if (callback != NULL && wereInSyncMode && device->state_ == FREESPACE_OPENED) {
        // Transition from sync mode to async mode.

        // Need to run the callback on all received messages.
        struct FreespaceReceiveTransfer* rt;
        rt = &device->receiveQueue_[device->receiveQueueHead_];
        while (rt->submitted_ == 0) {
            callback(device->id_,
                     (const uint8_t*) rt->buffer_,
                     rt->transfer_->actual_length,
                     cookie,
                     libusb_transfer_status_to_freespace_error(rt->transfer_->status));

            rt->submitted_ = 1;
            libusb_submit_transfer(rt->transfer_);
            device->receiveQueueHead_++;
            if (device->receiveQueueHead_ >= FREESPACE_RECEIVE_QUEUE_SIZE) {
                device->receiveQueueHead_ = 0;
            }

            rt = &device->receiveQueue_[device->receiveQueueHead_];
        }
    }
    return FREESPACE_SUCCESS;
}

int freespace_setReceiveMessageCallback(FreespaceDeviceId id,
                                        freespace_receiveMessageCallback callback,
                                        void* cookie) {
    struct FreespaceDevice* device = findDeviceById(id);
    int wereInSyncMode;
    int rc;

    if (device == NULL) {
        return FREESPACE_ERROR_NOT_FOUND;
    }

    wereInSyncMode = (device->receiveMessageCallback_ == NULL && device->receiveCallback_ == NULL);
    device->receiveMessageCallback_ = callback;
    device->receiveMessageCookie_ = cookie;

    if (callback != NULL && wereInSyncMode && device->state_ == FREESPACE_OPENED) {
        struct freespace_message m;
        
        // Transition from sync mode to async mode.

        // Need to run the callback on all received messages.
        struct FreespaceReceiveTransfer* rt;
        rt = &device->receiveQueue_[device->receiveQueueHead_];
        while (rt->submitted_ == 0) {
            rc = freespace_decode_message((const uint8_t*) rt->buffer_, rt->transfer_->actual_length, &m, device->api_->hVer_);
            if (rc == FREESPACE_SUCCESS) {
                callback(device->id_,
                         &m,
                         cookie,
                         libusb_transfer_status_to_freespace_error(rt->transfer_->status));
            } else {
                callback(device->id_,
                         NULL,
                         cookie,
                         rc);
            }

            rt->submitted_ = 1;
            libusb_submit_transfer(rt->transfer_);
            device->receiveQueueHead_++;
            if (device->receiveQueueHead_ >= FREESPACE_RECEIVE_QUEUE_SIZE) {
                device->receiveQueueHead_ = 0;
            }

            rt = &device->receiveQueue_[device->receiveQueueHead_];
        }
    }
    return FREESPACE_SUCCESS;
}

