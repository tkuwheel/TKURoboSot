/*
 * This file is part of libfreespace.
 *
 * Copyright (c) 2012-2013 Hillcrest Laboratories, Inc.
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
#include "freespace_config.h"

#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <poll.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <dirent.h>

#include <linux/types.h>
#include <linux/input.h>
#include <linux/hidraw.h>
#include <sys/inotify.h>

/**
 * TODO
 *    - bluetooth write support on Ubunutu/hidp
 * 	  - add sync support?
 *    - better device suppport
 */

//#define _FREESPACE_WARN
//#define _FREESPACE_DEBUG
//#define _FREESPACE_TRACE

#define LOGF(fmt, lvl, ...) fprintf(stderr, "libfreespace (%s:%d): " #lvl " " fmt "\n", __func__, __LINE__, ##__VA_ARGS__);

#ifdef _FREESPACE_WARN
#define WARN(fmt, ...) LOGF(fmt,  WARN, ##__VA_ARGS__)
#else
#define WARN(...)
#endif

#ifdef _FREESPACE_DEBUG
#define DEBUG(fmt, ...) LOGF(fmt, DEBUG, ##__VA_ARGS__)
#else
#define DEBUG(...)
#endif

#ifdef _FREESPACE_TRACE
#define TRACE(fmt, ...) LOGF(fmt, TRACE, ##__VA_ARGS__)
#else
#define TRACE(...)
#endif

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
	FREESPACE_NONE,
    FREESPACE_CONNECTED,
    FREESPACE_OPENED,
    FREESPACE_DISCONNECTED,
};

struct FreespaceDevice {
    FreespaceDeviceId id_;
    enum FreespaceDeviceState state_;

    int fd_;
    int num_;
    char hidrawPath_[16];
    struct FreespaceDeviceAPI const * api_;

    freespace_receiveCallback receiveCallback_;
    freespace_receiveMessageCallback receiveMessageCallback_;
    void* receiveCookie_;
    void* receiveMessageCookie_;
};

#define DEV_DIR "/dev/"
#define HIDRAW_PREFIX  "hidraw"

#define GET_DEVICE(id, device) \
	struct FreespaceDevice* device = findDeviceById(id); \
	if (device == NULL) { \
		return FREESPACE_ERROR_INVALID_DEVICE; \
	}

#define GET_DEVICE_IF_OPEN(id, device) \
	GET_DEVICE(id, device) \
	switch (device->state_) { \
		case FREESPACE_OPENED: \
			break; \
		case FREESPACE_CONNECTED: \
			 /* no error code for "not open" ? */ \
		case FREESPACE_DISCONNECTED: \
			return FREESPACE_ERROR_NO_DEVICE; \
		default:\
			return FREESPACE_ERROR_UNEXPECTED;\
	}

/* global variables */
static int numDevices = 0;
static FreespaceDeviceId nextFreeIndex = 0;
static struct FreespaceDevice* devices[FREESPACE_MAXIMUM_DEVICE_COUNT];
static int inotify_fd_ = -1;
static int inotify_wd_ = -1;

static freespace_pollfdAddedCallback userAddedCallback = NULL;
static freespace_pollfdRemovedCallback userRemovedCallback = NULL;
static freespace_hotplugCallback hotplugCallback = NULL;
static void* hotplugCookie;

/* local functions */
static int _init_inotify();
static int _scanDevices();
static int _pollDevice(struct FreespaceDevice * device);
static int _disconnect(struct FreespaceDevice * device);
static void _deallocateDevice(struct FreespaceDevice* device);
static int _write(int fd, const uint8_t* message, int length);

static int connectedDevices_ = 0;

#ifdef LIBFRESPACE_THREADED_WRITES
#include <pthread.h>

pthread_t writeThread_;
pthread_mutex_t writeMutex_ = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t  writeCond_ = PTHREAD_COND_INITIALIZER;

struct WriteJob {
	struct FreespaceDevice * dev;
	uint8_t message[FREESPACE_MAX_INPUT_MESSAGE_SIZE];
	int length;
	struct WriteJob * next;
};

struct WriteJob * writeJobsHead_ = 0;
struct WriteJob * writeJobsTail_ = 0;
struct WriteJob * freeJobsHead_ = 0;
static int writeThreadExit_  = 0;
static int numWriteJobsAllocated_ = 0;
static int numWriteJobsFree_ = 0;
static const int MaxWriteJobs = 5;
static const int MaxFreeWriteJobs = 3;

/* Allocate a WriteJob struct. Call with lock held */
static struct WriteJob * _allocateWriteJob();
/* Deallocate a WriteJob struct. Call with lock held */
static int _deallocateWriteJob(struct WriteJob *);
/* Pop the next WriteJob from the write queue. Call with lock held */
static struct WriteJob * _popWriteJob();
/* Push a write job to the write_queue. Call with lock held */
static int _pushWriteJob(struct WriteJob *);
/* Flush all write jobs associated with *devCall with lock held */
static void _flushWriteJobs(struct FreespaceDevice * dev);
/* pthread function for write queue */
static void * _writeThread_fn(void * ptr);
#endif

static int _scanAllDevices();

const char* freespace_version() {
    return LIBFREESPACE_VERSION;
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

int freespace_init() {
	int rc = 0;
	memset(&devices, 0, sizeof(devices));
	rc = _init_inotify();
	if (rc != 0) {
		return FREESPACE_ERROR_IO;
	}

#ifdef LIBFRESPACE_THREADED_WRITES
	rc = pthread_create( &writeThread_, NULL, &_writeThread_fn, NULL);
//	pthread_setname_np(writeThread_, "libfreespace-write");

	if (rc < 0) {
		WARN("pthread_create failed: %s", strerror(errno));
		return FREESPACE_ERROR_COULD_NOT_CREATE_THREAD;
	}
#endif

    return FREESPACE_SUCCESS;
}

void freespace_exit() {
	int i;
    for (i = 0; i < FREESPACE_MAXIMUM_DEVICE_COUNT; i++) {
    	struct FreespaceDevice * device = devices[i];
    	if (device == NULL) {
    		continue;
    	}

    	if (device->state_ != FREESPACE_DISCONNECTED) {
    		_disconnect(device);
		}

    	if (devices[i]) {
    		_deallocateDevice(devices[i]);
    	}
    }

    if (inotify_fd_ > 0) {
		if (userRemovedCallback) {
			userRemovedCallback(inotify_fd_);
		}
    }

#ifdef LIBFRESPACE_THREADED_WRITES
    // Signal the thread to shutdown...
    writeThreadExit_ = 1;
    pthread_cond_signal(&writeCond_);

    pthread_join(writeThread_, NULL);
	pthread_mutex_destroy(&writeMutex_);
    pthread_cond_destroy(&writeCond_);
#endif

    return;
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

    rc = _scanDevices();
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
    GET_DEVICE(id, device);

	info->vendor = device->api_->idVendor_;
	info->product = device->api_->idProduct_;
	info->name = device->api_->name_;
	info->hVer = device->api_->hVer_;
	return FREESPACE_SUCCESS;
}

// this hidraw impl handles only async messages
int freespace_openDevice(FreespaceDeviceId id) {
	GET_DEVICE(id, device);

    if (device->state_ == FREESPACE_DISCONNECTED) {
    	return FREESPACE_ERROR_NO_DEVICE;
    }

    if (device->state_ == FREESPACE_OPENED) {
        return FREESPACE_SUCCESS;
    }

    if (device->state_ != FREESPACE_CONNECTED) {
    	return FREESPACE_ERROR_UNEXPECTED;
    }

    device->fd_ = open(device->hidrawPath_, O_RDWR | O_NONBLOCK);
    if (device->fd_ < 0) {
    	WARN("Failed opening %s: %s", device->hidrawPath_, strerror(errno));
    	return FREESPACE_ERROR_IO;
    }

    // flush the device
    uint8_t buf[1024];
    while (read(device->fd_, buf, sizeof(buf)) > 0);

    if (userAddedCallback) {
    	userAddedCallback(device->fd_, POLLIN);
    }

    device->state_ = FREESPACE_OPENED;
    return FREESPACE_SUCCESS;
}

void freespace_closeDevice(FreespaceDeviceId id) {
	struct FreespaceDevice* device = findDeviceById(id);
	if (device == NULL) {
		DEBUG("closeDevice() -- failed to get device %d", id);
		return;
	}

	if (device->state_ == FREESPACE_CONNECTED) {
		TRACE("Close closed device");
		// not open
		return;
	}

	if (device->state_ == FREESPACE_OPENED) {
		DEBUG("Close opened device");
#ifdef LIBFRESPACE_THREADED_WRITES
		pthread_mutex_lock(&writeMutex_ );
		_flushWriteJobs(device);
		pthread_mutex_unlock(&writeMutex_);
#endif
		// return the device to the "connected" state
		if (device->fd_ > 0) {
			if (userRemovedCallback) {
				userRemovedCallback(device->fd_);
			}
			close(device->fd_);
			device->fd_ = -1;
		}
		device->state_ = FREESPACE_CONNECTED;
		return;
	}

	if (device->state_ == FREESPACE_DISCONNECTED) {
		DEBUG("Close device already Disconnected");
		// device is disconnected...
		// we've been waiting for this close() to deallocate it.
		_deallocateDevice(device);
		return;
	}
	DEBUG("Closed device %d", id);
}

int freespace_isNewDevice(FreespaceDeviceId id) {
    const struct FreespaceDeviceInfo*   pDeviceInfo = NULL;
    int rc;
    int idx;
    struct FreespaceDevice* device = findDeviceById(id);

    if (device == NULL) {
        return FREESPACE_ERROR_NO_DEVICE;
    }

    // Determine if the product ID represent a new device
    for (idx = 0; idx < freespace_newDeviceAPITableNum; ++idx)
    {
        pDeviceInfo = &freespace_newDeviceAPITable[idx];
        if ( (pDeviceInfo->vendor == device->api_->idVendor_) &&
             (pDeviceInfo->product == device->api_->idProduct_) )
        {
            return FREESPACE_SUCCESS;
        }
    }
    return FREESPACE_ERROR_NO_DEVICE;
}

int freespace_private_send(FreespaceDeviceId id, const uint8_t* message, int length) {
	return FREESPACE_ERROR_UINIMPLEMENTED;
}

int freespace_sendMessage(FreespaceDeviceId id, struct freespace_message* message) {
    int rc;
    uint8_t msgBuf[FREESPACE_MAX_OUTPUT_MESSAGE_SIZE];
    GET_DEVICE_IF_OPEN(id, device);
    
    // Address is reserved for now and must be set to 0 by the caller.
    if (message->dest == 0) {
        message->dest = FREESPACE_RESERVED_ADDRESS;
    }
    
    message->ver = device->api_->hVer_;

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
    int rc;
    GET_DEVICE_IF_OPEN(id, device);

	// TODO
    return FREESPACE_ERROR_UINIMPLEMENTED;
}

int freespace_readMessage(FreespaceDeviceId id,
                          struct freespace_message* message,
                          unsigned int timeoutMs) {
	GET_DEVICE_IF_OPEN(id, device);
    return FREESPACE_ERROR_UINIMPLEMENTED;

}

int freespace_flush(FreespaceDeviceId id) {
	// TODO
    return FREESPACE_ERROR_UINIMPLEMENTED;

}

int _write(int fd, const uint8_t* message, int length) {
	int rc = write(fd, message, length);
    if (rc < 0) {
    	if (errno == ENOENT || errno == ENODEV) {
    		// disconnected.... hotplug will catch this later
    		return FREESPACE_ERROR_NO_DEVICE;
    	}

		if (errno == ETIMEDOUT) {
			FREESPACE_ERROR_TIMEOUT;
		}

    	WARN("Write failed: %s", strerror(errno));
    	return FREESPACE_ERROR_IO;
    }

    if (rc != length) {
    	WARN("Write failed. Wrote %d bytes of %d", rc, length);
    	return FREESPACE_ERROR_IO;
    }

    return FREESPACE_SUCCESS;
}

int freespace_private_sendAsync(FreespaceDeviceId id,
                                const uint8_t* message,
                                int length,
                                unsigned int timeoutMs,
                                freespace_sendCallback callback,
                                void* cookie) {
    ssize_t rc;
    GET_DEVICE_IF_OPEN(id, device);

#ifndef LIBFRESPACE_THREADED_WRITES
    return _write(device->fd_, message, length);
#else
	pthread_mutex_lock(&writeMutex_ );
    struct WriteJob * job = _allocateWriteJob();
    if (!job) {
    	rc = FREESPACE_ERROR_OUT_OF_MEMORY;
    } else {
		job->dev = device;
		memcpy(job->message, message, length);
		job->length = length;

		rc = _pushWriteJob(job);
    }

	pthread_mutex_unlock(&writeMutex_);
	pthread_cond_signal(&writeCond_);
    return rc;
#endif
}

int freespace_sendMessageAsync(FreespaceDeviceId id,
                               struct freespace_message* message,
                               unsigned int timeoutMs,
                               freespace_sendCallback callback,
                               void* cookie) {

    int rc;
    uint8_t msgBuf[FREESPACE_MAX_OUTPUT_MESSAGE_SIZE];
	GET_DEVICE_IF_OPEN(id, device);

    // Address is reserved for now and must be set to 0 by the caller.
    if (message->dest == 0) {
        message->dest = FREESPACE_RESERVED_ADDRESS;
    }
    message->ver = device->api_->hVer_;

    rc = freespace_encode_message(message, msgBuf, FREESPACE_MAX_OUTPUT_MESSAGE_SIZE);
    if (rc <= FREESPACE_SUCCESS) {
        return rc;
    }

    return freespace_private_sendAsync(id, msgBuf, rc, timeoutMs, callback, cookie);
}

int freespace_getNextTimeout(int* timeoutMsOut) {
	// TODO
    *timeoutMsOut = -1;
	return FREESPACE_SUCCESS;
}

int freespace_perform() {
    int i;
    int n;
    int nfds;
    static int needToRescan = 1;

	if (needToRescan) {
		_scanAllDevices();
		needToRescan = 0;
	}

    struct pollfd fds[FREESPACE_MAXIMUM_DEVICE_COUNT + 1];

    // Populate fds[]
    fds[0].fd = inotify_fd_;
    fds[0].events = POLLIN;

	for (i = 0, n = 0; n < numDevices && i < FREESPACE_MAXIMUM_DEVICE_COUNT; ++i) {
		struct FreespaceDevice * device = devices[i];
		if (!device) {
			continue;
		}

        ++n;
        fds[n].fd = device->fd_;
        fds[n].events = POLLIN | POLLHUP | POLLERR;
    }

    // Poll for open file descripts
    nfds = poll(fds, numDevices + 1, 0);

    i = 0;
    n = 0;
    if (nfds > 0 ) {
        // inotify_fd_
        if (fds[0].revents & POLLIN) {
            DEBUG("inotify_fd_ received POLLIN. Call _scanDevices(). nfds = %d", nfds);
            fds[0].revents = 0;
            --nfds;
	        _scanDevices();
	    }

        // Other device->fd_
        while (nfds > 0) {
            while (n < numDevices && i < FREESPACE_MAXIMUM_DEVICE_COUNT) {
	            struct FreespaceDevice * device = devices[i];
                if (!device) {
                    ++i;
                    continue;
                }
                ++n;
                ++i;

                if (fds[n].revents) {
                    if (fds[n].revents & (POLLHUP | POLLERR)) {
                        DEBUG("Disconnect device %d", device->id_);
                        _disconnect(device);
                    }
                    if (fds[n].revents & POLLIN) {
                        if (device->state_ == FREESPACE_OPENED) {
                            _pollDevice(device);
                        }
                    }
                    fds[n].revents = 0;
                    --nfds;
                    break;
                }
            }
        }
    } else if (nfds < 0 ) {
        WARN("poll() failed: %s", strerror(errno));
        return FREESPACE_ERROR_UNEXPECTED;
    }

    return FREESPACE_SUCCESS;
}

void freespace_setFileDescriptorCallbacks(freespace_pollfdAddedCallback addedCallback,
                                          freespace_pollfdRemovedCallback removedCallback) {
    userAddedCallback = addedCallback;
    userRemovedCallback = removedCallback;
}

int freespace_syncFileDescriptors() {
    int i;
	int n;

    if (userAddedCallback == NULL) {
        return FREESPACE_SUCCESS;
    }

    // Add the hotplug code's fd
    userAddedCallback(inotify_fd_, POLLIN);

	i = 0;
	n = 0;
	for (; n < numDevices && i < FREESPACE_MAXIMUM_DEVICE_COUNT; i++) {
		struct FreespaceDevice * device = devices[i];
		if (device) {
			n++;
			if (device->state_ == FREESPACE_OPENED) {
				// assert(device->fd_ > 0);
				userAddedCallback(device->fd_, POLLIN);
			}
		}
	}

    return FREESPACE_SUCCESS;
}

int freespace_private_setReceiveCallback(FreespaceDeviceId id,
                                         freespace_receiveCallback callback,
                                         void* cookie) {
    GET_DEVICE(id, device);

    device->receiveCallback_ = callback;
    device->receiveCookie_ = cookie;

    return FREESPACE_SUCCESS;
}

int freespace_setReceiveMessageCallback(FreespaceDeviceId id,
                                        freespace_receiveMessageCallback callback,
                                        void* cookie) {
    int rc;
    GET_DEVICE(id, device);

    device->receiveMessageCallback_ = callback;
    device->receiveMessageCookie_ = cookie;

    return FREESPACE_SUCCESS;
}

static int _pollDevice(struct FreespaceDevice * device) {
    ssize_t rc;
    uint8_t buf[FREESPACE_MAX_OUTPUT_MESSAGE_SIZE];

    while (1) {
		rc = read(device->fd_, buf, sizeof(buf));
		if (rc < 0) {
			if (errno == EAGAIN) {
				// no more data
				break;
			}

			if (errno == ENOENT || errno == ENODEV) {
				// disconnected.... hotplug will catch this later and notify
				return FREESPACE_ERROR_NO_DEVICE;
			}

			WARN("Failed reading %s: %s", device->hidrawPath_, strerror(errno));
			return FREESPACE_ERROR_IO;
		}

		if (rc == 0) { // EOF
			// disconnected.... hotplug will catch this later and notify
			return FREESPACE_ERROR_NO_DEVICE;
		}

		if (device->receiveCallback_) {
			device->receiveCallback_(device->id_, buf, (int) rc, device->receiveCookie_, FREESPACE_SUCCESS);
		}

		if (device->receiveMessageCallback_) {
			struct freespace_message m;

			rc = freespace_decode_message(buf, rc, &m, device->api_->hVer_);

			device->receiveMessageCallback_(
					device->id_,
					rc == FREESPACE_SUCCESS ? &m : NULL,
					device->receiveMessageCookie_, rc);
		}
    }
	return FREESPACE_SUCCESS;
}

// check if device at hidraw path is a Freespace device.
static int _isFreespaceDevice(const char * path, struct FreespaceDeviceAPI const ** API) {

	int i;
	int rc;
	struct hidraw_devinfo info;
	int fd = open(path, O_RDONLY);

	*API = 0;
	if (fd < 0) {
		if (errno != EAGAIN) {
			WARN("Failed opening %s: %s", path, strerror(errno));
			return FREESPACE_ERROR_IO;
		}
		return FREESPACE_SUCCESS;
	}

	rc = ioctl(fd, HIDIOCGRAWINFO, &info);
	if (rc < 0) {
		WARN("Failed getting hidraw info for %s: %s", path, strerror(errno));
		close(fd);
		return FREESPACE_ERROR_IO;
	}

	for (i = 0; i < freespace_deviceAPITableNum; i++) {
		if (freespace_deviceAPITable[i].idVendor_ != info.vendor) {
			continue;
		}

		if (freespace_deviceAPITable[i].idProduct_ != (info.product & 0xffff )) {
			continue;
		}

		{
			struct hidraw_report_descriptor descriptor;
			rc = ioctl(fd, HIDIOCGRDESCSIZE, &descriptor.size);
			if (rc < 0) {
				DEBUG("HIDIOCGRDESCSIZE %s: %s", path, strerror(errno));
				close(fd);
				return FREESPACE_ERROR_IO;
			}
			rc = ioctl(fd, HIDIOCGRDESC, &descriptor);
			if (rc < 0) {
				DEBUG("HIDIOCGRDESC %s: %s", path, strerror(errno));
				close(fd);
				return FREESPACE_ERROR_IO;
			}

			TRACE("%s  - descriptor size: %d",  path,  descriptor.size);

			// TODO parse descriptors and do something intelligent....
			// Really, really crude matching for now..
			if (descriptor.size == 161) { // scoop
				*API = &freespace_deviceAPITable[i];
			} else if (descriptor.size == 174) { // Roku Broadcom Remote
				*API = &freespace_deviceAPITable[i];
			} else if (descriptor.size == 229) { // Ozmo Remote
				*API = &freespace_deviceAPITable[i];
			} else if (descriptor.size == 199) { // Ozmo Remote
				*API = &freespace_deviceAPITable[i];
			} else if (descriptor.size == 93) {  // TI Remote
				*API = &freespace_deviceAPITable[i];
			} else if (descriptor.size == 178) { // RC520
				*API = &freespace_deviceAPITable[i];
			}
		}
		break;
	}

	close(fd);
	return FREESPACE_SUCCESS;
}

static int _allocateNewDevice(struct FreespaceDevice** out_device) {
	struct FreespaceDevice* device;
	*out_device = 0;

	if (nextFreeIndex == -1) {
        return FREESPACE_ERROR_OUT_OF_MEMORY;
    }

	device = (struct FreespaceDevice*) malloc(sizeof(struct FreespaceDevice));
	if (device == NULL) {
		// Out of memory.
		return FREESPACE_ERROR_OUT_OF_MEMORY;
	}
	memset(device, 0, sizeof(struct FreespaceDevice));
    numDevices++;

    devices[nextFreeIndex] = device;
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

    * out_device = device;
    return FREESPACE_SUCCESS;
}

// check if /dev/hidraw<num> path already belongs to an existing device
static int _isNewDevice(int num, int * isNew) {

	int i = 0;
	int n = 0;

	for (; n < numDevices && i < FREESPACE_MAXIMUM_DEVICE_COUNT; i++) {
		struct FreespaceDevice * device = devices[i];

		if (device == 0) {
			continue;
		}

		n++;
		if (device->num_ != num) {
			continue;
		}

		switch (device->state_) {
			case FREESPACE_OPENED:
			case FREESPACE_CONNECTED:
				*isNew = 0; // /dev/hidraw<num>is already known
				return FREESPACE_SUCCESS;

			case FREESPACE_DISCONNECTED:
				*isNew = 1; // this is a "ghost" device that close() has not been called on.
				return FREESPACE_SUCCESS;

			default:
				break;
		}
	}

	// does not match one of our current devices
	*isNew = 1;
	return FREESPACE_SUCCESS;
}

// return <num> from /dev/hidraw<num>
static int _parseNum(const char * path) {
	return atoi(path + sizeof(HIDRAW_PREFIX) - 1);
}

static FreespaceDeviceId _assignId() {
    int i;
    
    for (i = 0; i < FREESPACE_MAXIMUM_DEVICE_COUNT; ++i) {
        if ((connectedDevices_ & (1 << i)) == 0) {
            connectedDevices_ |= (1 << i);
            WARN("Device ID %d is connected", i);
            return i;
        }
    }
    return -1;
}

static int _scanDevice(const char * dir, const char * filename) {

	int num;
	int isNew = 0;
	char abspath[NAME_MAX];
	struct FreespaceDeviceAPI const * API = 0;

    num = _parseNum(filename);
	_isNewDevice(num, &isNew);

	if (!isNew) {
		return 0;
	}

    // append the file name and store an abs path
	snprintf(abspath, sizeof(abspath), "%s%s", dir, filename);

    _isFreespaceDevice(abspath, &API);

	if (!API) {
		TRACE("Not a freespace device: %s", abspath);
		return 0;
	}

	// allocate a device
	{

		struct FreespaceDevice * device;
		int rc = _allocateNewDevice(&device);
		if (rc != FREESPACE_SUCCESS) {
			return rc;
		}

		device->state_ = FREESPACE_CONNECTED;
		device->fd_ = -1;
	    device->id_ = _assignId();
	    device->num_ = num;
		strncpy(device->hidrawPath_, abspath, sizeof(device->hidrawPath_));
		device->api_ = API;

        if (hotplugCallback) {
            hotplugCallback(FREESPACE_HOTPLUG_INSERTION, device->id_, hotplugCookie);
        }
	}

	DEBUG("Found freespace device at %s. ** Num devices: %d **", abspath, numDevices);
	return 0;
}

static int _scanAllDevices() {
	TRACE("Scanning all hidraw devices");
	// Check if a device has been added (iterate all of /dev)
	DIR*    dev_dir = opendir(DEV_DIR);
	if (dev_dir) {
        struct dirent*  ent;

        while ( (ent = readdir(dev_dir)) != NULL ) {
            if (strncmp(ent->d_name, HIDRAW_PREFIX, strlen(HIDRAW_PREFIX)) != 0) {
            	continue;
            }

        	_scanDevice(DEV_DIR, ent->d_name);
        }
	} else {
		WARN("Failed opening %s", DEV_DIR);
	}

	// TODO handle the case where devices drop when in the "connected" but not "opened" state...
	closedir(dev_dir);
	return FREESPACE_SUCCESS;
}

static int _init_inotify() {
	int rc;

	inotify_fd_ = inotify_init();
	if (inotify_fd_ < 0) {
		WARN("Failed inotify_init: %s", strerror(errno));
		return FREESPACE_ERROR_IO;
	}

	rc = fcntl(inotify_fd_, F_SETFL, O_NONBLOCK);  // set to non-blocking
	if (rc < 0) {
		WARN("Failed inotify -> non block: %s", strerror(errno));
		return FREESPACE_ERROR_IO;
	}

	inotify_wd_ = inotify_add_watch(inotify_fd_, DEV_DIR, IN_CREATE | IN_DELETE);
	if (inotify_wd_ < 0) {
		WARN("Failed inotify_add_watch: %s", strerror(errno));
		return FREESPACE_ERROR_IO;
	}

	if (userAddedCallback) {
		userAddedCallback(inotify_fd_, POLLIN);
	}
	return 0;
}

static struct FreespaceDevice * _findDeviceByHidrawNum(int num) {
	int i = 0;
	int n = 0;
	for (; n < numDevices && i < FREESPACE_MAXIMUM_DEVICE_COUNT; i++) {
		struct FreespaceDevice * device = devices[i];

		if (!device) {
			continue;
		}

		n++;
		if (device->state_!= FREESPACE_OPENED && device->state_!= FREESPACE_CONNECTED) {
			continue;
		}

		if (num == device->num_) {
			return device;
		}
	}

	return NULL;
}

static int _scanDevices() {
	int rc;

	// process inotify events
	char buf[(sizeof(struct inotify_event) + 32) * 16];
	ssize_t length = 0;
	ssize_t offset = 0;

	while(1) {
		if (offset > 0) {
			// move the trailing bytes to the beginning
			memmove(buf, buf + offset, length - offset);
		}

		length = read(inotify_fd_, buf + offset, sizeof(buf) - offset);
		if (length < 0) {
			if (errno == EAGAIN) {
				// done!
				return FREESPACE_SUCCESS;
			}
			return FREESPACE_ERROR_IO;
		}
		//TRACE("Inotify: read %d/%d bytes", length, sizeof(buf));

		// start at the beginning
		offset = 0;

		// process all events in our buffer
		while (offset < length) {

			struct inotify_event * event = (struct inotify_event *) (buf + offset);
			ssize_t remainder = length - offset;
			ssize_t expectedSize = sizeof(struct inotify_event);
			int num;

			if (event->wd != inotify_wd_) {
				WARN("Inotify watchdog does not match! -- %d != %d", event->wd, inotify_wd_);
			}

			// first, check to see if we have a complete inotify_event
			if (remainder < expectedSize) {
				// this event is incomplete (not aligned), break here and read again
				TRACE("Not enough space in the buffer for inotify struct... %d/%d", remainder, expectedSize);
				break;
			}

			// now, check to see if the path following it has been completely read.
			expectedSize += event->len;
			if (remainder < expectedSize) {
				TRACE("Not enough space in the buffer for inotify struct + string... %d/%d -- (%d + %d)", remainder, expectedSize, sizeof(struct inotify_event), event->len);
				break;
			}

			offset += expectedSize;

            if (strncmp(event->name, HIDRAW_PREFIX, strlen(HIDRAW_PREFIX)) != 0) {
            	TRACE("Skipping, inotify event - %04x:%s ", event->mask, event->name);
            	continue;
            }

			DEBUG("Handle inotify event - %04x:%s ", event->mask, event->name);
			num = _parseNum(event->name);

			if (event->mask & IN_CREATE) {
				struct FreespaceDevice * device = _findDeviceByHidrawNum(num);
				if (device) {
					TRACE("%s is alread added!", event->name);
					continue;
				}

#if 1
				// hack to allow udev some time...
				usleep(1000 * 500);
#endif
				_scanDevice(DEV_DIR, event->name);
				continue;
			}
		}
	}

	return FREESPACE_SUCCESS;

}

static void _deallocateDevice(struct FreespaceDevice* device) {
    int i;
    for (i = 0; i < FREESPACE_MAXIMUM_DEVICE_COUNT; i++) {
        if (devices[i] == device) {
            if (nextFreeIndex == -1) {
                nextFreeIndex = i;
            }

#if 1 // this should not be necessary.
        	if (device->fd_ > 0) {
        		DEBUG("Deallocate device (%s) -- fd still open!", device->hidrawPath_)
    			if (userRemovedCallback) {
    				userRemovedCallback(device->fd_);
    			}
    			close(device->fd_);
    			device->fd_ = -1;
    		}
#endif
            free(device);
            devices[i] = NULL;
            numDevices--;
			DEBUG("Freed device. ** Num devices: %d **", numDevices);
            return;
        }
    }

    WARN("Could not deallocate %p", device);
}

static int _disconnect(struct FreespaceDevice * device) {
	DEBUG("Freespace device (%d) at %s disconnected", device->id_, device->hidrawPath_);

#ifdef LIBFRESPACE_THREADED_WRITES
	pthread_mutex_lock(&writeMutex_ );
	_flushWriteJobs(device);
	pthread_mutex_unlock(&writeMutex_);
#endif
    // device is currently in use, we can't delete it outright
    if (device->state_ == FREESPACE_OPENED) {
    	if (device->fd_ > 0) {
			if (userRemovedCallback) {
				userRemovedCallback(device->fd_);
			}
			close(device->fd_);
			device->fd_ = -1;
		}

        // Indicate that the device is disconnected so that its ID can be reused
        connectedDevices_ &= ~((int)(1 << device->id_));
        WARN("Device ID %d is disconnected", device->id_);

		device->state_ = FREESPACE_DISCONNECTED;
		TRACE("*** Sending removal notification for device %d while opened", device->id_);
		if (hotplugCallback) {
			hotplugCallback(FREESPACE_HOTPLUG_REMOVAL, device->id_, hotplugCookie);
		}

		// we have to wait for closeDevice() to deallocate this device.
		return FREESPACE_SUCCESS;
    }

    if (device->state_ == FREESPACE_CONNECTED) {
    	int id = device->id_;
    	if (device->fd_ > 0) {
			if (userRemovedCallback) {
				userRemovedCallback(device->fd_);
			}
			close(device->fd_);
			device->fd_ = -1;
		}

        // Indicate that the device is disconnected so that its ID can be reused
        connectedDevices_ &= ~((int)(1 << device->id_));
        WARN("Device ID %d is disconnected", device->id_);

    	_deallocateDevice(device);
    	device = NULL;

		TRACE("*** Sending removal notification for device %d while connected", device->id_);
		if (hotplugCallback) {
			hotplugCallback(FREESPACE_HOTPLUG_REMOVAL, id, hotplugCookie);
		}

    	return FREESPACE_SUCCESS;
    }

	return FREESPACE_ERROR_UNEXPECTED;
}

#ifdef LIBFRESPACE_THREADED_WRITES

static struct WriteJob * _allocateWriteJob() {
	struct WriteJob * j;
	if (freeJobsHead_ != NULL) {
		j = freeJobsHead_;
		freeJobsHead_ = freeJobsHead_->next;
		j->next = NULL;
		return j;
	}

	if (numWriteJobsAllocated_ + 1 > MaxWriteJobs) {
		return NULL;
	}

	j = malloc(sizeof(struct WriteJob));
	if (j) {
		memset(j, 0, sizeof(*j));
		numWriteJobsAllocated_++;
		return j;
	}
	WARN("malloc failed", "");
	return NULL;
}

static int _deallocateWriteJob(struct WriteJob * j) {
	if (numWriteJobsFree_ + 1 > MaxFreeWriteJobs) {
		free(j);
		numWriteJobsAllocated_--;
		return 0;
	}

	numWriteJobsFree_++;
	j->next = freeJobsHead_;
	freeJobsHead_ = j;
	return FREESPACE_SUCCESS;
}


static struct WriteJob * _popWriteJob() {
	struct WriteJob * j = NULL;

	if (writeJobsHead_ != NULL) {
		j = writeJobsHead_;
		writeJobsHead_ = j->next;
		if (writeJobsHead_ == NULL) {
			writeJobsTail_ = NULL;
		}
	}

//	TRACE("Pop: %p Head: %p Tail: %p. Allocated: %d", j, writeJobsHead_, writeJobsTail_, numWriteJobsAllocated_);
	return j;
}

static int _pushWriteJob(struct WriteJob * j) {
	if (writeJobsTail_ == NULL) {
		writeJobsHead_ = j;
		writeJobsTail_ = j;
	} else {
		writeJobsTail_->next = j;
		writeJobsTail_ = j;
	}

//	TRACE("Pop: %p Head: %p Tail: %p. Allocated: %d", j, writeJobsHead_, writeJobsTail_, numWriteJobsAllocated_);
	return FREESPACE_SUCCESS;
}

static void _flushWriteJobs(struct FreespaceDevice * dev) {
	struct WriteJob * j = writeJobsHead_;
	struct WriteJob * prev = NULL;
	struct WriteJob * t = NULL;

	if (j == NULL) {
		return;
	}

	while (j) {
		if (j->dev != dev) {
			// keep this job
			prev = j;
			j = j->next;
		}

		// we have to remove j from the linked list
		if (prev == NULL) {
			writeJobsHead_ = j->next;
		}

		t = j->next;
		_deallocateWriteJob(j);
		if (prev) {
			prev->next = t;
		}
		j = t;
	}
}

static void * _writeThread_fn(void * ptr) {
	while (writeThreadExit_ == 0) {
		  struct WriteJob * j;

		 // Lock mutex and then wait for signal to relase mutex
		  pthread_mutex_lock(&writeMutex_ );
		  // wait for signal..
		  pthread_cond_wait(&writeCond_, &writeMutex_ );
		  while (writeThreadExit_ == 0 && (j = _popWriteJob()) != NULL) {
			  pthread_mutex_unlock(&writeMutex_);
			  _write(j->dev->fd_, j->message, j->length);
			  pthread_mutex_lock(&writeMutex_ );
			  _deallocateWriteJob(j);
		  }

		  pthread_mutex_unlock(&writeMutex_);
	}

	return 0;
}

#endif
