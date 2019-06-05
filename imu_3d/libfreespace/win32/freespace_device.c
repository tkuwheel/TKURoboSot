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

#include "freespace_device.h"
#include "freespace_deviceMgr.h"
#include "freespace_discovery.h"
#include <strsafe.h>
#include <malloc.h>

const int SEND_TIMEOUT = 1000; // in milliseconds
const unsigned long HID_NUM_INPUT_BUFFERS = 128;

/**
 * Initialize the send structure.
 *
 * @param send The structure to initialize.
 * @return FREESPACE_SUCCESS if ok.
 */
static int initializeSendStruct(struct FreespaceSendStruct* send);

/**
 * Finalize the send structure.
 * For asynchronous transactions, the structure is assumed to be recycled and the
 * associated event is kept alive.
 *
 * @param send The structure to finalize.
 * @param isExit True to close all open handles, false to recycle.
 * @return FREESPACE_SUCCESS
 */
static int finalizeSendStruct(struct FreespaceSendStruct* send, BOOL isClose);

/* Callback that is registered with BindIoCompletionCallback */
static VOID CALLBACK freespace_private_overlappedCallback(DWORD dwErrorCode,
                                                          DWORD dwNumberOfBytesTransfered,
                                                          LPOVERLAPPED lpOverlapped)
{
    SetEvent(freespace_instance_->performEvent_);
}


/**
 * Handle the case where the device unexpectedly fails.
 * @param device The device the failed.
 * @param ec The error code from GetLastError()
 * @return The device error code.
 */
int handleDeviceFailure(struct FreespaceDeviceStruct* device, int ec) {
    int rc = FREESPACE_ERROR_UNEXPECTED;
    int formatRc = 0;
    LPVOID lpMsgBuf;

    // Retrieve the system error message for the last-error code
    formatRc = FormatMessage(
        FORMAT_MESSAGE_ALLOCATE_BUFFER | 
        FORMAT_MESSAGE_FROM_SYSTEM |
        FORMAT_MESSAGE_IGNORE_INSERTS,
        NULL,
        ec,
        MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
        (LPTSTR) &lpMsgBuf,
        0, NULL );
    if (formatRc == 0) {
        DEBUG_WPRINTF(L"handleDeviceFailure: %d\n", ec);
    } else {
        // Display the error message and exit the process
        DEBUG_WPRINTF(L"handleDeviceFailure: %d => %s\n", ec, lpMsgBuf);
        LocalFree(lpMsgBuf);
    }

    // Clean up this device.
    freespace_private_removeDevice(device);
    freespace_private_forceCloseDevice(device);
    freespace_private_requestDeviceRescan();

    if (ec == ERROR_DEVICE_NOT_CONNECTED) {
        rc = FREESPACE_ERROR_NOT_FOUND;
    }

    return rc;
}


LIBFREESPACE_API int freespace_getDeviceInfo(FreespaceDeviceId id, struct FreespaceDeviceInfo* info) {
    struct FreespaceDeviceStruct* device = freespace_private_getDeviceById(id);
    if (device == NULL) {
        return FREESPACE_ERROR_NO_DEVICE;
    }
    info->name      = device->name_;
    info->product   = device->handle_[0].info_.idProduct_;
    info->vendor    = device->handle_[0].info_.idVendor_;
	info->hVer      = device->hVer_;

    return FREESPACE_SUCCESS;
}

LIBFREESPACE_API int freespace_isNewDevice(FreespaceDeviceId id) {
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

struct FreespaceDeviceStruct* freespace_private_createDevice(const char* name, const int hVer) {
    struct FreespaceDeviceStruct* device = (struct FreespaceDeviceStruct*) malloc(sizeof(struct FreespaceDeviceStruct));
    if (device == NULL) {
        return NULL;
    }
    memset(device, 0, sizeof(struct FreespaceDeviceStruct));

    // Assign the ID.
    device->id_ = freespace_instance_->nextDeviceId_;
    freespace_instance_->nextDeviceId_++;

    // Initialize the rest of the struct.
    device->status_ = FREESPACE_DISCOVERY_STATUS_UNKNOWN;
    device->name_ = name;
	device->hVer_ = hVer;
    device->isAvailable_ = FALSE;

    return device;
}

int freespace_private_freeDevice(struct FreespaceDeviceStruct* device) {
    int idx;

    // Try to close the device if it's still open.
    if (device->isOpened_) {
        freespace_closeDevice(device->id_);
    }

    // Free up everything allocated by addNewDevice.
    for (idx = 0; idx < device->handleCount_; idx++) {
        struct FreespaceSubStruct* s = &device->handle_[idx];
        if (s->devicePath != NULL) {
            free(s->devicePath);
        }
    }

    if (device->uniqueId_ != NULL) {
        free(device->uniqueId_);
    }

    // Free up everything allocated by freespace_private_createDevice
    free(device);
    return FREESPACE_SUCCESS;
}

void freespace_private_removeDevice(struct FreespaceDeviceStruct* device) {
    if (device->isAvailable_ == FALSE) {
        return;
    }
    device->isAvailable_ = FALSE;
    DEBUG_PRINTF("Device %d removed from API\n", device->id_);

    if (freespace_instance_->hotplugCallback_ != NULL) {
        freespace_instance_->hotplugCallback_(FREESPACE_HOTPLUG_REMOVAL, device->id_, freespace_instance_->hotplugCookie_);
    }
}

void freespace_private_insertDevice(struct FreespaceDeviceStruct* device) {
    if (device->isAvailable_ == TRUE) {
        return;
    }
    device->isAvailable_ = TRUE;
    DEBUG_PRINTF("Device %d inserted into API\n", device->id_);

    if (freespace_instance_->hotplugCallback_ != NULL) {
        freespace_instance_->hotplugCallback_(FREESPACE_HOTPLUG_INSERTION, device->id_, freespace_instance_->hotplugCookie_);
    }
}


struct FreespaceSendStruct* getNextSendBuffer(struct FreespaceDeviceStruct* device) {
    int i;
    struct FreespaceSendStruct* s;
    for (i = 0; i < FREESPACE_MAXIMUM_SEND_MESSAGE_COUNT; i++) {
        s = &device->send_[i];
        if (s->interface_ == NULL) {
            return s;
        }
    }
    return NULL;
}

static int initiateAsyncReceives(struct FreespaceDeviceStruct* device) {
    int idx;
    int funcRc = FREESPACE_SUCCESS;
    int rc;
	struct freespace_message m;

    // If no callback or not opened, then don't need to request to receive anything.
    if (!device->isOpened_ || (device->receiveCallback_ == NULL && device->receiveMessageCallback_ == NULL)) {
        return FREESPACE_SUCCESS;
    }

    // Initialize a new read operation on all handles that need it.
    for (idx = 0; idx < device->handleCount_; idx++) {
        struct FreespaceSubStruct* s = &device->handle_[idx];
        if (!s->readStatus_) {
            for (;;) {
                BOOL bResult = ReadFile(
					s->handle_,                 /* handle to device */
					s->readBuffer,              /* IN report buffer to fill */
					s->info_.inputReportByteLength_,  /* input buffer size */
					&s->readBufferSize,         /* returned buffer size */
					&s->readOverlapped_ );      /* long pointer to an OVERLAPPED structure */
                if (bResult) {
                    // Got something, so report it.
					if (device->receiveCallback_ || device->receiveMessageCallback_) {
						if (device->receiveCallback_) {
							device->receiveCallback_(device->id_, (char *) (s->readBuffer), s->readBufferSize, device->receiveCookie_, FREESPACE_SUCCESS);
						}
						if (device->receiveMessageCallback_) {
							rc = freespace_decode_message((char *) (s->readBuffer), s->readBufferSize, &m, device->hVer_);
							if (rc == FREESPACE_SUCCESS) {
								device->receiveMessageCallback_(device->id_, &m, device->receiveMessageCookie_, FREESPACE_SUCCESS);
							} else {
								device->receiveMessageCallback_(device->id_, NULL, device->receiveMessageCookie_, rc);
								DEBUG_PRINTF("freespace_decode_message failed with code %d\n", rc);
							}
						}
					} else {
                        // If no receiveCallback, then freespace_setReceiveCallback was called to stop
                        // receives from within the receiveCallback. Bail out to let it do its thing.
                        return FREESPACE_SUCCESS;
                    }
                } else {
                    // Error or would block - check below.
                    break;
                }
            }

            rc = GetLastError();
            if (rc == ERROR_IO_PENDING) {
                // We got a ReadFile to block, so mark it.
                s->readStatus_ = TRUE;
            } else {
                // Something severe happened to our device!
				if (device->receiveCallback_) {
				    device->receiveCallback_(device->id_, NULL, 0, device->receiveCookie_, rc);
				}
				if (device->receiveMessageCallback_) {
				    device->receiveMessageCallback_(device->id_, NULL, device->receiveMessageCookie_, rc);
				}
                DEBUG_PRINTF("initiateAsyncReceives : Error on %d : %d\n", idx, rc);
                return handleDeviceFailure(device, rc);
            }
        }
    }

    return funcRc;
}

int freespace_private_devicePerform(struct FreespaceDeviceStruct* device) {
    int idx;
    BOOL overlappedResult;
    struct FreespaceSendStruct* send;
	int rc;
	struct freespace_message m;

    // Handle the send messages
    for (idx = 0; idx < FREESPACE_MAXIMUM_SEND_MESSAGE_COUNT; idx++) {
        send = &device->send_[idx];
        if (send->interface_ == NULL) {
            continue;
        }
        overlappedResult = GetOverlappedResult(
                                               send->interface_->handle_,
                                               &send->overlapped_,
                                               &send->numBytes_,
                                               FALSE);

        if (!overlappedResult) {
            // No message available yet.
            continue;
        } else if (send->numBytes_ != send->interface_->info_.outputReportByteLength_) {
            // Unexpected error on the sent message.
            DEBUG_PRINTF("freespace_private_devicePerform: error on message size: %d != %d\n",
                         send->numBytes_, send->interface_->info_.outputReportByteLength_);
            if (send->callback_ != NULL) {
                send->callback_(device->id_, send->cookie_, FREESPACE_ERROR_IO);
            }
        } else {
            // successfully sent message
            if (send->callback_ != NULL) {
                send->callback_(device->id_, send->cookie_, FREESPACE_SUCCESS);
            }
        }
        if (finalizeSendStruct(send, FALSE) != FREESPACE_SUCCESS) {
            DEBUG_PRINTF("freespace_private_devicePerform: error while sending message");
        }
    }

    // Call GetOverlappedResult() on everything to check what
    // messages were received.
    for (idx = 0; idx < device->handleCount_; idx++) {
        struct FreespaceSubStruct* s = &device->handle_[idx];

        if (s->readStatus_) {
            int lastErr;
            BOOL bResult = GetOverlappedResult(
                                               s->handle_,                 /* handle to device */
                                               &s->readOverlapped_,        /* long pointer to an OVERLAPPED structure */
                                               &s->readBufferSize,         /* returned buffer size */
                                               FALSE);
            lastErr = GetLastError();
            if (bResult) {
                // Got something, so report it.
                if (device->receiveCallback_ || device->receiveMessageCallback_) {
					if (device->receiveCallback_) {
						device->receiveCallback_(device->id_, (char *) (s->readBuffer), s->readBufferSize, device->receiveCookie_, FREESPACE_SUCCESS);
					}
					if (device->receiveMessageCallback_) {
						rc = freespace_decode_message((char *) (s->readBuffer), s->readBufferSize, &m, device->hVer_);
						if (rc == FREESPACE_SUCCESS) {
							device->receiveMessageCallback_(device->id_, &m, device->receiveMessageCookie_, FREESPACE_SUCCESS);
						} else {
							device->receiveMessageCallback_(device->id_, NULL, device->receiveMessageCookie_, rc);
							DEBUG_PRINTF("freespace_decode_message failed with code %d\n", rc);
						}
					}
				}
                s->readStatus_ = FALSE;
            } else if (lastErr != ERROR_IO_INCOMPLETE) {
                // Something severe happened to our device!
				DEBUG_PRINTF("freespace_private_devicePerform : Error on %d : %d\n", idx, lastErr);
                if (device->receiveCallback_) {
				    device->receiveCallback_(device->id_, NULL, 0, device->receiveCookie_, FREESPACE_ERROR_NO_DATA);
				}
				if (device->receiveMessageCallback_) {
				    device->receiveMessageCallback_(device->id_, NULL, device->receiveMessageCookie_, FREESPACE_ERROR_NO_DATA);
				}
                return handleDeviceFailure(device, lastErr);
            }
        }
    }

    // Re-initiate the ReadFile calls for the next go around.
    return initiateAsyncReceives(device);
}

static int terminateAsyncReceives(struct FreespaceDeviceStruct* device) {
    int idx;

    // Cancel the read operation on all handles that need it.
    for (idx = 0; idx < device->handleCount_; idx++) {
        struct FreespaceSubStruct* s = &device->handle_[idx];
        if (s->readStatus_) {
            CancelIo(s->handle_);
            s->readStatus_ = FALSE;
        }
    }

    return FREESPACE_SUCCESS;
}

void freespace_private_forceCloseDevice(struct FreespaceDeviceStruct* device) {
    int idx;

    if (device == NULL) {
        return;
    }
    terminateAsyncReceives(device);

    // Free all send events.
    for (idx = 0; idx < FREESPACE_MAXIMUM_SEND_MESSAGE_COUNT; idx++) {
        finalizeSendStruct(&device->send_[idx], TRUE);
    }

    // Free all read events
    for (idx = 0; idx < device->handleCount_; idx++) {
        struct FreespaceSubStruct* s = &device->handle_[idx];
        if (s->handle_ != NULL) {
            CloseHandle(s->handle_);
            s->handle_ = NULL;
        }

        if (s->readOverlapped_.hEvent != NULL) {
            s->readStatus_ = FALSE;
            CloseHandle(s->readOverlapped_.hEvent);
            s->readOverlapped_.hEvent = NULL;
        }
    }
    device->isOpened_ = FALSE;
}

LIBFREESPACE_API int freespace_openDevice(FreespaceDeviceId id) {
    int idx;
    struct FreespaceDeviceStruct* device = freespace_private_getDeviceById(id);
    if (device == NULL) {
        return FREESPACE_ERROR_NO_DEVICE;
    }

    if (device->isOpened_) {
        // Each device can only be opened once.
        return FREESPACE_ERROR_BUSY;
    }


    for (idx = 0; idx < device->handleCount_; idx++) {
        struct FreespaceSubStruct* s = &device->handle_[idx];
        if (s->handle_ != NULL) {
            // Device was partially (incorrectly) opened.
            freespace_private_forceCloseDevice(device);
            return FREESPACE_ERROR_BUSY;
        }
        if (s->devicePath == NULL) {
            // Device was not fully enumerated.
            freespace_private_forceCloseDevice(device);
            return FREESPACE_ERROR_NO_DEVICE;
        }
        DEBUG_WPRINTF(L"Open %s\n", s->devicePath);
        s->handle_ = CreateFile(s->devicePath,
                                GENERIC_READ | GENERIC_WRITE,
                                FILE_SHARE_READ | FILE_SHARE_WRITE,
                                NULL,
                                OPEN_EXISTING,
                                FILE_FLAG_OVERLAPPED,
                                NULL);
        {
            DWORD d;
            if (!GetHandleInformation(s->handle_, &d)) {
                // We do not have the correct handle.
                DEBUG_PRINTF("freespace_openDevice failed with code %d\n", GetLastError());
            }
        }

        if (s->handle_ == INVALID_HANDLE_VALUE) {
            freespace_private_forceCloseDevice(device);
            return FREESPACE_ERROR_NO_DEVICE;
        }

        if (!BindIoCompletionCallback(s->handle_, freespace_private_overlappedCallback, 0)) {
            freespace_private_forceCloseDevice(device);
            return FREESPACE_ERROR_UNEXPECTED;
        }

        if (!HidD_SetNumInputBuffers(s->handle_, HID_NUM_INPUT_BUFFERS)) {
            freespace_private_forceCloseDevice(device);
            return FREESPACE_ERROR_NO_DEVICE;
        }

        // Create the read event.
        s->readOverlapped_.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
        if (s->readOverlapped_.hEvent == NULL) {
            freespace_private_forceCloseDevice(device);
            return FREESPACE_ERROR_UNEXPECTED;
        }
        s->readOverlapped_.Offset = 0;
        s->readOverlapped_.OffsetHigh = 0;
        s->readStatus_ = FALSE;
    }

    device->isOpened_ = TRUE;

    // Enable send by initializing all send events.
    for (idx = 0; idx < FREESPACE_MAXIMUM_SEND_MESSAGE_COUNT; idx++) {
        device->send_[idx].overlapped_.hEvent = NULL;
        if (initializeSendStruct(&device->send_[idx]) != FREESPACE_SUCCESS) {
            freespace_private_forceCloseDevice(device);
            return FREESPACE_ERROR_UNEXPECTED;
        }
    }

    // If async mode has been enabled already, then start the receive
    // process going.
    if (freespace_instance_->fdAddedCallback_) {
        int rc;
        rc = initiateAsyncReceives(device);
        if (rc != FREESPACE_SUCCESS) {
            freespace_private_forceCloseDevice(device);
            return rc;
        }
    }

    return FREESPACE_SUCCESS;
}

LIBFREESPACE_API void freespace_closeDevice(FreespaceDeviceId id) {
    struct FreespaceDeviceStruct* device = freespace_private_getDeviceById(id);
    if (device == NULL) {
        return;
    }

    // Don't bother if the device isn't open.
    if (!device->isOpened_) {
        return;
    }

    freespace_private_forceCloseDevice(device);
}

static int prepareSend(FreespaceDeviceId id, struct FreespaceSendStruct** sendOut, const char* report, int length) {
    int idx;
    int retVal;
    struct FreespaceSubStruct* s;
    struct FreespaceDeviceStruct* device;
    struct FreespaceSendStruct* send;

    // Get the device
    device = freespace_private_getDeviceById(id);
    if (device == NULL) {
        return FREESPACE_ERROR_NO_DEVICE;
    }

    if (!device->isOpened_) {
        // The device must be opened!
        return FREESPACE_ERROR_IO;
    }

    // Get the send buffer and initialize
    send = getNextSendBuffer(device);
    *sendOut = send;
    if (send == NULL) {
        return FREESPACE_ERROR_BUSY;
    }
    retVal = initializeSendStruct(send);
    if (retVal != FREESPACE_SUCCESS) {
        return retVal;
    }

    // Find the desired output port.
    s = &device->handle_[0];
    for (idx = 0; idx < device->handleCount_; idx++) {
        if (device->handle_[idx].info_.outputReportByteLength_ > s->info_.outputReportByteLength_) {
            s = &device->handle_[idx];
        }
    }
    send->interface_ = s;

    if (length > s->info_.outputReportByteLength_) {
        send->rc_ = FREESPACE_ERROR_SEND_TOO_LARGE;
        return send->rc_;
    }
    if (s->info_.outputReportByteLength_ > FREESPACE_MAX_OUTPUT_MESSAGE_SIZE) {
        send->rc_ = FREESPACE_ERROR_UNEXPECTED;
        return send->rc_;
    }

    // Copy over the report to the local buffer.
    for (idx = 0; idx < length; idx++) {
        send->report_[idx] = report[idx];
    }

    // Stuff with trailing zeros as needed.
    for (idx = length; idx < s->info_.outputReportByteLength_; idx++) {
        send->report_[idx] = 0;
    }

    send->rc_ = FREESPACE_SUCCESS;
    return send->rc_;
}

static int initializeSendStruct(struct FreespaceSendStruct* send) {
    if (send == NULL) {
        return FREESPACE_ERROR_BUSY;
    }
    send->interface_ = NULL;
    send->error_     = FREESPACE_SUCCESS;

    // initialize the report
    send->overlapped_.Offset = 0;
    send->overlapped_.OffsetHigh = 0;

    if (send->overlapped_.hEvent != NULL) {
        return FREESPACE_SUCCESS;
    }

    // create an overlapped report event.
    send->overlapped_.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
    if (send->overlapped_.hEvent == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }
    return FREESPACE_SUCCESS;
}

static int finalizeSendStruct(struct FreespaceSendStruct* send, BOOL doClose) {
    send->interface_ = NULL;
    if (send->overlapped_.hEvent == NULL) {
        // Already finalized!
    } else if (doClose) {
        // Close the overlapped report event.
        CloseHandle(send->overlapped_.hEvent);
        send->overlapped_.hEvent = NULL;
	} else {
	    ResetEvent(send->overlapped_.hEvent);
	}
    return send->rc_;
}

static int freespace_send_activate(struct FreespaceSendStruct* send) {
    int retVal = 0;
    DWORD lastError = 0;

    // Write to the file to send out the message
    retVal = WriteFile(
                       send->interface_->handle_,
                       send->report_,
                       send->interface_->info_.outputReportByteLength_,
                       &send->numBytes_,
                       &send->overlapped_);

    if (retVal) {
        // Completed as synchronous I/O
        DEBUG_PRINTF("freespace_send: completed synchronously\n");
        send->rc_ = FREESPACE_SUCCESS;
        return finalizeSendStruct(send, FALSE);
    }

    // check for errors
    lastError = GetLastError();
    if (lastError != ERROR_IO_PENDING) {
        // Abort any pending messages and return
        // WARNING: CancelIo will also affect READ!
        DEBUG_PRINTF("freespace_send: GetLastError = %d\n", lastError);
        CancelIo(send->interface_->handle_);
        send->interface_->readStatus_ = FALSE;
        send->rc_ = FREESPACE_ERROR_UNEXPECTED; //FREESPACE_OS_ERROR_BASE - lastError;
        return finalizeSendStruct(send, FALSE);
    }

    return FREESPACE_ERROR_IO;
}

int freespace_private_send(FreespaceDeviceId id,
                           const uint8_t* report,
                           int length) {

    struct FreespaceSendStruct* send;
    int retVal = 0;
    DWORD lastError = 0;

    retVal = prepareSend(id, &send, report, length);
    if (retVal != FREESPACE_SUCCESS) {
        return retVal;
    }

    // Send the message
    retVal = freespace_send_activate(send);
    if (retVal != FREESPACE_ERROR_IO) {
        return retVal;
    }

    // Wait for the message to be sent
    lastError = WaitForSingleObject(send->overlapped_.hEvent, SEND_TIMEOUT);

    if (lastError != WAIT_OBJECT_0) {
        // timed out
        BOOL overlappedResult = GetOverlappedResult(send->interface_->handle_,
                                                    &send->overlapped_,
                                                    &send->numBytes_,
                                                    FALSE);

        // Abort any pending messages and return
        // WARNING: CancelIo will also affect READ!
        DEBUG_PRINTF("freespace_send: error on WaitForSingleObject = %d\n", lastError);
        CancelIo(send->interface_->handle_);
        send->interface_->readStatus_ = FALSE;

        if (overlappedResult) {
            send->rc_ = FREESPACE_ERROR_TIMEOUT;
        } else {
            send->rc_ = FREESPACE_ERROR_IO; //FREESPACE_OS_ERROR_BASE - lastError;
        }
    } else {
        // success
        BOOL overlappedResult = GetOverlappedResult(send->interface_->handle_,
                                                    &send->overlapped_,
                                                    &send->numBytes_,
                                                    TRUE);

        if (!overlappedResult) {
            DEBUG_PRINTF("freespace_send: error on GetOverlappedResult\n");
            send->rc_ = FREESPACE_ERROR_IO;
        } else if (send->numBytes_ != send->interface_->info_.outputReportByteLength_) {
            DEBUG_PRINTF("freespace_send: error on message size: %d != %d\n",
                         send->numBytes_, send->interface_->info_.outputReportByteLength_);
            send->rc_ = FREESPACE_ERROR_IO;
        } else {
            // successfully sent message
            send->rc_ = FREESPACE_SUCCESS;
        }
    }

    return finalizeSendStruct(send, FALSE);
}

LIBFREESPACE_API int freespace_sendMessage(FreespaceDeviceId id,
                                           struct freespace_message* message) {

    int retVal;
    uint8_t msgBuf[FREESPACE_MAX_OUTPUT_MESSAGE_SIZE];
    struct FreespaceDeviceInfo info;

    memset(msgBuf, 0, sizeof(msgBuf));
    
    // Address is reserved for now and must be set to 0 by the caller.
    if (message->dest == 0) {
        message->dest = FREESPACE_RESERVED_ADDRESS;
    }

    retVal = freespace_getDeviceInfo(id, &info);
    if (retVal != FREESPACE_SUCCESS) {
        return retVal;
    }
    
    message->ver = info.hVer;
    retVal = freespace_encode_message(message, msgBuf, FREESPACE_MAX_OUTPUT_MESSAGE_SIZE);
    if (retVal <= FREESPACE_SUCCESS) {
        return retVal;
    }
    
    return freespace_private_send(id, msgBuf, retVal);
}

int freespace_private_sendAsync(FreespaceDeviceId id,
                                const uint8_t* message,
                                int length,
                                unsigned int timeoutMs,
                                freespace_sendCallback callback,
                                void* cookie) {

    struct FreespaceSendStruct* send;
    int retVal = 0;
    DWORD lastError = 0;

    retVal = prepareSend(id, &send, message, length);
    if (retVal != FREESPACE_SUCCESS) {
        return retVal;
    }
    send->callback_ = callback;
    send->cookie_ = cookie;
    send->timeoutMs_ = timeoutMs;

    // Send the message
    retVal = freespace_send_activate(send);
    if (retVal != FREESPACE_ERROR_IO) { // FAH: This looks wrong.
        return retVal;
    }
    return FREESPACE_SUCCESS;
}

LIBFREESPACE_API int freespace_sendMessageAsync(FreespaceDeviceId id,
                                                struct freespace_message* message,
                                                unsigned int timeoutMs,
                                                freespace_sendCallback callback,
                                                void* cookie) {

    int retVal;
    uint8_t msgBuf[FREESPACE_MAX_OUTPUT_MESSAGE_SIZE];
    struct FreespaceDeviceInfo info;

    memset(msgBuf, 0, sizeof(msgBuf));
    
    // Address is reserved for now and must be set to 0 by the caller.
    if (message->dest == 0) {
        message->dest = FREESPACE_RESERVED_ADDRESS;
    }
    
    retVal = freespace_getDeviceInfo(id, &info);
    if (retVal != FREESPACE_SUCCESS) {
        return retVal;
    }
    
    message->ver = info.hVer;
    retVal = freespace_encode_message(message, msgBuf, FREESPACE_MAX_OUTPUT_MESSAGE_SIZE);
    if (retVal <= FREESPACE_SUCCESS) {
        return retVal;
    }

    return freespace_private_sendAsync(id, msgBuf, retVal, timeoutMs, callback, cookie);
}

int freespace_private_read(FreespaceDeviceId id,
                           uint8_t* message,
                           int maxLength,
                           unsigned int timeoutMs,
                           int* actualLength) {

    HANDLE waitEvents[FREESPACE_HANDLE_COUNT_MAX];
    int idx;
    DWORD bResult;

    struct FreespaceDeviceStruct* device = freespace_private_getDeviceById(id);
    if (device == NULL) {
        return FREESPACE_ERROR_NO_DEVICE;
    }

    // Start the reads going.
    for (idx = 0; idx < device->handleCount_; idx++) {
        BOOL bResult;
        struct FreespaceSubStruct* s = &device->handle_[idx];
        waitEvents[idx] = s->readOverlapped_.hEvent;

        // Initiate a ReadFile on anything that doesn't already have
        // a ReadFile op pending.
        if (!s->readStatus_) {
            int lastErr;
            bResult = ReadFile(
                               s->handle_,                 /* handle to device */
                               s->readBuffer,              /* IN report buffer to fill */
                               s->info_.inputReportByteLength_,  /* input buffer size */
                               &s->readBufferSize,         /* returned buffer size */
                               &s->readOverlapped_ );      /* long pointer to an OVERLAPPED structure */
            lastErr = GetLastError();
            if (bResult) {
                // Got something immediately, so return it.
                *actualLength = min(s->readBufferSize, (unsigned long) maxLength);
                memcpy(message, s->readBuffer, *actualLength);
                return FREESPACE_SUCCESS;
            } else if (lastErr != ERROR_IO_PENDING) {
                // Something severe happened to our device!
                DEBUG_PRINTF("freespace_read 1: Error on %d : %d\n", idx, lastErr);
                return handleDeviceFailure(device, lastErr);
            }
            s->readStatus_ = TRUE;
        }
    }

    // Wait.
    bResult = WaitForMultipleObjects(device->handleCount_, waitEvents, FALSE, timeoutMs);
    if (bResult == WAIT_FAILED) {
        DEBUG_PRINTF("Error from WaitForMultipleObjects\n");
        return FREESPACE_ERROR_IO;
    } else if (bResult == WAIT_TIMEOUT) {
        return FREESPACE_ERROR_TIMEOUT;
    }

    // Check which read worked.
    for (idx = 0; idx < device->handleCount_; idx++) {
        int lastErr;
        struct FreespaceSubStruct* s = &device->handle_[idx];
        BOOL bResult = GetOverlappedResult(
                                           s->handle_,                 /* handle to device */
                                           &s->readOverlapped_,        /* long pointer to an OVERLAPPED structure */
                                           &s->readBufferSize,         /* returned buffer size */
                                           FALSE);
        lastErr = GetLastError();
        if (bResult) {
            // Got something, so report it.
            *actualLength = min(s->readBufferSize, (unsigned long) maxLength);
            memcpy(message, s->readBuffer, *actualLength);
            s->readStatus_ = FALSE;
            return FREESPACE_SUCCESS;
        } else if (lastErr != ERROR_IO_INCOMPLETE) {
            // Something severe happened to our device!
            DEBUG_PRINTF("freespace_read 2 : Error on %d : %d\n", idx, lastErr);
            return handleDeviceFailure(device, lastErr);
        }
    }

    return FREESPACE_ERROR_IO;
}

LIBFREESPACE_API int freespace_readMessage(FreespaceDeviceId id,
                                           struct freespace_message* message,
                                           unsigned int timeoutMs) {
    int retVal;
    uint8_t buffer[FREESPACE_MAX_INPUT_MESSAGE_SIZE];
    int actLen;
    struct FreespaceDeviceInfo info;
    
    retVal = freespace_getDeviceInfo(id, &info);
    if (retVal != FREESPACE_SUCCESS) {
        return retVal;
    }
    
    retVal = freespace_private_read(id, buffer, sizeof(buffer), timeoutMs, &actLen);
    
    if (retVal == FREESPACE_SUCCESS) {
        return freespace_decode_message(buffer, actLen, message, info.hVer);
    } else {
        return retVal;
    }
}

LIBFREESPACE_API int freespace_flush(FreespaceDeviceId id) {
    int idx;

    struct FreespaceDeviceStruct* device = freespace_private_getDeviceById(id);
    if (device == NULL) {
        return FREESPACE_ERROR_NO_DEVICE;
    }

    for (idx = 0; idx < device->handleCount_; idx++) {
        struct FreespaceSubStruct* s = &device->handle_[idx];
        CancelIo(s->handle_);
        s->readStatus_ = FALSE;
    }
    return FREESPACE_SUCCESS;
}

int freespace_private_setReceiveCallback(FreespaceDeviceId id,
                                         freespace_receiveCallback callback,
                                         void* cookie) {
    struct FreespaceDeviceStruct* device = freespace_private_getDeviceById(id);
    if (device == NULL) {
        return FREESPACE_ERROR_NO_DEVICE;
    }

    if (device->isOpened_) {
        if (device->receiveCallback_ != NULL && callback == NULL) {
            // Deregistering callback, so stop any pending receives if no callbacks.
            device->receiveCallback_ = NULL;
            device->receiveCookie_ = NULL;

            if (device->receiveMessageCallback_ == NULL) {
                return terminateAsyncReceives(device);
            } else {
                return FREESPACE_SUCCESS;
            }
        } else if (device->receiveCallback_ == NULL && callback != NULL) {
            // Registering a callback, so initiate a receive if there were no callbacks.
            device->receiveCookie_ = cookie;
            device->receiveCallback_ = callback;

            if (device->receiveMessageCallback_ == NULL) {
                return initiateAsyncReceives(device);
            } else {
                return FREESPACE_SUCCESS;
            }
        }
    }
    // Just update the cookie and callback.
    device->receiveCallback_ = callback;
    if (callback != NULL) {
        device->receiveCookie_ = cookie;
    } else {
        device->receiveCookie_ = NULL;
    }

    return FREESPACE_SUCCESS;
}

LIBFREESPACE_API int freespace_setReceiveMessageCallback(FreespaceDeviceId id,
                                                        freespace_receiveMessageCallback callback,
                                                        void* cookie) {
    struct FreespaceDeviceStruct* device = freespace_private_getDeviceById(id);
    if (device == NULL) {
        return FREESPACE_ERROR_NO_DEVICE;
    }

    if (device->isOpened_) {
        if (device->receiveMessageCallback_ != NULL && callback == NULL) {
            // Deregistering callback, so stop any pending receives if no callbacks.
            device->receiveMessageCallback_ = NULL;
            device->receiveMessageCookie_ = NULL;

            if (device->receiveCallback_ == NULL) {
                return terminateAsyncReceives(device);
            } else {
                return FREESPACE_SUCCESS;
            }
        } else if (device->receiveMessageCallback_ == NULL && callback != NULL) {
            // Registering a callback, so initiate a receive if there were no callbacks.
            device->receiveMessageCookie_ = cookie;
            device->receiveMessageCallback_ = callback;

            if (device->receiveCallback_ == NULL) {
                return initiateAsyncReceives(device);
            } else {
                return FREESPACE_SUCCESS;
            }
        }
    }
    // Just update the cookie and callback.
    device->receiveMessageCallback_ = callback;
    if (device->receiveMessageCallback_ != NULL) {
        device->receiveMessageCookie_ = cookie;
    } else {
        device->receiveMessageCookie_ = NULL;
    }

    return FREESPACE_SUCCESS;
}

