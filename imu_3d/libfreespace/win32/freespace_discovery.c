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

/*
 * For a discussion of the device discovery process in MS Windows, see
 * http://www.codeproject.com/KB/system/HwDetect.aspx
 * MSDN: WM_DEVICECHANGE Message 
 *     http://msdn.microsoft.com/en-us/library/aa363480%28VS.85%29.aspx
 */

#include "freespace_discovery.h"
#include <stdio.h>
#include <malloc.h>

static const LPWSTR szMainWndClass = L"FreespaceDiscoveryWindow";

/**
 * Internal method to create the hidden window to listen for plug-and-play events.
 */
static DWORD WINAPI discoveryWindow(LPVOID lpParam);

/**
 * Internal method called from the hidden window.
 */
static LRESULT CALLBACK discoveryCallback(HWND hwnd, UINT nMsg, WPARAM wParam, LPARAM lParam);


void freespace_private_requestDeviceRescan() {
    LARGE_INTEGER liDueTime;
    liDueTime.QuadPart = -1000000LL; // 0.1 seconds represented in 100 ns increments

    if (!SetWaitableTimer(freespace_instance_->discoveryEvent_, &liDueTime, 0, NULL, NULL, 0)) {
        printf("SetWaitableTimer failed (%d)\n", GetLastError());
    }
}

int freespace_private_discoveryThreadInit() {
    HANDLE thread;

    if (freespace_instance_->window_ != NULL) {
        return FREESPACE_ERROR_BUSY;
    }

    // Create the timer used to indicated device rescan is required.
    freespace_instance_->discoveryEvent_ = CreateWaitableTimer(NULL, TRUE, NULL);
    if (freespace_instance_->discoveryEvent_ == NULL) {
        return FREESPACE_ERROR_UNEXPECTED;
    }

    // Need to scan a first time.
    freespace_instance_->needToRescanDevicesFlag_ = TRUE;

    // Create the hidden window
    freespace_instance_->discoveryTheadStatus_ = FREESPACE_SUCCESS;
    thread = CreateThread(NULL, 0, discoveryWindow, NULL, 0, NULL);
    if (thread == NULL) {
        freespace_instance_->discoveryTheadStatus_ = FREESPACE_ERROR_COULD_NOT_CREATE_THREAD;
        return FREESPACE_ERROR_COULD_NOT_CREATE_THREAD;
    }

    return FREESPACE_SUCCESS;
}

int freespace_private_discoveryThreadExit() {
    if (freespace_instance_->window_ == NULL) {
        return FREESPACE_SUCCESS;
    }

    // Delete the window
    // Memory cleanup is performed in the "WM_DESTROY" window callback.
    SendMessage(freespace_instance_->window_, WM_CLOSE, 0, 0);
    while (freespace_instance_->window_ != NULL) {
        Sleep(1);
    }

    return FREESPACE_SUCCESS;
}

BOOL freespace_private_discoveryStatusChanged() {
    if ( freespace_instance_->needToRescanDevicesFlag_ || 
         WaitForSingleObject(freespace_instance_->discoveryEvent_, 0) == WAIT_OBJECT_0) {
        // Race condition note: the flags need to be reset before the scan takes
        // place. If device status changes again between when this thread is notified
        // and the flags get reset, we're ok, since the scan happens afterwards. If the
        // change occurs after the reset of the flags, the flags will be set again, and
        // we'll scan next trip around the event loop.
        freespace_instance_->needToRescanDevicesFlag_ = FALSE;

        // Force the timer to unsignaled by restarting and then cancelling.
        freespace_private_requestDeviceRescan();
        CancelWaitableTimer(freespace_instance_->discoveryEvent_);

        return TRUE;
    } else {
        return FALSE;
    }
}

HANDLE freespace_private_discoveryEventObject() {
    return freespace_instance_->discoveryEvent_;
}

int freespace_private_discoveryGetThreadStatus() {
    return freespace_instance_->discoveryTheadStatus_;
}

LRESULT CALLBACK discoveryCallback(HWND hwnd, UINT nMsg, WPARAM wParam, LPARAM lParam) {
    if (nMsg == WM_CLOSE) {
        DestroyWindow(hwnd);
        return DefWindowProc(hwnd,	nMsg, wParam, lParam);
    }

    if (nMsg == WM_DESTROY) {
        DEBUG_WPRINTF(L"discoveryCallback on WM_DESTROY\n");

        // Remove the device notification
        if (freespace_instance_->windowEvent_) {
            UnregisterDeviceNotification(freespace_instance_->windowEvent_);
            freespace_instance_->windowEvent_ = NULL;
        }

        CancelWaitableTimer(freespace_instance_->discoveryEvent_);
        CloseHandle(freespace_instance_->discoveryEvent_);
        freespace_instance_->discoveryEvent_ = NULL;

        PostQuitMessage(0);
        return DefWindowProc(hwnd,	nMsg, wParam, lParam);
    }

    if (nMsg == WM_DEVICECHANGE) {
        // Should start with DEV_BROADCAST_HDR and validate.
        DEV_BROADCAST_DEVICEINTERFACE* hdr;
        hdr = (DEV_BROADCAST_DEVICEINTERFACE*) lParam;

        // Schedule a device list rescan.
        freespace_private_requestDeviceRescan();

        // Only handle all devices arrived or all devices removed.
        if ((LOWORD(wParam) != DBT_DEVICEARRIVAL) &&
            (LOWORD(wParam) != DBT_DEVICEREMOVECOMPLETE)) {
            DEBUG_WPRINTF(L"WM_DEVICECHANGE => %d\n", LOWORD(wParam));
            return TRUE;
        }

        /*
         * NOTE: Device scan is only performed once changes have stabilized.
         * The scan routine must be able to handled a device being removed
         * and reinserted without scan calls between events.
         */

        if (hdr->dbcc_devicetype != DBT_DEVTYP_DEVICEINTERFACE) {
            return TRUE;
        }

        if (LOWORD(wParam) == DBT_DEVICEARRIVAL) {
            DEBUG_WPRINTF(L"DBT_DEVICEARRIVAL => %s\n", hdr->dbcc_name);
        } else if (LOWORD(wParam) == DBT_DEVICEREMOVECOMPLETE) {
            DEBUG_WPRINTF(L"DBT_DEVICEREMOVECOMPLETE => %s\n", hdr->dbcc_name);
        } else {
            DEBUG_WPRINTF(L"discoveryCallback on unexpected change (%d) => %s\n", 
                LOWORD(wParam), hdr->dbcc_name);
        }

        return TRUE;
    }

    return DefWindowProc(hwnd,	nMsg, wParam, lParam);
}

DWORD WINAPI discoveryWindow(LPVOID lpParam) {
    DEV_BROADCAST_DEVICEINTERFACE NotificationFilter;
    GUID HidGuid;					/* temporarily stores Windows HID Class GUID */
    MSG event;
    WNDCLASSEX* wndclass = freespace_instance_->wndclass_;

    // Register the hidden window class
    if (wndclass == NULL) {
        wndclass = (WNDCLASSEX*) malloc(sizeof(WNDCLASSEX));
        memset (wndclass, 0, sizeof(WNDCLASSEX));
        wndclass->cbSize = sizeof(WNDCLASSEX);
        wndclass->style = CS_HREDRAW | CS_VREDRAW;
        wndclass->lpfnWndProc = discoveryCallback;
        wndclass->hInstance = 0;
        wndclass->hIcon = LoadIcon (NULL, IDI_APPLICATION);
        wndclass->hCursor = LoadCursor (NULL, IDC_ARROW);
        wndclass->hbrBackground = (HBRUSH)(COLOR_WINDOW+1);

        wndclass->lpszClassName = szMainWndClass;
        wndclass->hIconSm = LoadIcon (NULL, IDI_APPLICATION);

        if (!RegisterClassEx(wndclass)) {
            freespace_instance_->discoveryTheadStatus_ = FREESPACE_ERROR_UNEXPECTED;
            return 0;
        }
        freespace_instance_->wndclass_ = wndclass;
    }

    freespace_instance_->window_ = CreateWindow (
                                                 szMainWndClass,				/* Class name */
                                                 szMainWndClass,				/* Caption */
                                                 WS_OVERLAPPEDWINDOW,        /* Style */
                                                 CW_USEDEFAULT,              /* Initial x (use default) */
                                                 CW_USEDEFAULT,              /* Initial y (use default) */
                                                 CW_USEDEFAULT,              /* Initial x size (use default) */
                                                 CW_USEDEFAULT,              /* Initial y size (use default) */
                                                 NULL,					    /* No parent window */
                                                 NULL,                       /* No menu */
                                                 0,                          /* This program instance */
                                                 NULL);                      /* Creation parameters */

    if (freespace_instance_->window_ == NULL) {
        // We have a problem
        freespace_instance_->discoveryTheadStatus_ = FREESPACE_ERROR_UNEXPECTED;
        return 0;
    }

    /* Hide the window.  Have to manually close it later by posting a
     * WM_CLOSE event
     */
    ShowWindow (freespace_instance_->window_, SW_HIDE /*SW_SHOW*/);
    UpdateWindow (freespace_instance_->window_);

    /* Set up device notification of plug or unplug of HID Devices */

    /* 1) get the HID GUID */
    // WARNING: we currently only monitor HID devices
    HidD_GetHidGuid(&HidGuid);

    /* 2) clear the notification filter */
    ZeroMemory(&NotificationFilter, sizeof(NotificationFilter));

    /* 3) assign the previously cleared structure with the correct data
       so that the application is notified of HID device un/plug events */
    NotificationFilter.dbcc_size = sizeof(DEV_BROADCAST_DEVICEINTERFACE);
    NotificationFilter.dbcc_devicetype = DBT_DEVTYP_DEVICEINTERFACE;
    NotificationFilter.dbcc_classguid = HidGuid;

    /* 4) register device notifications for this application */
    freespace_instance_->windowEvent_ = RegisterDeviceNotification(freespace_instance_->window_,
                                                                   &NotificationFilter,
                                                                   DEVICE_NOTIFY_WINDOW_HANDLE | DEVICE_NOTIFY_ALL_INTERFACE_CLASSES );

    /* 5) notify the calling procedure if the HID device will not be recognized */
    if (!freespace_instance_->windowEvent_) {
        // Somehow cleanup after this...
        freespace_instance_->discoveryTheadStatus_ = FREESPACE_ERROR_UNEXPECTED;
        return 0;
    }

    // A blocking loop to retrieve events
    while (GetMessage(&event, NULL, 0, 0)) {
        TranslateMessage(&event);
        DispatchMessage(&event);
    }

    // Unregister the hidden window class
    if (!UnregisterClass(freespace_instance_->wndclass_->lpszClassName, 
                         freespace_instance_->wndclass_->hInstance)) {
        DEBUG_PRINTF("Could not unregister window: %d\n", GetLastError());
    } else {
        free(wndclass);
        freespace_instance_->wndclass_ = NULL;
    }

    freespace_instance_->window_ = NULL;

    return 0;
}
