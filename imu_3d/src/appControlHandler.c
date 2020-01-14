/**
 * This file is part of libfreespace-examples.
 *
 * Copyright (c) 2009-2013, Hillcrest Laboratories, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in
 *       the documentation and/or other materials provided with the
 *       distribution.
 *     * Neither the name of the Hillcrest Laboratories, Inc. nor the names
 *       of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written
 *       permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * This file handles control signals (e.g. CTRL+C, CTRL+BREAK, etc.)
 */

#ifdef _WIN32
#include "win32/stdafx.h"
#include <windows.h>
#include <stdio.h>
#else
#include <stdlib.h>
#include <signal.h>
#include <stdio.h>
#endif

#include "appControlHandler.h"

// A pointer to the variable that gets updated when a control signal is received
static int * quitPtr;

/**
 * printVersionInfo
 * Common helper function that prints the application running and the version of libfreespace
 * being used.
 *
 * @param appname A pointer to a string containing the application name. Typically from the arguments
 * passed in to main.
 */
void printVersionInfo(const char* appname) {
    printf("%s: Using libfreespace %s\n",
           appname,
           freespace_version());
}

/**
 * printDeviceInfo
 * Common helper function that prints the information about a device
 *
 * @param id The ID of the device to print the info for.
 * @param FREESPACE_SUCCESS or an error
 */
int printDeviceInfo(FreespaceDeviceId id) {
    struct FreespaceDeviceInfo info;
    int rc;

    // Retrieve the information for the device
    rc = freespace_getDeviceInfo(id, &info);
    if (rc != FREESPACE_SUCCESS) {
        return rc;
    }

    printf("    Device = %s\n    Vendor ID  = 0x%x (%d)\n    Product ID = 0x%x (%d)\n",
           info.name, info.vendor, info.vendor, info.product, info.product);

    return FREESPACE_SUCCESS;
}

#ifdef _WIN32
/*
 * CtrlHandler
 * The function that handles the control signals
 */
static BOOL CtrlHandler(DWORD fdwCtrlType) {
    // All control signals are handled in the same manner
    if (quitPtr) {
        *quitPtr = 1;        
    }
    return TRUE;
}

/**
 * addControlHandler
 * Add the CtrlHandler function to the list of control handler functions
 * called when the process receives a control signal.
 *
 * @param quit - a pointer to a variable to update when a control signal is received
 */
void addControlHandler(int * quit) {
    if (!SetConsoleCtrlHandler( (PHANDLER_ROUTINE) CtrlHandler, TRUE )) {
        printf("Could not install control handler\n");
    }
    quitPtr = quit;
}

#else

static void sighandler(int num) {
    if (quitPtr) {
        *quitPtr = 1;        
    }
}
void addControlHandler(int * quit) {
    // Set up the signal handler to catch
    // CTRL-C and clean up gracefully.
    struct sigaction setmask;
    sigemptyset(&setmask.sa_mask);
    setmask.sa_handler = sighandler;
    setmask.sa_flags = 0;

    sigaction(SIGHUP, &setmask, NULL);
    sigaction(SIGINT, &setmask, NULL);
    quitPtr = quit;
}
#endif
