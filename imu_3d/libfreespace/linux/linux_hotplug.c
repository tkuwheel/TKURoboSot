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

#include "hotplug.h"
#include "freespace/freespace.h"

#include <sys/socket.h>
#include <linux/netlink.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>

#define FREESPACE_HOTPLUG_SETTLING_TIME 100 /*ms*/

// The socket for listening for hotplug events
static int sock_ = -1;
static int delay_ = FREESPACE_HOTPLUG_SETTLING_TIME;

int freespace_hotplug_init() {
    struct sockaddr_nl snl;
    int rc;
    int sock;
    const int on = 1;

    // Don't allow init to be called twice.
    if (sock_ != -1) {
        return FREESPACE_ERROR_UNEXPECTED;
    }

    // Initialize the socket for receiving UEVENTs
    sock = socket(PF_NETLINK, SOCK_DGRAM, NETLINK_KOBJECT_UEVENT);
    if (sock == -1) {
        return FREESPACE_ERROR_UNEXPECTED;
    }

    // Set the group and bind
    memset(&snl, 0, sizeof(struct sockaddr_nl));
    snl.nl_family = AF_NETLINK;
    snl.nl_groups = 1;
    rc = bind(sock, (struct sockaddr *)&snl, sizeof(struct sockaddr_nl));
    if (rc < 0) {
    	if (errno == EPERM) {
			// Running with a kernel that doesn't allow non-root users
			// to listen to uevents.
			// See http://git.kernel.org/?p=linux/kernel/git/gregkh/patches.git;a=blob;f=driver-core.current/driver-core-allow-non-root-users-to-listen-to-uevents.patch;h=ffcbd172f43c5cca6ece0294deb7c88cac8cf641;hb=b4a8f06022ecf4c5c279a61de89a1d3758e08e0e
			rc = FREESPACE_ERROR_ACCESS;
		} else {
			rc = FREESPACE_ERROR_UNEXPECTED;
        }
        close(sock);
        return rc;
    }

    // Enable credential receiving
    rc = setsockopt(sock, SOL_SOCKET, SO_PASSCRED, &on, sizeof(on));
    if (rc < 0) {
        close(sock);
        return FREESPACE_ERROR_UNEXPECTED;
    }

    // Set the socket to non-blocking
    rc = fcntl(sock, F_SETFL, O_NONBLOCK);
    if (rc < 0) {
        close(sock);
        return FREESPACE_ERROR_UNEXPECTED;
    }

    sock_ = sock;

    // See below for details on the state machine. Setting
    // delay here will signal a rescan the first time through.
    delay_ = FREESPACE_HOTPLUG_SETTLING_TIME;
    return FREESPACE_SUCCESS;
}

void freespace_hotplug_exit() {
    close(sock_);
    sock_ = -1;
}

int freespace_hotplug_getFD() {
    return sock_;
}

int freespace_hotplug_timeout() {
    return delay_;
}

int freespace_hotplug_perform(int* recheck) {
    char buf[16];
    int rc;
    int gotEvent = 0;

    // The uevent queue receives notifications on
    // device insertion, removal, and some changes. We
    // don't care what happens here. We just need to
    // know when to call into libusb to rescan the
    // USB bus.
    // NOTE: buf doesn't need to be large. The kernel
    //       will truncate notifications to fit inside
    //       buf and even though this looks like a
    //       stream interface, the kernel does not
    //       split messages up.
    for (;;) {
        // Drain the uevent queue until an error
        rc = recv(sock_, buf, sizeof(buf), 0);
        if (rc > 0) {
            gotEvent = 1;
        } else {
            break;
        }
    }

    // USB insertions cause a lot of events over a fraction
    // of a second. Wait until the system settles if we get
    // an event.
    if (gotEvent) {
        // Never recheck immediately.
        *recheck = 0;
        delay_ = FREESPACE_HOTPLUG_SETTLING_TIME;
    } else {
        // If we were delaying and now there are no events,
        // then signal a recheck. This happens on our
        // settling time timeout or if the user's event loop
        // polls us superfluously.
        if (delay_ > 0) {
            *recheck = 1;
            delay_ = 0;
        } else {
            *recheck = 0;
        }
    }
    if (errno == EAGAIN) {
        return FREESPACE_SUCCESS;
    } else {
        return FREESPACE_ERROR_IO;
    }
}
