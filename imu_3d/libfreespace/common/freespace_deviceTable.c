/*
 * This file is part of libfreespace.
 *
 * Copyright (c) 2010-2012 Hillcrest Laboratories, Inc.
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

#include "freespace/freespace_deviceTable.h"

/*
 * Define the devices recognized by libfreespace.
 * Naming convention:
 *   UserMeaningfulName vN (XXXX)
 *   N = USB interface version number
 *   XXXX = Advertised interfaces:
 *      M = Mouse
 *      K = Keyboard
 *      C = Consumer page
 *      V = Joined multi-axis and vendor-specific
 *      A = Separate multi-axis and vendor-specific (deprecated)
 */
const struct FreespaceDeviceAPI freespace_deviceAPITable[] = {
    { 0x1d5a, 0xc001, 0, 2, 1, {{4, 0xff01}, {8, 1}}, "Piranha"},
    { 0x1d5a, 0xc002, 0, 1, 0, {{0, 0},      {0, 0}}, "Piranha bootloader"},
    { 0x1d5a, 0xc003, 0, 2, 1, {{4, 0xff01}, {8, 1}}, "Piranha factory test dongle"},
    { 0x1d5a, 0xc004, 0, 1, 1, {{0, 0},      {0, 0}}, "Piranha sniffer dongle"},
    { 0x1d5a, 0xc005, 0, 2, 1, {{4, 0xff01}, {8, 1}}, "FSRK STM32F10x eval board (E)"},
    { 0x1d5a, 0xc006, 0, 1, 0, {{0, 0},      {0, 0}}, "Cortex Bootloader"},
    { 0x1d5a, 0xc007, 0, 2, 1, {{4, 0xff01}, {8, 1}}, "FSRK Gen4 Dongle"},
    { 0x1d5a, 0xc008, 0, 2, 1, {{4, 0xff01}, {8, 1}}, "SPI to USB adapter board v0"},
    { 0x1d5a, 0xc009, 0, 2, 1, {{4, 0xff01}, {8, 1}}, "USB RF Transceiver v0"},
    { 0x1d5a, 0xc00a, 1, 2, 1, {{4, 0xff01}, {8, 1}}, "Coprocessor to USB adapter v1 (MA)"},
    { 0x1d5a, 0xc00b, 1, 2, 1, {{4, 0xff01}, {8, 1}}, "USB RF Transceiver v1 (MKCA)"},
    { 0x1d5a, 0xc00c, 1, 2, 1, {{4, 0xff01}, {8, 1}}, "SPI to USB adapter v1 (MA)"},
    { 0x1d5a, 0xc010, 1, 1, 1, {{4, 0xff01}, {0, 0}}, "USB RF Transceiver v1 (MV)"},
    { 0x1d5a, 0xc011, 1, 1, 1, {{4, 0xff01}, {0, 0}}, "USB RF Transceiver v1 (MCV)"},
    { 0x1d5a, 0xc012, 1, 1, 1, {{4, 0xff01}, {0, 0}}, "USB RF Transceiver v1 (MKV)"},
    { 0x1d5a, 0xc013, 1, 1, 1, {{4, 0xff01}, {0, 0}}, "USB RF Transceiver v1 (MKCV)"},
    { 0x1d5a, 0xc020, 1, 1, 1, {{4, 0xff01}, {0, 0}}, "SPI to USB adapter v1 (MV)"},
    { 0x1d5a, 0xc021, 1, 1, 1, {{4, 0xff01}, {0, 0}}, "SPI to USB adapter v1 (V)"},
    { 0x1d5a, 0xc030, 1, 1, 1, {{4, 0xff01}, {0, 0}}, "Coprocessor to USB adapter v1 (MV)"},
    { 0x1d5a, 0xc031, 1, 1, 1, {{4, 0xff01}, {0, 0}}, "Coprocessor to USB adapter v1 (V)"},
    { 0x1d5a, 0xc040, 1, 1, 2, {{4, 0xff01}, {0, 0}}, "USB RF Transceiver v2 (MV)"},
    { 0x1d5a, 0xc041, 1, 1, 2, {{4, 0xff01}, {0, 0}}, "USB RF Transceiver v2 (MCV)"},
    { 0x1d5a, 0xc042, 1, 1, 2, {{4, 0xff01}, {0, 0}}, "USB RF Transceiver v2 (MKV)"},
    { 0x1d5a, 0xc043, 1, 1, 2, {{4, 0xff01}, {0, 0}}, "USB RF Transceiver v2 (MKCV)"},
    { 0x1d5a, 0xc044, 1, 1, 2, {{4, 0xff01}, {0, 0}}, "USB RF Transceiver AltRF v2 (MV)"},
    { 0x1d5a, 0xc045, 1, 1, 2, {{4, 0xff01}, {0, 0}}, "USB RF Transceiver AltRF v2 (MCV)"},
    { 0x1d5a, 0xc046, 1, 1, 2, {{4, 0xff01}, {0, 0}}, "USB RF Transceiver AltRF v2 (MKV)"},
    { 0x1d5a, 0xc047, 1, 1, 2, {{4, 0xff01}, {0, 0}}, "USB RF Transceiver AltRF v2 (MKCV)"},
    { 0x1d5a, 0xc050, 1, 1, 2, {{4, 0xff01}, {0, 0}}, "SPI to USB adapter v2 (MV)"},
    { 0x1d5a, 0xc051, 1, 1, 2, {{4, 0xff01}, {0, 0}}, "SPI to USB adapter v2 (V)"},
    { 0x1d5a, 0xc052, 1, 1, 2, {{4, 0xff01}, {0, 0}}, "SPI to USB adapter v2 (MKCV)"},
    { 0x1d5a, 0xc060, 1, 1, 2, {{4, 0xff01}, {0, 0}}, "Coprocessor to USB adapter v2 (MV)"},
    { 0x1d5a, 0xc061, 1, 1, 2, {{4, 0xff01}, {0, 0}}, "Coprocessor to USB adapter v2 (V)"},
    { 0x1d5a, 0xc070, 1, 1, 2, {{4, 0xff01}, {0, 0}}, "Bluetooth v2 (MV)"},
    { 0x1d5a, 0xc071, 1, 1, 2, {{4, 0xff01}, {0, 0}}, "Bluetooth v2 (MCV)"},
    { 0x1d5a, 0xc022, 1, 1, 2, {{4, 0xff01}, {0, 0}}, "Bluetooth v2 (MKV)"},
    { 0x1d5a, 0xc073, 1, 1, 2, {{4, 0xff01}, {0, 0}}, "Bluetooth v2 (MKCV)"},
    { 0x1d5a, 0xc080, 1, 1, 2, {{4, 0xff01}, {0, 0}}, "USB Freespace Module (MV)"},
    { 0x1d5a, 0xc090, 1, 1, 2, {{4, 0xff01}, {0, 0}}, "Bluetooth Remote(CKV)"},
	{ 0x1d5a, 0xc0b0, 1, 1, 2, {{4, 0xff01}, {0, 0}}, "Smart USB RF Transceiver v2 (MV)"},
	{ 0x1d5a, 0xc0b1, 1, 1, 2, {{4, 0xff01}, {0, 0}}, "Smart USB RF Transceiver v2 (MCV)"},
	{ 0x1d5a, 0xc0b2, 1, 1, 2, {{4, 0xff01}, {0, 0}}, "Smart USB RF Transceiver v2 (MKV)"},
	{ 0x1d5a, 0xc0b3, 1, 1, 2, {{4, 0xff01}, {0, 0}}, "Smart USB RF Transceiver v2 (MKCV)"},
	{ 0x7045, 0x2860, 1, 1, 2, {{4, 0xff01}, {0, 0}}, "Nanosic Smart USB RF Transceiver"},
	{ 0x1d5a, 0xc0c0, 2, 1, 2, {{4, 0xff01}, {0, 0}}, "RF4CE RemoTI ZID dongle (MKCV)"},
	{ 0x1d5a, 0xc0d0, 2, 1, 2, {{4, 0xff01}, {0, 0}}, "BLE Scoop (MKCV)"},
    { 0x1d5a, 0xc0e0, 1, 1, 2, {{4, 0xff01}, {0, 0}}, "USB Freespace Module (MV)"},
    { 0x1d5a, 0xc100, 1, 1, 0, {{0, 0},      {0, 0}}, "USB Bootloader for WP160"},
    { 0x1d5a, 0xc101, 1, 1, 0, {{0, 0},      {0, 0}}, "USB Bootloader for WP260"},
    { 0x1d5a, 0xc102, 1, 1, 0, {{0, 0},      {0, 0}}, "USB Bootloader for S2U160"},
    { 0x1d5a, 0xc103, 1, 1, 0, {{0, 0},      {0, 0}}, "USB Bootloader for S2U260"},
    { 0x1d5a, 0xc104, 1, 1, 0, {{0, 0},      {0, 0}}, "USB Bootloader for CP160"},
    { 0x1d5a, 0xc105, 1, 1, 0, {{0, 0},      {0, 0}}, "USB Bootloader for CP260"},
    { 0x1d5a, 0xc106, 1, 1, 0, {{0, 0},      {0, 0}}, "USB Bootloader for WP261"},
	{ 0x1d5a, 0xc107, 1, 1, 0, {{0, 0},      {0, 0}}, "USB Bootloader for FSP262"},
    { 0x1d5a, 0xc200, 1, 1, 0, {{0, 0},      {0, 0}}, "USB Bootloader for FSP275"},
    { 0x1d5a, 0xc201, 1, 1, 0, {{0, 0},      {0, 0}}, "USB Bootloader for FSP276"},
	{ 0x1d5a, 0xc2b3, 1, 1, 2, {{4, 0xff01}, {0, 0}}, "Smart USB RF Transceiver v2 (MKCV)"},
	{ 0x1d5a, 0xc300, 1, 1, 0, {{0, 0},      {0, 0}}, "USB Bootloader for WP160"},
	{ 0x1d5a, 0xc301, 1, 1, 0, {{0, 0},      {0, 0}}, "USB Bootloader for WP260"},
	{ 0x1d5a, 0xc302, 1, 1, 0, {{0, 0},      {0, 0}}, "USB Bootloader for S2U160"},
	{ 0x1d5a, 0xc303, 1, 1, 0, {{0, 0},      {0, 0}}, "USB Bootloader for S2U260"},
	{ 0x1d5a, 0xc304, 1, 1, 0, {{0, 0},      {0, 0}}, "USB Bootloader for CP160"},
	{ 0x1d5a, 0xc305, 1, 1, 0, {{0, 0},      {0, 0}}, "USB Bootloader for CP260"},
	{ 0x1d5a, 0xc306, 1, 1, 0, {{0, 0},      {0, 0}}, "USB Bootloader for WP261"},
	{ 0x1d5a, 0xc307, 1, 1, 0, {{0, 0},      {0, 0}}, "USB Bootloader for FSP262"},
	{ 0x1d5a, 0xc400, 1, 1, 0, {{0, 0},      {0, 0}}, "USB Bootloader for WP160"},
	{ 0x1d5a, 0xc401, 1, 1, 0, {{0, 0},      {0, 0}}, "USB Bootloader for WP260"},
	{ 0x1d5a, 0xc402, 1, 1, 0, {{0, 0},      {0, 0}}, "USB Bootloader for S2U160"},
	{ 0x1d5a, 0xc403, 1, 1, 0, {{0, 0},      {0, 0}}, "USB Bootloader for S2U260"},
	{ 0x1d5a, 0xc404, 1, 1, 0, {{0, 0},      {0, 0}}, "USB Bootloader for CP160"},
	{ 0x1d5a, 0xc405, 1, 1, 0, {{0, 0},      {0, 0}}, "USB Bootloader for CP260"},
	{ 0x1d5a, 0xc406, 1, 1, 0, {{0, 0},      {0, 0}}, "USB Bootloader for WP261"},
	{ 0x1d5a, 0xc407, 1, 1, 0, {{0, 0},      {0, 0}}, "USB Bootloader for FSP262"},
    { 0x300,  0x30,   1, 1, 2, {{1, 0xff00}, {0, 0}}, "Transceiver"},
};

const int freespace_deviceAPITableNum = sizeof(freespace_deviceAPITable) / sizeof(struct FreespaceDeviceAPI);


const struct FreespaceDeviceInfo freespace_newDeviceAPITable[] = {
	{ "Smart USB RF Transceiver v2 (MKCV)", 0x1d5a, 0xc0b3, 2 },
	{ "Smart USB RF Transceiver v2 (MKCV)", 0x1d5a, 0xc2b3, 2 },
};

const int freespace_newDeviceAPITableNum = sizeof(freespace_newDeviceAPITable) / sizeof(freespace_newDeviceAPITable[0]);

