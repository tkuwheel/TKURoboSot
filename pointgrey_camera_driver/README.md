pointgrey_camera_driver
=======================

[![Build Status](https://travis-ci.org/ros-drivers/pointgrey_camera_driver.png?branch=master)](https://travis-ci.org/ros-drivers/pointgrey_camera_driver)

ROS-compatible Camera drivers originally provided by NREC, part of Carnegie Mellon University's robotics institute.
These drives are included along with modifications of the standard ros image messages that enable HDR and physics based vision.

This code was originally developed by the National Robotics Engineering Center (NREC), part of the Robotics Institute at Carnegie Mellon University. Its development was funded by DARPA under the LS3 program and submitted for public release on June 7th, 2012. Release was granted on August, 21st 2012 with Distribution Statement "A" (Approved for Public Release, Distribution Unlimited).

This software is released under a BSD license:

Copyright (c) 2012, Carnegie Mellon University. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
Neither the name of the Carnegie Mellon University nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# 參考教學網頁：https://www.ptgrey.com/tan/10548 

# FlyCapture SDK安裝:
## 先安裝必要套件
```bash
Ubuntu 16.04
user$: sudo apt-get install libraw1394-11 libgtkmm-2.4-1v5 libglademm-2.4-1v5 libgtkglextmm-x11-1.2-dev libgtkglextmm-x11-1.2 libusb-1.0-0
Ubuntu 14.04
user$: sudo apt-get install libraw1394-11 libgtkmm-2.4-1c2a libglademm-2.4-1c2a libgtkglextmm-x11-1.2-dev libgtkglextmm-x11-1.2 libusb-1.0-0
```
## 確認Linux核心版本 > 3.5.0，並開啟USB3.1
```bash
$ uname -r      # checking linux version is newer than 3.5.0, if not, upgrade your kernel
$ sudo modprobe usbcore usbfs_memory_mb=1000
$ sudo vim /etc/default/grub
# 把GRUB_CMDLINE_LINUX_DEFAULT="quiet splash"這行修改成
# GRUB_CMDLINE_LINUX_DEFAULT="quiet splash usbcore.usbfs_memory_mb=1000"
$ sudo update-grub
$ sudo reboot  # 重開機
$ cat /sys/module/usbcore/parameters/usbfs_memory_mb # 檢查是否成功，應顯示1000
```
## 到[FLEA官網](https://www.ptgrey.com/support/downloads)下載SDK並解壓縮
```bash
$ cd <Path to flycapture2-2.13.3.31-amd64>
$ sudo sh install_flycapture.sh
```

## 下載ROS套件，執行
```bash
$ cd <Path to your workspace>/src
$ git clone https://github.com/ros-drivers/pointgrey_camera_driver.git
```
_因為套件預設最高fps只能調整到100，故下載套件原始碼修改最大值_
```bash
$ cd <Path to pointgrey_camera_driver package>/pointgrey_camera_driver/cfg
$ vim PointGrey.cfg
# 修改gen.add("frame_rate", double_t, SensorLevels.RECONFIGURE_RUNNING, "Camera speed (frames per second).", 7, 0, 100)
# gen.add("frame_rate", double_t, SensorLevels.RECONFIGURE_RUNNING, "Camera speed (frames per second).", 7, 0, 166)
$ rosrun pointgrey_camera_driver list_cameras # 察看攝影機序號
$ roslaunch pointgrey_camera_driver camera.launch camera_serial:=xxxxxxxx
```

