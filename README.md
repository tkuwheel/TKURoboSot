# TKU RoboSot Project
The purpose of this project is implementing, researching, developing the Soccer Robot for FIRA RoboSot League.
<hr>

# Requirement & Installation
### System:
~ROS Kinetic w/ Ubuntu 16.04 [Install](http://wiki.ros.org/kinetic/Installation/Ubuntu)~
~ROS Melodic w/ Ubuntu 18.04 [Install](http://wiki.ros.org/melodic/Installation/Ubuntu)~

## Setup:
```bash
$ mkdir -p robosot_ws/src && cd robosot_ws
$ git clone https://github.com/tkuwheel/TKURoboSot.git src/
```
### ROS Packages:
```bash
$ sudo apt-get install ros-kinetic-rosbridge-server
```
### Python packages:
```bash
$ cd src/
$ pip install -r requirements.txt
```
### Installization and Requirments of FLIR Grasshopper3 Camera
**see [pointgrey_camera_driver/README.md](pointgrey_camera_driver/README.md)**

## Compiling
```bash
$ catkin_make
```

<hr>

# Startup
### Environment Setting
```bash
# Get Camera's serial number
$ rosrun pointgrey_camera_driver list_cameras
# Setting Robot's number & Camera's Serail number
$ source Setting.sh
```
```bash
# Gazebo Simulator
$ roslaunch nubot_gazebo game_ready.launch

# Strategy w/ simulation mode
$ roslaunch strategy core.launch sim:=true

# GUI
# using plugin of dynamic_reconfigure
$ rqt
```

# Troubleshooting
### import error
```bash
from error import *
ImportError: No module named 'error'
```
Missing 'rospkg' 'catkin_tools'
```bash
pip install rospkg catkin_tools
```
### import rospkg library error:
```bash
ImportError: No module named 'rospkg'
```

Add the line to ~/.bashrc
```bash
export PYTHONPATH=$PYTHONPATH:/usr/lib/python2.7/dist-packages
```
### pip3 syntax error
Error message:
```bash
Traceback (most recent call last):
  File "/usr/bin/pip3", line 9, in <module>
    from pip import main
  File "/usr/lib/python2.7/dist-packages/pip/__init__.py", line 16, in <module>
    from pip.vcs import git, mercurial, subversion, bazaar  # noqa
  File "/usr/lib/python2.7/dist-packages/pip/vcs/subversion.py", line 9, in <module>
    from pip.index import Link
  File "/usr/lib/python2.7/dist-packages/pip/index.py", line 30, in <module>
    from pip.wheel import Wheel, wheel_ext
  File "/usr/lib/python2.7/dist-packages/pip/wheel.py", line 6, in <module>
    import compileall
  File "/usr/lib/python3.5/compileall.py", line 20, in <module>
    from concurrent.futures import ProcessPoolExecutor
  File "/usr/lib/python2.7/dist-packages/concurrent/futures/__init__.py", line 8, in <module>
    from concurrent.futures._base import (FIRST_COMPLETED,
  File "/usr/lib/python2.7/dist-packages/concurrent/futures/_base.py", line 357
    raise type(self._exception), self._exception, self._traceback
                               ^
SyntaxError: invalid syntax
```

Edit ```/usr/lib/python2.7/dist-packages/concurrent/futures/_base.py```

Replace ```raise type(self._exception), self._exception, self._traceback```

with ```raise Exception(self._exception).with_traceback(self._traceback)```
<hr>

# About Us
[Tamkang University Intelligent Automation And Robotics Center](http://www.iarc.tku.edu.tw/)<br>
[Our Research](http://www.iarc.tku.edu.tw/robots/)
