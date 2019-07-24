methodology1
If the RS-232 open fail, add the dialout group into the user.
$ sudo usermod -a -G dialout [username]
Then restart the PC.
or

open the MotorControl.h program, then modify the port name.
dev = "[port name]"

To check the RS232 port name
$ cd /dev
$ ls 
The RS232 port may be ttyUSB*.
to change the rs232 port permissions
$ chmod 777 /dev/ttyUSB*

methodology2
check the rs232 chip
$ lsusb
$ udevadm info -a --name=/dev/bus/usb/003/052
$ udevadm info -a -n /dev/ttyUSB0 | grep id


modify .rules file ATTR{idVendor}=="1a86", ATTR{idProduct}=="7523"

$ sudo service udev reload
$ sudo service udev restart

copy rule file
sudo cp rule/99-devices.rules /etc/udev/rules.d/ 
