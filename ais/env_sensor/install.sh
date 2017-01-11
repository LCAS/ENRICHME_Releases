#!/bin/bash

if [[ $EUID -ne 0 ]]; then
   echo "This script must be run as root" 1>&2
   exit 1
fi

echo "Adding User to correct groups"
sudo usermod -a -G dialout $USER

#see detect.sh for these info
vendNum=""
prodNum=""

echo "ACTION==\"add\", ATTRS{idVendor}==\"0403\", ATTRS{idProduct}==\"a858\", RUN+=\"/sbin/modprobe ftdi_sio\" RUN+=\"/bin/sh -c 'echo 0403 a858 > /sys/bus/usb-serial/drivers/ftdi_sio/new_id'\"" > /etc/udev/rules.d/67-persistant-detect-airbox.rules
echo "Adding Airbox to serial usb devices"

echo "SUBSYSTEM==\"usb\", ATTRS{idVendor}==\"0403\", ATTRS{idProduct}==\"a858\", MODE=\"0666\"" > /etc/udev/rules.d/68-persistant-permission-airbox.rules
echo "Making sistemwide readable"


echo "SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"0403\", ATTRS{idProduct}==\"a858\", ATTRS{serial}==\"A9U6X66Y\", MODE=\"0666\", SYMLINK+=\"airbox\"" > /etc/udev/rules.d/69-persistant-airbox.rules
echo "Persistant symlink created at /dev/airbox for this device"

sudo udevadm control --reload-rules

echo "Settings updated, unplug and replug AirBox device for /dev/airbox to appear"


echo "Finished Installing"
