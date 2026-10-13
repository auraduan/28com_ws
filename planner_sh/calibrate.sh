#!/bin/bash

echo "nvidia" | sudo -S chmod 777 /dev/ttyUSB0
sleep 2s;
gnome-terminal --tab --title="usb_cam" -- bash -c "roslaunch usb_cam usb_cam-test.launch; exec bash";
sleep 5s;
gnome-terminal --tab --title="calibration" -- bash -c "source $HOME/28com_ws/devel/setup.bash; cd /home/nvidia/28com_ws/src/Calibrate; python3 Calibrate.py; exec bash";



