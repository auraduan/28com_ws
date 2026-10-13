#!/bin/bash

echo "nvidia" | sudo -S chmod 777 /dev/ttyTHS0
echo "nvidia" | sudo -S chmod 777 /dev/ttyUSB0


gnome-terminal --tab --title="mavros" -- bash -c "source ~/.bashrc; roslaunch mavros px4.launch  fcu_url:=/dev/ttyTHS0:921600 gcs_url:=udp://@192.168.151.80:15501; exec bash";
sleep 5;
gnome-terminal --tab --title="rplidar" -- bash -c "source $HOME/28com_ws/devel/setup.bash; roslaunch rplidar_ros rplidar_s2.launch; exec bash";
sleep 8;
gnome-terminal --tab --title="usb_cam" -- bash -c "roslaunch usb_cam usb_cam-test.launch; exec bash";
sleep 5s;
gnome-terminal --tab --title="cartographer" -- bash -c "rosrun mavros mavcmd long 511 105 10000 0 0 0 0 0; source $HOME/28com_ws/devel/setup.bash; roslaunch cartographer_ros droneyee_mapping.launch; exec bash";
sleep 5s;
gnome-terminal --tab --title="yoloE" -- bash -c "source $HOME/28com_ws/devel/setup.bash; cd /home/nvidia/28com_ws/src/model_location; python3 auto_pose_ros1.py; exec bash";
sleep 20s
gnome-terminal --tab --title="offboard" -- bash -c "source $HOME/28com_ws/devel/setup.bash; roslaunch offboard_pkg offboard.launch; exec bash"; 
#sleep 3s
#gnome-terminal --tab --title="offboard" -- bash -c "source $HOME/car_planner_ws/devel/setup.bash; rostopic echo /planning/pos_cmd"; 

