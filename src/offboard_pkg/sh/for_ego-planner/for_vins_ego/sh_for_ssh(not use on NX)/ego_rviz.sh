#!/bin/bash
ip=$1
#echo "ip=$ip"
export ROS_MASTER_URI=http://$ip:11311
export ROS_IP=$ip
rviz -d /home/maxi/sh_for_vinsfusiongpu_egoplanner/default.rviz



