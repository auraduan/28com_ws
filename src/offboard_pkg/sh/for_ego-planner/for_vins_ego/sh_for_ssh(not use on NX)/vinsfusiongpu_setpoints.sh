#!/bin/bash

home_ip="192.168.110.81"
host_ip="192.168.110.242"
host_login="nvidia"
host_password="nvidia"

sleep 1
gnome-terminal --tab -t "roscore" -- bash -c "roscore  ;exec bash"
sleep 3
gnome-terminal --tab -t "max_nx_cpu_freq" -- bash -c "./max_nx_cpu_freq.sh $home_ip $host_ip  $host_login $host_password;exec bash"
sleep 3
gnome-terminal --tab -t "max_nx_gpu_freq" -- bash -c "./max_nx_gpu_freq.sh $home_ip $host_ip  $host_login $host_password;exec bash"
sleep 3
gnome-terminal --tab -t "jetson_clocks" -- bash -c "./jetson_clocks.sh $home_ip $host_ip  $host_login $host_password;exec bash"
sleep 6
gnome-terminal --tab -t "D435i" -- bash -c "./d435i_for_vinsgpu.sh $home_ip $host_ip  $host_login $host_password;exec bash"
sleep 15
gnome-terminal --tab -t "vins_estimator" -- bash -c "./vins_estimator.sh  $home_ip $host_ip $host_login $host_password;exec bash" 
sleep 6
gnome-terminal --tab -t "vinsfusion_to_mavros" -- bash -c "./vinsfusion_to_mavros.sh $home_ip $host_ip  $host_login $host_password;exec bash"
sleep 6
gnome-terminal --tab -t "ttyTHS0" -- bash -c "./ttyTHS0.sh $home_ip $host_ip  $host_login $host_password;exec bash"
sleep 6
gnome-terminal --tab -t "mavros" -- bash -c "./mavros.sh $home_ip $host_ip  $host_login $host_password;exec bash"
sleep 6
gnome-terminal --tab -t "square_setpoint" -- bash -c "./square_setpoint.sh $home_ip $host_ip  $host_login $host_password;exec bash"
sleep 3
gnome-terminal --window -t "mavros_state" -- bash -c "rostopic echo /mavros/state;exec bash"
sleep 3
gnome-terminal --window -t "local_position" -- bash -c "rostopic echo /mavros/local_position/pose;exec bash"
