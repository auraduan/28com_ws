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
gnome-terminal --tab -t "D435i" -- bash -c "./d435i_for_vinsgpu_ego.sh $home_ip $host_ip  $host_login $host_password;exec bash"
sleep 15
gnome-terminal --tab -t "vins_estimator" -- bash -c "./vins_estimator.sh  $home_ip $host_ip $host_login $host_password;exec bash" 
sleep 6
gnome-terminal --tab -t "vinsfusion_to_mavros" -- bash -c "./vinsfusion_to_mavros.sh $home_ip $host_ip  $host_login $host_password;exec bash"
sleep 6
gnome-terminal --tab -t "ttyTHS0" -- bash -c "./ttyTHS0.sh $home_ip $host_ip  $host_login $host_password;exec bash"
sleep 6
gnome-terminal --tab -t "mavros" -- bash -c "./mavros.sh $home_ip $host_ip  $host_login $host_password;exec bash"
sleep 10
gnome-terminal --tab -t "vins_ego_tf_sub" -- bash -c "./vins_ego_tf_sub.sh $home_ip $host_ip  $host_login $host_password;exec bash"
sleep 6
gnome-terminal --tab -t "ego_planner" -- bash -c "./ego_planner.sh $home_ip $host_ip  $host_login $host_password;exec bash"
sleep 10
gnome-terminal --tab -t "mission" -- bash -c "./mission.sh $home_ip $host_ip  $host_login $host_password;exec bash"
sleep 5
gnome-terminal --tab -t "mavros_sender_new" -- bash -c "./mavros_sender_new.sh $home_ip $host_ip  $host_login $host_password;exec bash"
sleep 3
gnome-terminal --tab -t "ego_rviz" -- bash -c "./ego_rviz.sh  $home_ip $host_ip;exec bash"
sleep 3
gnome-terminal --window -t "mavros_state" -- bash -c "rostopic echo /mavros/state;exec bash"
sleep 3
gnome-terminal --window -t "local_position" -- bash -c "rostopic echo /mavros/local_position/pose;exec bash"
sleep 3
gnome-terminal --window -t "setpoint_raw" -- bash -c "rostopic echo /mavros/setpoint_raw/local;exec bash"
