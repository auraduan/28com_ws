gnome-terminal --window -e 'bash -c " roslaunch realsense2_camera rs_t265.launch; exec bash"' \
--tab -e 'bash -c "sleep 10; source /home/nvidia/multifunc_ws/devel/setup.bash;roslaunch rplidar_ros rplidar_s1.launch; exec bash"' \
--tab -e 'bash -c "sleep 17; source /home/nvidia/cartographer_ws/install_isolated/setup.bash;roslaunch cartographer_ros cartographer_demo_rplidar.launch; exec bash"' \
--tab -e 'bash -c "sleep 25; source ~/multifunc_ws/devel/setup.bash;rosrun offboard_pkg cartographer_t265_to_mavros; exec bash"' \
--tab -e 'bash -c "sleep 5; ./ttyTHS0.sh; exec bash"' \
--tab -e 'bash -c "sleep 10; roslaunch mavros px4.launch; exec bash"' \
--tab -e 'bash -c "sleep 25; source ~/multifunc_ws/devel/setup.bash;roslaunch offboard_pkg move_base.launch; exec bash"' \
--tab -e 'bash -c "sleep 25; source ~/multifunc_ws/devel/setup.bash;roslaunch offboard_pkg move_base_rviz.launch; exec bash"' \
--tab -e 'bash -c "sleep 25; source ~/multifunc_ws/devel/setup.bash;rosrun offboard_pkg move_base_to_mavros; exec bash"' \
--window -e 'bash -c "sleep 25; rostopic echo /mavros/local_position/pose; exec bash"' \

