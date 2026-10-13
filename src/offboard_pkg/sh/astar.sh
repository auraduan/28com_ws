gnome-terminal --window -e 'bash -c " source ~/multifunc_ws/devel/setup.bash;roslaunch rplidar_ros rplidar_s1_drone.launch; exec bash"' \
--tab -e 'bash -c "sleep 10; ./ttyTHS0.sh; exec bash"' \
--tab -e 'bash -c "sleep 20; source ~/multifunc_ws/devel/setup.bash;roslaunch offboard_pkg astar_onboard.launch; exec bash"' \


