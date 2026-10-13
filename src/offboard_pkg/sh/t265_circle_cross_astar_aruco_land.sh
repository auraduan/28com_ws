gnome-terminal --window -e 'bash -c " source ~/multifunc_ws/devel/setup.bash;roslaunch offboard_pkg rs_t265_image.launch; exec bash"' \
--tab -e 'bash -c "sleep 10; source ~/multifunc_ws/devel/setup.bash;roslaunch rplidar_ros rplidar_s1_drone.launch; exec bash"' \
--tab -e 'bash -c "sleep 15; source ~/multifunc_ws/devel/setup.bash;roslaunch offboard_pkg usb_cam0.launch; exec bash"' \
--tab -e 'bash -c "sleep 10; source ~/multifunc_ws/devel/setup.bash;roslaunch offboard_pkg t265_circle_det.launch; exec bash"' \
--tab -e 'bash -c "sleep 10; source ~/multifunc_ws/devel/setup.bash;roslaunch offboard_pkg aruco_det.launch; exec bash"' \
--tab -e 'bash -c "sleep 10; source ~/multifunc_ws/devel/setup.bash;rosrun offboard_pkg t265_to_mavros; exec bash"' \
--tab -e 'bash -c "sleep 20; rqt_image_view; exec bash"' \
--tab -e 'bash -c "sleep 5; ./ttyTHS0.sh; exec bash"' \
--tab -e 'bash -c "sleep 10; roslaunch mavros px4.launch; exec bash"' \
--tab -e 'bash -c "sleep 25; source ~/multifunc_ws/devel/setup.bash;rosrun offboard_pkg t265_circle_cross_astar_aruco_land; exec bash"' \
--tab -e 'bash -c "sleep 20; source ~/multifunc_ws/devel/setup.bash;roslaunch offboard_pkg astar_onboard_part.launch; exec bash"' \
--window -e 'bash -c "sleep 20; source ~/multifunc_ws/devel/setup.bash;rostopic echo /drone/object_detection/ellipse_det; exec bash"' \
--window -e 'bash -c "sleep 20; source ~/multifunc_ws/devel/setup.bash;rostopic echo /drone/object_detection/aruco_det; exec bash"' \
--window -e 'bash -c "sleep 20; rostopic echo /mavros/local_position/pose; exec bash"' \
--window -e 'bash -c "sleep 20; rostopic echo /mavros/state; exec bash"' \
--window -e 'bash -c "sleep 20; rostopic echo /mavros/setpoint_raw/local; exec bash"' \

