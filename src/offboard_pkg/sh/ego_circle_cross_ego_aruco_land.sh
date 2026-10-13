gnome-terminal --window -e 'bash -c "source ~/multifunc_ws/devel/setup.bash;roslaunch offboard_pkg rs_d400_and_t265.launch; exec bash"' \
--tab -e 'bash -c "sleep 15; source ~/multifunc_ws/devel/setup.bash;roslaunch offboard_pkg usb_cam0.launch; exec bash"' \
--tab -e 'bash -c "sleep 10; source ~/multifunc_ws/devel/setup.bash;rosrun offboard_pkg t265_to_mavros; exec bash"' \
--tab -e 'bash -c "sleep 19; source ~/multifunc_ws/devel/setup.bash;roslaunch offboard_pkg aruco_det.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; ./ttyTHS0.sh; exec bash"' \
--tab -e 'bash -c "sleep 15; roslaunch mavros px4.launch; exec bash"' \
--tab -e 'bash -c "sleep 18; source ~/multifunc_ws/devel/setup.bash;roslaunch offboard_pkg t265_ego_tf_sub.launch; exec bash"' \
--tab -e 'bash -c "sleep 20; source ~/multifunc_ws/devel/setup.bash;roslaunch ego_planner rviz.launch; exec bash"' \
--tab -e 'bash -c "sleep 25; source ~/multifunc_ws/devel/setup.bash;roslaunch ego_planner run_in_exp_310temptest.launch; exec bash"' \
--tab -e 'bash -c "sleep 30; source ~/multifunc_ws/devel/setup.bash;rosrun offboard_pkg ego_circle_cross_ego_aruco_land; exec bash"' \
--tab -e 'bash -c "sleep 40; rqt_image_view; exec bash"' \
--window -e 'bash -c "sleep 39; rostopic echo /mavros/setpoint_raw/local; exec bash"' \
--window -e 'bash -c "sleep 39; rostopic echo /mavros/local_position/pose; exec bash"' \
--window -e 'bash -c "sleep 39; rostopic echo /mavros/state; exec bash"' \


