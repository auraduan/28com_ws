gnome-terminal --window -e 'bash -c "source ~/multifunc_ws/devel/setup.bash;roslaunch offboard_pkg rs_camera_vinsgpu_ego.launch; exec bash"' \
--tab -e 'bash -c "sleep 10; rosrun vins vins_node ~/vinsfusiongpu_ws/src/vins-fusion-gpu-speed_opti/config/maxidroneyee_realsense_d435i/realsense_stereo_imu_config.yaml; exec bash"' \
--tab -e 'bash -c "sleep 18; source ~/multifunc_ws/devel/setup.bash;rosrun offboard_pkg vinsfusion_to_mavros; exec bash"' \
--tab -e 'bash -c "sleep 5; ./ttyTHS0.sh; exec bash"' \
--tab -e 'bash -c "sleep 15; source ~/multifunc_ws/devel/setup.bash;roslaunch offboard_pkg px4.launch; exec bash"' \
--tab -e 'bash -c "sleep 22; source ~/multifunc_ws/devel/setup.bash;roslaunch offboard_pkg vins_ego_tf_sub.launch; exec bash"' \
--tab -e 'bash -c "sleep 25; source ~/multifunc_ws/devel/setup.bash;roslaunch ego_planner rviz.launch; exec bash"' \
--tab -e 'bash -c "sleep 27; source ~/multifunc_ws/devel/setup.bash;roslaunch ego_planner run_in_exp_310_with_vins.launch; exec bash"' \
--tab -e 'bash -c "sleep 33; source ~/multifunc_ws/devel/setup.bash;roslaunch offboard_pkg ego_four_goals.launch; exec bash"' \
--tab -e 'bash -c "sleep 39; rostopic echo /mavros/setpoint_raw/local; exec bash"' \
--window -e 'bash -c "sleep 39; rostopic echo /mavros/local_position/pose; exec bash"' \
--window -e 'bash -c "sleep 39; rostopic echo /mavros/state; exec bash"' \


