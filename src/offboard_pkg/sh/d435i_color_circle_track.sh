gnome-terminal --window -e 'bash -c " source ~/multifunc_ws/devel/setup.bash;roslaunch offboard_pkg rs_d400_and_t265.launch; exec bash"' \
--tab -e 'bash -c "sleep 10; source ~/multifunc_ws/devel/setup.bash;roslaunch offboard_pkg circle_det.launch; exec bash"' \
--tab -e 'bash -c "sleep 10; source ~/multifunc_ws/devel/setup.bash;rosrun offboard_pkg t265_to_mavros; exec bash"' \
--tab -e 'bash -c "sleep 20; rqt_image_view; exec bash"' \
--tab -e 'bash -c "sleep 5; ./ttyTHS0.sh; exec bash"' \
--tab -e 'bash -c "sleep 10; roslaunch mavros px4_ego.launch; exec bash"' \
--tab -e 'bash -c "sleep 20; source ~/multifunc_ws/devel/setup.bash;rosrun offboard_pkg circle_track; exec bash"' \
--window -e 'bash -c "sleep 20; source ~/multifunc_ws/devel/setup.bash;rostopic echo /drone/object_detection/ellipse_det; exec bash"' \
--window -e 'bash -c "sleep 20; rostopic echo /mavros/local_position/pose; exec bash"' \

