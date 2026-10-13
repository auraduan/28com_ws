#!/usr/bin/expect
 set home_ip [lindex $argv 0]
 set host_ip [lindex $argv 1]
 set password [lindex $argv 3]
 set host_name [lindex $argv 2]

 spawn ssh $host_name@$host_ip         
 expect {                 
 "*yes/no" { send "yes\r"; exp_continue}    
 "*password:" { send "$password\r"; exp_continue }
 "*Last login:" { send "export ROS_MASTER_URI=http://$home_ip:11311\r"; exp_continue }
 "ROS_MASTER_URI" { send "export ROS_IP=$host_ip\r";exp_continue }
 "ROS_IP" { send "source /home/nvidia/multifunc_ws/devel/setup.bash\r";exp_continue }
 "source" { send "roslaunch offboard_pkg square_setpoint.launch\r"; }
 }  


interact

