#!/usr/bin/expect
 set home_ip [lindex $argv 0]
 set host_ip [lindex $argv 1]
 set password [lindex $argv 3]
 set host_name [lindex $argv 2]

 spawn ssh $host_name@$host_ip         
 expect {                 
 "*yes/no" { send "yes\r"; exp_continue}    
 "*password:" { send "$password\r"; exp_continue }
 "*Last login:" { send "sudo chmod 777 /dev/ttyTHS0\r"; exp_continue }
 "nvidia" { send "$password\r";}
 }  


interact

