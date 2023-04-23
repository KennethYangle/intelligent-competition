#! /bin/bash

source ~/Rfly_Attack/devel/setup.bash
# Set FC into Hex+ mode

# roslaunch tracker_pkg calc_depth.launch & PID0=$!
roslaunch csi_cam main.launch & PID0=$!
sleep 10s
roslaunch mavros px4.launch fcu_url:="/dev/ttyACM0:57600" & PID1=$!
sleep 10s
roslaunch offboard_pkg obs_acc.launch & PID2=$!
sleep 5s

wait
kill -9 PID0 PID1 PID2 &
exit