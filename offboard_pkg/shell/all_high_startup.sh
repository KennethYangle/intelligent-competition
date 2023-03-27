#! /bin/bash


gnome-terminal -x bash -c "source /home/unionsys/Rfly_Attack/devel/setup.bash; roslaunch csi_cam main.launch; exec bash"
sleep 10s

gnome-terminal -x bash -c "source /home/unionsys/Rfly_Attack/devel/setup.bash; roslaunch mavros px4.launch fcu_url:="/dev/ttyACM0:57600"; exec bash"
sleep 10s

gnome-terminal -x bash -c "source /home/unionsys/Rfly_Attack/devel/setup.bash; roslaunch offboard_pkg obs_high.launch; exec bash"
sleep 10s
