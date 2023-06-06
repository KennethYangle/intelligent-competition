#! /bin/bash
MAVID=1

gnome-terminal -x bash -c "source /home/unionsys/Rfly_Attack/devel/setup.bash; roslaunch mavros px4.launch fcu_url:="/dev/ttyACM0:57600"; exec bash"
sleep 10s

gnome-terminal -x bash -c "source /home/unionsys/Rfly_Attack/devel/setup.bash; rosrun csi_cam img_pub.py; exec bash"
sleep 10s

gnome-terminal -x bash -c "source /home/unionsys/Rfly_Attack/devel/setup.bash; roslaunch offboard_pkg obs_acc.launch mav_id:=${MAVID}; exec bash"
sleep 10s
