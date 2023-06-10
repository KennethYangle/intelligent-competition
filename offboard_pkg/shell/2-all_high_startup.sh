#! /bin/bash
MAVID=1

gnome-terminal -x bash -c "source ${HOME}/Rfly_Attack/devel/setup.bash; roslaunch mavros px4.launch fcu_url:="/dev/ttyACM0:57600"; exec bash"
sleep 10s

gnome-terminal -x bash -c "source ${HOME}/Rfly_Attack/devel/setup.bash; roslaunch csi_cam main.launch; exec bash"
sleep 10s

gnome-terminal -x bash -c "source ${HOME}/Rfly_Attack/devel/setup.bash; rosrun jr_identify jr_identify; exec bash"
sleep 10s

gnome-terminal -x bash -c "source ${HOME}/Rfly_Attack/devel/setup.bash; roslaunch union_ros_serial main.launch mav_id:=${mav_id}; exec bash"
sleep 10s

gnome-terminal -x bash -c "source ${HOME}/Rfly_Attack/devel/setup.bash; roslaunch offboard_pkg obs_acc2.launch mav_id:=${MAVID}; exec bash"
sleep 10s
