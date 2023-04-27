#! /bin/bash

gnome-terminal -x bash -c "source /home/nvidia/Rfly_Attack/devel/setup.bash; roslaunch offboard_pkg bs_d435i_30hz.launch; exec bash"
sleep 10s

gnome-terminal -x bash -c "source /home/nvidia/Rfly_Attack/devel/setup.bash; roslaunch d435_cam main.launch; exec bash"
sleep 10s

gnome-terminal -x bash -c "source /home/nvidia/Rfly_Attack/devel/setup.bash; roslaunch mavros px4.launch fcu_url:="/dev/ttyACM0:57600"; exec bash"
sleep 10s

gnome-terminal -x bash -c "source /home/nvidia/Rfly_Attack/devel/setup.bash; roslaunch offboard_pkg bs_t265_bridge.launch; exec bash"
sleep 10s

gnome-terminal -x bash -c "source /home/nvidia/Rfly_Attack/devel/setup.bash; roslaunch offboard_pkg obs_acc.launch; exec bash"
sleep 10s
