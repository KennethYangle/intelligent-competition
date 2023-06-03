
MAVID=1
USE_PIX=false
# RflySim仿真参数
UE4IP="192.168.1.179"

echo "this MAV id :${MAVID}"


roslaunch offboard_pkg bs_mavros.launch  mav_id:=${MAVID} use_pix:=${USE_PIX} port1:=`expr ${MAVID} \* 2 + 20099` port2:=`expr ${MAVID} \* 2 + 20098` & PID1=$!
# roslaunch mavros px4.launch fcu_url:="udp://:20101@192.168.1.179:20100" & PID1=$!
sleep 10s
roslaunch rflysim_sensor_rospkg rgb_newprotocol_cpp.launch & PID2=$!
sleep 10s
roslaunch simulation rflysim_sphere.launch & PID3=$!
sleep 2s
roslaunch simulation sim_rfly.launch  mav_id:=${MAVID} & PID4=$!

# exit
wait
kill -9 PID1 PID2 PID3 PID4 &
exit
