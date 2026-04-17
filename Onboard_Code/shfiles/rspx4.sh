sudo chmod 777 /dev/ttyACM0 & sleep 1;
roslaunch mavros px4.launch & sleep 3;
roslaunch livox_ros_driver2 msg_MID360.launch & sleep 3;
roslaunch fast_lio mapping_mid360.launch
wait;
