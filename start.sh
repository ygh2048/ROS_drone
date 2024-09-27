#!/bin/bash
echo "start task"

#cd ~/catkin_ws
#source devel/setup.bash

source ~/.bashrc

roslaunch mavros px4.launch &
<<<<<<< HEAD
echo "*********** sleep 10 second ***********"
sleep 10

roslaunch px4_realsense_bridge bridge.launch &
echo "*********** sleep 10 second ***********"
sleep 10
=======
echo "*********** sleep 5 second ***********"
sleep 5

roslaunch px4_bridge bridge.launch &
echo "*********** sleep 5 second ***********"
sleep 5
>>>>>>> ca253d21301eb8d4aa2cdb65d804e9cb085fa1fd


roslaunch rplidar_ros rplidar_a2m7.launch &
echo "*********** sleep 5 second ***********"
sleep 5

roslaunch my_nav nav_demo_indoor3.launch.launch &
echo "pleaes start rosnode"

#安全结束
exit 0