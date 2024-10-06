#!/bin/bash
echo "start task"

source ~/.bashrc




roslaunch my_nav px4.launch &
echo "*********** sleep 20 second ***********"
sleep 20

roslaunch my_nav t265.launch &
echo "*********** sleep 20 second ***********"
sleep 20

roslaunch my_nav rplidar.launch &
echo "*********** sleep 20 second ***********"
sleep 20


#cd ~/drone_ws
#source ~/.bashrc
#roslaunch my_nav nav_demo_indoor3.launch.launch &
#echo "pleaes start rosnode"

#安全结束
exit 0