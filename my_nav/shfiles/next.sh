#!/bin/bash
echo "start next"

source ~/.bashrc


roslaunch my_nav nav_demo_indoor3.launch &
echo "*********** sleep 20 second ***********"
sleep 20

echo “ready to task_start”

roslaunch my_nav task_start.launch &
echo "*********** sleep 20 second ***********"
sleep 20