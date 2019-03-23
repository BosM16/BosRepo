#!/bin/sh

killall roscore

cd bebop_ws/
roscore &
rviz &
rqt &
source devel/setup.bash
roslaunch bebop_demo bebop_autonomy.launch &

gnome-terminal --tab
source devel/setup.bash
roslaunch bebop_demo bebop_demo.launch


