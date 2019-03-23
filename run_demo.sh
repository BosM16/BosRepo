#!/bin/sh

killall roscore
roscore &
rviz &
rqt&
cd bebop_ws/
source devel/setup.bash
roslaunch bebop_demo bebop_autonomy.launch &

gnome-terminal
source devel/setup.bash
roslaunch bebop_demo bebop_demo.launch


