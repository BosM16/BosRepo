#!/bin/bash

gnome-terminal\
 --tab -e "roscore"\
 --tab -e 'bash -c "sleep 2; rviz"'\
 --tab -e 'bash -c "sleep 2; rqt"'\
 --tab -e 'bash -c "cd bebop_ws; source devel/setup.bash; sleep 2; roslaunch bebop_demo bebop_autonomy.launch"'\
 --tab -e 'bash -c "cd bebop_ws; source devel/setup.bash; sleep 5; roslaunch bebop_demo bebop_demo.launch"'\
