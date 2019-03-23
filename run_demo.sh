#!/bin/bash

gnome-terminal\
 --tab -e "roscore"\
 --tab -e 'sh -c "sleep 2; rviz"'\
 --tab -e 'sh -c "sleep 2; rqt"'\
 --tab -e 'sh -c "cd bebop_ws; source devel/setup.bash; sleep 2; roslaunch bebop_demo bebop_autonomy.launch"'\
 --tab -e 'sh -c "cd bebop_ws; source devel/setup.bash; sleep 5; roslaunch bebop_demo bebop_demo.launch"'\

