#!/bin/sh

killall roscore

cd bebop_ws/
roscore &
sleep 2
rviz &
rqt &
echo tralala

WID=$(xprop -root | grep "_NET_ACTIVE_WINDOW(WINDOW)"| awk '{print $5}')
xdotool windowfocus $WID
xdotool key ctrl+shift+t
xdotool key alt+1
wmctrl -i -a $WID

#source devel/setup.bash
#roslaunch bebop_demo bebop_autonomy.launch

WID=$(xprop -root | grep "_NET_ACTIVE_WINDOW(WINDOW)"| awk '{print $5}')
xdotool windowfocus $WID
xdotool key ctrl+shift+t
xdotool key alt+1
#wmctrl -i -a $WID
echo joepie


#source devel/setup.bash
#gnome-terminal -e cd bebop_ws/ roslaunch bebop_demo bebop_demo.launch
echo hoihoi
