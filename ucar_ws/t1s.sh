#!/bin/bash
source /home/ucar/ucar_ws/devel/setup.bash
#source /etc/profile

gnome-terminal --window -e "python /home/ucar/yolov5-v1.0/detect3.py"
sleep 2s
gnome-terminal --window -e "roslaunch ucar_nav gazebo_nav.launch" #
sleep 2s
gnome-terminal --window  -e "./t2s_test1.py"
