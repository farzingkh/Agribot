#! /bin/sh
xterm  -e  " source /opt/ros/kinetic/setup.bash; roscore" &
sleep 5
xterm -e "source home/workspace/Service_Robot/catkin/devel/setup.bash; roslaunch robot world.launch" &
sleep 5
xterm -e "source home/workspace/Service_Robot/catkin/devel/setup.bash; roslaunch gmapping mapping.launch" &
sleep 5 
xterm -e "rosrun teleop_twist_keyboard teleop_twist_keyboard.py "