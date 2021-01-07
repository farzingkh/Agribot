#! /bin/sh
xterm  -e  " source /opt/ros/kinetic/setup.bash; roscore" &
sleep 5
xterm -e " home/workspace/Service_Robot/catkin/devel/setup.bash; roslaunch robot world.launch" &
sleep 5 
xterm -e " home/workspace/Service_Robot/catkin/devel/setup.bash; roslaunch amcl_localization amcl.launch" &
sleep 5 
xterm -e " home/workspace/Service_Robot/catkin/devel/setup.bash; rosrun add_markers add_markers test"

