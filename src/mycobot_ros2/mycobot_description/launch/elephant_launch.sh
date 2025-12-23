#! /bin/bash

# launch publisher and subscriber nodes with cleanup handler function

cleanup(){
    echo "Restarting ROS 2 daemon to cleanup before shutting down all processes ..."
    ros2 daemon stop 
    sleep 1
    ros2 daemon start
    echo "Terminating all ROS 2-related processes..."
    kill 0
    exit
}

trap 'cleanup' SIGINT

# launch the rviz with elephant mycobot
ros2 launch urdf_tutorial display.launch.py model:=/home/hitesh/Desktop/ROS2projects/ros2_ws/src/mycobot_ros2/mycobot_description/urdf/robots/mycobot_280.urdf.xacro

