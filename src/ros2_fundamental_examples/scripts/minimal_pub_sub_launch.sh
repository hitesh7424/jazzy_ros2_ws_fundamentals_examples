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

# launch the publisher node
ros2 run ros2_fundamental_examples py_minimal_publisher.py &

sleep 2

# launch the subscriber node
ros2 run ros2_fundamental_examples py_minimal_subscriber.py
