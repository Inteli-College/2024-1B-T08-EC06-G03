#!/bin/bash

source /opt/ros/humble/setup.sh

colcon build

source install/local_setup.bash

ros2 run bolin bolin 

ros2 run bolin_camera camera
