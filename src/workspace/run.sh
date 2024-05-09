#!/bin/bash

# Encontra o path do flask
FLASK_PATH=$(pip show flask | grep "Location:" | awk '{print $2}')

if [ -z "$FLASK_PATH" ]; then
    echo "Flask not found. Please make sure it is installed."
    exit 1
fi

# Adiciona o path do flask ao PYTHONPATH
export PYTHONPATH="$PYTHONPATH:$FLASK_PATH"

source /opt/ros/humble/setup.sh

colcon build

source install/local_setup.bash

ros2 run robot_navigation bot
