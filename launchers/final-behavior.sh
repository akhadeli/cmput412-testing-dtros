#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# launch publisher
rosrun my_package final_behavior.py

# wait for app to end
dt-launchfile-join