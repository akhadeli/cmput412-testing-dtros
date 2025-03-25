#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# launch publisher
rosrun my_package pd_crosswalk.py

# wait for app to end
dt-launchfile-join