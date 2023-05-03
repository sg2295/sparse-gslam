#!/bin/bash
# Quick script to pull latest changes, build and install sparse-gslam
git pull
catkin_make -DCMAKE_BUILD_TYPE=Release
catkin_make install
source install/setup.bash
source devel/setup.bash
