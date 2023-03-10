#!/bin/bash

git pull
catkin_make -DCMAKE_BUILD_TYPE=Release
catkin_make install
source install/setup.bash
source devel/setup.bash
# roslaunch sparse_gslam log_runner.launch dataset:=intel-lab
