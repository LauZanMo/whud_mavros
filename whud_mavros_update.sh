#!/bin/bash -e
cp src/whud* ~/Code/mavros_ws/src/mavros/mavros_extras/src/plugins
cp CMakeLists.txt ~/Code/mavros_ws/src/mavros/mavros_extras
cp mavros_plugins.xml ~/Code/mavros_ws/src/mavros/mavros_extras

cd ~/Code/mavros_ws
catkin build
