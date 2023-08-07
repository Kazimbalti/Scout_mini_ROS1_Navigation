#!/bin/bash

if [ ! -d packages ]; then
    mkdir packages

fi

cd packages

# slam-gmapping package download
# Reference site : http://wiki.ros.org/gmapping
# Reference site : https://aur.archlinux.org/packages/ros-noetic-slam-gmapping
# Reference site : https://aur.archlinux.org/packages/ros-noetic-openslam-gmapping
if [ ! -d slam-gmapping ]; then
    mkdir slam-gmapping
    cd slam-gmapping

    wget https://github.com/ros-perception/slam_gmapping/archive/1.4.1.tar.gz -O ros-noetic-slam-gmapping-1.4.1.tar.gz
    wget https://github.com/ros-perception/openslam_gmapping/archive/0.2.1.tar.gz -O ros-noetic-openslam-gmapping-0.2.1.tar.gz

    tar -xvf ros-noetic-slam-gmapping-1.4.1.tar.gz
    tar -xvf ros-noetic-openslam-gmapping-0.2.1.tar.gz

    cd ..

else
    echo "slam-gmapping already exists!"

fi

# robot_localization package download
# Reference site : http://wiki.ros.org/robot_localization
# Reference site : https://github.com/cra-ros-pkg/robot_localization
if [ ! -d robot_localization ]; then
    git clone -b noetic-devel https://github.com/cra-ros-pkg/robot_localization.git 

else
    echo "robot_localization alread exists!"

fi

if [ ! -d navigation ]; then
    # ROS navigation package download
    # Reference site : http://wiki.ros.org/navigation
    # Reference site : https://github.com/ros-planning/navigation

    git clone -b noetic-devel https://github.com/ros-planning/navigation.git

else
    echo "navigation already exists!"

fi

cd ..