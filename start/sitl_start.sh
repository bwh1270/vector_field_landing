#! /bin/bash

ROSDIR="/home/bwh/Aims/als_ws"
PX4DIR="/home/bwh/Dropbox/px4/px4_firmware"
ROSPKG="aims_als"

cd ${PX4DIR}
DONT_RUN=1 make px4_sitl gazebo-classic_typhoon_h480

source ${ROSDIR}/devel/setup.bash
source Tools/simulation/gazebo-classic/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/simulation/gazebo-classic/sitl_gazebo-classic

export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(rospack find aims_als)/models
roslaunch ${ROSPKG} sitl.launch vehicle:=iris
