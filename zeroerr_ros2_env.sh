#!/usr/bin/env bash

conda deactivate 2>/dev/null || true

source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/install/setup.bash"

export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

export LD_LIBRARY_PATH=\
/usr/local/etherlab/lib:${LD_LIBRARY_PATH:-}

export EROBOT_MOVEIT_PKG="$HOME/ros2_ws/src/erobot_moveit_config"
export EROBOT_REAL_URDF="$HOME/ros2_ws/install/erobot_urdf_description/share/erobot_urdf_description/urdf/robot_real.urdf"
