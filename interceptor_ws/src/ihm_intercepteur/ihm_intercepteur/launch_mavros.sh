#!/bin/bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
ros2 run mavros mavros_node --ros-args \
  -p fcu_url:=$1:$2 \
  -p tgt_system:=1 \
  -p tgt_component:=1 \
  -p system_id:=255 \
  -p component_id:=191