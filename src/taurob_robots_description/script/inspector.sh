#!/bin/bash

# Copyright (c) 2021 taurob GmbH. All rights reserved. Confidential.
# Perfektastrasse 57/7, 1230 Wien, Austria. office@taurob.com

WAIT_DURATION=10
if [ $# -gt 0 ]; then
    WAIT_DURATION=$1
fi

echo "Wait for ${WAIT_DURATION} sec before settings the joints ..."
sleep ${WAIT_DURATION} 

rosservice call /gazebo/set_model_configuration "model_name: 'robot_description'
urdf_param_name: ''
joint_names:
- 'arm_joint_1'
joint_positions:
- 3.69" 

rosservice call /gazebo/set_model_configuration "model_name: 'robot_description'
urdf_param_name: ''
joint_names:
- 'arm_joint_2'
joint_positions:
- 2.04" 

rosservice call /gazebo/unpause_physics "{}" 
