#!/bin/bash

# Copyright (c) 2021 taurob GmbH. All rights reserved. Confidential.
# Perfektastrasse 57/7, 1230 Wien, Austria. office@taurob.com

# Define the local and container paths for the src folder
LOCAL_SRC="/home/sabrina/Downloads/Docker_Taurob/src/"  # Replace with your actual local path
CONTAINER_SRC="/root/workspace/catkin_ws/src/"   # Path inside the container


xhost +local:root
XAUTH=/tmp/.docker.xauth
if [ ! -f $XAUTH ]
then
    xauth_list=$(xauth nlist :0 | sed -e 's/^..../ffff/')
    if [ ! -z "$xauth_list" ]
    then
        echo $xauth_list | xauth -f $XAUTH nmerge -
    else
        touch $XAUTH
    fi
    chmod a+r $XAUTH
fi

docker container rm sim_test 

docker run -it \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --env="XAUTHORITY=$XAUTH" \
    --env="SVGA_VGPU10=0" \
    --volume="$XAUTH:$XAUTH" \
    --name=sim_test  \
    --net=host \
    -v $LOCAL_SRC:$CONTAINER_SRC \
    simulation  \
    bash
