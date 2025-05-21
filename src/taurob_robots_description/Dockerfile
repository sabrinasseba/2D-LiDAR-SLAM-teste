# #########################  #
#  Container for Simulation  #
# #########################  #
ARG BASE_IMAGE=osrf/ros:noetic-desktop-full

FROM $BASE_IMAGE

ARG ROS_VERSION=noetic

#=====================
# Updates for debconf
# Prevent message 'debconf: unable to initialize frontend: Dialog'
# Prevent message 'debconf: delaying package configuration, since apt-utils is not installed'
#=====================
RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections

# Install packages
RUN apt-get update && apt-get install -y  \
  apt-utils \
  cifs-utils \
  ros-$ROS_VERSION-mrpt-localization \
  ros-$ROS_VERSION-rosbash \
  ros-$ROS_VERSION-velodyne \
  ros-$ROS_VERSION-velodyne-description \
  ros-$ROS_VERSION-velodyne-driver \
  ros-$ROS_VERSION-velodyne-gazebo-plugins \
  ros-$ROS_VERSION-velodyne-laserscan \
  ros-$ROS_VERSION-velodyne-msgs \
  ros-$ROS_VERSION-velodyne-pointcloud \
  ros-$ROS_VERSION-velodyne-simulator \
  ros-$ROS_VERSION-joint-trajectory-controller \
  ros-$ROS_VERSION-hector-xacro-tools \
  ros-$ROS_VERSION-hector-gazebo \
  ros-$ROS_VERSION-hector-gazebo-plugins \
  ros-$ROS_VERSION-effort-controllers \
  ros-$ROS_VERSION-navigation \
  ros-$ROS_VERSION-roboticsgroup-upatras-gazebo-plugins \
  nano \
  tmux \
  iputils-ping \
  subversion \
  ros-$ROS_VERSION-teb-local-planner \
  ros-$ROS_VERSION-teleop-twist-joy \
  ros-$ROS_VERSION-rosserial \
  ros-$ROS_VERSION-rqt-joint-trajectory-controller \
  ros-$ROS_VERSION-gazebo-ros-control \
  ros-$ROS_VERSION-joint-state-publisher \
  ros-$ROS_VERSION-joint-state-publisher-gui \
  libgazebo11-dev \
  libgl1-mesa-glx \
  libdxflib-dev \
  ros-$ROS_VERSION-ecl-threads \
  ros-$ROS_VERSION-tf2-geometry-msgs \
  ros-$ROS_VERSION-tf2-msgs \
  ros-$ROS_VERSION-tf2-sensor-msgs \
  mesa-utils \
  wget \
  vim \
  ros-$ROS_VERSION-rviz \ 
  libxrender1 \
  libfontconfig1 \
  x11-apps \
  libgl1-mesa-dri \
  ros-$ROS_VERSION-teleop-twist-keyboard \
  python3-pip \
  build-essential \
 && rm -rf /var/lib/apt/lists/* \
 && apt-get clean

# Create ws and check out git
RUN mkdir -p /root/workspace/catkin_ws; mkdir -p /root/workspace/catkin_ws/src

# Atualizar pip, numpy e instalar open3d
RUN pip3 install --upgrade pip \
 && pip3 install --upgrade numpy \
 && pip3 install open3d

# Copy taurob_robots_describtion pkg
COPY . /root/workspace/catkin_ws/src/taurob_robots_description

# Source setup.bash and build the ros pkg using catkin_make
RUN /bin/bash -c '. /opt/ros/$ROS_VERSION/setup.bash; cd /root/workspace/catkin_ws; catkin_make'

# Add aliases for bash script
ADD misc/.bash_aliases /root

# Fix ignite error
RUN mkdir -p /root/.ignition/fuel
COPY misc/config.yaml /root/.ignition/fuel

# Graphics instruction set to OpenGL2
#ENV SVGA_VGPU10=0
ENV LIBGL_ALWAYS_SOFTWARE=1


#Linha adicionada
# Definindo variáveis de ambiente para X11
ENV DISPLAY=:0
ENV QT_X11_NO_MITSHM=1
