#!/bin/bash

apt-get update && apt-get install -y --no-install-recommends \
	inetutils-ping \
	iproute2 \
	mesa-utils \
	nano \
	net-tools \
	build-essential \
	cmake \
	git \
	wget \
	apt-transport-https \
	pkg-config \
	libatlas-base-dev \
	libboost-all-dev \
	libgflags-dev \
	libgoogle-glog-dev \
	libhdf5-serial-dev \
	libleveldb-dev \
	liblmdb-dev \
	libopencv-dev \
	libprotobuf-dev \
	libsnappy-dev \
	libavcodec-dev \
	libavformat-dev \
	libswscale-dev \
	libdc1394-22-dev \
	libtbb2 \
	libtbb-dev \
	libjpeg-dev \
	libpng-dev \
	libtiff-dev \
	libcanberra-gtk3-module \
	libgconf2-dev \
	libgsl-dev \
	protobuf-compiler \
	python-dev \
	python-numpy \
	python-pip \
	python-setuptools \
	python-scipy \
	python-sklearn \
	python-tk \
	python-pymodbus \
	exuberant-ctags

curl https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > microsoft.gpg \
	&& mv microsoft.gpg /etc/apt/trusted.gpg.d/microsoft.gpg \
	&& echo "deb [arch=amd64] https://packages.microsoft.com/repos/vscode stable main" > /etc/apt/sources.list.d/vscode.list

apt-get update && apt-get install -y --no-install-recommends \
	code

update-alternatives --set editor /usr/bin/code

apt-get update && apt-get install -y --no-install-recommends \
	gvfs-bin \
	fonts-takao \
	libxtst6

code --install-extension ms-python.python --user-data-dir=/root/.vscode/ \
	&& code --install-extension ms-vscode.cpptools --user-data-dir=/root/.vscode/ \
	&& code --install-extension PeterJausovec.vscode-docker --user-data-dir=/root/.vscode/ \
	&& code --install-extension streetsidesoftware.code-spell-checker --user-data-dir=/root/.vscode/ \
	&& code --install-extension eamodio.gitlens --user-data-dir=/root/.vscode/ \
	&& code --install-extension ajshort.ros --user-data-dir=/root/.vscode/ \
	&& code --install-extension ajshort.msg --user-data-dir=/root/.vscode/

echo 'deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo bionic main' | tee /etc/apt/sources.list.d/realsense-public.list
apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
apt-get update && apt-get install -y --no-install-recommends \
	librealsense2-dkms \
	librealsense2-utils \
	librealsense2-dev \
	librealsense2-dbg

apt-get update && apt-get install -y \
	python-catkin-tools \
	ros-melodic-moveit \
	ros-melodic-moveit-core \
	ros-melodic-moveit-ros \
	ros-melodic-moveit-kinematics \
	ros-melodic-moveit-resources \
	ros-melodic-moveit-ros-planning-interface \
	ros-melodic-moveit-visual-tools \
	ros-melodic-gazebo-ros-control \
	ros-melodic-ros-controllers \
	ros-melodic-universal-robot
	ros-melodic-dynamixel-sdk \
	ros-melodic-qt-build \
	ros-melodic-dynamixel-workbench \
	ros-melodic-dynamixel-workbench-msgs \
	ros-melodic-dynamixel-workbench-toolbox \
	ros-melodic-ddynamic-reconfigure \
	ros-melodic-fetch-gazebo \
	ros-melodic-fetch-ros

# Install Python linter as recommended by Python extension.
/usr/bin/python -m pip install -U \
	pylint \
	autopep8

pip install pyserial
pip install opencv-python

dpkg --remove --force-depends python-pyassimp
pip install --upgrade pip
pip install --upgrade pyassimp

