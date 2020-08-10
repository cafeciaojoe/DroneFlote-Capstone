FROM ubuntu:focal
WORKDIR /ros
RUN apt update && apt upgrade -y

# Install ros

RUN apt update && apt upgrade -y && apt install -y curl gnupg2 lsb-release locales && apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" >> /etc/apt/sources.list.d/ros-latest.list'
#RUN sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros-latest.list'

ENV DEBIAN_FRONTEND=noninteractive

RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
RUN export LANG=en_US.UTF-8

RUN apt update && apt install -y ros-melodic-desktop-full python3-rosdep python3-catkin-pkg python3-rospkg
RUN rosdep init
RUN rosdep update

# Install posenet