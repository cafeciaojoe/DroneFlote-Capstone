#FROM ubuntu:bionic
#WORKDIR /ros/src
#RUN apt update && apt upgrade -y
#
## Install ros
#
#RUN apt update && apt upgrade -y && apt install -y curl gnupg2 apt-utils lsb-release locales && apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
#RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" >> /etc/apt/sources.list.d/ros-latest.list'
##RUN sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros-latest.list'
#
#ENV DEBIAN_FRONTEND=noninteractive
#
#RUN locale-gen en_US en_US.UTF-8
#RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
#RUN export LANG=en_US.UTF-8
#
## Install ros melodic packages
#RUN apt update && apt install -y ros-melodic-desktop-full --install-recommends ros-melodic-std-msgs
#RUN apt install -y ros-melodic-joy ros-melodic-octomap-ros ros-melodic-mavlink ros-melodic-octomap-mapping ros-melodic-catkin
#RUN apt update && apt install -y ros-melodic-control-msgs ros-melodic-dynamic-reconfigure ros-melodic-realtime-tools ros-melodic-roscpp ros-melodic-message-runtime
#RUN apt update && apt install -y python-rosdep python-rospkg git python-rosinstall python-rosinstall-generator
#RUN apt install -y build-essential python-wstool protobuf-compiler libgoogle-glog-dev
#RUN rosdep init
#RUN rosdep update
#
#RUN apt install -y python-catkin-tools --install-recommends ros-melodic-std-msgs
#RUN apt install -y python-catkin-pkg
#RUN apt install -f
##RUN apt install -y catkin
##RUN rosdep init
#RUN rosdep update
#
## Init workspace
#RUN chmod +x /opt/ros/melodic/setup.bash
#RUN /opt/ros/melodic/setup.bash
#RUN git clone -b dev/ros-melodic https://github.com/gsilano/CrazyS.git
#RUN git clone -b med18_gazebo9 https://github.com/gsilano/mav_comm.git
#
## Install deps for gazebo
#WORKDIR /ros
#RUN catkin init
#RUN rosdep install --from-paths src -i -y
#RUN apt install ros-melodic-rqt-rotors ros-melodic-rotors-comm ros-melodic-mav-msgs ros-melodic-rotors-control
#RUN apt install ros-melodic-rotors-gazebo ros-melodic-rotors-evaluation ros-melodic-rotors-joy-interface
#RUN apt install ros-melodic-rotors-gazebo-plugins ros-melodic-mav-planning-msgs ros-melodic-rotors-description ros-melodic-rotors-hil-interface

FROM ros:melodic-ros-core
RUN apt-key del 421C365BD9FF1F717815A3895523BAEEB01FA116
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

ENV DEBIAN_FRONTEND=noninteractive
RUN apt update && apt dist-upgrade -y && apt install -y curl gnupg2 apt-utils lsb-release locales git
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
RUN export LANG=en_US.UTF-8

# Setup basic deps for ros
RUN apt update && apt dist-upgrade -y && \
    # Install some base dependencies
    apt install -y \
	iproute2 host wget less sudo vim-tiny iputils-ping python-wstool build-essential
RUN apt-get -qq update && \
    # Install some base dependencies
    apt-get -qq install -y \
        # Required for installing dependencies
        python-rosdep \
        # Preferred build tool
        python-catkin-tools
ENV TERM xterm
RUN rosdep init
RUN rosdep update

# Install gazebo and reqs
#RUN echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable xenial main" > /etc/apt/sources.list.d/gazebo-stable.list
#RUN wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add -
#RUN apt install -y gazebo
#RUN curl -sSL http://get.gazebosim.org | sh
#RUN apt update && apt upgrade -y && apt -y install ros-melodic-gazebo-ros libgazebo9 libignition-common ros-melodic-gazebo-dev
RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
RUN wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
RUN apt update && apt install -y gazebo9 gazebo9-* ros-melodic-gazebo-* && apt upgrade -y

# Clone CrazyS
WORKDIR /simulator/src
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash && /bin/bash -c catkin_init_workspace"
RUN git clone -b dev/ros-melodic https://github.com/gsilano/CrazyS.git
#RUN git clone https://github.com/gsilano/mav_comm.git
WORKDIR /simulator
RUN rosdep update
RUN echo "rosdep install --from-paths src/ -r -y" > rosinst.sh
RUN chmod +x rosinst.sh
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash && /bin/bash -c ./rosinst.sh"
#RUN rosdep install --from-paths src/ -r

# Init the workspace
RUN catkin init
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash && /bin/bash -c catkin_make"
RUN rosdep update
