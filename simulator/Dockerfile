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
        python-catkin-tools \
        # Requirements for CrazyS
        ros-melodic-catkin python-catkin-pkg python-catkin-tools \
        python-catkin-pkg python-catkin-pkg-modules \
        python-pip python-rospkg python-dev \
        ros-melodic-rospy ros-melodic-roslib \
        ros-melodic-cpp-common
ENV TERM xterm
RUN rosdep init
RUN rosdep update

# Install gazebo and reqs
RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
RUN wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
RUN apt update && apt install -y gazebo9 gazebo9-* ros-melodic-gazebo-* && apt upgrade -y

WORKDIR /simulator/src
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash && /bin/bash -c catkin_init_workspace"

# Copy python demo listener to workspace package
RUN catkin_create_pkg demo_listener std_msgs rospy roscpp
WORKDIR /simulator/src/demo_listener
RUN mkdir python
COPY *.* python/
RUN chmod +x -R python
RUN pip2 install -r python/requirements.txt
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash && /bin/bash -c \
    echo catkin_install_python\(PROGRAMS python/demo_listener.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}\) >> CMakeLists.txt"

# Clone only mavcomm as it must be compiled before CrazyS
WORKDIR /simulator/src
RUN git clone -b med18_gazebo9 https://github.com/gsilano/mav_comm.git
RUN apt install -y ros-melodic-rqt-rotors ros-melodic-rotors-comm ros-melodic-mav-msgs ros-melodic-rotors-control
RUN apt install -y ros-melodic-rotors-gazebo ros-melodic-rotors-evaluation ros-melodic-rotors-joy-interface
RUN apt install -y ros-melodic-rotors-gazebo-plugins ros-melodic-mav-planning-msgs ros-melodic-rotors-description ros-melodic-rotors-hil-interface
WORKDIR /simulator
RUN rosdep update
RUN echo "rosdep install --from-paths src/ -r -y" > rosinst.sh
RUN chmod +x rosinst.sh
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash && /bin/bash -c ./rosinst.sh"

# Clone CrazyS now MAV COMM dependencies are satisfied
WORKDIR /simulator/src
RUN git clone -b dev/ros-melodic https://github.com/gsilano/CrazyS.git
WORKDIR /simulator
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash && /bin/bash -c ./rosinst.sh"
RUN rosdep update

# Init the workspace
RUN catkin init
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash && /bin/bash -c catkin_make"
RUN rosdep update

RUN mv /simulator/src/demo_listener/python/pubsubdemo.sh .

# Demo for pubsub
ENTRYPOINT ["./pubsubdemo.sh"]
