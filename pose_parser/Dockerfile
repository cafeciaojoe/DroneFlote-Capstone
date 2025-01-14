FROM ros:melodic-ros-core
RUN apt-key del 421C365BD9FF1F717815A3895523BAEEB01FA116
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

ENV DEBIAN_FRONTEND=noninteractive
RUN apt update && apt dist-upgrade -y && apt install -y curl gnupg2 apt-utils lsb-release locales git python3 python3-pip
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
        ros-melodic-cpp-common \
        # Messaging requirements for imports
        # ros-melodic-trajectory-msgs ros-melodic-geometry-msgs python-trajectory-msgs python-geometry-msgs
        ros-melodic-common-msgs \
        libeigen3-dev ros-melodic-eigenpy \
        ros-melodic-mav-msgs

RUN rosdep init
RUN rosdep update

# Web server setup for pose interface TODO --- Late milestone interface to python, use flask webserver or apache?
#RUN apt update && apt install -y apache2 gnupg
#EXPOSE 80
#EXPOSE 443
#EXPOSE 9999
#RUN echo "ServerName 127.0.0.1" >> /etc/apache2/apache2.conf
# TODO - serve basic page to alter pose metrics

# Install reqs
#RUN apt update && apt upgrade -y

WORKDIR /parser/src
# Clone standard msgs extras

RUN /bin/bash -c "source /opt/ros/melodic/setup.bash && /bin/bash -c catkin_init_workspace"

# Copy python demo listener to workspace package
RUN catkin_create_pkg parser std_msgs rospy roscpp trajectory_msgs mav_msgs geometry_msgs eigenpy
WORKDIR /parser/src/parser/python
#RUN mkdir python
#COPY *.* python/
COPY *.* ./
#RUN git clone -b jade-devel https://github.com/ros/common_msgs.git
#RUN cp -r common_msgs/* ${CATKIN_PACKAGE_BIN_DESTINATION}

WORKDIR /parser/src/parser
RUN chmod +x -R python
RUN pip3 install wheel
RUN pip3 install -r python/requirements.txt
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash && /bin/bash -c \
    echo catkin_install_python\(PROGRAMS python/parser.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}\) >> CMakeLists.txt"
#RUN /bin/bash -c "source /opt/ros/melodic/setup.bash && /bin/bash -c \
#    echo include_directories\(${Eigen_INCLUDE_DIRS}\) >> CMakeLists.txt"

WORKDIR /parser
RUN git clone -b HTTYD https://github.com/cafeciaojoe/crazyflie-clients-python.git
RUN python3 -m pip install -e crazyflie-clients-python/


ENV ROS_PYTHON_VERSION=3
RUN rosdep update
RUN echo "rosdep install --from-paths src/ -r -y" > rosinst.sh
RUN chmod +x rosinst.sh
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash && /bin/bash -c ./rosinst.sh"
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash && /bin/bash -c catkin_make"

# Init the workspace
RUN catkin init
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash && /bin/bash -c catkin_make"
RUN rosdep update
RUN apt install -y python3-pyqt5

# Move script to local dir
RUN mv /parser/src/parser/python/start.sh .

# Start script to run package
ENTRYPOINT ["./start.sh"]
