FROM ros:melodic-ros-core

WORKDIR /ros
RUN apt update && apt upgrade -y

# Install ros
RUN apt update && apt upgrade -y && apt install -y curl gnupg2 lsb-release locales && apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

ENV DEBIAN_FRONTEND=noninteractive

RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
RUN export LANG=en_US.UTF-8
RUN apt update && apt install -y ros-melodic-desktop-full --install-recommends
RUN apt install -y ros-melodic-catkin python-catkin-pkg python-catkin-tools
RUN apt install -y --install-recommends python-catkin-pkg python-catkin-pkg-modules
RUN apt install -y python-rosdep python-pip python-rospkg python-dev libssl1.0-dev nodejs nodejs-dev node-gyp npm git \
    ros-melodic-rospy ros-melodic-roslib
#RUN apt update && apt install -y ros-melodic-desktop-full python3-rosdep python3-catkin-pkg python3-rospkg
RUN rosdep init
RUN rosdep update

# Install posenet
WORKDIR /
RUN pip install tensorflow
RUN git clone https://github.com/tensorflow/tfjs-models.git
WORKDIR /tfjs-models/posenet
RUN npm install
# Additional node packages for alternative posenet deployment
RUN npm install p5@1.1.9 ml5@0.3.1 p5.js-svg p5.js-plot

# Create webserver
RUN apt update && apt install -y apache2 gnupg
EXPOSE 80
EXPOSE 443
EXPOSE 5000
COPY templates/*.* /var/www/html/
RUN echo "ServerName 127.0.0.1" >> /etc/apache2/apache2.conf
RUN service apache2 start
WORKDIR /pose/src

# Init catkin workspace
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash && /bin/bash -c catkin_init_workspace"
RUN catkin_create_pkg publisher std_msgs rospy roscpp
WORKDIR /pose/src/publisher
RUN mkdir python
RUN apt install python python-pip
RUN pip install rosdep rospkg rosinstall_generator rosinstall wstool vcstools catkin_tools catkin_pkg
COPY *.* python/
RUN chmod +x -R python
RUN pip2 install -r python/requirements.txt

# Setup catkin package
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash && /bin/bash -c \
    echo catkin_install_python\(PROGRAMS python/poser.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}\) >> CMakeLists.txt"
RUN mv python/package.xml .
WORKDIR /pose
RUN rosdep install --from-paths src/ -r
RUN catkin init
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash && /bin/bash -c catkin_make"
RUN rosdep update
RUN mv /pose/src/publisher/python/run.sh .
ENTRYPOINT ["./run.sh"]