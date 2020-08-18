#FROM ubuntu:bionic
FROM ros:melodic-ros-core
WORKDIR /ros
RUN apt update && apt upgrade -y

# Install ros

RUN apt update && apt upgrade -y && apt install -y curl gnupg2 lsb-release locales && apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
#RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" >> /etc/apt/sources.list.d/ros-latest.list'
#RUN sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros-latest.list'

ENV DEBIAN_FRONTEND=noninteractive

RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
RUN export LANG=en_US.UTF-8
RUN apt update && apt install -y ros-melodic-desktop-full --install-recommends
RUN apt install -y python-rosdep python-catkin-pkg python-pip python-rospkg python-dev libssl1.0-dev nodejs nodejs-dev node-gyp npm git ros-melodic-rospy
#RUN apt update && apt install -y ros-melodic-desktop-full python3-rosdep python3-catkin-pkg python3-rospkg
RUN rosdep init
RUN rosdep update

# Install posenet
WORKDIR /
RUN pip install tensorflow
RUN git clone https://github.com/tensorflow/tfjs-models.git
WORKDIR /tfjs-models/posenet
#RUN npm install @tensorflow/tfjs-core @tensorflow/tfjs-converter
#RUN npm install @tensorflow-models/posenet
RUN npm install

# Create webserver
RUN apt update && apt install -y apache2 gnupg
EXPOSE 80
EXPOSE 443
COPY templates/*.* /var/www/html/
RUN echo "ServerName 0.0.0.0" >> /etc/apache2/apache2.conf
RUN service apache2 restart

# Get python running
WORKDIR /pose
COPY *.* /pose/
RUN apt install python python-pip
RUN pip install -r requirements.txt
RUN pip install rosdep rospkg rosinstall_generator rosinstall wstool vcstools catkin_tools catkin_pkg
RUN rosdep update
CMD ["python poser.py"]