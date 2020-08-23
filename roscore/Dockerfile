FROM ros:melodic-ros-core
WORKDIR /roscore
RUN apt-key del 421C365BD9FF1F717815A3895523BAEEB01FA116
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN apt update && \
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
COPY run.sh .
RUN chmod +x run.sh
ENTRYPOINT ["./run.sh"]
