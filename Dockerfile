FROM ubuntu:18.04

# install curl
RUN apt update && apt upgrade && apt install -y curl lsb-core

# add ros package server
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \
    curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | apt-key add - && \
    # fix broken hash checksum mismatch for binfmt-support_2.1.6-1_amd64.deb dependency
    echo "Acquire::By-Hash \"yes\"; " > /etc/apt/apt.conf.d/01byhash

# add gazebo package server
RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-latest.list' && \
    curl -sSL "http://packages.osrfoundation.org/gazebo.key" | apt-key add -

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Australia/Brisbane

# install debian requirements
RUN apt update && \
    apt upgrade && \
    apt install -y \
        gazebo9 \
        libgazebo9-dev \
        libjansson-dev \
        # todo: remove this
        nodejs \
        libboost-dev \
        cmake \
        build-essential \
        mercurial \
        ros-melodic-ros-core

# install nodejs 10 - it supports native es6 but also still v8::Handle (deprecated) needed for gzweb
RUN curl -sL https://deb.nodesource.com/setup_10.x | sudo -E bash - && \
    apt install -y \
        nodejs \
        python3-catkin-pkg \
        python3-rosdistro \
        python3-rosdep \
        python3-rospkg \
        python3-rosinstall \
        python3-rosinstall-generator \
        python3-wstool \
        imagemagick


ENV ROS_PYTHON_VERSION 3

# initialize rosdep
RUN rosdep init

# setup development user
ARG MASTER_USER="ros"
ARG MASTER_PASS
RUN useradd -ms /bin/bash ${MASTER_USER} -p ${MASTER_PASS}
USER ${MASTER_USER}
WORKDIR /home/${MASTER_USER}

RUN rosdep update

# build gzweb
RUN hg clone https://bitbucket.org/osrf/gzweb && \
    cd gzweb && \
    hg up gzweb_1.4.0 && \
    npm run deploy --- -m -c

# setup catkin workspace
RUN mkdir -p ros/src && \
    cd ros && \
    git clone https://github.com/osrf/rvizweb/ && \
    rosdep install --from-paths src --ignore-src -r -y && \
    # configure catkin_make to use python3
    . /opt/ros/melodic/setup.sh && \
    catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 && \
    . devel/setup.bash && catkin_make install

ARG GZWEB_PORT="8080"
ARG RVIZWEB_PORT="8081"

EXPOSE ${GZWEB_PORT} ${RVIZWEB_PORT}
CMD tmux new-session -s "backend" -d && \
    tmux split-window -h bash -c "gzserver --verbose" && \
    tmux split-window -h bash -c "cd gzweb && npm start -p ${GZWEB_PORT}" && \
    tmux split-window -h bash -c "roslaunch rvizweb rvizweb.launch" && \
    tmux -2 attach-session -d