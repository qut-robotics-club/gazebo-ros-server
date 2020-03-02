FROM gazebo:gzserver7-xenial

# install gzweb requirements
RUN apt update && apt upgrade && apt install -y \
    gazebo7 \
    libgazebo7-dev \
    libjansson-dev \
    # todo: remove this
    nodejs \
    libboost-dev \
    cmake \
    build-essential \
    mercurial

RUN apt install curl

# install ros kinetic (supports gazebo7)
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \
    curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | apt-key add - && \
    # try to fix broken hash dum mismatch for binfmt-support_2.1.6-1_amd64.deb
    echo "Acquire::By-Hash \"yes\"; ">/etc/apt/apt.conf.d/01byhash && \
    apt update && \
    apt install -y ros-kinetic-ros-base

# initialize rosdep
RUN rosdep init

# setup python development packages
RUN curl -sL https://deb.nodesource.com/setup_13.x | sudo -E bash - && \
    apt install -y \
        nodejs \
        python-rosinstall \
        python-rosinstall-generator \
        python-wstool \
        imagemagick

# setup development user
ARG USER="user"
RUN useradd -ms /bin/bash ${USER}
USER ${USER}
WORKDIR /home/${USER}

# setup ROS and Gazebo development environment (variables & PATH)
RUN . /opt/ros/kinetic/setup.sh && \
    . /usr/share/gazebo/setup.sh && \
    rosdep update

# build gzweb
RUN hg clone https://bitbucket.org/osrf/gzweb && \
    cd gzweb && \
    hg up gzweb_1.4.0 && \
    npm run deploy --- -m -c

# setup catkin workspace
RUN mkdir -p ws/src && \
    cd ws && \
    git clone https://github.com/osrf/rvizweb/ && \
    rosdep install --from-paths src --ignore-src -r -y && \
    # configure catkin_make to use python3
    catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 && \
    . devel/setup.bash && catkin_make install


EXPOSE 8080
CMD gzserver --verbose &; cd gzweb && npm start -p 8080