FROM osrf/ros:humble-desktop-full

ARG USER_ID
ARG GROUP_ID

RUN if [ ${USER_ID:-0} -ne 0 ] && [ ${GROUP_ID:-0} -ne 0 ]; then \
    groupadd -g ${GROUP_ID} duser && \
    useradd -l -u ${USER_ID} -g duser duser && \
    install -d -m 0755 -o duser -g duser /home/duser && \
    usermod -aG sudo duser && \
    echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers\
;fi

# Setup ROS
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        lsb-release \
        wget \
        gnupg \
        ros-humble-turtlesim \
        ros-humble-rqt* \
        ros-humble-navigation2 \
        ros-humble-nav2-bringup \
        ros-humble-turtlebot3* \
        ros-humble-gazebo-* \
        ros-humble-demo-nodes-cpp \
        ros-humble-demo-nodes-py \
        python3-venv \
        libcanberra-gtk-module \
        libcanberra-gtk3-module && \
    rm -rf /var/lib/apt/lists/*

RUN ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash"]

# Build OpenCV (for optional modules)
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        cmake \
        gcc \
        g++ \
        python3-dev \
        python3-numpy \ 
        libavcodec-dev \
        libavformat-dev \
        libswscale-dev \
        libgstreamer-plugins-base1.0-dev \
        libgstreamer1.0-dev \
        libgtk-3-dev \
        git

# OpenCV contrib has the extra modules
RUN cd ~ &&\
    git clone https://github.com/opencv/opencv_contrib

RUN cd ~ &&\
    git clone https://github.com/opencv/opencv.git &&\
    cd opencv &&\
    mkdir build &&\
    cd build &&\
    cmake -D CMAKE_INSTALL_PREFIX=/usr/local \
        -D OPENCV_EXTRA_MODULES_PATH=~/opencv_contrib/modules \
        -D BUILD_EXAMPLES=OFF .. &&\
    make -j$(nproc) &&\
    make install &&\
    ldconfig

USER duser
WORKDIR /home/duser

RUN export PYTHONWARNINGS="ignore:::setuptools.command.install,ignore:::setuptools.command.easy_install,ignore:::pkg_resources"
