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
    ros-humble-demo-nodes-py && \
    rm -rf /var/lib/apt/lists/*

RUN PYTHONWARNINGS="ignore:::setuptools.command.install,ignore:::setuptools.command.easy_install,ignore:::pkg_resources" && export PYTHONWARNINGS

#RUN apt-get install -y software-properties-common
#RUN add-apt-repository ppa:deadsnakes/ppa -y
#RUN apt-get install python3.8 python3.8-dev python3.8-venv

RUN ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash"]

USER duser
WORKDIR /home/duser
