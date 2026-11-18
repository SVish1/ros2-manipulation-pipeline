FROM osrf/ros:humble-desktop

RUN apt-get update && apt-get install -y \
    ros-humble-moveit \
    ros-humble-moveit-resources \
    ros-humble-moveit-resources-panda-moveit-config \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-controller-manager \
    python3-pip \
    python3-colcon-common-extensions \
    python3-pytest \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install pytest

WORKDIR /workspace

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

CMD ["bash"]
