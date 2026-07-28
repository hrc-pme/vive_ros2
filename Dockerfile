FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive

WORKDIR /ros2_ws


RUN apt-get update && apt-get install -y \
    ros-humble-rmw-cyclonedds-cpp \
    libopenvr-dev \
    git \
    python3-pip \
    nano \
    net-tools \
    iputils-ping \
    && rm -rf /var/lib/apt/lists/*


RUN mkdir -p /root/libraries && \
    git clone https://github.com/ValveSoftware/openvr.git /root/libraries/openvr


COPY src /ros2_ws/src
COPY record_vive.sh /ros2_ws/record_vive.sh
COPY entrypoint.sh /entrypoint.sh


RUN chmod +x /ros2_ws/record_vive.sh /entrypoint.sh


RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]