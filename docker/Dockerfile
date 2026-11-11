# Generated / assisted by Grok
# Hummingbird mapping workspace image (ROS 2 Humble).
#
#   docker build -f docker/Dockerfile -t hummingbird:humble .            # x86_64 dev box
#   docker build -f docker/Dockerfile -t hummingbird:humble \
#       --build-arg BASE=dustynv/ros:humble-ros-base-l4t-r36.2.0 .       # Jetson (JetPack 6), UNTESTED
#
#   docker run --rm -it --net=host --privileged -v /dev:/dev hummingbird:humble \
#       ros2 launch hummingbird_bringup mapping.launch.py
#
# --net=host: the Mid-360 talks UDP on 192.168.1.0/24 and DDS discovery needs the host network.
ARG BASE=ros:humble-ros-base
FROM ${BASE}

ARG LIVOX_SDK2_URL=https://github.com/lucascygu/Livox-SDK2.git
ARG LIVOX_SDK2_REF=c0796f04c143143899c87a773d9f6b7136453c0b
ARG XRCE_AGENT_REF=v2.4.2
ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
        git build-essential cmake python3-pip python3-vcstool python3-colcon-common-extensions \
        python3-numpy python3-yaml python3-pytest \
        libpcl-dev libeigen3-dev \
        ros-humble-pcl-ros ros-humble-pcl-conversions ros-humble-tf2-ros \
        ros-humble-sensor-msgs-py ros-humble-std-srvs ros-humble-nav-msgs \
    && rm -rf /var/lib/apt/lists/*

# Livox-SDK2 (system install, required by livox_ros_driver2)
RUN git clone ${LIVOX_SDK2_URL} /opt/Livox-SDK2 && cd /opt/Livox-SDK2 && git checkout ${LIVOX_SDK2_REF} \
    && mkdir build && cd build && cmake .. && make -j"$(nproc)" && make install && ldconfig

# Micro XRCE-DDS Agent v2.x (PX4 uXRCE-DDS client is v2.x; v3.x agent is incompatible)
RUN git clone -b ${XRCE_AGENT_REF} https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /opt/xrce-agent \
    && cd /opt/xrce-agent && mkdir build && cd build && cmake .. && make -j"$(nproc)" && make install && ldconfig

WORKDIR /ws
COPY ros2_ws/hummingbird.repos /ws/hummingbird.repos
RUN mkdir -p src && vcs import --recursive src < hummingbird.repos \
    # livox_ros_driver2 ships package_ROS1/ROS2.xml; build.sh normally does this swap.
    && cp src/livox_ros_driver2/package_ROS2.xml src/livox_ros_driver2/package.xml \
    && cp -r src/livox_ros_driver2/launch_ROS2 src/livox_ros_driver2/launch

COPY ros2_ws/src/ /ws/src/
RUN apt-get update && rosdep update --rosdistro humble \
    && rosdep install --from-paths src --ignore-src -r -y --rosdistro humble \
        --skip-keys "livox_ros_driver2 fast_lio" \
    && rm -rf /var/lib/apt/lists/*

RUN source /opt/ros/humble/setup.bash \
    && colcon build --symlink-install --parallel-workers 2 \
        --cmake-args -DCMAKE_BUILD_TYPE=Release -DROS_EDITION=ROS2 -DDISTRO_ROS=humble

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
