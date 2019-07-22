ARG ARCH=arm32v7

FROM duckietown/dt-ros-commons:master19-${ARCH}

RUN ["cross-build-start"]

RUN apt-get update && apt-get install -y \
    ros-kinetic-tf-conversions \
    ros-kinetic-joy

COPY requirements.txt /requirements.txt

RUN pip install -r /requirements.txt

RUN mkdir "${SOURCE_DIR}"
COPY ./catkin_ws "${SOURCE_DIR}/catkin_ws"

# build packages
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
  catkin_build \
    -j \
    -C ${SOURCE_DIR}/catkin_ws/

# turn off ARM emulation
RUN ["cross-build-end"]

LABEL maintainer="Andrea F. Daniele (afdaniele@ttic.edu)"

# CMD ["./run_all_drivers.sh"]
