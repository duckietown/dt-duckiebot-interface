ARG ARCH=arm32v7

FROM duckietown/dt-ros-commons:master19-${ARCH}

RUN ["cross-build-start"]

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

RUN apt-get update && apt-get install -y \
    ros-kinetic-tf-conversions \
    ros-kinetic-joy

COPY requirements.txt /requirements.txt

RUN pip install -r /requirements.txt

COPY ./catkin_ws "${SOURCE_DIR}/catkin_ws"
COPY ["docker_setup.sh", "run_all_drivers.sh", "${SOURCE_DIR}/"]

# build packages
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
  catkin build \
    --workspace ${SOURCE_DIR}/catkin_ws/

# turn off ARM emulation
RUN ["cross-build-end"]

LABEL maintainer="Andrea F. Daniele (afdaniele@ttic.edu)"

# CMD ["./run_all_drivers.sh"]
