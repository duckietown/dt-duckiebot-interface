ARG ARCH=arm32v7

FROM duckietown/dt-ros-commons:master19-${ARCH}

RUN ["cross-build-start"]

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

RUN apt-get update && apt-get install -y \
    ros-kinetic-tf-conversions \
    ros-kinetic-joy

ARG REPO_PATH="${CATKIN_WS_DIR}/src/duckiebot-interface"

# create repo directory
RUN mkdir -p "${REPO_PATH}"

# copy entire repo
COPY . "${REPO_PATH}/"

RUN pip install -r ${REPO_PATH}/requirements.txt

# build packages
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
  catkin build \
    --workspace ${CATKIN_WS_DIR}/

# turn off ARM emulation
RUN ["cross-build-end"]

LABEL maintainer="Andrea F. Daniele (afdaniele@ttic.edu)"

CMD ["${REPO_PATH}/run_all_drivers.sh"]
