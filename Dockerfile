# syntax=docker/dockerfile:1.3.1
FROM ros:galactic

ARG UID=1000
ARG PASS="docker"
ARG WORKSPACE=/home/dev/ros2_ws

ENV WORKSPACE=${WORKSPACE}
ENV DEBIAN_FRONTEND=noninteractive

RUN useradd -m -s /bin/bash dev -u ${UID} \
    && echo "dev:${PASS}" | chpasswd \
    && usermod -aG sudo dev

RUN --mount=target=/var/lib/apt/lists,type=cache,sharing=locked \
    --mount=target=/var/cache/apt,type=cache,sharing=locked \
    --mount=target=/etc/ros/rosdep/sources.list.d/,type=cache,sharing=locked \
    apt update \ 
    && apt install libopencv-dev -y \
    && cp -r /usr/include/opencv4/opencv2/ /usr/include/opencv2/ \
    && rm -rf /etc/ros/rosdep/sources.list.d/* \
    && rosdep init

USER dev
RUN --mount=target=/home/dev/.ros/,type=cache,sharing=locked,uid=${UID} \
    rosdep update --include-eol-distros

# This folder only contains package.xml files, so that the rosdep install command
# can be cached when only the source code changes
COPY --chown=dev:dev build/.docker/src ${WORKSPACE}/src

USER root

# Install ROS2 packages
RUN --mount=target=/var/lib/apt/lists,type=cache,sharing=locked \
    --mount=target=/var/cache/apt,type=cache,sharing=locked \
    . /opt/ros/galactic/setup.sh \
    && apt update \
    && rosdep install --from-paths ${WORKSPACE}/src/ --ignore-src -y

RUN chown -R dev /home/dev
USER dev

COPY --chown=dev:dev src/ ${WORKSPACE}/src/
COPY --chown=dev:dev mk ${WORKSPACE}

ARG MK_ARGS=""
RUN cd ${WORKSPACE} \
    && . /opt/ros/galactic/setup.sh \
    && ./mk

COPY --chown=dev:dev run ${WORKSPACE}
COPY --chown=dev:dev resources/default.rviz /home/dev/.rviz2/default.rviz

CMD cd ${WORKSPACE} \
    && . install/setup.sh \
    && ./run