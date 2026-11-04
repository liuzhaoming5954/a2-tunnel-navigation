# Candidate local development image; freeze ROS_IMAGE to a digest after M0.
# This image contains no Unitree SDK, sensor driver, or robot launch entrypoint.
ARG ROS_IMAGE=ros:humble-ros-base-jammy
FROM ${ROS_IMAGE}

ARG DEV_UID=1000
ARG DEV_GID=1000
USER root
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git \
    python3-colcon-common-extensions python3-rosdep python3-vcstool \
    python3-pytest \
    ros-${ROS_DISTRO}-rosidl-default-generators \
    ros-${ROS_DISTRO}-diagnostic-msgs ros-${ROS_DISTRO}-ament-cmake-gtest \
    && rm -rf /var/lib/apt/lists/*
RUN groupadd --gid ${DEV_GID} dev \
    && useradd --uid ${DEV_UID} --gid ${DEV_GID} --create-home dev
ENV ROS_LOCALHOST_ONLY=1
ENV ROS_DOMAIN_ID=42
WORKDIR /work/a2_project
USER dev
CMD ["bash"]
