# Description:   Robotnik RB-THERON simulation.
#
# Company:       Robotnik Automation S.L.
# Creation Year: 2023
# Author:        Guillem Gari <ggari@robotnik.es>
#
#
# Copyright (c) 2023, Robotnik Automation S.L.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Robotnik Automation S.L. nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL Robotnik Automation S.L. BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# Global ARGs
ARG output_path=/tmp
ARG output_pkg=${output_path}/deb.pkgs.tar
ARG base_image="robotnik/ros"
ARG ros_distro="noetic"
ARG image_base_version="use-args"
ARG version="use-args"
# With no local copy or the repos
# base: use the git from vcs
# local: use the ones from the local repo
# ARG builder_type=base
ARG builder_type=local

FROM ${base_image}:${ros_distro}-gui-$image_base_version as base

LABEL maintainer="Guillem Gari <ggari@robotnik.es>" \
      org.opencontainers.image.title="rb-theron-${ros_distro}" \
      org.opencontainers.image.version="${version} " \
      org.opencontainers.image.description="RB Theron gazebo simulation." \
      org.opencontainers.image.source="https://github.com/RobotnikAutomation/rb_theron_sim.git" \
      org.opencontainers.image.vendor="Robotnik Automation S.L." \
      org.opencontainers.image.licenses="BSD-3-Clause" \
      org.opencontainers.image.authors="Guillem Gari <ggari@robotnik.es>" \
      org.opencontainers.image.date="2023-11-03"

FROM ${base_image}:${ros_distro}-builder-$image_base_version as builder_base

USER root

ENV DEBIAN_FRONTEND=noninteractive

# Install compiled packages
RUN --mount=type=bind,\
target=/tmp/requirements.txt,\
source=container/builder/requirements/builder.txt \
    true \
    && if \
        curl -IsS http://ros.mirror.robotnik.ws &>/dev/null; then \
        sed -i \
            's#packages.ros.org#ros.mirror.robotnik.ws#' \
            /etc/apt/sources.list.d/ros-latest.list ;\
        fi \
    && apt-fast update \
    && apt-fast install -q -y \
        --no-install-recommends \
        $(eval "echo $(cat /tmp/requirements.txt | xargs)") \
    && apt-get clean -q -y \
    && apt-get autoremove -q -y \
    && rm -rf /var/lib/apt/lists/* \
    && true

USER ${USER_NAME}

RUN --mount=type=bind,\
source=./repos/rb_theron_sim.repos.yaml,\
target=/tmp/common.repo.yml,ro \
     vcs import \
        --input /tmp/common.repo.yml  \
        --shallow

# Debs tweaks
RUN true \
	&& touch ${USER_WORKSPACE}/src/rb_theron_common/rb_theron_navigation_private/CATKIN_IGNORE \
	&& touch ${USER_WORKSPACE}/src/rb_theron_common/rb_theron_common_private/CATKIN_IGNORE \
	&& touch ${USER_WORKSPACE}/src/rb_theron_common/rb_theron_perception/CATKIN_IGNORE \
	&& touch ${USER_WORKSPACE}/src/rb_theron_common/rb_theron_robot_local_control/CATKIN_IGNORE \
	&& true

RUN true \
	&& rm -rf  ${USER_WORKSPACE}/src/rb_theron_common/rb_theron_navigation_private \
	&& rm -rf  ${USER_WORKSPACE}/src/rb_theron_common/rb_theron_common_private \
	&& rm -rf  ${USER_WORKSPACE}/src/rb_theron_common/rb_theron_perception \
	&& rm -rf  ${USER_WORKSPACE}/src/rb_theron_common/rb_theron_robot_local_control \
	&& true

# increase the version in order to force to ours
RUN sed -i "s#<version>1.0.0</version>#<version>1.1.0</version>#" ${USER_WORKSPACE}/src/ira_laser_tools/package.xml


# optional use of local repos for simulation
FROM builder_base as builder_local

ENV SIM_PATH=${USER_WORKSPACE}/src/rb_theron_sim

RUN true \
	&& rm -rf ${SIM_PATH} \
	&& mkdir -p ${SIM_PATH}

COPY --chown=$user_name \
	rb_theron_gazebo \
	$SIM_PATH/rb_theron_gazebo

COPY --chown=$user_name \
	rb_theron_sim \
	$SIM_PATH/rb_theron_sim

COPY --chown=$user_name \
	rb_theron_sim_bringup \
	$SIM_PATH/rb_theron_sim_bringup


FROM builder_${builder_type} AS builder

ENV DEBIAN_FRONTEND=noninteractive

# Generate deb packages
RUN generate_debs.sh

ARG output_pkg
RUN tar -cvzf ${output_pkg} ./debs

FROM ${base_image}:${ros_distro}-builder-${image_base_version} AS compressed

ARG output_path
ARG output_pkg
COPY --from=builder ${output_pkg} ${output_path}/builder.pkgs.tar.gz

RUN find ${output_path} -name '*.pkgs.tar.gz' -exec tar -xvzf {} -C ${output_path} \;

# Generate Packages.gz
RUN cd ${output_path}/debs && dpkg-scanpackages . | gzip -9c > Packages.gz

# Compress all
RUN cd ${output_path} && tar -cvzf ${output_pkg} ./debs


# Output setup
FROM alpine:3.17.5 AS packages
RUN mkdir -p /data
WORKDIR /data
ARG output_pkg
COPY --from=compressed ${output_pkg} /data/debs.pkgs.tar.gz
ARG pkg_path=/data/debs.pkgs.tar.gz
RUN cd /data && tar -xf $pkg_path --strip-components=1

# Download gazebo models
FROM base AS gazebo-models

USER root
RUN true \
    && apt-fast update \
    && apt-fast install -q -y \
    --no-install-recommends \
        unzip \
	&& apt-get clean -q -y \
	&& apt-get autoremove -q -y \
	&& rm -rf /var/lib/apt/lists/*

USER ${USER_NAME}
RUN true \
    && mkdir -p /tmp/gazebo_models \
    && cd /tmp/gazebo_models \
    && aria2c \
        --no-conf -c -j 5 -x 10 -s 8 \
        --min-split-size=1M \
        --connect-timeout=200 --timeout=200 -m0 \
        https://codeload.github.com/osrf/gazebo_models/zip/refs/heads/master \
        -o models.zip \
    && unzip models.zip \
    && mv gazebo_models-master/* . \
    && rm -rf gazebo_models-master models.zip

FROM base

USER root
# Install dependancies
RUN \
    --mount=\
type=bind,\
target=/tmp/requirements.txt,\
source=container/builder/requirements/simulation.txt \
    true \
    && if \
        curl -IsS http://ros.mirror.robotnik.ws &>/dev/null; then \
        sed -i \
            's#packages.ros.org#ros.mirror.robotnik.ws#' \
            /etc/apt/sources.list.d/ros-latest.list ;\
        fi \
    && apt-fast update \
    && apt-fast install -q -y \
        --no-install-recommends \
        $(eval "echo $(cat /tmp/requirements.txt | xargs)") \
    && sed -i 's#ros.mirror.robotnik.ws#packages.ros.org#' /etc/apt/sources.list.d/ros-latest.list \
    && apt-get clean -q -y \
    && apt-get autoremove -q -y \
    && rm -rf /var/lib/apt/lists/* \
    && true

COPY \
    --from=gazebo-models \
    --chown=${USER_NAME}:${USER_NAME} \
    /tmp/gazebo_models/ \
    ${USER_HOME}/.gazebo/models/


# Install compiled packages
RUN \
    --mount=\
type=bind,\
from=packages,\
source=/data/debs,\
target=/tmp/debs \
    --mount=\
type=bind,\
target=/tmp/requirements.txt,\
source=container/builder/requirements/local.txt \
    true \
    && echo "deb [trusted=yes] file:///tmp/debs/ ./" | tee /etc/apt/sources.list.d/debs.list \
    && apt-fast update \
    && apt-fast install -q -y \
        --no-install-recommends \
        $(eval "echo $(cat /tmp/requirements.txt | xargs)") \
    && apt-get clean -q -y \
    && apt-get autoremove -q -y \
    && rm -rf /var/lib/apt/lists/* \
    && rm /etc/apt/sources.list.d/debs.list \
    && true

USER ${USER_NAME}

RUN mkdir -p $USER_HOME/config

# Set environment variables
ENV STARTUP_TYPE        "launch"
ENV ROS_BU_PKG          "rb_theron_sim_bringup"
ENV ROS_BU_LAUNCH       "rb_theron_complete.launch"


ENV ROBOT_ID            "robot"

