# Development image for the PC: same OS and ROS as the NX (Ubuntu 22.04 + Humble).
# The NX does not use this image, it builds natively (scripts/nx_update.sh).
ARG BASE_IMAGE=ros:humble-ros-base
FROM ${BASE_IMAGE}

ARG USERNAME=dev
ARG USER_UID=1000
ARG USER_GID=1000

ENV DEBIAN_FRONTEND=noninteractive

COPY scripts/install_deps.sh /tmp/install_deps.sh
RUN /tmp/install_deps.sh system \
    && apt-get install -y --no-install-recommends sudo bash-completion less vim \
    && rm -rf /var/lib/apt/lists/* /tmp/install_deps.sh

RUN groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd --uid ${USER_UID} --gid ${USER_GID} -m -s /bin/bash ${USERNAME} \
    && echo "${USERNAME} ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

USER ${USERNAME}
RUN rosdep update --rosdistro humble \
    && echo 'source /ws/scripts/env.sh' >> ~/.bashrc

WORKDIR /ws
CMD ["bash"]
