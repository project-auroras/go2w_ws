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

# PlotJuggler 4 from its apt repo (PC only, not on the NX)
COPY docker/plotjuggler.asc /etc/apt/keyrings/plotjuggler.asc
RUN echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/plotjuggler.asc] https://apt.plotjuggler.io stable main" \
        > /etc/apt/sources.list.d/plotjuggler.list \
    && apt-get update \
    && apt-get install -y --no-install-recommends plotjuggler4 \
    && rm -rf /var/lib/apt/lists/*

# CUDA 13 + cuDNN 9 runtime for the yolo_ros ONNX Runtime GPU build (RTX 50xx needs CUDA 13).
# PC only; the NX uses JetPack's CUDA. Build yolo_ros with -DONNX_GPU=ON.
RUN curl -fsSL -o /tmp/cuda-keyring.deb \
        https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb \
    && dpkg -i /tmp/cuda-keyring.deb && rm /tmp/cuda-keyring.deb \
    && apt-get update \
    && apt-get install -y --no-install-recommends cuda-libraries-13-0 libcudnn9-cuda-13 \
    && rm -rf /var/lib/apt/lists/*
ENV CUDA_VERSION=13.0
ENV LD_LIBRARY_PATH=/usr/local/cuda/lib64

RUN groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd --uid ${USER_UID} --gid ${USER_GID} -m -s /bin/bash ${USERNAME} \
    && echo "${USERNAME} ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

USER ${USERNAME}
RUN rosdep update --rosdistro humble \
    && echo 'source /ws/scripts/env.sh' >> ~/.bashrc

WORKDIR /ws
CMD ["bash"]
