# Development image for the PC: same OS and ROS as the NX (Ubuntu 22.04 + Humble).
# The NX does not use this image, it builds natively (scripts/nx_update.sh).
ARG BASE_IMAGE=ros:humble-ros-base

# Only the package.xml files of src/, so the rosdep layer below is rebuilt
# when dependencies change, not on every source edit.
FROM ${BASE_IMAGE} AS pkgs
COPY src /src
RUN mkdir /pkgs && cd /src && find . -name package.xml -exec cp --parents {} /pkgs \;

FROM ${BASE_IMAGE}

ARG USERNAME=dev
ARG USER_UID=1000
ARG USER_GID=1000

ENV DEBIAN_FRONTEND=noninteractive

COPY scripts/install_deps.sh /tmp/install_deps.sh
RUN /tmp/install_deps.sh system \
    && apt-get install -y --no-install-recommends sudo bash-completion less vim \
    && rm -rf /var/lib/apt/lists/* /tmp/install_deps.sh

# Gazebo Harmonic + its ros_gz from the OSRF repo (PC only, not on the NX).
# navmap_gis_tool generates gz-sim worlds, which Fortress (Humble's default) can't run.
RUN apt-get update \
    && apt-get install -y --no-install-recommends curl gnupg lsb-release \
    && curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
        -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=amd64 signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
        > /etc/apt/sources.list.d/gazebo-stable.list \
    && apt-get update \
    && apt-get install -y --no-install-recommends gz-harmonic ros-humble-ros-gzharmonic \
    && rm -rf /var/lib/apt/lists/*

# Workspace dependencies (same as `install_deps.sh ws`), baked in so a
# recreated container doesn't lose them. src/external must be imported first.
# ros_gz keys resolve to the Fortress packages on Humble, which conflict with
# ros-humble-ros-gzharmonic installed above.
COPY --from=pkgs /pkgs /tmp/ws_pkgs
RUN apt-get update \
    && rosdep update --rosdistro humble \
    && rosdep install --from-paths /tmp/ws_pkgs --ignore-src --rosdistro humble -y -r \
        --skip-keys "ros_gz ros_gz_sim ros_gz_bridge gz_sim_vendor gz_plugin_vendor" \
    && rm -rf /var/lib/apt/lists/* /tmp/ws_pkgs

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

# EGL of the NVIDIA driver (mounted by nvidia-container-toolkit) for Gazebo's sensor
# rendering: without this vendor file glvnd only knows Mesa, which can't use the NVIDIA GPU
# ("OpenGL 3.3 is not supported" / "Failed to create dummy render window")
RUN mkdir -p /usr/share/glvnd/egl_vendor.d \
    && printf '{"file_format_version": "1.0.0", "ICD": {"library_path": "libEGL_nvidia.so.0"}}\n' \
        > /usr/share/glvnd/egl_vendor.d/10_nvidia.json

RUN groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd --uid ${USER_UID} --gid ${USER_GID} -m -s /bin/bash ${USERNAME} \
    && echo "${USERNAME} ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

USER ${USERNAME}
RUN rosdep update --rosdistro humble \
    && echo 'source /ws/scripts/env.sh' >> ~/.bashrc

WORKDIR /ws
CMD ["bash"]
