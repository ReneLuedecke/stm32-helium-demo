# Xi 640 ETH - Zephyr Development Container
# Complete isolated environment for testing without local Zephyr installation

FROM ubuntu:22.04

# Prevent interactive prompts during installation
ENV DEBIAN_FRONTEND=noninteractive

# Install system dependencies
RUN apt-get update && apt-get install -y \
    git \
    cmake \
    ninja-build \
    gperf \
    ccache \
    dfu-util \
    device-tree-compiler \
    wget \
    python3-dev \
    python3-pip \
    python3-setuptools \
    python3-wheel \
    xz-utils \
    file \
    make \
    gcc \
    g++ \
    gcc-multilib \
    g++-multilib \
    libsdl2-dev \
    libmagic1 \
    && rm -rf /var/lib/apt/lists/*

# Install west
RUN pip3 install west

# Download and install Zephyr SDK
WORKDIR /opt
RUN wget -q https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v0.16.8/zephyr-sdk-0.16.8_linux-x86_64.tar.xz && \
    tar xf zephyr-sdk-0.16.8_linux-x86_64.tar.xz && \
    rm zephyr-sdk-0.16.8_linux-x86_64.tar.xz && \
    cd zephyr-sdk-0.16.8 && \
    ./setup.sh -h -c

# Set environment variables
ENV ZEPHYR_SDK_INSTALL_DIR=/opt/zephyr-sdk-0.16.8
ENV PATH="${PATH}:/opt/zephyr-sdk-0.16.8"

# Create workspace
WORKDIR /workspace

# Copy project files (optional - can also mount volume)
# COPY . /workspace

# Default command
CMD ["/bin/bash"]

# Build instructions:
#   docker build -t xi640-zephyr .
#
# Run container:
#   docker run -it --rm -v $(pwd):/workspace xi640-zephyr
#
# Inside container:
#   west init -l .
#   west update
#   west build -b native_sim
#   ./build/zephyr/zephyr.exe
