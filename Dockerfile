# Start from Ubuntu 16.04
FROM ubuntu:16.04

WORKDIR /app

# Set maintainer label
LABEL maintainer="your-email@example.com"

# Install necessary dependencies and tools
RUN apt-get update && apt-get install -y \
    software-properties-common \
    build-essential \
    g++-5 \
    gcc-5 \
    git \
    cmake \
    wget \
    libusb-1.0-0-dev \
    libudev-dev \
    libboost-all-dev \
    libeigen3-dev \
    libflann-dev

# Set gcc and g++ version to 5.5
RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-5 60 \
    --slave /usr/bin/g++ g++ /usr/bin/g++-5 \
    && update-alternatives --config gcc

# Install PCL v1.7.2
RUN apt-get install -y \
    libvtk5-dev \
    libvtk5.10 \
    libpcap-dev \
    libqhull* \
    libproj-dev

RUN cd /tmp && \
    wget https://github.com/PointCloudLibrary/pcl/archive/pcl-1.7.2.tar.gz && \
    tar xvf pcl-1.7.2.tar.gz && \
    cd pcl-pcl-1.7.2 && \
    mkdir build && cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release .. && \
    make -j$(nproc) && \
    make install && \
    cd / && rm -rf /tmp/*

# Set the C++ standard to C++11
ENV CXX_STANDARD 11

# Command to keep the container running
CMD ["bash"]
