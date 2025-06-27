FROM nvidia/cuda:12.8.0-devel-ubuntu22.04

# Create a non-root user and group for security
RUN addgroup --system appgroup
RUN adduser --system appuser --ingroup appgroup

# Install dependencies for DearPyGui and ZMQ
RUN apt-get update \
    && apt-get --no-install-recommends install -y \
    libgl1-mesa-glx \
    libglib2.0-0 \
    python3 \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

RUN ln -sf /usr/bin/python3 /usr/bin/python \
    && ln -sf /usr/bin/pip3 /usr/bin/pip

# Switch to the non-root user for security
USER appuser

# Install Python dependencies
# - pytorch: for CUDA 12.8
# - dearpygui: GUI framework for Python
# - zmq: ZeroMQ messaging library
# - opencv-python: Computer vision library
# - protobuf: Protocol Buffers library

RUN pip install torch --index-url https://download.pytorch.org/whl/cu128

RUN pip install \
    dearpygui \
    pyzmq==26.4.0 \
    opencv-python \
    protobuf==5.26.0

# Set the working directory for the application
WORKDIR /isaac-zmq-server/src

# Add the source code to the image (We will also mount it for dynamic updates)
ADD ./src /isaac-zmq-server/src