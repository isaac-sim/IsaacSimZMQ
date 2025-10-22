FROM nvidia/cuda:12.8.0-devel-ubuntu22.04

# Create a non-root user and group for security
RUN addgroup --system appgroup
RUN adduser --system appuser --ingroup appgroup

# Detect architecture during build time
ARG TARGETARCH
RUN echo "Target architecture: ${TARGETARCH}"

# Install dependencies based on architecture
RUN apt-get update && \
    if [ "${TARGETARCH}" = "arm64" ]; then \
        # Full dependencies for aarch64 (to build DearPyGui)
        apt-get --no-install-recommends install -y \
        python3 \
        python3-pip \
        libgl1-mesa-glx \
        libglib2.0-0 \
        git \
        build-essential \
        cmake \
        libpython3-dev \
        python3-setuptools \
        python3-wheel \
        libgl1-mesa-glx \
        libx11-dev \
        libgl-dev \
        libxrandr-dev \
        libxinerama-dev \
        libxcursor-dev \
        libxi-dev \
        && rm -rf /var/lib/apt/lists/*; \
    else \
        # Minimal dependencies for x86_64
        apt-get --no-install-recommends install -y \
        libgl1-mesa-glx \
        libglib2.0-0 \
        python3 \
        python3-pip \
        && rm -rf /var/lib/apt/lists/*; \
    fi

RUN ln -sf /usr/bin/python3 /usr/bin/python \
    && ln -sf /usr/bin/pip3 /usr/bin/pip

# Switch to the non-root user for security
USER appuser

# Install Python dependencies
# - dearpygui: GUI framework for Python
RUN if [ "${TARGETARCH}" = "arm64" ]; then \
        # For aarch64, build dearpygui from source
        cd /tmp && \
        git clone https://github.com/hoffstadt/DearPyGui.git && \
        cd DearPyGui && \
        git checkout v2.0.0 && git submodule update --init --recursive && \
        python3 -m setup bdist_wheel --plat-name linux_$(uname -m) --dist-dir ../dist && \
        cd ../dist && \
        pip install *.whl; \
    else \
        # For x86_64, install dearpygui directly
        pip install dearpygui==2.0.0; \
    fi

ENV MESA_GL_VERSION_OVERRIDE=4.5

# Install Python dependencies
# - pytorch: for CUDA 12.8
# - zmq: ZeroMQ messaging library
# - opencv-python: Computer vision library
# - protobuf: Protocol Buffers library
RUN pip install torch --index-url https://download.pytorch.org/whl/cu128
RUN pip install \
    pyzmq==26.4.0 \
    opencv-python \
    protobuf==5.26.0

# Set the working directory for the application
WORKDIR /isaac-zmq-server/src

# Add the source code to the image (We will also mount it for dynamic updates)
ADD ./src /isaac-zmq-server/src