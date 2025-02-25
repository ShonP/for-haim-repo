FROM axisecp/acap-native-sdk:latest-aarch64-ubuntu24.04

# Install additional development tools
RUN apt-get update && apt-get install -y \
    git \
    curl \
    wget \
    build-essential \
    cmake \
    gdb \
    sudo \
    && rm -rf /var/lib/apt/lists/*

# Set the working directory
WORKDIR /workspace

# Ensure root is the default user
USER root 