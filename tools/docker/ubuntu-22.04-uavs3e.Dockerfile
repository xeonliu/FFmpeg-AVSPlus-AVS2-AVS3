FROM ubuntu:22.04

RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
        build-essential \
        ca-certificates \
        cmake \
        gawk \
        git \
        nasm \
        perl \
        pkg-config \
        python3 \
        xz-utils \
        yasm && \
    rm -rf /var/lib/apt/lists/*
