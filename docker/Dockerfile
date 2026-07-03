# Dockerfile for SVEA images
#
# Author: Kaj Munhoz Arfvidsson

ARG BUILD_TAG
ARG ROSDISTRO

FROM ${BUILD_TAG:-ghcr.io/kth-sml/svea:latest}

ARG ROSDISTRO
ARG DEBIAN_FRONTEND=noninteractive

## Install dependencies using apt-get

RUN apt-get update -y && \
    apt-get install --no-install-recommends -y \
    ros-$ROSDISTRO-foxglove-msgs

# # You can add apt dependencies like this...
# RUN apt-get install --no-install-recommends -y \
#         git curl

# ======== FOR GT-MPC ======================

# Install Julia
RUN curl -fsSL https://install.julialang.org | sh -s -- --yes
ENV PATH="/root/.juliaup/bin:$PATH"

# Install juliacall early (before COPY src) so we can use it to install
# Julia packages into juliacall's own pyjuliapkg environment
RUN pip install --break-system-packages juliacall

# Install Julia packages through juliacall so they land in the pyjuliapkg
# environment that juliacall actually uses. Note:
# - Pkg.add downloads packages.
# - "using" DyNECT and DAQPBase precompiles them to speed up import time.
# - DyNECT is installed from GitHub fork with changed project.toml file
RUN python3 -c "\
from juliacall import Main as jl; \
jl.seval('import Pkg'); \
jl.seval('Pkg.add(url=\"https://github.com/darnstrom/ParametricDAQP.jl\")'); \
jl.seval('Pkg.add(url=\"https://github.com/nicomignoni/Monviso.jl\")'); \
jl.seval('Pkg.add(\"DAQPBase\")'); \
jl.seval('Pkg.add(url=\"https://github.com/harskarodlan/DyNECT\", rev=\"iterative_linearization\")'); \
jl.seval('using DyNECT; using DAQPBase'); \
print('Julia packages OK'); \
"

# ==========================================

# This is for any added ROS packages
COPY src ./src

COPY entrypoint requirements.txt ./
RUN rosdep install \
        --rosdistro $ROSDISTRO \
        --from-paths src \
        --ignore-src \
        -qry \
        && \
    pip install \
        --break-system-packages \
        -r requirements.txt

# Build the ROS 2 workspace (improves build speeds later on).
RUN /bin/bash -c "source /opt/ros/$ROSDISTRO/setup.bash && \
    colcon build --symlink-install"
