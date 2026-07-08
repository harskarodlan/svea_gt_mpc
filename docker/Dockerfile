# Dockerfile for SVEA images
#
# Author: Kaj Munhoz Arfvidsson

ARG BUILD_TAG
ARG ROSDISTRO

FROM ${BUILD_TAG:-ghcr.io/kth-sml/svea:latest}

ARG ROSDISTRO
ARG DEBIAN_FRONTEND=noninteractive

## Update Infrastructure (may have changed from base build)

COPY entrypoint ./

## Setup Development Environment

RUN apt-get update -y

# # You can add extra apt dependencies like this...
# RUN apt-get install --no-install-recommends -y \
#         tmux htop

# Foxglove support
RUN apt-get install --no-install-recommends -y \
        ros-$ROSDISTRO-foxglove-msgs

# ======== FOR GT-MPC ======================

# Install Julia
RUN curl -fsSL https://install.julialang.org | sh -s -- --yes
ENV PATH="/root/.juliaup/bin:$PATH"

# Install juliacall early (before COPY src) so we can use it to install
# Julia packages into juliacall's own pyjuliapkg environment
RUN pip install --break-system-packages juliacall

# Install Julia packages through juliacall so they land in the pyjuliapkg
# environment that juliacall actually uses.
# - Pkg.add downloads packages and resolves/updates the shared Manifest.
# - DyNECT is installed from GitHub fork with changed project.toml file.
# - Monviso is pinned to an older commit because newer Monviso versions
#   removed the "VI" type that this DyNECT fork still relies on.
# - Overtake is our own local package (src/gt_mpc/gt_mpc/julia/Overtake),
#   added via Pkg.develop so it's loaded straight from that path - editing
#   it later doesn't need a rebuild, just a relaunch (src/ is live-mounted
#   at runtime the same way it's bind-mounted here at build time).
RUN --mount=type=bind,source=./src,target=./src,readonly python3 -c "\
from juliacall import Main as jl; \
jl.seval('import Pkg'); \
jl.seval('Pkg.add(url=\"https://github.com/darnstrom/ParametricDAQP.jl\")'); \
jl.seval('Pkg.add(url=\"https://github.com/nicomignoni/Monviso.jl\", rev=\"a99d4e447e39db8086c54906ab1d5d09a8f983a0\")'); \
jl.seval('Pkg.add(\"DAQPBase\")'); \
jl.seval('Pkg.add(url=\"https://github.com/harskarodlan/DyNECT\", rev=\"iterative_linearization\")'); \
jl.seval('Pkg.develop(path=\"/svea_ws/src/gt_mpc/gt_mpc/julia/Overtake\")'); \
"

# Run this in a new process (not the one that ran Pkg.add above), so it
# compiles against the final package versions written to disk - not
# versions still sitting in that other process's memory. Otherwise the
# cache doesn't match what's actually used later, so Julia recompiles
# everything every time the app runs.
# "using" DyNECT and DAQPBase precompiles them to speed up import time.
RUN --mount=type=bind,source=./src,target=./src,readonly python3 -c "\
from juliacall import Main as jl; \
jl.seval('using Overtake'); \
print('Julia packages OK'); \
"

# ==========================================

# Install pip packages specified in requirements.txt
RUN --mount=type=bind,source=./requirements.txt,target=/tmp/requirements.txt,readonly \
    pip install \
        --break-system-packages \
        -r /tmp/requirements.txt

# (Pre-)Build the ROS 2 workspace
# This improves build speeds later on.
RUN --mount=type=bind,source=./src,target=./src,readonly /bin/bash <<EOF
set -e
source /opt/svea/$ROSDISTRO/setup.bash
rosdep install \
    --rosdistro $ROSDISTRO \
    --from-paths src \
    --ignore-src \
    -qry
colcon build --symlink-install
EOF
