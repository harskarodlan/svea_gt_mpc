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
# - Pkg.add downloads packages.
# - DyNECT is installed from GitHub fork with changed project.toml file.
# - Monviso is pinned to an older commit because newer Monviso versions
#   removed the "VI" type that this DyNECT fork still relies on.
# - Deliberately does NOT install Overtake.jl here so that editing it
#   does not invalidate cache of the below packages.
RUN python3 -c "\
from juliacall import Main as jl; \
jl.seval('import Pkg'); \
jl.seval('Pkg.add(url=\"https://github.com/darnstrom/ParametricDAQP.jl\")'); \
jl.seval('Pkg.add(url=\"https://github.com/nicomignoni/Monviso.jl\", rev=\"a99d4e447e39db8086c54906ab1d5d09a8f983a0\")'); \
jl.seval('Pkg.add(\"DAQPBase\")'); \
jl.seval('Pkg.add(url=\"https://github.com/harskarodlan/DyNECT\", rev=\"iterative_linearization\")'); \
"

# Bind mount and precompile local Overtake package seperately so that it compiles
# against the final package versions written to disk above 
# (needed not to trigger recompilation of these packages at every run).
# - Pkg.develop instead of Pkg.add for bind mounting
# - 'using Overtake' precompiles local Overtake package
# Bind mounting means that the local Overtake.jl file is used, so it can be edited and 
# re-run triggering only a ~1-2 min recompile of Overtake.jl, not the other packages.
RUN --mount=type=bind,source=./src/gt_mpc/gt_mpc/julia/Overtake,target=/svea_ws/src/gt_mpc/gt_mpc/julia/Overtake,readonly python3 -c "\
from juliacall import Main as jl; \
jl.seval('import Pkg'); \
jl.seval('Pkg.develop(path=\"/svea_ws/src/gt_mpc/gt_mpc/julia/Overtake\")'); \
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
