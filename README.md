# SVEA GT-MPC

Game-theoretic model predictive control for application on SVEA platform.

# Installation

## Install Docker Engine
For the instructions below to work, you need to first install Docker Engine.

For Windows users, you will need to install WSL 2 and set it's distribution to
Ubuntu (20 or higher recommended). Use the commands listed [here](https://learn.microsoft.com/en-us/windows/wsl/basic-commands)
to "Install", "List available Linux distributions", and "Set default WSL Version" to
Ubuntu 20 or higher.

To install Docker Engine, follow the instructions for your operating system
[here](https://docs.docker.com/engine/).

## Installing the Docker image
Start by going to the folder where you want the code to reside.
For example, choose the home directory or a directory for keeping projects in.
Once you are in the chosen directory, use the command:

```bash
git clone git@github.com:harskarodlan/svea_gt_mpc.git
```

to download the library. Then, a new directory will appear called
`./svea_gt_mpc`. Go into the directory with command:

```bash
cd svea_gt_mpc
```

To install the Docker image containing the entire codebase run:

```bash
util/build
```

If it all runs without an error, you have installed the Docker image!

## Installing Foxglove Studio
For visualization, we recommend Foxglove Studio. To install Foxglove Studio,
follow the instructions for your operating system [here](https://foxglove.dev/download)

**Note**: alternatively, you can use the Web version of Foxglove Studio which is
also available from the installation link.

# Running in simulation

Enter the Docker container by going to the root of `svea_gt_mpc` and running:

```bash
util/run
```

Once inside the container, launch the game-theoretic
overtake controller with both SVEAs simulated:

```bash
ros2 launch gt_mpc overtake.launch.py
```

This launches two simulated SVEAs (`svea_a`, `svea_b`) on the `floor2` map,
the centralized `overtake` controller node, a `car_bridge` node per vehicle,
and the Foxglove bridge — all defaults are already set for simulation
(`is_sim:=True`), so no extra arguments are needed.

In Foxglove Studio, click "Open connection" and connect to your local host.

Once everything has loaded and both simulated SVEAs show up, publish `true`
to the `/start` topic to begin the overtake maneuver, e.g.:

```bash
ros2 topic pub /start std_msgs/msg/Bool "data: true" --once
```

Or in Foxglove Studio, add a 'Publish' panel to publish to the `/start` topic. When ready, set `data` to `true` and press Publish.


