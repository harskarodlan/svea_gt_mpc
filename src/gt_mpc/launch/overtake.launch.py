#! /usr/bin/env python3
from better_launch import BetterLaunch, launch_this

# MAP_NAME = "sml"

# same map image as floor2, but no matching .obstacles.yaml, 
# so sim_lidar's obstacle ray-casting never runs (see simulation.launch.py)
# (sim_lidar was taking too much time, causing scheduling conflict with controller.
# Since obstacle detection is not needed in this scenario, we dont need sim_lidar.)
MAP_NAME = "floor2_no_obstacles" 

@launch_this
def main(
    is_sim: bool = True,
    use_foxglove: bool = True,
    # FOR FLOOR2
    initial_pose_x_b: float = -7.3434,
    initial_pose_y_b: float = -15.1011,
    initial_pose_a: float = +0.9,
    initial_pose_x_a: float = -5.2,
    initial_pose_y_a: float = -12.4,
    # FOR SML
    # initial_pose_x_b: float = 0.5,
    # initial_pose_y_b: float = -5.0,
    # initial_pose_a: float = +1.57079632679,
    # initial_pose_x_a: float = 0.5,
    # initial_pose_y_a: float = 0.0,
    points: str = '[[-2.3, -7.1], [10.5, 11.7], [5.7,  15.0], [-7.0, -4.0]]',
):
    bl = BetterLaunch()

    if not is_sim:
        # REAL SVEA
        # TO BE FIXED

        # Start SVEA in real-world mode
        bl.include("svea_core", "svea.launch.py",
                   is_sim=is_sim, # = False
                   map_name=MAP_NAME,
                   initial_pose_x=initial_pose_x_a,
                   initial_pose_y=initial_pose_y_a,
                   initial_pose_a=initial_pose_a)

        # The SVEA launch system is built to be compatible with multiple SVEAs running simultaneously.
        # Default name is "self", so to add the pure_pursuit node we need to namespace accordingly.
        with bl.group("self"):
        
            bl.node("svea_examples", "overtake.py",
                    name="overtake",
                    params={'points': points})

    if is_sim:
        # Start two SVEAs (svea_a and svea_b) in simulation, each with its own car_bridge node
        # Also start a centralized overtake controller node

        # ====== SVEA CARS WITH CAR BRIDGE ==================

        INITIAL_POSES = {
            "svea_a": (initial_pose_x_a, initial_pose_y_a, initial_pose_a),
            "svea_b": (initial_pose_x_b, initial_pose_y_b, initial_pose_a),
        }

        for name, (init_x, init_y, init_a) in INITIAL_POSES.items():
            
            bl.include("svea_core", "svea.launch.py",
                       name=name,
                       is_sim=is_sim,
                       is_indoor=True,
                       map_name=MAP_NAME,
                       initial_pose_x=init_x,
                       initial_pose_y=init_y,
                       initial_pose_a=init_a)

            # Add namespace to car_bridge node so that it can be run for each SVEA independently
            with bl.group(name):

                bl.node("gt_mpc", "car_bridge.py",
                        name="car_bridge",
                        params={
                            # "points": points,
                            "localization/base_frame": f"{name}/base_link",
                        })

        # ====== OVERTAKE CONTROLLER ======================

        bl.node("gt_mpc", "overtake.py",
                name="overtake",
                params={
                    "initial_pose_x_a": initial_pose_x_a,
                    "initial_pose_y_a": initial_pose_y_a,
                    "initial_pose_x_b": initial_pose_x_b,
                    "initial_pose_y_b": initial_pose_y_b,
                    "initial_pose_a": initial_pose_a,
                })
        

    bl.include("svea_core", "map_and_foxglove.launch.py",
               map_name=MAP_NAME,
               use_foxglove=use_foxglove)
