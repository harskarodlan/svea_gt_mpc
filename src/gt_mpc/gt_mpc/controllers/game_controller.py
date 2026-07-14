from juliacall import Main as jl

class GameController:
    # car length in meters
    # OBS: must match lcar in Overtake.jl
    LCAR = 0.523 

    def __init__(self):
        # Overtake is a Julia package (src/gt_mpc/gt_mpc/julia/Overtake),
        # precompiled ahead of time (including a warm-up call to solve_step)
        print("loading Overtake...")
        jl.seval("using Overtake")
        print("Overtake loaded!")



    def compute_control(self, state, case):
        """
        Compute the control actions (steering, acceleration) using game-theoretic MPC.
        
        :param state: Current joint state of the vehicles in SI units
                      [p1, v1, l1, yaw1, p2, v2, l2, yaw2]
        :param mode: Current case as string, either "platoon" or "overtake"
        :return: controls u1, u2 in SI units
        """
        # Overtake.jl expects state in car lenght units, so convert from meters
        p1, v1, l1, yaw1, p2, v2, l2, yaw2 = state
        state_lcar = [p1/self.LCAR, v1/self.LCAR, l1/self.LCAR, yaw1,
                      p2/self.LCAR, v2/self.LCAR, l2/self.LCAR, yaw2]
        # compute control at this time step ut = [u1, u2]
        ut = jl.Overtake.solve_step(state_lcar, case)

        # controls for each car ui = [acceleration, steering]
        # convert from juliacall.Float64 to regular floats
        # Overtake.jl returns controls in SI units, so no conversion needed
        u1 = [float(ut[0][0]), float(ut[0][1])] # svea_a
        u2 = [float(ut[1][0]), float(ut[1][1])] # svea_b
        
        return u1, u2