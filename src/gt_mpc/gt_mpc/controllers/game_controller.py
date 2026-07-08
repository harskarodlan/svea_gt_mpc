from juliacall import Main as jl

class GameController:

    def __init__(self):
        # Overtake is a Julia package (src/gt_mpc/gt_mpc/julia/Overtake),
        # precompiled ahead of time (including a warm-up call to solve_step)
        print("loading Overtake...")
        jl.seval("using Overtake")
        print("Overtake loaded!")



    def compute_control(self, state, case):
        """
        Compute the control actions (steering, acceleration) using game-theoretic MPC.
        
        :param state: Current joint state of the vehicles 
                      [p1, v1, l1, p2, v2, l2]
        :param mode: Current case as string, either "platoon" or "overtake"
        :return: controls u1, u2
        """
        # compute control at this time step ut = [u1, u2]
        ut = jl.Overtake.solve_step(state, case)

        # controls for each car ui = [acceleration, steering]
        # convert from juliacall.Float64 to regular floats
        u1 = [float(ut[0][0]), float(ut[0][1])] # svea_a
        u2 = [float(ut[1][0]), float(ut[1][1])] # svea_b
        
        return u1, u2