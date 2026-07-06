from juliacall import Main as jl
from pathlib import Path

class GameController:

    def __init__(self):
        # get current parent directory: /svea_ws/src/gt_mpc/controllers/
        dir_path = Path(__file__).parent


        print("loading overtake_solver.jl...")
        # call overtake_solver.jl through julia code str eval
        # overtake_solver.jl defines game, parameters and solver function
        jl.seval(f'include("{dir_path}/overtake_solver.jl")')
        print("overtake_solver.jl loaded! \nWarm up 'Platoon' compute_control...")

        # dummy warm up call to compute_control to compile solve_step 
        # (takes ~45s)
        self.compute_control([0., 6., .5, -4., 6., .5], 'Platoon')
        print("Warm up done!")



    def compute_control(self, state, case):
        """
        Compute the control actions (steering, acceleration) using game-theoretic MPC.
        
        :param state: Current joint state of the vehicles 
                      [p1, v1, l1, p2, v2, l2]
        :param mode: Current case as string, either "platoon" or "overtake"
        :return: controls u1, u2
        """
        # compute control at this time step ut = [u1, u2]
        ut = jl.solve_step(state, case)

        # controls for each car ui = [acceleration, steering]
        # convert from juliacall.Float64 to regular floats
        u1 = [float(ut[0][0]), float(ut[0][1])] # svea_a
        u2 = [float(ut[1][0]), float(ut[1][1])] # svea_b
        
        return u1, u2