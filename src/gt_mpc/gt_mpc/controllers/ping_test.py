from game_controller import GameController
import time

# dx_min = 2. # safety longitudinal distance, in car length-unit
# dl_min = 0.5 # safety lateral distance, 1=lane width
# v_min = 3.
# v_max = 10.
# right lane: l = 0.5
# left lane: -l = 0.5

# feasible state [p1, v1, l1, p2, v2, l2]
# choose p1 = 0 => p2 < p1 - dx_min = -2 => choose p2 = -4
# 3 < v1, v2 < 10 => choose v1, v2 = 6
# choose cars both in right lane (platoon): l1 = l2 = 0.5
state = [0.0, 6.0, 0.5, -4.0, 6.0, 0.5]
case = "Platoon"

print("Loading GameController...")
t0 = time.time()
gc = GameController()
print(f'finished in {time.time() - t0:.1f}s')
print("GameController (and Julia) loaded successfully!")

print(f'Computing 1st control for state: {state}, case: {case}')
t0 = time.time()
u1, u2 = gc.compute_control(state, case)
print(f'finished in {time.time() - t0:.1f}s')
print(f'u1 = {u1} | u2 = {u2}')
print(f'type(u1): type: {type(u1).__name__}')
print(f'type(u1[0]): type: {type(u1[0]).__name__}')

print(f'Computing 2nd control for state: {state}, case: {case}')
t0 = time.time()
u1, u2 = gc.compute_control(state, case)
print(f'finished in {time.time() - t0:.1f}s')
print(f'u1 = {u1} | u2 = {u2}')