module Overtake # make into a module so it can be cached by juliacall

using DyNECT
using DAQPBase
using PrecompileTools: @compile_workload   # for precompiling solve_step call
using PythonCall: PyList, pylist   # to build the same argument type juliacall passes, for the warmup call
#using LinearAlgebra
#using Plots
#using CSV
#using DataFrames
# using CommonSolve
#using DynamicalSystems
#using Statistics
#using Random
#Random.seed!(1234)

"""
On startup:
    - game_controller.py runs
        from juliacall import Main as jl
        jl.seval('using Overtake')

Overtake.jl (this package):
    - loads DyNECT
    - builds game
        - Defines game parameters, dynamics, objectives, constraints
        - Builds 'game' dict (DynGame objects for Platoon and Overtake)
        - Initialises nominal control sequence 'useq' as a global (persists between calls)
    - defines solve_step(xt, mode_str) used later to solve at each time step
        1. 'DyNECT.LQapprox' — linearise around useq/xt
        2. 'DynLQGame2mpAVI -> AVI -> DAQPBase.avi' — solve for δuseq
        3. 'ut = useq[1] + δuseq[1]' — first input to apply
        4. Update and shift useq for next call
        5. Return ut = [[a1, γ1], [a2, γ2]]

At each time step:
    - game_controller.py runs
        ut = jl.Overtake.solve_step(xt, mode)
    - extracts u1, u2 from ut
    - return u1, u2 to overtake.py
"""


lcar = 0.523 # car length in meters
lane_width = 1. 

# =================================================================================
# DEFINE GAME, based on nonlinear.jl in DyNECT repo example
# =================================================================================

# Parameters
#T_sim = 200 # Simulation length
T_hor = 10
Δt = 0.1 #sampling time
v_ref = [0.6, 1.1] ./ lcar # [5., 7.] # reference speed for each agent in car length/s
v_min = -0.1 # 3.
v_max = 1.2 / lcar # 10.
#d_overtake = 3. # Distance at which overtake is initiated, in car length-unit
a_max = 2. # max acceleration
a_min = -2.
angle_max = pi / 32
angle_min = -pi / 32
l_ref = [0.5*lane_width, -0.5*lane_width] ./ lcar #reference lateral position for normal and overtake lane

L = 0.32 # wheelbase from svea_core/models/bicycle.py
steer_max = 40*pi/180 # from ActuationInterface
steer_min = -steer_max

dx_min = 2. # safety longitudinal distance, in car length-unit
dl_min = 0.5*lane_width / lcar # safety lateral distance, 1=lane width

# Fixed values
N = 2
nu = [2,2]
nx = 8 # 6

## Initialization

# State:
#x0 = [0., (v_max + v_min) / 2, l_ref[1], -5., (v_max + v_min) / 2, l_ref[1]]
#u0=[zeros(2), zeros(2)]

# Dynamics
function unicycle_dynamics(x,u)
    # x = state [p, v, l]
    # u = control [a, γ] = [acceleration, steering]
    # OBS: γ is angle of velocity vector, as calculated to be 
    # the desired direction. 
    # I.e. it assumes instantaneous steering to the desired direction.
    # This is different from Ackerman-car model of SVEA, which takes
    # steering command as a control input to the global yaw: 
    #   From svea_core/svea_core/models/bicycle.py :
    #       x += vel * np.cos(yaw) * dt
    #       y += vel * np.sin(yaw) * dt
    #       yaw += vel / self.L * np.tan(delta) * dt
    #       vel += accel * dt
    #       self.state = (x, y, yaw, vel)
    #   where delta is steering angle input
    return [
        x[2] * cos(u[2]),   # dp/dt
        u[1],               # dv/dt
        x[2] * sin(u[2])    # dl/dt
    ]
end

# Dynamics
function bicycle_dynamics(x,u)
    # x = state [p, v, l, yaw]
    # u = control [a, delta] = [acceleration, steering]
    #   From svea_core/svea_core/models/bicycle.py :
    #       x += vel * np.cos(yaw) * dt
    #       y += vel * np.sin(yaw) * dt
    #       yaw += vel / self.L * np.tan(delta) * dt
    #       vel += accel * dt
    #       self.state = (x, y, yaw, vel)
    #   where delta is steering angle input
    return [
        x[2] * cos(x[4]),   # dp/dt
        u[1],               # dv/dt
        -x[2] * sin(x[4]),    # dl/dt
        x[2]/L * tan(u[2])
    ]
end

function f(x, u1, u2) # discretized unicycles
    dx = vcat(bicycle_dynamics(x[1:4],u1), bicycle_dynamics(x[5:8],u2))
    return x .+ Δt .* dx
end

# Continuous-time simulator
# function unicycle_dynamics!(dx::AbstractVector, x::AbstractVector, u::AbstractVector, t::Float64) # Compatible function with DynamicalSystems.jl
#     dx .= unicycle_dynamics(x,u)
#     return nothing
# end
# agent1 = CoupledODEs(unicycle_dynamics!, x0[1:3], zeros(2)) #initialize system with zero input
# agent2 = CoupledODEs(unicycle_dynamics!, x0[4:6], zeros(2))

# Define objectives
# J = ‖v₁ - vᵈᵉˢ‖² + ‖l₁ - lᵈᵉˢ‖² + 5 * a² + 20 * γ²
J1_platoon(x, u1, u2) = (x[2] - v_ref[1])^2 + 10 * (x[3] - l_ref[1])^2 + 5 * u1[1]^2 + 50 * u1[2]^2
J2_platoon(x, u1, u2) = (x[6] - v_ref[2])^2 + 10 * (x[7] - l_ref[1])^2 + 5 * u2[1]^2 + 50 * u2[2]^2
J_platoon = [J1_platoon, J2_platoon]

J1_overtake(x, u1, u2) = (x[2] - v_ref[1])^2 + 10 * (x[3] - l_ref[1])^2 + 5 * u1[1]^2 + 50 * u1[2]^2
J2_overtake(x, u1, u2) = (x[6] - v_ref[2])^2 + 10 * (x[7] - l_ref[2])^2 + 5 * u2[1]^2 + 50 * u2[2]^2
J_overtake = [J1_overtake, J2_overtake]

# State constraints
gx(x) = [
    1 - ( (x[1] - x[5])^2 / dx_min^2 + (x[3] - x[7])^2 / dl_min^2 ); # Safety distance: Ellipse
    v_min - x[2];  # min speed agent 1 
    x[2] - v_max;  # Max speed agent 1
    v_min - x[6];  # min speed agent 2
    x[6] - v_max;  # Max speed agent 2
    angle_min - x[4];  # min yaw agent 1
    x[4] - angle_max;  # Max yaw agent 1
    angle_min - x[8];  # min yaw agent 2
    x[8] - angle_max;  # Max yaw agent 2
]

gloc1(u1) = [
        a_min - u1[1];
        u1[1] - a_max;
        steer_min - u1[2];
        u1[2] - steer_max ] # Input constraints agent 1
gloc2(u2) = [
        a_min - u2[1];
        u2[1] - a_max;
        steer_min - u2[2];
        u2[2] - steer_max ] # Input constraints agent 2
gloc = [gloc1, gloc2]

gu(u1,u2) = -1.0 # Dummy

# Define games
game = Dict(
    :Platoon  => DyNECT.DynGame(f, J_platoon,  gx, gu, gloc, nx, nu, 9, 1, [4,4], N),
    :Overtake => DyNECT.DynGame(f, J_overtake, gx, gu, gloc, nx, nu, 9, 1, [4,4], N),
)


# useq = nominal trajectory 
# is a global so persists between calls
useq = [[zeros(nui) for nui in nu] for t in 1:T_hor]


# =================================================================================
# DEFINE THE SOLVER
# =================================================================================


function solve_step(xt, case_str)
    # JuliaCall passes Python lists as PyList(Any).
    # Convert to Vector{Float64} as expected by LQapprox.
    xt = convert(Vector{Float64}, xt)

    # Python passes the case, eg "Platoon" as string
    # Julia receives it as a Julia String
    # game dict uses Symbol keys, eg :Platoon
    # so we convert the string to symbol keys
    case = Symbol(case_str)

    # ----------------------------------------------------------
    # Linearise and solve NE to get control deviations δuseq
    # Based ons nonlinear.jl example in DyNECT repo
    # ----------------------------------------------------------
    lq_game = DyNECT.LQapprox(game[case], useq, xt, T_hor) # state/input are δx,δu: deviation from reference input/state
    mpavi = DyNECT.DynLQGame2mpAVI(lq_game)
    avi = DyNECT.AVI(mpavi, zeros(nx))
    sol = DAQPBase.avi(avi.H, avi.f, avi.A, avi.b, -Inf.*ones(length(avi.b)))
    exitflag = sol[3]
    if exitflag < 0
        println("infeasible!")
        # if infeasible, use last time steps controls instead
        ut = useq[1]
        # Shift control sequence
        useq[1:end-1] = useq[2:end]
        useq[end] = [zeros(nui) for nui in nu]
        return ut
    end
    exitflag >0 && println("OK")
    δuseq = DyNECT.arrange_vector_as_time_seq(sol[1], nu, N, T_hor)

    # Apply bias
    useq .= useq .+ δuseq 
    # Apply control
    ut = useq[1] 
    
    # Shift control sequence
    useq[1:end-1] = useq[2:end]
    useq[end] = [zeros(nui) for nui in nu] 

    return ut # [[a1, γ1], [a2, γ2]]
end

# warmup calls to solve_step so it gets precompiled and cached by juliacall.
# Uses PyList(Any) since that is the type juliacall actually passes xt as.
@compile_workload begin
    solve_step(PyList{Any}(pylist([0., 1., .5, .0, -4., 1., .5, .0])), "Platoon")
    solve_step(PyList{Any}(pylist([0., 1., .5, .0, -4., 1., .5, .0])), "Overtake")
    # deliberately infeasible (v1 far exceeds v_max) to precompile the exitflag<0 branch
    solve_step(PyList{Any}(pylist([0., 100., .5, .0, -4., 1., .5, .0])), "Platoon")
end

end