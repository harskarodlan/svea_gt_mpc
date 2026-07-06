using DyNECT
using DAQPBase
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
        jl.seval('include("overtake_solver.jl")')

overtake_solver.jl:
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
        ut = jl.solve_step(xt, mode)
    - extracts u1, u2 from ut
    - return u1, u2 to overtake.py
"""


# =================================================================================
# DEFINE GAME, copied from nonlinear.jl in DyNECT repo example
# =================================================================================

# Parameters
#T_sim = 200 # Simulation length
T_hor = 10
Δt = 0.1 #sampling time
v_ref = [5., 7.] # reference speed for each agent in carlength/s. Car length is 3 m (defined in the plot script)
v_min = 0.2 # 3.
v_max = 1.7 # 10.
#d_overtake = 3. # Distance at which overtake is initiated, in car length-unit
a_max = 1. # max acceleration
a_min = -1.
angle_max = pi / 32
angle_min = -pi / 32
l_ref = [0.5, -0.5] #reference lateral position for normal and overtake lane 

dx_min = 2. # safety longitudinal distance, in car length-unit
dl_min = 0.5 # safety lateral distance, 1=lane width

# Fixed values
N = 2
nu = [2,2]
nx = 6

## Initialization

# State:
#x0 = [0., (v_max + v_min) / 2, l_ref[1], -5., (v_max + v_min) / 2, l_ref[1]]
#u0=[zeros(2), zeros(2)]

# Dynamics
function unicycle_dynamics(x,u)
    return [
        x[2] * cos(u[2]),
        u[1],
        x[2] * sin(u[2])
    ]
end
function f(x, u1, u2) # discretized unicycles
    dx = vcat(unicycle_dynamics(x[1:3],u1), unicycle_dynamics(x[4:6],u2))
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
J1_platoon(x, u1, u2) = (x[2] - v_ref[1])^2 + (x[3] - l_ref[1])^2 + 5 * u1[1]^2 + 50 * u1[2]^2
J2_platoon(x, u1, u2) = (x[5] - v_ref[2])^2 + (x[6] - l_ref[1])^2 + 5 * u2[1]^2 + 50 * u2[2]^2
J_platoon = [J1_platoon, J2_platoon]

J1_overtake(x, u1, u2) = (x[2] - v_ref[1])^2 + (x[3] - l_ref[1])^2 + 5 * u1[1]^2 + 50 * u1[2]^2
J2_overtake(x, u1, u2) = (x[5] - v_ref[2])^2 + (x[6] - l_ref[2])^2 + 5 * u2[1]^2 + 50 * u2[2]^2
J_overtake = [J1_overtake, J2_overtake]

# State constraints
gx(x) = [
    1 - ( (x[1] - x[4])^2 / dx_min^2 + (x[3] - x[6])^2 / dl_min^2 ); # Safety distance: Ellipse
    v_min - x[2];  # min speed agent 1 
    x[2] - v_max;  # Max speed agent 1
    v_min - x[5];  # min speed agent 2
    x[5] - v_max;  # Max speed agent 2
]

gloc1(u1) = [
        a_min - u1[1];
        u1[1] - a_max;
        angle_min - u1[2];
        u1[2] - angle_max ] # Input constraints agent 1
gloc2(u2) = [
        a_min - u2[1];
        u2[1] - a_max;
        angle_min - u2[2];
        u2[2] - angle_max ] # Input constraints agent 2
gloc = [gloc1, gloc2]

gu(u1,u2) = -1.0 # Dummy

# Define games
game = Dict(
    :Platoon  => DyNECT.DynGame(f, J_platoon,  gx, gu, gloc, nx, nu, 5, 1, [4,4], N),
    :Overtake => DyNECT.DynGame(f, J_overtake, gx, gu, gloc, nx, nu, 5, 1, [4,4], N),
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
    # Copied from nonlinear.jl example in DyNECT repo
    # ----------------------------------------------------------
    lq_game = DyNECT.LQapprox(game[case], useq, xt, T_hor) # state/input are δx,δu: deviation from reference input/state
    mpavi = DyNECT.DynLQGame2mpAVI(lq_game)
    avi = DyNECT.AVI(mpavi, zeros(nx))
    sol = DAQPBase.avi(avi.H, avi.f, avi.A, avi.b, -Inf.*ones(length(avi.b)))
    exitflag = sol[3]
    if exitflag < 0
        println("infeasible!")
        return [[0.0, 0.0], [0.0, 0.0]]
    end
    exitflag >0 && println("OK")
    δuseq = DyNECT.arrange_vector_as_time_seq(sol[1], nu, N, T_hor)

    # Apply bias
    useq .= useq .+ δuseq 
    # Apply control
    ut = useq[1] + δuseq[1]    
    # ??Should apply bias and control lines be switched??
    
    # Shift control sequence
    useq[1:end-1] = useq[2:end]
    useq[end] = [zeros(nui) for nui in nu] 

    return ut # [[a1, γ1], [a2, γ2]]
end
