import math
from pathlib import Path
from symrocket import SymRocket
from helpers import load_sim_params, pprint, linearize
from sympy import Matrix, atan, init_printing, Symbol, solve, lambdify
from sympy.physics.mechanics import dynamicsymbols, time_derivative
import numpy as np
import control as ct
import matplotlib.pyplot as plt
from rocket.pitch_dynamics import RocketPitchDynamics
from rocket.pitch_dynamics_with_u import RocketPitchDynamicsWithU

init_printing(use_unicode=True)


def load_params(path: Path) -> dict:
    # Load parameters from file for value substitution
    params = load_sim_params(path)

    # Extract only stuff we need in a flattened dictionary
    p_rocket = params["sim"]["rocket"]["crater"]
    p_aero = params["sim"]["rocket"]["crater"]["aero"]

    p_flat = {
        "m": p_rocket["mass"],
        "Iy": p_rocket["inertia"][1],
        "d": p_rocket["diameter"],
        "S": math.pi * (p_rocket["diameter"] / 2) ** 2,
    }
    p_flat.update(p_aero)

    return p_flat


params = load_params(Path("sim/config/params.toml"))
params.update(
    {
        "rho": 1.06,
        "u": 100,
        "delta_y": 0,
        "delta_r": 0,
        "delta_s": 0,
    }
)

rocket = RocketPitchDynamics()
rocket_with_u = RocketPitchDynamicsWithU()

pprint(rocket.rocket.eq_motion(), "rocket")
pprint(rocket_with_u.rocket.eq_motion(), "rocket_with_u")


# Substitute varibales and convert to numpy matrix
A, B = rocket.linearized_eq((0, 0))
A = np.array(A.subs(params)).astype(np.float64)
B = np.array(B.subs(params)).astype(np.float64)


ct_lin = ct.ss(
    A,
    B,
    np.array([0, 1]),
    np.array([0]),
    inputs=["dp_d"],
    outputs=["q"],
    dt=0,
    name="Simple",
)


A_with_u, B_with_u = rocket_with_u.linearized_eq((100, 0, 0))
A_with_u = np.array(A_with_u.subs(params)).astype(np.float64)
B_with_u = np.array(B_with_u.subs(params)).astype(np.float64)

ct_lin_with_u = ct.ss(
    A_with_u,
    B_with_u,
    np.array([0, 0, 1, 0]),
    np.array([0]),
    inputs=["dp_d"],
    outputs=["q"],
    dt=0,
)


derivator = ct.tf([1000, 0], [1, 1000], dt=0, inputs=["dp"], outputs=["dp_d"])
ct_lin_with_u = ct.interconnect(
    [derivator, ct.ss2tf(ct_lin_with_u)],
    inputs=["dp"],
    outputs=["q"],
    name="With U",
)


plt.figure()
# ct.step_response(ct_nlsys, T=np.linspace(0, 2, 10000)).plot()
ct.step_response(ct_lin, T=np.linspace(0, 2, 10000)).plot()
ct.step_response(ct_lin_with_u, T=np.linspace(0, 2, 10000)).plot()
plt.grid()


# plt.figure()
# ct.bode_plot(ct_lin_int, Hz=True, dB=True)


# plt.figure()
# ct.nyquist_plot(ct_lin_int)


# plt.figure()
# response = ct.pole_zero_map(ct_lin_int)
# ct.pole_zero_plot(response)
# plt.grid()


# servo = ct.tf([1], [1 / (100 * math.pi), 1])
# print(servo)
# plt.figure()
# ct.bode_plot(servo, Hz=True, dB=True)
plt.show()
