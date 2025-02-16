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
params.update({"u": 100, "rho": 1.06})

rocket = RocketPitchDynamics()

pprint(rocket.rocket.eq_motion(), "eq_rocket")


## Nonlinear step response
eq_ode = rocket.eq_motion(True)
pprint(eq_ode, "eq_ode")

eq_ode: Matrix = eq_ode.subs(params)
pprint(eq_ode, "eq_ode")


eq_ode_lambda = lambdify(
    (dynamicsymbols("alpha"), dynamicsymbols("q"), Symbol("delta_p")), eq_ode, "numpy"
)


def ode_fn(t, x, u, params):
    return eq_ode_lambda(x[0], x[1], np.deg2rad(u))


ct_nlsys = ct.nlsys(
    ode_fn,
    lambda t, x, y, params: np.rad2deg(x[1]),
    states=2,
    name="NL",
    inputs=["dp"],
    outputs=["alpha"],
)

pprint(rocket.rocket.Mx, "Mx")
pprint(rocket.rocket.My, "My")
pprint(rocket.rocket.Mz, "Mz")


# Substitute varibales and convert to numpy matrix
A, B = rocket.linearized_eq((0, 0))
A = np.array(A.subs(params)).astype(np.float64)
B = np.array(B.subs(params)).astype(np.float64)

print("")
print(f"A = {A}")
print(f"B = {B}")

ct_lin = ct.ss(
    A,
    B,
    np.array([0, 1]),
    np.array([0]),
    inputs=["dp"],
    outputs=["alpha"],
    dt=0,
    name="Linear",
)


# print(f"poles = {ct_sys.poles()}")
# print(f"zeros = {ct_sys.zeros()}")

tf = ct.ss2tf(ct_lin)
print(tf)

plt.figure()
ct.step_response(ct_nlsys, T=np.linspace(0, 2, 10000)).plot()
ct.step_response(ct_lin, T=np.linspace(0, 2, 10000)).plot()
plt.grid()

plt.figure()
ct.bode_plot(ct_lin, Hz=True, dB=True)


plt.figure()
ct.nyquist_plot(ct_lin)


plt.figure()
response = ct.pole_zero_map(ct_lin)
ct.pole_zero_plot(response)
plt.grid()


plt.show()
