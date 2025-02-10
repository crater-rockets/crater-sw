import math
from pathlib import Path
from symrocket import SymRocket
from helpers import load_sim_params, pprint, linearize
from sympy import Matrix, atan, init_printing, Symbol, solve, lambdify
from sympy.physics.mechanics import dynamicsymbols, time_derivative
import numpy as np
import control as ct
import matplotlib.pyplot as plt

init_printing(use_unicode=True)


class RocketPitchDynamics:
    rocket: SymRocket

    def __init__(self, rocket: SymRocket | None = None):
        if rocket is None:
            self.rocket = SymRocket()
        else:
            self.rocket = rocket

        # ** Assumptions **
        # Constant mass and inertia
        self.rocket.m = Symbol("m")
        self.rocket.Ix = Symbol("Ix")
        self.rocket.Iy = Symbol("Iy")
        self.rocket.Iz = Symbol("Iz")
        # Constant u
        self.rocket.u = Symbol("u")
        # v = 0
        self.rocket.v = 0
        # null p, r angular rates
        self.rocket.p = 0
        self.rocket.r = 0
        # null lever arm of force from center of mass
        self.rocket.bx = 0
        self.rocket.by = 0
        self.rocket.bz = 0
        # beta is null, norm of velocity is dominated by "u" term, others are negligible
        self.rocket.use_linearized_forces(
            alpha=dynamicsymbols("alpha"), beta=0, v_norm=self.rocket.u
        )

    def eq_motion(self, solve_for_acc: bool) -> Matrix:
        lin_acc, ang_acc = self.rocket.eq_motion()

        # Just on body x-z plane, also it was assumed that u is constant, so d/dt(u) = 0
        lin_acc = lin_acc.dot(self.rocket.B.z)
        ang_acc = ang_acc.dot(self.rocket.B.y)

        # alpha symbol
        alpha = dynamicsymbols("alpha")

        # alpha = atan(w/u) --> atan(w/u) - alpha = 0
        eq_alpha = atan(self.rocket.w / self.rocket.u) - alpha

        # solve for w in terms of alpha
        w = solve(eq_alpha, self.rocket.w)[0].simplify()
        w_d = time_derivative(w, self.rocket.B).simplify()

        # Replace w * w_d in equations of motion
        lin_acc = lin_acc.subs(self.rocket.w_d, w_d)
        lin_acc = lin_acc.subs(self.rocket.w, w)

        ang_acc = ang_acc.subs(self.rocket.w, w)

        if not solve_for_acc:
            return Matrix([lin_acc, ang_acc])
        else:
            alpha_d = solve(lin_acc, dynamicsymbols("alpha", 1))[0].simplify()
            q_d = solve(ang_acc, dynamicsymbols("q", 1))[0].simplify()

            return Matrix([alpha_d, q_d])

    def linearized_eq(self, init_cond: tuple) -> tuple[Matrix, Matrix]:
        nonlin_sys = self.eq_motion(solve_for_acc=False)

        # Linearize equation of motion at (0, 0)
        vars = (dynamicsymbols("alpha"), dynamicsymbols("q"))
        vars_d = (dynamicsymbols("alpha", 1), dynamicsymbols("q", 1))
        vars_0 = init_cond
        vars_input = (self.rocket.delta_p,)

        # Usual linear system notation: x_d = A * x + B * u
        A, B = linearize(nonlin_sys, vars, vars_d, vars_input, vars_0)

        return A, B


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


servo = ct.tf([1], [1 / (100 * math.pi), 1])
print(servo)
plt.figure()
ct.bode_plot(servo, Hz=True, dB=True)
plt.show()
