import math
from pathlib import Path
from symrocket import SymRocket
from helpers import linearize
from sympy import Matrix, atan, Symbol, solve
from sympy.physics.mechanics import dynamicsymbols, time_derivative
import numpy as np
import control as ct
import matplotlib.pyplot as plt


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

        self.rocket.delta_p = dynamicsymbols("delta_p")

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
