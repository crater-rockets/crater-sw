import math
from pathlib import Path
from symrocket import SymRocket
from helpers import linearize
from sympy import Matrix, atan, Symbol, solve
from sympy.physics.mechanics import dynamicsymbols


class RocketPitchDynamicsWithU:
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
        # self.rocket.u = Symbol("u")
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
        acc = Matrix(
            [
                lin_acc.dot(self.rocket.B.x),
                lin_acc.dot(self.rocket.B.z),
                ang_acc.dot(self.rocket.B.y),
            ]
        )

        # alpha symbol
        alpha = dynamicsymbols("alpha")

        # alpha = atan(w/u)
        eq_alpha = atan(self.rocket.w / self.rocket.u)

        acc = acc.subs(alpha, eq_alpha)

        if not solve_for_acc:
            return acc
        else:
            u_d = solve(acc[0], dynamicsymbols("u", 1))[0].simplify()
            w_d = solve(acc[1], dynamicsymbols("w", 1))[0].simplify()
            q_d = solve(acc[2], dynamicsymbols("q", 1))[0].simplify()

            return Matrix([u_d, w_d, q_d])

    def linearized_eq(self, init_cond: tuple) -> tuple[Matrix, Matrix]:
        nonlin_sys = self.eq_motion(solve_for_acc=False)
        nonlin_sys = nonlin_sys.row_insert(
            3, Matrix([dynamicsymbols("delta_p", 1) - dynamicsymbols("u_delta_p_d")])
        )

        # Linearize equation of motion at (0, 0)
        vars = (
            dynamicsymbols("u"),
            dynamicsymbols("w"),
            dynamicsymbols("q"),
            dynamicsymbols("delta_p"),
        )
        vars_d = (
            dynamicsymbols("u", 1),
            dynamicsymbols("w", 1),
            dynamicsymbols("q", 1),
            dynamicsymbols("delta_p", 1),
        )
        vars_0 = init_cond + (0,)
        vars_input = (dynamicsymbols("u_delta_p_d"),)

        # Usual linear system notation: x_d = A * x + B * u
        A, B = linearize(nonlin_sys, vars, vars_d, vars_input, vars_0)

        return A, B
