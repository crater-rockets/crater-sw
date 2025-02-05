from symrocket import SymRocketTerms, rocket_accelerations
from helpers import pprint
from sympy import atan, init_printing, Symbol, solve
from sympy.physics.mechanics import dynamicsymbols, time_derivative


init_printing(use_unicode=True)


rocket = SymRocketTerms()

# ** Assumptions **
# Constant mass and inertia
rocket.m = Symbol("m")
rocket.Ix = Symbol("Ix")
rocket.Iy = Symbol("Iy")
rocket.Iz = Symbol("Iz")
# Constant u
rocket.u = Symbol("u")
# v = 0
rocket.v = 0
# null p, r angular rates
rocket.p = 0
rocket.r = 0
# null lever arm of force from center of mass
rocket.bx = 0
rocket.by = 0
rocket.bz = 0
# beta is null, norm of velocity is dominated by "u" term, others are negligible
rocket.use_linearized_forces(alpha=dynamicsymbols("a"), beta=0, v_norm=rocket.u**2)

# Equations of motion
lin_acc, ang_acc = rocket_accelerations(rocket)
pprint(lin_acc, "lin_acc")

# Just on body x-z plane, also it was assumed that u is constant, so d/dt(u) = 0
lin_acc = lin_acc.dot(rocket.B.z)
ang_acc = ang_acc.dot(rocket.B.y)

# alpha symbol
alpha = dynamicsymbols("a")

# alpha = atan(w/u) --> atan(w/u) - alpha = 0
eq_alpha = atan(rocket.w / rocket.u) - alpha

# solve for w in terms of alpha
w = solve(eq_alpha, rocket.w)[0].simplify()
w_d = time_derivative(w, rocket.B).simplify()

# Replace w * w_d in equations of motion
lin_acc = lin_acc.subs(rocket.w_d, w_d)
lin_acc = lin_acc.subs(rocket.w, w)

ang_acc = ang_acc.subs(rocket.w, w)


pprint(lin_acc, "lin_acc")
pprint(ang_acc, "ang_acc")

# Equation of motion in terms of alpha_d, q_d
alpha_d = solve(lin_acc, dynamicsymbols("a", 1))[0].simplify()
q_d = solve(ang_acc, dynamicsymbols("q", 1))[0].simplify()

pprint(alpha_d, "alpha_d")
pprint(q_d, "q_d")
