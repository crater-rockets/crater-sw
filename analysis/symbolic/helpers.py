from sympy import pretty_print
from sympy.solvers.solvers import solve
from sympy.physics.mechanics import (
    ReferenceFrame,
    Vector,
)

# init_printing(use_unicode=True)


def pprint(expr, name: str | None = None):
    if name:
        print(f"{name} =")

    pretty_print(expr, use_unicode=True)
    print("")


def solve_vec(rf: ReferenceFrame, vec: Vector, terms: Vector) -> Vector:
    terms = (terms.dot(rf.x), terms.dot(rf.y), terms.dot(rf.z))
    solved = solve([vec.dot(rf.x), vec.dot(rf.y), vec.dot(rf.z)], terms)

    return solved[terms[0]] * rf.x + solved[terms[1]] * rf.y + solved[terms[2]] * rf.z
