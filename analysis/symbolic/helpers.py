from pathlib import Path
import tomllib
from sympy import pretty_print
from sympy.solvers.solvers import solve
from sympy.physics.mechanics import (
    ReferenceFrame,
    Vector,
)
from sympy import Matrix


def pprint(expr, name: str | None = None):
    if name:
        print(f"{name} =")

    pretty_print(expr, use_unicode=True)
    print("")


def solve_vec(rf: ReferenceFrame, vec: Vector, terms: Vector) -> Vector:
    terms = (terms.dot(rf.x), terms.dot(rf.y), terms.dot(rf.z))
    solved = solve([vec.dot(rf.x), vec.dot(rf.y), vec.dot(rf.z)], terms)

    return solved[terms[0]] * rf.x + solved[terms[1]] * rf.y + solved[terms[2]] * rf.z


def linearize(
    sys: Matrix,
    q: tuple,
    qd: tuple,
    u: tuple,
    q0: tuple,
) -> tuple[Matrix, Matrix]:
    jq = sys.jacobian(Matrix(list(q)))
    jqd = sys.jacobian(Matrix(list(qd)))
    ju = sys.jacobian(Matrix(list(u)))

    for i, q_val in enumerate(q0):
        jq = jq.subs(q[i], q_val)
        jqd = jqd.subs(q[i], q_val)
        ju = ju.subs(q[i], q_val)

    # pprint(jq, "jq")
    # pprint(jqd, "jqd")
    # pprint(ju, "ju")

    jqd_inv = jqd.inv()
    return jqd_inv * -jq, jqd_inv * -ju


def load_sim_params(file: Path) -> dict:
    def to_plain_dict(params: dict) -> dict:
        out = {}
        for k, v in params.items():
            if "val" in v and "dtype" in v:
                out[k] = v["val"]
            else:
                out[k] = to_plain_dict(v)

        return out
    
    with open(file, "rb") as f:
        params = tomllib.load(f)
    
    return to_plain_dict(params)