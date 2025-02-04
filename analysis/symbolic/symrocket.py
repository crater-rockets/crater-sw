from sympy.physics.mechanics import (
    ReferenceFrame,
    Vector,
    Inertia,
    dynamicsymbols,
    inertia,
    time_derivative,
)
from sympy import Symbol


class AeroCoefficients:
    cA_0 = Symbol("cA_0")
    cA_a = Symbol("cA_a")
    cA_b = Symbol("cA_b")
    cA_dy = Symbol("cA_dy")
    cA_dp = Symbol("cA_dp")
    cA_dr = Symbol("cA_dr")
    cA_ds = Symbol("cA_ds")
    cY_b = Symbol("cY_b")
    cY_r = Symbol("cY_r")
    cY_dy = Symbol("cY_dy")
    cN_a = Symbol("cN_a")
    cN_q = Symbol("cN_q")
    cN_dp = Symbol("cN_dp")
    cl_0 = Symbol("cl_0")
    cl_p = Symbol("cl_p")
    cl_dr = Symbol("cl_dr")
    cm_a = Symbol("cm_a")
    cm_q = Symbol("cm_q")
    cm_dp = Symbol("cm_dp")
    cn_b = Symbol("cn_b")
    cn_r = Symbol("cn_r")
    cn_dy = Symbol("cn_dy")


class SymRocketTerms:
    # Body frame
    B = ReferenceFrame("B")

    # Body frame velocities & accelerations
    u = dynamicsymbols("u")
    v = dynamicsymbols("v")
    w = dynamicsymbols("w")

    u_d = dynamicsymbols("u", 1)
    v_d = dynamicsymbols("v", 1)
    w_d = dynamicsymbols("w", 1)

    # Body frame angular rates & angular accelerations
    p = dynamicsymbols("p")
    q = dynamicsymbols("q")
    r = dynamicsymbols("r")

    p_d = dynamicsymbols("p_d")
    q_d = dynamicsymbols("q_d")
    r_d = dynamicsymbols("r_d")

    # Body frame orientation
    qx = dynamicsymbols("qx")
    qy = dynamicsymbols("qy")
    qz = dynamicsymbols("qz")
    qw = dynamicsymbols("qw")

    # Body frame forces
    Fx = dynamicsymbols("Fx")
    Fy = dynamicsymbols("Fy")
    Fz = dynamicsymbols("Fz")

    # Body frame torques
    Mx = dynamicsymbols("Mx")
    My = dynamicsymbols("My")
    Mz = dynamicsymbols("Mz")

    # mass & inertias
    m = dynamicsymbols("m")
    Ix = dynamicsymbols("Ix")
    Iy = dynamicsymbols("Iy")
    Iz = dynamicsymbols("Iz")

    # Body frame point of application of forces wrt center of mass
    bx = dynamicsymbols("bx")
    by = dynamicsymbols("by")
    bz = dynamicsymbols("bz")

    # Gravitational acceleration
    g = Symbol("g")

    # Forces and torques
    Fx = Symbol("Fx")
    Fy = Symbol("Fy")
    Fz = Symbol("Fz")
    Mx = Symbol("Mx")
    My = Symbol("My")
    Mz = Symbol("Mz")

    # Aerodynamic coefficients
    ac = AeroCoefficients()

    # Control angles
    delta_y = Symbol("delta_y")
    delta_p = Symbol("delta_p")
    delta_r = Symbol("delta_r")
    delta_s = Symbol("delta_s")

    # Atmospheric density
    rho = Symbol("rho")

    # Reference surface area
    S = Symbol("S")
    # Reference diameter
    d = Symbol("d")

    def use_linearized_forces(self, alpha, beta, v_norm):
        """Updates the definition of forces and torques to use linearized aero coefficients, instead of being plain symbols

        Args:
            alpha: Custom symbolic expression for alpha (required)
            beta: Custom symbolic expression for beta (required)
            vnorm: Custom symbolic expression for vnorm (u**2+v**2+w**2 if not specified)
        """
        cA = (
            self.ac.cA_0
            + self.ac.cA_a * alpha**2
            + self.ac.cA_b * beta**2
            + self.ac.cA_dy * self.delta_y**2
            + self.ac.cA_dp * self.delta_p**2
            + self.ac.cA_dr * self.delta_r**2
            + self.ac.cA_ds * self.delta_s**2
        )
        cY = self.ac.cY_b * beta + self.ac.cY_dy * self.delta_y
        cN = self.ac.cN_a * alpha + self.ac.cN_dp * self.delta_p
        cl = self.ac.cl_0 + self.ac.cl_dr * self.delta_r
        cm = self.ac.cm_a * alpha + self.ac.cm_dp * self.delta_p
        cn = self.ac.cn_b * beta + self.ac.cn_dy * self.delta_y

        if not v_norm:
            v = self.u * self.B.x + self.v * self.B.y + self.w * self.B.z
            v_norm = v.dot(v)

        q_v = 0.5 * self.rho * v_norm
        q = 0.5 * self.rho * (v_norm**2)

        self.Fx = -q * self.S * cA
        self.Fy = q * self.S * cY + q_v * self.S * self.d * self.ac.cY_r * self.r
        self.Fz = -q * self.S * cN - q_v * self.S * self.d * self.ac.cN_q * self.q

        self.Mx = (
            q * self.S * cl * self.d
            + 0.5 * q_v * self.S * self.d**2 * self.ac.cl_p * self.p
        )
        self.My = (
            q * self.S * cm * self.d
            + 0.5 * q_v * self.S * self.d**2 * self.ac.cm_q * self.q
        )
        self.Mz = (
            q * self.S * cn * self.d
            + 0.5 * q_v * self.S * self.d**2 * self.ac.cn_r * self.r
        )


def rocket_accelerations(terms: SymRocketTerms) -> tuple[Vector, Vector]:
    # All values in body frame unless otherwise specified

    # Angular rates vector
    w = terms.p * terms.B.x + terms.q * terms.B.y + terms.r * terms.B.z

    # Velocity vector
    v = terms.u * terms.B.x + terms.v * terms.B.y + terms.w * terms.B.z

    # Linear momentum
    P = terms.m * v

    # Angular momentum
    I: Inertia = inertia(terms.B, terms.Ix, terms.Iy, terms.Iz)

    L = I.dot(w)

    # Torque
    M = terms.Mx * terms.B.x + terms.My * terms.B.y + terms.Mz * terms.B.z

    # Force
    F = terms.Fx * terms.B.x + terms.Fy * terms.B.y + terms.Fz * terms.B.z

    # Point of application of force wrt center of mass
    b = terms.bx * terms.B.x + terms.by * terms.B.y + terms.by * terms.B.y

    eq_vel = time_derivative(P, terms.B) + w.cross(P) - F
    eq_rot = time_derivative(L, terms.B) + w.cross(L) - M - b.cross(F)

    return (eq_vel.simplify(), eq_rot.simplify())
