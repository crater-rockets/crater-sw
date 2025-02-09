from sympy.physics.mechanics import (
    ReferenceFrame,
    Vector,
    Inertia,
    dynamicsymbols,
    inertia,
    time_derivative,
)
from sympy import Symbol, sqrt


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


class SymRocket:
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
            v_norm = sqrt(v_norm)

        pdyn_v = 0.5 * self.rho * v_norm

        pdyn = pdyn_v * v_norm

        self.Fx = -pdyn * self.S * cA
        self.Fy = pdyn * self.S * cY + pdyn_v * self.S * self.d * self.ac.cY_r * self.r
        self.Fz = -pdyn * self.S * cN - pdyn_v * self.S * self.d * self.ac.cN_q * self.q

        self.Mx = (
            pdyn * self.S * cl * self.d
            + 0.5 * pdyn_v * self.S * self.d**2 * self.ac.cl_p * self.p
        )
        self.My = (
            pdyn * self.S * cm * self.d
            + 0.5 * pdyn_v * self.S * self.d**2 * self.ac.cm_q * self.q
        )
        self.Mz = (
            pdyn * self.S * cn * self.d
            + 0.5 * pdyn_v * self.S * self.d**2 * self.ac.cn_r * self.r
        )

    def eq_motion(self) -> tuple[Vector, Vector]:
        """Returns the equation of motion for linear and angular accelerations in body frame"""

        # All values in body frame unless otherwise specified

        # Angular rates vector
        w = self.p * self.B.x + self.q * self.B.y + self.r * self.B.z

        # Velocity vector
        v = self.u * self.B.x + self.v * self.B.y + self.w * self.B.z

        # Linear momentum
        P = self.m * v

        # Angular momentum
        I: Inertia = inertia(self.B, self.Ix, self.Iy, self.Iz)

        L = I.dot(w)

        # Torque
        M = self.Mx * self.B.x + self.My * self.B.y + self.Mz * self.B.z

        # Force
        F = self.Fx * self.B.x + self.Fy * self.B.y + self.Fz * self.B.z

        # Point of application of force wrt center of mass
        b = self.bx * self.B.x + self.by * self.B.y + self.by * self.B.y

        eq_vel = time_derivative(P, self.B) + w.cross(P) - F

        eq_rot = time_derivative(L, self.B) + w.cross(L) - M - b.cross(F)

        return (eq_vel.simplify(), eq_rot.simplify())
