import numpy as np


class Acrobot:
    """Two-link planar robot with unactuated shoulder, actuated elbow,
    and (optionally) viscous friction at the elbow joint.

    Angle convention (Xin & Kaneda, 2007):
        q1 — angle of link 1 from horizontal, counter-clockwise positive.
        q2 — relative angle of link 2 with respect to link 1.
        Upright equilibrium:  q1 = pi/2,  q2 = 0.
        Hanging equilibrium:  q1 = -pi/2, q2 = 0.

    Equations of motion with elbow viscous friction b2 >= 0:
        M(q) ddq + C(q, dq) dq + G(q) + [0; b2 * dq2] = [0; tau_2].

    Setting ``b2 = 0`` recovers the frictionless plant of Project 1.
    """

    def __init__(self, params, b2=0.0):
        self.m1 = params.m1
        self.m2 = params.m2
        self.l1 = params.l1
        self.l2 = params.l2
        self.lc1 = params.lc1
        self.lc2 = params.lc2
        self.I1 = params.I1
        self.I2 = params.I2
        self.g = params.g
        self.b2 = float(b2)

        # Lumped inertia parameters (Xin & Kaneda eq. 5)
        self.alpha1 = params.m1 * params.lc1**2 + params.m2 * params.l1**2 + params.I1
        self.alpha2 = params.m2 * params.lc2**2 + params.I2
        self.alpha3 = params.m2 * params.l1 * params.lc2

        # Lumped gravity parameters
        self.beta1 = (params.m1 * params.lc1 + params.m2 * params.l1) * params.g
        self.beta2 = params.m2 * params.lc2 * params.g

        # Total energy at the upright equilibrium (Xin & Kaneda eq. 12)
        self.E_upright = self.beta1 + self.beta2

    def mass_matrix(self, q2):
        """Inertia matrix M(q2), 2x2 symmetric positive-definite."""
        c2 = np.cos(q2)
        M11 = self.alpha1 + self.alpha2 + 2.0 * self.alpha3 * c2
        M12 = self.alpha2 + self.alpha3 * c2
        M22 = self.alpha2
        return np.array([[M11, M12], [M12, M22]])

    def coriolis(self, q2, dq1, dq2):
        """Coriolis and centrifugal terms C(q, dq) dq."""
        s2 = np.sin(q2)
        H1 = -self.alpha3 * (2.0 * dq1 * dq2 + dq2**2) * s2
        H2 = self.alpha3 * dq1**2 * s2
        return np.array([H1, H2])

    def gravity(self, q1, q2):
        """Gravitational torque vector G(q)."""
        G1 = self.beta1 * np.cos(q1) + self.beta2 * np.cos(q1 + q2)
        G2 = self.beta2 * np.cos(q1 + q2)
        return np.array([G1, G2])

    def friction(self, dq2):
        """Plant-side viscous friction vector F(dq) = [0, b2 * dq2]."""
        return np.array([0.0, self.b2 * dq2])

    def potential_energy(self, q1, q2):
        """Potential energy P(q)."""
        return self.beta1 * np.sin(q1) + self.beta2 * np.sin(q1 + q2)

    def total_energy(self, q1, q2, dq1, dq2):
        """Total mechanical energy E = T + P (excludes friction work)."""
        dq = np.array([dq1, dq2])
        M = self.mass_matrix(q2)
        return 0.5 * dq @ M @ dq + self.potential_energy(q1, q2)

    def dynamics(self, state, u):
        """State derivative [dq1, dq2, ddq1, ddq2] given state and scalar control u.

        The plant friction term is INCLUDED here; the controller does not
        see ``self.b2`` directly — that's the whole point of the adaptive
        problem.
        """
        q1, q2, dq1, dq2 = state
        tau = np.array([0.0, u])
        M = self.mass_matrix(q2)
        C = self.coriolis(q2, dq1, dq2)
        G = self.gravity(q1, q2)
        F = self.friction(dq2)
        ddq = np.linalg.solve(M, tau - C - G - F)
        return np.array([dq1, dq2, ddq[0], ddq[1]])

    def linearize_at_upright(self, b2_for_lqr=0.0):
        """Linearize about the upright equilibrium (q1=pi/2, q2=0, dq=0).

        ``b2_for_lqr`` lets the LQR see a friction estimate when one is
        available (it shows up as extra damping in the (dq2 -> ddq2) row).
        Pass 0.0 to recover the frictionless linearization used in P1.

        Returns (A, B) such that  dx/dt = A x + B u,
        where x = [q1 - pi/2, q2, dq1, dq2].
        """
        M_eq = self.mass_matrix(0.0)
        M_inv = np.linalg.inv(M_eq)

        G_jac = np.array([
            [-(self.beta1 + self.beta2), -self.beta2],
            [-self.beta2, -self.beta2],
        ])

        F_jac = np.array([
            [0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, b2_for_lqr],
        ])  # dF/d[q1,q2,dq1,dq2] at equilibrium

        A = np.zeros((4, 4))
        A[0, 2] = 1.0
        A[1, 3] = 1.0
        A[2:, :2] = -M_inv @ G_jac
        A[2:, 2:] = -M_inv @ F_jac[:, 2:]

        B = np.zeros((4, 1))
        B[2:, 0] = M_inv @ np.array([0.0, 1.0])

        return A, B

    def solvability_bound(self, n_samples=20000):
        """Minimum kD for a well-defined energy-based control law.

        For the denominator  kD*M11 + (E-Er)*Delta  to be strictly positive
        for all states we need
            kD > max_{q2} [ (P_extreme(q2) + Er) * Delta(q2) / M11(q2) ],
        where P_extreme = sqrt(beta1^2 + beta2^2 + 2 beta1 beta2 cos q2).
        """
        q2_vals = np.linspace(0.0, 2.0 * np.pi, n_samples)
        f_max = -np.inf
        for q2 in q2_vals:
            M = self.mass_matrix(q2)
            M11 = M[0, 0]
            Delta = M[0, 0] * M[1, 1] - M[0, 1] * M[1, 0]
            P_extreme = np.sqrt(
                self.beta1**2
                + self.beta2**2
                + 2.0 * self.beta1 * self.beta2 * np.cos(q2)
            )
            f = (P_extreme + self.E_upright) * Delta / M11
            if f > f_max:
                f_max = f
        return f_max
