"""Plant model: Clohessy-Wiltshire equations for spacecraft rendezvous.

The chaser spacecraft state is expressed in the target's Local-Vertical /
Local-Horizontal (LVLH) frame, with the target spacecraft at the origin
on a circular reference orbit.

Convention (Clohessy & Wiltshire 1960; Curtis 2014, Sec. 7.4):
    x  --  radial direction, positive outward from Earth
    y  --  along-track direction, positive along the target's velocity
    z  --  cross-track (out-of-plane); decoupled, omitted here

For circular target orbits the linearised relative dynamics decouple into
an in-plane 4-state block in (x, y) and an out-of-plane 2-state block in
(z); only the in-plane block carries the unstable secular drift and the
coupled oscillation that make MPC interesting, so that is the block we
model.

State vector  s = [x, y, x_dot, y_dot]^T  in [m, m, m/s, m/s].
Control input u = [u_x, u_y]^T in newtons (per-axis thrust).
"""

import numpy as np
from scipy.linalg import expm


class ClohessyWiltshire:
    """LTI relative-orbital-motion model with exact ZOH discretisation."""

    def __init__(self, params):
        self.n = float(params.n)        # mean orbital rate [rad/s]
        self.m = float(params.m)        # chaser mass [kg]
        self.Ts = float(params.Ts)      # sampling period [s]

        # Continuous-time state matrix.  Rows correspond to derivatives
        # of [x, y, x_dot, y_dot].  Off-diagonal Coriolis couplings come
        # from writing the relative-motion equations in the rotating
        # LVLH frame and dropping terms above first order in the relative
        # state.
        n = self.n
        self.A = np.array([
            [0.0,      0.0,   1.0,   0.0],
            [0.0,      0.0,   0.0,   1.0],
            [3 * n * n, 0.0,   0.0,   2 * n],
            [0.0,      0.0,  -2 * n, 0.0],
        ])
        self.B = np.array([
            [0.0,        0.0       ],
            [0.0,        0.0       ],
            [1.0 / self.m, 0.0       ],
            [0.0,        1.0 / self.m],
        ])

        # Exact ZOH discretisation via the augmented-matrix trick:
        # exp([[A, B]; [0, 0]] * Ts) = [[A_d, B_d]; [0, I]]
        nx, nu = self.A.shape[0], self.B.shape[1]
        M = np.zeros((nx + nu, nx + nu))
        M[:nx, :nx] = self.A
        M[:nx, nx:] = self.B
        eM = expm(M * self.Ts)
        self.A_d = eM[:nx, :nx]
        self.B_d = eM[:nx, nx:]

    # ---- forward simulation ------------------------------------------------

    def dynamics(self, s, u):
        """Continuous-time state derivative s_dot = A s + B u."""
        return self.A @ np.asarray(s) + self.B @ np.asarray(u)

    def step(self, s, u):
        """One-step zero-order-hold update: s_{k+1} = A_d s_k + B_d u_k."""
        return self.A_d @ np.asarray(s) + self.B_d @ np.asarray(u)

    # ---- analytical free-drift (for validation) ---------------------------

    def free_drift(self, s0, t):
        """Closed-form CW solution for u = 0 at absolute time `t`.

        Useful as a ground truth against repeated `step` applications --
        agreement to many digits confirms the discretisation is correct.

        Reference: Curtis (2014) eq. (7.53)-(7.56).
        """
        n = self.n
        x0, y0, vx0, vy0 = float(s0[0]), float(s0[1]), float(s0[2]), float(s0[3])
        nt = n * t
        c, s = np.cos(nt), np.sin(nt)
        x = (4.0 - 3.0 * c) * x0 + (s / n) * vx0 + (2.0 * (1.0 - c) / n) * vy0
        y = (6.0 * (s - nt)) * x0 + y0 + (2.0 * (c - 1.0) / n) * vx0 + \
            ((4.0 * s - 3.0 * nt) / n) * vy0
        vx = (3.0 * n * s) * x0 + c * vx0 + (2.0 * s) * vy0
        vy = (6.0 * n * (c - 1.0)) * x0 + (-2.0 * s) * vx0 + (4.0 * c - 3.0) * vy0
        return np.array([x, y, vx, vy])

    # ---- spectra (for sanity checks and reporting) ------------------------

    def eigenvalues_continuous(self):
        """Spectrum of A.  Expected: {0, 0, +i n, -i n}."""
        return np.linalg.eigvals(self.A)

    def eigenvalues_discrete(self):
        """Spectrum of A_d.  Expected: on/near the unit circle (|.|=1)."""
        return np.linalg.eigvals(self.A_d)
