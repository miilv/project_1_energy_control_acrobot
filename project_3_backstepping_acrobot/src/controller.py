import numpy as np
from scipy.linalg import solve_continuous_are


def _energy_virtual_torque(acrobot, q1, q2, dq1, dq2, kD, kP, kV):
    """Project-1 energy-shaping torque, used here as the backstepping virtual control.

    Drives the Lyapunov function  V0 = (1/2)(E - Er)^2 + (1/2)kD*dq2^2 + (1/2)kP*q2^2
    along  V0_dot = -kV*dq2^2  when applied as the actual joint-2 torque.
    """
    E_err = acrobot.total_energy(q1, q2, dq1, dq2) - acrobot.E_upright
    M = acrobot.mass_matrix(q2)
    M11, M12, M21, M22 = M[0, 0], M[0, 1], M[1, 0], M[1, 1]
    Delta = M11 * M22 - M12 * M21
    C = acrobot.coriolis(q2, dq1, dq2)
    G = acrobot.gravity(q1, q2)

    numerator = (kV * dq2 + kP * q2) * Delta + kD * (
        M21 * (C[0] + G[0]) - M11 * (C[1] + G[1])
    )
    denominator = kD * M11 + E_err * Delta
    return -numerator / denominator


class BackstepSwingUpController:
    """Backstepping swing-up controller for the acrobot with actuator dynamics.

    Treats the actual joint-2 torque tau2 as a state and uses the Project-1
    energy-shaping torque tau2* as the virtual control.  The actuator-tracking
    error  z = tau2 - tau2*  is killed by a feedforward-plus-proportional
    inner-loop command:

        u = tau2* + Ta * dtau2*_chain  -  kz * Ta * z .

    The first two terms are the feedforward needed to anticipate the actuator
    lag (a first-order Taylor estimate of tau2*(t + Ta)).  The third term is
    proportional feedback on the inner tracking error.  Substituting into the
    actuator equation Ta * dtau2 = u - tau2 gives

        dz = -((1 + kz) / Ta) * z + dtau2*_chain * 0 ... wait
        dz = dtau2 - dtau2*_chain
            = (u - tau2)/Ta - dtau2*_chain
            = (tau2* - tau2 + Ta*dtau2*_chain - kz*Ta*z)/Ta - dtau2*_chain
            = -z/Ta - kz*z
            = -(1/Ta + kz) z ,

    a pure exponential decay of the actuator-tracking error with time constant
    Ta / (1 + kz*Ta).  With z driven fast to zero, the outer energy controller
    inherits the Project-1 closed-loop behaviour:  V0_dot -> -kV * dq2^2 once
    the inner loop has settled (a few inner time constants).

    The chain-rule derivative dtau2*_chain is computed by numerical state
    gradient to keep the implementation honest:

        dtau2*_chain = sum_k (d tau2* / d x_k) * (dx_k / dt) ,  k = 1..4
        (tau2* depends only on the mechanical state, not on tau2 itself.)
    """

    def __init__(self, acrobot, kD, kP, kV, kz, u_max, h_grad=1e-5):
        self.acrobot = acrobot
        self.kD = kD
        self.kP = kP
        self.kV = kV
        self.kz = kz
        self.u_max = u_max
        self.h_grad = h_grad  # finite-difference step for d tau2* / d x

    def virtual_torque(self, state5):
        q1, q2, dq1, dq2, _ = state5
        return _energy_virtual_torque(
            self.acrobot, q1, q2, dq1, dq2, self.kD, self.kP, self.kV
        )

    def _grad_virtual_torque(self, state5):
        """Numerical gradient of tau2* with respect to (q1, q2, dq1, dq2)."""
        h = self.h_grad
        grad = np.zeros(4)
        x = np.asarray(state5, dtype=float)
        for k in range(4):
            x_plus = x.copy()
            x_minus = x.copy()
            x_plus[k] += h
            x_minus[k] -= h
            tp = _energy_virtual_torque(
                self.acrobot, x_plus[0], x_plus[1], x_plus[2], x_plus[3],
                self.kD, self.kP, self.kV,
            )
            tm = _energy_virtual_torque(
                self.acrobot, x_minus[0], x_minus[1], x_minus[2], x_minus[3],
                self.kD, self.kP, self.kV,
            )
            grad[k] = (tp - tm) / (2.0 * h)
        return grad

    def compute(self, state5):
        q1, q2, dq1, dq2, tau2 = state5
        ac = self.acrobot

        tau2_star = self.virtual_torque(state5)
        z = tau2 - tau2_star

        # dtau2*_chain via dot product of state gradient with mechanical state
        # derivatives evaluated at the *actual* current tau2 (not tau2*).
        ddq = ac.mechanical_accel(state5)
        xdot4 = np.array([dq1, dq2, ddq[0], ddq[1]])
        grad = self._grad_virtual_torque(state5)
        dtau2_star = float(grad @ xdot4)

        u = tau2_star + ac.Ta * dtau2_star - self.kz * ac.Ta * z
        return np.clip(u, -self.u_max, self.u_max)


class LQR5DController:
    """Linear-quadratic regulator for the 5D plant near the upright equilibrium.

    Uses Project-1's 4D LQR for the mechanical state and adds a small extra
    proportional feedback on the actuator state tau2 to damp any residual lag.
    The full-5D Riccati solution turned out to demand high gains that saturate
    against u_max and destabilise the closed loop on entry; the P1 LQR gains
    are conservative, applied through the first-order actuator, and reliably
    catch the swing-up exit.
    """

    def __init__(self, acrobot, Q4, R, k_tau=0.0):
        # 4D linearisation of the acrobot at upright (ignores the actuator state)
        M_eq = acrobot.mass_matrix(0.0)
        M_inv = np.linalg.inv(M_eq)
        G_jac = np.array([
            [-(acrobot.beta1 + acrobot.beta2), -acrobot.beta2],
            [-acrobot.beta2, -acrobot.beta2],
        ])
        A4 = np.zeros((4, 4))
        A4[0, 2] = 1.0
        A4[1, 3] = 1.0
        A4[2:, :2] = -M_inv @ G_jac
        B4 = np.zeros((4, 1))
        B4[2:, 0] = M_inv @ np.array([0.0, 1.0])

        P = solve_continuous_are(A4, B4, Q4, R)
        K4 = np.linalg.solve(R, B4.T @ P).flatten()  # shape (4,)

        # Extend to 5D by appending a small proportional gain on tau2 — this
        # damps the actuator state without saturating the command.
        self.K = np.concatenate([K4, [k_tau]])
        self.x_ref = np.array([np.pi / 2, 0.0, 0.0, 0.0, 0.0])

    def compute(self, state5):
        x_err = np.array(state5) - self.x_ref
        x_err[0] = (x_err[0] + np.pi) % (2.0 * np.pi) - np.pi  # wrap to [-pi, pi]
        return float(-self.K @ x_err)


class SwitchingController:
    """Two-phase controller: backstepping swing-up followed by 5D LQR.

    Switches from swing-up to LQR when the weighted mechanical-state-error norm
    (over the first 4 components only) falls below a threshold.  The switch is
    one-way (once in LQR, stays in LQR).
    """

    def __init__(self, swing_up, lqr, threshold, u_max):
        self.swing_up = swing_up
        self.lqr = lqr
        self.threshold = threshold
        self.u_max = u_max
        self.switched = False
        self.switch_time = None

    def _state_error_norm(self, state5):
        e0 = (state5[0] - np.pi / 2 + np.pi) % (2.0 * np.pi) - np.pi
        return abs(e0) + abs(state5[1]) + 0.1 * abs(state5[2]) + 0.1 * abs(state5[3])

    def compute(self, state5, t=None):
        if not self.switched and self._state_error_norm(state5) < self.threshold:
            self.switched = True
            self.switch_time = t

        if self.switched:
            u = self.lqr.compute(state5)
        else:
            u = self.swing_up.compute(state5)

        return np.clip(u, -self.u_max, self.u_max)

    def reset(self):
        """Reset switching state for trajectory replay."""
        self.switched = False
        self.switch_time = None


class P1BaselineController:
    """Project-1 energy controller naively applied to the actuator-laden plant.

    Outputs the same tau2* as Project 1 *as if it were the commanded u*, ignoring
    the existence of the first-order actuator dynamics.  This is the §10
    comparison baseline: it works fine when Ta -> 0 and degrades as Ta grows.
    """

    def __init__(self, acrobot, kD, kP, kV, u_max):
        self.acrobot = acrobot
        self.kD = kD
        self.kP = kP
        self.kV = kV
        self.u_max = u_max

    def compute(self, state5):
        q1, q2, dq1, dq2, _ = state5
        u = _energy_virtual_torque(
            self.acrobot, q1, q2, dq1, dq2, self.kD, self.kP, self.kV
        )
        return np.clip(u, -self.u_max, self.u_max)
