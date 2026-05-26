"""Controllers for spacecraft rendezvous: constrained MPC and two LQR baselines.

Three controllers are exposed, sharing the same Riccati-derived terminal cost
matrix P:

    * ``LinearMPCController``      -- receding-horizon, hard input constraints,
                                       terminal cost x_N' P x_N, optional
                                       terminal set X_f = { x : x' P x <= rho }.
    * ``LQRController``            -- unconstrained linear feedback u = -K x;
                                       baseline that ignores actuator bounds.
    * ``SaturatedLQRController``   -- same gain as LQR with post-hoc clipping;
                                       baseline that handles constraints naively.

All three return ``(u, J)`` where ``J`` is a Lyapunov-style scalar used by the
visualiser:  for MPC it is the optimal value ``J*(x)``; for the LQR baselines
it is the terminal-cost surrogate ``x' P x`` (still a Lyapunov function under
the unconstrained closed loop).
"""

import numpy as np
import cvxpy as cp
from scipy.linalg import solve_discrete_are


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _solve_dare_with_K(A, B, Q, R):
    """Solve the discrete-time algebraic Riccati equation and return P, K.

    With the convention  u = -K x, the LQR cost-to-go is  V(x) = x' P x, and
    P is the unique symmetric positive-definite solution of

        P = Q + A' P A - A' P B (R + B' P B)^{-1} B' P A.

    The gain is

        K = (R + B' P B)^{-1} B' P A.
    """
    P = solve_discrete_are(A, B, Q, R)
    # Symmetrise to absorb solver round-off; result is still valid because P
    # is theoretically symmetric.
    P = 0.5 * (P + P.T)
    K = np.linalg.solve(R + B.T @ P @ B, B.T @ P @ A)
    return P, K


def terminal_set_rho_admissible(K, P, u_max):
    """Largest rho such that  || K x ||_inf <= u_max  on  { x : x' P x <= rho }.

    For each row k_i of K the maximum of  |k_i' x|  over the ellipsoid
    { x' P x <= rho } equals  sqrt(rho * k_i' P^{-1} k_i)  (Cauchy-Schwarz in
    the P-induced norm).  Requiring this to be at most u_max for every input
    channel gives

        rho <= u_max^2 / max_i  k_i' P^{-1} k_i.

    Returns this upper bound.
    """
    P_inv = np.linalg.inv(P)
    worst = max(float(K[i] @ P_inv @ K[i].T) for i in range(K.shape[0]))
    return (float(u_max) ** 2) / worst


def terminal_set_rho_admissible_state(P, v_max, vel_indices=(2, 3)):
    """Largest rho such that  |x_i| <= v_max  on  { x : x' P x <= rho }  for
    every state index i in ``vel_indices``.

    Same Cauchy-Schwarz argument as ``terminal_set_rho_admissible`` but with
    the coordinate-axis rows e_i in place of the LQR rows.  Used to ensure
    the terminal set lies inside the velocity-state box used in the MPC
    formulation -- this closes the state-admissibility gap in the
    recursive-feasibility argument.
    """
    P_inv = np.linalg.inv(P)
    nx = P.shape[0]
    worst = 0.0
    for i in vel_indices:
        e_i = np.zeros(nx)
        e_i[i] = 1.0
        worst = max(worst, float(e_i @ P_inv @ e_i))
    return (float(v_max) ** 2) / worst


# ---------------------------------------------------------------------------
# MPC
# ---------------------------------------------------------------------------

class LinearMPCController:
    """Linear MPC with input bounds, terminal cost, optional terminal set.

    Receding-horizon optimisation problem at each step k, given the current
    state ``x_k``:

        min  sum_{i=0}^{N-1} ( x_i' Q x_i + u_i' R u_i ) + x_N' P x_N
        s.t. x_0   = x_k
             x_{i+1} = A_d x_i + B_d u_i
             -u_max <= u_i <= u_max          element-wise
             x_N' P x_N <= rho               (if ``use_terminal_set``)

    The first input ``u_0*`` is applied and the problem is re-solved at the
    next sampling instant.
    """

    def __init__(self, system, mpc_params, T_max, v_max=None, v_margin=0.05):
        self.A_d = np.asarray(system.A_d, dtype=float).copy()
        self.B_d = np.asarray(system.B_d, dtype=float).copy()
        self.nx = self.A_d.shape[0]
        self.nu = self.B_d.shape[1]
        self.N = int(mpc_params.N)
        self.Q = np.asarray(mpc_params.Q, dtype=float).copy()
        self.R = np.asarray(mpc_params.R, dtype=float).copy()
        self.T_max = float(T_max)
        # State constraint on per-axis relative velocity.  ``None`` means
        # "no state constraint"; a positive value adds the box
        # |x_dot|, |y_dot| <= v_max to the MPC formulation.
        self.v_max = None if v_max is None else float(v_max)
        # Robustness margin: plan against a tighter bound v_max * (1 - margin)
        # so additive process disturbance does not push the actual state
        # outside the reported v_max.  This is a poor-man's tube-MPC.
        self.v_margin = float(v_margin)
        self.use_terminal_set = bool(mpc_params.use_terminal_set)

        # Riccati / LQR ingredients.
        self.P, self.K = _solve_dare_with_K(self.A_d, self.B_d, self.Q, self.R)
        self.A_K = self.A_d - self.B_d @ self.K

        # Maximum admissible terminal-set radius.  Two requirements combine:
        #   (a) input admissibility:  || K x ||_inf <= T_max  for x in X_f,
        #   (b) state admissibility:  |x_dot|, |y_dot| <= v_plan  for x in X_f,
        # so X_f must lie inside both the input-admissible and the
        # state-admissible ellipsoids.  rho is clipped to the tighter bound.
        self.rho_admissible_input = terminal_set_rho_admissible(self.K, self.P, self.T_max)
        if self.v_max is not None:
            v_plan = self.v_max * (1.0 - self.v_margin)
            self.rho_admissible_state = terminal_set_rho_admissible_state(self.P, v_plan)
            self.rho_admissible = min(self.rho_admissible_input, self.rho_admissible_state)
        else:
            self.rho_admissible_state = float("inf")
            self.rho_admissible = self.rho_admissible_input
        # User-supplied rho: negative sentinel means "auto = admissible max".
        # Positive values are respected but clipped to the admissible bound
        # so the terminal LQR action stays feasible.
        user_rho = float(mpc_params.rho)
        if user_rho <= 0.0:
            self.rho = self.rho_admissible
        else:
            self.rho = min(user_rho, self.rho_admissible)

        # Build the parametric CVXPY problem once and reuse it.
        self._x_var = cp.Variable((self.nx, self.N + 1))
        self._u_var = cp.Variable((self.nu, self.N))
        self._x0_par = cp.Parameter(self.nx)

        Q_psd = cp.psd_wrap(self.Q)
        R_psd = cp.psd_wrap(self.R)
        P_psd = cp.psd_wrap(self.P)

        cost = 0
        constraints = [self._x_var[:, 0] == self._x0_par]
        for k in range(self.N):
            cost += cp.quad_form(self._x_var[:, k], Q_psd)
            cost += cp.quad_form(self._u_var[:, k], R_psd)
            constraints += [
                self._x_var[:, k + 1] == self.A_d @ self._x_var[:, k] + self.B_d @ self._u_var[:, k],
                self._u_var[:, k] <= self.T_max,
                self._u_var[:, k] >= -self.T_max,
            ]
            if self.v_max is not None:
                # Velocity components are states 2 and 3 (x_dot, y_dot).
                # Enforced on the predicted states from the second step on
                # (predicted state 0 is the current measurement; if it is
                # already outside the box, no instantaneous action can fix
                # it).  We tighten by ``v_margin`` so an additive disturbance
                # cannot push the *actual* state past the reported bound.
                v_plan = self.v_max * (1.0 - self.v_margin)
                if k >= 1:
                    constraints += [
                        self._x_var[2, k] <= v_plan,
                        self._x_var[2, k] >= -v_plan,
                        self._x_var[3, k] <= v_plan,
                        self._x_var[3, k] >= -v_plan,
                    ]
        if self.v_max is not None:
            v_plan = self.v_max * (1.0 - self.v_margin)
            constraints += [
                self._x_var[2, self.N] <= v_plan,
                self._x_var[2, self.N] >= -v_plan,
                self._x_var[3, self.N] <= v_plan,
                self._x_var[3, self.N] >= -v_plan,
            ]
        cost += cp.quad_form(self._x_var[:, self.N], P_psd)
        if self.use_terminal_set:
            constraints += [cp.quad_form(self._x_var[:, self.N], P_psd) <= self.rho]

        self._problem = cp.Problem(cp.Minimize(cost), constraints)

    def compute(self, x):
        """Return (u_0*, J*) at the current state ``x``.

        Raises ``RuntimeError`` on solver failure (caller can decide whether
        to substitute a safe back-up controller).
        """
        self._x0_par.value = np.asarray(x, dtype=float)
        # ECOS handles QP+SOC cleanly when ``use_terminal_set`` is True;
        # for pure QP, OSQP would also work but ECOS keeps the code path
        # single-solver for both cases.
        try:
            self._problem.solve(solver=cp.ECOS, verbose=False)
        except cp.SolverError:
            self._problem.solve(solver=cp.SCS, verbose=False)
        if self._problem.status not in ("optimal", "optimal_inaccurate"):
            raise RuntimeError(f"MPC problem status: {self._problem.status}")
        u0 = np.array(self._u_var.value[:, 0], dtype=float)
        # Defensive clip: SCS/ECOS may overshoot the hard bound by their
        # feasibility tolerance (typ. 1e-3) when the optimum lies on the
        # constraint.  Enforce the actuator limit unconditionally.
        u0 = np.clip(u0, -self.T_max, self.T_max)
        J_star = float(self._problem.value)
        return u0, J_star


# ---------------------------------------------------------------------------
# LQR baselines
# ---------------------------------------------------------------------------

class LQRController:
    """Unconstrained discrete-time LQR feedback.

    Provided as a comparison baseline that ignores actuator bounds: it
    exposes the saturation problem that MPC is designed to avoid.
    """

    def __init__(self, system, mpc_params):
        self.A_d = np.asarray(system.A_d, dtype=float).copy()
        self.B_d = np.asarray(system.B_d, dtype=float).copy()
        self.Q = np.asarray(mpc_params.Q, dtype=float).copy()
        self.R = np.asarray(mpc_params.R, dtype=float).copy()
        self.P, self.K = _solve_dare_with_K(self.A_d, self.B_d, self.Q, self.R)
        self.A_K = self.A_d - self.B_d @ self.K

    def compute(self, x):
        x = np.asarray(x, dtype=float)
        u = -self.K @ x
        J = float(x.T @ self.P @ x)
        return u, J


class SaturatedLQRController(LQRController):
    """LQR feedback with post-hoc element-wise clipping to +/- T_max.

    Demonstrates the simplest "engineering" fix to LQR-with-constraints:
    just clip the action.  The clipped closed loop no longer has the
    stability guarantee of the underlying LQR and may oscillate when the
    unconstrained command lies far outside the feasible set.
    """

    def __init__(self, system, mpc_params, T_max):
        super().__init__(system, mpc_params)
        self.T_max = float(T_max)

    def compute(self, x):
        u_raw, J = super().compute(x)
        u_sat = np.clip(u_raw, -self.T_max, self.T_max)
        return u_sat, J
