import numpy as np
from scipy.integrate import solve_ivp

from src.controller import AdaptiveSwingUpController, state_error_norm


def simulate(
    acrobot,
    swing_up,
    lqr,
    u_max,
    switch_threshold,
    x0,
    t_final,
    dt=0.005,
):
    """Two-phase simulation: energy swing-up (possibly adaptive), then LQR.

    The phase-1 ODE state is augmented with ``b_hat`` whenever the
    swing-up controller is adaptive.  This way the parameter estimate is
    integrated by the same RK45 stepper that integrates the dynamics,
    avoiding solver-order hacks.

    Phase 2 keeps ``b_hat`` frozen at its hand-over value; LQR runs on
    the four-dimensional dynamic state alone.

    Returns a dict of trajectories suitable for :mod:`src.visualization`.
    """
    is_adaptive = isinstance(swing_up, AdaptiveSwingUpController)
    if is_adaptive:
        swing_up.reset()

    t_eval = np.arange(0.0, t_final, dt)

    def rhs_swing(t, state):
        if is_adaptive:
            x, b_hat = state[:4], state[4]
            u = np.clip(swing_up.compute(x, b_hat=b_hat), -u_max, u_max)
            ds = acrobot.dynamics(x, u)
            db = swing_up.adapt_rate(x)
            return np.concatenate([ds, [db]])
        u = np.clip(swing_up.compute(state), -u_max, u_max)
        return acrobot.dynamics(state, u)

    def switch_event(t, state):
        return state_error_norm(state[:4]) - switch_threshold

    switch_event.terminal = True
    switch_event.direction = -1

    init = list(x0) + [swing_up.b_hat_0] if is_adaptive else list(x0)

    sol1 = solve_ivp(
        rhs_swing,
        [0.0, t_final],
        init,
        events=switch_event,
        t_eval=t_eval,
        method="RK45",
        rtol=1e-8,
        atol=1e-8,
        max_step=0.005,
    )

    switch_time = None
    if sol1.t_events[0].size > 0:
        switch_time = float(sol1.t_events[0][0])
        x_switch = sol1.y_events[0][0]

        mask1 = sol1.t <= switch_time
        t1 = sol1.t[mask1]
        y1 = sol1.y[:, mask1]

        x4_switch = x_switch[:4]
        if is_adaptive:
            swing_up.b_hat = float(x_switch[4])

        def rhs_lqr(t, state):
            u = np.clip(lqr.compute(state), -u_max, u_max)
            return acrobot.dynamics(state, u)

        t_eval_2 = t_eval[t_eval > switch_time]
        sol2 = solve_ivp(
            rhs_lqr,
            [switch_time, t_final],
            list(x4_switch),
            t_eval=t_eval_2,
            method="RK45",
            rtol=1e-8,
            atol=1e-8,
        )

        if is_adaptive:
            n2 = sol2.y.shape[1]
            y2 = np.vstack([sol2.y, np.full(n2, swing_up.b_hat)])
        else:
            y2 = sol2.y

        t_all = np.concatenate([t1, sol2.t])
        y_all = np.concatenate([y1, y2], axis=1)
    else:
        t_all = sol1.t
        y_all = sol1.y

    n = len(t_all)
    u_vals = np.empty(n)
    E_vals = np.empty(n)
    V_vals = np.empty(n)
    b_hat_vals = np.full(n, np.nan)

    for i in range(n):
        s = y_all[:, i]
        x4 = s[:4]
        if switch_time is not None and t_all[i] >= switch_time:
            u_vals[i] = np.clip(lqr.compute(x4), -u_max, u_max)
        elif is_adaptive:
            u_vals[i] = np.clip(swing_up.compute(x4, b_hat=s[4]), -u_max, u_max)
        else:
            u_vals[i] = np.clip(swing_up.compute(x4), -u_max, u_max)
        E_vals[i] = acrobot.total_energy(*x4)
        if is_adaptive:
            b_hat_vals[i] = s[4]
            V_vals[i] = swing_up.lyapunov(x4, b_hat=s[4], b_true=acrobot.b2)
        else:
            E_err = E_vals[i] - acrobot.E_upright
            V_vals[i] = (
                0.5 * E_err**2
                + 0.5 * swing_up.kD * x4[3] ** 2
                + 0.5 * swing_up.kP * x4[1] ** 2
            )

    return {
        "t": t_all,
        "q1": y_all[0],
        "q2": y_all[1],
        "dq1": y_all[2],
        "dq2": y_all[3],
        "u": u_vals,
        "E": E_vals,
        "E_upright": acrobot.E_upright,
        "V": V_vals,
        "b_hat": b_hat_vals,
        "b_true": acrobot.b2,
        "switch_time": switch_time,
        "is_adaptive": is_adaptive,
    }
