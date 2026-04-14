import numpy as np
from scipy.integrate import solve_ivp


def _state_error_norm(state):
    """Weighted state-error norm relative to the upright equilibrium."""
    e0 = (state[0] - np.pi / 2 + np.pi) % (2.0 * np.pi) - np.pi
    return abs(e0) + abs(state[1]) + 0.1 * abs(state[2]) + 0.1 * abs(state[3])


def simulate(acrobot, swing_up, lqr, u_max, switch_threshold,
             x0, t_final, dt=0.005):
    """Two-phase simulation: energy swing-up, then LQR stabilization.

    Uses solve_ivp event detection to locate the switch point cleanly,
    avoiding mutable controller state inside the adaptive ODE solver.

    Phase 1 integrates with the energy-based controller until the weighted
    state-error norm drops below *switch_threshold*.  Phase 2 continues
    from that state with the LQR controller until *t_final*.
    """
    t_eval = np.arange(0.0, t_final, dt)

    # ---- Phase 1: Energy-based swing-up ----
    def rhs_swing(t, state):
        u = np.clip(swing_up.compute(state), -u_max, u_max)
        return acrobot.dynamics(state, u)

    def switch_event(t, state):
        return _state_error_norm(state) - switch_threshold

    switch_event.terminal = True
    switch_event.direction = -1  # fire on downward zero-crossing

    sol1 = solve_ivp(
        rhs_swing, [0, t_final], list(x0),
        events=switch_event,
        t_eval=t_eval,
        method="RK45", rtol=1e-8, atol=1e-8, max_step=0.005,
    )

    switch_time = None

    if sol1.t_events[0].size > 0:
        switch_time = float(sol1.t_events[0][0])
        x_switch = sol1.y_events[0][0]

        # Keep only phase-1 output points up to the switch
        mask1 = sol1.t <= switch_time
        t1 = sol1.t[mask1]
        y1 = sol1.y[:, mask1]

        # ---- Phase 2: LQR stabilization ----
        def rhs_lqr(t, state):
            u = np.clip(lqr.compute(state), -u_max, u_max)
            return acrobot.dynamics(state, u)

        t_eval_2 = t_eval[t_eval > switch_time]
        sol2 = solve_ivp(
            rhs_lqr, [switch_time, t_final], list(x_switch),
            t_eval=t_eval_2,
            method="RK45", rtol=1e-8, atol=1e-8,
        )

        t_all = np.concatenate([t1, sol2.t])
        y_all = np.concatenate([y1, sol2.y], axis=1)
    else:
        t_all = sol1.t
        y_all = sol1.y

    # ---- Post-process: compute control and energy at output points ----
    n = len(t_all)
    u_vals = np.empty(n)
    E_vals = np.empty(n)
    for i in range(n):
        s = y_all[:, i]
        if switch_time is not None and t_all[i] >= switch_time:
            u_vals[i] = np.clip(lqr.compute(s), -u_max, u_max)
        else:
            u_vals[i] = np.clip(swing_up.compute(s), -u_max, u_max)
        E_vals[i] = acrobot.total_energy(*s)

    return {
        "t": t_all,
        "q1": y_all[0],
        "q2": y_all[1],
        "dq1": y_all[2],
        "dq2": y_all[3],
        "u": u_vals,
        "E": E_vals,
        "E_upright": acrobot.E_upright,
        "switch_time": switch_time,
    }
