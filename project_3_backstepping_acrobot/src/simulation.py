import numpy as np
from scipy.integrate import solve_ivp


def _state_error_norm(state5):
    """Weighted mechanical-state-error norm relative to the upright equilibrium.

    Only the first four state components participate; the actuator state tau2
    is not part of the LQR switching criterion (the 5D LQR handles it on the
    other side of the switch).
    """
    e0 = (state5[0] - np.pi / 2 + np.pi) % (2.0 * np.pi) - np.pi
    return abs(e0) + abs(state5[1]) + 0.1 * abs(state5[2]) + 0.1 * abs(state5[3])


def simulate(acrobot, swing_up, lqr, u_max, switch_threshold,
             x0, t_final, dt=0.005):
    """Two-phase simulation of the 5D acrobot-with-actuator system.

    Phase 1 integrates the backstepping swing-up controller until the weighted
    mechanical-state-error norm drops below *switch_threshold*.  Phase 2 takes
    over with the 5D LQR until *t_final*.  solve_ivp event detection finds the
    switch point cleanly, avoiding mutable controller state inside the adaptive
    ODE solver.

    The control input passed to acrobot.dynamics() is the commanded torque u,
    not the actuator output: the integrator handles the first-order lag.
    """
    t_eval = np.arange(0.0, t_final, dt)

    # ---- Phase 1: Backstepping swing-up ----
    def rhs_swing(t, state5):
        u = np.clip(swing_up.compute(state5), -u_max, u_max)
        return acrobot.dynamics(state5, u)

    def switch_event(t, state5):
        return _state_error_norm(state5) - switch_threshold

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
        def rhs_lqr(t, state5):
            u = np.clip(lqr.compute(state5), -u_max, u_max)
            return acrobot.dynamics(state5, u)

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

    # ---- Post-process: control, virtual control, and energy at output points ----
    n = len(t_all)
    u_vals = np.empty(n)
    tau_star_vals = np.empty(n)
    E_vals = np.empty(n)
    for i in range(n):
        s = y_all[:, i]
        if switch_time is not None and t_all[i] >= switch_time:
            u_vals[i] = np.clip(lqr.compute(s), -u_max, u_max)
        else:
            u_vals[i] = np.clip(swing_up.compute(s), -u_max, u_max)
        tau_star_vals[i] = swing_up.virtual_torque(s)
        E_vals[i] = acrobot.total_energy(s[0], s[1], s[2], s[3])

    return {
        "t": t_all,
        "q1": y_all[0],
        "q2": y_all[1],
        "dq1": y_all[2],
        "dq2": y_all[3],
        "tau2": y_all[4],
        "u": u_vals,
        "tau2_star": tau_star_vals,
        "E": E_vals,
        "E_upright": acrobot.E_upright,
        "switch_time": switch_time,
    }


def simulate_baseline(acrobot, baseline, lqr, u_max, switch_threshold,
                      x0, t_final, dt=0.005):
    """Two-phase simulation of the P1 baseline controller on the 5D plant.

    The baseline commands u = tau2*(x) directly — exactly P1's energy law,
    naively applied through the first-order actuator with no awareness of the
    lag.  Phase-1 runs the baseline until the weighted state-error norm drops
    below *switch_threshold*; phase-2 hands over to the same LQR used by P3.

    The §10 comparison expects this to FAIL: with actuator dynamics the
    baseline cannot pump enough energy through the lagged actuator to enter
    the LQR basin, so the switch event never fires and the trajectory
    sustains a limit cycle around E < E_r.
    """
    t_eval = np.arange(0.0, t_final, dt)

    def rhs_swing(t, state5):
        u = np.clip(baseline.compute(state5), -u_max, u_max)
        return acrobot.dynamics(state5, u)

    def switch_event(t, state5):
        return _state_error_norm(state5) - switch_threshold

    switch_event.terminal = True
    switch_event.direction = -1

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

        mask1 = sol1.t <= switch_time
        t1 = sol1.t[mask1]
        y1 = sol1.y[:, mask1]

        def rhs_lqr(t, state5):
            u = np.clip(lqr.compute(state5), -u_max, u_max)
            return acrobot.dynamics(state5, u)

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

    n = len(t_all)
    u_vals = np.empty(n)
    E_vals = np.empty(n)
    for i in range(n):
        s = y_all[:, i]
        if switch_time is not None and t_all[i] >= switch_time:
            u_vals[i] = np.clip(lqr.compute(s), -u_max, u_max)
        else:
            u_vals[i] = np.clip(baseline.compute(s), -u_max, u_max)
        E_vals[i] = acrobot.total_energy(s[0], s[1], s[2], s[3])

    return {
        "t": t_all,
        "q1": y_all[0],
        "q2": y_all[1],
        "dq1": y_all[2],
        "dq2": y_all[3],
        "tau2": y_all[4],
        "u": u_vals,
        "tau2_star": u_vals.copy(),  # baseline outputs tau2* directly as u
        "E": E_vals,
        "E_upright": acrobot.E_upright,
        "switch_time": switch_time,
    }
