"""Closed-loop simulation of MPC and LQR baselines for the rendezvous problem.

A single ``simulate(...)`` routine drives the linear CW plant through
``n_steps`` zero-order-hold updates, invoking the supplied controller at
every step.  An optional constant additive acceleration disturbance models
slowly-varying perturbations (differential drag, J2, third-body), entering
through the same channel as the control input.

Because the plant is exactly LTI, ZOH integration is exact -- no numerical
ODE solver is involved.  The trajectory dict returned matches the
conventions used elsewhere in the project for direct plug-in to
``src.visualization``.
"""

import numpy as np


def simulate(system, controller, sim_params, T_max, v_max=None):
    """Run one closed-loop trajectory.

    Parameters
    ----------
    system : ClohessyWiltshire
        Plant model with ``A_d, B_d, step, Ts, m`` attributes.
    controller : object exposing ``compute(x) -> (u, J)``
        Any of the controllers from ``src.controller``.  ``J`` is the
        controller's internal Lyapunov-style value (``J*(x)`` for MPC,
        ``x' P x`` for LQR baselines).
    sim_params : SimParams
        Initial state, simulation horizon, and disturbance specification.
    T_max : float
        Per-axis input bound used purely for violation logging.

    Returns
    -------
    dict with keys ``t``, ``x``, ``y``, ``vx``, ``vy``, ``ux``, ``uy``,
    ``J``, ``violations``, ``violation_count``, ``state``, ``control``,
    ``Ts``, ``T_max``, ``disturbance``.  Time vector ``t`` has length
    ``n_steps + 1``; state-like arrays match its length; input arrays
    are length ``n_steps``.
    """
    Ts = float(system.Ts)
    # Floor (with a small epsilon for float robustness) so we never simulate
    # past the requested horizon when t_final / Ts is not exactly integral.
    n_steps = int(np.floor(float(sim_params.t_final) / Ts + 1e-9))

    nx = system.A_d.shape[0]
    nu = system.B_d.shape[1]

    # Disturbance enters as a constant acceleration on the (x_ddot, y_ddot)
    # channels.  Converted to a thrust-equivalent so we can reuse the input
    # matrix B_d for ZOH integration.
    a_disturbance = np.asarray(sim_params.disturbance, dtype=float)
    w_thrust = float(system.m) * a_disturbance

    state = np.zeros((n_steps + 1, nx))
    control = np.zeros((n_steps, nu))
    J = np.zeros(n_steps + 1)
    violations = np.zeros(n_steps, dtype=int)
    state_violations = np.zeros(n_steps + 1, dtype=int)

    state[0] = np.asarray(sim_params.x0, dtype=float)
    if v_max is not None and max(abs(state[0, 2]), abs(state[0, 3])) > v_max + 1e-9:
        state_violations[0] = 1

    for k in range(n_steps):
        x_k = state[k]
        u_k, J_k = controller.compute(x_k)
        control[k] = u_k
        J[k] = J_k
        if float(np.max(np.abs(u_k))) > T_max + 1e-9:
            violations[k] = 1
        # ZOH update including the additive disturbance.
        state[k + 1] = system.A_d @ x_k + system.B_d @ (u_k + w_thrust)
        if v_max is not None and max(abs(state[k + 1, 2]), abs(state[k + 1, 3])) > v_max + 1e-9:
            state_violations[k + 1] = 1

    # Lyapunov value at the terminal state, for plotting continuity.
    _, J[n_steps] = controller.compute(state[n_steps])

    t = np.arange(n_steps + 1) * Ts

    return {
        "t": t,
        "x": state[:, 0],
        "y": state[:, 1],
        "vx": state[:, 2],
        "vy": state[:, 3],
        "ux": control[:, 0],
        "uy": control[:, 1],
        "J": J,
        "violations": violations,
        "violation_count": int(violations.sum()),
        "state_violations": state_violations,
        "state_violation_count": int(state_violations.sum()),
        "state": state,
        "control": control,
        "Ts": Ts,
        "T_max": float(T_max),
        "v_max": None if v_max is None else float(v_max),
        "disturbance": np.asarray(sim_params.disturbance, dtype=float).copy(),
    }
