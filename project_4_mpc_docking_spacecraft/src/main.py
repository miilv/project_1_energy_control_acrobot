"""Entry point: run the three rendezvous scenarios and generate all artifacts.

The three scenarios share the same Clohessy-Wiltshire plant and initial
state; only the controller changes.

    1. ``mpc``                -- constrained linear MPC with terminal cost
                                 and terminal set (the main result).
    2. ``lqr_unconstrained``  -- LQR with no constraint enforcement
                                 (illustrates actuator saturation).
    3. ``lqr_saturated``      -- LQR with post-hoc clipping of the command
                                 to the actuator bound.

Usage (from the project root directory):

    python -m src.main                          # all three scenarios
    python -m src.main --no-anim                # skip the GIFs (faster)
    python -m src.main --scenario mpc           # one scenario only
"""

import argparse
from pathlib import Path

import numpy as np

from configs.params import PhysicalParams, MPCParams, SimParams
from src.system import ClohessyWiltshire
from src.controller import (
    LinearMPCController,
    LQRController,
    SaturatedLQRController,
)
from src.simulation import simulate
from src import visualization as viz
from src import animation as ani


PROJECT_ROOT = Path(__file__).resolve().parent.parent
FIG_DIR = PROJECT_ROOT / "figures"
ANIM_DIR = PROJECT_ROOT / "animations"

SCENARIOS = ("mpc", "lqr_unconstrained", "lqr_saturated")


def _build_controller(name, system, mpc_params, T_max, v_max=None):
    if name == "mpc":
        return LinearMPCController(system, mpc_params, T_max, v_max=v_max)
    if name == "lqr_unconstrained":
        return LQRController(system, mpc_params)
    if name == "lqr_saturated":
        return SaturatedLQRController(system, mpc_params, T_max)
    raise ValueError(f"unknown scenario {name!r}")


def _generate_scenario_plots(name, result):
    out = FIG_DIR / name
    viz.plot_state_trajectories(result, str(out / "states.png"), scenario=name)
    viz.plot_control(result, str(out / "control.png"), scenario=name)
    viz.plot_lyapunov(result, str(out / "lyapunov.png"), scenario=name)
    viz.plot_position_trajectory(result, str(out / "position_trajectory.png"), scenario=name)
    viz.plot_distance_to_target(result, str(out / "range.png"), scenario=name)


def main():
    parser = argparse.ArgumentParser(
        description="Spacecraft rendezvous: MPC and LQR baselines on the CW plant.",
    )
    parser.add_argument("--no-anim", action="store_true",
                        help="Skip GIF generation (much faster).")
    parser.add_argument("--scenario", choices=SCENARIOS + ("all",), default="all",
                        help="Which scenario(s) to run.")
    args = parser.parse_args()

    phys = PhysicalParams()
    mpc_params = MPCParams()
    sim_params = SimParams()
    system = ClohessyWiltshire(phys)

    print("=== Spacecraft rendezvous (Clohessy-Wiltshire MPC) ===")
    print(f"  orbital rate n     = {phys.n:.4e} rad/s  (period {phys.period:.1f} s)")
    print(f"  chaser mass        = {phys.m:.1f} kg")
    print(f"  per-axis thrust    = +/- {phys.T_max:.1f} N")
    print(f"  per-axis velocity  = +/- {phys.v_max:.2f} m/s   (MPC state constraint)")
    print(f"  initial state      = {tuple(sim_params.x0)}")
    print(f"  simulation horizon = {sim_params.t_final:.1f} s")
    print()

    scenarios = SCENARIOS if args.scenario == "all" else (args.scenario,)
    results = {}
    for name in scenarios:
        print(f"--- running {name} ---")
        ctrl = _build_controller(name, system, mpc_params, phys.T_max, v_max=phys.v_max)
        if name == "mpc":
            print(f"  rho admissible (input) = {ctrl.rho_admissible_input:.4e}")
            print(f"  rho admissible (state) = {ctrl.rho_admissible_state:.4e}")
            print(f"  rho used               = {ctrl.rho:.4e}")
            A_K_eigs = np.linalg.eigvals(ctrl.A_K)
            print(f"  max |lambda(A_K)|     = {max(abs(A_K_eigs)):.6f}   (Schur => stable)")
        r = simulate(system, ctrl, sim_params, phys.T_max, v_max=phys.v_max)
        results[name] = r
        end_range = float(np.sqrt(r['x'][-1] ** 2 + r['y'][-1] ** 2))
        print(f"  final range = {end_range:.3f} m, input violations = {r['violation_count']}, "
              f"state violations = {r['state_violation_count']}, final J = {r['J'][-1]:.3e}")
        print()

    print("--- generating per-scenario figures ---")
    for name, r in results.items():
        _generate_scenario_plots(name, r)

    if len(results) > 1:
        print("--- generating comparison figures ---")
        viz.plot_comparison_position(results, str(FIG_DIR / "comparison_position.png"))
        viz.plot_comparison_J(results, str(FIG_DIR / "comparison_lyapunov.png"))
        viz.plot_comparison_thrust(results, str(FIG_DIR / "comparison_thrust.png"))
        viz.plot_comparison_velocity(results, str(FIG_DIR / "comparison_velocity.png"))
        viz.plot_constraint_violations(results, str(FIG_DIR / "comparison_violations.png"))

    if not args.no_anim:
        print("--- rendering animations ---")
        for name, r in results.items():
            print(f"  {name}.gif ...")
            ani.make_gif(r, str(ANIM_DIR / f"{name}.gif"), fps=20, scenario=name)

    print("done.")


if __name__ == "__main__":
    main()
