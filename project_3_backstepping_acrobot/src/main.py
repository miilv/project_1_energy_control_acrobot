"""Entry point: simulate the acrobot with actuator dynamics under backstepping,
generate the required plots, the comparison overlays against the P1 baseline,
and the animation.

Usage (from the project root directory):
    python -m src.main              # full run (plots + comparison + animation)
    python -m src.main --no-anim    # skip GIF (much faster)
"""

import argparse
from pathlib import Path

import numpy as np

from configs.params import PhysicalParams, ActuatorParams, ControlParams, SimParams
from src.system import Acrobot
from src.controller import (
    BackstepSwingUpController,
    LQR5DController,
    P1BaselineController,
)
from src.simulation import simulate, simulate_baseline
from src.visualization import (
    generate_all_plots,
    plot_comparison_energy,
    plot_comparison_error,
)
from src.animation import create_animation

PROJECT_ROOT = Path(__file__).resolve().parent.parent


def main():
    parser = argparse.ArgumentParser(
        description="Acrobot backstepping swing-up with first-order actuator dynamics"
    )
    parser.add_argument("--no-anim", action="store_true",
                        help="Skip GIF generation (faster)")
    args = parser.parse_args()

    phys = PhysicalParams()
    act = ActuatorParams()
    ctrl = ControlParams()
    sim = SimParams()

    # --- Build system ---
    acrobot = Acrobot(phys, act)

    kD_min = acrobot.solvability_bound()
    print(f"Solvability bound:  kD > {kD_min:.3f}")
    print(f"Using kD = {ctrl.kD}  (margin {ctrl.kD - kD_min:.3f})")
    assert ctrl.kD > kD_min, (
        f"kD = {ctrl.kD} does not exceed solvability bound {kD_min:.3f}"
    )
    print(f"Actuator time constant Ta = {act.Ta} s")
    print(f"Backstepping gain      kz = {ctrl.kz}")

    # --- Build controllers ---
    swing_up = BackstepSwingUpController(
        acrobot, ctrl.kD, ctrl.kP, ctrl.kV, ctrl.kz, ctrl.u_max,
    )

    Q = np.diag(ctrl.Q_diag)
    R = np.array([[ctrl.R_val]])
    lqr = LQR5DController(acrobot, Q, R, k_tau=ctrl.k_tau)
    print(f"5D LQR gain K = {np.array2string(lqr.K, precision=3)}")

    # --- Simulate backstepping ---
    print(f"\n[P3] Simulating {sim.t_final} s from x0 = {sim.x0} ...")
    results = simulate(
        acrobot, swing_up, lqr, ctrl.u_max, ctrl.switch_threshold,
        sim.x0, sim.t_final, sim.dt,
    )

    if results["switch_time"] is not None:
        print(f"[P3] Switched to LQR at t = {results['switch_time']:.2f} s")
    else:
        print("[P3] Warning: LQR switch did not occur within the simulation window.")

    # --- Plots ---
    figures_dir = PROJECT_ROOT / "figures"
    print(f"Generating plots in {figures_dir}/ ...")
    generate_all_plots(results, ctrl.u_max, figures_dir)
    print("Plots saved.")

    # --- §10 comparison: P1 energy controller naively applied to the same plant ---
    print(f"\n[P1 baseline] Simulating energy controller on plant with Ta = {act.Ta} s ...")
    baseline = P1BaselineController(acrobot, ctrl.kD, ctrl.kP, ctrl.kV, ctrl.u_max)
    results_baseline = simulate_baseline(
        acrobot, baseline, lqr, ctrl.u_max, ctrl.switch_threshold,
        sim.x0, sim.t_final, sim.dt,
    )
    if results_baseline["switch_time"] is None:
        print(f"[P1 baseline] LQR switch did not fire — baseline failed to enter the basin.")
    else:
        print(f"[P1 baseline] LQR switch at t = {results_baseline['switch_time']:.2f} s")

    comparison = {"p1_baseline": results_baseline, "p3_backstep": results}
    plot_comparison_energy(comparison, figures_dir / "comparison_energy.png")
    plot_comparison_error(comparison, figures_dir / "comparison_error.png")
    print("Comparison plots saved.")

    # --- Animation ---
    if not args.no_anim:
        anim_dir = PROJECT_ROOT / "animations"
        anim_dir.mkdir(parents=True, exist_ok=True)
        gif_path = anim_dir / "acrobot_swingup.gif"
        print(f"\nCreating animation: {gif_path} ...")
        ani = create_animation(acrobot, results)
        ani.save(str(gif_path), writer="pillow", fps=30)
        print("Animation saved.")
    else:
        print("Skipping animation (--no-anim).")

    print("\nDone.")


if __name__ == "__main__":
    main()
