"""Plot generation for the spacecraft-rendezvous MPC project.

Each function saves a single figure to a given path.  Style follows the
Project-1 conventions used elsewhere in the repository: titled, labelled
with units, legend whenever more than one curve is drawn, grid on at
alpha 0.4, dpi 150.
"""

import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection


def _ensure_dir(path):
    parent = os.path.dirname(path)
    if parent:
        os.makedirs(parent, exist_ok=True)


# ---------------------------------------------------------------------------
# Per-scenario plots
# ---------------------------------------------------------------------------

def plot_state_trajectories(result, save_path):
    """Relative position and velocity versus time."""
    _ensure_dir(save_path)
    fig, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True)

    axes[0].plot(result["t"], result["x"], label="$x$ (radial)")
    axes[0].plot(result["t"], result["y"], label="$y$ (along-track)")
    axes[0].axhline(0.0, color="gray", linestyle=":", alpha=0.6)
    axes[0].set_ylabel("Relative position [m]")
    axes[0].set_title("Relative position: chaser approaches the target at the origin")
    axes[0].grid(True, alpha=0.4)
    axes[0].legend()

    axes[1].plot(result["t"], result["vx"], label=r"$\dot x$")
    axes[1].plot(result["t"], result["vy"], label=r"$\dot y$")
    axes[1].axhline(0.0, color="gray", linestyle=":", alpha=0.6)
    v_max = result.get("v_max")
    if v_max is not None:
        axes[1].axhline(v_max, color="red", linestyle="--", alpha=0.7,
                        label=fr"$\pm v_\mathrm{{max}} = {v_max:g}$ m/s")
        axes[1].axhline(-v_max, color="red", linestyle="--", alpha=0.7)
    axes[1].set_xlabel("Time [s]")
    axes[1].set_ylabel("Relative velocity [m/s]")
    axes[1].set_title(r"Relative velocity: state constraint $|\dot x|, |\dot y| \leq v_\mathrm{max}$")
    axes[1].grid(True, alpha=0.4)
    axes[1].legend()

    fig.tight_layout()
    fig.savefig(save_path, dpi=150)
    plt.close(fig)


def plot_control(result, save_path):
    """Per-axis thrust with the saturation envelope."""
    _ensure_dir(save_path)
    T_max = float(result["T_max"])
    fig, ax = plt.subplots(figsize=(10, 5))
    t_u = result["t"][:len(result["ux"])]
    ax.plot(t_u, result["ux"], color="navy", linewidth=1.3, label=r"$u_x$")
    ax.plot(t_u, result["uy"], color="darkorange", linewidth=1.3, label=r"$u_y$")
    ax.axhline(T_max, color="red", linestyle="--", alpha=0.7, label=fr"$\pm T_\mathrm{{max}} = {T_max:g}$ N")
    ax.axhline(-T_max, color="red", linestyle="--", alpha=0.7)
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Thrust [N]")
    ax.set_title("Control input: per-axis thrust within saturation limits")
    ax.grid(True, alpha=0.4)
    ax.legend()
    fig.tight_layout()
    fig.savefig(save_path, dpi=150)
    plt.close(fig)


def plot_lyapunov(result, save_path):
    """Optimal value (MPC) or terminal-cost surrogate (LQR) versus time, log scale."""
    _ensure_dir(save_path)
    fig, ax = plt.subplots(figsize=(10, 5))
    ax.semilogy(result["t"], np.maximum(result["J"], 1e-12),
                color="purple", linewidth=1.6, label="$J^*(t)$")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Lyapunov value $J^*$")
    ax.set_title("MPC optimal value $J^*(x_k)$: monotone decrease confirms stability")
    ax.grid(True, which="both", alpha=0.4)
    ax.legend()
    fig.tight_layout()
    fig.savefig(save_path, dpi=150)
    plt.close(fig)


def plot_position_trajectory(result, save_path):
    """2D x-y trace coloured by time, with start and target markers."""
    _ensure_dir(save_path)
    fig, ax = plt.subplots(figsize=(8, 7))
    pts = np.array([result["x"], result["y"]]).T.reshape(-1, 1, 2)
    segs = np.concatenate([pts[:-1], pts[1:]], axis=1)
    norm = plt.Normalize(result["t"].min(), result["t"].max())
    lc = LineCollection(segs, cmap="viridis", norm=norm, linewidth=1.8)
    lc.set_array(result["t"][:-1])
    ax.add_collection(lc)
    ax.scatter(result["x"][0], result["y"][0], color="green", s=130,
               marker="o", label="start", zorder=5)
    ax.scatter(0.0, 0.0, color="red", s=200, marker="*", label="target",
               zorder=5, edgecolors="black", linewidths=1.0)
    ax.scatter(result["x"][-1], result["y"][-1], color="black", s=110,
               marker="x", label="end", zorder=5)
    pad = 5.0
    xlim = (min(result["x"].min(), -pad) - pad, max(result["x"].max(), pad) + pad)
    ylim = (min(result["y"].min(), -pad) - pad, max(result["y"].max(), pad) + pad)
    ax.set_xlim(xlim)
    ax.set_ylim(ylim)
    ax.set_xlabel("$x$ -- radial [m]")
    ax.set_ylabel("$y$ -- along-track [m]")
    ax.set_title("2D trajectory in the target's LVLH frame")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.4)
    ax.legend(loc="upper right")
    cbar = fig.colorbar(lc, ax=ax)
    cbar.set_label("Time [s]")
    fig.tight_layout()
    fig.savefig(save_path, dpi=150)
    plt.close(fig)


def plot_distance_to_target(result, save_path):
    """Range $r(t) = \sqrt{x^2 + y^2}$ on log scale."""
    _ensure_dir(save_path)
    r = np.sqrt(result["x"] ** 2 + result["y"] ** 2)
    fig, ax = plt.subplots(figsize=(10, 5))
    ax.semilogy(result["t"], np.maximum(r, 1e-12), color="darkred", linewidth=1.5)
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Range to target [m]")
    ax.set_title("Range to target: exponential approach under MPC")
    ax.grid(True, which="both", alpha=0.4)
    fig.tight_layout()
    fig.savefig(save_path, dpi=150)
    plt.close(fig)


# ---------------------------------------------------------------------------
# Cross-scenario comparison plots
# ---------------------------------------------------------------------------

_COLORS = {
    "mpc": "seagreen",
    "lqr_unconstrained": "crimson",
    "lqr_saturated": "steelblue",
}

_LABELS = {
    "mpc": "MPC (ours)",
    "lqr_unconstrained": "LQR -- unconstrained (saturates)",
    "lqr_saturated": "LQR -- post-hoc saturated",
}


def plot_comparison_position(results_by_name, save_path):
    """Overlay 2D trajectories across all scenarios."""
    _ensure_dir(save_path)
    fig, ax = plt.subplots(figsize=(9, 8))
    for name, r in results_by_name.items():
        ax.plot(r["x"], r["y"], linewidth=1.6,
                color=_COLORS.get(name), label=_LABELS.get(name, name))
        ax.scatter(r["x"][0], r["y"][0], color=_COLORS.get(name),
                   s=70, marker="o", zorder=4)
        ax.scatter(r["x"][-1], r["y"][-1], color=_COLORS.get(name),
                   s=70, marker="x", zorder=4)
    ax.scatter(0.0, 0.0, color="black", s=200, marker="*",
               edgecolors="white", linewidths=1.0, label="target", zorder=5)
    ax.set_xlabel("$x$ -- radial [m]")
    ax.set_ylabel("$y$ -- along-track [m]")
    ax.set_title("Comparison: chaser path under MPC vs LQR baselines")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.4)
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(save_path, dpi=150)
    plt.close(fig)


def plot_comparison_J(results_by_name, save_path):
    """Overlay J*(t) across scenarios, log scale."""
    _ensure_dir(save_path)
    fig, ax = plt.subplots(figsize=(10, 5))
    for name, r in results_by_name.items():
        ax.semilogy(r["t"], np.maximum(r["J"], 1e-12), linewidth=1.5,
                    color=_COLORS.get(name), label=_LABELS.get(name, name))
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Lyapunov value $J^*$")
    ax.set_title("Comparison: Lyapunov decrease across scenarios")
    ax.grid(True, which="both", alpha=0.4)
    ax.legend()
    fig.tight_layout()
    fig.savefig(save_path, dpi=150)
    plt.close(fig)


def plot_comparison_thrust(results_by_name, save_path):
    """Per-scenario max-norm of the commanded thrust over time."""
    _ensure_dir(save_path)
    fig, ax = plt.subplots(figsize=(10, 5))
    T_max = next(iter(results_by_name.values()))["T_max"]
    for name, r in results_by_name.items():
        t = r["t"][:len(r["ux"])]
        u_norm = np.maximum(np.abs(r["ux"]), np.abs(r["uy"]))
        ax.plot(t, u_norm, linewidth=1.4,
                color=_COLORS.get(name), label=_LABELS.get(name, name))
    ax.axhline(T_max, color="black", linestyle="--", alpha=0.7,
               label=fr"$T_\mathrm{{max}} = {T_max:g}$ N")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel(r"$\|u\|_\infty$ [N]")
    ax.set_title("Comparison: commanded thrust magnitude vs the actuator bound")
    ax.grid(True, alpha=0.4)
    ax.legend()
    fig.tight_layout()
    fig.savefig(save_path, dpi=150)
    plt.close(fig)


def plot_constraint_violations(results_by_name, save_path):
    """Grouped bar chart of input and state constraint violations per scenario."""
    _ensure_dir(save_path)
    names = list(results_by_name.keys())
    in_counts = [results_by_name[n]["violation_count"] for n in names]
    st_counts = [results_by_name[n].get("state_violation_count", 0) for n in names]
    labels = [_LABELS.get(n, n) for n in names]
    width = 0.4
    x_pos = np.arange(len(names))
    fig, ax = plt.subplots(figsize=(10, 5))
    bars_in = ax.bar(x_pos - width / 2.0, in_counts, width,
                     color="indianred", label=r"input: $\|u\|_\infty > T_\mathrm{max}$")
    bars_st = ax.bar(x_pos + width / 2.0, st_counts, width,
                     color="steelblue", label=r"state: $\max(|\dot x|, |\dot y|) > v_\mathrm{max}$")
    for bar, c in zip(bars_in, in_counts):
        ax.text(bar.get_x() + bar.get_width() / 2.0, bar.get_height() + 0.5,
                str(c), ha="center", va="bottom", fontsize=10)
    for bar, c in zip(bars_st, st_counts):
        ax.text(bar.get_x() + bar.get_width() / 2.0, bar.get_height() + 0.5,
                str(c), ha="center", va="bottom", fontsize=10)
    ax.set_xticks(x_pos)
    ax.set_xticklabels(labels, fontsize=9)
    ax.set_ylabel("Number of violations")
    ax.set_title("Comparison: input and state constraint violations across scenarios")
    ax.grid(True, axis="y", alpha=0.4)
    ax.legend()
    fig.tight_layout()
    fig.savefig(save_path, dpi=150)
    plt.close(fig)


def plot_comparison_velocity(results_by_name, save_path):
    """Per-scenario max(|x_dot|, |y_dot|)(t) vs the velocity bound."""
    _ensure_dir(save_path)
    fig, ax = plt.subplots(figsize=(10, 5))
    v_max = None
    for name, r in results_by_name.items():
        v_norm = np.maximum(np.abs(r["vx"]), np.abs(r["vy"]))
        ax.plot(r["t"], v_norm, linewidth=1.4,
                color=_COLORS.get(name), label=_LABELS.get(name, name))
        if r.get("v_max") is not None:
            v_max = r["v_max"]
    if v_max is not None:
        ax.axhline(v_max, color="black", linestyle="--", alpha=0.7,
                   label=fr"$v_\mathrm{{max}} = {v_max:g}$ m/s")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel(r"$\max(|\dot x|, |\dot y|)$ [m/s]")
    ax.set_title("Comparison: relative-velocity envelope vs the state-constraint bound")
    ax.grid(True, alpha=0.4)
    ax.legend()
    fig.tight_layout()
    fig.savefig(save_path, dpi=150)
    plt.close(fig)
