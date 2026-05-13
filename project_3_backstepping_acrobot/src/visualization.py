import numpy as np
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from pathlib import Path


def _switch_vline(ax, switch_time):
    """Add a vertical line at the controller switch time."""
    if switch_time is not None:
        ax.axvline(
            switch_time, color="green", linestyle=":", alpha=0.7,
            label=f"LQR switch (t={switch_time:.1f} s)",
        )


def plot_state_trajectories(results, output_dir):
    """Joint angles and velocities versus time."""
    t = results["t"]

    fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True)

    ax = axes[0]
    ax.plot(t, results["q1"] - np.pi / 2, label=r"$q_1 - \pi/2$", lw=1.5)
    ax.plot(t, results["q2"], label=r"$q_2$", lw=1.5)
    ax.axhline(0, color="gray", ls="--", alpha=0.4)
    _switch_vline(ax, results["switch_time"])
    ax.set_ylabel("Angle [rad]")
    ax.set_title(
        "Joint Angles: both converge to zero once the upright equilibrium is reached"
    )
    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.3)

    ax = axes[1]
    ax.plot(t, results["dq1"], label=r"$\dot{q}_1$", lw=1.5)
    ax.plot(t, results["dq2"], label=r"$\dot{q}_2$", lw=1.5)
    ax.axhline(0, color="gray", ls="--", alpha=0.4)
    _switch_vline(ax, results["switch_time"])
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Angular velocity [rad/s]")
    ax.set_title(
        "Joint Velocities: oscillations during swing-up, damped after LQR switch"
    )
    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(Path(output_dir) / "state_trajectories.png", dpi=150)
    plt.close(fig)


def plot_control_signal(results, u_max, output_dir):
    """Commanded torque u(t) and actuator output tau2(t) with saturation bounds."""
    t = results["t"]

    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(t, results["u"], lw=1.5, color="tab:purple", label=r"$u$ (commanded)")
    ax.plot(t, results["tau2"], lw=1.5, color="tab:orange", alpha=0.85,
            label=r"$\tau_2$ (actuator output)")
    ax.axhline(u_max, color="red", ls="--", alpha=0.6,
               label=f"Saturation $\\pm${u_max} N·m")
    ax.axhline(-u_max, color="red", ls="--", alpha=0.6)
    _switch_vline(ax, results["switch_time"])
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Torque [N·m]")
    ax.set_title(
        "Control Signal: commanded torque and actuator output through the first-order lag"
    )
    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(Path(output_dir) / "control_signal.png", dpi=150)
    plt.close(fig)


def plot_energy(results, output_dir):
    """Total energy versus time with the upright reference."""
    t = results["t"]

    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(t, results["E"], lw=1.5, label="$E(t)$")
    ax.axhline(
        results["E_upright"], color="red", ls="--",
        label=f"$E_r = {results['E_upright']:.1f}$ J",
    )
    _switch_vline(ax, results["switch_time"])
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Energy [J]")
    ax.set_title(
        "Energy Convergence: backstepping drives $E$ to $E_r$ despite actuator lag"
    )
    ax.legend(loc="right")
    ax.grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(Path(output_dir) / "energy.png", dpi=150)
    plt.close(fig)


def plot_tracking_error(results, output_dir):
    """Weighted state-error norm versus time (log scale)."""
    t = results["t"]
    q1_err = (results["q1"] - np.pi / 2 + np.pi) % (2 * np.pi) - np.pi
    err = (
        np.abs(q1_err) + np.abs(results["q2"])
        + 0.1 * np.abs(results["dq1"]) + 0.1 * np.abs(results["dq2"])
    )

    fig, ax = plt.subplots(figsize=(10, 4))
    ax.semilogy(t, np.maximum(err, 1e-14), lw=1.5, color="tab:orange",
                label="State error")
    _switch_vline(ax, results["switch_time"])
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Weighted state error (log)")
    ax.set_title(
        "Tracking Error: exponential decay after the LQR switch confirms stabilization"
    )
    ax.legend()
    ax.grid(True, alpha=0.3, which="both")

    fig.tight_layout()
    fig.savefig(Path(output_dir) / "tracking_error.png", dpi=150)
    plt.close(fig)


def plot_phase_portrait(results, output_dir):
    """Phase portrait (q1 - pi/2, dq1) colored by time."""
    t = results["t"]
    q1s = results["q1"] - np.pi / 2
    dq1 = results["dq1"]

    points = np.column_stack([q1s, dq1]).reshape(-1, 1, 2)
    segments = np.concatenate([points[:-1], points[1:]], axis=1)

    norm = plt.Normalize(t[0], t[-1])
    lc = LineCollection(segments, cmap="plasma", norm=norm, linewidth=1.5)
    lc.set_array(t[:-1])

    fig, ax = plt.subplots(figsize=(8, 6))
    ax.add_collection(lc)
    ax.scatter(q1s[0], dq1[0], marker="o", c="green", s=120, label="Start", zorder=5)
    ax.scatter(q1s[-1], dq1[-1], marker="x", c="red", s=120, label="End", zorder=5)

    pad_x = (q1s.max() - q1s.min()) * 0.1 + 0.1
    pad_y = (dq1.max() - dq1.min()) * 0.1 + 0.1
    ax.set_xlim(q1s.min() - pad_x, q1s.max() + pad_x)
    ax.set_ylim(dq1.min() - pad_y, dq1.max() + pad_y)

    cb = fig.colorbar(lc, ax=ax)
    cb.set_label("Time [s]")

    ax.set_xlabel(r"$q_1 - \pi/2$ [rad]")
    ax.set_ylabel(r"$\dot{q}_1$ [rad/s]")
    ax.set_title(
        "Phase Portrait: link-1 trajectory spirals toward the upright fixed point"
    )
    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(Path(output_dir) / "phase_portrait.png", dpi=150)
    plt.close(fig)


def plot_actuator_tracking(results, output_dir):
    """Virtual control tau2* vs actuator output tau2 and the tracking error z.

    This is the signature plot of the backstepping design: the outer loop
    produces a desired tau2*, the inner loop drives tau2 to track it, and the
    error z = tau2 - tau2* decays to zero — the inner Lyapunov component.
    """
    t = results["t"]
    z = results["tau2"] - results["tau2_star"]

    fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True)

    ax = axes[0]
    ax.plot(t, results["tau2_star"], lw=1.5, color="tab:blue",
            label=r"$\tau_2^\ast$ (virtual control)")
    ax.plot(t, results["tau2"], lw=1.5, color="tab:orange", alpha=0.85,
            label=r"$\tau_2$ (actuator output)")
    _switch_vline(ax, results["switch_time"])
    ax.set_ylabel("Torque [N·m]")
    ax.set_title(
        "Virtual-Control Tracking: actuator output chases the energy-shaping target"
    )
    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.3)

    ax = axes[1]
    ax.plot(t, z, lw=1.5, color="tab:red",
            label=r"$z = \tau_2 - \tau_2^\ast$")
    ax.axhline(0, color="gray", ls="--", alpha=0.4)
    _switch_vline(ax, results["switch_time"])
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Tracking error [N·m]")
    ax.set_title(
        "Backstepping Error: $z(t)$ is the new Lyapunov component killed by the outer command"
    )
    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(Path(output_dir) / "actuator_tracking.png", dpi=150)
    plt.close(fig)


def plot_comparison_energy(results_by_name, save_path):
    """Overlay E(t) for the P1 baseline vs the P3 backstepping run."""
    fig, ax = plt.subplots(figsize=(11, 5))
    colors = {"p1_baseline": "crimson", "p3_backstep": "seagreen"}
    labels = {
        "p1_baseline": "P1 energy controller (ignores actuator)",
        "p3_backstep": "P3 backstepping (ours)",
    }
    for name, r in results_by_name.items():
        ax.plot(r["t"], r["E"], color=colors.get(name, None),
                linewidth=1.5, label=labels.get(name, name))
    Er = next(iter(results_by_name.values()))["E_upright"]
    ax.axhline(Er, color="black", linestyle="--", alpha=0.6,
               label=fr"$E_r = {Er:.2f}$ J")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Total mechanical energy [J]")
    ax.set_title(
        "Comparison: with actuator dynamics, only backstepping closes the energy gap"
    )
    ax.grid(True, alpha=0.4)
    ax.legend()
    fig.tight_layout()
    fig.savefig(save_path, dpi=150)
    plt.close(fig)


def plot_comparison_error(results_by_name, save_path):
    """Overlay weighted error norm for the P1 baseline vs P3 backstepping."""
    fig, ax = plt.subplots(figsize=(11, 5))
    colors = {"p1_baseline": "crimson", "p3_backstep": "seagreen"}
    labels = {
        "p1_baseline": "P1 energy controller (ignores actuator)",
        "p3_backstep": "P3 backstepping (ours)",
    }
    for name, r in results_by_name.items():
        e0 = (r["q1"] - np.pi / 2 + np.pi) % (2.0 * np.pi) - np.pi
        err = (np.abs(e0) + np.abs(r["q2"])
               + 0.1 * np.abs(r["dq1"]) + 0.1 * np.abs(r["dq2"]))
        ax.semilogy(r["t"], np.maximum(err, 1e-14),
                    color=colors.get(name, None),
                    linewidth=1.5, label=labels.get(name, name))
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Weighted state-error norm (log scale)")
    ax.set_title(
        "Comparison: under actuator dynamics, only backstepping drives the error to zero"
    )
    ax.grid(True, which="both", alpha=0.4)
    ax.legend()
    fig.tight_layout()
    fig.savefig(save_path, dpi=150)
    plt.close(fig)


def generate_all_plots(results, u_max, output_dir):
    """Generate all standard P1 plots plus the backstepping-specific tracking plot."""
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    plot_state_trajectories(results, output_dir)
    plot_control_signal(results, u_max, output_dir)
    plot_energy(results, output_dir)
    plot_tracking_error(results, output_dir)
    plot_phase_portrait(results, output_dir)
    plot_actuator_tracking(results, output_dir)
