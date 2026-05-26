"""2D rendezvous animation: chaser approaching the target in the LVLH frame.

A single ``make_gif(result, save_path, ...)`` call renders the scene at a
configurable frame rate.  The chaser is drawn as a small square with a
fading position trail; the target sits at the origin as a star; the
currently commanded thrust is shown as an arrow whose length scales with
``|u|``.
"""

import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.patches import Rectangle


def make_gif(result, save_path, fps=20, downsample=None, trail_length=80):
    """Render a 2D animation of the chaser approaching the target.

    Parameters
    ----------
    result : dict
        Output of ``simulation.simulate``.
    save_path : str
        Output path (must end in ``.gif``).
    fps : int
        Animation frame rate.
    downsample : int or None
        Subsample factor on the trajectory.  None auto-picks so the
        rendered GIF lasts roughly 8-10 seconds.
    trail_length : int
        Number of past samples drawn as a fading trail.
    """
    parent = os.path.dirname(save_path)
    if parent:
        os.makedirs(parent, exist_ok=True)
    x = np.asarray(result["x"])
    y = np.asarray(result["y"])
    ux = np.asarray(result["ux"])
    uy = np.asarray(result["uy"])
    t = np.asarray(result["t"])
    T_max = float(result["T_max"])

    n = len(x)
    if downsample is None:
        downsample = max(1, n // (fps * 10))
    idx = list(range(0, n, downsample))
    if idx[-1] != n - 1:
        idx.append(n - 1)

    pad = 5.0
    xmin, xmax = float(min(x.min(), -pad) - pad), float(max(x.max(), pad) + pad)
    ymin, ymax = float(min(y.min(), -pad) - pad), float(max(y.max(), pad) + pad)
    span = max(xmax - xmin, ymax - ymin)
    cx, cy = 0.5 * (xmin + xmax), 0.5 * (ymin + ymax)
    xmin, xmax = cx - 0.5 * span, cx + 0.5 * span
    ymin, ymax = cy - 0.5 * span, cy + 0.5 * span

    fig, ax = plt.subplots(figsize=(7, 7))
    ax.set_xlim(xmin, xmax)
    ax.set_ylim(ymin, ymax)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("$x$ -- radial [m]")
    ax.set_ylabel("$y$ -- along-track [m]")
    ax.set_title("Rendezvous in the target's LVLH frame")
    ax.grid(True, alpha=0.3)

    # Target marker at origin
    ax.scatter(0.0, 0.0, color="red", s=260, marker="*",
               edgecolors="black", linewidths=1.0, zorder=4, label="target")

    # Static trail (will be redrawn per frame as a fading line)
    trail_line, = ax.plot([], [], color="seagreen", alpha=0.7, linewidth=1.6,
                          label="chaser trail")
    chaser_size = max(0.02 * span, 0.5)
    chaser_patch = Rectangle((x[0] - chaser_size / 2, y[0] - chaser_size / 2),
                             chaser_size, chaser_size,
                             facecolor="navy", edgecolor="white", linewidth=1.0,
                             zorder=5)
    ax.add_patch(chaser_patch)

    # Thrust arrow (drawn as an annotation for redraw flexibility)
    thrust_arrow = ax.annotate("", xy=(x[0], y[0]), xytext=(x[0], y[0]),
                               arrowprops=dict(arrowstyle="->", color="darkorange",
                                                lw=1.5))
    time_text = ax.text(0.02, 0.97, "", transform=ax.transAxes,
                        fontsize=11, va="top",
                        bbox=dict(boxstyle="round", facecolor="white", alpha=0.8))
    range_text = ax.text(0.02, 0.91, "", transform=ax.transAxes,
                         fontsize=11, va="top",
                         bbox=dict(boxstyle="round", facecolor="white", alpha=0.8))
    ax.legend(loc="lower right")

    arrow_scale = 0.15 * span / max(T_max, 1e-6)  # arrow length scales with thrust

    def update(k):
        i0 = max(0, k - trail_length)
        trail_line.set_data(x[i0:k + 1], y[i0:k + 1])
        chaser_patch.set_xy((x[k] - chaser_size / 2, y[k] - chaser_size / 2))
        if k < len(ux):
            ax_pos = (x[k], y[k])
            tip = (x[k] + ux[k] * arrow_scale, y[k] + uy[k] * arrow_scale)
            thrust_arrow.xy = tip
            thrust_arrow.set_position(ax_pos)
        time_text.set_text(f"t = {t[k]:6.1f} s")
        range_text.set_text(f"range = {np.sqrt(x[k] ** 2 + y[k] ** 2):6.2f} m")
        return trail_line, chaser_patch, thrust_arrow, time_text, range_text

    ani = FuncAnimation(fig, update, frames=idx, interval=1000 / fps, blit=False)
    writer = PillowWriter(fps=fps)
    ani.save(save_path, writer=writer, dpi=110)
    plt.close(fig)
