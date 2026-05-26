"""Numerical configuration for Project 4 (spacecraft rendezvous MPC).

Three parameter groups -- physical (orbit, mass, thrust), MPC (horizon,
weights, terminal set), and simulation (initial state, horizon,
disturbance) -- each carried by a ``@dataclass`` so they can be
forwarded to ``ClohessyWiltshire`` and the controllers as plain Python
objects.
"""

from dataclasses import dataclass, field
import numpy as np


@dataclass
class PhysicalParams:
    """Orbital and chaser-mass constants."""

    mu: float = 3.986004418e14    # Earth GM [m^3 / s^2]
    altitude: float = 400e3        # orbital altitude AGL [m]
    R_earth: float = 6378e3        # Earth equatorial radius [m]
    m: float = 5000.0              # chaser mass [kg]  (~Dragon-class)
    T_max: float = 100.0           # per-axis thrust limit [N]   (~RCS quad)
    Ts: float = 1.0                # sampling period [s]

    @property
    def a(self) -> float:
        """Reference orbit semi-major axis (circular: a = R + h)."""
        return self.R_earth + self.altitude

    @property
    def n(self) -> float:
        """Mean orbital rate n = sqrt(mu / a^3)  [rad/s]."""
        return float(np.sqrt(self.mu / self.a ** 3))

    @property
    def period(self) -> float:
        """Orbital period T = 2 pi / n  [s]."""
        return float(2.0 * np.pi / self.n)


@dataclass
class MPCParams:
    """MPC controller parameters."""

    N: int = 30                                       # prediction horizon
    Q_diag: tuple = (10.0, 10.0, 1.0, 1.0)            # state weights
    R_diag: tuple = (0.1, 0.1)                        # input weights
    rho: float = 1.0                                  # terminal set level
    cone_angle_deg: float = 10.0                      # docking-cone half-angle
    cone_active_range: float = 50.0                   # cone active when |s|<=this [m]
    use_terminal_set: bool = True                     # toggle for ablation runs

    @property
    def Q(self) -> np.ndarray:
        return np.diag(self.Q_diag).astype(float)

    @property
    def R(self) -> np.ndarray:
        return np.diag(self.R_diag).astype(float)


@dataclass
class SimParams:
    """Simulation parameters."""

    x0: tuple = (100.0, 100.0, 0.0, 0.0)  # initial relative state [m, m, m/s, m/s]
    t_final: float = 300.0                 # simulation horizon [s]
    # Constant additive process noise in the acceleration channel (modelling
    # differential drag, third-body, etc).  Applied to (x_ddot, y_ddot).
    disturbance: tuple = (0.0, 5e-4)
    rng_seed: int = 0
