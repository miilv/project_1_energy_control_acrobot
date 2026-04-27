from dataclasses import dataclass


@dataclass
class PhysicalParams:
    """Physical parameters of the acrobot."""

    m1: float = 1.0    # mass of link 1 [kg]
    m2: float = 1.0    # mass of link 2 [kg]
    l1: float = 1.0    # length of link 1 [m]
    l2: float = 2.0    # length of link 2 [m]
    lc1: float = 0.5   # distance to centre of mass of link 1 [m]
    lc2: float = 1.0   # distance to centre of mass of link 2 [m]
    I1: float = 0.083  # moment of inertia of link 1 [kg*m^2]
    I2: float = 0.33   # moment of inertia of link 2 [kg*m^2]
    g: float = 9.8     # gravitational acceleration [m/s^2]


@dataclass
class FrictionParams:
    """Plant-side viscous friction at joint 2.

    Treated as a single unknown scalar in the adaptive setting.
    The controller has its own estimate ``b2_hat`` that lives in
    AdaptiveParams.
    """

    b2_true: float = 1.5  # true (plant) viscous friction at joint 2 [N*m*s/rad]


@dataclass
class ControlParams:
    """Energy-based swing-up + LQR tuning.

    The energy-shaping gains are taken from Xin & Kaneda (2007); kD must
    exceed the analytical solvability bound (~35.74 for the textbook
    parameter set).  Friction adds dissipation to phase 1, so the LQR
    handover threshold is a touch looser than in Project 1 (0.06 vs 0.04).

    Q and R are chosen so that the resulting LQR gain is large enough to
    catch the residual link-1 velocity at handover.  The Q values were
    tuned upward from Project 1 (10/10/1/1) until the controller reliably
    stabilises after the swing-up under the friction-aware linearisation.
    """

    kD: float = 35.8     # energy-shaping damping gain
    kP: float = 61.2     # energy-shaping position gain
    kV: float = 66.3     # energy-shaping velocity damping gain
    u_max: float = 50.0  # torque saturation limit [N*m]
    switch_threshold_p1: float = 0.04  # frictionless P1 threshold
    switch_threshold: float = 0.06     # friction-tolerant threshold
    Q_diag: tuple = (15.0, 15.0, 2.0, 2.0)  # LQR state cost diagonal
    R_val: float = 1.0  # LQR input cost


@dataclass
class AdaptiveParams:
    """Parameters of the certainty-equivalence friction adaptation law.

    Update:    d(b_hat)/dt = -gamma * (E - E_r) * dq2^2
    Initial:   b_hat(0) = b2_hat_0

    A small but non-zero gamma is essential.  Too small and adaptation
    cannot keep up with the swing-up transient; too large and b_hat
    overshoots into negative values and drives the controller unstable.
    The chosen value was tuned within a narrow range on the textbook
    parameter set.
    """

    gamma: float = 0.005    # adaptation gain
    b2_hat_0: float = 0.0   # initial parameter estimate


@dataclass
class SimParams:
    """Simulation parameters."""

    t_final: float = 30.0  # simulation duration [s]
    dt: float = 0.005      # output time step [s]
    # Initial state [q1, q2, dq1, dq2].  A small q2 offset breaks the
    # symmetry of the spurious LaSalle invariant set {q2 = 0, dq2 = 0,
    # link 1 spinning} and lets the trajectory escape into the upright
    # basin in finite time.
    x0: tuple = (-1.4, 0.001, 0.0, 0.0)
    t_final_swingup: float = 30.0  # gives swing-up enough time under friction
