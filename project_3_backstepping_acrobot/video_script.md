# Pitch Script - ~8 min, 5 Speakers

---

## Mariya - The System (0:00 - 1:30)

**[Show: title slide, then acrobot system slide]**

Hello, we are Mariya, Safina, Hajira, Ilya, and Mikhail. This is our third project: backstepping swing-up and stabilization of the acrobot with actuator dynamics.

The mechanical acrobot is the same two-link system from Project 1. The top joint is passive, and the only physical torque acts at the elbow.

The difference is that we do not command that elbow torque directly anymore. We command `u`, and the actuator output `tau-two` follows `u` through a first-order lag with time constant `T-a`.

So the state has one extra coordinate: `tau-two` itself. In Project 1 the state had four variables: the two angles and the two angular velocities. In Project 3 the state is five-dimensional: the same four mechanical variables plus the actuator torque.

Our goal is still to start near the hanging position, swing the acrobot up to the unstable upright position, and then keep it there. But now the controller must compensate for actuator lag while staying inside the same fifty newton-meter command limit.

---

## Safina - The Math and Virtual Control (1:30 - 3:15)

**[Show: equations of motion slide]**

The mechanical equations are unchanged from Project 1. We still have the mass matrix `M`, the Coriolis terms, and the gravity vector. The first joint is passive, so the first torque component is always zero.

The new equation is the actuator equation: `T-a` times the derivative of `tau-two` equals `u` minus `tau-two`. This means `tau-two` is the torque that the robot actually receives, while `u` is only the command we send to the actuator.

This distinction is the reason we need backstepping.

**[Show: backstepping idea slide]**

Backstepping starts from the controller we already know. Project 1 gives us an energy Lyapunov function, called `V-zero`, and a torque `tau-two-star` that makes the derivative of that Lyapunov function negative.

If we could apply `tau-two-star` directly, the energy would move toward the upright target energy. But in this project we cannot apply it directly. The actuator torque is a state, so `tau-two-star` becomes a virtual control. It is the torque we want the actuator output to track.

So the outer loop is the Project 1 energy controller, and the inner loop is an actuator-tracking controller. The inner loop must make the actual `tau-two` follow the desired `tau-two-star` fast enough that the outer energy proof still works.

---

## Hajira - Backstepping Law and Switching (3:15 - 5:00)

**[Show: backstepping control law slide]**

The virtual torque is exactly the Project 1 energy-shaping formula. We define the tracking error `z` as actual torque minus virtual torque: `z = tau-two - tau-two-star`.

The command law has three parts.

First, we ask for `tau-two-star`. Second, we add a feedforward term, `T-a` times the derivative of `tau-two-star`. This predicts where the desired torque is moving after one actuator time constant. Third, we subtract a proportional correction on `z`.

When we substitute this command into the actuator equation, the result is a simple stable first-order error equation. The tracking error `z` decays exponentially. With our parameters, the inner time constant is about four point four milliseconds.

The derivative of `tau-two-star` is computed using the chain rule along the actual trajectory. The accelerations come from the real actuator output, not the virtual torque. The gradient is evaluated by central finite differences, which avoids a long hand-derived expression.

**[Show: LQR and switching slide]**

For the augmented Lyapunov function, we add one half `z` squared to the Project 1 Lyapunov function. The inner loop drives this extra error down, and the outer loop recovers the Project 1 swing-up behavior.

Near the top, we still switch to LQR. In practice, a full five-dimensional Riccati gain was too aggressive under saturation, so we use the Project 1 mechanical LQR and add a small damping gain on the actuator state.

The switch happens when the weighted mechanical error drops below zero point zero four. In the simulation, this happens at about seven point eight seconds.

---

## Ilya - Code and Design (5:00 - 6:30)

**[Show: implementation slide]**

The implementation keeps the same modular structure as Project 1.

The system file contains the five-dimensional plant dynamics, including actuator lag. The controller file contains the virtual torque, the backstepping command, the LQR controller, and the Project 1 baseline used for comparison.

The simulation file handles the one-way switch from backstepping to LQR. The visualization file generates the standard plots plus the Project 3-specific actuator-tracking plot and comparison plots.

**[Show: algorithm listing]**

The algorithm is:

First, build the plant with the actuator equation.

Second, verify that the inherited `k-D` solvability condition is satisfied.

Third, compute the Project 1 virtual torque `tau-two-star`.

Fourth, estimate its time derivative using the chain rule and finite differences.

Fifth, command `u = tau-two-star + T-a dot tau-two-star - k-z T-a z`.

Sixth, switch to LQR when the weighted error falls below the threshold.

Finally, generate the plots, comparison overlays, and animation.

To reproduce the results, run `pip install -r requirements.txt`, then `python -m src.main`. For a faster run without the GIF, use `python -m src.main --no-anim`.

---

## Mikhail - Results (6:30 - 8:00)

**[Play: animation on screen]**

Here is the animation. The robot starts near the hanging position and swings up. During swing-up, the backstepping controller makes the actuator output track the virtual torque. At about seven point eight seconds, the state enters the LQR region and the stabilizer takes over.

**[Show: baseline comparison slide]**

These comparison plots show why the extra backstepping loop matters. The naive Project 1 baseline commands the virtual torque directly as `u` and ignores actuator lag.

It still injects energy on average, but the lag distorts the pass through the upright. The weighted error bottoms out around a few tenths, so the LQR switch never fires.

With backstepping, the actuator output tracks the virtual control closely, the trajectory reaches the switch threshold, and LQR can take over.

**[Show: actuator-tracking plot]**

This is the signature plot for Project 3. The top panel compares `tau-two-star`, which is the torque requested by the outer energy controller, with the actual actuator output `tau-two`.

The bottom panel shows the tracking error `z`. During swing-up, `z` stays very small, which is exactly what the backstepping inner loop was designed to do.

**[Show: closed-loop response slide]**

The tracking-error plot shows the transition from swing-up to stabilization. After the switch, the conservative LQR keeps the state close to upright. It is not as tight as the direct-torque Project 1 result, but it is stable under actuator and saturation constraints.

The control plot overlays the command and actuator output. They are almost indistinguishable during most of swing-up because the actuator time constant is small.

To summarize: Project 3 takes the energy controller from Project 1 and uses it as a virtual control inside a backstepping design. The new inner loop makes the real actuator torque track that virtual torque, which fixes the actuator-lag failure mode.

The method reaches the LQR basin and stabilizes near upright, while the naive Project 1 baseline does not switch under the same actuator dynamics. The main limitations are the conservative LQR residual and the sensitivity to larger actuator time constants.

Thank you. We are happy to take questions.
