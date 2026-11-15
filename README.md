# Self-Stabilizing Robot: Inverted Pendulum on a Cart

A MATLAB/Simulink model of a self-balancing robot, treated as an **inverted pendulum on a cart**, with a discrete **PID controller** that holds the body upright (θ = π) by applying a force to the cart base.

The pendulum was built from first principles in Simulink (equations of motion to blocks), integrated with a cart subsystem, and then stabilized with a controller. A MATLAB script animates the result.

<!-- Add a GIF or screenshot of the animation here -->
<!-- ![Animation](media/animation.gif) -->

---

## Contents

- [System model](#system-model)
- [Project structure](#project-structure)
- [How it was built](#how-it-was-built)
- [Controller](#controller)
- [Parameters](#parameters)
- [Running the project](#running-the-project)
- [Animation](#animation)
- [Results](#results)
- [Requirements](#requirements)
- [Credits](#credits)

---

## System model

| Symbol | Meaning |
|---|---|
| `m` | mass of the robot body (pendulum) |
| `ℓ` | length of the robot body |
| `M` | mass of the robot base (cart) |
| `b` | damping coefficient (friction) |
| `g` | gravitational acceleration |
| `F` | force applied to the cart (control input) |
| `θ` | pendulum angle (`θ = 0` hanging down, **`θ = π` upright**) |
| `x` | horizontal cart position |

Equations of motion:

```
m·ℓ²·θ'' + m·g·ℓ·sin(θ) + m·ℓ·cos(θ)·x'' = 0

(M + m)·x'' + b·x' + m·ℓ·cos(θ)·θ'' − m·ℓ·sin(θ)·(θ')² = F
```

The force `F` acts on the cart only. The pendulum feels it indirectly, through the cart's acceleration `x''`.

Substituting `θ''` into the cart equation gives a form with no dependence on `θ''`, which avoids an algebraic loop in Simulink:

```
x'' = [ F − b·x' + m·ℓ·sin(θ)·(θ')² + m·g·sin(θ)·cos(θ) ] / ( M + m·sin²(θ) )
```

---

## Project structure

```
.
├── ProjectModel.slx          # Top-level model: pendulum + cart + controller
├── PendulumController.slx    # Model reference: discrete PID controller
├── animate_pendulum.m        # Animation script (reads logged x, theta, tout)
├── media/                    # Screenshots and animation
└── README.md
```

> Rename these to match your repository.

---

## How it was built

The project was done in four stages:

1. **Unforced pendulum.** `θ'' = −(g/ℓ)·sin(θ)` built with two integrators, a sine block and a gain. Verified that small angles around 0 oscillate and that θ near π is unstable.
2. **Forced pendulum.** Added the cart's acceleration as an input: `θ'' = −(g·sin(θ) + cos(θ)·x'') / ℓ`.
3. **Integration with the cart.** The pendulum subsystem takes `ddx` and outputs `theta` and `dtheta`. The cart subsystem takes `F`, `theta` and `dtheta` and outputs `ddx`. The pendulum and cart equations were also built by hand and compared against the supplied cart as a cross-check.
4. **Controller.** Started with a PD law on the angle, then extended the supplied discrete PI controller to a full discrete PID.

---

## Controller

The controller is a discrete PID acting on the angle error `e = π − θ`:

```
y(k) = yp(k) + yi(k) + yd(k)

yp(k) = kp · e(k)
yi(k) = yi(k−1) + ki · Ts · e(k)
yd(k) = (kd / Ts) · [ e(k) − e(k−1) ]
```

The derivative term uses a Unit Delay for `e(k−1)`, a Sum for the difference, and a Gain of `kd/Ts`. The output passes through a Saturation block before going to the cart as `F`.

Sample time: `Ts = 0.01 s`.

---

## Parameters

Plant parameters used in the model workspace:

| Parameter | Value |
|---|---|
| `g` | 9.81 m/s² |
| `L` (ℓ) | 1 m |
| `M` | 1 kg |
| `m` | 0.1 kg |
| `b` | 0.1 N·s/m |
| `theta0` | `pi + 0.1` rad (initial angle) |

Controller parameters: `Ts = 0.01`, `kd = 21`, plus `kp` and `ki` as set in `PendulumController.slx`.

---

## Running the project

1. Open `ProjectModel.slx` in MATLAB/Simulink.
2. Make sure the model workspace contains the parameters above.
3. Set the stop time (for example 10 s) and run.
4. Open the scopes to view `theta` and `x`.

To change the starting tilt, edit `theta0`. Try `pi + 0.3` or `pi + 0.5` to see how far the controller can recover.

---

## Animation

Log `x` and `theta` with **To Workspace** blocks (Save format: **Array**, sample time `0.02`), run the model, then run the animation script:

```matlab
run('animate_pendulum.m')
```

The script draws the cart and rod from the logged data, follows the cart with the camera, and can optionally export an MP4 (`saveVideo = true`). Set `L` in the script to match the model.

---

## Results

- With the PD prototype, `θ` settles to **π (≈ 3.1416) in about 1.5 s** from an initial tilt of 0.1 rad, with no visible oscillation.
- The cart position `x` drifts at roughly constant velocity, because the controller regulates only the angle. Removing the drift needs feedback on `x` and `x'` as well, for example full-state feedback with LQR.
- The final discrete PID controller keeps the robot upright (θ = π) in `ProjectModel.slx`.

<!-- Add scope screenshots here -->
<!-- ![Theta](media/theta.png) ![Cart position](media/x.png) -->

---

## Requirements

- MATLAB and Simulink (add your release, for example R2024a)
- No additional toolboxes are needed for the model itself

---

## Credits

This project is the final project of a MATLAB/Simulink course. The cart subsystem and the PI controller template were provided by the course (© The MathWorks, Inc.). The pendulum model, the cart cross-check, the derivative term, the animation script and the analysis are my own work.

> If you are not sure the course materials may be redistributed, leave the supplied `.slx` files out of the repository and describe them here instead.
