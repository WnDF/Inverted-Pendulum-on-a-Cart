# Modelling, Identification, Estimation and Control of an Inverted Pendulum on a Cart

A complete control pipeline of the INTECO pendulum on a cart rig, covering dynamical
modelling by the Lagrangian method, nonlinear grey-box parameter identification, state estimation
with a Kalman filter, and the design of LQR, LQI and MPC controllers for the regulation,
reference tracking and swing-up problems. The implementation is in MATLAB and Simulink.

![Simulink model](Plots/pendulum_cart.png)

---

## Problem statement

The system considered is the INTECO pendulum on a cart rig. A cart translates along a rail of
length 1.8 m and is actuated by a DC motor driven by PWM command. A rigid pendulum is mounted on a pivot fixed to the cart and is free to rotate through a full revolution. Two incremental encoders provide measurements of the cart position and the pendulum angle, and no other quantity is measured.

Three control objectives are considered, namely (i) regulation of the pendulum about its unstable
upright equilibrium, (ii) tracking of a cart position reference while the pendulum is held upright, and (iii) swing up of the pendulum from the stable downward equilibrium to the upright
equilibrium. Each objective must be achieved subject to the finite rail length, the PWM saturation limit, and the requirement that the pendulum angle remain within the region in which the linearised model is a valid approximation of the dynamics.

The actuation is indirect. The motor applies a force to the cart only, and the pendulum is
affected solely through the reaction at the cart. A displacement of the cart therefore requires a transient pendulum deflection, which is itself bounded, so every cart manoeuvre must be arranged such that the deflection is introduced and subsequently removed within the admissible region.

The plant is a physical system and is subject to Coulomb and viscous friction on the rail, viscous friction at the pivot, a constant bias force attributed to rail inclination and cable drag, and back-EMF damping in the motor. These effects are retained in the model. The two velocity states are not measured and are reconstructed by the state estimator.

**Physical parameters** (nominal values, used as initial estimates for identification).

| Symbol | Meaning | Value |
|---|---|---|
| $m$ | equivalent translating mass (cart and pendulum) | 0.872 kg |
| $\ell$ | pivot to pendulum centre of mass | 0.011 m |
| $J_p$ | pendulum inertia about the pivot | 2.92 × 10⁻³ kg·m² |
| $f_c$ | cart viscous friction | 0.5 N·s/m |
| $f_s$ | Coulomb (static) friction | 1.203 N |
| $f_p$ | pivot viscous friction | 6.65 × 10⁻⁵ N·m·s/rad |
| $p_1$ | PWM-command to force gain | 9.4 N |
| $p_2$ | back-EMF damping | −0.548 N·s/m |
| $g$ | gravitational acceleration | 9.81 m/s² |
| $R_l$ | rail length | 1.8 m |
| $u_{\max}$ | PWM command magnitude limit | 0.5 |

**Model and simulation settings.**

| Setting | Value |
|---|---|
| Sample period $T_s$ | 0.01 s |
| Sample rate | 100 Hz |
| Discretisation | zero-order hold (`c2d`) |
| Friction-smoothing constant $\alpha$ | 10 |
| Control-design equilibrium | upright, $x_e = [0,\ \pi,\ 0,\ 0]^\top$ |

---

## Dynamical model

### Generalised coordinates and states

The cart displacement $x$ and the pendulum angle $\theta$ are taken as the generalised coordinates. The state and measurement vectors are

$$
x = [x_1,\ x_2,\ x_3,\ x_4]^\top = [x,\ \theta,\ \dot{x},\ \dot{\theta}]^\top,
\qquad
y = \begin{bmatrix} 1 & 0 & 0 & 0 \\ 0 & 1 & 0 & 0 \end{bmatrix} x .
$$

### Actuator model

The armature electrical dynamics are assumed fast relative to the mechanical dynamics, so that
$L\,\dot I \approx 0$. With the PWM to voltage relation $V = \alpha u$ and the experimentally
observed sign convention, in which a positive command produces motion in the negative $x$ direction, the force transmitted to the cart is

$$
F = -p_1 u + p_2 \dot{x},
\qquad
p_1 = \frac{K_f \alpha}{R},
\qquad
p_2 = -\frac{K_f K_e}{R} < 0 .
$$

The term $-p_1 u$ represents the commanded actuation and $p_2\dot{x}$ represents dissipative
back-EMF damping.

### Lagrangian formulation

The pendulum centre of mass is located at $(x + \ell\sin\theta,\ -\ell\cos\theta)$. The kinetic
energy, potential energy and Lagrangian are

$$
T = \tfrac12 m \dot{x}^2 + m\ell\,\dot{x}\dot{\theta}\cos\theta + \tfrac12 J_p \dot{\theta}^2,
\qquad
V = mg\ell\cos\theta,
\qquad
\mathcal{L} = T - V .
$$

Application of the Euler-Lagrange equations with the generalised nonconservative forces

$$
Q_x = F - f_c \dot{x} - f_s \tanh(\alpha \dot{x}) + F_d,
\qquad
Q_\theta = -f_p \dot{\theta},
$$

in which $F_d$ is the constant bias force, yields the coupled equations of motion

$$
a_2\,\ddot{x} + \ddot{\theta}\cos\theta = k_1 u + \dot\theta^2\sin\theta - k_2\dot{x} - k_s\tanh(\alpha\dot{x}) + k_d,
$$

$$
\ddot{x}\cos\theta + a_1\,\ddot{\theta} = g\sin\theta - k_3\dot{\theta}.
$$

### Nonlinear state-space representation

Solving the two equations above for the accelerations gives the nonlinear state-space model
implemented in `SysID/invPend_model.m`,

$$
\ddot{x} = \frac{a_1\!\left(k_1 u - \dot\theta^2\sin\theta - k_2\dot{x} - k_s\tanh(\alpha\dot{x}) + k_d\right) - \left(g\sin\theta + k_3\dot\theta\right)\cos\theta}{a_1 a_2 - \cos^2\theta},
$$

$$
\ddot{\theta} = \frac{\left(k_1 u - \dot\theta^2\sin\theta - k_2\dot{x} - k_s\tanh(\alpha\dot{x}) + k_d\right)\cos\theta - a_2\!\left(g\sin\theta + k_3\dot\theta\right)}{a_1 a_2 - \cos^2\theta}.
$$

The physical parameters are grouped into the structured vector estimated during identification,

$$
\theta_p = [a_1,\ a_2,\ k_1,\ k_2,\ k_3,\ k_s,\ k_d]^\top,
$$

$$
a_1 = \frac{J_p}{m\ell},\quad
a_2 = \frac{1}{\ell},\quad
k_1 = -\frac{p_1}{m\ell},\quad
k_2 = \frac{f_c - p_2}{m\ell},\quad
k_3 = \frac{f_p}{m\ell},\quad
k_s = \frac{f_s}{m\ell},\quad
k_d = \frac{F_d}{m\ell}.
$$

### Linearisation

The model is linearised about an equilibrium $(x_e, u_e)$, where $u_e = -k_d/k_1$ compensates the
bias term. The downward equilibrium $x_e = [0,0,0,0]^\top$ is used for estimator tuning, since
open loop excitation of the rig is more convenient in that configuration, and the upward
equilibrium $x_e = [0,\pi,0,0]^\top$ is used for controller design. Continuous time linearisation
followed by ZOH discretisation at 100 Hz yields the design models. For the identified
parameters the upward model is

$$
A_\text{up} \approx
\begin{bmatrix}
0 & 0 & 1 & 0\\
0 & 0 & 0 & 1\\
0 & -0.2444 & -11.3455 & -0.0002\\
0 & -27.7735 & -31.3399 & -0.0266
\end{bmatrix},
\qquad
B_\text{up} \approx
\begin{bmatrix} 0 \\ 0 \\ -18.0449 \\ -49.8458 \end{bmatrix}.
$$

Both discretised pairs were verified to be controllable and observable using `ctrb` and `obsv`.

In every closed loop result reported below, the linear controllers act on the full nonlinear plant (the nonlinear equations in simulation and the physical rig on the bench), so the reported
behaviour includes the model mismatch rejected by the closed loop.

---

## System identification

Because the model structure is derived from first principles, a nonlinear grey-box approach is
adopted. The equations of motion are retained and only the unknown entries of $\theta_p$ are
estimated, using `idnlgrey` and `nlgreyest`. Estimation is carried out about the downward
equilibrium, and the identified nonlinear model is subsequently linearised as required. The
procedure is organised in three stages, each initialised from the preceding one in order to reduce the risk of convergence to a poor local minimum. In every stage the measured record is partitioned into 75 % for estimation and 25 % for validation.

### Pendulum dynamics (Stage 1)

The cart is mechanically clamped and the pendulum is released from an initial angle of
approximately 20 to 30° and allowed to oscillate freely. The cart acceleration term then vanishes
and only $a_1$ and $k_3$ are identifiable.

<p align="center">
  <img src="Plots/Pendulum%20Dynamics%20ID/pendulumDynamicsData.png" width="32%"/>
  <img src="Plots/Pendulum%20Dynamics%20ID/pendulumDynamicsTraining.png" width="32%"/>
  <img src="Plots/Pendulum%20Dynamics%20ID/pendulumDynamicsValidation.png" width="32%"/>
</p>

From left to right, the measured free oscillation record, the estimation fit (99.58 %), and the
validation fit (99.48 %).

### Cart dynamics (Stage 2)

The pendulum centre of mass is displaced to the pivot so that the rotational dynamics decouple,
and the cart is driven from one rail end to the other with a PWM sequence that includes unactuated intervals to excite the friction characteristic. Parameters $k_1$, $k_2$, $k_s$ and $k_d$ are identified.

<p align="center">
  <img src="Plots/Cart%20Dynamics%20ID/cartDynamicsData.png" width="32%"/>
  <img src="Plots/Cart%20Dynamics%20ID/cartDynamicsTraining.png" width="32%"/>
  <img src="Plots/Cart%20Dynamics%20ID/cartDynamicsValidation.png" width="32%"/>
</p>

From left to right, the PWM excitation with the cart response, the estimation fit (94.24 %), and
the validation fit (95.64 %).

### Coupled cart-pendulum dynamics (Stage 3)

Both bodies are free to move. All seven parameters are restimated under the full coupled
nonlinear model, with $a_2 = 1/\ell$ fixed by geometry and the remaining parameters constrained to within ±25 % of their stage 1 and stage 2 values.

<p align="center">
  <img src="Plots/Coupled%20Dynamics%20ID/coupledDynamicsData.png" width="32%"/>
  <img src="Plots/Coupled%20Dynamics%20ID/coupledDynamicsTraining.png" width="32%"/>
  <img src="Plots/Coupled%20Dynamics%20ID/coupledDynamicsValidation.png" width="32%"/>
</p>

From left to right, the coupled excitation with the position and angle responses, the estimation
fit (93.9 % cart, 77.6 % pendulum), and the validation fit (96.8 % and 83.3 %).

### Identified parameters

The parameter vector obtained from stage 3, used by all estimators and controllers in the
repository (`main.m`), is given below.

| $a_1$ | $a_2$ | $k_1$ | $k_2$ | $k_3$ | $k_s$ | $k_d$ |
|---|---|---|---|---|---|---|
| 0.362015 | 113.636338 | −2000.714368 | 238.679879 | 0.009382 | 101.924343 | 15.346870 |

---

## State estimation

The controllers require the full state vector, whereas only $x$ and $\theta$ are measured. A
discrete time Kalman filter based on the downward linearised model is used to reconstruct the two
velocity states. The measured angle is unwrapped by `angleNormalization` prior to the update step, and the filter operates in coordinates defined relative to the equilibrium. The process and
measurement covariances were tuned through open-loop experiments with manual excitation, giving

$$
Q = \operatorname{diag}(10^{-4},\ 10^{-2},\ 1.5\times10^{2},\ 1.5\times10^{2}),
\qquad
R = \operatorname{diag}(10^{-6},\ 10^{-6}).
$$

The small measurement covariance reflects a high confidence in the encoder signals, while the
large process noise terms on the velocity states reduce the reliance of the filter on the model
for the unmeasured quantities.

<p align="center">
  <img src="Plots/Kalman%20Filter%20Estimates/positionEstimate.png" width="48%"/>
  <img src="Plots/Kalman%20Filter%20Estimates/angleEstimate.png" width="48%"/>
</p>
<p align="center">
  <img src="Plots/Kalman%20Filter%20Estimates/velocityEstimate.png" width="48%"/>
  <img src="Plots/Kalman%20Filter%20Estimates/angularVelocityEstimate.png" width="48%"/>
</p>

The cart position (top left) and the pendulum angle (top right) are reconstructed almost exactly,
since they are measured directly and only the encoder noise must be rejected. The cart velocity
(bottom left) and the pendulum angular velocity (bottom right) are not measured; the corresponding estimates follow the low pass filtered finite difference references closely, with small lag and
residual noise.

---

## Controller design

### LQR/LQI

A discrete time state feedback gain is computed with `dlqr` for the upward model. The regulator
(LQR) addresses objective 1 and the integral formulation (LQI) addresses objective 2. The
weighting matrices are

| | $x$ | $\theta$ | $\dot{x}$ | $\dot{\theta}$ | $\xi$ (integral) | $R$ |
|---|---|---|---|---|---|---|
| LQR $Q$ | 80 | 100 | 75 | 100 | n/a | 50 |
| LQI $Q$ | 75 | 100 | 75 | 100 | 250 | 75 |

The applied command is $u = u_e - K\hat{x}$, followed by a saturation block for hardware
protection. In the LQI formulation the plant is augmented with the integral of the cart position
error. A velocity reference obtained by differentiating the position reference is added to the
velocity error feedback term rather than introducing a second integrator, which would render the
augmented pair uncontrollable.

### MPC

An output feedback MPC is implemented with the Simulink MPC block, using the Kalman filter above
as a custom state estimator. The prediction horizon is $N = 40$ (0.4 s), the control horizon is
10, the terminal weight is the solution of the discrete algebraic Riccati equation, and the
nominal input is $u_e$. The state and input constraints are enforced within the quadratic program.

| | $x$ | $\theta$ | $\dot{x}$ | $\dot{\theta}$ | $\Delta u$ |
|---|---|---|---|---|---|
| weight | 95 | 100 | 30 | 10 | 12.5 |

| constraint | $u$ | $x$ (regulation) | $x$ (tracking) | $\theta$ |
|---|---|---|---|---|
| bound | ±0.75 | ±0.02 m | ±0.8 m | ±0.2 rad |

The ±0.8 m position bound maintains a margin to the ±0.9 m rail ends, and the ±0.2 rad angle bound confines the state to the region in which the linear prediction model is accurate. The output constraints are treated as soft, so that a large transient relaxes them rather than rendering the program infeasible.

---

## Results

The three objectives are reported below for both controllers. In each pair the left panel shows
the cart position and the right panel shows the pendulum angle, with the upright equilibrium at
zero. All runs use the nonlinear plant with the Kalman filter in the loop, and several include
external disturbances applied manually to the pendulum.

### Regulation (Objective 1)

Under LQR the pendulum angle is maintained within approximately ±0.01 rad (±0.57°) and the cart
within approximately ±5 cm, and applied disturbances are rejected. In the absence of integral
action the cart does not return to its original position after a disturbance.

<p align="center">
  <img src="Plots/LQR%20Results/LQR_position.png" width="48%"/>
  <img src="Plots/LQR%20Results/LQR_angle.png" width="48%"/>
</p>

Under MPC the angle and cart bands are comparable (±0.01 rad and ±2 cm), with the position
constraint active, and the angle response is smoother between disturbances.

<p align="center">
  <img src="Plots/MPC%20Results/MPC_Position.png" width="48%"/>
  <img src="Plots/MPC%20Results/MPC_Angle.png" width="48%"/>
</p>

### Reference tracking (Objective 2)

Both controllers track a sinusoidal cart-position reference of amplitude 0.70 m and frequency
0.065 Hz while regulating the pendulum.

Under LQI the reference is tracked accurately after an initial transient, which is caused by the
large initial mismatch between the velocity reference and the measured cart velocity. The pendulum angle remains near ±0.01 rad, with larger excursions at the disturbance instants.

<p align="center">
  <img src="Plots/LQR%20Results/LQI_position.png" width="48%"/>
  <img src="Plots/LQR%20Results/LQI_angle.png" width="48%"/>
</p>

Under MPC the tracking accuracy is similar, with a slightly larger error near the extrema of the
reference, reflecting the simultaneous penalisation of tracking error, control effort and the
angle constraint. The angle response is smoother.

<p align="center">
  <img src="Plots/MPC%20Results/MPC_Position_RefTrack.png" width="48%"/>
  <img src="Plots/MPC%20Results/MPC_Angle_RefTrack.png" width="48%"/>
</p>

### Swing-up (Objective 3)

The pendulum is swung up from the downward equilibrium by the chirp excitation. The angle record
alternates between ±π because the upward-linearised model wraps the measured angle. Capture occurs at approximately 6 s, after which the stabilising controller is engaged.

**LQR stabiliser**

<p align="center">
  <img src="Plots/LQR%20Results/LQR_SwingUp_Position.png" width="48%"/>
  <img src="Plots/LQR%20Results/LQR_SwingUp_Angle.png" width="48%"/>
</p>

**MPC stabiliser**

<p align="center">
  <img src="Plots/MPC%20Results/MPC_swingup_position.png" width="48%"/>
  <img src="Plots/MPC%20Results/MPC_swingup_angle.png" width="48%"/>
</p>

---

## Comparison of LQI and MPC

| Aspect | LQI | MPC |
|---|---|---|
| Sinusoidal position tracking | accurate, faster transient, larger overshoot | accurate, smoother, small lag near the extrema |
| Angle regulation during tracking | within ±0.01 rad, larger oscillation | within ±0.01 rad, smaller oscillation |
| Constraint handling | post-hoc saturation and switching logic | rail, PWM and angle bounds enforced in the optimisation |
| Swing-up capture time | approximately 6 s | approximately 6 s |
| Cart motion after swing-up hand-over | larger excursions, up to approximately 0.29 m | more bounded, approximately 0.1 m |
| Disturbance rejection | faster, larger overshoot | slower, more conservative |

**Reference tracking**, both controllers overlaid, with cart position on the left and pendulum
angle on the right.

<p align="center">
  <img src="Plots/LQR%20vs%20MPC%20Comparison/LQRvsMPC_position.png" width="48%"/>
  <img src="Plots/LQR%20vs%20MPC%20Comparison/LQRvsMPC_angle.png" width="48%"/>
</p>

**Swing-up**, both controllers overlaid.

<p align="center">
  <img src="Plots/LQR%20vs%20MPC%20Comparison/LQRvsMPC_positionSwingUp.png" width="48%"/>
  <img src="Plots/LQR%20vs%20MPC%20Comparison/LQRvsMPC_angleSwingUp.png" width="48%"/>
</p>

The LQI controller is preferable where implementation simplicity and rapid tracking are the
priorities. The MPC controller is preferable where the rail limit, the PWM limit and the
pendulum angle bound must be respected simultaneously, since these constraints are incorporated in the optimisation rather than enforced by post-hoc saturation.

---

## Repository layout

| Path | Purpose |
|---|---|
| `main.m` | Loads the identified parameters, builds the linearised models, computes the LQR and LQI gains, constructs the MPC, and generates the reference and swing up signals. Executed before the Simulink model. |
| `pendulum_cart.slx` | Simulink model for the rig, containing the RT-DAC/USB2 driver, angle normalisation, Kalman estimator, LQR and LQI subsystem, MPC block, swing up chirp and switching logic, safety saturation, and scopes. |
| `SysID/fixed_cart_id.m`, `fixed_cart_model.m` | Stage 1, pendulum dynamics with the fixed cart ($a_1$, $k_3$). |
| `SysID/fixed_pendulum_id.m`, `fixed_pendulum_model.m` | Stage 2, cart dynamics with the fixed pendulum ($k_1$, $k_2$, $k_s$, $k_d$). |
| `SysID/invPend_id.m`, `invPend_model.m` | Stage 3, coupled nonlinear grey box estimation. |
| `dataExtraction.m` | Converts a Simulink scope log into the `expData` structure used for identification. |
| `generatePWM.m` | Rail to rail PWM excitation with unactuated intervals, used in the identification experiments. |
| `comparisonPlotRefTracking.m` | LQI and MPC reference tracking figures from the stored logs. |
| `comparisonPlotSwingUp.m` | LQI and MPC swing up figures from the stored logs. |
| `Dataset/` | Identification records and controller comparison logs (`.mat`). |
| `Plots/` | Figures, organised by category. `pendulum_cart.pdf` and `pendulum_cart.png` contain the block diagram. |

---

## Software requirements

- MATLAB R2020a or later, with Simulink
- Control System Toolbox (`c2d`, `ss`, `dlqr`, `dare`, `ctrb`, `obsv`, `kalman`)
- System Identification Toolbox (`idnlgrey`, `nlgreyest`, `compare`), for the `SysID/` scripts
- Model Predictive Control Toolbox (`mpc`, `mpcstate`, `setEstimator`)
- Signal Processing Toolbox (`smoothdata` with the `sgolay` method), for the identification scripts
- INTECO **RT-DAC/USB2** real-time toolbox and the physical rig

The identification scripts, the comparison scripts and all figures in `Plots/` can be run without
hardware. See [`requirements.txt`](requirements.txt) for the same list in a checkable form.

---

## Usage

Construct the model, gains and MPC object, then open and run the Simulink model.

```matlab
main
open_system('pendulum_cart')     % requires the rig and the real-time toolbox
```

Reproduce the identification from the recorded data sets.

```matlab
cd SysID
fixed_cart_id       % stage 1, pendulum
fixed_pendulum_id   % stage 2, cart
invPend_id          % stage 3, coupled
```

Regenerate the comparison figures from the stored logs.

```matlab
comparisonPlotRefTracking
comparisonPlotSwingUp
```

---

## Authors

Kadir Aksu, Erdem Yozdemir

## License

Released under the MIT License. See [`LICENSE`](LICENSE) for the full text.
