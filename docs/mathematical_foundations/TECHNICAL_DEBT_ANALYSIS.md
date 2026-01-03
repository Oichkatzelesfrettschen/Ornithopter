# Technical Debt and Lacunae Analysis
## Mathematical Foundations of Ornithopter Control System

---

## 1. Executive Summary

This document provides a detailed analysis of technical debt (debitum technicum) and knowledge gaps (lacunae) in the current ornithopter control system from a mathematical perspective.

---

## 2. Current System Mathematical Framework

### 2.1 PID Control Implementation

**Current Code**:
```cpp
void PID::compute(float measurement, float target)
{
    error = target - measurement;
    integration += error;
    integration = integration >  integLimit ?  integLimit : integration;
    integration = integration < -integLimit ? -integLimit : integration;
    output = Kp * error + Kd * (error - last_error) + Ki * integration;
    output = output > outputUpLimit ? outputUpLimit : output;
    output = output < outputDownLimit ? outputDownLimit : output;
    last_error = error;
}
```

**Mathematical Formulation**:
```
u(t) = Kp·e(t) + Ki·∫e(τ)dτ + Kd·de(t)/dt
```

### 2.2 Identified Issues

#### Issue 1: Derivative Kick Problem
**Problem**: When setpoint changes rapidly, derivative term causes large spikes.

**Mathematical Explanation**:
```
e(t) = r(t) - y(t)
de/dt = dr/dt - dy/dt
```

When r(t) changes (step input), dr/dt → ∞, causing derivative kick.

**Solution**: Derivative on measurement only:
```
de/dt → -dy/dt
```

#### Issue 2: Integral Windup
**Current Mitigation**: Simple clamping
```cpp
integration = integration > integLimit ? integLimit : integration;
```

**Problem**: Not optimal. Better solutions:
- Back-calculation (Åström-Hägglund)
- Conditional integration
- Intelligent integrator

**Back-calculation Method**:
```
if (output_saturated) {
    integration = integration - (1/Ti) * (output_actual - output_desired) * dt
}
```

#### Issue 3: Fixed Sampling Time
**Current**: 
```cpp
delay(5);  // 5ms non-deterministic delay
```

**Problem**: Jitter affects derivative and integral calculations.

**Mathematical Impact**:
```
Δt_actual ≠ Δt_nominal
```
Causes drift in integral and noise amplification in derivative.

**Solution**: Timer-based interrupts
```cpp
hw_timer_t * timer = timerBegin(0, 80, true);
timerAttachInterrupt(timer, &onTimer, true);
timerAlarmWrite(timer, 5000, true);  // 5ms = 200Hz
```

---

## 3. State-Space Representation Lacuna

### 3.1 Current Gap

The system lacks formal state-space model. Current approach: independent PID loops.

**Consequence**: Cannot analyze:
- Controllability
- Observability  
- Optimal control (LQR)
- Multi-variable interactions

### 3.2 Required State-Space Model

**State Vector**:
```
x = [θ, φ, ψ, θ̇, φ̇, ψ̇]ᵀ
```
where θ=roll, φ=pitch, ψ=yaw

**Linearized Dynamics** (small angle approximation):
```
ẋ = Ax + Bu
y = Cx + Du
```

**A Matrix** (6×6 system dynamics):
```
A = [0    0    0    1    0    0  ]
    [0    0    0    0    1    0  ]
    [0    0    0    0    0    1  ]
    [a31  0    0   a34   0   a36 ]
    [0   a42   0    0   a45   0  ]
    [0    0   a53   0    0   a56 ]
```

Elements depend on:
- Moment of inertia: Ix, Iy, Iz
- Aerodynamic derivatives: Cl_δ, Cm_δ, Cn_δ
- Control effectiveness

**B Matrix** (6×4 control input):
```
B = [0    0    0    0  ]
    [0    0    0    0  ]
    [0    0    0    0  ]
    [b41  0    0   b44 ]
    [0   b52  0    0  ]
    [0    0   b63  0  ]
```

Control inputs: u = [δ1, δ2, δ3, δ4]ᵀ (servo commands)

### 3.3 System Identification Procedure

**Steps to derive A and B**:

1. **Experimental Data Collection**:
   - Apply step inputs to each actuator
   - Record IMU response (θ, φ, ψ, ωx, ωy, ωz)
   - Multiple trials for statistical significance

2. **Parameter Estimation**:
   - Least squares fitting
   - Subspace identification (N4SID)
   - Maximum likelihood estimation

3. **Validation**:
   - Cross-validation with test data
   - Residual analysis
   - Frequency response (Bode plots)

**Mathematical Framework**:
```
min_θ Σ ||y_measured(t) - y_model(t; θ)||²
```
where θ = {elements of A, B}

---

## 4. Nonlinear Dynamics Considerations

### 4.1 Limitation of Linear Models

For large angles, small-angle approximation fails:
```
sin(θ) ≈ θ  only valid for |θ| < 15°
```

### 4.2 Full Nonlinear Equations

**Euler's Rotation Equations**:
```
Ix·ω̇x = (Iy - Iz)·ωy·ωz + Lx
Iy·ω̇y = (Iz - Ix)·ωz·ωx + Ly
Iz·ω̇z = (Ix - Iy)·ωx·ωy + Lz
```

**Kinematic Equations** (body to inertial):
```
[θ̇]   [1  sin(θ)tan(φ)  cos(θ)tan(φ)] [ωx]
[φ̇] = [0      cos(θ)        -sin(θ)  ] [ωy]
[ψ̇]   [0  sin(θ)sec(φ)  cos(θ)sec(φ)] [ωz]
```

**Singularity**: φ = ±90° (gimbal lock) → use quaternions!

### 4.3 Feedback Linearization

Transform nonlinear system to linear via change of coordinates:
```
v = α(x) + β(x)·u
```

Choose α and β such that:
```
ż = Az + Bv  (linear)
```

**Benefit**: Apply linear control theory to nonlinear plant.

**Limitation**: Requires accurate model; sensitive to uncertainties.

---

## 5. Coupling Effects

### 5.1 Current Approach: Decoupled PIDs

```cpp
pitchPID.compute(0, channels[1]-992.0);
rollPID.compute(0, channels[0]-993.0);
yawPID.compute(0, channels[3]-990.0);
```

Treats pitch, roll, yaw independently.

### 5.2 Actual Coupling

**Aerodynamic Coupling**:
- Roll generates sideslip → yaw moment
- Pitch changes airspeed → all moments change
- Yaw rate creates apparent side wind → roll coupling

**Gyroscopic Coupling**:
```
L_gyro = (Iy - Iz)·ωy·ωz
M_gyro = (Iz - Ix)·ωz·ωx
N_gyro = (Ix - Iy)·ωx·ωy
```

**Inertial Coupling**: For asymmetric mass distribution

### 5.3 Remedy: MIMO Control

**Multi-Input Multi-Output** design:
```
u = -K·x
```
where K is 4×6 gain matrix optimized via LQR:
```
K = R⁻¹·Bᵀ·P
```
P solves Riccati equation.

**Advantage**: Explicitly handles coupling.

---

## 6. Stability Analysis Gaps

### 6.1 No Formal Stability Proof

Current system: heuristic tuning of PID gains.

**Missing**:
- Stability margins (gain margin, phase margin)
- Region of attraction
- Robustness to parameter variations

### 6.2 Lyapunov Analysis

**Propose Lyapunov Function**:
```
V(x) = xᵀ·P·x
```

**Stability Condition**:
```
V̇(x) = xᵀ(AᵀP + PA - 2PBK)x < 0
```

For closed-loop system: ẋ = (A - BK)x

**Implementation**:
```cpp
float lyapunov_value() {
    Eigen::VectorXf x(6);
    x << roll, pitch, yaw, gyro_x, gyro_y, gyro_z;
    return x.transpose() * P * x;
}
```

### 6.3 Robustness Analysis

**Structured Singular Value (μ-analysis)**:
```
μ(M, Δ) = 1 / min{σ̄(Δ) : det(I - M·Δ) = 0}
```

Tests stability against uncertainty set Δ.

**H∞ Control** for worst-case disturbance rejection:
```
||T_zw||_∞ < γ
```

---

## 7. Sensor Fusion Lacuna

### 7.1 Current State

MPU9250 library performs internal sensor fusion (6-axis or 9-axis).

**Problem**: Black box. Cannot:
- Tune filter parameters
- Add additional sensors (barometer, GPS)
- Characterize uncertainty

### 7.2 Required: Kalman Filter

**Extended Kalman Filter** (nonlinear system):

**Prediction**:
```
x̂⁻ = f(x̂⁺, u)
P⁻ = F·P⁺·Fᵀ + Q
```

**Update**:
```
K = P⁻·Hᵀ·(H·P⁻·Hᵀ + R)⁻¹
x̂⁺ = x̂⁻ + K·(z - h(x̂⁻))
P⁺ = (I - K·H)·P⁻
```

**Jacobians**:
```
F = ∂f/∂x |_{x̂,u}
H = ∂h/∂x |_{x̂}
```

**Covariance Matrices**:
- Q: Process noise (model uncertainty)
- R: Measurement noise (sensor specs)

### 7.3 Complementary Filter (Lightweight Alternative)

**Advantage**: Computationally cheaper than EKF.

```
θ = α·(θ_prev + ωx·dt) + (1-α)·θ_accel
```

**Tuning α**:
```
α = τ / (τ + dt)
```
where τ is cutoff time constant (0.5-2 seconds typical).

**Frequency Domain**:
```
H_gyro(s) = τs / (τs + 1)  (high-pass)
H_accel(s) = 1 / (τs + 1)  (low-pass)
```

---

## 8. Observability and Controllability

### 8.1 Definitions

**Controllability Matrix**:
```
C = [B  AB  A²B  ...  Aⁿ⁻¹B]
```
System controllable if rank(C) = n.

**Observability Matrix**:
```
O = [C; CA; CA²; ...; CAⁿ⁻¹]
```
System observable if rank(O) = n.

### 8.2 Physical Interpretation

**Controllability**: Can we drive system from any state to any other state?

For ornithopter:
- Can we command arbitrary attitude?
- Are there unreachable states?

**Observability**: Can we uniquely determine state from measurements?

For ornithopter:
- IMU measures angular velocity and acceleration
- Can we reconstruct position and velocity?
- GPS needed for absolute position

### 8.3 Verification

```python
import numpy as np
from scipy.linalg import matrix_rank

# System matrices (example)
A = np.array([[0, 0, 0, 1, 0, 0],
              [0, 0, 0, 0, 1, 0],
              [0, 0, 0, 0, 0, 1],
              [-1, 0, 0, -0.1, 0, 0],
              [0, -1, 0, 0, -0.1, 0],
              [0, 0, -0.5, 0, 0, -0.05]])

B = np.array([[0, 0, 0, 0],
              [0, 0, 0, 0],
              [0, 0, 0, 0],
              [1, 0, 0, 0],
              [0, 1, 0, 0],
              [0, 0, 1, 0]])

# Controllability matrix
n = A.shape[0]
C = B
for i in range(1, n):
    C = np.hstack((C, np.linalg.matrix_power(A, i) @ B))

rank_C = matrix_rank(C)
print(f"Controllability matrix rank: {rank_C}/{n}")
if rank_C == n:
    print("System is controllable")
```

---

## 9. Optimal Control Framework

### 9.1 Linear Quadratic Regulator (LQR)

**Cost Function**:
```
J = ∫₀^∞ (xᵀQx + uᵀRu) dt
```

**Q Matrix**: State penalty (diagonal)
```
Q = diag([q_θ, q_φ, q_ψ, q_θ̇, q_φ̇, q_ψ̇])
```

Larger values → tighter tracking.

**R Matrix**: Control penalty (diagonal)
```
R = diag([r₁, r₂, r₃, r₄])
```

Larger values → less aggressive control.

**Optimal Gain**:
```
K = R⁻¹·Bᵀ·P
```

P solves Continuous Algebraic Riccati Equation (CARE):
```
AᵀP + PA - PBR⁻¹BᵀP + Q = 0
```

### 9.2 Implementation

```python
import numpy as np
from scipy.linalg import solve_continuous_are

# Design weights
Q = np.diag([10, 10, 5, 1, 1, 0.5])  # State weights
R = np.diag([1, 1, 1, 1])             # Control weights

# Solve Riccati equation
P = solve_continuous_are(A, B, Q, R)

# Compute optimal gain
K = np.linalg.inv(R) @ B.T @ P

print("Optimal LQR gain matrix K:")
print(K)
```

### 9.3 Advantages over PID

1. **MIMO design**: Handles coupling naturally
2. **Optimal**: Minimizes cost function
3. **Guaranteed stability**: If (A,B) controllable and (A,C) observable
4. **Systematic tuning**: Via Q and R matrices

---

## 10. Model Predictive Control (MPC)

### 10.1 Concept

Optimize control over finite horizon:
```
min_{u(0),...,u(N-1)} Σ (xᵀQx + uᵀRu)
subject to:
    x(k+1) = Ax(k) + Bu(k)
    u_min ≤ u(k) ≤ u_max
    x_min ≤ x(k) ≤ x_max
```

**Receding Horizon**: Apply u(0), then recompute.

### 10.2 Advantages for Ornithopter

1. **Constraint handling**: Explicitly enforce actuator limits
2. **Predictive**: Anticipates future behavior
3. **Trajectory optimization**: Can follow complex paths

### 10.3 Computational Challenge

**Quadratic Programming** (QP) solver required.

**ESP32 feasibility**:
- Small horizons (N ≤ 10) feasible
- Fast QP solvers: qpOASES, OSQP
- May need ~50-100ms per solve

**Alternative**: Explicit MPC
- Pre-compute control law offline
- Online: lookup table / piecewise affine function

---

## 11. Adaptive Control Needs

### 11.1 Motivation

Ornithopter parameters vary:
- Mass changes (battery discharge)
- Aerodynamics change (wing wear, damage)
- Center of gravity shifts

**Fixed gains** → degraded performance or instability.

### 11.2 Gain Scheduling

Simplest adaptive approach:
```
K = K(ρ)
```
where ρ is scheduling parameter (e.g., airspeed).

**Implementation**:
```cpp
float compute_gain(float airspeed) {
    if (airspeed < 5.0) return Kp_low;
    else if (airspeed < 10.0) return Kp_mid;
    else return Kp_high;
}
```

### 11.3 Model Reference Adaptive Control (MRAC)

**Reference Model**: Desired closed-loop behavior
```
ẋ_m = A_m·x_m + B_m·r
```

**Adaptive Law**:
```
K̇ = -Γ·x·eᵀPB
```
where e = x - x_m (tracking error).

**Guarantee**: Lyapunov stability proof exists.

### 11.4 Self-Tuning Regulator

**Recursive Parameter Estimation**:
```
θ̂(k) = θ̂(k-1) + P(k)·φ(k)·ε(k)
```
where:
- θ̂: parameter estimates
- φ: regressor vector
- ε: prediction error

**Control Law**: Design based on θ̂ (certainty equivalence).

---

## 12. Frequency Domain Analysis

### 12.1 Missing Tools

Current system: no frequency analysis.

**Required**:
- Bode plots (gain and phase vs frequency)
- Nyquist plots (stability margins)
- Nichols charts

### 12.2 Transfer Function

For SISO loop (e.g., pitch):
```
G(s) = C(s)·P(s) / (1 + C(s)·P(s))
```
where:
- C(s): Controller transfer function
- P(s): Plant transfer function

**PID Controller**:
```
C(s) = Kp + Ki/s + Kd·s
```

**Plant** (second-order approximation):
```
P(s) = K / (s² + 2ζωₙs + ωₙ²)
```

### 12.3 Stability Margins

**Gain Margin**: How much gain increase until instability?
```
GM = 1 / |G(jω_π)|
```
where ∠G(jω_π) = -180°.

**Phase Margin**: How much phase lag until instability?
```
PM = 180° + ∠G(jω_c)
```
where |G(jω_c)| = 1 (crossover frequency).

**Design Targets**:
- GM > 6 dB
- PM > 45°

---

## 13. Priority Ranking of Technical Debt

| Priority | Item | Impact | Effort | Recommendation |
|----------|------|--------|--------|----------------|
| 1 | Fixed timing → Timer ISR | High | Low | Immediate |
| 2 | Derivative kick fix | Medium | Low | Short-term |
| 3 | State-space model | High | High | Short-term |
| 4 | Kalman filter | High | Medium | Medium-term |
| 5 | LQR control | Medium | Medium | Medium-term |
| 6 | Stability monitoring | High | Low | Short-term |
| 7 | Nonlinear control | Medium | High | Long-term |
| 8 | MPC | Low | High | Long-term |
| 9 | Adaptive gains | Medium | Medium | Medium-term |

---

## 14. Recommended Immediate Actions

### Action 1: Timer-Based Control Loop
Replace `delay(5)` with hardware timer.

### Action 2: Implement Basic Kalman Filter
Even simplified 6-state EKF will improve performance.

### Action 3: System Identification
Collect flight data to build A and B matrices.

### Action 4: Stability Monitoring
Add Lyapunov function computation to detect instability early.

### Action 5: PID Improvements
- Derivative on measurement
- Better anti-windup (back-calculation)
- Bumpless transfer for mode changes

---

## Conclusion

This analysis has identified critical mathematical gaps in the current system and prioritized remedies. Implementation of these recommendations will transform the ad-hoc PID controller into a theoretically sound, provably stable, and optimal control system.

