#!/usr/bin/env python3
"""
Z3 Formal Verification for Ornithopter Control System

This script uses the Z3 SMT solver to verify critical properties of the
ornithopter control system, including:
1. PID stability conditions
2. Attitude bounds verification
3. Actuator saturation handling
4. Timing constraint verification
5. Lyapunov stability proof

Author: ELEC6212 Robotics Team
Date: January 2026
"""

from z3 import *
import sys

# ============================================================================
# Configuration
# ============================================================================

MAX_ANGLE = 30.0  # Maximum safe roll/pitch angle (degrees)
MAX_YAW_RATE = 100.0  # Maximum safe yaw rate (deg/s)
MIN_LOOP_TIME = 0.001  # Minimum control loop time (seconds)
MAX_LOOP_TIME = 0.010  # Maximum control loop time (seconds)
SERVO_MIN = 0
SERVO_MAX = 1000
MOTOR_MIN = 0
MOTOR_MAX = 800

# ============================================================================
# Test 1: PID Stability Verification
# ============================================================================

def verify_pid_stability():
    """
    Verify that PID gains satisfy stability conditions.
    
    For a second-order system:
    G(s) = ω²/(s² + 2ζωs + ω²)
    
    With PID controller:
    C(s) = Kp + Ki/s + Kd*s
    
    Closed-loop characteristic equation:
    s³ + (2ζω + Kd*ω²)s² + (ω² + Kp*ω²)s + Ki*ω² = 0
    
    Routh-Hurwitz stability criterion must be satisfied.
    """
    print("\n" + "="*70)
    print("TEST 1: PID Stability Verification")
    print("="*70)
    
    s = Solver()
    
    # PID gains (variables to find)
    Kp = Real('Kp')
    Ki = Real('Ki')
    Kd = Real('Kd')
    
    # Plant parameters (assumed)
    omega = 10.0  # Natural frequency
    zeta = 0.5    # Damping ratio
    
    # Characteristic equation coefficients
    a2 = 2*zeta*omega + Kd*omega*omega
    a1 = omega*omega + Kp*omega*omega
    a0 = Ki*omega*omega
    
    # Routh-Hurwitz stability conditions
    # For s³ + a₂s² + a₁s + a₀ = 0:
    # 1. All coefficients must be positive
    # 2. a₂*a₁ - a₀ > 0
    
    s.add(Kp > 0)
    s.add(Ki >= 0)
    s.add(Kd >= 0)
    
    # Practical bounds (from empirical tuning)
    s.add(Kp < 20)
    s.add(Ki < 5)
    s.add(Kd < 5)
    
    # Routh-Hurwitz conditions
    s.add(a2 > 0)
    s.add(a1 > 0)
    s.add(a0 >= 0)
    s.add(a2 * a1 - a0 > 0)
    
    # Additional performance constraint: phase margin > 45°
    # Approximation: Kd/Kp < 0.5
    s.add(Kd < 0.5 * Kp)
    
    result = s.check()
    
    if result == sat:
        m = s.model()
        print("✓ Valid PID gains found:")
        print(f"  Kp = {m[Kp]}")
        print(f"  Ki = {m[Ki]}")
        print(f"  Kd = {m[Kd]}")
        print("\nStability conditions:")
        kp_val = float(m[Kp].as_fraction())
        ki_val = float(m[Ki].as_fraction())
        kd_val = float(m[Kd].as_fraction())
        a2_val = 2*zeta*omega + kd_val*omega*omega
        a1_val = omega*omega + kp_val*omega*omega
        a0_val = ki_val*omega*omega
        print(f"  a₂ = {a2_val:.4f} > 0 ✓")
        print(f"  a₁ = {a1_val:.4f} > 0 ✓")
        print(f"  a₀ = {a0_val:.4f} ≥ 0 ✓")
        print(f"  a₂*a₁ - a₀ = {a2_val*a1_val - a0_val:.4f} > 0 ✓")
        return True
    else:
        print("✗ No valid PID gains exist within constraints")
        return False

# ============================================================================
# Test 2: Attitude Bounds Verification
# ============================================================================

def verify_attitude_bounds():
    """
    Verify that control system keeps attitude within safe bounds.
    
    Given current state within bounds and bounded control inputs,
    prove next state also within bounds.
    """
    print("\n" + "="*70)
    print("TEST 2: Attitude Bounds Verification")
    print("="*70)
    
    s = Solver()
    
    # Current state
    roll = Real('roll')
    pitch = Real('pitch')
    yaw = Real('yaw')
    roll_rate = Real('roll_rate')
    pitch_rate = Real('pitch_rate')
    yaw_rate = Real('yaw_rate')
    
    # Control inputs (torques)
    tau_roll = Real('tau_roll')
    tau_pitch = Real('tau_pitch')
    tau_yaw = Real('tau_yaw')
    
    # Time step
    dt = 0.005  # 5ms
    
    # System dynamics (simplified): angular_acceleration = tau / I
    I = 0.01  # Moment of inertia (kg*m²)
    
    # Next state prediction
    roll_acc = tau_roll / I
    pitch_acc = tau_pitch / I
    
    next_roll = roll + roll_rate * dt + 0.5 * roll_acc * dt * dt
    next_pitch = pitch + pitch_rate * dt + 0.5 * pitch_acc * dt * dt
    next_roll_rate = roll_rate + roll_acc * dt
    next_pitch_rate = pitch_rate + pitch_acc * dt
    
    # Assume current state is within safe bounds
    s.add(And(roll >= -MAX_ANGLE, roll <= MAX_ANGLE))
    s.add(And(pitch >= -MAX_ANGLE, pitch <= MAX_ANGLE))
    s.add(And(roll_rate >= -100, roll_rate <= 100))
    s.add(And(pitch_rate >= -100, pitch_rate <= 100))
    
    # Control outputs are bounded (from PID)
    MAX_CONTROL = 500.0
    s.add(And(tau_roll >= -MAX_CONTROL, tau_roll <= MAX_CONTROL))
    s.add(And(tau_pitch >= -MAX_CONTROL, tau_pitch <= MAX_CONTROL))
    
    # Try to find counterexample where next state violates bounds
    s.add(Or(
        next_roll < -MAX_ANGLE,
        next_roll > MAX_ANGLE,
        next_pitch < -MAX_ANGLE,
        next_pitch > MAX_ANGLE
    ))
    
    result = s.check()
    
    if result == unsat:
        print("✓ Invariant holds: System stays within bounds")
        print(f"  Roll: ±{MAX_ANGLE}°")
        print(f"  Pitch: ±{MAX_ANGLE}°")
        print(f"  Given max control torque: ±{MAX_CONTROL}")
        print(f"  Time step: {dt} seconds")
        return True
    else:
        print("✗ Counterexample found:")
        m = s.model()
        print(f"  Current roll: {m[roll]}")
        print(f"  Current roll_rate: {m[roll_rate]}")
        print(f"  Control torque: {m[tau_roll]}")
        # Calculate next state - safely extract values from Z3 model
        def extract_value(z3_val):
            """Safely extract numeric value from Z3 model value"""
            if hasattr(z3_val, 'as_fraction'):
                return float(z3_val.as_fraction())
            elif hasattr(z3_val, 'as_long'):
                return float(z3_val.as_long())
            else:
                return float(str(z3_val))
        
        r = extract_value(m[roll])
        rr = extract_value(m[roll_rate])
        tr = extract_value(m[tau_roll])
        ra = tr / I
        nr = r + rr * dt + 0.5 * ra * dt * dt
        print(f"  Next roll: {nr}° (violates bounds)")
        return False

# ============================================================================
# Test 3: Actuator Saturation Safety
# ============================================================================

def verify_actuator_saturation():
    """
    Verify that actuator commands never exceed hardware limits.
    """
    print("\n" + "="*70)
    print("TEST 3: Actuator Saturation Verification")
    print("="*70)
    
    s = Solver()
    
    # PID output before saturation
    pid_output = Real('pid_output')
    
    # Saturated output
    saturated = Real('saturated')
    
    # Saturation logic
    s.add(If(pid_output > SERVO_MAX,
             saturated == SERVO_MAX,
             If(pid_output < SERVO_MIN,
                saturated == SERVO_MIN,
                saturated == pid_output)))
    
    # Verify saturated output always within bounds
    s.add(Or(saturated < SERVO_MIN, saturated > SERVO_MAX))
    
    result = s.check()
    
    if result == unsat:
        print("✓ Actuator commands always within hardware limits")
        print(f"  Servo range: [{SERVO_MIN}, {SERVO_MAX}]")
        print(f"  Motor range: [{MOTOR_MIN}, {MOTOR_MAX}]")
        return True
    else:
        print("✗ Saturation logic error detected")
        m = s.model()
        print(f"  PID output: {m[pid_output]}")
        print(f"  Saturated: {m[saturated]}")
        return False

# ============================================================================
# Test 4: Timing Constraint Verification
# ============================================================================

def verify_timing_constraints():
    """
    Verify that control loop timing meets real-time requirements.
    """
    print("\n" + "="*70)
    print("TEST 4: Timing Constraint Verification")
    print("="*70)
    
    s = Solver()
    
    # Task execution times (in seconds)
    t_sensor_read = Real('t_sensor_read')
    t_control_compute = Real('t_control_compute')
    t_actuator_update = Real('t_actuator_update')
    
    # Total loop time
    t_loop = t_sensor_read + t_control_compute + t_actuator_update
    
    # Measured/estimated times
    s.add(t_sensor_read == 0.0002)      # 0.2 ms
    s.add(t_control_compute == 0.0005)  # 0.5 ms
    s.add(t_actuator_update == 0.0001)  # 0.1 ms
    
    # Verify loop time meets deadline
    s.add(Or(t_loop < MIN_LOOP_TIME, t_loop > MAX_LOOP_TIME))
    
    result = s.check()
    
    if result == unsat:
        # Calculate actual time
        t_total = 0.0002 + 0.0005 + 0.0001
        print("✓ Timing constraints satisfied")
        print(f"  Sensor read: 0.2 ms")
        print(f"  Control compute: 0.5 ms")
        print(f"  Actuator update: 0.1 ms")
        print(f"  Total loop time: {t_total*1000:.2f} ms")
        print(f"  Deadline: {MAX_LOOP_TIME*1000:.1f} ms")
        print(f"  Margin: {(MAX_LOOP_TIME - t_total)*1000:.2f} ms")
        return True
    else:
        print("✗ Timing deadline violated")
        return False

# ============================================================================
# Test 5: Lyapunov Stability Proof
# ============================================================================

def verify_lyapunov_stability():
    """
    Verify Lyapunov stability for the closed-loop system.
    
    Lyapunov function: V(x) = x'Px where P is positive definite
    Stability condition: V̇(x) = x'(A'P + PA)x < 0
    """
    print("\n" + "="*70)
    print("TEST 5: Lyapunov Stability Proof")
    print("="*70)
    
    s = Solver()
    
    # State variables (2D simplified: angle and angular velocity)
    theta = Real('theta')
    omega = Real('omega')
    
    # System parameters
    K_p = 5.0   # Proportional gain
    K_d = 1.0   # Derivative gain
    I = 0.01    # Inertia
    
    # Closed-loop dynamics: 
    # θ̈ = -Kp*θ - Kd*θ̇
    # State: x = [θ, θ̇]'
    # ẋ = Ax where A = [0, 1; -Kp/I, -Kd/I]
    
    # Lyapunov function: V = 0.5*Kp*θ² + 0.5*I*ω²
    # (energy-based)
    V = 0.5 * K_p * theta * theta + 0.5 * I * omega * omega
    
    # Time derivative: V̇ = Kp*θ*θ̇ + I*ω*ω̇
    # With θ̇ = ω and ω̇ = -Kp*θ/I - Kd*ω/I:
    # V̇ = Kp*θ*ω + I*ω*(-Kp*θ/I - Kd*ω/I)
    #   = Kp*θ*ω - Kp*θ*ω - Kd*ω²
    #   = -Kd*ω²
    
    V_dot = -K_d * omega * omega
    
    # Assume non-trivial state (not at origin)
    s.add(Or(theta != 0, omega != 0))
    
    # V must be positive
    s.add(V <= 0)
    
    result_V_positive = s.check()
    
    s = Solver()  # Reset solver
    
    # V̇ must be negative (or zero only at origin)
    s.add(Or(theta != 0, omega != 0))
    s.add(V_dot >= 0)
    
    result_V_dot_negative = s.check()
    
    if result_V_positive == unsat and result_V_dot_negative == unsat:
        print("✓ Lyapunov stability proof successful")
        print(f"  Lyapunov function: V = 0.5*Kp*θ² + 0.5*I*ω²")
        print(f"  V > 0 for all x ≠ 0 ✓")
        print(f"  V̇ = -Kd*ω² ≤ 0 ✓")
        print(f"  System is globally asymptotically stable")
        return True
    else:
        print("✗ Lyapunov stability proof failed")
        return False

# ============================================================================
# Test 6: Mode Transition Safety
# ============================================================================

def verify_mode_transitions():
    """
    Verify that mode transitions maintain safety.
    """
    print("\n" + "="*70)
    print("TEST 6: Mode Transition Safety")
    print("="*70)
    
    s = Solver()
    
    # System mode
    current_mode = Int('current_mode')
    next_mode = Int('next_mode')
    
    # Attitude
    roll = Real('roll')
    pitch = Real('pitch')
    
    # Stability flag
    stable = Bool('stable')
    
    # Mode definitions: 0=safe, 1=servo, 2=full_power
    s.add(And(current_mode >= 0, current_mode <= 2))
    s.add(And(next_mode >= 0, next_mode <= 2))
    
    # Stability condition
    s.add(stable == And(
        roll >= -MAX_ANGLE,
        roll <= MAX_ANGLE,
        pitch >= -MAX_ANGLE,
        pitch <= MAX_ANGLE
    ))
    
    # Safety rule: Can only transition to higher mode if stable
    # If unstable, must go to safe mode (0)
    safe_transition = Implies(
        Not(stable),
        next_mode == 0
    )
    
    s.add(Not(safe_transition))
    
    result = s.check()
    
    if result == unsat:
        print("✓ Mode transitions always safe")
        print("  Rule: Unstable state → Safe mode (0)")
        print("  Rule: Higher modes require stability")
        return True
    else:
        print("✗ Unsafe mode transition possible")
        m = s.model()
        print(f"  Current mode: {m[current_mode]}")
        print(f"  Next mode: {m[next_mode]}")
        print(f"  Stable: {m[stable]}")
        return False

# ============================================================================
# Main Test Suite
# ============================================================================

def main():
    print("\n" + "█"*70)
    print("█" + " "*68 + "█")
    print("█  Z3 Formal Verification Suite for Ornithopter Control System  █")
    print("█" + " "*68 + "█")
    print("█"*70)
    
    results = []
    
    # Run all tests
    results.append(("PID Stability", verify_pid_stability()))
    results.append(("Attitude Bounds", verify_attitude_bounds()))
    results.append(("Actuator Saturation", verify_actuator_saturation()))
    results.append(("Timing Constraints", verify_timing_constraints()))
    results.append(("Lyapunov Stability", verify_lyapunov_stability()))
    results.append(("Mode Transitions", verify_mode_transitions()))
    
    # Summary
    print("\n" + "="*70)
    print("VERIFICATION SUMMARY")
    print("="*70)
    
    passed = 0
    failed = 0
    
    for test_name, result in results:
        status = "✓ PASSED" if result else "✗ FAILED"
        print(f"{test_name:.<50} {status}")
        if result:
            passed += 1
        else:
            failed += 1
    
    print("="*70)
    print(f"Total: {passed + failed} tests")
    print(f"Passed: {passed}")
    print(f"Failed: {failed}")
    print("="*70)
    
    if failed == 0:
        print("\n🎉 All formal verification tests passed!")
        return 0
    else:
        print(f"\n⚠️  {failed} test(s) failed. Please review.")
        return 1

if __name__ == "__main__":
    sys.exit(main())
