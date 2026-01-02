---------------------------- MODULE Ornithopter ----------------------------
(***************************************************************************
 * TLA+ Specification for Ornithopter Flight Control System
 * 
 * This module specifies the behavior of the ornithopter control system,
 * including safety properties, liveness properties, and timing constraints.
 *
 * Author: ELEC6212 Robotics Team
 * Date: January 2026
 ***************************************************************************)

EXTENDS Integers, Reals, Sequences, TLC

-----------------------------------------------------------------------------

CONSTANTS 
    MAX_ANGLE,              \* Maximum safe attitude angle (degrees)
    MIN_CONTROL_FREQ,       \* Minimum control loop frequency (Hz)
    MAX_SENSOR_LATENCY,     \* Maximum sensor read latency (ms)
    MAX_ACTUATOR_DELAY,     \* Maximum actuator response time (ms)
    SENSOR_TIMEOUT,         \* Sensor timeout threshold (ms)
    NUM_SENSORS,            \* Number of sensor channels
    NUM_ACTUATORS           \* Number of actuator channels

ASSUME 
    /\ MAX_ANGLE \in 1..90
    /\ MIN_CONTROL_FREQ > 0
    /\ MAX_SENSOR_LATENCY > 0
    /\ MAX_ACTUATOR_DELAY > 0

-----------------------------------------------------------------------------

VARIABLES
    attitude,               \* Current attitude [roll, pitch, yaw]
    angular_velocity,       \* Current angular velocity [wx, wy, wz]
    control_output,         \* Control signals to actuators
    sensor_data,            \* Raw sensor readings from IMU
    system_mode,            \* 0: safe, 1: servo-only, 2: full-power
    requested_mode,         \* Requested mode change
    stable,                 \* Boolean: system stability flag
    sensor_status,          \* Sensor health status
    actuator_status,        \* Actuator health status
    last_sensor_time,       \* Timestamp of last sensor reading
    last_control_time,      \* Timestamp of last control computation
    clock                   \* System clock (milliseconds)

vars == <<attitude, angular_velocity, control_output, sensor_data, 
          system_mode, requested_mode, stable, sensor_status, actuator_status,
          last_sensor_time, last_control_time, clock>>

-----------------------------------------------------------------------------

(***************************************************************************
 * Type Invariants
 ***************************************************************************)

Angle == -180..180
SafeAngle == -MAX_ANGLE..MAX_ANGLE
AngularVel == -1000..1000  \* degrees per second
SystemMode == {0, 1, 2}
SensorReading == [accel: Seq(Reals), gyro: Seq(Reals), mag: Seq(Reals)]
ControlSignal == [servo1: 0..1000, servo2: 0..1000, servo3: 0..1000, 
                  servo4: 0..1000, motor: 0..800]

TypeInvariant ==
    /\ attitude.roll \in Angle
    /\ attitude.pitch \in Angle
    /\ attitude.yaw \in Angle
    /\ angular_velocity.x \in AngularVel
    /\ angular_velocity.y \in AngularVel
    /\ angular_velocity.z \in AngularVel
    /\ system_mode \in SystemMode
    /\ stable \in BOOLEAN
    /\ sensor_status \in BOOLEAN
    /\ actuator_status \in BOOLEAN
    /\ clock \in Nat

-----------------------------------------------------------------------------

(***************************************************************************
 * Safety Properties
 ***************************************************************************)

\* SAFETY 1: System must enter safe mode when unstable
SafetyModeWhenUnstable ==
    ~stable => system_mode = 0

\* SAFETY 2: Attitude must stay within safe bounds when system is stable
AttitudeBoundsWhenStable ==
    stable => 
        /\ attitude.roll \in SafeAngle
        /\ attitude.pitch \in SafeAngle

\* SAFETY 3: Motor only runs in full-power mode
MotorSafety ==
    (system_mode < 2) => (control_output.motor = 0)

\* SAFETY 4: Sensor timeout triggers safe mode
SensorTimeoutSafety ==
    (clock - last_sensor_time > SENSOR_TIMEOUT) => system_mode = 0

\* SAFETY 5: Control signals must be bounded
ControlBounds ==
    /\ control_output.servo1 \in 0..1000
    /\ control_output.servo2 \in 0..1000
    /\ control_output.servo3 \in 0..1000
    /\ control_output.servo4 \in 0..1000
    /\ control_output.motor \in 0..800

\* Combined safety property
SafetyProperty ==
    /\ SafetyModeWhenUnstable
    /\ AttitudeBoundsWhenStable
    /\ MotorSafety
    /\ SensorTimeoutSafety
    /\ ControlBounds

-----------------------------------------------------------------------------

(***************************************************************************
 * Liveness Properties
 ***************************************************************************)

\* LIVENESS 1: Control loop eventually executes
ControlLoopProgress ==
    []<>(last_control_time > 0)

\* LIVENESS 2: Sensors eventually read
SensorProgress ==
    []<>(last_sensor_time > 0)

\* LIVENESS 3: System eventually becomes stable (under normal conditions)
EventualStability ==
    (sensor_status /\ actuator_status) ~> stable

\* LIVENESS 4: Mode changes are eventually processed
ModeChangeResponsive ==
    \A new_mode \in SystemMode : 
        (requested_mode = new_mode) ~> (system_mode = new_mode)

-----------------------------------------------------------------------------

(***************************************************************************
 * Initial State
 ***************************************************************************)

Init ==
    /\ attitude = [roll |-> 0, pitch |-> 0, yaw |-> 0]
    /\ angular_velocity = [x |-> 0, y |-> 0, z |-> 0]
    /\ control_output = [servo1 |-> 500, servo2 |-> 500, servo3 |-> 500,
                         servo4 |-> 500, motor |-> 0]
    /\ sensor_data = [accel |-> <<0, 0, 9.81>>, 
                      gyro |-> <<0, 0, 0>>, 
                      mag |-> <<0, 0, 0>>]
    /\ system_mode = 0
    /\ requested_mode = 0
    /\ stable = TRUE
    /\ sensor_status = TRUE
    /\ actuator_status = TRUE
    /\ last_sensor_time = 0
    /\ last_control_time = 0
    /\ clock = 0

-----------------------------------------------------------------------------

(***************************************************************************
 * Actions
 ***************************************************************************)

\* Read sensor data from IMU
ReadSensors ==
    /\ sensor_status = TRUE
    /\ \E new_data \in SensorReading :
        /\ sensor_data' = new_data
        /\ last_sensor_time' = clock
        /\ UNCHANGED <<attitude, angular_velocity, control_output, 
                       system_mode, stable, actuator_status, 
                       last_control_time, clock>>

\* Compute control outputs using PID
ComputeControl ==
    /\ system_mode > 0
    /\ clock - last_sensor_time <= MAX_SENSOR_LATENCY
    /\ \E new_control \in ControlSignal :
        /\ control_output' = new_control
        /\ last_control_time' = clock
        /\ UNCHANGED <<attitude, angular_velocity, sensor_data, 
                       system_mode, stable, sensor_status, 
                       actuator_status, last_sensor_time, clock>>

\* Apply control to actuators and update system state
UpdateActuators ==
    /\ actuator_status = TRUE
    /\ control_output # [servo1 |-> 500, servo2 |-> 500, servo3 |-> 500,
                         servo4 |-> 500, motor |-> 0]
    /\ \E new_attitude \in [roll: Angle, pitch: Angle, yaw: Angle],
          new_ang_vel \in [x: AngularVel, y: AngularVel, z: AngularVel] :
        /\ attitude' = new_attitude
        /\ angular_velocity' = new_ang_vel
        /\ UNCHANGED <<control_output, sensor_data, system_mode, stable,
                       sensor_status, actuator_status, last_sensor_time,
                       last_control_time, clock>>

\* Check system stability
CheckStability ==
    /\ stable' = /\ (attitude.roll \in SafeAngle)
                 /\ (attitude.pitch \in SafeAngle)
                 /\ (ABS(angular_velocity.x) < 500)
                 /\ (ABS(angular_velocity.y) < 500)
                 /\ (ABS(angular_velocity.z) < 500)
    /\ IF ~stable' 
       THEN system_mode' = 0
       ELSE UNCHANGED system_mode
    /\ UNCHANGED <<attitude, angular_velocity, control_output, sensor_data,
                   sensor_status, actuator_status, last_sensor_time,
                   last_control_time, clock>>

\* Handle sensor failure
SensorFailure ==
    /\ sensor_status' = FALSE
    /\ system_mode' = 0
    /\ stable' = FALSE
    /\ UNCHANGED <<attitude, angular_velocity, control_output, sensor_data,
                   actuator_status, last_sensor_time, last_control_time,
                   clock>>

\* Handle actuator failure
ActuatorFailure ==
    /\ actuator_status' = FALSE
    /\ system_mode' = 0
    /\ stable' = FALSE
    /\ UNCHANGED <<attitude, angular_velocity, control_output, sensor_data,
                   sensor_status, last_sensor_time, last_control_time,
                   clock>>

\* Advance system clock
TickClock ==
    /\ clock' = clock + 1
    /\ IF clock' - last_sensor_time > SENSOR_TIMEOUT
       THEN /\ system_mode' = 0
            /\ stable' = FALSE
       ELSE UNCHANGED <<system_mode, stable>>
    /\ UNCHANGED <<attitude, angular_velocity, control_output, sensor_data,
                   sensor_status, actuator_status, last_sensor_time,
                   last_control_time>>

-----------------------------------------------------------------------------

(***************************************************************************
 * Next State Relation
 ***************************************************************************)

Next ==
    \/ ReadSensors
    \/ ComputeControl
    \/ UpdateActuators
    \/ CheckStability
    \/ SensorFailure
    \/ ActuatorFailure
    \/ TickClock

-----------------------------------------------------------------------------

(***************************************************************************
 * Specification
 ***************************************************************************)

\* Weak fairness: if action continuously enabled, eventually taken
Spec == 
    /\ Init 
    /\ [][Next]_vars
    /\ WF_vars(ReadSensors)      \* Sensors eventually read
    /\ WF_vars(ComputeControl)   \* Control eventually computed
    /\ WF_vars(CheckStability)   \* Stability eventually checked
    /\ WF_vars(TickClock)        \* Clock eventually advances

-----------------------------------------------------------------------------

(***************************************************************************
 * Theorems to be verified
 ***************************************************************************)

\* Main safety theorem
THEOREM SafetyTheorem == Spec => []SafetyProperty

\* Liveness theorem
THEOREM LivenessTheorem == Spec => ControlLoopProgress /\ SensorProgress

\* No deadlock
THEOREM NoDeadlock == Spec => []<><<Next>>_vars

=============================================================================

(***************************************************************************
 * Model Checking Configuration
 * 
 * To check this specification with TLC:
 * 1. Define constants in a .cfg file:
 *    MAX_ANGLE = 30
 *    MIN_CONTROL_FREQ = 200
 *    MAX_SENSOR_LATENCY = 10
 *    MAX_ACTUATOR_DELAY = 5
 *    SENSOR_TIMEOUT = 100
 *    NUM_SENSORS = 9
 *    NUM_ACTUATORS = 5
 * 
 * 2. Run TLC model checker:
 *    java -jar tla2tools.jar -config Ornithopter.cfg Ornithopter.tla
 * 
 * Expected properties to verify:
 * - SafetyProperty (invariant)
 * - TypeInvariant (invariant)
 * - ControlLoopProgress (temporal)
 * - SensorProgress (temporal)
 ***************************************************************************)
