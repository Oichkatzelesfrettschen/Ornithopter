#ifndef __QUATERNION_MATH_H__
#define __QUATERNION_MATH_H__

#include <Arduino.h>
#include <math.h>

/**
 * @file quaternion_math.h
 * @brief Advanced quaternion mathematics for spatial rotations
 * 
 * Implements quaternion operations for rotation representation,
 * avoiding gimbal lock and providing smooth interpolation (SLERP).
 * 
 * Quaternion representation: q = w + xi + yj + zk
 * where w is scalar part, (x,y,z) is vector part
 * 
 * @author ELEC6212 Robotics Team
 * @date January 2026
 */

// ============================================================================
// Quaternion Structure
// ============================================================================

struct Quaternion {
    float w;  // Scalar part
    float x;  // i component
    float y;  // j component
    float z;  // k component
    
    // Constructors
    Quaternion() : w(1.0f), x(0.0f), y(0.0f), z(0.0f) {}
    Quaternion(float w_, float x_, float y_, float z_) : w(w_), x(x_), y(y_), z(z_) {}
    
    // Create from axis-angle representation
    static Quaternion fromAxisAngle(float ax, float ay, float az, float angle);
    
    // Create from Euler angles (roll, pitch, yaw in radians)
    static Quaternion fromEuler(float roll, float pitch, float yaw);
    
    // Create from rotation matrix
    static Quaternion fromRotationMatrix(float R[3][3]);
};

// ============================================================================
// Quaternion Operations
// ============================================================================

/**
 * @brief Compute quaternion magnitude/norm
 * @param q Input quaternion
 * @return ||q|| = sqrt(w² + x² + y² + z²)
 */
float quaternion_magnitude(const Quaternion& q);

/**
 * @brief Normalize quaternion to unit length
 * @param q Input quaternion
 * @return Unit quaternion q/||q||
 */
Quaternion quaternion_normalize(const Quaternion& q);

/**
 * @brief Compute quaternion conjugate
 * @param q Input quaternion
 * @return q* = w - xi - yj - zk
 */
Quaternion quaternion_conjugate(const Quaternion& q);

/**
 * @brief Compute quaternion inverse
 * @param q Input quaternion
 * @return q⁻¹ = q*/||q||²
 */
Quaternion quaternion_inverse(const Quaternion& q);

/**
 * @brief Quaternion multiplication (Hamilton product)
 * @param q1 First quaternion
 * @param q2 Second quaternion
 * @return q1 * q2
 * 
 * Note: Quaternion multiplication is NOT commutative!
 * q1 * q2 ≠ q2 * q1 in general
 */
Quaternion quaternion_multiply(const Quaternion& q1, const Quaternion& q2);

/**
 * @brief Rotate a 3D vector by a quaternion
 * @param q Rotation quaternion (should be unit)
 * @param v Input vector [x, y, z]
 * @param result Output rotated vector [x', y', z']
 * 
 * Uses formula: v' = q * v * q⁻¹
 * where v is treated as pure quaternion (0, vx, vy, vz)
 */
void quaternion_rotate_vector(const Quaternion& q, const float v[3], float result[3]);

/**
 * @brief Convert quaternion to Euler angles (intrinsic ZYX)
 * @param q Input quaternion
 * @param roll Output roll angle (rotation about X-axis) in radians
 * @param pitch Output pitch angle (rotation about Y-axis) in radians
 * @param yaw Output yaw angle (rotation about Z-axis) in radians
 */
void quaternion_to_euler(const Quaternion& q, float& roll, float& pitch, float& yaw);

/**
 * @brief Convert quaternion to rotation matrix
 * @param q Input quaternion
 * @param R Output 3x3 rotation matrix
 */
void quaternion_to_rotation_matrix(const Quaternion& q, float R[3][3]);

// ============================================================================
// Advanced Quaternion Operations
// ============================================================================

/**
 * @brief Spherical Linear Interpolation (SLERP)
 * @param q1 Start quaternion
 * @param q2 End quaternion
 * @param t Interpolation parameter [0, 1]
 * @return Interpolated quaternion
 * 
 * Provides smooth interpolation along great circle on unit hypersphere.
 * Essential for smooth trajectory generation.
 * 
 * Formula: slerp(q1, q2, t) = (sin((1-t)θ)/sin(θ))q1 + (sin(tθ)/sin(θ))q2
 * where θ = cos⁻¹(q1 · q2)
 */
Quaternion quaternion_slerp(const Quaternion& q1, const Quaternion& q2, float t);

/**
 * @brief Quaternion dot product
 * @param q1 First quaternion
 * @param q2 Second quaternion
 * @return q1 · q2 = w1*w2 + x1*x2 + y1*y2 + z1*z2
 */
float quaternion_dot(const Quaternion& q1, const Quaternion& q2);

/**
 * @brief Quaternion exponential
 * @param q Input quaternion
 * @return exp(q)
 * 
 * Used in advanced operations like integration on manifold
 */
Quaternion quaternion_exp(const Quaternion& q);

/**
 * @brief Quaternion logarithm
 * @param q Input quaternion (should be unit)
 * @return log(q)
 * 
 * Inverse of exponential map
 */
Quaternion quaternion_log(const Quaternion& q);

/**
 * @brief Compute angular velocity from quaternion derivative
 * @param q Current quaternion orientation
 * @param q_dot Quaternion time derivative
 * @param omega Output angular velocity [ωx, ωy, ωz] in body frame
 * 
 * Formula: ω = 2 * q⁻¹ * q̇ (vector part)
 */
void quaternion_to_angular_velocity(const Quaternion& q, const Quaternion& q_dot, 
                                     float omega[3]);

/**
 * @brief Integrate quaternion with angular velocity
 * @param q Current quaternion
 * @param omega Angular velocity [ωx, ωy, ωz] in rad/s
 * @param dt Time step in seconds
 * @return Updated quaternion
 * 
 * Uses first-order integration:
 * q(t+dt) = q(t) + 0.5 * q(t) * [0, ωx, ωy, ωz] * dt
 */
Quaternion quaternion_integrate(const Quaternion& q, const float omega[3], float dt);

// ============================================================================
// Dual Quaternion Structure (for screw motion)
// ============================================================================

/**
 * @brief Dual quaternion for combined rotation and translation
 * 
 * Dual quaternion: q̂ = q_r + εq_d
 * where ε² = 0 (dual unit), q_r is rotation, q_d encodes translation
 */
struct DualQuaternion {
    Quaternion real;  // Rotation part
    Quaternion dual;  // Translation part
    
    DualQuaternion() {}
    DualQuaternion(const Quaternion& r, const Quaternion& d) : real(r), dual(d) {}
    
    // Create from rotation and translation
    static DualQuaternion fromTransform(const Quaternion& rotation, 
                                        const float translation[3]);
};

/**
 * @brief Apply dual quaternion transformation to point
 * @param dq Dual quaternion (rotation + translation)
 * @param point Input point [x, y, z]
 * @param result Output transformed point [x', y', z']
 */
void dual_quaternion_transform_point(const DualQuaternion& dq, 
                                      const float point[3], 
                                      float result[3]);

// ============================================================================
// Utility Functions
// ============================================================================

/**
 * @brief Check if two quaternions represent the same rotation
 * @param q1 First quaternion
 * @param q2 Second quaternion
 * @param tolerance Comparison tolerance (default 1e-6)
 * @return true if equivalent (considering q and -q are same rotation)
 */
bool quaternion_equal(const Quaternion& q1, const Quaternion& q2, float tolerance = 1e-6f);

/**
 * @brief Compute rotation angle from quaternion
 * @param q Input quaternion
 * @return Rotation angle in radians [0, π]
 */
float quaternion_angle(const Quaternion& q);

/**
 * @brief Extract rotation axis from quaternion
 * @param q Input quaternion
 * @param axis Output rotation axis [x, y, z] (unit vector)
 */
void quaternion_axis(const Quaternion& q, float axis[3]);

/**
 * @brief Print quaternion for debugging
 * @param q Quaternion to print
 * @param label Optional label string
 */
void quaternion_print(const Quaternion& q, const char* label = "Quaternion");

#endif // __QUATERNION_MATH_H__
