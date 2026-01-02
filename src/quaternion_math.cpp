#include "../include/quaternion_math.h"

// ============================================================================
// Static Factory Methods
// ============================================================================

Quaternion Quaternion::fromAxisAngle(float ax, float ay, float az, float angle) {
    float half_angle = angle * 0.5f;
    float s = sin(half_angle);
    float c = cos(half_angle);
    
    // Normalize axis
    float norm = sqrt(ax*ax + ay*ay + az*az);
    if (norm < 1e-8f) {
        return Quaternion(1.0f, 0.0f, 0.0f, 0.0f);  // Identity
    }
    ax /= norm;
    ay /= norm;
    az /= norm;
    
    return Quaternion(c, ax*s, ay*s, az*s);
}

Quaternion Quaternion::fromEuler(float roll, float pitch, float yaw) {
    // ZYX intrinsic rotation (yaw-pitch-roll)
    float cr = cos(roll * 0.5f);
    float sr = sin(roll * 0.5f);
    float cp = cos(pitch * 0.5f);
    float sp = sin(pitch * 0.5f);
    float cy = cos(yaw * 0.5f);
    float sy = sin(yaw * 0.5f);
    
    Quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;
    
    return q;
}

Quaternion Quaternion::fromRotationMatrix(float R[3][3]) {
    float trace = R[0][0] + R[1][1] + R[2][2];
    Quaternion q;
    
    if (trace > 0.0f) {
        float s = 0.5f / sqrt(trace + 1.0f);
        q.w = 0.25f / s;
        q.x = (R[2][1] - R[1][2]) * s;
        q.y = (R[0][2] - R[2][0]) * s;
        q.z = (R[1][0] - R[0][1]) * s;
    } else if (R[0][0] > R[1][1] && R[0][0] > R[2][2]) {
        float s = 2.0f * sqrt(1.0f + R[0][0] - R[1][1] - R[2][2]);
        q.w = (R[2][1] - R[1][2]) / s;
        q.x = 0.25f * s;
        q.y = (R[0][1] + R[1][0]) / s;
        q.z = (R[0][2] + R[2][0]) / s;
    } else if (R[1][1] > R[2][2]) {
        float s = 2.0f * sqrt(1.0f + R[1][1] - R[0][0] - R[2][2]);
        q.w = (R[0][2] - R[2][0]) / s;
        q.x = (R[0][1] + R[1][0]) / s;
        q.y = 0.25f * s;
        q.z = (R[1][2] + R[2][1]) / s;
    } else {
        float s = 2.0f * sqrt(1.0f + R[2][2] - R[0][0] - R[1][1]);
        q.w = (R[1][0] - R[0][1]) / s;
        q.x = (R[0][2] + R[2][0]) / s;
        q.y = (R[1][2] + R[2][1]) / s;
        q.z = 0.25f * s;
    }
    
    return q;
}

// ============================================================================
// Basic Operations
// ============================================================================

float quaternion_magnitude(const Quaternion& q) {
    return sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
}

Quaternion quaternion_normalize(const Quaternion& q) {
    float mag = quaternion_magnitude(q);
    if (mag < 1e-8f) {
        return Quaternion(1.0f, 0.0f, 0.0f, 0.0f);  // Return identity
    }
    return Quaternion(q.w/mag, q.x/mag, q.y/mag, q.z/mag);
}

Quaternion quaternion_conjugate(const Quaternion& q) {
    return Quaternion(q.w, -q.x, -q.y, -q.z);
}

Quaternion quaternion_inverse(const Quaternion& q) {
    float mag_sq = q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z;
    if (mag_sq < 1e-8f) {
        return Quaternion(1.0f, 0.0f, 0.0f, 0.0f);
    }
    return Quaternion(q.w/mag_sq, -q.x/mag_sq, -q.y/mag_sq, -q.z/mag_sq);
}

Quaternion quaternion_multiply(const Quaternion& q1, const Quaternion& q2) {
    Quaternion result;
    result.w = q1.w*q2.w - q1.x*q2.x - q1.y*q2.y - q1.z*q2.z;
    result.x = q1.w*q2.x + q1.x*q2.w + q1.y*q2.z - q1.z*q2.y;
    result.y = q1.w*q2.y - q1.x*q2.z + q1.y*q2.w + q1.z*q2.x;
    result.z = q1.w*q2.z + q1.x*q2.y - q1.y*q2.x + q1.z*q2.w;
    return result;
}

void quaternion_rotate_vector(const Quaternion& q, const float v[3], float result[3]) {
    // Create pure quaternion from vector
    Quaternion v_quat(0.0f, v[0], v[1], v[2]);
    
    // Compute q * v * q^-1
    Quaternion q_conj = quaternion_conjugate(q);
    Quaternion temp = quaternion_multiply(q, v_quat);
    Quaternion rotated = quaternion_multiply(temp, q_conj);
    
    result[0] = rotated.x;
    result[1] = rotated.y;
    result[2] = rotated.z;
}

void quaternion_to_euler(const Quaternion& q, float& roll, float& pitch, float& yaw) {
    // Roll (x-axis rotation)
    float sinr_cosp = 2.0f * (q.w * q.x + q.y * q.z);
    float cosr_cosp = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
    roll = atan2(sinr_cosp, cosr_cosp);
    
    // Pitch (y-axis rotation)
    float sinp = 2.0f * (q.w * q.y - q.z * q.x);
    if (fabs(sinp) >= 1.0f)
        pitch = copysign(M_PI / 2.0f, sinp);  // Use 90 degrees if out of range
    else
        pitch = asin(sinp);
    
    // Yaw (z-axis rotation)
    float siny_cosp = 2.0f * (q.w * q.z + q.x * q.y);
    float cosy_cosp = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
    yaw = atan2(siny_cosp, cosy_cosp);
}

void quaternion_to_rotation_matrix(const Quaternion& q, float R[3][3]) {
    float xx = q.x * q.x;
    float xy = q.x * q.y;
    float xz = q.x * q.z;
    float xw = q.x * q.w;
    float yy = q.y * q.y;
    float yz = q.y * q.z;
    float yw = q.y * q.w;
    float zz = q.z * q.z;
    float zw = q.z * q.w;
    
    R[0][0] = 1.0f - 2.0f * (yy + zz);
    R[0][1] = 2.0f * (xy - zw);
    R[0][2] = 2.0f * (xz + yw);
    
    R[1][0] = 2.0f * (xy + zw);
    R[1][1] = 1.0f - 2.0f * (xx + zz);
    R[1][2] = 2.0f * (yz - xw);
    
    R[2][0] = 2.0f * (xz - yw);
    R[2][1] = 2.0f * (yz + xw);
    R[2][2] = 1.0f - 2.0f * (xx + yy);
}

// ============================================================================
// Advanced Operations
// ============================================================================

float quaternion_dot(const Quaternion& q1, const Quaternion& q2) {
    return q1.w*q2.w + q1.x*q2.x + q1.y*q2.y + q1.z*q2.z;
}

Quaternion quaternion_slerp(const Quaternion& q1, const Quaternion& q2, float t) {
    Quaternion q2_adjusted = q2;
    
    // Compute dot product
    float dot = quaternion_dot(q1, q2);
    
    // If dot < 0, negate q2 to take shorter path
    if (dot < 0.0f) {
        q2_adjusted.w = -q2.w;
        q2_adjusted.x = -q2.x;
        q2_adjusted.y = -q2.y;
        q2_adjusted.z = -q2.z;
        dot = -dot;
    }
    
    // If quaternions are very close, use linear interpolation
    if (dot > 0.9995f) {
        float w = q1.w + t * (q2_adjusted.w - q1.w);
        float x = q1.x + t * (q2_adjusted.x - q1.x);
        float y = q1.y + t * (q2_adjusted.y - q1.y);
        float z = q1.z + t * (q2_adjusted.z - q1.z);
        return quaternion_normalize(Quaternion(w, x, y, z));
    }
    
    // Perform spherical linear interpolation
    float theta = acos(dot);
    float sin_theta = sin(theta);
    float w1 = sin((1.0f - t) * theta) / sin_theta;
    float w2 = sin(t * theta) / sin_theta;
    
    return Quaternion(
        q1.w * w1 + q2_adjusted.w * w2,
        q1.x * w1 + q2_adjusted.x * w2,
        q1.y * w1 + q2_adjusted.y * w2,
        q1.z * w1 + q2_adjusted.z * w2
    );
}

Quaternion quaternion_exp(const Quaternion& q) {
    float theta = sqrt(q.x*q.x + q.y*q.y + q.z*q.z);
    
    if (theta < 1e-8f) {
        return Quaternion(exp(q.w), 0.0f, 0.0f, 0.0f);
    }
    
    float exp_w = exp(q.w);
    float coeff = exp_w * sin(theta) / theta;
    
    return Quaternion(
        exp_w * cos(theta),
        q.x * coeff,
        q.y * coeff,
        q.z * coeff
    );
}

Quaternion quaternion_log(const Quaternion& q) {
    float mag = quaternion_magnitude(q);
    if (mag < 1e-8f) {
        return Quaternion(0.0f, 0.0f, 0.0f, 0.0f);
    }
    
    float theta = acos(q.w / mag);
    float sin_theta = sin(theta);
    
    if (fabs(sin_theta) < 1e-8f) {
        return Quaternion(log(mag), 0.0f, 0.0f, 0.0f);
    }
    
    float coeff = theta / sin_theta;
    
    return Quaternion(
        log(mag),
        q.x * coeff,
        q.y * coeff,
        q.z * coeff
    );
}

void quaternion_to_angular_velocity(const Quaternion& q, const Quaternion& q_dot, 
                                     float omega[3]) {
    // ω = 2 * q^-1 * q_dot (vector part)
    Quaternion q_inv = quaternion_inverse(q);
    Quaternion omega_quat = quaternion_multiply(q_inv, q_dot);
    
    omega[0] = 2.0f * omega_quat.x;
    omega[1] = 2.0f * omega_quat.y;
    omega[2] = 2.0f * omega_quat.z;
}

Quaternion quaternion_integrate(const Quaternion& q, const float omega[3], float dt) {
    // q_dot = 0.5 * q * [0, ωx, ωy, ωz]
    Quaternion omega_quat(0.0f, omega[0], omega[1], omega[2]);
    Quaternion q_dot = quaternion_multiply(q, omega_quat);
    
    // Scale by 0.5 * dt
    q_dot.w *= 0.5f * dt;
    q_dot.x *= 0.5f * dt;
    q_dot.y *= 0.5f * dt;
    q_dot.z *= 0.5f * dt;
    
    // Integrate: q_new = q + q_dot
    Quaternion q_new(
        q.w + q_dot.w,
        q.x + q_dot.x,
        q.y + q_dot.y,
        q.z + q_dot.z
    );
    
    // Normalize to prevent drift
    return quaternion_normalize(q_new);
}

// ============================================================================
// Dual Quaternion Operations
// ============================================================================

DualQuaternion DualQuaternion::fromTransform(const Quaternion& rotation, 
                                              const float translation[3]) {
    // dual part: q_d = 0.5 * t * q_r
    Quaternion t_quat(0.0f, translation[0], translation[1], translation[2]);
    Quaternion q_d = quaternion_multiply(t_quat, rotation);
    q_d.w *= 0.5f;
    q_d.x *= 0.5f;
    q_d.y *= 0.5f;
    q_d.z *= 0.5f;
    
    return DualQuaternion(rotation, q_d);
}

void dual_quaternion_transform_point(const DualQuaternion& dq, 
                                      const float point[3], 
                                      float result[3]) {
    // First apply rotation
    quaternion_rotate_vector(dq.real, point, result);
    
    // Extract translation from dual part: t = 2 * q_d * q_r*
    Quaternion q_r_conj = quaternion_conjugate(dq.real);
    Quaternion t_quat = quaternion_multiply(dq.dual, q_r_conj);
    
    // Apply translation
    result[0] += 2.0f * t_quat.x;
    result[1] += 2.0f * t_quat.y;
    result[2] += 2.0f * t_quat.z;
}

// ============================================================================
// Utility Functions
// ============================================================================

bool quaternion_equal(const Quaternion& q1, const Quaternion& q2, float tolerance) {
    float dot = quaternion_dot(q1, q2);
    return fabs(fabs(dot) - 1.0f) < tolerance;
}

float quaternion_angle(const Quaternion& q) {
    return 2.0f * acos(fabs(q.w));
}

void quaternion_axis(const Quaternion& q, float axis[3]) {
    float sin_half_angle = sqrt(1.0f - q.w * q.w);
    
    if (sin_half_angle < 1e-8f) {
        axis[0] = 1.0f;
        axis[1] = 0.0f;
        axis[2] = 0.0f;
        return;
    }
    
    axis[0] = q.x / sin_half_angle;
    axis[1] = q.y / sin_half_angle;
    axis[2] = q.z / sin_half_angle;
}

void quaternion_print(const Quaternion& q, const char* label) {
    Serial.print(label);
    Serial.print(": [");
    Serial.print(q.w, 4);
    Serial.print(", ");
    Serial.print(q.x, 4);
    Serial.print(", ");
    Serial.print(q.y, 4);
    Serial.print(", ");
    Serial.print(q.z, 4);
    Serial.println("]");
}
