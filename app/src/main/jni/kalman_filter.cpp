/**
 * Kalman Filter Implementation
 */

#include "kalman_filter.h"
#include <cmath>
#include <algorithm>

KalmanFilter::KalmanFilter()
    : m_stateDim(8), m_measureDim(4),
      m_stdWeightPosition(1.0f / 20.0f),
      m_stdWeightVelocity(1.0f / 160.0f) {
}

void KalmanFilter::initiate(std::vector<float>& mean, std::vector<float>& covariance) {
    // Initialize covariance matrix (diagonal)
    // State: [x, y, a, h, vx, vy, va, vh]
    float h = mean[3];

    std::fill(covariance.begin(), covariance.end(), 0.0f);

    // Position variances
    float std_pos = 2.0f * m_stdWeightPosition * h;
    covariance[0 * 8 + 0] = std_pos * std_pos;  // x
    covariance[1 * 8 + 1] = std_pos * std_pos;  // y
    covariance[2 * 8 + 2] = 1e-2f;              // aspect ratio
    covariance[3 * 8 + 3] = std_pos * std_pos;  // h

    // Velocity variances
    float std_vel = 10.0f * m_stdWeightVelocity * h;
    covariance[4 * 8 + 4] = std_vel * std_vel;  // vx
    covariance[5 * 8 + 5] = std_vel * std_vel;  // vy
    covariance[6 * 8 + 6] = 1e-5f;              // va
    covariance[7 * 8 + 7] = std_vel * std_vel;  // vh
}

void KalmanFilter::predict(std::vector<float>& mean, std::vector<float>& covariance) {
    // State transition: x' = F * x
    // F is identity with velocity integration
    // [x]     [1 0 0 0 1 0 0 0] [x ]
    // [y]     [0 1 0 0 0 1 0 0] [y ]
    // [a]  =  [0 0 1 0 0 0 1 0] [a ]
    // [h]     [0 0 0 1 0 0 0 1] [h ]
    // [vx]    [0 0 0 0 1 0 0 0] [vx]
    // [vy]    [0 0 0 0 0 1 0 0] [vy]
    // [va]    [0 0 0 0 0 0 1 0] [va]
    // [vh]    [0 0 0 0 0 0 0 1] [vh]

    // Apply state transition
    mean[0] += mean[4];  // x += vx
    mean[1] += mean[5];  // y += vy
    mean[2] += mean[6];  // a += va
    mean[3] += mean[7];  // h += vh

    // Process noise
    float h = mean[3];
    float std_pos = m_stdWeightPosition * h;
    float std_vel = m_stdWeightVelocity * h;

    // Add process noise to covariance diagonal
    covariance[0 * 8 + 0] += std_pos * std_pos;
    covariance[1 * 8 + 1] += std_pos * std_pos;
    covariance[2 * 8 + 2] += 1e-4f;
    covariance[3 * 8 + 3] += std_pos * std_pos;
    covariance[4 * 8 + 4] += std_vel * std_vel;
    covariance[5 * 8 + 5] += std_vel * std_vel;
    covariance[6 * 8 + 6] += 1e-6f;
    covariance[7 * 8 + 7] += std_vel * std_vel;

    // Update covariance with state transition (simplified: assuming diagonal dominance)
    // Full: P' = F * P * F^T + Q
    // We approximate since F is simple
    for (int i = 0; i < 4; i++) {
        covariance[i * 8 + i] += covariance[(i + 4) * 8 + (i + 4)];
    }
}

void KalmanFilter::update(std::vector<float>& mean, std::vector<float>& covariance,
                          const std::vector<float>& measurement) {
    // Measurement matrix H maps state to measurement
    // H = [1 0 0 0 0 0 0 0]
    //     [0 1 0 0 0 0 0 0]
    //     [0 0 1 0 0 0 0 0]
    //     [0 0 0 1 0 0 0 0]

    // Innovation (measurement residual)
    std::vector<float> y(4);
    y[0] = measurement[0] - mean[0];
    y[1] = measurement[1] - mean[1];
    y[2] = measurement[2] - mean[2];
    y[3] = measurement[3] - mean[3];

    // Measurement noise
    float h = mean[3];
    float std_m = m_stdWeightPosition * h;

    // Innovation covariance: S = H * P * H^T + R
    std::vector<float> S(16);
    S[0 * 4 + 0] = covariance[0 * 8 + 0] + std_m * std_m;
    S[1 * 4 + 1] = covariance[1 * 8 + 1] + std_m * std_m;
    S[2 * 4 + 2] = covariance[2 * 8 + 2] + 1e-2f;
    S[3 * 4 + 3] = covariance[3 * 8 + 3] + std_m * std_m;

    // Kalman gain: K = P * H^T * S^-1
    // Simplified for diagonal S
    std::vector<float> K(32);  // 8x4
    for (int i = 0; i < 8; i++) {
        for (int j = 0; j < 4; j++) {
            K[i * 4 + j] = covariance[i * 8 + j] / S[j * 4 + j];
        }
    }

    // Update state: x = x + K * y
    for (int i = 0; i < 8; i++) {
        for (int j = 0; j < 4; j++) {
            mean[i] += K[i * 4 + j] * y[j];
        }
    }

    // Update covariance: P = (I - K * H) * P
    // Simplified update
    for (int i = 0; i < 8; i++) {
        for (int j = 0; j < 4; j++) {
            covariance[i * 8 + j] *= (1.0f - K[i * 4 + j]);
        }
    }
}

void KalmanFilter::matMul(const std::vector<float>& A, const std::vector<float>& B,
                          std::vector<float>& C, int m, int n, int k) {
    std::fill(C.begin(), C.end(), 0.0f);
    for (int i = 0; i < m; i++) {
        for (int j = 0; j < k; j++) {
            for (int l = 0; l < n; l++) {
                C[i * k + j] += A[i * n + l] * B[l * k + j];
            }
        }
    }
}

void KalmanFilter::matAdd(std::vector<float>& A, const std::vector<float>& B, int size) {
    for (int i = 0; i < size; i++) {
        A[i] += B[i];
    }
}

void KalmanFilter::matTranspose(const std::vector<float>& A, std::vector<float>& At, int m, int n) {
    for (int i = 0; i < m; i++) {
        for (int j = 0; j < n; j++) {
            At[j * m + i] = A[i * n + j];
        }
    }
}

bool KalmanFilter::matInverse4x4(const std::vector<float>& A, std::vector<float>& Ainv) {
    // Simple 4x4 matrix inversion (for diagonal matrices, just invert diagonal)
    // This is a simplification - full inversion would be more complex
    for (int i = 0; i < 4; i++) {
        float val = A[i * 4 + i];
        if (std::abs(val) < 1e-10f) return false;
        Ainv[i * 4 + i] = 1.0f / val;
    }
    return true;
}
