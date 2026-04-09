/**
 * Kalman Filter for Object Tracking
 *
 * Simple implementation for ByteTrack motion model.
 * State: [x, y, a, h, vx, vy, va, vh]
 * Measurement: [x, y, a, h]
 */

#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <vector>

class KalmanFilter {
public:
    KalmanFilter();

    /**
     * Initialize filter with initial state
     */
    void initiate(std::vector<float>& mean, std::vector<float>& covariance);

    /**
     * Predict next state
     */
    void predict(std::vector<float>& mean, std::vector<float>& covariance);

    /**
     * Update state with measurement
     */
    void update(std::vector<float>& mean, std::vector<float>& covariance,
                const std::vector<float>& measurement);

private:
    int m_stateDim;      // 8: [x, y, a, h, vx, vy, va, vh]
    int m_measureDim;    // 4: [x, y, a, h]

    float m_stdWeightPosition;
    float m_stdWeightVelocity;

    // Matrix operations (simple implementations for 8x8 and 4x4 matrices)
    void matMul(const std::vector<float>& A, const std::vector<float>& B,
                std::vector<float>& C, int m, int n, int k);
    void matAdd(std::vector<float>& A, const std::vector<float>& B, int size);
    void matTranspose(const std::vector<float>& A, std::vector<float>& At, int m, int n);
    bool matInverse4x4(const std::vector<float>& A, std::vector<float>& Ainv);
};

#endif // KALMAN_FILTER_H
