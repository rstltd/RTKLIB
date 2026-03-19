package com.gnss.rtklib.core;

/**
 * Kalman filter measurement update interface.
 * <p>
 * Abstracts the EKF update step so that alternative implementations
 * (e.g., UD factorized, SRIF) can be substituted without modifying
 * positioning code.
 */
public interface KalmanFilter {
    /**
     * Measurement update of state vector and covariance.
     *
     * @param x state vector (n x 1), updated in-place
     * @param P covariance matrix (n x n, column-major), updated in-place
     * @param H design matrix (n x m, column-major)
     * @param v innovation vector (m x 1)
     * @param R measurement noise covariance (m x m, column-major)
     * @param n number of states
     * @param m number of measurements
     * @return 0 on success, non-zero on failure
     */
    int update(double[] x, double[] P, double[] H, double[] v,
               double[] R, int n, int m);
}
