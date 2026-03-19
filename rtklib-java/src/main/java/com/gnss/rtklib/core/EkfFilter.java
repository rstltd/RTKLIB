package com.gnss.rtklib.core;

/**
 * Standard Extended Kalman Filter implementation.
 * Delegates to {@link MatrixUtil#filter(double[], double[], double[], double[], double[], int, int)}.
 */
public final class EkfFilter implements KalmanFilter {

    /** Singleton instance */
    public static final EkfFilter INSTANCE = new EkfFilter();

    private EkfFilter() {}

    @Override
    public int update(double[] x, double[] P, double[] H, double[] v,
                      double[] R, int n, int m) {
        return MatrixUtil.filter(x, P, H, v, R, n, m);
    }
}
