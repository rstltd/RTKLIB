package com.gnss.rtklib.model;

import com.gnss.rtklib.core.EkfFilter;
import com.gnss.rtklib.core.KalmanFilter;

import static com.gnss.rtklib.core.Constants.*;

/**
 * Abstract base class for positioning filter state.
 * <p>
 * Holds the common state vector, covariance, and metadata shared by
 * both RTK ({@link RtkState}) and PPP ({@link PppState}) positioning modes.
 */
public abstract class FilterState {

    /** Total number of states */
    public int nx;

    /** Number of real (non-ambiguity) states */
    public int na;

    /** Float state vector (nx x 1) */
    public double[] x;

    /** Float covariance (nx x nx, column-major) */
    public double[] P;

    /** Fixed state vector (na x 1) */
    public double[] xa;

    /** Fixed covariance (na x na, column-major) */
    public double[] Pa;

    /** Time difference between current and previous epoch (s) */
    public double tt;

    /** Epoch counter */
    public int epoch;

    /** Number of continuous fixes */
    public int nfix;

    /** Current solution */
    public Solution sol = new Solution();

    /** Processing options */
    public ProcessingOptions opt;

    /** Kalman filter implementation */
    public KalmanFilter filter = EkfFilter.INSTANCE;

    /**
     * Initialize state and covariance for state i.
     * Sets x[i] = xi, clears row/column i of P, sets P[i,i] = var.
     */
    public void initx(double xi, double var, int i) {
        x[i] = xi;
        for (int j = 0; j < nx; j++) {
            P[i + j * nx] = 0.0;
            P[j + i * nx] = 0.0;
        }
        P[i + i * nx] = var;
    }

    /** Number of frequencies (NF) — same formula for RTK and PPP */
    public static int NF(ProcessingOptions opt) {
        return opt.ionoopt == IONOOPT_IFLC ? 1 : opt.nf;
    }

    /** Number of position states (NP) — same formula for RTK and PPP */
    public static int NP(ProcessingOptions opt) {
        return opt.dynamics != 0 ? 9 : 3;
    }
}
