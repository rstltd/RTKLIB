package com.gnss.rtklib.core;

/**
 * Matrix and vector utility functions ported from RTKLIB rtkcmn.c.
 * <p>
 * All matrices are stored as column-major {@code double[]} arrays,
 * matching RTKLIB/Fortran convention: element (i,j) of an n-row matrix
 * is at index {@code A[i + j*n]}.
 */
public final class MatrixUtil {

    private MatrixUtil() {}

    // ---------------------------------------------------------------
    // Vector operations
    // ---------------------------------------------------------------

    /**
     * Dot product of two vectors.
     *
     * @param a first vector
     * @param b second vector
     * @param n number of elements
     * @return a . b
     */
    public static double dot(double[] a, double[] b, int n) {
        double c = 0.0;
        for (int i = n - 1; i >= 0; i--) {
            c += a[i] * b[i];
        }
        return c;
    }

    /**
     * Dot product of two 3-element vectors.
     */
    public static double dot3(double[] a, double[] b) {
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    }

    /**
     * Dot product with offsets into arrays.
     */
    public static double dot(double[] a, int aOff, double[] b, int bOff, int n) {
        double c = 0.0;
        for (int i = 0; i < n; i++) c += a[aOff + i] * b[bOff + i];
        return c;
    }

    /**
     * Euclidean norm of a vector.
     *
     * @param a vector
     * @param n number of elements
     * @return ||a||
     */
    public static double norm(double[] a, int n) {
        return Math.sqrt(dot(a, a, n));
    }

    /**
     * Cross product of two 3-element vectors: c = a x b.
     *
     * @param a input vector (3x1)
     * @param b input vector (3x1)
     * @param c output vector (3x1)
     */
    public static void cross3(double[] a, double[] b, double[] c) {
        c[0] = a[1] * b[2] - a[2] * b[1];
        c[1] = a[2] * b[0] - a[0] * b[2];
        c[2] = a[0] * b[1] - a[1] * b[0];
    }

    // ---------------------------------------------------------------
    // Matrix multiplication
    // ---------------------------------------------------------------

    /**
     * General matrix multiply: C = alpha * op(A) * op(B) + beta * C.
     * <p>
     * {@code tr} controls transposition: "NN", "NT", "TN", or "TT".
     * Dimensions: op(A) is n x m, op(B) is m x k, C is n x k.
     * All matrices are column-major.
     *
     * @param tr    transpose flags (two characters)
     * @param n     rows of op(A) and C
     * @param k     columns of op(B) and C
     * @param m     inner dimension
     * @param alpha scalar multiplier for A*B
     * @param A     left matrix
     * @param B     right matrix
     * @param beta  scalar multiplier for existing C
     * @param C     result matrix (n x k, column-major)
     */
    public static void matmul(String tr, int n, int k, int m,
                              double alpha, double[] A, double[] B,
                              double beta, double[] C) {
        boolean tA = tr.charAt(0) != 'N';
        boolean tB = tr.charAt(1) != 'N';

        if (!tA && !tB) {
            matmulNN(n, k, m, alpha, A, B, beta, C);
        } else if (!tA && tB) {
            matmulNT(n, k, m, alpha, A, B, beta, C);
        } else {
            // TN, TT: j-i-x loop (already cache-friendly for TN)
            for (int j = 0; j < k; j++) {
                for (int i = 0; i < n; i++) {
                    double d = 0.0;
                    for (int x = 0; x < m; x++) {
                        double aVal = A[x + i * m]; // tA: contiguous in x
                        double bVal = tB ? B[j + x * k] : B[x + j * m];
                        d += aVal * bVal;
                    }
                    if (beta == 0.0) C[i + j * n] = alpha * d;
                    else             C[i + j * n] = alpha * d + beta * C[i + j * n];
                }
            }
        }
    }

    /** Cache-optimized NN kernel: column-axpy C[:,j] += alpha * B[x,j] * A[:,x] */
    private static void matmulNN(int n, int k, int m,
                                  double alpha, double[] A, double[] B,
                                  double beta, double[] C) {
        for (int j = 0; j < k; j++) {
            int cOff = j * n;
            if (beta == 0.0) {
                for (int i = 0; i < n; i++) C[cOff + i] = 0.0;
            } else if (beta != 1.0) {
                for (int i = 0; i < n; i++) C[cOff + i] *= beta;
            }
            for (int x = 0; x < m; x++) {
                double bVal = alpha * B[x + j * m];
                int aOff = x * n;
                for (int i = 0; i < n; i++) {
                    C[cOff + i] += bVal * A[aOff + i];
                }
            }
        }
    }

    /** Cache-optimized NT kernel: column-axpy with transposed B */
    private static void matmulNT(int n, int k, int m,
                                  double alpha, double[] A, double[] B,
                                  double beta, double[] C) {
        for (int j = 0; j < k; j++) {
            int cOff = j * n;
            if (beta == 0.0) {
                for (int i = 0; i < n; i++) C[cOff + i] = 0.0;
            } else if (beta != 1.0) {
                for (int i = 0; i < n; i++) C[cOff + i] *= beta;
            }
            for (int x = 0; x < m; x++) {
                double bVal = alpha * B[j + x * k];
                int aOff = x * n;
                for (int i = 0; i < n; i++) {
                    C[cOff + i] += bVal * A[aOff + i];
                }
            }
        }
    }

    /**
     * Simplified matrix multiply: C = A * B (alpha=1, beta=0).
     * <p>
     * Matches RTKLIB's {@code matmul("NN",n,k,m,A,B,C)}.
     *
     * @param tr transpose flags
     * @param n  rows of op(A) and C
     * @param k  columns of op(B) and C
     * @param m  inner dimension
     * @param A  left matrix
     * @param B  right matrix
     * @param C  result matrix
     */
    public static void matmul(String tr, int n, int k, int m,
                              double[] A, double[] B, double[] C) {
        matmul(tr, n, k, m, 1.0, A, B, 0.0, C);
    }

    // ---------------------------------------------------------------
    // Matrix inverse (LU decomposition)
    // ---------------------------------------------------------------

    /**
     * In-place matrix inverse using LU decomposition with partial pivoting.
     *
     * @param A matrix (n x n, column-major) — replaced by A^-1 on success
     * @param n size of matrix
     * @return 0 on success, -1 if singular
     */
    public static int matinv(double[] A, int n) {
        // LU decomposition
        int[] indx = new int[n];
        double[] B = new double[n * n];
        System.arraycopy(A, 0, B, 0, n * n);

        double[] d = {1.0};
        if (ludcmp(B, n, indx, d) != 0) {
            return -1;
        }

        // Solve for each column of the identity
        for (int j = 0; j < n; j++) {
            for (int i = 0; i < n; i++) {
                A[i + j * n] = 0.0;
            }
            A[j + j * n] = 1.0;
            lubksb(B, n, indx, A, j * n);
        }
        return 0;
    }

    /**
     * LU decomposition with partial pivoting.
     *
     * @return 0 on success, -1 if singular
     */
    private static int ludcmp(double[] A, int n, int[] indx, double[] d) {
        double[] vv = new double[n];
        d[0] = 1.0;

        for (int i = 0; i < n; i++) {
            double big = 0.0;
            for (int j = 0; j < n; j++) {
                double tmp = Math.abs(A[i + j * n]);
                if (tmp > big) big = tmp;
            }
            if (big > 0.0) {
                vv[i] = 1.0 / big;
            } else {
                return -1;
            }
        }

        for (int j = 0; j < n; j++) {
            for (int i = 0; i < j; i++) {
                double s = A[i + j * n];
                for (int k = 0; k < i; k++) {
                    s -= A[i + k * n] * A[k + j * n];
                }
                A[i + j * n] = s;
            }
            double big = 0.0;
            int imax = 0;
            for (int i = j; i < n; i++) {
                double s = A[i + j * n];
                for (int k = 0; k < j; k++) {
                    s -= A[i + k * n] * A[k + j * n];
                }
                A[i + j * n] = s;
                double tmp = vv[i] * Math.abs(s);
                if (tmp >= big) {
                    big = tmp;
                    imax = i;
                }
            }
            if (j != imax) {
                for (int k = 0; k < n; k++) {
                    double tmp = A[imax + k * n];
                    A[imax + k * n] = A[j + k * n];
                    A[j + k * n] = tmp;
                }
                d[0] = -d[0];
                vv[imax] = vv[j];
            }
            indx[j] = imax;
            if (A[j + j * n] == 0.0) {
                return -1;
            }
            if (j != n - 1) {
                double tmp = 1.0 / A[j + j * n];
                for (int i = j + 1; i < n; i++) {
                    A[i + j * n] *= tmp;
                }
            }
        }
        return 0;
    }

    /**
     * LU back-substitution.
     *
     * @param A      LU-decomposed matrix
     * @param n      size
     * @param indx   pivot indices
     * @param b      right-hand side / solution vector (stored in array at given offset)
     * @param offset starting index in b
     */
    private static void lubksb(double[] A, int n, int[] indx, double[] b, int offset) {
        int ii = -1;
        for (int i = 0; i < n; i++) {
            int ip = indx[i];
            double s = b[offset + ip];
            b[offset + ip] = b[offset + i];
            if (ii >= 0) {
                for (int j = ii; j < i; j++) {
                    s -= A[i + j * n] * b[offset + j];
                }
            } else if (s != 0.0) {
                ii = i;
            }
            b[offset + i] = s;
        }
        for (int i = n - 1; i >= 0; i--) {
            double s = b[offset + i];
            for (int j = i + 1; j < n; j++) {
                s -= A[i + j * n] * b[offset + j];
            }
            b[offset + i] = s / A[i + i * n];
        }
    }

    // ---------------------------------------------------------------
    // Fixed-interval smoother
    // ---------------------------------------------------------------

    /**
     * Fixed-interval smoother: combine forward and backward filter solutions.
     * Ported from RTKLIB rtkcmn.c:smoother().
     *
     * @param xf forward solution (n x 1)
     * @param Qf forward covariance (n x n, column-major)
     * @param xb backward solution (n x 1)
     * @param Qb backward covariance (n x n, column-major)
     * @param n  number of states
     * @param xs smoothed solution (n x 1, output)
     * @param Qs smoothed covariance (n x n, column-major, output)
     * @return 0 on success, -1 on error
     */
    public static int smoother(double[] xf, double[] Qf, double[] xb,
                                double[] Qb, int n, double[] xs, double[] Qs) {
        double[] invQf = new double[n * n];
        double[] invQb = new double[n * n];
        double[] xx = new double[n];

        System.arraycopy(Qf, 0, invQf, 0, n * n);
        System.arraycopy(Qb, 0, invQb, 0, n * n);

        if (matinv(invQf, n) != 0 || matinv(invQb, n) != 0) return -1;

        for (int i = 0; i < n * n; i++) Qs[i] = invQf[i] + invQb[i];
        if (matinv(Qs, n) != 0) return -1;

        matmul("NN", n, 1, n, invQf, xf, xx);
        matmulp("NN", n, 1, n, invQb, xb, xx);
        matmul("NN", n, 1, n, Qs, xx, xs);

        return 0;
    }

    // ---------------------------------------------------------------
    // Least squares
    // ---------------------------------------------------------------

    /**
     * Least-squares estimation: x = (A*A')^{-1} * A * y.
     * <p>
     * A is n x m (column-major), y is m x 1.
     * On output, x is n x 1, Q = (A*A')^{-1} is n x n.
     *
     * @param A design matrix transpose (n x m, column-major)
     * @param y measurement vector (m x 1)
     * @param n number of parameters
     * @param m number of measurements (must be >= n)
     * @param x estimated parameters (n x 1, output)
     * @param Q covariance of estimated parameters (n x n, column-major, output)
     * @return 0 on success, -1 if m &lt; n or singular
     */
    public static int lsq(double[] A, double[] y, int n, int m,
                           double[] x, double[] Q) {
        if (m < n) return -1;

        double[] Ay = new double[n];
        matmul("NN", n, 1, m, A, y, Ay);       // Ay = A * y
        matmul("NT", n, n, m, A, A, Q);         // Q  = A * A'

        int info = matinv(Q, n);
        if (info == 0) {
            matmul("NN", n, 1, n, Q, Ay, x);   // x  = Q^-1 * Ay
        }
        return info;
    }

    // ---------------------------------------------------------------
    // Matrix multiply variants (C += A*B, C -= A*B)
    // ---------------------------------------------------------------

    /**
     * Matrix multiply accumulate: C += op(A) * op(B).
     * Matches RTKLIB's matmulp().
     */
    public static void matmulp(String tr, int n, int k, int m,
                                double[] A, double[] B, double[] C) {
        matmul(tr, n, k, m, 1.0, A, B, 1.0, C);
    }

    /**
     * Matrix multiply subtract: C -= op(A) * op(B).
     * Matches RTKLIB's matmulm().
     */
    public static void matmulm(String tr, int n, int k, int m,
                                double[] A, double[] B, double[] C) {
        matmul(tr, n, k, m, -1.0, A, B, 1.0, C);
    }

    // ---------------------------------------------------------------
    // Solve linear equation
    // ---------------------------------------------------------------

    /**
     * Solve linear equation: op(A) * X = Y.
     *
     * @param tr "N" for A*X=Y, "T" for A'*X=Y
     * @param A  matrix (n x n, column-major)
     * @param Y  right-hand side (n x m, column-major)
     * @param n  size of A
     * @param m  number of right-hand sides
     * @param X  solution (n x m, column-major, output)
     * @return 0 on success, -1 if singular
     */
    public static int solve(String tr, double[] A, double[] Y, int n, int m,
                             double[] X) {
        double[] B = new double[n * n];
        System.arraycopy(A, 0, B, 0, n * n);
        int info = matinv(B, n);
        if (info == 0) {
            matmul(tr.charAt(0) == 'N' ? "NN" : "TN", n, m, n, B, Y, X);
        }
        return info;
    }

    // ---------------------------------------------------------------
    // Utility: copy, identity, zeros
    // ---------------------------------------------------------------

    /**
     * Copy matrix: dst = src (n x m, column-major).
     */
    public static void matcpy(double[] dst, double[] src, int n, int m) {
        System.arraycopy(src, 0, dst, 0, n * m);
    }

    /**
     * Create identity matrix (n x n, column-major).
     */
    public static double[] eye(int n) {
        double[] I = new double[n * n];
        for (int i = 0; i < n; i++) I[i + i * n] = 1.0;
        return I;
    }

    /**
     * Create zero matrix (n x m, column-major).
     */
    public static double[] zeros(int n, int m) {
        return new double[n * m];
    }

    // ---------------------------------------------------------------
    // Kalman filter measurement update
    // ---------------------------------------------------------------

    /**
     * EKF measurement update (in-place).
     * <p>
     * K = P*H*(H'*P*H+R)^{-1}, x = x+K*v, P = (I-K*H')*P
     * <p>
     * States with x[i]==0 and P[i,i]<=0 are compressed out.
     * Matches RTKLIB's filter() in rtkcmn.c.
     *
     * @param x state vector (n x 1), updated in place
     * @param P covariance matrix (n x n, column-major), updated in place
     * @param H design matrix transpose (n x m, column-major)
     * @param v innovation vector (m x 1)
     * @param R measurement covariance (m x m, column-major)
     * @param n number of states
     * @param m number of measurements
     * @return 0 on success, -1 on error
     */
    public static int filter(double[] x, double[] P, double[] H, double[] v,
                              double[] R, int n, int m) {
        // Build index of non-zero states
        int[] ix = new int[n];
        int k = 0;
        for (int i = 0; i < n; i++) {
            if (x[i] != 0.0 && P[i + i * n] > 0.0) ix[k++] = i;
        }

        // Compress arrays
        double[] x_ = new double[k];
        double[] xp_ = new double[k];
        double[] P_ = new double[k * k];
        double[] Pp_ = new double[k * k];
        double[] H_ = new double[k * m];

        for (int i = 0; i < k; i++) {
            x_[i] = x[ix[i]];
            for (int j = 0; j < k; j++) P_[i + j * k] = P[ix[i] + ix[j] * n];
            for (int j = 0; j < m; j++) H_[i + j * k] = H[ix[i] + j * n];
        }

        // Run filter on compressed arrays
        int info = filter_(x_, P_, H_, v, R, k, m, xp_, Pp_);

        // Copy back
        for (int i = 0; i < k; i++) {
            x[ix[i]] = xp_[i];
            for (int j = 0; j < k; j++) P[ix[i] + ix[j] * n] = Pp_[i + j * k];
        }
        return info;
    }

    /**
     * Core EKF update (no compression).
     */
    private static int filter_(double[] x, double[] P, double[] H, double[] v,
                                double[] R, int n, int m,
                                double[] xp, double[] Pp) {
        double[] F = new double[n * m];
        double[] Q = new double[m * m];
        double[] K = new double[n * m];
        double[] I = eye(n);

        System.arraycopy(R, 0, Q, 0, m * m);
        System.arraycopy(x, 0, xp, 0, n);

        matmul("NN", n, m, n, P, H, F);          // F = P*H
        matmulp("TN", m, m, n, H, F, Q);          // Q = H'*P*H + R

        int info = matinv(Q, m);
        if (info != 0) return info;

        matmul("NN", n, m, m, F, Q, K);           // K = P*H*Q^-1
        matmulp("NN", n, 1, m, K, v, xp);          // xp = x + K*v
        matmulm("NT", n, n, m, K, H, I);           // I = I - K*H'
        matmul("NN", n, n, n, I, P, Pp);            // Pp = (I-K*H')*P

        return 0;
    }

    // ---------------------------------------------------------------
    // Workspace-aware filter (pre-allocated arrays)
    // ---------------------------------------------------------------

    /** Pre-allocated workspace for filter update to eliminate per-epoch GC. */
    public static class FilterWorkspace {
        double[] F, Q, K, I_;
        double[] x_, xp_, P_, Pp_, H_;
        int[] ix;
        int capN, capM;

        public void ensureCapacity(int n, int m) {
            if (n <= capN && m <= capM) return;
            capN = Math.max(n, capN);
            capM = Math.max(m, capM);
            F = new double[capN * capM];
            Q = new double[capM * capM];
            K = new double[capN * capM];
            I_ = new double[capN * capN];
            x_ = new double[capN];
            xp_ = new double[capN];
            P_ = new double[capN * capN];
            Pp_ = new double[capN * capN];
            H_ = new double[capN * capM];
            ix = new int[capN];
        }
    }

    /**
     * EKF measurement update with pre-allocated workspace.
     * Functionally identical to {@link #filter(double[], double[], double[], double[], double[], int, int)}
     * but avoids per-call array allocation.
     */
    public static int filter(double[] x, double[] P, double[] H, double[] v,
                              double[] R, int n, int m, FilterWorkspace ws) {
        ws.ensureCapacity(n, m);

        // Build index of non-zero states
        int k = 0;
        for (int i = 0; i < n; i++) {
            if (x[i] != 0.0 && P[i + i * n] > 0.0) ws.ix[k++] = i;
        }

        // Compress arrays into workspace
        for (int i = 0; i < k; i++) {
            ws.x_[i] = x[ws.ix[i]];
            for (int j = 0; j < k; j++) ws.P_[i + j * k] = P[ws.ix[i] + ws.ix[j] * n];
            for (int j = 0; j < m; j++) ws.H_[i + j * k] = H[ws.ix[i] + j * n];
        }

        // Run filter on compressed arrays using workspace
        int info = filter_(ws.x_, ws.P_, ws.H_, v, R, k, m,
                           ws.xp_, ws.Pp_, ws.F, ws.Q, ws.K, ws.I_);

        // Copy back
        for (int i = 0; i < k; i++) {
            x[ws.ix[i]] = ws.xp_[i];
            for (int j = 0; j < k; j++) P[ws.ix[i] + ws.ix[j] * n] = ws.Pp_[i + j * k];
        }
        return info;
    }

    /** Core EKF update using pre-allocated work arrays. */
    private static int filter_(double[] x, double[] P, double[] H, double[] v,
                                double[] R, int n, int m,
                                double[] xp, double[] Pp,
                                double[] F, double[] Q, double[] K, double[] I) {
        java.util.Arrays.fill(F, 0, n * m, 0.0);
        System.arraycopy(R, 0, Q, 0, m * m);
        System.arraycopy(x, 0, xp, 0, n);

        // I = identity(n)
        java.util.Arrays.fill(I, 0, n * n, 0.0);
        for (int i = 0; i < n; i++) I[i + i * n] = 1.0;

        matmul("NN", n, m, n, P, H, F);          // F = P*H
        matmulp("TN", m, m, n, H, F, Q);          // Q = H'*P*H + R

        int info = matinv(Q, m);
        if (info != 0) return info;

        matmul("NN", n, m, m, F, Q, K);           // K = P*H*Q^-1
        matmulp("NN", n, 1, m, K, v, xp);          // xp = x + K*v
        matmulm("NT", n, n, m, K, H, I);           // I = I - K*H'
        matmul("NN", n, n, n, I, P, Pp);            // Pp = (I-K*H')*P

        return 0;
    }
}
