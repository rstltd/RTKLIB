/*------------------------------------------------------------------------------
 * Lambda.java : integer ambiguity resolution
 *
 *          Ported from RTKLIB lambda.c by T.TAKASU
 *          Java port Copyright (C) 2026
 *
 * references:
 *     [1] P.J.G.Teunissen, The least-square ambiguity decorrelation adjustment,
 *         J.Geodesy, Vol.70, 65-82, 1995
 *     [2] X.-W.Chang, X.Yang, T.Zhou, MLAMBDA: A modified LAMBDA method for
 *         integer least-squares estimation, J.Geodesy, Vol.79, 552-565, 2005
 *
 * Licensed under BSD 2-clause license.
 *-----------------------------------------------------------------------------*/
package com.gnss.rtklib.positioning;

import com.gnss.rtklib.core.MatrixUtil;

/**
 * Integer ambiguity resolution using LAMBDA/MLAMBDA.
 * Ported from RTKLIB lambda.c.
 */
public final class Lambda {

    private Lambda() {}

    private static final int LOOPMAX = 10000;

    // ---------------------------------------------------------------
    // LD factorization (Q = L' * diag(D) * L)
    // ---------------------------------------------------------------

    static int LD(int n, double[] Q, double[] L, double[] D) {
        double[] A = new double[n * n];
        System.arraycopy(Q, 0, A, 0, n * n);

        for (int i = n - 1; i >= 0; i--) {
            D[i] = A[i + i * n];
            if (D[i] <= 0.0) return -1;
            double a = Math.sqrt(D[i]);
            for (int j = 0; j <= i; j++) L[i + j * n] = A[i + j * n] / a;
            for (int j = 0; j <= i - 1; j++) {
                for (int k = 0; k <= j; k++) {
                    A[j + k * n] -= L[i + k * n] * L[i + j * n];
                }
            }
            for (int j = 0; j <= i; j++) L[i + j * n] /= L[i + i * n];
        }
        return 0;
    }

    // ---------------------------------------------------------------
    // Integer Gauss transformation
    // ---------------------------------------------------------------

    private static void gauss(int n, double[] L, double[] Z, int i, int j) {
        int mu = (int) Math.floor(L[i + j * n] + 0.5);
        if (mu != 0) {
            for (int k = i; k < n; k++) L[k + n * j] -= (double) mu * L[k + i * n];
            for (int k = 0; k < n; k++) Z[k + n * j] -= (double) mu * Z[k + i * n];
        }
    }

    // ---------------------------------------------------------------
    // Permutations
    // ---------------------------------------------------------------

    private static void perm(int n, double[] L, double[] D, int j, double del, double[] Z) {
        double eta = D[j] / del;
        double lam = D[j + 1] * L[j + 1 + j * n] / del;
        D[j] = eta * D[j + 1];
        D[j + 1] = del;
        for (int k = 0; k <= j - 1; k++) {
            double a0 = L[j + k * n], a1 = L[j + 1 + k * n];
            L[j + k * n] = -L[j + 1 + j * n] * a0 + a1;
            L[j + 1 + k * n] = eta * a0 + lam * a1;
        }
        L[j + 1 + j * n] = lam;
        for (int k = j + 2; k < n; k++) {
            double tmp = L[k + j * n];
            L[k + j * n] = L[k + (j + 1) * n];
            L[k + (j + 1) * n] = tmp;
        }
        for (int k = 0; k < n; k++) {
            double tmp = Z[k + j * n];
            Z[k + j * n] = Z[k + (j + 1) * n];
            Z[k + (j + 1) * n] = tmp;
        }
    }

    // ---------------------------------------------------------------
    // LAMBDA reduction (z=Z'*a, Qz=Z'*Q*Z=L'*diag(D)*L)
    // ---------------------------------------------------------------

    static void reduction(int n, double[] L, double[] D, double[] Z) {
        int j = n - 2, k = n - 2;
        while (j >= 0) {
            if (j <= k) {
                for (int i = j + 1; i < n; i++) gauss(n, L, Z, i, j);
            }
            double del = D[j] + L[j + 1 + j * n] * L[j + 1 + j * n] * D[j + 1];
            if (del + 1E-6 < D[j + 1]) {
                perm(n, L, D, j, del, Z);
                k = j;
                j = n - 2;
            } else {
                j--;
            }
        }
    }

    // ---------------------------------------------------------------
    // MLAMBDA search
    // ---------------------------------------------------------------

    static int search(int n, int m, double[] L, double[] D,
                      double[] zs, double[] zn, double[] s) {
        int nn = 0, imax = 0;
        double maxdist = 1E99;
        double[] S = new double[n * n];
        double[] dist = new double[n];
        double[] zb = new double[n];
        double[] z = new double[n];
        double[] step = new double[n];

        int k = n - 1;
        dist[k] = 0.0;
        zb[k] = zs[k];
        z[k] = Math.floor(zb[k] + 0.5);
        double y = zb[k] - z[k];
        step[k] = y <= 0.0 ? -1.0 : 1.0;

        int c;
        for (c = 0; c < LOOPMAX; c++) {
            double newdist = dist[k] + y * y / D[k];
            if (newdist < maxdist) {
                if (k != 0) {
                    dist[--k] = newdist;
                    for (int i = 0; i <= k; i++) {
                        S[k + i * n] = S[k + 1 + i * n] + (z[k + 1] - zb[k + 1]) * L[k + 1 + i * n];
                    }
                    zb[k] = zs[k] + S[k + k * n];
                    z[k] = Math.floor(zb[k] + 0.5);
                    y = zb[k] - z[k];
                    step[k] = y <= 0.0 ? -1.0 : 1.0;
                } else {
                    if (nn < m) {
                        if (nn == 0 || newdist > s[imax]) imax = nn;
                        for (int i = 0; i < n; i++) zn[i + nn * n] = z[i];
                        s[nn++] = newdist;
                    } else {
                        if (newdist < s[imax]) {
                            for (int i = 0; i < n; i++) zn[i + imax * n] = z[i];
                            s[imax] = newdist;
                            for (int i = imax = 0; i < m; i++) {
                                if (s[imax] < s[i]) imax = i;
                            }
                        }
                        maxdist = s[imax];
                    }
                    z[0] += step[0];
                    y = zb[0] - z[0];
                    step[0] = -step[0] - (step[0] <= 0.0 ? -1.0 : 1.0);
                }
            } else {
                if (k == n - 1) break;
                k++;
                z[k] += step[k];
                y = zb[k] - z[k];
                step[k] = -step[k] - (step[k] <= 0.0 ? -1.0 : 1.0);
            }
        }

        // Sort by s
        for (int i = 0; i < m - 1; i++) {
            for (int j = i + 1; j < m; j++) {
                if (s[i] < s[j]) continue;
                double tmp = s[i]; s[i] = s[j]; s[j] = tmp;
                for (k = 0; k < n; k++) {
                    tmp = zn[k + i * n]; zn[k + i * n] = zn[k + j * n]; zn[k + j * n] = tmp;
                }
            }
        }

        if (c >= LOOPMAX) return -2;
        return 0;
    }

    // ---------------------------------------------------------------
    // Public API: lambda()
    // ---------------------------------------------------------------

    /**
     * LAMBDA/MLAMBDA integer least-square estimation.
     *
     * @param n number of float parameters
     * @param m number of fixed solutions
     * @param a float parameters (n x 1)
     * @param Q covariance matrix (n x n, column-major)
     * @param F fixed solutions (n x m, column-major, output)
     * @param s sum of squared residuals (m x 1, output)
     * @return 0 on success, other on error
     */
    public static int lambda(int n, int m, double[] a, double[] Q,
                              double[] F, double[] s) {
        if (n <= 0 || m <= 0) return -1;

        double[] L = new double[n * n];
        double[] D = new double[n];
        double[] Z = MatrixUtil.eye(n);
        double[] z = new double[n];
        double[] E = new double[n * m];

        int info = LD(n, Q, L, D);
        if (info != 0) return info;

        reduction(n, L, D, Z);
        MatrixUtil.matmul("TN", n, 1, n, Z, a, z);  // z = Z' * a

        info = search(n, m, L, D, z, E, s);
        if (info != 0) return info;

        info = MatrixUtil.solve("T", Z, E, n, m, F);  // F = Z'\E
        return info;
    }
}
