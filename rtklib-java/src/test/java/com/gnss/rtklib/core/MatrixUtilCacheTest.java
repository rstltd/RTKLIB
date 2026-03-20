package com.gnss.rtklib.core;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvSource;

import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Tests for matmul cache optimization (NN/NT axpy kernels)
 * and FilterWorkspace equivalence.
 */
class MatrixUtilCacheTest {

    // ---------------------------------------------------------------
    // matmul correctness: compare optimized vs naive reference
    // ---------------------------------------------------------------

    /** Naive j-i-x matmul for reference (always correct, never optimized) */
    private static void matmulNaive(String tr, int n, int k, int m,
                                     double alpha, double[] A, double[] B,
                                     double beta, double[] C) {
        boolean tA = tr.charAt(0) != 'N';
        boolean tB = tr.charAt(1) != 'N';
        for (int j = 0; j < k; j++) {
            for (int i = 0; i < n; i++) {
                double d = 0.0;
                for (int x = 0; x < m; x++) {
                    double aVal = tA ? A[x + i * m] : A[i + x * n];
                    double bVal = tB ? B[j + x * k] : B[x + j * m];
                    d += aVal * bVal;
                }
                C[i + j * n] = alpha * d + beta * C[i + j * n];
            }
        }
    }

    @ParameterizedTest
    @CsvSource({
        "NN, 1.0, 0.0",
        "NN, 2.5, 0.0",
        "NN, 1.0, 1.0",
        "NN, -1.0, 1.0",
        "NN, 0.5, 0.3",
        "NT, 1.0, 0.0",
        "NT, 2.5, 0.0",
        "NT, 1.0, 1.0",
        "NT, -1.0, 1.0",
        "NT, 0.5, 0.3",
        "TN, 1.0, 0.0",
        "TN, 1.0, 1.0",
        "TT, 1.0, 0.0",
        "TT, 1.0, 1.0"
    })
    void matmulMatchesNaive(String tr, double alpha, double beta) {
        int n = 20, k = 15, m = 25;
        Random rng = new Random(42);

        // A storage: tA ? m*n : n*m
        boolean tA = tr.charAt(0) != 'N';
        boolean tB = tr.charAt(1) != 'N';
        double[] A = randomArray(rng, tA ? m * n : n * m);
        double[] B = randomArray(rng, tB ? k * m : m * k);
        double[] Cinit = randomArray(rng, n * k);

        double[] Copt = Cinit.clone();
        double[] Cref = Cinit.clone();

        MatrixUtil.matmul(tr, n, k, m, alpha, A, B, beta, Copt);
        matmulNaive(tr, n, k, m, alpha, A, B, beta, Cref);

        assertArrayEquals(Cref, Copt, 1e-10,
                "matmul " + tr + " alpha=" + alpha + " beta=" + beta);
    }

    @Test
    void matmulNNSmall() {
        // 2x2 * 2x2 identity check
        // A = [[1,3],[2,4]] col-major: {1,2,3,4}
        // B = I = {1,0,0,1}
        double[] A = {1, 2, 3, 4};
        double[] B = {1, 0, 0, 1};
        double[] C = new double[4];
        MatrixUtil.matmul("NN", 2, 2, 2, A, B, C);
        assertArrayEquals(A, C, 1e-15);
    }

    @Test
    void matmulNTSmall() {
        // A * B' where A = {1,2,3,4} (2x2), B = I
        double[] A = {1, 2, 3, 4};
        double[] B = {1, 0, 0, 1};
        double[] C = new double[4];
        MatrixUtil.matmul("NT", 2, 2, 2, A, B, C);
        assertArrayEquals(A, C, 1e-15);
    }

    @Test
    void matmulpAndMatmulmUseOptimized() {
        int n = 10, k = 8, m = 12;
        Random rng = new Random(99);
        double[] A = randomArray(rng, n * m);
        double[] B = randomArray(rng, m * k);
        double[] C1 = randomArray(rng, n * k);
        double[] C2 = C1.clone();

        // matmulp: C += A*B  => alpha=1, beta=1
        MatrixUtil.matmulp("NN", n, k, m, A, B, C1);
        matmulNaive("NN", n, k, m, 1.0, A, B, 1.0, C2);
        assertArrayEquals(C2, C1, 1e-10, "matmulp NN");

        C1 = randomArray(rng, n * k);
        C2 = C1.clone();
        MatrixUtil.matmulm("NT", n, k, m, A, B, C1);
        matmulNaive("NT", n, k, m, -1.0, A, B, 1.0, C2);
        assertArrayEquals(C2, C1, 1e-10, "matmulm NT");
    }

    // ---------------------------------------------------------------
    // FilterWorkspace equivalence
    // ---------------------------------------------------------------

    @Test
    void filterWorkspaceMatchesOriginal() {
        int n = 30, m = 8;
        Random rng = new Random(123);

        // Generate valid state: some zeros, some active
        double[] x = new double[n];
        double[] P = new double[n * n];
        for (int i = 0; i < n; i++) {
            x[i] = (i % 3 == 0) ? 0.0 : rng.nextGaussian();
            P[i + i * n] = (i % 3 == 0) ? 0.0 : Math.abs(rng.nextGaussian()) + 0.01;
        }
        // Fill off-diag P symmetrically
        for (int i = 0; i < n; i++) {
            for (int j = i + 1; j < n; j++) {
                double val = rng.nextGaussian() * 0.01;
                P[i + j * n] = val;
                P[j + i * n] = val;
            }
        }

        double[] H = randomArray(rng, n * m);
        double[] v = randomArray(rng, m);
        double[] R = new double[m * m];
        for (int i = 0; i < m; i++) R[i + i * m] = Math.abs(rng.nextGaussian()) + 1.0;

        // Clone for both paths
        double[] x1 = x.clone(), P1 = P.clone();
        double[] x2 = x.clone(), P2 = P.clone();

        int r1 = MatrixUtil.filter(x1, P1, H.clone(), v.clone(), R.clone(), n, m);
        MatrixUtil.FilterWorkspace ws = new MatrixUtil.FilterWorkspace();
        int r2 = MatrixUtil.filter(x2, P2, H.clone(), v.clone(), R.clone(), n, m, ws);

        assertEquals(r1, r2, "return code mismatch");
        assertArrayEquals(x1, x2, 1e-10, "state vector mismatch");
        assertArrayEquals(P1, P2, 1e-10, "covariance mismatch");
    }

    @Test
    void filterWorkspaceReuse() {
        // Verify workspace can be reused across calls with different sizes
        MatrixUtil.FilterWorkspace ws = new MatrixUtil.FilterWorkspace();

        for (int trial = 0; trial < 3; trial++) {
            int n = 10 + trial * 5;
            int m = 3 + trial;
            Random rng = new Random(200 + trial);

            double[] x = new double[n];
            double[] P = new double[n * n];
            for (int i = 0; i < n; i++) {
                x[i] = rng.nextGaussian();
                P[i + i * n] = Math.abs(rng.nextGaussian()) + 0.1;
            }
            double[] H = randomArray(rng, n * m);
            double[] v = randomArray(rng, m);
            double[] R = new double[m * m];
            for (int i = 0; i < m; i++) R[i + i * m] = Math.abs(rng.nextGaussian()) + 1.0;

            double[] x1 = x.clone(), P1 = P.clone();
            double[] x2 = x.clone(), P2 = P.clone();

            MatrixUtil.filter(x1, P1, H.clone(), v.clone(), R.clone(), n, m);
            MatrixUtil.filter(x2, P2, H.clone(), v.clone(), R.clone(), n, m, ws);

            assertArrayEquals(x1, x2, 1e-10, "trial " + trial + " x");
            assertArrayEquals(P1, P2, 1e-10, "trial " + trial + " P");
        }
    }

    // ---------------------------------------------------------------
    // Helpers
    // ---------------------------------------------------------------

    private static double[] randomArray(Random rng, int len) {
        double[] a = new double[len];
        for (int i = 0; i < len; i++) a[i] = rng.nextGaussian();
        return a;
    }
}
