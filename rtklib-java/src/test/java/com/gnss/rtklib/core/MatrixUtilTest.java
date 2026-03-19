package com.gnss.rtklib.core;

import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

class MatrixUtilTest {

    @Test
    void dotProduct() {
        assertEquals(32.0, MatrixUtil.dot(new double[]{1, 2, 3}, new double[]{4, 5, 6}, 3), 1e-12);
    }

    @Test
    void normPythagorean() {
        assertEquals(5.0, MatrixUtil.norm(new double[]{3, 4}, 2), 1e-12);
    }

    @Test
    void matinvIdentity() {
        // Column-major 2x2 identity
        double[] I = {1, 0, 0, 1};
        assertEquals(0, MatrixUtil.matinv(I, 2));
        assertEquals(1.0, I[0], 1e-12);
        assertEquals(0.0, I[1], 1e-12);
        assertEquals(0.0, I[2], 1e-12);
        assertEquals(1.0, I[3], 1e-12);
    }

    @Test
    void matinvKnown2x2() {
        // Column-major: A = [[2, 1], [5, 3]]
        // col0 = {2, 5}, col1 = {1, 3}
        double[] A = {2, 5, 1, 3};
        assertEquals(0, MatrixUtil.matinv(A, 2));
        // A^-1 = [[3, -1], [-5, 2]]
        assertEquals(3.0, A[0], 1e-10);
        assertEquals(-5.0, A[1], 1e-10);
        assertEquals(-1.0, A[2], 1e-10);
        assertEquals(2.0, A[3], 1e-10);
    }

    @Test
    void lsqSimpleSystem() {
        // Solve: x0 + 0*x1 = 3, 0*x0 + x1 = 7
        // A (transposed design matrix) is 2x2 column-major: identity
        double[] A = {1, 0, 0, 1};
        double[] y = {3, 7};
        double[] x = new double[2];
        double[] Q = new double[4];

        int ret = MatrixUtil.lsq(A, y, 2, 2, x, Q);
        assertEquals(0, ret);
        assertEquals(3.0, x[0], 1e-10);
        assertEquals(7.0, x[1], 1e-10);
    }

    @Test
    void lsqOverdeterminedSystem() {
        // 3 measurements, 1 unknown: A (1x3) = {1, 1, 1} (column-major)
        // y = {2, 4, 6} -> LSQ solution: mean = 4
        double[] A = {1, 1, 1};
        double[] y = {2, 4, 6};
        double[] x = new double[1];
        double[] Q = new double[1];

        int ret = MatrixUtil.lsq(A, y, 1, 3, x, Q);
        assertEquals(0, ret);
        assertEquals(4.0, x[0], 1e-10);
    }

    @Test
    void smootherBasicFusion() {
        // xf=[1,0,0], Qf=I, xb=[0,1,0], Qb=I
        // Expected: xs=[0.5,0.5,0], Qs=0.5*I
        double[] xf = {1, 0, 0};
        double[] Qf = {1, 0, 0, 0, 1, 0, 0, 0, 1}; // 3x3 identity, column-major
        double[] xb = {0, 1, 0};
        double[] Qb = {1, 0, 0, 0, 1, 0, 0, 0, 1};
        double[] xs = new double[3];
        double[] Qs = new double[9];

        int ret = MatrixUtil.smoother(xf, Qf, xb, Qb, 3, xs, Qs);
        assertEquals(0, ret);
        assertEquals(0.5, xs[0], 1e-12);
        assertEquals(0.5, xs[1], 1e-12);
        assertEquals(0.0, xs[2], 1e-12);
        // Qs = (I^-1 + I^-1)^-1 = 0.5*I
        assertEquals(0.5, Qs[0], 1e-12);
        assertEquals(0.5, Qs[4], 1e-12);
        assertEquals(0.5, Qs[8], 1e-12);
        assertEquals(0.0, Qs[1], 1e-12);
    }

    @Test
    void smootherUnequalCovariance() {
        // Forward: xf=[2], Qf=[1], Backward: xb=[4], Qb=[3]
        // xs = (1/1 + 1/3)^-1 * (1/1*2 + 1/3*4) = (4/3)^-1 * (10/3) = 3/4 * 10/3 = 10/4 = 2.5
        // Qs = (1/1 + 1/3)^-1 = 3/4 = 0.75
        double[] xf = {2}, Qf = {1}, xb = {4}, Qb = {3};
        double[] xs = new double[1], Qs = new double[1];

        int ret = MatrixUtil.smoother(xf, Qf, xb, Qb, 1, xs, Qs);
        assertEquals(0, ret);
        assertEquals(2.5, xs[0], 1e-12);
        assertEquals(0.75, Qs[0], 1e-12);
    }

    @Test
    void smootherSingularReturnsNeg1() {
        double[] xf = {1, 0};
        double[] Qf = {0, 0, 0, 0}; // singular
        double[] xb = {0, 1};
        double[] Qb = {1, 0, 0, 1};
        double[] xs = new double[2];
        double[] Qs = new double[4];

        int ret = MatrixUtil.smoother(xf, Qf, xb, Qb, 2, xs, Qs);
        assertEquals(-1, ret);
    }

    @Test
    void lsqSingularMatrixReturnsNeg1() {
        // Singular: two identical rows -> A*A' is singular
        // A is 2x2 column-major, both rows the same
        double[] A = {1, 1, 0, 0};
        double[] y = {1, 1};
        double[] x = new double[2];
        double[] Q = new double[4];

        int ret = MatrixUtil.lsq(A, y, 2, 2, x, Q);
        assertEquals(-1, ret);
    }
}
