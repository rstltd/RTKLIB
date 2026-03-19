package com.gnss.rtklib.core;

import static com.gnss.rtklib.core.Constants.*;

/**
 * Coordinate conversion functions ported from RTKLIB rtkcmn.c.
 * <p>
 * Geodetic positions are {lat, lon, h} in radians and meters.
 * ECEF positions are {x, y, z} in meters.
 * All rotation matrices are stored column-major as {@code double[9]}.
 */
public final class Coord {

    private Coord() {}

    /**
     * Transform ECEF position to geodetic position (WGS84).
     * <p>
     * Uses iterative method for latitude convergence.
     *
     * @param r ECEF position {x, y, z} (m)
     * @return geodetic position {lat, lon, h} (rad, m)
     */
    public static double[] ecef2pos(double[] r) {
        double e2 = FE_WGS84 * (2.0 - FE_WGS84);
        double r2 = r[0] * r[0] + r[1] * r[1]; // dot2(r, r)
        double z = r[2];
        double zk = 0.0;
        double v = RE_WGS84;
        double sinp;

        while (Math.abs(z - zk) >= 1E-4) {
            zk = z;
            sinp = z / Math.sqrt(r2 + z * z);
            v = RE_WGS84 / Math.sqrt(1.0 - e2 * sinp * sinp);
            z = r[2] + v * e2 * sinp;
        }

        double[] pos = new double[3];
        pos[0] = r2 > 1E-12 ? Math.atan(z / Math.sqrt(r2))
                             : (r[2] > 0.0 ? PI / 2.0 : -PI / 2.0);
        pos[1] = r2 > 1E-12 ? Math.atan2(r[1], r[0]) : 0.0;
        pos[2] = Math.sqrt(r2 + z * z) - v;
        return pos;
    }

    /**
     * Transform geodetic position to ECEF position (WGS84).
     *
     * @param pos geodetic position {lat, lon, h} (rad, m)
     * @return ECEF position {x, y, z} (m)
     */
    public static double[] pos2ecef(double[] pos) {
        double sinp = Math.sin(pos[0]), cosp = Math.cos(pos[0]);
        double sinl = Math.sin(pos[1]), cosl = Math.cos(pos[1]);
        double e2 = FE_WGS84 * (2.0 - FE_WGS84);
        double v = RE_WGS84 / Math.sqrt(1.0 - e2 * sinp * sinp);

        double[] r = new double[3];
        r[0] = (v + pos[2]) * cosp * cosl;
        r[1] = (v + pos[2]) * cosp * sinl;
        r[2] = (v * (1.0 - e2) + pos[2]) * sinp;
        return r;
    }

    /**
     * Compute ECEF-to-local-ENU coordinate transformation matrix.
     * <p>
     * The returned matrix E is stored column-major as {@code double[9]}:
     * <pre>
     *   E[0]=-sinl       E[3]= cosl       E[6]=0
     *   E[1]=-sinp*cosl  E[4]=-sinp*sinl   E[7]=cosp
     *   E[2]= cosp*cosl  E[5]= cosp*sinl   E[8]=sinp
     * </pre>
     * Alternatively, as a row-major 3x3 Java array:
     * <pre>
     *   row 0 (East):  [-sinl,        cosl,       0    ]
     *   row 1 (North): [-sinp*cosl,  -sinp*sinl,  cosp ]
     *   row 2 (Up):    [ cosp*cosl,   cosp*sinl,  sinp ]
     * </pre>
     *
     * @param pos geodetic position {lat, lon} (rad)
     * @return 3x3 rotation matrix (column-major, 9 elements)
     */
    public static double[][] xyz2enu(double[] pos) {
        double sinp = Math.sin(pos[0]), cosp = Math.cos(pos[0]);
        double sinl = Math.sin(pos[1]), cosl = Math.cos(pos[1]);

        // Return as row-major 3x3 Java array for natural Java usage
        return new double[][] {
            { -sinl,       cosl,        0.0  },  // East
            { -sinp * cosl, -sinp * sinl, cosp },  // North
            {  cosp * cosl,  cosp * sinl, sinp }   // Up
        };
    }

    /**
     * Transform ECEF vector to local ENU coordinate.
     *
     * @param pos geodetic position {lat, lon} (rad)
     * @param r   vector in ECEF {x, y, z}
     * @return vector in ENU {e, n, u}
     */
    public static double[] ecef2enu(double[] pos, double[] r) {
        double sinp = Math.sin(pos[0]), cosp = Math.cos(pos[0]);
        double sinl = Math.sin(pos[1]), cosl = Math.cos(pos[1]);

        // E * r  (E is the xyz2enu matrix, applied directly)
        double[] e = new double[3];
        e[0] = -sinl * r[0] + cosl * r[1];
        e[1] = -sinp * cosl * r[0] - sinp * sinl * r[1] + cosp * r[2];
        e[2] =  cosp * cosl * r[0] + cosp * sinl * r[1] + sinp * r[2];
        return e;
    }

    /**
     * Transform covariance from ECEF to local ENU coordinate.
     * <p>
     * Q = E * P * E'   where E is the xyz2enu rotation matrix.
     *
     * @param pos geodetic position {lat, lon} (rad)
     * @param P   covariance in ECEF (3x3, column-major, 9 elements)
     * @param Q   covariance in ENU (3x3, column-major, 9 elements, output)
     */
    public static void covenu(double[] pos, double[] P, double[] Q) {
        // Build column-major E matrix
        double sinp = Math.sin(pos[0]), cosp = Math.cos(pos[0]);
        double sinl = Math.sin(pos[1]), cosl = Math.cos(pos[1]);

        double[] E = new double[9];
        E[0] = -sinl;       E[3] =  cosl;       E[6] = 0.0;
        E[1] = -sinp * cosl; E[4] = -sinp * sinl; E[7] = cosp;
        E[2] =  cosp * cosl; E[5] =  cosp * sinl; E[8] = sinp;

        double[] EP = new double[9];
        MatrixUtil.matmul("NN", 3, 3, 3, E, P, EP);  // EP = E * P
        MatrixUtil.matmul("NT", 3, 3, 3, EP, E, Q);   // Q  = EP * E'
    }
}
