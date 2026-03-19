package com.gnss.rtklib.model;

import com.gnss.rtklib.core.GTime;

/**
 * Position solution, matching C RTKLIB's sol_t.
 */
public class Solution {

    /** Solution time (GPST) */
    public GTime time = new GTime(0, 0.0);

    /** Time of event (GPST) */
    public GTime eventime = new GTime(0, 0.0);

    /**
     * Position/velocity (m | m/s).
     * {x,y,z,vx,vy,vz} or {e,n,u,ve,vn,vu}.
     */
    public double[] rr = new double[6];

    /**
     * Position variance/covariance (m^2).
     * {c_xx,c_yy,c_zz,c_xy,c_yz,c_zx}.
     */
    public float[] qr = new float[6];

    /** Velocity variance/covariance (m^2/s^2) */
    public float[] qv = new float[6];

    /** Receiver clock bias to time systems (s) */
    public double[] dtr = new double[6];

    /** Type: 0=xyz-ecef, 1=enu-baseline */
    public int type;

    /** Solution status (SOLQ_xxx) */
    public int stat;

    /** Number of valid satellites */
    public int ns;

    /** Age of differential (s) */
    public float age;

    /** AR ratio factor for validation */
    public float ratio;

    /** Previous initial AR ratio */
    public float prevRatio1;

    /** Previous final AR ratio */
    public float prevRatio2;

    /** AR ratio threshold for validation */
    public float thres;

    /** Reference station ID */
    public int refStationId;

    public Solution() {
    }

    /** Deep copy of this solution. */
    public Solution copy() {
        Solution s = new Solution();
        s.time = this.time;
        s.eventime = this.eventime;
        System.arraycopy(this.rr, 0, s.rr, 0, 6);
        System.arraycopy(this.qr, 0, s.qr, 0, 6);
        System.arraycopy(this.qv, 0, s.qv, 0, 6);
        System.arraycopy(this.dtr, 0, s.dtr, 0, 6);
        s.type = this.type;
        s.stat = this.stat;
        s.ns = this.ns;
        s.age = this.age;
        s.ratio = this.ratio;
        s.prevRatio1 = this.prevRatio1;
        s.prevRatio2 = this.prevRatio2;
        s.thres = this.thres;
        s.refStationId = this.refStationId;
        return s;
    }
}
