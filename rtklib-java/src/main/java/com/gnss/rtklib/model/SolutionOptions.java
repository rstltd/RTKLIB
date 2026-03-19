package com.gnss.rtklib.model;

import com.gnss.rtklib.core.Constants;

/**
 * Solution output options, matching C RTKLIB's solopt_t.
 * Default values from solopt_default in rtkcmn.c.
 */
public class SolutionOptions {

    /** Solution format (SOLF_xxx) */
    public int posf = Constants.SOLF_LLH;

    /** Time system (TIMES_xxx) */
    public int times = Constants.TIMES_GPST;

    /** Time format (0:sssss.s, 1:yyyy/mm/dd hh:mm:ss.s) */
    public int timef = 1;

    /** Time digits under decimal point */
    public int timeu = 3;

    /** Latitude/longitude format (0:ddd.ddd, 1:ddd mm ss) */
    public int degf = 0;

    /** Output header (0:no, 1:yes) */
    public int outhead = 1;

    /** Output processing options (0:no, 1:yes) */
    public int outopt = 0;

    /** Output velocity (0:no, 1:yes) */
    public int outvel = 0;

    /** Datum (0:WGS84, 1:Tokyo) */
    public int datum = 0;

    /** Height (0:ellipsoidal, 1:geodetic) */
    public int height = 0;

    /** Geoid model (0:EGM96, 1:JGD2000) */
    public int geoid = 0;

    /** Solution of static mode (0:all, 1:single) */
    public int solstatic = 0;

    /** Solution statistics level (0:off, 1:states, 2:residuals) */
    public int sstat = 0;

    /** Debug trace level (0:off, 1-5:debug) */
    public int trace = 0;

    /** NMEA output interval (s) {GPRMC/GPGGA, GPGSV} */
    public double[] nmeaintv = {0.0, 0.0};

    /** Field separator */
    public String separator = " ";

    /** Program name */
    public String prog = "";

    /** Max std-dev for solution output (m), 0=all */
    public double maxsolstd = 0.0;

    public SolutionOptions() {
    }
}
