package com.gnss.rtklib.model;

import com.gnss.rtklib.core.GTime;

import static com.gnss.rtklib.core.Constants.*;

/**
 * Antenna phase center parameters, matching C RTKLIB's pcv_t.
 * Stores PCO (phase center offset) and PCV (phase center variation) per frequency.
 */
public class AntennaModel {

    /** Satellite number (0 for receiver antenna) */
    public int sat;

    /** Antenna type string */
    public String type = "";

    /** Serial number or satellite code */
    public String code = "";

    /** Valid from time */
    public GTime ts;

    /** Valid until time */
    public GTime te;

    /** Phase center offset: [NFREQ][3] in E/N/U (receiver) or X/Y/Z (satellite) (m) */
    public double[][] off = new double[NFREQ][3];

    /** Phase center variation by elevation: [NFREQ][19] at 0-90 deg, 5 deg step (m) */
    public double[][] var = new double[NFREQ][19];
}
