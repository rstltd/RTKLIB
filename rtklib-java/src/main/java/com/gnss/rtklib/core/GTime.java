package com.gnss.rtklib.core;

import java.util.Objects;

/**
 * GNSS time representation corresponding to C RTKLIB's gtime_t.
 * Immutable. Stores seconds since 1970-01-01 00:00:00 UTC (time) plus
 * fractional second (sec), with 0 <= sec < 1.
 */
public final class GTime {

    /** Seconds since 1970-01-01 00:00:00 UTC (integer part). */
    public final long time;

    /** Fractional second [0, 1). */
    public final double sec;

    /* ---- time system reference epochs ---- */
    private static final double[] GPST0 = {1980, 1,  6, 0, 0, 0}; // GPS time
    private static final double[] GST0  = {1999, 8, 22, 0, 0, 0}; // Galileo
    private static final double[] BDT0  = {2006, 1,  1, 0, 0, 0}; // BeiDou

    /* ---- leap seconds table (y,m,d,h,m,s, utc-gpst) newest first ---- */
    private static final double[][] LEAPS = {
        {2017, 1, 1, 0, 0, 0, -18},
        {2015, 7, 1, 0, 0, 0, -17},
        {2012, 7, 1, 0, 0, 0, -16},
        {2009, 1, 1, 0, 0, 0, -15},
        {2006, 1, 1, 0, 0, 0, -14},
        {1999, 1, 1, 0, 0, 0, -13},
        {1997, 7, 1, 0, 0, 0, -12},
        {1996, 1, 1, 0, 0, 0, -11},
        {1994, 7, 1, 0, 0, 0, -10},
        {1993, 7, 1, 0, 0, 0,  -9},
        {1992, 7, 1, 0, 0, 0,  -8},
        {1991, 1, 1, 0, 0, 0,  -7},
        {1990, 1, 1, 0, 0, 0,  -6},
        {1988, 1, 1, 0, 0, 0,  -5},
        {1985, 7, 1, 0, 0, 0,  -4},
        {1983, 7, 1, 0, 0, 0,  -3},
        {1982, 7, 1, 0, 0, 0,  -2},
        {1981, 7, 1, 0, 0, 0,  -1},
    };

    /* day-of-year table (1-indexed month): day number of first day of each month */
    private static final int[] DOY = {1, 32, 60, 91, 121, 152, 182, 213, 244, 274, 305, 335};

    /* days in each month for a 4-year cycle starting at a non-leap year (1970) */
    private static final int[] MDAY = {
        31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31,  // year +0 (non-leap)
        31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31,  // year +1
        31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31,  // year +2 (leap)
        31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31   // year +3
    };

    // -----------------------------------------------------------------------
    // Constructor
    // -----------------------------------------------------------------------

    /**
     * Create a GTime, normalizing so that 0 <= sec < 1.
     */
    public GTime(long time, double sec) {
        /* normalize: push whole seconds from sec into time */
        double tt = Math.floor(sec);
        this.time = time + (long) tt;
        this.sec  = sec - tt;
    }

    // -----------------------------------------------------------------------
    // epoch2time / time2epoch
    // -----------------------------------------------------------------------

    /**
     * Convert calendar epoch {year,month,day,hour,min,sec} to GTime.
     * Valid for 1970-2099.
     */
    public static GTime epoch2time(double[] ep) {
        int year = (int) ep[0];
        int mon  = (int) ep[1];
        int day  = (int) ep[2];

        if (year < 1970 || year > 2099 || mon < 1 || mon > 12) {
            return new GTime(0, 0.0);
        }

        /* leap year if year%4==0 in 1901-2099 */
        int days = (year - 1970) * 365 + (year - 1969) / 4
                   + DOY[mon - 1] + day - 2
                   + (year % 4 == 0 && mon >= 3 ? 1 : 0);

        int secInt = (int) Math.floor(ep[5]);
        long t = (long) days * 86400L + (int) ep[3] * 3600 + (int) ep[4] * 60 + secInt;
        double frac = ep[5] - secInt;
        return new GTime(t, frac);
    }

    /**
     * Convert this GTime to calendar epoch {year,month,day,hour,min,sec}.
     */
    public double[] time2epoch() {
        double[] ep = new double[6];
        int days = (int) (time / 86400L);
        int secOfDay = (int) (time - (long) days * 86400L);

        int day = days % 1461;
        int mon = 0;
        for (; mon < 48; mon++) {
            if (day >= MDAY[mon]) {
                day -= MDAY[mon];
            } else {
                break;
            }
        }
        ep[0] = 1970 + (days / 1461) * 4 + mon / 12;
        ep[1] = mon % 12 + 1;
        ep[2] = day + 1;
        ep[3] = secOfDay / 3600;
        ep[4] = (secOfDay % 3600) / 60;
        ep[5] = secOfDay % 60 + sec;
        return ep;
    }

    // -----------------------------------------------------------------------
    // GPS week/tow
    // -----------------------------------------------------------------------

    /**
     * Convert GPS week number and time-of-week (s) to GTime.
     */
    public static GTime gpst2time(int week, double sec) {
        GTime t = epoch2time(GPST0);
        if (sec < -1E9 || sec > 1E9) sec = 0.0;
        long wholeSec = (long) sec;
        return new GTime(t.time + 86400L * 7 * week + wholeSec, sec - wholeSec);
    }

    /**
     * Convert this GTime to GPS week and time-of-week.
     * @return double[2]: {week, tow}
     */
    public double[] time2gpst() {
        GTime t0 = epoch2time(GPST0);
        long secDiff = time - t0.time;
        int week = (int) (secDiff / (86400L * 7));
        double tow = (double) (secDiff - (long) week * 86400L * 7) + sec;
        return new double[]{week, tow};
    }

    // -----------------------------------------------------------------------
    // Galileo system time
    // -----------------------------------------------------------------------

    /**
     * Convert Galileo week number and time-of-week (s) to GTime.
     */
    public static GTime gst2time(int week, double sec) {
        GTime t = epoch2time(GST0);
        if (sec < -1E9 || sec > 1E9) sec = 0.0;
        long wholeSec = (long) sec;
        return new GTime(t.time + 86400L * 7 * week + wholeSec, sec - wholeSec);
    }

    /**
     * Convert this GTime to Galileo week and time-of-week.
     * @return double[2]: {week, tow}
     */
    public double[] time2gst() {
        GTime t0 = epoch2time(GST0);
        long secDiff = time - t0.time;
        int week = (int) (secDiff / (86400L * 7));
        double tow = (double) (secDiff - (long) week * 86400L * 7) + sec;
        return new double[]{week, tow};
    }

    // -----------------------------------------------------------------------
    // BeiDou time
    // -----------------------------------------------------------------------

    /**
     * Convert BeiDou week number and time-of-week (s) to GTime.
     */
    public static GTime bdt2time(int week, double sec) {
        GTime t = epoch2time(BDT0);
        if (sec < -1E9 || sec > 1E9) sec = 0.0;
        long wholeSec = (long) sec;
        return new GTime(t.time + 86400L * 7 * week + wholeSec, sec - wholeSec);
    }

    /**
     * Convert this GTime to BeiDou week and time-of-week.
     * @return double[2]: {week, tow}
     */
    public double[] time2bdt() {
        GTime t0 = epoch2time(BDT0);
        long secDiff = time - t0.time;
        int week = (int) (secDiff / (86400L * 7));
        double tow = (double) (secDiff - (long) week * 86400L * 7) + sec;
        return new double[]{week, tow};
    }

    // -----------------------------------------------------------------------
    // GPST <-> BDT  (BDT = GPST - 14s, epoch 2006/1/1)
    // -----------------------------------------------------------------------

    /** Convert this time from GPST to BDT. */
    public GTime gpst2bdt() {
        return add(-14.0);
    }

    /** Convert this time from BDT to GPST. */
    public GTime bdt2gpst() {
        return add(14.0);
    }

    // -----------------------------------------------------------------------
    // timeadd / timediff
    // -----------------------------------------------------------------------

    /**
     * Return a new GTime with the given number of seconds added.
     */
    public GTime add(double addSec) {
        double newSec = sec + addSec;
        double tt = Math.floor(newSec);
        return new GTime(time + (long) tt, newSec - tt);
    }

    /** Alias for {@link #add(double)}. */
    public GTime timeadd(double addSec) {
        return add(addSec);
    }

    /**
     * Compute this minus other in seconds.
     */
    public double diff(GTime other) {
        return (double) (time - other.time) + (sec - other.sec);
    }

    /** Alias for {@link #diff(GTime)}. */
    public double timediff(GTime t) {
        return diff(t);
    }

    /**
     * Convert time to day of year.
     * Matches C RTKLIB's time2doy().
     *
     * @return day of year (1.0 = Jan 1 00:00)
     */
    public double time2doy() {
        double[] ep = time2epoch();
        ep[1] = 1.0; ep[2] = 1.0; ep[3] = 0.0; ep[4] = 0.0; ep[5] = 0.0;
        return diff(epoch2time(ep)) / 86400.0 + 1.0;
    }

    // -----------------------------------------------------------------------
    // UTC <-> GPST
    // -----------------------------------------------------------------------

    /**
     * Convert this time from UTC to GPST using the leap seconds table.
     */
    public GTime utc2gpst() {
        for (double[] leap : LEAPS) {
            GTime leapEpoch = epoch2time(leap); // first 6 elements
            if (diff(leapEpoch) >= 0.0) {
                return add(-leap[6]); // utc-gpst is negative, so subtracting gives gpst
            }
        }
        return this;
    }

    /**
     * Convert this time from GPST to UTC using the leap seconds table.
     */
    public GTime gpst2utc() {
        for (double[] leap : LEAPS) {
            GTime tu = add(leap[6]);
            GTime leapEpoch = epoch2time(leap);
            if (tu.diff(leapEpoch) >= 0.0) {
                return tu;
            }
        }
        return this;
    }

    // -----------------------------------------------------------------------
    // Formatting
    // -----------------------------------------------------------------------

    /**
     * Format as "yyyy/mm/dd hh:mm:ss.sss" with the given number of
     * fractional-second digits.
     */
    public String format(int digits) {
        if (digits < 0) digits = 0;
        if (digits > 12) digits = 12;

        /* round to requested precision — may roll seconds forward */
        GTime rounded = this;
        if (1.0 - sec < 0.5 / Math.pow(10.0, digits)) {
            rounded = new GTime(time + 1, 0.0);
        }
        double[] ep = rounded.time2epoch();

        if (digits == 0) {
            return String.format("%04.0f/%02.0f/%02.0f %02.0f:%02.0f:%02.0f",
                    ep[0], ep[1], ep[2], ep[3], ep[4], ep[5]);
        }
        /* separate integer and fractional seconds for precise formatting */
        int secInt = (int) ep[5];
        double secFrac = ep[5] - secInt;
        String secStr = String.format("%0" + (digits + 3) + "." + digits + "f",
                secInt + secFrac);
        return String.format("%04.0f/%02.0f/%02.0f %02.0f:%02.0f:%s",
                ep[0], ep[1], ep[2], ep[3], ep[4], secStr);
    }

    // -----------------------------------------------------------------------
    // Object overrides
    // -----------------------------------------------------------------------

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (!(o instanceof GTime)) return false;
        GTime g = (GTime) o;
        return time == g.time && Double.compare(sec, g.sec) == 0;
    }

    @Override
    public int hashCode() {
        return Objects.hash(time, sec);
    }

    /**
     * Default toString uses 3 fractional digits.
     */
    @Override
    public String toString() {
        return format(3);
    }
}
