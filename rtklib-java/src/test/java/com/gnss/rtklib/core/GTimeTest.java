package com.gnss.rtklib.core;

import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

class GTimeTest {

    @Test
    void epoch2024Jan1ToGpsWeekAndTow() {
        // 2024-01-01 is a Monday; GPS week 2295 starts on Sunday 2023-12-31
        // so TOW = 1 day = 86400 s
        GTime t = GTime.epoch2time(new double[]{2024, 1, 1, 0, 0, 0});
        double[] wt = t.time2gpst();
        assertEquals(2295, (int) wt[0], "GPS week");
        assertEquals(86400.0, wt[1], 1e-6, "TOW (Monday = 1 day into week)");
    }

    @Test
    void roundTripEpochTimePrecision() {
        double[] epIn = {2024, 6, 15, 12, 30, 45.123456789};
        GTime t = GTime.epoch2time(epIn);
        double[] epOut = t.time2epoch();

        assertEquals(epIn[0], epOut[0], 1e-9, "year");
        assertEquals(epIn[1], epOut[1], 1e-9, "month");
        assertEquals(epIn[2], epOut[2], 1e-9, "day");
        assertEquals(epIn[3], epOut[3], 1e-9, "hour");
        assertEquals(epIn[4], epOut[4], 1e-9, "min");
        assertEquals(epIn[5], epOut[5], 1e-9, "sec");
    }

    @Test
    void gpsWeek0Reference() {
        GTime t = GTime.gpst2time(0, 0);
        double[] ep = t.time2epoch();
        assertEquals(1980, (int) ep[0], "year");
        assertEquals(1, (int) ep[1], "month");
        assertEquals(6, (int) ep[2], "day");
        assertEquals(0, (int) ep[3], "hour");
        assertEquals(0, (int) ep[4], "min");
        assertEquals(0.0, ep[5], 1e-9, "sec");
    }

    @Test
    void addAndDiff() {
        GTime t = GTime.epoch2time(new double[]{2024, 1, 1, 0, 0, 0});
        GTime t2 = t.add(100.0);
        assertEquals(100.0, t2.diff(t), 1e-12);
    }

    @Test
    void utc2gpstRoundTrip() {
        // Use a time well after the last leap second (2017)
        GTime gpst = GTime.epoch2time(new double[]{2024, 6, 1, 0, 0, 0});
        GTime utc = gpst.gpst2utc();
        GTime back = utc.utc2gpst();

        // Round-trip should be exact to sub-nanosecond
        assertEquals(0.0, gpst.diff(back), 1e-9);
    }

    @Test
    void gpst2utcHasLeapSecondOffset() {
        // In 2024, UTC-GPST = -18s, so GPST to UTC subtracts 18s
        GTime gpst = GTime.epoch2time(new double[]{2024, 6, 1, 0, 0, 0});
        GTime utc = gpst.gpst2utc();
        // GPST is ahead of UTC by 18 seconds
        assertEquals(-18.0, utc.diff(gpst), 1e-9);
    }

    @Test
    void bdt2gpstOffset14Seconds() {
        GTime gpst = GTime.epoch2time(new double[]{2024, 6, 1, 0, 0, 0});
        GTime bdt = gpst.gpst2bdt();
        GTime back = bdt.bdt2gpst();

        // BDT = GPST - 14s, so bdt2gpst adds 14s back
        assertEquals(14.0, back.diff(bdt), 1e-12);
        // Round-trip
        assertEquals(0.0, gpst.diff(back), 1e-12);
    }
}
