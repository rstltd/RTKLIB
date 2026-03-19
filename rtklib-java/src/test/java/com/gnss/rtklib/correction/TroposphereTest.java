package com.gnss.rtklib.correction;

import com.gnss.rtklib.core.GTime;
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;
import static com.gnss.rtklib.core.Constants.*;

class TroposphereTest {

    @Test
    void saastamoinenSeaLevelZenith() {
        GTime time = GTime.epoch2time(new double[]{2024, 6, 1, 12, 0, 0});
        double[] pos = {45.0 * D2R, 0.0, 0.0}; // sea level
        double[] azel = {0.0, PI / 2.0}; // zenith

        double delay = Troposphere.saastamoinen(time, pos, azel, 0.7);
        // Typical zenith tropospheric delay at sea level is ~2.3 m
        assertTrue(delay > 2.0, "zenith delay should be > 2.0 m, got " + delay);
        assertTrue(delay < 2.8, "zenith delay should be < 2.8 m, got " + delay);
    }

    @Test
    void saastamoinenLowElevationLargerDelay() {
        GTime time = GTime.epoch2time(new double[]{2024, 6, 1, 12, 0, 0});
        double[] pos = {45.0 * D2R, 0.0, 0.0};
        double[] azelZenith = {0.0, PI / 2.0};
        double[] azelLow = {0.0, 10.0 * D2R};

        double delayZenith = Troposphere.saastamoinen(time, pos, azelZenith, 0.7);
        double delayLow = Troposphere.saastamoinen(time, pos, azelLow, 0.7);

        assertTrue(delayLow > delayZenith,
                "low el delay (" + delayLow + ") should exceed zenith (" + delayZenith + ")");
        // At 10 deg elevation, the mapping factor is roughly 1/sin(10)~5.76
        assertTrue(delayLow > 10.0, "at 10 deg el, delay should be > 10 m, got " + delayLow);
    }

    @Test
    void tropcorrOffReturnsZeroDelayWithVariance() {
        GTime time = GTime.epoch2time(new double[]{2024, 6, 1, 12, 0, 0});
        double[] pos = {45.0 * D2R, 0.0, 0.0};
        double[] azel = {0.0, 45.0 * D2R};

        double[] result = Troposphere.tropcorr(time, pos, azel, TROPOPT_OFF);
        assertNotNull(result);
        assertEquals(0.0, result[0], 1e-12, "delay should be 0 when TROPOPT_OFF");
        // ERR_TROP = 3.0, var = 3^2 = 9
        assertEquals(9.0, result[1], 1e-6, "variance should be ERR_TROP^2");
    }
}
