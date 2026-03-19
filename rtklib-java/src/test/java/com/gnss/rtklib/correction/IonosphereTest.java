package com.gnss.rtklib.correction;

import com.gnss.rtklib.core.GTime;
import com.gnss.rtklib.model.Navigation;
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;
import static com.gnss.rtklib.core.Constants.*;

class IonosphereTest {

    @Test
    void klobucharZeroIonParamsGivesSmallDelay() {
        GTime time = GTime.epoch2time(new double[]{2024, 6, 1, 12, 0, 0});
        double[] ion = new double[8]; // all zeros -> defaults will be used
        double[] pos = {45.0 * D2R, -75.0 * D2R, 100.0};
        double[] azel = {0.0, 45.0 * D2R};

        double delay = Ionosphere.klobuchar(time, ion, pos, azel);
        // With default params, should produce a non-trivial but reasonable delay
        assertTrue(delay > 0, "delay should be positive, got " + delay);
        assertTrue(delay < 50, "delay should be < 50 m, got " + delay);
    }

    @Test
    void ionocorrOffReturnsZeroDelayWithVariance() {
        GTime time = GTime.epoch2time(new double[]{2024, 6, 1, 12, 0, 0});
        Navigation nav = new Navigation();
        double[] pos = {45.0 * D2R, -75.0 * D2R, 100.0};
        double[] azel = {0.0, 45.0 * D2R};

        double[] result = Ionosphere.ionocorr(time, nav, 1, pos, azel, IONOOPT_OFF);
        assertNotNull(result);
        assertEquals(0.0, result[0], 1e-12, "delay should be 0 when IONOOPT_OFF");
        // ERR_ION = 5.0, var = 5^2 = 25
        assertEquals(25.0, result[1], 1e-6, "variance should be ERR_ION^2");
    }
}
