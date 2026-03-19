package com.gnss.rtklib.core;

import com.gnss.rtklib.model.Navigation;
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;
import static com.gnss.rtklib.core.Constants.*;

class SignalUtilTest {

    @Test
    void obs2codeL1C() {
        assertEquals(CODE_L1C, SignalUtil.obs2code("1C"));
    }

    @Test
    void code2obsL1C() {
        assertEquals("1C", SignalUtil.code2obs(CODE_L1C));
    }

    @Test
    void code2freqGpsL1C() {
        assertEquals(FREQL1, SignalUtil.code2freq(SYS_GPS, CODE_L1C, 0), 1e-3);
    }

    @Test
    void sat2freqGpsL1C() {
        Navigation nav = new Navigation();
        // sat=1 is GPS PRN 1
        double freq = SignalUtil.sat2freq(1, CODE_L1C, nav);
        assertEquals(FREQL1, freq, 1e-3);
    }

    @Test
    void getcodepriGpsL1CPositive() {
        int pri = SignalUtil.getcodepri(SYS_GPS, CODE_L1C, null);
        assertTrue(pri > 0, "GPS L1C priority should be > 0, got " + pri);
    }

    @Test
    void obs2codeUnknownReturnsNone() {
        assertEquals(CODE_NONE, SignalUtil.obs2code("ZZ"));
    }

    @Test
    void code2obsInvalidReturnsEmpty() {
        assertEquals("", SignalUtil.code2obs(CODE_NONE));
        assertEquals("", SignalUtil.code2obs(MAXCODE + 1));
    }

    @Test
    void code2freqInvalidSystemReturnsZero() {
        assertEquals(0.0, SignalUtil.code2freq(SYS_NONE, CODE_L1C, 0));
    }

    @Test
    void code2freqGpsL2W() {
        assertEquals(FREQL2, SignalUtil.code2freq(SYS_GPS, CODE_L2W, 0), 1e-3);
    }

    @Test
    void code2freqGpsL5I() {
        assertEquals(FREQL5, SignalUtil.code2freq(SYS_GPS, CODE_L5I, 0), 1e-3);
    }
}
