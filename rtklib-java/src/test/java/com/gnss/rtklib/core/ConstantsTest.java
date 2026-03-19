package com.gnss.rtklib.core;

import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;
import static com.gnss.rtklib.core.Constants.*;

class ConstantsTest {

    @Test
    void maxsatEquals204() {
        assertEquals(204, MAXSAT);
    }

    @Test
    void nsatGps() {
        assertEquals(32, NSATGPS);
    }

    @Test
    void nsatGlo() {
        assertEquals(27, NSATGLO);
    }

    @Test
    void nsatGal() {
        assertEquals(36, NSATGAL);
    }

    @Test
    void nsatQzs() {
        assertEquals(10, NSATQZS);
    }

    @Test
    void nsatCmp() {
        assertEquals(46, NSATCMP);
    }

    @Test
    void nsatIrn() {
        assertEquals(14, NSATIRN);
    }

    @Test
    void nsatSbs() {
        assertEquals(39, NSATSBS);
    }

    @Test
    void nsatLeo() {
        assertEquals(0, NSATLEO);
    }

    @Test
    void maxsatIsSumOfAllConstellations() {
        assertEquals(NSATGPS + NSATGLO + NSATGAL + NSATQZS + NSATCMP
                     + NSATIRN + NSATLEO + NSATSBS, MAXSAT);
    }

    @Test
    void chisqrArrayLength() {
        assertEquals(100, CHISQR.length);
    }

    @Test
    void chisqrFirstElement() {
        assertEquals(10.8, CHISQR[0], 1e-10);
    }
}
