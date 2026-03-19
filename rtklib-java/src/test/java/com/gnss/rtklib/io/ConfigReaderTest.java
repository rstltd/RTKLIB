package com.gnss.rtklib.io;

import com.gnss.rtklib.core.Constants;
import com.gnss.rtklib.model.FileOptions;
import com.gnss.rtklib.model.ProcessingOptions;
import com.gnss.rtklib.model.SolutionOptions;
import org.junit.jupiter.api.Test;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;

import static com.gnss.rtklib.core.Constants.*;
import static org.junit.jupiter.api.Assertions.*;

class ConfigReaderTest {

    private static final double EPS = 1e-10;

    // ---------------------------------------------------------------
    // Test: load ppp.conf
    // ---------------------------------------------------------------
    @Test
    void testLoadPppConf() throws IOException {
        String confPath = "test/data/ppp/ppp.conf";
        if (!new File(confPath).exists()) {
            confPath = "../test/data/ppp/ppp.conf";
        }
        if (!new File(confPath).exists()) return; // skip if not found

        ProcessingOptions popt = new ProcessingOptions();
        SolutionOptions sopt = new SolutionOptions();
        FileOptions fopt = ConfigReader.load(confPath, popt, sopt);

        // mode = ppp-static (8)
        assertEquals(PMODE_PPP_STATIC, popt.mode);

        // nf = 3 (l1+l2+l5)
        assertEquals(3, popt.nf);

        // soltype = combined-nophasereset (3)
        assertEquals(SOLTYPE_COMBINED_NORESET, popt.soltype);

        // elmin = 15 deg in radians
        assertEquals(15.0 * D2R, popt.elmin, EPS);

        // navsys = 61
        assertEquals(61, popt.navsys);

        // tidecorr = 7
        assertEquals(7, popt.tidecorr);

        // ionoopt = dual-freq (3 = IFLC)
        assertEquals(IONOOPT_IFLC, popt.ionoopt);

        // tropopt = est-ztdgrad (4)
        assertEquals(TROPOPT_ESTG, popt.tropopt);

        // sateph = precise (1)
        assertEquals(EPHOPT_PREC, popt.sateph);

        // dynamics = on (1)
        assertEquals(1, popt.dynamics);

        // SNR mask L1
        assertEquals(1, popt.snrmask.ena[0]); // rover on
        assertEquals(0, popt.snrmask.ena[1]); // base off
        double[] expectedL1 = {36, 35, 34, 33, 32, 31, 30, 29, 28};
        for (int j = 0; j < 9; j++) {
            assertEquals(expectedL1[j], popt.snrmask.mask[0][j], EPS,
                         "L1 SNR mask[" + j + "]");
        }

        // SNR mask L2
        double[] expectedL2 = {34, 33, 32, 31, 30, 29, 28, 27, 26};
        for (int j = 0; j < 9; j++) {
            assertEquals(expectedL2[j], popt.snrmask.mask[1][j], EPS,
                         "L2 SNR mask[" + j + "]");
        }

        // ant2 postype = llh (0), pos = 23.9876888609992, 120.794631778, 211.946600001305
        assertEquals(POSOPT_POS_LLH, popt.refpos);
        // rb should be ECEF converted from LLH
        double[] llh = {23.9876888609992 * D2R, 120.794631778 * D2R, 211.946600001305};
        double[] expectedEcef = com.gnss.rtklib.core.Coord.pos2ecef(llh);
        assertEquals(expectedEcef[0], popt.rb[0], 0.001, "rb[0] ECEF X");
        assertEquals(expectedEcef[1], popt.rb[1], 0.001, "rb[1] ECEF Y");
        assertEquals(expectedEcef[2], popt.rb[2], 0.001, "rb[2] ECEF Z");

        // ant2 anttype
        assertEquals("MBA20", popt.anttype[1]);

        // ant2 antdelu = 0.7
        assertEquals(0.7, popt.antdel[1][2], EPS);

        // stats
        assertEquals(120.0, popt.eratio[0], EPS);
        assertEquals(0.003, popt.err[1], EPS);
        assertEquals(0.01, popt.err[2], EPS);
        assertEquals(30.0, popt.std[0], EPS);
        assertEquals(0.03, popt.std[1], EPS);
        assertEquals(0.3, popt.std[2], EPS);

        // AR
        assertEquals(0, popt.modear); // off
        assertEquals(5, popt.minlock);
        assertEquals(5, popt.niter);
        assertEquals(0.1, popt.maxtdiff, EPS);

        // out
        assertEquals(SOLF_XYZ, sopt.posf);
        assertEquals(0, sopt.outhead); // off
        assertEquals(1, sopt.timef); // hms
    }

    // ---------------------------------------------------------------
    // Test: load static.conf
    // ---------------------------------------------------------------
    @Test
    void testLoadStaticConf() throws IOException {
        String confPath = "test/data/static/static.conf";
        if (!new File(confPath).exists()) {
            confPath = "../test/data/static/static.conf";
        }
        if (!new File(confPath).exists()) return;

        ProcessingOptions popt = new ProcessingOptions();
        SolutionOptions sopt = new SolutionOptions();
        FileOptions fopt = ConfigReader.load(confPath, popt, sopt);

        // mode = static (3)
        assertEquals(PMODE_STATIC, popt.mode);

        // modear = fix-and-hold (3)
        assertEquals(3, popt.modear);

        // ant2 postype = xyz (1)
        assertEquals(POSOPT_POS_XYZ, popt.refpos);

        // rb should be the XYZ values directly
        assertEquals(-3026183.355, popt.rb[0], 0.001, "rb[0]");
        assertEquals(4975933.285, popt.rb[1], 0.001, "rb[1]");
        assertEquals(2598179.547, popt.rb[2], 0.001, "rb[2]");

        // nf = 3
        assertEquals(3, popt.nf);

        // navsys = 127
        assertEquals(127, popt.navsys);

        // stats
        assertEquals(150.0, popt.eratio[0], EPS);
        assertEquals(0.006, popt.err[2], EPS);

        // AR params
        assertEquals(20, popt.minlock);
        assertEquals(0.032, popt.thresslip, EPS);
    }

    // ---------------------------------------------------------------
    // Test: str2enum
    // ---------------------------------------------------------------
    @Test
    void testStr2Enum() {
        // Basic enum lookups
        assertEquals(0, ConfigReader.str2enum("off", "0:off,1:on"));
        assertEquals(1, ConfigReader.str2enum("on", "0:off,1:on"));

        // Mode options
        assertEquals(0, ConfigReader.str2enum("single", "0:single,1:dgps,2:kinematic,3:static"));
        assertEquals(3, ConfigReader.str2enum("static", "0:single,1:dgps,2:kinematic,3:static"));
        assertEquals(8, ConfigReader.str2enum("ppp-static",
                "0:single,1:dgps,2:kinematic,3:static,4:static-start,5:movingbase,6:fixed,7:ppp-kine,8:ppp-static,9:ppp-fixed"));

        // Frequency
        assertEquals(2, ConfigReader.str2enum("l1+l2", "1:l1,2:l1+l2,3:l1+l2+l5"));
        assertEquals(3, ConfigReader.str2enum("l1+l2+l5", "1:l1,2:l1+l2,3:l1+l2+l5"));

        // Solution type
        assertEquals(3, ConfigReader.str2enum("combined-nophasereset",
                "0:forward,1:backward,2:combined,3:combined-nophasereset"));

        // Ionosphere
        assertEquals(3, ConfigReader.str2enum("dual-freq",
                "0:off,1:brdc,2:sbas,3:dual-freq,4:est-stec"));

        // AR mode
        assertEquals(3, ConfigReader.str2enum("fix-and-hold",
                "0:off,1:continuous,2:instantaneous,3:fix-and-hold"));

        // Numeric fallback
        assertEquals(61, ConfigReader.str2enum("61", "1:gps+2:sbas+4:glo"));

        // Phase wind-up
        assertEquals(2, ConfigReader.str2enum("precise", "0:off,1:on,2:precise"));
    }

    // ---------------------------------------------------------------
    // Test: excluded satellites
    // ---------------------------------------------------------------
    @Test
    void testExclSats() throws IOException {
        File tmp = File.createTempFile("conf", ".conf");
        tmp.deleteOnExit();
        try (PrintWriter pw = new PrintWriter(new FileWriter(tmp))) {
            pw.println("pos1-posmode     =single");
            pw.println("pos1-exclsats    =G01 G02 +G03");
        }

        ProcessingOptions popt = new ProcessingOptions();
        SolutionOptions sopt = new SolutionOptions();
        ConfigReader.load(tmp.getAbsolutePath(), popt, sopt);

        // G01 = sat 1, excluded (1)
        assertEquals(1, popt.exsats[0], "G01 excluded");
        // G02 = sat 2, excluded (1)
        assertEquals(1, popt.exsats[1], "G02 excluded");
        // G03 = sat 3, included (2)
        assertEquals(2, popt.exsats[2], "G03 force-included");
        // G04 = sat 4, not set (0)
        assertEquals(0, popt.exsats[3], "G04 not set");
    }

    // ---------------------------------------------------------------
    // Test: SNR mask parsing
    // ---------------------------------------------------------------
    @Test
    void testSnrMaskParsing() throws IOException {
        File tmp = File.createTempFile("conf", ".conf");
        tmp.deleteOnExit();
        try (PrintWriter pw = new PrintWriter(new FileWriter(tmp))) {
            pw.println("pos1-snrmask_L1  =36,35,34,33,32,31,30,29,28");
            pw.println("pos1-snrmask_L2  =10,20,30,40,50,60,70,80,90");
        }

        ProcessingOptions popt = new ProcessingOptions();
        SolutionOptions sopt = new SolutionOptions();
        ConfigReader.load(tmp.getAbsolutePath(), popt, sopt);

        double[] expectedL1 = {36, 35, 34, 33, 32, 31, 30, 29, 28};
        for (int j = 0; j < 9; j++) {
            assertEquals(expectedL1[j], popt.snrmask.mask[0][j], EPS);
        }

        double[] expectedL2 = {10, 20, 30, 40, 50, 60, 70, 80, 90};
        for (int j = 0; j < 9; j++) {
            assertEquals(expectedL2[j], popt.snrmask.mask[1][j], EPS);
        }
    }

    // ---------------------------------------------------------------
    // Test: empty values handled gracefully
    // ---------------------------------------------------------------
    @Test
    void testEmptyValues() throws IOException {
        File tmp = File.createTempFile("conf", ".conf");
        tmp.deleteOnExit();
        try (PrintWriter pw = new PrintWriter(new FileWriter(tmp))) {
            pw.println("pos1-posmode     =single");
            pw.println("pos1-exclsats    =");
            pw.println("out-fieldsep     =");
            pw.println("file-satantfile  =");
            pw.println("ant1-anttype     =");
        }

        ProcessingOptions popt = new ProcessingOptions();
        SolutionOptions sopt = new SolutionOptions();
        FileOptions fopt = ConfigReader.load(tmp.getAbsolutePath(), popt, sopt);

        assertEquals(PMODE_SINGLE, popt.mode);
        assertEquals("", fopt.satantp);
        assertEquals("", popt.anttype[0]);
        // No exsats set
        for (int j = 0; j < MAXSAT; j++) {
            assertEquals(0, popt.exsats[j]);
        }
    }

    // ---------------------------------------------------------------
    // Test: comments and blank lines
    // ---------------------------------------------------------------
    @Test
    void testCommentsAndBlankLines() throws IOException {
        File tmp = File.createTempFile("conf", ".conf");
        tmp.deleteOnExit();
        try (PrintWriter pw = new PrintWriter(new FileWriter(tmp))) {
            pw.println("# This is a comment");
            pw.println("");
            pw.println("pos1-posmode     =ppp-static # (0:single,...)");
            pw.println("   ");
            pw.println("pos1-elmask      =20         # (deg)");
        }

        ProcessingOptions popt = new ProcessingOptions();
        SolutionOptions sopt = new SolutionOptions();
        ConfigReader.load(tmp.getAbsolutePath(), popt, sopt);

        assertEquals(PMODE_PPP_STATIC, popt.mode);
        assertEquals(20.0 * D2R, popt.elmin, EPS);
    }

    // ---------------------------------------------------------------
    // Test: unknown options are silently skipped
    // ---------------------------------------------------------------
    @Test
    void testUnknownOptionsSkipped() throws IOException {
        File tmp = File.createTempFile("conf", ".conf");
        tmp.deleteOnExit();
        try (PrintWriter pw = new PrintWriter(new FileWriter(tmp))) {
            pw.println("pos1-posmode     =static");
            pw.println("unknown-option   =foobar");
            pw.println("pos1-elmask      =10");
        }

        ProcessingOptions popt = new ProcessingOptions();
        SolutionOptions sopt = new SolutionOptions();
        ConfigReader.load(tmp.getAbsolutePath(), popt, sopt);

        assertEquals(PMODE_STATIC, popt.mode);
        assertEquals(10.0 * D2R, popt.elmin, EPS);
    }
}
