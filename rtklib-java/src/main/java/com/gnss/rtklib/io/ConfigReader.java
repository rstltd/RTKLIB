package com.gnss.rtklib.io;

import com.gnss.rtklib.core.Constants;
import com.gnss.rtklib.core.Coord;
import com.gnss.rtklib.core.SatelliteUtil;
import com.gnss.rtklib.model.FileOptions;
import com.gnss.rtklib.model.ProcessingOptions;
import com.gnss.rtklib.model.SolutionOptions;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.function.IntConsumer;

import static com.gnss.rtklib.core.Constants.*;

/**
 * Configuration file reader for RTKLIB .conf format.
 * Ported from C RTKLIB options.c (loadopts, str2enum, buff2sysopts).
 */
public class ConfigReader {

    private ConfigReader() {}

    // ---------------------------------------------------------------
    // Option entry: replaces C opt_t with functional setters
    // ---------------------------------------------------------------

    private static class OptEntry {
        final String name;
        final int format;     // 0=int, 1=double, 2=string, 3=enum
        final String comment;
        IntConsumer setInt;
        DoubleConsumer setDouble;
        Consumer<String> setStr;

        OptEntry(String name, int format, String comment) {
            this.name = name;
            this.format = format;
            this.comment = comment;
        }
    }

    private static OptEntry intOpt(String name, String comment, IntConsumer setter) {
        OptEntry e = new OptEntry(name, 0, comment);
        e.setInt = setter;
        return e;
    }

    private static OptEntry doubleOpt(String name, String comment, DoubleConsumer setter) {
        OptEntry e = new OptEntry(name, 1, comment);
        e.setDouble = setter;
        return e;
    }

    private static OptEntry strOpt(String name, String comment, Consumer<String> setter) {
        OptEntry e = new OptEntry(name, 2, comment);
        e.setStr = setter;
        return e;
    }

    private static OptEntry enumOpt(String name, String comment, IntConsumer setter) {
        OptEntry e = new OptEntry(name, 3, comment);
        e.setInt = setter;
        return e;
    }

    // ---------------------------------------------------------------
    // Enum label strings (matching C options.c lines 44-66)
    // ---------------------------------------------------------------

    private static final String SWTOPT  = "0:off,1:on";
    private static final String MODOPT  = "0:single,1:dgps,2:kinematic,3:static,4:static-start,5:movingbase,6:fixed,7:ppp-kine,8:ppp-static,9:ppp-fixed";
    private static final String FRQOPT  = "1:l1,2:l1+l2,3:l1+l2+l5,4:l1+l2+l5+l6";
    private static final String TYPOPT  = "0:forward,1:backward,2:combined,3:combined-nophasereset";
    private static final String IONOPT  = "0:off,1:brdc,2:sbas,3:dual-freq,4:est-stec,5:ionex-tec,6:qzs-brdc";
    private static final String TRPOPT  = "0:off,1:saas,2:sbas,3:est-ztd,4:est-ztdgrad";
    private static final String EPHOPT  = "0:brdc,1:precise,2:brdc+sbas,3:brdc+ssrapc,4:brdc+ssrcom";
    private static final String NAVOPT  = "1:gps+2:sbas+4:glo+8:gal+16:qzs+32:bds+64:navic";
    private static final String GAROPT  = "0:off,1:on,2:autocal,3:fix-and-hold";
    private static final String SOLOPT  = "0:llh,1:xyz,2:enu,3:nmea";
    private static final String TSYOPT  = "0:gpst,1:utc,2:jst";
    private static final String TFTOPT  = "0:tow,1:hms";
    private static final String DFTOPT  = "0:deg,1:dms";
    private static final String HGTOPT  = "0:ellipsoidal,1:geodetic";
    private static final String GEOOPT  = "0:internal,1:egm96,2:egm08_2.5,3:egm08_1,4:gsi2000";
    private static final String STAOPT  = "0:all,1:single";
    private static final String STSOPT  = "0:off,1:state,2:residual";
    private static final String ARMOPT  = "0:off,1:continuous,2:instantaneous,3:fix-and-hold";
    private static final String POSOPT  = "0:llh,1:xyz,2:single,3:posfile,4:rinexhead,5:rtcm";
    private static final String TIDEOPT = "1:solid+2:otl+4:spole";
    private static final String PHWOPT  = "0:off,1:on,2:precise";
    private static final String WEIGHTOPT = "0:elevation,1:snr";

    // ---------------------------------------------------------------
    // Build options table
    // ---------------------------------------------------------------

    private static List<OptEntry> buildOptionsTable(
            ProcessingOptions popt, SolutionOptions sopt, FileOptions fopt,
            double[] elmask, double[] elmaskar, double[] elmaskhold,
            double[][] antpos, String[] exsats, String[] snrmask) {

        List<OptEntry> opts = new ArrayList<>();

        // pos1- options
        opts.add(enumOpt("pos1-posmode",    MODOPT, v -> popt.mode = v));
        opts.add(enumOpt("pos1-frequency",  FRQOPT, v -> popt.nf = v));
        opts.add(enumOpt("pos1-soltype",    TYPOPT, v -> popt.soltype = v));
        opts.add(doubleOpt("pos1-elmask",   "deg",  v -> elmask[0] = v));
        opts.add(enumOpt("pos1-snrmask_r",  SWTOPT, v -> popt.snrmask.ena[0] = v));
        opts.add(enumOpt("pos1-snrmask_b",  SWTOPT, v -> popt.snrmask.ena[1] = v));
        opts.add(strOpt("pos1-snrmask_L1",  "",     v -> snrmask[0] = v));
        opts.add(strOpt("pos1-snrmask_L2",  "",     v -> snrmask[1] = v));
        opts.add(strOpt("pos1-snrmask_L5",  "",     v -> snrmask[2] = v));
        if (NFREQ > 3) {
            opts.add(strOpt("pos1-snrmask_L6", "", v -> snrmask[3] = v));
        } else {
            opts.add(strOpt("pos1-snrmask_L6", "", v -> { /* ignore if NFREQ<=3 */ }));
        }
        opts.add(enumOpt("pos1-dynamics",   SWTOPT, v -> popt.dynamics = v));
        opts.add(intOpt("pos1-tidecorr",    TIDEOPT, v -> popt.tidecorr = v));
        opts.add(enumOpt("pos1-ionoopt",    IONOPT, v -> popt.ionoopt = v));
        opts.add(enumOpt("pos1-tropopt",    TRPOPT, v -> popt.tropopt = v));
        opts.add(enumOpt("pos1-sateph",     EPHOPT, v -> popt.sateph = v));
        opts.add(enumOpt("pos1-posopt1",    SWTOPT, v -> popt.posopt[0] = v));
        opts.add(enumOpt("pos1-posopt2",    SWTOPT, v -> popt.posopt[1] = v));
        opts.add(enumOpt("pos1-posopt3",    PHWOPT, v -> popt.posopt[2] = v));
        opts.add(enumOpt("pos1-posopt4",    SWTOPT, v -> popt.posopt[3] = v));
        opts.add(enumOpt("pos1-posopt5",    SWTOPT, v -> popt.posopt[4] = v));
        opts.add(enumOpt("pos1-posopt6",    SWTOPT, v -> popt.posopt[5] = v));
        opts.add(strOpt("pos1-exclsats",    "prn ...", v -> exsats[0] = v));
        opts.add(intOpt("pos1-navsys",      NAVOPT, v -> popt.navsys = v));

        // pos2- options
        opts.add(enumOpt("pos2-armode",     ARMOPT, v -> popt.modear = v));
        opts.add(enumOpt("pos2-gloarmode",  GAROPT, v -> popt.glomodear = v));
        opts.add(enumOpt("pos2-bdsarmode",  SWTOPT, v -> popt.bdsmodear = v));
        opts.add(enumOpt("pos2-arfilter",   SWTOPT, v -> popt.arfilter = v));
        opts.add(doubleOpt("pos2-arthres",     "", v -> popt.thresar[0] = v));
        opts.add(doubleOpt("pos2-arthresmin",  "", v -> popt.thresar[5] = v));
        opts.add(doubleOpt("pos2-arthresmax",  "", v -> popt.thresar[6] = v));
        opts.add(doubleOpt("pos2-arthres1",    "", v -> popt.thresar[1] = v));
        opts.add(doubleOpt("pos2-arthres2",    "", v -> popt.thresar[2] = v));
        opts.add(doubleOpt("pos2-arthres3",    "", v -> popt.thresar[3] = v));
        opts.add(doubleOpt("pos2-arthres4",    "", v -> popt.thresar[4] = v));
        opts.add(doubleOpt("pos2-varholdamb",  "cyc^2", v -> popt.varholdamb = v));
        opts.add(doubleOpt("pos2-gainholdamb", "", v -> popt.gainholdamb = v));
        opts.add(intOpt("pos2-arlockcnt",   "", v -> popt.minlock = v));
        opts.add(intOpt("pos2-minfixsats",  "", v -> popt.minfixsats = v));
        opts.add(intOpt("pos2-minholdsats", "", v -> popt.minholdsats = v));
        opts.add(intOpt("pos2-mindropsats", "", v -> popt.mindropsats = v));
        opts.add(doubleOpt("pos2-arelmask", "deg", v -> elmaskar[0] = v));
        opts.add(intOpt("pos2-arminfix",    "", v -> popt.minfix = v));
        opts.add(intOpt("pos2-armaxiter",   "", v -> popt.armaxiter = v));
        opts.add(doubleOpt("pos2-elmaskhold","deg", v -> elmaskhold[0] = v));
        opts.add(intOpt("pos2-aroutcnt",    "", v -> popt.maxout = v));
        opts.add(doubleOpt("pos2-maxage",   "s", v -> popt.maxtdiff = v));
        opts.add(enumOpt("pos2-syncsol",    SWTOPT, v -> popt.syncsol = v));
        opts.add(doubleOpt("pos2-slipthres","m", v -> popt.thresslip = v));
        opts.add(doubleOpt("pos2-dopthres", "m", v -> popt.thresdop = v));
        opts.add(doubleOpt("pos2-rejionno", "m", v -> popt.maxinno[0] = v));
        opts.add(doubleOpt("pos2-rejcode",  "m", v -> popt.maxinno[1] = v));
        opts.add(intOpt("pos2-niter",       "", v -> popt.niter = v));
        opts.add(doubleOpt("pos2-baselen",  "m", v -> popt.baseline[0] = v));
        opts.add(doubleOpt("pos2-basesig",  "m", v -> popt.baseline[1] = v));

        // out- options
        opts.add(enumOpt("out-solformat",   SOLOPT, v -> sopt.posf = v));
        opts.add(enumOpt("out-outhead",     SWTOPT, v -> sopt.outhead = v));
        opts.add(enumOpt("out-outopt",      SWTOPT, v -> sopt.outopt = v));
        opts.add(enumOpt("out-outvel",      SWTOPT, v -> sopt.outvel = v));
        opts.add(enumOpt("out-timesys",     TSYOPT, v -> sopt.times = v));
        opts.add(enumOpt("out-timeform",    TFTOPT, v -> sopt.timef = v));
        opts.add(intOpt("out-timendec",     "", v -> sopt.timeu = v));
        opts.add(enumOpt("out-degform",     DFTOPT, v -> sopt.degf = v));
        opts.add(strOpt("out-fieldsep",     "", v -> sopt.separator = v));
        opts.add(enumOpt("out-outsingle",   SWTOPT, v -> popt.outsingle = v));
        opts.add(doubleOpt("out-maxsolstd", "m", v -> sopt.maxsolstd = v));
        opts.add(enumOpt("out-height",      HGTOPT, v -> sopt.height = v));
        opts.add(enumOpt("out-geoid",       GEOOPT, v -> sopt.geoid = v));
        opts.add(enumOpt("out-solstatic",   STAOPT, v -> sopt.solstatic = v));
        opts.add(doubleOpt("out-nmeaintv1", "s", v -> sopt.nmeaintv[0] = v));
        opts.add(doubleOpt("out-nmeaintv2", "s", v -> sopt.nmeaintv[1] = v));
        opts.add(enumOpt("out-outstat",     STSOPT, v -> sopt.sstat = v));

        // stats- options
        opts.add(doubleOpt("stats-eratio1",    "", v -> popt.eratio[0] = v));
        opts.add(doubleOpt("stats-eratio2",    "", v -> popt.eratio[1] = v));
        opts.add(doubleOpt("stats-eratio5",    "", v -> popt.eratio[2] = v));
        opts.add(doubleOpt("stats-eratio6",    "", v -> popt.eratio[3] = v));
        opts.add(doubleOpt("stats-errphase",   "m", v -> popt.err[1] = v));
        opts.add(doubleOpt("stats-errphaseel", "m", v -> popt.err[2] = v));
        opts.add(doubleOpt("stats-errphasebl", "m/10km", v -> popt.err[3] = v));
        opts.add(doubleOpt("stats-errdoppler", "Hz", v -> popt.err[4] = v));
        opts.add(doubleOpt("stats-snrmax",     "dB.Hz", v -> popt.err[5] = v));
        opts.add(doubleOpt("stats-errsnr",     "m", v -> popt.err[6] = v));
        opts.add(doubleOpt("stats-errrcv",     "", v -> popt.err[7] = v));
        opts.add(doubleOpt("stats-stdbias",    "m", v -> popt.std[0] = v));
        opts.add(doubleOpt("stats-stdiono",    "m", v -> popt.std[1] = v));
        opts.add(doubleOpt("stats-stdtrop",    "m", v -> popt.std[2] = v));
        opts.add(doubleOpt("stats-prnaccelh",  "m/s^2", v -> popt.prn[3] = v));
        opts.add(doubleOpt("stats-prnaccelv",  "m/s^2", v -> popt.prn[4] = v));
        opts.add(doubleOpt("stats-prnbias",    "m", v -> popt.prn[0] = v));
        opts.add(doubleOpt("stats-prniono",    "m", v -> popt.prn[1] = v));
        opts.add(doubleOpt("stats-prntrop",    "m", v -> popt.prn[2] = v));
        opts.add(doubleOpt("stats-prnpos",     "m", v -> popt.prn[5] = v));
        opts.add(doubleOpt("stats-clkstab",    "s/s", v -> popt.sclkstab = v));

        // ant1- options
        opts.add(enumOpt("ant1-postype",    POSOPT, v -> popt.rovpos = v));
        opts.add(doubleOpt("ant1-pos1",     "deg|m", v -> antpos[0][0] = v));
        opts.add(doubleOpt("ant1-pos2",     "deg|m", v -> antpos[0][1] = v));
        opts.add(doubleOpt("ant1-pos3",     "m|m",   v -> antpos[0][2] = v));
        opts.add(strOpt("ant1-anttype",     "", v -> popt.anttype[0] = v));
        opts.add(doubleOpt("ant1-antdele",  "m", v -> popt.antdel[0][0] = v));
        opts.add(doubleOpt("ant1-antdeln",  "m", v -> popt.antdel[0][1] = v));
        opts.add(doubleOpt("ant1-antdelu",  "m", v -> popt.antdel[0][2] = v));

        // ant2- options
        opts.add(enumOpt("ant2-postype",    POSOPT, v -> popt.refpos = v));
        opts.add(doubleOpt("ant2-pos1",     "deg|m", v -> antpos[1][0] = v));
        opts.add(doubleOpt("ant2-pos2",     "deg|m", v -> antpos[1][1] = v));
        opts.add(doubleOpt("ant2-pos3",     "m|m",   v -> antpos[1][2] = v));
        opts.add(strOpt("ant2-anttype",     "", v -> popt.anttype[1] = v));
        opts.add(doubleOpt("ant2-antdele",  "m", v -> popt.antdel[1][0] = v));
        opts.add(doubleOpt("ant2-antdeln",  "m", v -> popt.antdel[1][1] = v));
        opts.add(doubleOpt("ant2-antdelu",  "m", v -> popt.antdel[1][2] = v));
        opts.add(intOpt("ant2-maxaveep",    "", v -> popt.maxaveep = v));
        opts.add(enumOpt("ant2-initrst",    SWTOPT, v -> popt.initrst = v));

        // misc- options
        opts.add(enumOpt("misc-timeinterp", SWTOPT, v -> popt.intpref = v));
        opts.add(intOpt("misc-sbasatsel",   "0:all", v -> popt.sbassatsel = v));
        opts.add(strOpt("misc-rnxopt1",     "", v -> popt.rnxopt[0] = v));
        opts.add(strOpt("misc-rnxopt2",     "", v -> popt.rnxopt[1] = v));
        opts.add(strOpt("misc-pppopt",      "", v -> popt.pppopt = v));

        // file- options
        opts.add(strOpt("file-satantfile",  "", v -> fopt.satantp = v));
        opts.add(strOpt("file-rcvantfile",  "", v -> fopt.rcvantp = v));
        opts.add(strOpt("file-staposfile",  "", v -> fopt.stapos = v));
        opts.add(strOpt("file-geoidfile",   "", v -> fopt.geoid = v));
        opts.add(strOpt("file-ionofile",    "", v -> fopt.iono = v));
        opts.add(strOpt("file-dcbfile",     "", v -> fopt.dcb = v));
        opts.add(strOpt("file-eopfile",     "", v -> fopt.eop = v));
        opts.add(strOpt("file-blqfile",     "", v -> fopt.blq = v));
        opts.add(strOpt("file-tempdir",     "", v -> fopt.tempdir = v));
        opts.add(strOpt("file-geexefile",   "", v -> fopt.geexe = v));
        opts.add(strOpt("file-solstatfile", "", v -> fopt.solstat = v));
        opts.add(strOpt("file-tracefile",   "", v -> fopt.trace = v));

        return opts;
    }

    // ---------------------------------------------------------------
    // str2enum: label string -> integer value
    // Ported from C options.c:str2enum()
    // ---------------------------------------------------------------

    static int str2enum(String str, String comment) {
        str = str.trim();

        // Search for str as a label in comment
        int startPos = 0;
        while (true) {
            int p = comment.indexOf(str, startPos);
            if (p < 0) break;

            // Check that it's a complete label match (not substring)
            int end = p + str.length();
            if (end < comment.length()) {
                char next = comment.charAt(end);
                if (next != ',' && next != ')' && next != '+') {
                    startPos = p + 1;
                    continue;
                }
            }

            // Search backwards for N: prefix
            if (p < 1) { startPos = p + 1; continue; }
            int i = p - 1;
            if (comment.charAt(i) != ':') { startPos = p + 1; continue; }
            i--;
            int j = i;
            while (j >= 0 && comment.charAt(j) >= '0' && comment.charAt(j) <= '9') {
                j--;
            }
            if (j == i) { startPos = p + 1; continue; } // no digits
            try {
                return Integer.parseInt(comment.substring(j + 1, i + 1));
            } catch (NumberFormatException e) {
                startPos = p + 1;
                continue;
            }
        }

        // Try parsing str as "N:..." (numeric value with colon)
        int colonIdx = str.indexOf(':');
        if (colonIdx > 0) {
            String numPart = str.substring(0, colonIdx);
            if (comment.contains(numPart + ":")) {
                try {
                    return Integer.parseInt(numPart);
                } catch (NumberFormatException e) {
                    // fall through
                }
            }
        }

        // Try parsing str as plain integer
        try {
            return Integer.parseInt(str);
        } catch (NumberFormatException e) {
            return -1;
        }
    }

    // ---------------------------------------------------------------
    // searchopt: find option by name
    // ---------------------------------------------------------------

    private static OptEntry searchOpt(String name, List<OptEntry> opts) {
        for (OptEntry opt : opts) {
            if (opt.name.contains(name)) return opt;
        }
        return null;
    }

    // ---------------------------------------------------------------
    // str2opt: set option value from string
    // ---------------------------------------------------------------

    private static boolean str2opt(OptEntry opt, String val) {
        val = val.trim();
        switch (opt.format) {
            case 0: // int
                try {
                    opt.setInt.accept(Integer.parseInt(val));
                } catch (NumberFormatException e) {
                    return false;
                }
                break;
            case 1: // double
                try {
                    opt.setDouble.accept(Double.parseDouble(val));
                } catch (NumberFormatException e) {
                    return false;
                }
                break;
            case 2: // string
                opt.setStr.accept(val);
                break;
            case 3: // enum
                int v = str2enum(val, opt.comment);
                if (v < 0) return false;
                opt.setInt.accept(v);
                break;
            default:
                return false;
        }
        return true;
    }

    // ---------------------------------------------------------------
    // loadopts: parse config file
    // Ported from C options.c:loadopts()
    // ---------------------------------------------------------------

    private static void loadopts(String file, List<OptEntry> opts) throws IOException {
        try (BufferedReader br = new BufferedReader(new FileReader(file))) {
            String line;
            int lineNum = 0;
            while ((line = br.readLine()) != null) {
                lineNum++;

                // Strip comment
                int commentIdx = line.indexOf('#');
                if (commentIdx >= 0) line = line.substring(0, commentIdx);

                line = line.trim();
                if (line.isEmpty()) continue;

                // Split on '='
                int eqIdx = line.indexOf('=');
                if (eqIdx < 0) {
                    System.err.printf("invalid option %s (%s:%d)%n", line, file, lineNum);
                    continue;
                }

                String key = line.substring(0, eqIdx).trim();
                String val = line.substring(eqIdx + 1).trim();

                OptEntry opt = searchOpt(key, opts);
                if (opt == null) continue; // unknown key: silently skip

                if (!str2opt(opt, val)) {
                    System.err.printf("invalid option value %s (%s:%d)%n", key, file, lineNum);
                }
            }
        }
    }

    // ---------------------------------------------------------------
    // buff2sysopts: post-processing after parsing
    // Ported from C options.c:buff2sysopts()
    // ---------------------------------------------------------------

    private static void buff2sysopts(ProcessingOptions popt,
                                      double[] elmask, double[] elmaskar, double[] elmaskhold,
                                      double[][] antpos, String[] exsats, String[] snrmask) {

        // Clamp nf
        if (popt.nf > NFREQ) popt.nf = NFREQ;

        // Degrees to radians
        popt.elmin = elmask[0] * D2R;
        popt.elmaskar = elmaskar[0] * D2R;
        popt.elmaskhold = elmaskhold[0] * D2R;

        // Antenna positions: convert LLH (deg) -> ECEF, or copy XYZ directly
        for (int i = 0; i < 2; i++) {
            int postype = (i == 0) ? popt.rovpos : popt.refpos;
            double[] dest = (i == 0) ? popt.ru : popt.rb;

            if (postype == POSOPT_POS_LLH) {
                // LLH in degrees -> radians -> ECEF
                double[] pos = {
                    antpos[i][0] * D2R,
                    antpos[i][1] * D2R,
                    antpos[i][2]
                };
                double[] ecef = Coord.pos2ecef(pos);
                System.arraycopy(ecef, 0, dest, 0, 3);
            } else if (postype == POSOPT_POS_XYZ) {
                // XYZ directly
                dest[0] = antpos[i][0];
                dest[1] = antpos[i][1];
                dest[2] = antpos[i][2];
            }
            // Other types (single, posfile, rinexhead, rtcm) are handled at runtime
        }

        // Excluded satellites
        java.util.Arrays.fill(popt.exsats, (byte) 0);
        if (exsats[0] != null && !exsats[0].trim().isEmpty()) {
            String[] tokens = exsats[0].trim().split("\\s+");
            for (String tok : tokens) {
                boolean include = false;
                if (tok.startsWith("+")) {
                    include = true;
                    tok = tok.substring(1);
                }
                int sat = SatelliteUtil.satid2no(tok);
                if (sat > 0 && sat <= MAXSAT) {
                    popt.exsats[sat - 1] = (byte) (include ? 2 : 1);
                }
            }
        }

        // SNR mask: comma-separated values -> double[NFREQ][9]
        for (int i = 0; i < NFREQ && i < snrmask.length; i++) {
            if (snrmask[i] == null || snrmask[i].trim().isEmpty()) continue;
            String[] vals = snrmask[i].split(",");
            for (int j = 0; j < 9 && j < vals.length; j++) {
                try {
                    popt.snrmask.mask[i][j] = Double.parseDouble(vals[j].trim());
                } catch (NumberFormatException e) {
                    // ignore
                }
            }
        }
    }

    // ---------------------------------------------------------------
    // Public API
    // ---------------------------------------------------------------

    /**
     * Load configuration from RTKLIB .conf file.
     * Populates popt and sopt, returns FileOptions.
     *
     * @param file path to .conf file
     * @param popt processing options (modified in place)
     * @param sopt solution options (modified in place)
     * @return file options parsed from config
     * @throws IOException if file cannot be read
     */
    public static FileOptions load(String file, ProcessingOptions popt,
                                    SolutionOptions sopt) throws IOException {
        FileOptions fopt = new FileOptions();

        // Intermediate buffers
        double[] elmask = {15.0};
        double[] elmaskar = {0.0};
        double[] elmaskhold = {0.0};
        double[][] antpos = new double[2][3];
        String[] exsats = {""};
        String[] snrmask = new String[MAXFREQ];

        // Build table, parse file, post-process
        List<OptEntry> opts = buildOptionsTable(popt, sopt, fopt,
                elmask, elmaskar, elmaskhold, antpos, exsats, snrmask);
        loadopts(file, opts);
        buff2sysopts(popt, elmask, elmaskar, elmaskhold, antpos, exsats, snrmask);

        return fopt;
    }
}
