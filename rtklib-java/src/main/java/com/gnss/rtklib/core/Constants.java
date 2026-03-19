package com.gnss.rtklib.core;

/**
 * GNSS constants ported from RTKLIB rtklib.h.
 * All constellations enabled (no conditional compilation).
 */
public final class Constants {

    private Constants() {}

    // Physical constants
    public static final double PI      = 3.1415926535897932;
    public static final double D2R     = PI / 180.0;
    public static final double R2D     = 180.0 / PI;
    public static final double CLIGHT  = 299792458.0;
    public static final double SC2RAD  = 3.1415926535898;
    public static final double AU      = 149597870691.0;
    public static final double AS2R    = D2R / 3600.0;

    public static final double OMGE    = 7.2921151467E-5;
    public static final double RE_WGS84 = 6378137.0;
    public static final double FE_WGS84 = 1.0 / 298.257223563;
    public static final double HION    = 350000.0;

    // Carrier frequencies (Hz)
    public static final double FREQL1     = 1.57542E9;
    public static final double FREQL2     = 1.22760E9;
    public static final double FREQE5b    = 1.20714E9;
    public static final double FREQL5     = 1.17645E9;
    public static final double FREQL6     = 1.27875E9;
    public static final double FREQE5ab   = 1.191795E9;
    public static final double FREQs      = 2.492028E9;
    public static final double FREQ1_GLO  = 1.60200E9;
    public static final double DFRQ1_GLO  = 0.56250E6;
    public static final double FREQ2_GLO  = 1.24600E9;
    public static final double DFRQ2_GLO  = 0.43750E6;
    public static final double FREQ3_GLO  = 1.202025E9;
    public static final double FREQ1a_GLO = 1.600995E9;
    public static final double FREQ2a_GLO = 1.248060E9;
    public static final double FREQ1_CMP  = 1.561098E9;
    public static final double FREQ2_CMP  = 1.20714E9;
    public static final double FREQ3_CMP  = 1.26852E9;

    // Error factors
    public static final double EFACT_GPS = 1.0;
    public static final double EFACT_GLO = 1.5;
    public static final double EFACT_GAL = 1.0;
    public static final double EFACT_QZS = 1.0;
    public static final double EFACT_CMP = 1.0;
    public static final double EFACT_IRN = 1.5;
    public static final double EFACT_SBS = 3.0;

    // Navigation systems
    public static final int SYS_NONE = 0x00;
    public static final int SYS_GPS  = 0x01;
    public static final int SYS_SBS  = 0x02;
    public static final int SYS_GLO  = 0x04;
    public static final int SYS_GAL  = 0x08;
    public static final int SYS_QZS  = 0x10;
    public static final int SYS_CMP  = 0x20;
    public static final int SYS_IRN  = 0x40;
    public static final int SYS_LEO  = 0x80;
    public static final int SYS_ALL  = 0xFF;

    // Number of frequencies
    public static final int NFREQ   = 3;
    public static final int NEXOBS  = 3;
    public static final int MAXFREQ = 6;

    // Satellite PRN ranges (all constellations enabled, LEO disabled)
    public static final int MINPRNGPS = 1;
    public static final int MAXPRNGPS = 32;
    public static final int NSATGPS   = MAXPRNGPS - MINPRNGPS + 1; // 32

    public static final int MINPRNGLO = 1;
    public static final int MAXPRNGLO = 27;
    public static final int NSATGLO   = MAXPRNGLO - MINPRNGLO + 1; // 27

    public static final int MINPRNGAL = 1;
    public static final int MAXPRNGAL = 36;
    public static final int NSATGAL   = MAXPRNGAL - MINPRNGAL + 1; // 36

    public static final int MINPRNQZS   = 193;
    public static final int MAXPRNQZS   = 202;
    public static final int MINPRNQZS_S = 183;
    public static final int MAXPRNQZS_S = 191;
    public static final int NSATQZS     = MAXPRNQZS - MINPRNQZS + 1; // 10

    public static final int MINPRNCMP = 1;
    public static final int MAXPRNCMP = 46;
    public static final int NSATCMP   = MAXPRNCMP - MINPRNCMP + 1; // 46

    public static final int MINPRNIRN = 1;
    public static final int MAXPRNIRN = 14;
    public static final int NSATIRN   = MAXPRNIRN - MINPRNIRN + 1; // 14

    public static final int MINPRNLEO = 0;
    public static final int MAXPRNLEO = 0;
    public static final int NSATLEO   = 0;

    public static final int MINPRNSBS = 120;
    public static final int MAXPRNSBS = 158;
    public static final int NSATSBS   = MAXPRNSBS - MINPRNSBS + 1; // 39

    public static final int MAXSAT = NSATGPS + NSATGLO + NSATGAL + NSATQZS
                                   + NSATCMP + NSATIRN + NSATLEO + NSATSBS; // 204

    // Observation limits
    public static final int MAXOBS     = 96;
    public static final int MAXOBSTYPE = 64;
    public static final double DTTOL   = 0.025;

    // Max time difference to Toe (s)
    public static final double MAXDTOE     = 7200.0;
    public static final double MAXDTOE_QZS = 7200.0;
    public static final double MAXDTOE_GAL = 14400.0;
    public static final double MAXDTOE_CMP = 21600.0;
    public static final double MAXDTOE_GLO = 1800.0;
    public static final double MAXDTOE_IRN = 7200.0;
    public static final double MAXDTOE_SBS = 360.0;
    public static final double MAXDTOE_S   = 86400.0;
    public static final double MAXGDOP     = 300.0;

    // Code biases
    public static final int MAX_CODE_BIASES     = 3;
    public static final int MAX_CODE_BIAS_FREQS = 2;

    // Observation codes (CODE_xxx)
    public static final int CODE_NONE = 0;
    public static final int CODE_L1C  = 1;
    public static final int CODE_L1P  = 2;
    public static final int CODE_L1W  = 3;
    public static final int CODE_L1Y  = 4;
    public static final int CODE_L1M  = 5;
    public static final int CODE_L1N  = 6;
    public static final int CODE_L1S  = 7;
    public static final int CODE_L1L  = 8;
    public static final int CODE_L1E  = 9;
    public static final int CODE_L1A  = 10;
    public static final int CODE_L1B  = 11;
    public static final int CODE_L1X  = 12;
    public static final int CODE_L1Z  = 13;
    public static final int CODE_L2C  = 14;
    public static final int CODE_L2D  = 15;
    public static final int CODE_L2S  = 16;
    public static final int CODE_L2L  = 17;
    public static final int CODE_L2X  = 18;
    public static final int CODE_L2P  = 19;
    public static final int CODE_L2W  = 20;
    public static final int CODE_L2Y  = 21;
    public static final int CODE_L2M  = 22;
    public static final int CODE_L2N  = 23;
    public static final int CODE_L5I  = 24;
    public static final int CODE_L5Q  = 25;
    public static final int CODE_L5X  = 26;
    public static final int CODE_L7I  = 27;
    public static final int CODE_L7Q  = 28;
    public static final int CODE_L7X  = 29;
    public static final int CODE_L6A  = 30;
    public static final int CODE_L6B  = 31;
    public static final int CODE_L6C  = 32;
    public static final int CODE_L6X  = 33;
    public static final int CODE_L6Z  = 34;
    public static final int CODE_L6S  = 35;
    public static final int CODE_L6L  = 36;
    public static final int CODE_L8I  = 37;
    public static final int CODE_L8Q  = 38;
    public static final int CODE_L8X  = 39;
    public static final int CODE_L2I  = 40;
    public static final int CODE_L2Q  = 41;
    public static final int CODE_L6I  = 42;
    public static final int CODE_L6Q  = 43;
    public static final int CODE_L3I  = 44;
    public static final int CODE_L3Q  = 45;
    public static final int CODE_L3X  = 46;
    public static final int CODE_L1I  = 47;
    public static final int CODE_L1Q  = 48;
    public static final int CODE_L5A  = 49;
    public static final int CODE_L5B  = 50;
    public static final int CODE_L5C  = 51;
    public static final int CODE_L9A  = 52;
    public static final int CODE_L9B  = 53;
    public static final int CODE_L9C  = 54;
    public static final int CODE_L9X  = 55;
    public static final int CODE_L1D  = 56;
    public static final int CODE_L5D  = 57;
    public static final int CODE_L5P  = 58;
    public static final int CODE_L5Z  = 59;
    public static final int CODE_L6E  = 60;
    public static final int CODE_L7D  = 61;
    public static final int CODE_L7P  = 62;
    public static final int CODE_L7Z  = 63;
    public static final int CODE_L8D  = 64;
    public static final int CODE_L8P  = 65;
    public static final int CODE_L4A  = 66;
    public static final int CODE_L4B  = 67;
    public static final int CODE_L4X  = 68;
    public static final int CODE_L6D  = 69;
    public static final int CODE_L6P  = 70;
    public static final int MAXCODE   = 70;

    // Positioning modes
    public static final int PMODE_SINGLE       = 0;
    public static final int PMODE_DGPS         = 1;
    public static final int PMODE_KINEMA       = 2;
    public static final int PMODE_STATIC       = 3;
    public static final int PMODE_STATIC_START = 4;
    public static final int PMODE_MOVEB        = 5;
    public static final int PMODE_FIXED        = 6;
    public static final int PMODE_PPP_KINEMA   = 7;
    public static final int PMODE_PPP_STATIC   = 8;
    public static final int PMODE_PPP_FIXED    = 9;

    // Solution formats
    public static final int SOLF_LLH  = 0;
    public static final int SOLF_XYZ  = 1;
    public static final int SOLF_ENU  = 2;
    public static final int SOLF_NMEA = 3;
    public static final int SOLF_STAT = 4;
    public static final int SOLF_GSIF = 5;

    // Solution status
    public static final int SOLQ_NONE   = 0;
    public static final int SOLQ_FIX    = 1;
    public static final int SOLQ_FLOAT  = 2;
    public static final int SOLQ_SBAS   = 3;
    public static final int SOLQ_DGPS   = 4;
    public static final int SOLQ_SINGLE = 5;
    public static final int SOLQ_PPP    = 6;
    public static final int SOLQ_DR     = 7;
    public static final int MAXSOLQ     = 7;

    // Solution types
    public static final int SOLTYPE_FORWARD          = 0;
    public static final int SOLTYPE_BACKWARD         = 1;
    public static final int SOLTYPE_COMBINED         = 2;
    public static final int SOLTYPE_COMBINED_NORESET = 3;

    // Time systems (for display)
    public static final int TIMES_GPST = 0;
    public static final int TIMES_UTC  = 1;
    public static final int TIMES_JST  = 2;

    // Time systems (internal)
    public static final int TSYS_GPS = 0;
    public static final int TSYS_UTC = 1;
    public static final int TSYS_GLO = 2;
    public static final int TSYS_GAL = 3;
    public static final int TSYS_QZS = 4;
    public static final int TSYS_CMP = 5;
    public static final int TSYS_IRN = 6;

    // Ionosphere options
    public static final int IONOOPT_OFF  = 0;
    public static final int IONOOPT_BRDC = 1;
    public static final int IONOOPT_SBAS = 2;
    public static final int IONOOPT_IFLC = 3;
    public static final int IONOOPT_EST  = 4;
    public static final int IONOOPT_TEC  = 5;
    public static final int IONOOPT_QZS  = 6;

    // Troposphere options
    public static final int TROPOPT_OFF  = 0;
    public static final int TROPOPT_SAAS = 1;
    public static final int TROPOPT_SBAS = 2;
    public static final int TROPOPT_EST  = 3;
    public static final int TROPOPT_ESTG = 4;

    // Ephemeris options
    public static final int EPHOPT_BRDC   = 0;
    public static final int EPHOPT_PREC   = 1;
    public static final int EPHOPT_SBAS   = 2;
    public static final int EPHOPT_SSRAPC = 3;
    public static final int EPHOPT_SSRCOM = 4;

    // LLI flags
    public static final int LLI_SLIP   = 0x01;
    public static final int LLI_HALFC  = 0x02;
    public static final int LLI_BOCTRK = 0x04;
    public static final int LLI_HALFA  = 0x40;
    public static final int LLI_HALFS  = 0x80;

    // Antenna position types
    public static final int POSOPT_POS_LLH = 0;
    public static final int POSOPT_POS_XYZ = 1;
    public static final int POSOPT_POS_SINGLE = 2;
    public static final int POSOPT_POS_FILE = 3;

    // Chi-squared (n) critical values (alpha=0.001)
    public static final double[] CHISQR = {
        10.8,13.8,16.3,18.5,20.5,22.5,24.3,26.1,27.9,29.6,
        31.3,32.9,34.5,36.1,37.7,39.3,40.8,42.3,43.8,45.3,
        46.8,48.3,49.7,51.2,52.6,54.1,55.5,56.9,58.3,59.7,
        61.1,62.5,63.9,65.2,66.6,68.0,69.3,70.7,72.1,73.4,
        74.7,76.0,77.3,78.6,80.0,81.3,82.6,84.0,85.4,86.7,
        88.0,89.3,90.6,91.9,93.3,94.7,96.0,97.4,98.7,100,
        101,102,103,104,105,107,108,109,110,112,
        113,114,115,116,118,119,120,122,123,125,
        126,127,128,129,131,132,133,134,135,137,
        138,139,140,142,143,144,145,147,148,149
    };
}
